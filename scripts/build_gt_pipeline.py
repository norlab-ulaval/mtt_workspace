#!/usr/bin/env python3
"""Orchestrate the complete offline GT dataset pipeline: offline ICP qualification
→ factor graph solve → 100Hz reformat → qualification report → reference reshape →
canonical dataset assembly → manifest + catalog.

Never touches live /mapping/icp_odom; requires an explicit, human-approved offline
ICP CSV. Produces a frozen, hashed, cataloged dataset/ package with provenance
tracking (config, git commit, ICP source, file SHAs).
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import yaml
from pathlib import Path

import numpy as np
import pandas as pd

from gt_catalog import assert_not_frozen, upsert_catalog_row
from gt_provenance import file_record, git_commit_hash, git_is_dirty


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True,
                         help="Session directory (contains bag/).")
    parser.add_argument("--offline-icp", type=Path, required=True,
                         help="Qualified offline ICP CSV, visually approved by a human.")
    parser.add_argument("--icp-approved-by", required=True,
                         help="Name of the person who approved --offline-icp.")
    parser.add_argument("--calibration", choices=["v1", "v2"], required=True)
    parser.add_argument("--output", type=Path, required=True,
                         help="Session-level output root; dataset/ is written under this.")
    parser.add_argument("--map-reference", type=Path, action="append", default=[],
                         help="Path to an already-exported map file (PLY/VTK) from "
                              "Mohamed's manual offline mapping. Repeatable; at least one "
                              "required unless --dry-run/--validate-only.")
    parser.add_argument("--session-id", default=None,
                         help="Defaults to --session's directory name.")
    parser.add_argument("--terrain", required=True)
    parser.add_argument("--surface", required=True)
    parser.add_argument("--reference-grade", default="B_operational",
                         help="Per DATASET_SCHEMA.md's grade table -- B_operational matches "
                              "'qualified offline multi-sensor or ICP-based operational "
                              "reference', the exact description of what this pipeline "
                              "produces. Override only with a documented reason.")
    parser.add_argument("--reference-source", default="factor_graph_offline_icp_v1",
                         help="Free-form identifier for the reference pipeline used, "
                              "recorded verbatim in the canonical CSV and catalog.")
    parser.add_argument("--reference-world-frame", default="map")
    parser.add_argument("--force-overwrite-ready", action="store_true",
                         help="Override the frozen-dataset guard (Task 7). Do not use this "
                              "on Campus or Ice rink without discussing with Mohamed first.")
    parser.add_argument("--dry-run", action="store_true",
                         help="Print the resolved stage plan and exit without running anything.")
    parser.add_argument("--validate-only", action="store_true",
                         help="Run DATASET_SCHEMA.md gates against an already-built "
                              "--output/dataset/ directory and exit; runs no pipeline stages.")
    return parser.parse_args()


def validate_dataset_dir(dataset_dir: Path) -> list[str]:
    """Implements DATASET_SCHEMA.md section 13's gates. Returns a list of
    human-readable failures; empty list means all gates passed."""
    failures: list[str] = []
    canonical_path = dataset_dir / "canonical_100hz.csv.gz"
    if not canonical_path.exists():
        return [f"{canonical_path} does not exist"]
    df = pd.read_csv(canonical_path, compression="gzip")
    t = df["time_s"].to_numpy(dtype=float)
    if not np.all(np.isfinite(t)):
        failures.append("time_s contains non-finite values")
    if not np.all(np.diff(t) > 0):
        failures.append("time_s is not strictly increasing")
    if len(t) > 1:
        dt = np.diff(t)
        if abs(float(np.median(dt)) - 0.01) > 1e-6:
            failures.append(f"median dt_s = {np.median(dt):.6f}, expected 0.01")
    for col in ["session_id", "terrain", "surface", "calibration_id",
                "reference_grade", "reference_source", "body_reference_frame"]:
        if col not in df or df[col].isna().any() or (df[col].astype(str).str.len() == 0).any():
            failures.append(f"mandatory identifier column {col!r} is missing, empty, or non-constant")
    quat_cols = ["gt_track_qx", "gt_track_qy", "gt_track_qz", "gt_track_qw"]
    if all(c in df for c in quat_cols):
        valid = df["valid_gt_pose"].astype(bool)
        q = df.loc[valid, quat_cols].to_numpy(dtype=float)
        if len(q):
            norms = np.linalg.norm(q, axis=1)
            if not np.all(np.isfinite(norms)) or np.max(np.abs(norms - 1.0)) > 1e-3:
                failures.append("quaternions on valid rows are not finite/unit-norm")
    reread = pd.read_csv(canonical_path, compression="gzip")
    if reread.shape != df.shape or list(reread.columns) != list(df.columns):
        failures.append("gzip CSV does not read back with identical shape/columns")
    for forbidden_substr in ["m0_", "m5_", "ape_", "rpe_", "split_contiguous"]:
        matches = [c for c in df.columns if forbidden_substr in c.lower()]
        if matches:
            failures.append(f"forbidden columns present (contains {forbidden_substr!r}): {matches}")
    return failures


def print_dry_run_plan(args: argparse.Namespace) -> None:
    print("build_gt_pipeline dry run -- no stage will actually execute.")
    print(f"  session:            {args.session}")
    print(f"  offline_icp:        {args.offline_icp} (approved_by={args.icp_approved_by})")
    print(f"  calibration:        {args.calibration}")
    print(f"  output:             {args.output}")
    print(f"  output/dataset:     {args.output / 'dataset'}")
    print("  planned stages:")
    print("    1. extract_v2_measurements.py -> offline_reference/audit/tf_static.yaml "
          "(+ measurements/, superseded by stage 2 below for icp/track_odom/zed_odom.csv)")
    print("    2. offline_reference.py       -> offline_reference/{measurements,graph}/ "
          "(runs after stage 1 -- see build_gt_pipeline.py's stage-order comment)")
    print("    3. build_gt_v2_100hz.py       -> offline_reference/poses/")
    print("    4. qualify_gt_v2.py           -> offline_reference/qualification/")
    print("    5. build_gt_reference_csv.py  -> offline_reference/reference.csv")
    print("    6. build_session_dataset.py   -> dataset/canonical_100hz.csv.gz, trajectory_reference.ply")
    print("    7. copy --map-reference file(s) -> dataset/map_reference.*")
    print("    8. export_bag_preview.py      -> dataset/preview.mp4")
    print("    9. gt_provenance/gt_catalog   -> dataset/manifest.yaml, dataset_catalog.csv row")


def run_checked(cmd: list[str]) -> None:
    print(f"+ {' '.join(cmd)}")
    subprocess.run(cmd, check=True)


def relpath_or_absolute(path: Path, base: Path) -> str:
    try:
        return str(path.relative_to(base))
    except ValueError:
        return str(path.resolve())


def main() -> int:
    args = parse_args()
    if args.dry_run:
        print_dry_run_plan(args)
        return 0

    dataset_dir = args.output / "dataset"
    if args.validate_only:
        failures = validate_dataset_dir(dataset_dir)
        if failures:
            print(f"VALIDATION FAILED ({len(failures)} issue(s)):")
            for f in failures:
                print(f"  - {f}")
            return 1
        print(f"{dataset_dir}: all DATASET_SCHEMA.md gates passed.")
        return 0

    if not args.offline_icp.is_file():
        raise SystemExit(f"--offline-icp does not exist: {args.offline_icp}")
    if not args.map_reference:
        raise SystemExit(
            "--map-reference is required for a real run (at least one map file from "
            "Mohamed's manual offline mapping). This pipeline does not run mapping itself.")

    session_id = args.session_id or args.session.resolve().name
    repo_root = Path(__file__).resolve().parents[1]
    catalog_path = Path("/data/mtt_bags/dataset_catalog.csv")
    assert_not_frozen(catalog_path, session_id, args.force_overwrite_ready)

    work_dir = args.output / "offline_reference"
    work_dir.mkdir(parents=True, exist_ok=True)

    # Stage 1 (run second) / Stage 2 (run first) both write into work_dir's
    # measurements/ subdirectory, and their outputs overlap on three filenames
    # (icp.csv, track_odom.csv, zed_odom.csv). icp.csv is byte-identical
    # regardless of writer (both read the same --offline-icp file with the
    # same t = sec + nanosec*1e-9 formula); track_odom.csv/zed_odom.csv are
    # NOT -- each script independently re-parses the same bag topic. Running
    # extract_v2_measurements.py (Task 2) FIRST and offline_reference.py
    # (Task 1) SECOND makes Task 1's versions the ones left on disk for all
    # three shared filenames -- the same measurement stream that actually fed
    # the factor graph solve below, so qualify_gt_v2.py's diagnostics (Stage 4)
    # stay self-consistent with what was optimized. Task 2's own unique files
    # (zed_imu.csv, articulation_state.csv, hardware_phi.csv, etc. -- read
    # unconditionally, some without an .exists() guard, by qualify_gt_v2.py)
    # are untouched by Task 1 and survive regardless of order.

    # Stage 2: extract_v2_measurements.py (Task 2) -- audit/tf_static.yaml
    # (needed by Stage 3) plus its own measurements/ files, superseded below
    # for the three overlapping filenames.
    run_checked([
        sys.executable, str(repo_root / "scripts" / "extract_v2_measurements.py"),
        "--session", str(args.session), "--offline-icp", str(args.offline_icp),
        "--output-dir", str(work_dir),
    ])

    # Stage 1: offline_reference.py (Task 1) -- measurements/ + graph/. Runs
    # after Stage 2 so its icp.csv/track_odom.csv/zed_odom.csv are the ones
    # that persist (see comment above).
    run_checked([
        sys.executable, str(repo_root / "scripts" / "offline_reference.py"),
        str(args.session), "--offline-icp", str(args.offline_icp),
        "--icp-approved-by", args.icp_approved_by,
        "--output-dir", str(work_dir),
    ])

    # Stage 3: build_gt_v2_100hz.py (Task 3)
    run_checked([
        sys.executable, str(repo_root / "scripts" / "build_gt_v2_100hz.py"),
        "--graph-dir", str(work_dir / "graph"),
        "--audit-dir", str(work_dir / "audit"),
        "--out-dir", str(work_dir / "poses"),
    ])

    # Stage 4: qualify_gt_v2.py (Task 4)
    run_checked([
        sys.executable, str(repo_root / "scripts" / "qualify_gt_v2.py"),
        "--base-dir", str(work_dir), "--out-dir", str(work_dir / "qualification"),
    ])

    # Stage 5: build_gt_reference_csv.py (Task 5)
    run_checked([
        sys.executable, str(repo_root / "scripts" / "build_gt_reference_csv.py"),
        "--pose", str(work_dir / "poses" / "pose_robot_gt_100hz.csv"),
        "--output", str(work_dir / "reference.csv"),
    ])

    # Stage 6: build_session_dataset.py (unmodified, already compliant)
    dataset_dir.mkdir(parents=True, exist_ok=True)
    run_checked([
        sys.executable, str(repo_root / "scripts" / "build_session_dataset.py"),
        "--session", str(args.session),
        "--reference", str(work_dir / "reference.csv"),
        "--output", str(dataset_dir / "canonical_100hz.csv.gz"),
        "--trajectory", str(dataset_dir / "trajectory_reference.ply"),
        "--session-id", session_id, "--terrain", args.terrain, "--surface", args.surface,
        "--calibration-id", args.calibration,
        "--reference-grade", args.reference_grade,
        "--reference-source", args.reference_source,
        "--reference-world-frame", args.reference_world_frame,
        "--offline-icp", str(args.offline_icp),
    ])

    # Stage 7: copy map reference file(s) atomically. Named after the source
    # file's own stem (not just its suffix) -- multiple --map-reference
    # files sharing an extension (e.g. a KISS-ICP session's map.ply plus a
    # separate trajectory .ply) would otherwise silently collide on the same
    # destination name, with only the last one surviving.
    for src in args.map_reference:
        dst = dataset_dir / f"map_reference_{src.stem}{src.suffix}"
        tmp = dst.with_suffix(dst.suffix + ".tmp")
        tmp.write_bytes(src.read_bytes())
        tmp.replace(dst)

    # Stage 8: preview video (export_bag_preview.py is already generic)
    run_checked([
        sys.executable, str(repo_root / "scripts" / "export_bag_preview.py"),
        str(args.session), str(dataset_dir / "preview.mp4"), "--force",
    ])

    # Stage 9: provenance + catalog
    failures = validate_dataset_dir(dataset_dir)
    if failures:
        raise SystemExit(f"Post-build validation FAILED, dataset NOT marked ready: {failures}")

    manifest = {
        "session_id": session_id,
        "calibration_id": args.calibration,
        "icp_source_path": str(args.offline_icp),
        "icp_approved_by": args.icp_approved_by,
        "git_commit": git_commit_hash(repo_root),
        "git_dirty": git_is_dirty(repo_root),
        "config": vars(args) | {"session": str(args.session), "output": str(args.output),
                                  "offline_icp": str(args.offline_icp),
                                  "map_reference": [str(p) for p in args.map_reference]},
        "files": {
            "canonical_100hz.csv.gz": file_record(dataset_dir / "canonical_100hz.csv.gz"),
            "trajectory_reference.ply": file_record(dataset_dir / "trajectory_reference.ply"),
            "preview.mp4": file_record(dataset_dir / "preview.mp4"),
            **{f"map_reference_{p.stem}{p.suffix}": file_record(
                   dataset_dir / f"map_reference_{p.stem}{p.suffix}")
               for p in args.map_reference},
        },
    }
    manifest_tmp = dataset_dir / "manifest.yaml.tmp"
    with manifest_tmp.open("w") as f:
        yaml.safe_dump(manifest, f, sort_keys=False)
    manifest_tmp.replace(dataset_dir / "manifest.yaml")

    upsert_catalog_row(catalog_path, {
        "session_id": session_id,
        "calibration_id": args.calibration,
        "reference_grade": args.reference_grade,
        "reference_source": args.reference_source,
        "canonical_status": "ready",
        "canonical_sha256": manifest["files"]["canonical_100hz.csv.gz"]["sha256"],
        "trajectory_reference_sha256": manifest["files"]["trajectory_reference.ply"]["sha256"],
        "preview_sha256": manifest["files"]["preview.mp4"]["sha256"],
        "canonical_relpath": relpath_or_absolute(
            dataset_dir / "canonical_100hz.csv.gz", catalog_path.parent),
    })
    print(f"build_gt_pipeline: {session_id} -> {dataset_dir} (ready)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
