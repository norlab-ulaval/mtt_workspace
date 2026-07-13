#!/usr/bin/env python3
"""Build quality-labelled motion research datasets from postprocess outputs."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from lib.mtt_motion_research import (
    assign_segments,
    derive_research_rows,
    find_motion_csv,
    finite_stats,
    parse_bool,
    read_csv_rows,
    resolve_sessions,
    segment_summary,
    write_csv_rows,
)


def build_session(session: Path, args: argparse.Namespace) -> dict:
    source_csv = find_motion_csv(session)
    out_dir = args.output_dir / session.name if args.output_dir else session / "motion_research"
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = assign_segments(
        derive_research_rows(
            read_csv_rows(source_csv),
            session_name=session.name,
            articulation_preference=args.articulation_source,
            max_icp_gap_s=args.max_icp_gap_s,
            max_icp_step_m=args.max_icp_step_m,
            max_icp_speed_ms=args.max_icp_speed_ms,
            max_icp_yaw_rate_rad_s=args.max_icp_yaw_rate_rad_s,
        ),
        min_segment_s=args.min_segment_s,
        max_gap_s=args.segment_gap_s,
    )
    segments = segment_summary(rows)
    dataset_csv = out_dir / "dataset.csv"
    segments_csv = out_dir / "segments.csv"
    write_csv_rows(dataset_csv, rows)
    write_csv_rows(segments_csv, segments)
    valid = [row for row in rows if parse_bool(row.get("research_quality_ok"))]
    real_tacho = [row for row in rows if parse_bool(row.get("has_real_tacho_research"))]
    summary = {
        "session": session.name,
        "source_csv": str(source_csv),
        "output_dir": str(out_dir),
        "rows": len(rows),
        "valid_rows": len(valid),
        "valid_ratio": len(valid) / max(len(rows), 1),
        "real_tacho_rows": len(real_tacho),
        "real_tacho_ratio": len(real_tacho) / max(len(rows), 1),
        "segments": len(segments),
        "motion_class_counts": {
            klass: sum(1 for row in rows if row.get("motion_class") == klass)
            for klass in sorted({str(row.get("motion_class", "")) for row in rows})
        },
        "quality_class_counts": {
            klass: sum(1 for row in rows if row.get("quality_class") == klass)
            for klass in sorted({str(row.get("quality_class", "")) for row in rows})
        },
        "delta_kappa_exact": finite_stats(float(row["delta_kappa_exact"]) for row in valid if str(row.get("delta_kappa_exact")) != "nan"),
        "outputs": {"dataset_csv": str(dataset_csv), "segments_csv": str(segments_csv)},
    }
    (out_dir / "quality_summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="Session directory or parent directory containing sessions")
    parser.add_argument("--output-dir", type=Path, help="Optional global output directory")
    parser.add_argument("--articulation-source", default="auto", choices=["auto", "mtt", "trailer", "hardware"])
    parser.add_argument("--max-icp-gap-s", type=float, default=0.35)
    parser.add_argument("--max-icp-step-m", type=float, default=1.0)
    parser.add_argument("--max-icp-speed-ms", type=float, default=8.0)
    parser.add_argument("--max-icp-yaw-rate-rad-s", type=float, default=2.5)
    parser.add_argument("--min-segment-s", type=float, default=1.0)
    parser.add_argument("--segment-gap-s", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sessions = resolve_sessions(args.path)
    summaries = []
    for session in sessions:
        try:
            summaries.append(build_session(session, args))
            print(f"[ok] {session.name}")
        except Exception as exc:  # noqa: BLE001
            summaries.append({"session": session.name, "status": "failed", "error": str(exc)})
            print(f"[failed] {session.name}: {exc}")
    result = {"sessions": len(sessions), "ok": sum("error" not in s for s in summaries), "summaries": summaries}
    print(yaml.safe_dump(result, sort_keys=False))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
