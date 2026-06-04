#!/usr/bin/env python3
"""Batch runner for the direct offline ICP engine."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[1]


def resolve_sessions(path: Path) -> list[Path]:
    path = path.expanduser()
    if path.is_file():
        return [
            Path(line.strip()).expanduser()
            for line in path.read_text().splitlines()
            if line.strip() and not line.strip().startswith("#")
        ]
    if (path / "bag").exists():
        return [path]
    return sorted(p for p in path.iterdir() if (p / "bag").exists())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_path", type=Path)
    parser.add_argument("--profile", choices=["hesai_imu", "hesai_wheel", "hesai_lidar_only"], default="hesai_imu")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--max-clouds", type=int, default=0)
    parser.add_argument("--checkpoint-every-scans", type=int, default=500)
    parser.add_argument("--local-map-radius", type=float, default=60.0)
    parser.add_argument("--global-min-dist-new-point", type=float, default=0.05)
    parser.add_argument("--ros-domain-id", type=int, default=77)
    return parser.parse_args()


def main() -> int:
    root = workspace_root()
    args = parse_args()
    sessions = resolve_sessions(args.input_path)
    failures = 0
    for index, session in enumerate(sessions, start=1):
        session = session if session.is_absolute() else root / session
        run_name = f"direct_{args.profile}"
        out = session / "offline_icp_direct_runs" / run_name
        quality = out / "quality.yaml"
        if quality.exists() and not args.force:
            print(f"[{index}/{len(sessions)}] {session.name}: SKIP existing {quality}", flush=True)
            continue

        print(f"[{index}/{len(sessions)}] {session.name}: direct {args.profile}", flush=True)
        cmd = [
            sys.executable,
            str(root / "scripts/direct_offline_icp.py"),
            str(session),
            "--profile", args.profile,
            "--experiment-name", run_name,
            "--checkpoint-every-scans", str(args.checkpoint_every_scans),
            "--local-map-radius", str(args.local_map_radius),
            "--global-min-dist-new-point", str(args.global_min_dist_new_point),
            "--max-clouds", str(args.max_clouds),
            "--ros-domain-id", str(args.ros_domain_id),
        ]
        code = subprocess.call(cmd, cwd=str(root))
        if code != 0:
            failures += 1
            print(f"  FAILED code={code}", flush=True)
        else:
            print(f"  OK {out}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
