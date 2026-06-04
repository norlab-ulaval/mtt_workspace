#!/usr/bin/env python3
"""Run the direct rosbag2 -> libpointmatcher offline ICP engine.

This wrapper intentionally avoids ros2 bag play, DDS pub/sub, /clock, and topic
recording. It only starts the C++ engine installed in norlab_icp_mapper_ros.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_config(root: Path, profile: str) -> Path:
    mapping = root / "src/external/norlab_robot/config/mapping"
    if profile == "hesai_wheel":
        return mapping / "_config_hesai_wheel_replay.yaml"
    if profile == "hesai_imu":
        return mapping / "_config_hesai_imu_replay.yaml"
    return mapping / "_config_hesai_imu_replay.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="Session directory containing bag/")
    parser.add_argument("--profile", choices=["hesai_imu", "hesai_wheel", "hesai_lidar_only"], default="hesai_imu")
    parser.add_argument("--points-topic", default="/hesai_lidar/points")
    parser.add_argument("--mapping-config", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--experiment-name", default=None)
    parser.add_argument("--local-map-radius", type=float, default=60.0)
    parser.add_argument("--global-min-dist-new-point", type=float, default=0.05)
    parser.add_argument("--local-min-dist-new-point", type=float, default=0.08)
    parser.add_argument("--checkpoint-every-scans", type=int, default=500)
    parser.add_argument("--max-clouds", type=int, default=0)
    parser.add_argument("--ros-domain-id", type=int, default=77)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = workspace_root()
    session = args.session.expanduser()
    if not session.is_absolute():
        session = (root / session).resolve()
    else:
        session = session.resolve()
    bag = session / "bag"
    if not bag.exists():
        raise SystemExit(f"bag directory not found: {bag}")

    mapping_config = (args.mapping_config or default_config(root, args.profile)).expanduser().resolve()
    if not mapping_config.exists():
        raise SystemExit(f"mapping config not found: {mapping_config}")

    run_name = args.experiment_name or (
        datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + f"_direct_{args.profile}"
    )
    output = args.output.expanduser().resolve() if args.output else session / "offline_icp_direct_runs" / run_name
    output.mkdir(parents=True, exist_ok=True)

    command = [
        "ros2", "run", "norlab_icp_mapper_ros", "offline_icp_engine",
        "--bag", str(bag),
        "--output", str(output),
        "--mapping-config", str(mapping_config),
        "--points-topic", args.points_topic,
        "--profile", args.profile,
        "--local-map-radius", str(args.local_map_radius),
        "--global-min-dist-new-point", str(args.global_min_dist_new_point),
        "--local-min-dist-new-point", str(args.local_min_dist_new_point),
        "--checkpoint-every-scans", str(args.checkpoint_every_scans),
        "--max-clouds", str(args.max_clouds),
    ]

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    env.setdefault("ROS_LOG_DIR", str(output / "ros_logs"))

    print("Output:", output, flush=True)
    print("Command:", " ".join(command), flush=True)
    return subprocess.call(command, cwd=str(root), env=env)


if __name__ == "__main__":
    raise SystemExit(main())
