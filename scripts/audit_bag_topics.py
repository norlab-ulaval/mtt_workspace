#!/usr/bin/env python3
"""Compare one bag against the expected MTT topic list."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import yaml


def infer_workspace_root(script_path: Path) -> Path:
    for candidate in [script_path.parent, *script_path.parents]:
        if (candidate / "src").exists() and (candidate / "demos").exists():
            return candidate
    return script_path.parent


def resolve_bag_dir(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.is_file():
        return path.parent
    if (path / "metadata.yaml").is_file():
        return path
    if (path / "bag" / "metadata.yaml").is_file():
        return path / "bag"
    raise SystemExit(f"Could not resolve bag directory from: {path}")


def load_expected_topics(config_path: Path, key: str) -> list[str]:
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    topics = data.get(key, [])
    if not isinstance(topics, list) or not topics:
        raise SystemExit(f"No topics found under key '{key}' in {config_path}")
    return topics


def load_bag_metadata(metadata_path: Path) -> tuple[dict[str, int], int, int]:
    with metadata_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    info = data.get("rosbag2_bagfile_information", data)
    counts = {
        entry["topic_metadata"]["name"]: int(entry["message_count"])
        for entry in info.get("topics_with_message_count", [])
    }
    duration_ns = int(info.get("duration", {}).get("nanoseconds", 0))
    total_messages = int(info.get("message_count", 0))
    return counts, duration_ns, total_messages


def group_name(topic: str) -> str:
    if topic.startswith("/zed/"):
        return "ZED"
    if topic.startswith("/oak/"):
        return "OAK"
    if topic.startswith("/hesai") or topic.startswith("/rsairy") or topic == "/lidar_packets":
        return "LiDAR"
    if topic.startswith("/gps"):
        return "GPS"
    if topic.startswith("/mti"):
        return "IMU"
    if topic.startswith("/mapping") or topic.startswith("/localization") or topic.startswith("/trailer") or topic.startswith("/merged_points"):
        return "Mapping"
    if topic.startswith("/mtt") or topic in {"/from_can_bus", "/to_can_bus"}:
        return "MTT/CAN"
    if topic in {"/tf", "/tf_static", "/robot_description", "/joint_states", "/runtime_joint_states", "/clock", "/session/events"}:
        return "Infra"
    if topic.startswith("/cmd_vel") or topic in {"/controller/cmd_vel", "/teleop_estop", "/joy"}:
        return "Control"
    return "Other"


def main() -> int:
    script_path = Path(__file__).resolve()
    workspace_root = infer_workspace_root(script_path)

    parser = argparse.ArgumentParser(description="Audit one bag against the expected MTT topic list.")
    parser.add_argument("bag", help="Session directory, bag directory, or bag_0.mcap path.")
    parser.add_argument(
        "--expected-topics",
        default=str(workspace_root / "src/external/norlab_robot/config/rosbag_record/all_sensors_full.yaml"),
        help="YAML file that defines the expected topic list.",
    )
    parser.add_argument("--topics-key", default="topics_main", help="Key inside the expected-topics YAML.")
    parser.add_argument("--show-ok", action="store_true", help="Also print topics that were recorded correctly.")
    args = parser.parse_args()

    bag_dir = resolve_bag_dir(Path(args.bag))
    metadata_path = bag_dir / "metadata.yaml"
    expected_topics = load_expected_topics(Path(args.expected_topics), args.topics_key)
    counts, duration_ns, total_messages = load_bag_metadata(metadata_path)
    duration_s = duration_ns / 1e9 if duration_ns else 0.0

    missing = [t for t in expected_topics if t not in counts]
    zero = [t for t in expected_topics if counts.get(t, -1) == 0]
    recorded = [t for t in expected_topics if counts.get(t, 0) > 0]
    extra = sorted(set(counts) - set(expected_topics))

    print(f"Bag:           {bag_dir}")
    print(f"Duration:      {duration_s:.1f}s")
    print(f"Total msgs:    {total_messages}")
    print(f"Expected:      {len(expected_topics)} topics")
    print(f"Recorded > 0:  {len(recorded)}")
    print(f"Zero-count:    {len(zero)}")
    print(f"Missing:       {len(missing)}")
    print(f"Extra:         {len(extra)}")

    def print_topics(title: str, topics: list[str], include_counts: bool = False) -> None:
        if not topics:
            return
        print(f"\n{title}")
        last_group = None
        for topic in topics:
            group = group_name(topic)
            if group != last_group:
                print(f"  [{group}]")
                last_group = group
            if include_counts:
                print(f"    {topic} : {counts.get(topic, 0)}")
            else:
                print(f"    {topic}")

    print_topics("Missing from bag metadata", missing)
    print_topics("Present but zero messages", zero)
    if args.show_ok:
        print_topics("Recorded with messages", recorded, include_counts=True)
    else:
        print_topics("Recorded with messages (camera / 3D focus)", [
            t for t in recorded if group_name(t) in {"ZED", "OAK", "LiDAR", "Mapping"}
        ], include_counts=True)
    print_topics("Extra recorded topics", extra, include_counts=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
