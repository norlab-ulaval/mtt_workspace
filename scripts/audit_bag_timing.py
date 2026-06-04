#!/usr/bin/env python3
from __future__ import annotations

import argparse
import statistics
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


DEFAULT_TOPICS = {
    "/hesai_lidar/points",
    "/hesai_lidar/lidar_packets_loss",
    "/mapping/scan_after_input_filters",
    "/mapping/scan_after_deskew",
    "/mapping/aligned_scan",
    "/mapping/icp_odom",
    "/mapping/map",
    "/mtt_odometry",
    "/mtt_tachometer",
    "/tf",
    "/tf_static",
    "/zed/zed_node/odom",
    "/mti100/data",
}


def resolve_bag(path: Path) -> Path:
    if (path / "metadata.yaml").exists():
        return path
    if (path / "bag" / "metadata.yaml").exists():
        return path / "bag"
    raise FileNotFoundError(f"{path} does not contain metadata.yaml or bag/metadata.yaml")


def stamp_s(msg: Any, bag_time_ns: int) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and (stamp.sec != 0 or stamp.nanosec != 0):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(bag_time_ns) * 1e-9


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = int(round((pct / 100.0) * (len(ordered) - 1)))
    return ordered[max(0, min(len(ordered) - 1, idx))]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--topic", action="append", dest="topics")
    args = parser.parse_args()

    bag = resolve_bag(Path(args.bag))
    wanted = set(args.topics) if args.topics else DEFAULT_TOPICS

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_by_topic = {info.name: info.type for info in reader.get_all_topics_and_types()}
    msg_types = {
        topic: get_message(type_by_topic[topic])
        for topic in wanted
        if topic in type_by_topic
    }
    stats = {
        topic: {
            "n": 0,
            "first": None,
            "last": None,
            "prev": None,
            "gaps": [],
            "points": [],
            "sizes": [],
        }
        for topic in msg_types
    }
    map_samples: list[tuple[float, int, int]] = []

    while reader.has_next():
        topic, raw, bag_time_ns = reader.read_next()
        msg_type = msg_types.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(raw, msg_type)
        t = stamp_s(msg, bag_time_ns)
        row = stats[topic]
        row["n"] += 1
        if row["first"] is None:
            row["first"] = t
        row["last"] = t
        if row["prev"] is not None:
            row["gaps"].append(t - row["prev"])
        row["prev"] = t

        if hasattr(msg, "width") and hasattr(msg, "height") and hasattr(msg, "data"):
            points = int(msg.width) * int(msg.height)
            size = len(msg.data)
            row["points"].append(points)
            row["sizes"].append(size)
            if topic == "/mapping/map":
                map_samples.append((t, points, size))

    print(f"bag: {bag}")
    print("== Topic Timing ==")
    for topic in sorted(stats):
        row = stats[topic]
        n = int(row["n"])
        if n == 0:
            continue
        first = float(row["first"])
        last = float(row["last"])
        duration = max(0.0, last - first)
        hz = (n - 1) / duration if n > 1 and duration > 0.0 else 0.0
        line = f"{topic}: n={n} first={first:.3f} last={last:.3f} dur={duration:.1f}s hz={hz:.2f}"
        gaps = row["gaps"]
        if gaps:
            line += (
                f" gap_med={statistics.median(gaps):.3f}s"
                f" p99={percentile(gaps, 99):.3f}s"
                f" max={max(gaps):.3f}s"
                f" gaps>0.2={sum(g > 0.2 for g in gaps)}"
            )
        points = row["points"]
        if points:
            sizes = row["sizes"]
            line += (
                f" pts_med={int(statistics.median(points))}"
                f" pts_max={max(points)}"
                f" MB_med={statistics.median(sizes) / 1e6:.2f}"
            )
        print(line)

    if map_samples:
        print("== /mapping/map Samples ==")
        for sample in map_samples[:5]:
            print(f"first t={sample[0]:.3f} points={sample[1]} MB={sample[2] / 1e6:.2f}")
        for sample in map_samples[-5:]:
            print(f"last  t={sample[0]:.3f} points={sample[1]} MB={sample[2] / 1e6:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
