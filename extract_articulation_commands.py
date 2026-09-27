#!/usr/bin/env python3
"""Export recorded articulation commands to CSV using the built ROS workspace.

Only setpoint_rad is converted to degrees; normalized steering is not an angle.
Existing output files are never overwritten.
"""
import argparse
import csv
import math
from contextlib import ExitStack
from pathlib import Path

from extract_mathis_topics import find_mcap_bag_directory

TOPICS = {
    "/articulation_servo/setpoint_rad": "articulation_servo_setpoint_rad.csv",
    "/articulation_servo/steer_cmd": "articulation_servo_steer_cmd.csv",
    "/mtt/articulation_cmd": "mtt_articulation_cmd.csv",
    "/mtt_articulation_setpoint": "mtt_articulation_setpoint.csv",
    "/mtt_control/com_steer": "mtt_control_com_steer.csv",
}


def command_degrees(topic, value):
    return math.degrees(value) if topic == "/articulation_servo/setpoint_rad" else ""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-path", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        parser.error(f"Source ROS 2 and the built workspace: {error}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=find_mcap_bag_directory(args.bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {info.name: info.type for info in reader.get_all_topics_and_types()}
    present = {topic: filename for topic, filename in TOPICS.items() if topic in types}
    if not present:
        parser.error("No articulation command topics found in the bag")
    allowed_types = {"std_msgs/msg/Float32", "std_msgs/msg/Float64"}
    for topic in present:
        if types[topic] not in allowed_types:
            parser.error(f"Unsupported scalar type for {topic}: {types[topic]}")
        if (args.output_dir / present[topic]).exists():
            parser.error(f"Output already exists: {args.output_dir / present[topic]}")
    message_types = {topic: get_message(types[topic]) for topic in present}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(present)))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    counts = dict.fromkeys(present, 0)
    start = None
    with ExitStack() as stack:
        writers = {}
        for topic, filename in present.items():
            stream = stack.enter_context((args.output_dir / filename).open("x", newline=""))
            writers[topic] = csv.writer(stream)
            writers[topic].writerow([
                "timestamp_sec", "timestamp_nanosec", "time_relative_sec",
                "command_value", "command_value_deg",
            ])
        while reader.has_next():
            topic, raw, timestamp = reader.read_next()
            if topic not in writers:
                continue
            if start is None:
                start = timestamp
            value = deserialize_message(raw, message_types[topic]).data
            writers[topic].writerow([
                f"{timestamp * 1e-9:.9f}", timestamp, f"{(timestamp - start) * 1e-9:.9f}",
                value, command_degrees(topic, value),
            ])
            counts[topic] += 1
    for topic, count in counts.items():
        print(f"{topic}: {count} rows -> {args.output_dir / present[topic]}")


if __name__ == "__main__":
    main()
