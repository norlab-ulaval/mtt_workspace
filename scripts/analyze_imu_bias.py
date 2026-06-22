#!/usr/bin/env python3
"""Measure gyro bias and quaternion yaw drift in a recorded bag.

Reads the first N seconds of /mti100/data, /mti100/data_unbiased and
/mti100/data_raw, then reports per-topic:
  - mean/std of angular_velocity.z (gyro bias when stationary)
  - yaw extracted from the orientation quaternion at start vs end
    (drift rate while the robot is stationary)

Usage:
  python3 scripts/analyze_imu_bias.py <bag_dir> [--window-s 80]
"""

import argparse
import math
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

TOPICS = ["/mti100/data", "/mti100/data_unbiased", "/mti100/data_raw"]


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir")
    parser.add_argument("--window-s", type=float, default=80.0)
    args = parser.parse_args()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag_dir, storage_id="mcap"),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in TOPICS if t in type_map]
    if not wanted:
        sys.exit(f"none of {TOPICS} found in bag")
    reader.set_filter(__import__("rosbag2_py").StorageFilter(topics=wanted))

    stats = {
        t: {"n": 0, "sum_z": 0.0, "sum_z2": 0.0,
            "first_yaw": None, "last_yaw": None,
            "first_t": None, "last_t": None,
            "orient_valid": None}
        for t in wanted
    }
    msg_types = {t: get_message(type_map[t]) for t in wanted}

    t0 = None
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if t0 is None:
            t0 = t_ns
        if (t_ns - t0) * 1e-9 > args.window_s:
            break
        msg = deserialize_message(raw, msg_types[topic])
        s = stats[topic]
        s["n"] += 1
        z = msg.angular_velocity.z
        s["sum_z"] += z
        s["sum_z2"] += z * z
        yaw = quat_to_yaw(msg.orientation)
        if s["first_yaw"] is None:
            s["first_yaw"] = yaw
            s["first_t"] = t_ns
            s["orient_valid"] = msg.orientation_covariance[0] >= 0.0
        s["last_yaw"] = yaw
        s["last_t"] = t_ns

    for topic in wanted:
        s = stats[topic]
        if s["n"] == 0:
            print(f"{topic}: no messages in window")
            continue
        mean = s["sum_z"] / s["n"]
        var = max(s["sum_z2"] / s["n"] - mean * mean, 0.0)
        dt = (s["last_t"] - s["first_t"]) * 1e-9
        dyaw = math.atan2(math.sin(s["last_yaw"] - s["first_yaw"]),
                          math.cos(s["last_yaw"] - s["first_yaw"]))
        drift_deg_s = math.degrees(dyaw) / dt if dt > 0 else float("nan")
        print(f"{topic}")
        print(f"  msgs              : {s['n']} over {dt:.1f}s")
        print(f"  gyro z mean (bias): {mean:+.5f} rad/s")
        print(f"  gyro z std        : {math.sqrt(var):.5f} rad/s")
        print(f"  orientation valid : {s['orient_valid']}")
        print(f"  yaw start → end   : {math.degrees(s['first_yaw']):+.2f}° → "
              f"{math.degrees(s['last_yaw']):+.2f}°  "
              f"(drift {drift_deg_s:+.4f}°/s = "
              f"{drift_deg_s * 60:+.2f}°/min)")


if __name__ == "__main__":
    main()
