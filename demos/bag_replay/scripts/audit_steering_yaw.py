#!/usr/bin/env python3
"""Offline steering/yaw sanity check for an MTT rosbag."""

from __future__ import annotations

import argparse
import math
from collections import defaultdict
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import TwistStamped, TransformStamped
from mtt_msgs.msg import MttTachometerData
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage


def stamp_sec(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def stats(values: list[float]) -> str:
    if not values:
        return "n=0"
    values = sorted(values)
    def pct(p: float) -> float:
        idx = min(len(values) - 1, max(0, int(round((len(values) - 1) * p))))
        return values[idx]
    return (
        f"n={len(values)} min={values[0]:+.4f} p50={pct(0.50):+.4f} "
        f"p95={pct(0.95):+.4f} max={values[-1]:+.4f}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()

    storage_options = rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {
        "/mtt_tachometer",
        "/cmd_vel",
        "/hardware/articulation_angle",
        "/tf",
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in wanted if t in topics]))
    type_cache = {name: get_message(type_name) for name, type_name in topics.items()}

    steer_cmd = []
    speed = []
    signed_speed = []
    cmd_ang = []
    hw_artic = []
    odom_yaw_samples: list[tuple[float, float]] = []

    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, type_cache[topic])
        if topic == "/mtt_tachometer":
            t = stamp_sec(msg)
            sign = -1.0 if msg.direction == "Reverse" else 1.0
            s = msg.speed_ms if msg.speed_ms < -1e-4 else msg.speed_ms * sign
            steer_cmd.append(msg.steer_cmd)
            speed.append(msg.speed_ms)
            signed_speed.append(s)
        elif topic == "/cmd_vel":
            cmd_ang.append(msg.twist.angular.z)
        elif topic == "/hardware/articulation_angle":
            hw_artic.append(msg.data)
        elif topic == "/tf":
            for tf in msg.transforms:
                if tf.header.frame_id == "odom" and tf.child_frame_id == "base_footprint":
                    odom_yaw_samples.append((stamp_sec(tf), yaw_from_quat(tf.transform.rotation)))

    yaw_rates = []
    for (t0, y0), (t1, y1) in zip(odom_yaw_samples, odom_yaw_samples[1:]):
        dt = t1 - t0
        if dt <= 1e-6:
            continue
        dy = math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))
        yaw_rates.append(dy / dt)

    print("STEERING/YAW AUDIT")
    print(f"bag={args.bag}")
    print(f"steer_cmd(rad-or-norm): {stats(steer_cmd)}")
    print(f"speed_ms(raw):          {stats(speed)}")
    print(f"signed_speed_ms:        {stats(signed_speed)}")
    print(f"cmd_vel.angular.z:      {stats(cmd_ang)}")
    print(f"hardware_artic_rad:     {stats(hw_artic)}")
    print(f"recorded odom yaw_rate: {stats(yaw_rates)}")
    if steer_cmd and hw_artic:
        print(
            "NOTE: compare steer_cmd to hardware_artic_rad. If steer_cmd is already radians, "
            "using normalized_steer_to_articulation_rad() multiplies it by 60deg again."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
