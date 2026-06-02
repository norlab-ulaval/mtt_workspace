#!/usr/bin/env python3
"""Offline audit for MTT odometry sign, command direction, and odom TF resets."""

from __future__ import annotations

import argparse
import bisect
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import rosbag2_py
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage


def stamp_s_from_msg(msg: Any) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Cmd:
    t: float
    linear_x: float
    angular_z: float


@dataclass
class Tacho:
    t: float
    speed_ms: float
    signed_speed_ms: float
    direction: str
    distance_km: float
    cumulative: int
    steer_cmd: float
    synthetic: bool
    source: str


@dataclass
class Pose:
    t: float
    x: float
    y: float
    yaw: float
    frame_id: str
    child_frame_id: str


def sign(value: float, eps: float = 1e-3) -> int:
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


def nearest_by_time(samples: list[Any], times: list[float], t: float, max_dt: float) -> Optional[Any]:
    if not samples:
        return None
    i = bisect.bisect_left(times, t)
    candidates = []
    if i < len(samples):
        candidates.append(samples[i])
    if i > 0:
        candidates.append(samples[i - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda sample: abs(sample.t - t))
    return best if abs(best.t - t) <= max_dt else None


def pose_from_odom(msg: Odometry) -> Pose:
    p = msg.pose.pose.position
    return Pose(
        stamp_s_from_msg(msg),
        float(p.x),
        float(p.y),
        yaw_from_quat(msg.pose.pose.orientation),
        str(msg.header.frame_id),
        str(msg.child_frame_id),
    )


def pose_from_tf(stamped: Any) -> Pose:
    t = stamped.transform.translation
    return Pose(
        float(stamped.header.stamp.sec) + float(stamped.header.stamp.nanosec) * 1e-9,
        float(t.x),
        float(t.y),
        yaw_from_quat(stamped.transform.rotation),
        str(stamped.header.frame_id),
        str(stamped.child_frame_id),
    )


def projected_body_step(prev: Pose, cur: Pose) -> float:
    dx = cur.x - prev.x
    dy = cur.y - prev.y
    return dx * math.cos(prev.yaw) + dy * math.sin(prev.yaw)


def dist_from_origin(pose: Pose, origin: Pose) -> float:
    return math.hypot(pose.x - origin.x, pose.y - origin.y)


def load_bag(bag_path: Path, topics: set[str]) -> tuple[dict[str, list[Any]], list[str]]:
    metadata = bag_path / "metadata.yaml"
    if not metadata.exists() and (bag_path / "bag" / "metadata.yaml").exists():
        bag_path = bag_path / "bag"
    if not (bag_path / "metadata.yaml").exists():
        raise FileNotFoundError(f"{bag_path} does not contain metadata.yaml")

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    type_by_topic = {
        info.name: info.type for info in reader.get_all_topics_and_types()
    }
    missing = sorted(topics - set(type_by_topic))
    msg_type_by_topic = {
        topic: get_message(type_by_topic[topic])
        for topic in topics
        if topic in type_by_topic
    }
    data: dict[str, list[Any]] = {topic: [] for topic in topics}

    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in msg_type_by_topic:
            continue
        msg = deserialize_message(raw, msg_type_by_topic[topic])
        data[topic].append(msg)

    return data, missing


def print_static_tf_summary(static_msgs: list[TFMessage]) -> None:
    wanted = {
        ("base_footprint", "base_link"),
        ("base_link", "reference_point"),
        ("reference_point", "center_lidar_link"),
        ("center_lidar_link", "hesai_lidar"),
        ("base_link", "hesai_lidar"),
        ("base_footprint", "hesai_lidar"),
    }
    seen = {}
    for msg in static_msgs:
        for tf in msg.transforms:
            key = (tf.header.frame_id, tf.child_frame_id)
            if key in wanted:
                seen[key] = tf

    print("\n== Static TF Recorded In Bag ==")
    if not seen:
        print("No direct expected static TF edges found in recorded /tf_static.")
        print("Replay excludes bag /tf_static and uses current robot_state_publisher URDF.")
        return
    for key, tf in sorted(seen.items()):
        tr = tf.transform.translation
        yaw = math.degrees(yaw_from_quat(tf.transform.rotation))
        print(
            f"{key[0]} -> {key[1]}: "
            f"xyz=({tr.x:+.3f},{tr.y:+.3f},{tr.z:+.3f}) yaw={yaw:+.1f}deg"
        )


def audit_direction(cmds: list[Cmd], tachos: list[Tacho], max_dt: float) -> None:
    cmd_times = [cmd.t for cmd in cmds]
    conflicts = []
    reverse = 0
    forward = 0
    moving = 0
    for tacho in tachos:
        if abs(tacho.speed_ms) < 0.03:
            continue
        moving += 1
        if tacho.direction == "Reverse":
            reverse += 1
        elif tacho.direction == "Forward":
            forward += 1
        cmd = nearest_by_time(cmds, cmd_times, tacho.t, max_dt)
        if not cmd or abs(cmd.linear_x) < 0.03:
            continue
        if sign(cmd.linear_x) != sign(tacho.signed_speed_ms):
            conflicts.append((tacho, cmd))

    print("\n== Command vs Tachometer Direction ==")
    print(f"moving_tacho_samples={moving} forward={forward} reverse={reverse}")
    print(f"cmd/tacho sign conflicts within {max_dt:.2f}s: {len(conflicts)}")
    for tacho, cmd in conflicts[:12]:
        print(
            f"  t={tacho.t:.3f} cmd.linear.x={cmd.linear_x:+.3f} "
            f"tacho={tacho.direction} speed={tacho.speed_ms:+.3f} "
            f"signed={tacho.signed_speed_ms:+.3f}"
        )


def audit_tachometer_deltas(tachos: list[Tacho], jump_thresh: float) -> None:
    print("\n== Tachometer Distance Deltas ==")
    if len(tachos) < 2:
        print("Not enough tachometer samples.")
        return

    anomalies = []
    signed_total = 0.0
    for prev, cur in zip(tachos, tachos[1:]):
        dt = max(cur.t - prev.t, 1e-9)
        delta_m_abs = (cur.distance_km - prev.distance_km) * 1000.0
        signed_delta_m = delta_m_abs * (-1.0 if cur.direction == "Reverse" else 1.0)
        signed_total += signed_delta_m
        implied_speed = signed_delta_m / dt
        tick_delta = cur.cumulative - prev.cumulative
        if abs(signed_delta_m) > jump_thresh or abs(implied_speed) > 8.0:
            anomalies.append((prev, cur, signed_delta_m, implied_speed, tick_delta, dt))

    print(f"signed_distance_sum={signed_total:+.3f}m")
    print(f"delta anomalies: {len(anomalies)} (|delta|>{jump_thresh:.2f}m or |speed|>8m/s)")
    for prev, cur, signed_delta_m, implied_speed, tick_delta, dt in anomalies[:20]:
        print(
            f"  t={cur.t:.3f} dt={dt:.3f}s signed_delta={signed_delta_m:+.3f}m "
            f"implied_speed={implied_speed:+.1f}m/s tick_delta={tick_delta:+d} "
            f"dir={cur.direction} speed_msg={cur.speed_ms:+.3f}m/s "
            f"distance_km={prev.distance_km:.6f}->{cur.distance_km:.6f}"
        )


def audit_pose_series(
    name: str,
    poses: list[Pose],
    tachos: list[Tacho],
    max_dt: float,
    jump_thresh: float,
) -> None:
    tacho_times = [sample.t for sample in tachos]
    sign_conflicts = []
    jumps = []
    origin_snaps = []
    if not poses:
        print(f"\n== {name} ==\nNo samples.")
        return

    origin = poses[0]
    total_forward = 0.0
    for prev, cur in zip(poses, poses[1:]):
        dt = max(cur.t - prev.t, 1e-9)
        step = math.hypot(cur.x - prev.x, cur.y - prev.y)
        forward_step = projected_body_step(prev, cur)
        total_forward += forward_step
        tacho = nearest_by_time(tachos, tacho_times, cur.t, max_dt)
        if tacho and abs(tacho.signed_speed_ms) > 0.05 and abs(forward_step / dt) > 0.05:
            if sign(tacho.signed_speed_ms) != sign(forward_step):
                sign_conflicts.append((prev, cur, tacho, forward_step, dt))
        if step > jump_thresh:
            jumps.append((prev, cur, step, dt))
        prev_d0 = dist_from_origin(prev, origin)
        cur_d0 = dist_from_origin(cur, origin)
        if prev_d0 > 0.75 and cur_d0 + 0.40 < prev_d0:
            origin_snaps.append((prev, cur, prev_d0, cur_d0, dt))

    print(f"\n== {name} ==")
    print(f"samples={len(poses)} duration={poses[-1].t - poses[0].t:.2f}s")
    print(
        f"start=({origin.x:+.3f},{origin.y:+.3f},{math.degrees(origin.yaw):+.1f}deg) "
        f"end=({poses[-1].x:+.3f},{poses[-1].y:+.3f},{math.degrees(poses[-1].yaw):+.1f}deg)"
    )
    print(f"integrated_body_x_step_sum={total_forward:+.3f}m")
    print(f"jumps>{jump_thresh:.2f}m={len(jumps)} origin_snaps={len(origin_snaps)}")
    print(f"pose/tacho sign conflicts within {max_dt:.2f}s: {len(sign_conflicts)}")
    for prev, cur, tacho, forward_step, dt in sign_conflicts[:12]:
        print(
            f"  SIGN t={cur.t:.3f} pose_body_step={forward_step:+.3f}m "
            f"({forward_step / dt:+.2f}m/s) tacho={tacho.direction} "
            f"signed_speed={tacho.signed_speed_ms:+.2f}m/s"
        )
    for prev, cur, step, dt in jumps[:8]:
        print(
            f"  JUMP t={cur.t:.3f} step={step:.3f}m dt={dt:.3f}s "
            f"from=({prev.x:+.3f},{prev.y:+.3f}) to=({cur.x:+.3f},{cur.y:+.3f})"
        )
    for prev, cur, prev_d0, cur_d0, dt in origin_snaps[:8]:
        print(
            f"  ORIGIN_SNAP t={cur.t:.3f} d0 {prev_d0:.3f}->{cur_d0:.3f}m "
            f"dt={dt:.3f}s pose=({cur.x:+.3f},{cur.y:+.3f})"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--max-sync-dt", type=float, default=0.10)
    parser.add_argument("--jump-thresh", type=float, default=0.50)
    args = parser.parse_args()

    data, missing = load_bag(
        args.bag,
        {"/cmd_vel", "/mtt_tachometer", "/mtt_odometry", "/tf", "/tf_static"},
    )
    if missing:
        print(f"Missing topics: {', '.join(missing)}")

    cmds = [
        Cmd(stamp_s_from_msg(msg), float(msg.twist.linear.x), float(msg.twist.angular.z))
        for msg in data.get("/cmd_vel", [])
        if isinstance(msg, TwistStamped)
    ]
    tachos = [
        Tacho(
            stamp_s_from_msg(msg),
            float(msg.speed_ms),
            float(msg.speed_ms) * (-1.0 if msg.direction == "Reverse" else 1.0),
            str(msg.direction),
            float(msg.distance_km),
            int(msg.tachometer_cumulative),
            float(msg.steer_cmd),
            bool(msg.tachometer_is_synthetic),
            str(msg.tachometer_source),
        )
        for msg in data.get("/mtt_tachometer", [])
    ]
    odoms = [pose_from_odom(msg) for msg in data.get("/mtt_odometry", []) if isinstance(msg, Odometry)]
    odom_tfs: list[Pose] = []
    for msg in data.get("/tf", []):
        if not isinstance(msg, TFMessage):
            continue
        for tf in msg.transforms:
            if tf.header.frame_id == "odom" and tf.child_frame_id == "base_footprint":
                odom_tfs.append(pose_from_tf(tf))

    print(f"bag={args.bag}")
    print(f"cmd_vel={len(cmds)} tachometer={len(tachos)} mtt_odometry={len(odoms)} odom_tf={len(odom_tfs)}")
    if tachos:
        print(
            f"tachometer source(s)={sorted(set(t.source for t in tachos))} "
            f"synthetic={sorted(set(t.synthetic for t in tachos))}"
        )
        print(
            f"distance_km range={tachos[0].distance_km:.6f}->{tachos[-1].distance_km:.6f} "
            f"cumulative_ticks={tachos[0].cumulative}->{tachos[-1].cumulative}"
        )

    print_static_tf_summary(data.get("/tf_static", []))
    audit_direction(cmds, tachos, args.max_sync_dt)
    audit_tachometer_deltas(tachos, args.jump_thresh)
    audit_pose_series("Recorded /mtt_odometry", odoms, tachos, args.max_sync_dt, args.jump_thresh)
    audit_pose_series("Recorded /tf odom->base_footprint", odom_tfs, tachos, args.max_sync_dt, args.jump_thresh)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
