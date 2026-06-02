#!/usr/bin/env python3
"""Fast field check for /mapping/icp_odom quality before WILN repeat."""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
    )


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Sample:
    t: float
    x: float
    y: float
    z: float
    yaw: float
    frame_id: str
    child_frame_id: str


class IcpCheckNode(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("mtt_icp_odom_check")
        self.samples: list[Sample] = []
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Odometry, topic, self._on_odom, qos)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.samples.append(
            Sample(
                t=time.monotonic(),
                x=float(p.x),
                y=float(p.y),
                z=float(p.z),
                yaw=yaw_from_quat(q),
                frame_id=str(msg.header.frame_id),
                child_frame_id=str(msg.child_frame_id),
            )
        )


def fmt_bool(ok: bool) -> str:
    return f"{GREEN}OK{RESET}" if ok else f"{RED}FAIL{RESET}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/mapping/icp_odom")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--min-hz", type=float, default=6.0)
    parser.add_argument("--max-gap-s", type=float, default=0.35)
    parser.add_argument("--max-age-s", type=float, default=0.75)
    parser.add_argument("--max-xy-jump-m", type=float, default=0.75)
    parser.add_argument("--max-z-jump-m", type=float, default=0.75)
    parser.add_argument("--max-yaw-jump-rad", type=float, default=0.61)
    parser.add_argument("--max-speed-ms", type=float, default=6.0)
    parser.add_argument("--max-yaw-rate-rads", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = IcpCheckNode(args.topic)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        samples = list(node.samples)
        node.destroy_node()
        rclpy.shutdown()

    print(f"{BOLD}== ICP Odom Quick Check =={RESET}")
    print(f"topic: {args.topic}")
    print(f"duration_s: {args.duration:.1f}")

    if len(samples) < 2:
        print(f"samples: {len(samples)}")
        print(f"verdict: {RED}FAIL{RESET} no ICP odom samples")
        return 2

    elapsed = max(samples[-1].t - samples[0].t, 1e-6)
    hz = (len(samples) - 1) / elapsed
    age = time.monotonic() - samples[-1].t
    gaps = [b.t - a.t for a, b in zip(samples, samples[1:]) if b.t >= a.t]
    max_gap = max(gaps) if gaps else float("inf")
    xy_jumps = [math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(samples, samples[1:])]
    z_jumps = [abs(b.z - a.z) for a, b in zip(samples, samples[1:])]
    yaw_jumps = [abs(wrap_to_pi(b.yaw - a.yaw)) for a, b in zip(samples, samples[1:])]
    speeds = [jump / max(gap, 1e-6) for jump, gap in zip(xy_jumps, gaps)]
    yaw_rates = [jump / max(gap, 1e-6) for jump, gap in zip(yaw_jumps, gaps)]

    max_xy_jump = max(xy_jumps) if xy_jumps else 0.0
    max_z_jump = max(z_jumps) if z_jumps else 0.0
    max_yaw_jump = max(yaw_jumps) if yaw_jumps else 0.0
    max_speed = max(speeds) if speeds else 0.0
    max_yaw_rate = max(yaw_rates) if yaw_rates else 0.0
    distance = sum(xy_jumps)
    median_hz = 1.0 / statistics.median(gaps) if gaps and statistics.median(gaps) > 0 else 0.0
    frame_ok = bool(samples[-1].frame_id)

    checks = {
        "freq": hz >= args.min_hz,
        "median_freq": median_hz >= args.min_hz,
        "gap": max_gap <= args.max_gap_s,
        "age": age <= args.max_age_s + 0.2,
        "xy_jump": max_xy_jump <= args.max_xy_jump_m,
        "z_jump": max_z_jump <= args.max_z_jump_m,
        "yaw_jump": max_yaw_jump <= args.max_yaw_jump_rad,
        "speed": max_speed <= args.max_speed_ms,
        "yaw_rate": max_yaw_rate <= args.max_yaw_rate_rads,
        "frame": frame_ok,
    }

    print(f"samples: {len(samples)}")
    print(f"hz_mean: {hz:.2f}  {fmt_bool(checks['freq'])}")
    print(f"hz_median: {median_hz:.2f}  {fmt_bool(checks['median_freq'])}")
    print(f"max_gap_s: {max_gap:.3f}  {fmt_bool(checks['gap'])}")
    print(f"last_sample_age_s: {age:.3f}  {fmt_bool(checks['age'])}")
    print(f"max_xy_jump_m: {max_xy_jump:.3f}  {fmt_bool(checks['xy_jump'])}")
    print(f"max_z_jump_m: {max_z_jump:.3f}  {fmt_bool(checks['z_jump'])}")
    print(f"max_yaw_jump_deg: {math.degrees(max_yaw_jump):.1f}  {fmt_bool(checks['yaw_jump'])}")
    print(f"max_speed_ms_from_icp: {max_speed:.2f}  {fmt_bool(checks['speed'])}")
    print(f"max_yaw_rate_rads_from_icp: {max_yaw_rate:.2f}  {fmt_bool(checks['yaw_rate'])}")
    print(f"distance_observed_m: {distance:.2f}")
    print(f"last_frame_id: {samples[-1].frame_id or '<empty>'}")
    print(f"last_child_frame_id: {samples[-1].child_frame_id or '<empty>'}")

    ok = all(checks.values())
    if ok:
        print(f"verdict: {GREEN}OK{RESET} ICP is usable for teach/repeat")
        return 0

    print(f"verdict: {RED}FAIL{RESET} do not start repeat before fixing ICP")
    failed = [name for name, passed in checks.items() if not passed]
    print(f"failed_checks: {', '.join(failed)}")
    if "freq" in failed or "gap" in failed:
        print("hint: mapper may be overloaded or point cloud/TF input is unstable")
    if "xy_jump" in failed or "yaw_jump" in failed:
        print("hint: ICP is jumping; check TF frame, initial alignment, lidar filtering, and map quality")
    if "frame" in failed:
        print("hint: odometry frame_id is empty; route/follower frame consistency is unsafe")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
