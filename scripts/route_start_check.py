#!/usr/bin/env python3
"""Compare a WILN route start pose against the current live ICP pose.

Diagnoses why WILN repeat says "route too far" even when the robot appears
to be at the same physical location.  The root cause is almost always that
each mapper startup initialises the map frame at the robot's current
odom->base_footprint pose, so the map frame origin differs across sessions.

Usage:
    python3 scripts/route_start_check.py --route /path/to/route.ltr [--icp-samples 10]
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# Reuse LTR parser from validate_wiln_route.py
sys.path.insert(0, str(Path(__file__).parent))
from validate_wiln_route import Pose2D, read_ltr  # noqa: E402


GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"

# Distance threshold that mtt_route_manager uses to reject start (metres)
MAX_START_DISTANCE_M = 2.0


def yaw_from_quat(q) -> float:
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


class IcpPoseCollector(Node):
    def __init__(self, topic: str, n_samples: int) -> None:
        super().__init__("mtt_route_start_check")
        self.n_samples = n_samples
        self.samples: list[tuple[float, float, float, str]] = []  # x, y, yaw, frame_id
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Odometry, topic, self._on_odom, qos)

    def _on_odom(self, msg: Odometry) -> None:
        if len(self.samples) >= self.n_samples:
            return
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.samples.append((float(p.x), float(p.y), yaw, str(msg.header.frame_id)))

    def is_done(self) -> bool:
        return len(self.samples) >= self.n_samples


def _median_pose(samples: list[tuple[float, float, float, str]]) -> tuple[float, float, float, str]:
    xs = sorted(s[0] for s in samples)
    ys = sorted(s[1] for s in samples)
    n = len(xs)
    mid = n // 2
    med_x = xs[mid] if n % 2 else (xs[mid - 1] + xs[mid]) / 2
    med_y = ys[mid] if n % 2 else (ys[mid - 1] + ys[mid]) / 2
    # Circular mean for yaw
    sin_sum = sum(math.sin(s[2]) for s in samples)
    cos_sum = sum(math.cos(s[2]) for s in samples)
    med_yaw = math.atan2(sin_sum, cos_sum)
    frame_id = samples[-1][3]
    return med_x, med_y, med_yaw, frame_id


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", type=Path, required=True,
                        help="Path to the WILN .ltr route file")
    parser.add_argument("--icp-topic", default="/mapping/icp_odom")
    parser.add_argument("--icp-samples", type=int, default=10,
                        help="Number of ICP odom samples to collect (default 10)")
    parser.add_argument("--timeout", type=float, default=15.0,
                        help="Timeout waiting for ICP odom (default 15s)")
    args = parser.parse_args()

    # --- Parse route ---
    try:
        frame_id, segments = read_ltr(args.route)
    except (OSError, ValueError) as exc:
        print(f"{RED}ERROR{RESET}: cannot read route: {exc}")
        return 2

    flat = [pose for seg in segments for pose in seg]
    if not flat:
        print(f"{RED}ERROR{RESET}: route has no poses")
        return 2

    route_start: Pose2D = flat[0]
    route_end: Pose2D = flat[-1]
    total_poses = len(flat)
    path_length = sum(
        math.hypot(b.x - a.x, b.y - a.y)
        for a, b in zip(flat, flat[1:])
    )

    print(f"{BOLD}== WILN Route Start Check =={RESET}")
    print(f"route: {args.route}")
    print(f"route_frame_id: {frame_id!r}")
    print(f"route_poses: {total_poses}")
    print(f"route_path_length_m: {path_length:.2f}")
    print(
        f"route_start: x={route_start.x:.3f} y={route_start.y:.3f} "
        f"yaw={math.degrees(route_start.yaw):.1f}deg"
    )
    print()

    # --- Collect live ICP pose ---
    print(f"Collecting {args.icp_samples} ICP odom samples from {args.icp_topic}...")
    rclpy.init()
    node = IcpPoseCollector(args.icp_topic, args.icp_samples)
    start_t = time.monotonic()
    while not node.is_done() and time.monotonic() - start_t < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    samples = list(node.samples)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if len(samples) < 2:
        print(f"{RED}FAIL{RESET}: no ICP odom received from {args.icp_topic} within {args.timeout:.0f}s")
        print("hint: is the mapping stack running? (docker compose up robot)")
        return 2

    robot_x, robot_y, robot_yaw, robot_frame = _median_pose(samples)
    print(
        f"robot_pose: x={robot_x:.3f} y={robot_y:.3f} "
        f"yaw={math.degrees(robot_yaw):.1f}deg  frame={robot_frame!r}"
    )
    print(f"icp_samples_used: {len(samples)}")
    print()

    # --- Compute distance ---
    dx = route_start.x - robot_x
    dy = route_start.y - robot_y
    xy_dist = math.hypot(dx, dy)
    yaw_diff = wrap_pi(route_start.yaw - robot_yaw)
    bearing_to_start = math.degrees(math.atan2(dy, dx))

    print(f"distance_to_route_start_m: {xy_dist:.3f}")
    print(f"yaw_diff_deg: {math.degrees(yaw_diff):.1f}")
    print(f"bearing_to_start_deg: {bearing_to_start:.1f}  (in {robot_frame!r} frame)")
    print()

    # --- Frame ID consistency ---
    frame_ok = True
    if frame_id and robot_frame and frame_id != robot_frame:
        print(
            f"{RED}WARNING{RESET}: route frame_id={frame_id!r} does not match "
            f"current ICP frame={robot_frame!r}. This will cause incorrect pose comparison."
        )
        frame_ok = False

    # --- Diagnosis ---
    within_threshold = xy_dist <= MAX_START_DISTANCE_M
    print(f"max_start_distance_m (route_manager threshold): {MAX_START_DISTANCE_M:.1f}")

    if within_threshold and frame_ok:
        print(f"verdict: {GREEN}OK{RESET} - robot is within start threshold, repeat should work")
        return 0

    print(f"verdict: {RED}FAIL{RESET} - robot is too far from route start")
    print()
    print("Diagnosis:")

    if not frame_ok:
        print(f"  - Frame mismatch: route was saved in {frame_id!r} but current ICP is in {robot_frame!r}")

    if xy_dist > MAX_START_DISTANCE_M:
        print(f"  - XY distance {xy_dist:.2f}m exceeds threshold {MAX_START_DISTANCE_M:.1f}m")
        print()
        print("  Most likely cause:")
        print("    Each mapper startup initialises the MAP frame at the robot's current")
        print("    odom->base_footprint pose.  If the robot was at a different location")
        print("    (or orientation) when mapping started this session vs the teach session,")
        print("    the map frame origin differs even though the robot is physically at the")
        print("    same place.")
        print()
        print("  Possible fixes:")
        print("    1. Re-teach the route in the current mapping session (simplest).")
        print("    2. Load the teach session's saved map before starting repeat:")
        print("       ros2 service call /mapping/load_map std_srvs/srv/Empty")
        print("    3. Use: python3 scripts/align_route_to_pose.py --route <route.ltr>")
        print("       to rigid-align the route start to the current robot pose (SE2 transform).")

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
