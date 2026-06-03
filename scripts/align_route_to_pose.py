#!/usr/bin/env python3
"""SE(2) rigid alignment of a WILN .ltr route to the current robot pose.

Computes the 2D rigid transform (translation + rotation) that maps the
route's first pose to the current ICP odom pose, then applies it to all
route poses.  Writes the aligned route to <route>_aligned.ltr (or --output)
and keeps a backup at <route>.bak.

Use this when WILN repeat says "route too far" because the map frame origin
differs between the teach session and the current repeat session.

Usage:
    python3 scripts/align_route_to_pose.py --route /path/to/route.ltr
    python3 scripts/align_route_to_pose.py --route route.ltr --output route_aligned.ltr
    python3 scripts/align_route_to_pose.py --route route.ltr --inplace  # overwrite with backup
"""

from __future__ import annotations

import argparse
import math
import shutil
import sys
import time
from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

sys.path.insert(0, str(Path(__file__).parent))
from validate_wiln_route import Pose2D, read_ltr  # noqa: E402


GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"


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


def pose_to_quat(yaw: float) -> tuple[float, float, float, float]:
    """Convert yaw to quaternion (qx, qy, qz, qw) for z-rotation only."""
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


# ---------------------------------------------------------------------------
# SE(2) transform helpers
# ---------------------------------------------------------------------------

def se2_from_route_start_to_robot(
    route_start: Pose2D, robot_x: float, robot_y: float, robot_yaw: float
) -> tuple[float, float, float]:
    """Compute (dx, dy, dyaw) such that T * route_start = robot_pose.

    The transform T rotates the route by dyaw (in-plane) and translates by (dx, dy),
    aligning the route start to the current robot pose.
    """
    dyaw = wrap_pi(robot_yaw - route_start.yaw)
    # After rotation, the route start is at:
    cos_d = math.cos(dyaw)
    sin_d = math.sin(dyaw)
    rotated_x = cos_d * route_start.x - sin_d * route_start.y
    rotated_y = sin_d * route_start.x + cos_d * route_start.y
    dx = robot_x - rotated_x
    dy = robot_y - rotated_y
    return dx, dy, dyaw


def apply_se2(pose: Pose2D, dx: float, dy: float, dyaw: float) -> Pose2D:
    cos_d = math.cos(dyaw)
    sin_d = math.sin(dyaw)
    rx = cos_d * pose.x - sin_d * pose.y + dx
    ry = sin_d * pose.x + cos_d * pose.y + dy
    return Pose2D(x=rx, y=ry, z=pose.z, yaw=wrap_pi(pose.yaw + dyaw))


# ---------------------------------------------------------------------------
# LTR writer -- preserves structure including "changing direction" markers
# ---------------------------------------------------------------------------

def write_ltr(
    path: Path,
    frame_id: str,
    segments: list[list[Pose2D]],
    source_lines_header: list[str],
) -> None:
    """Write a .ltr file with aligned poses, preserving the original header."""
    with path.open("w", encoding="utf-8") as fh:
        for line in source_lines_header:
            fh.write(line)
        for i, segment in enumerate(segments):
            if i > 0:
                fh.write("changing direction\n")
            for pose in segment:
                qx, qy, qz, qw = pose_to_quat(pose.yaw)
                fh.write(f"{pose.x:.6f},{pose.y:.6f},{pose.z:.6f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}\n")


def read_ltr_raw(path: Path) -> tuple[str, list[list[Pose2D]], list[str]]:
    """Read .ltr and also return the raw header lines (before the ##### separator)."""
    frame_id, segments = read_ltr(path)
    header_lines: list[str] = []
    in_trajectory = False
    with path.open("r", encoding="utf-8") as fh:
        for raw_line in fh:
            if raw_line.strip().startswith("#############################"):
                header_lines.append(raw_line)
                in_trajectory = True
                break
            header_lines.append(raw_line)
    if not in_trajectory:
        header_lines.append("#############################\n")
    # Ensure frame_id line is present
    has_frame_line = any("frame_id" in l for l in header_lines)
    if not has_frame_line and frame_id:
        header_lines.append(f"frame_id: {frame_id}\n")
    return frame_id, segments, header_lines


# ---------------------------------------------------------------------------
# ICP pose collector
# ---------------------------------------------------------------------------

class PoseCollector(Node):
    def __init__(self, topic: str, n: int) -> None:
        super().__init__("mtt_align_route_pose")
        self.n = n
        self.samples: list[tuple[float, float, float, str]] = []
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Odometry, topic, self._cb, qos)

    def _cb(self, msg: Odometry) -> None:
        if len(self.samples) >= self.n:
            return
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.samples.append((float(p.x), float(p.y), yaw, str(msg.header.frame_id)))

    def done(self) -> bool:
        return len(self.samples) >= self.n


def _median_pose(samples: list) -> tuple[float, float, float, str]:
    xs = sorted(s[0] for s in samples)
    ys = sorted(s[1] for s in samples)
    n = len(xs)
    mid = n // 2
    mx = xs[mid] if n % 2 else (xs[mid - 1] + xs[mid]) / 2
    my = ys[mid] if n % 2 else (ys[mid - 1] + ys[mid]) / 2
    sin_sum = sum(math.sin(s[2]) for s in samples)
    cos_sum = sum(math.cos(s[2]) for s in samples)
    my_yaw = math.atan2(sin_sum, cos_sum)
    return mx, my, my_yaw, samples[-1][3]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None,
                        help="Output .ltr path (default: <route>_aligned.ltr)")
    parser.add_argument("--inplace", action="store_true",
                        help="Overwrite the route file (a .bak backup is created)")
    parser.add_argument("--icp-topic", default="/mapping/icp_odom")
    parser.add_argument("--icp-samples", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--dry-run", action="store_true",
                        help="Compute and print the transform but do not write the file")
    args = parser.parse_args()

    # --- Read route ---
    try:
        frame_id, segments, header_lines = read_ltr_raw(args.route)
    except (OSError, ValueError) as exc:
        print(f"{RED}ERROR{RESET}: cannot read route: {exc}")
        return 2

    flat = [p for seg in segments for p in seg]
    if not flat:
        print(f"{RED}ERROR{RESET}: route has no poses")
        return 2

    route_start = flat[0]
    n_poses = len(flat)
    path_len = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(flat, flat[1:]))

    print(f"{BOLD}== Align Route to Current Pose =={RESET}")
    print(f"route: {args.route}")
    print(f"route_frame_id: {frame_id!r}")
    print(f"route_poses: {n_poses}  path_length: {path_len:.2f}m")
    print(
        f"route_start: x={route_start.x:.4f} y={route_start.y:.4f} "
        f"yaw={math.degrees(route_start.yaw):.2f}deg"
    )

    # --- Collect live pose ---
    print(f"\nCollecting {args.icp_samples} ICP samples from {args.icp_topic}...")
    rclpy.init()
    collector = PoseCollector(args.icp_topic, args.icp_samples)
    t0 = time.monotonic()
    while not collector.done() and time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(collector, timeout_sec=0.1)
    samples = list(collector.samples)
    collector.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if len(samples) < 2:
        print(f"{RED}FAIL{RESET}: no ICP odom received from {args.icp_topic}")
        return 2

    robot_x, robot_y, robot_yaw, robot_frame = _median_pose(samples)
    print(
        f"robot_pose: x={robot_x:.4f} y={robot_y:.4f} "
        f"yaw={math.degrees(robot_yaw):.2f}deg  frame={robot_frame!r}"
    )

    # --- Warn on frame mismatch ---
    if frame_id and robot_frame and frame_id != robot_frame:
        print(
            f"\n{YELLOW}WARNING{RESET}: route frame_id={frame_id!r} != ICP frame={robot_frame!r}."
        )

    # --- Compute SE(2) transform ---
    dx, dy, dyaw = se2_from_route_start_to_robot(route_start, robot_x, robot_y, robot_yaw)
    print(f"\nSE(2) transform: dx={dx:.4f}m  dy={dy:.4f}m  dyaw={math.degrees(dyaw):.2f}deg")

    pre_dist = math.hypot(route_start.x - robot_x, route_start.y - robot_y)
    print(f"distance before alignment: {pre_dist:.3f}m")

    # Verify alignment
    aligned_start = apply_se2(route_start, dx, dy, dyaw)
    post_dist = math.hypot(aligned_start.x - robot_x, aligned_start.y - robot_y)
    print(f"distance after alignment: {post_dist:.4f}m  (should be ~0)")

    if post_dist > 0.01:
        print(f"{RED}ERROR{RESET}: SE(2) alignment residual too large ({post_dist:.4f}m). Bug!")
        return 2

    if args.dry_run:
        print(f"\n{YELLOW}Dry run -- no file written.{RESET}")
        return 0

    # --- Apply transform to all segments ---
    aligned_segments = [
        [apply_se2(p, dx, dy, dyaw) for p in seg]
        for seg in segments
    ]

    # --- Determine output path ---
    if args.output:
        out_path = args.output
    elif args.inplace:
        bak_path = args.route.with_suffix(".ltr.bak")
        shutil.copy2(args.route, bak_path)
        print(f"\nBackup written to {bak_path}")
        out_path = args.route
    else:
        out_path = args.route.with_stem(args.route.stem + "_aligned")

    write_ltr(out_path, frame_id, aligned_segments, header_lines)
    print(f"\n{GREEN}Aligned route written to {out_path}{RESET}")
    print(f"Poses: {n_poses}  Path length: {path_len:.2f}m (unchanged)")
    print("\nNext steps:")
    if not args.inplace and not args.output:
        print(f"  Load the aligned route: wiln_route_manager load {out_path}")
    print("  Run route_start_check.py to verify the alignment before repeat.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
