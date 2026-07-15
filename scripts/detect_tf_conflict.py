#!/usr/bin/env python3
"""Detect multiple publishers on the odom → base_footprint TF edge.

Run WHILE a bag replay (--profile mapping) is active to check whether more than one
service is broadcasting the odom → base_footprint transform simultaneously.

Usage (from workspace root, inside the Docker dev shell or any ROS 2 Jazzy env):

    # Inside the container:
    docker compose run --rm bash -c \
        'python3 /workspace/scripts/detect_tf_conflict.py'

    # Or on host if ROS 2 Jazzy is sourced:
    python3 scripts/detect_tf_conflict.py

    # Custom frames / alert threshold:
    python3 scripts/detect_tf_conflict.py \
        --parent odom --child base_footprint \
        --yaw-alert 25.0 --duration 60

What it does:
1. Counts publishers on /tf (via ros2 topic info).  >1 publisher on /tf is expected
   (bag player + runtime_odometry), but the composition tells you which nodes are active.
2. Subscribes to /tf and tracks every transform for the odom→base_footprint edge.
   Prints yaw-over-time and raises an alert when consecutive yaw jumps exceed the threshold.
   A jump ≥ 85° (configurable) is the 90° bug signature.

Interpretation:
  - 0 alerts, smooth yaw ramp → single healthy publisher, no TF conflict.
  - Alert with jump ~90° → two publishers interleaving; ZED convention (camera-forward ≠ robot-forward).
  - Alert at random values → encoder spike propagated through odom publisher.
"""

import argparse
import math
import subprocess
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def wrap_to_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q) -> float:
    """Quaternion → yaw (ZYX convention)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TfConflictDetector(Node):
    def __init__(self, parent: str, child: str, yaw_alert_deg: float, duration: float,
                 settle: float = 5.0):
        super().__init__("tf_conflict_detector")
        self.parent = parent
        self.child = child
        self.yaw_alert_rad = math.radians(yaw_alert_deg)
        self.duration = duration
        self.settle = settle

        self._last_yaw: Optional[float] = None
        self._last_stamp: Optional[float] = None
        self._first_stamp: Optional[float] = None
        self._alerts = 0
        self._settle_ignored = 0
        self._count = 0
        self._start = time.monotonic()

        self._sub = self.create_subscription(TFMessage, "/tf", self._tf_cb, 100)
        self.get_logger().info(
            f"Monitoring TF: {parent} → {child} | "
            f"yaw-alert threshold: {yaw_alert_deg:.1f}° | "
            f"duration: {duration:.0f}s"
        )

    def _tf_cb(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.header.frame_id == self.parent and t.child_frame_id == self.child:
                stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                yaw = yaw_from_quat(t.transform.rotation)
                yaw_deg = math.degrees(yaw)

                if self._first_stamp is None:
                    self._first_stamp = stamp
                if self._last_yaw is not None:
                    delta_deg = abs(math.degrees(wrap_to_pi(yaw - self._last_yaw)))
                    dt = stamp - self._last_stamp if self._last_stamp is not None else 0.0

                    if delta_deg >= math.degrees(self.yaw_alert_rad):
                        # One-time heading initialisation (e.g. runtime_odometry
                        # aligning to the first IMU sample) is not a publisher
                        # conflict — a real conflict flip-flops continuously.
                        if stamp - self._first_stamp < self.settle:
                            self._settle_ignored += 1
                            self.get_logger().warning(
                                f"Yaw jump {delta_deg:.1f}° within settle window "
                                f"({stamp - self._first_stamp:.1f}s < {self.settle:.1f}s) — "
                                "ignored (heading initialisation)."
                            )
                        else:
                            self._alerts += 1
                            self.get_logger().error(
                                f"[ALERT #{self._alerts}] YAW JUMP on {self.parent}→{self.child}: "
                                f"{delta_deg:.1f}° in {dt:.3f}s | "
                                f"yaw: {math.degrees(self._last_yaw):.1f}° → {yaw_deg:.1f}° | "
                                f"stamp={stamp:.3f}"
                            )
                    else:
                        self.get_logger().debug(
                            f"  TF {self.parent}→{self.child}: "
                            f"yaw={yaw_deg:.2f}° Δ={delta_deg:.2f}° dt={dt:.3f}s"
                        )
                else:
                    self.get_logger().info(
                        f"  First {self.parent}→{self.child}: yaw={yaw_deg:.2f}° stamp={stamp:.3f}"
                    )

                self._last_yaw = yaw
                self._last_stamp = stamp
                self._count += 1

    def done(self) -> bool:
        return (time.monotonic() - self._start) >= self.duration

    def summary(self) -> None:
        print("\n" + "=" * 60)
        print(f"TF Conflict Detector — Summary ({self.parent} → {self.child})")
        print("=" * 60)
        print(f"  Transforms received : {self._count}")
        print(f"  Yaw jump alerts     : {self._alerts}")
        if self._alerts == 0:
            print("  Result: ✓ No yaw jumps detected. Single healthy publisher likely.")
            print("          (Also verify with: ros2 topic info /tf --verbose)")
        else:
            print(
                f"  Result: ✗ {self._alerts} yaw jump(s) ≥ {math.degrees(self.yaw_alert_rad):.1f}° "
                "detected."
            )
            print("          → Multiple publishers on this TF edge confirmed.")
            print("          → Run: ros2 topic info /tf --verbose  to list publisher nodes.")
        print("=" * 60)


def show_tf_publishers() -> None:
    """Print publisher list for /tf using ros2 CLI (best-effort)."""
    print("\n── /tf publishers (ros2 topic info) ──")
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", "/tf", "--verbose"],
            capture_output=True, text=True, timeout=5
        )
        lines = result.stdout.splitlines()
        # Print only publisher section
        in_pub = False
        for line in lines:
            if "Publisher" in line:
                in_pub = True
            if in_pub:
                print(line)
                if line.strip() == "" and in_pub:
                    break
    except Exception as e:
        print(f"  (ros2 topic info failed: {e})")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--parent", default="odom",
                        help="Parent TF frame (default: odom)")
    parser.add_argument("--child", default="base_footprint",
                        help="Child TF frame (default: base_footprint)")
    parser.add_argument("--yaw-alert", type=float, default=25.0,
                        help="Alert threshold for yaw jump in degrees (default: 25.0)")
    parser.add_argument("--duration", type=float, default=120.0,
                        help="How long to monitor in seconds (default: 120)")
    parser.add_argument("--settle", type=float, default=5.0,
                        help="Grace window (s, bag time from first transform) during "
                             "which yaw jumps are treated as heading initialisation "
                             "and ignored (default: 5.0)")
    parser.add_argument("--min-alerts", type=int, default=3,
                        help="Fail only when at least this many post-settle jumps are "
                             "seen — a real dual-publisher conflict alternates "
                             "continuously (default: 3)")
    args = parser.parse_args()

    print(f"\n{'='*60}")
    print(f"TF Conflict Detector")
    print(f"  Edge      : {args.parent} → {args.child}")
    print(f"  Alert at  : {args.yaw_alert:.1f}° yaw jump per consecutive transform")
    print(f"  Duration  : {args.duration:.0f}s")
    print(f"{'='*60}\n")

    show_tf_publishers()

    rclpy.init()
    node = TfConflictDetector(args.parent, args.child, args.yaw_alert, args.duration,
                              settle=args.settle)

    try:
        while not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[Interrupted by user]")

    node.summary()
    exit_code = 0 if node._alerts < args.min_alerts else 1
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
