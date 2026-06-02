#!/usr/bin/env python3
"""Directed diagnostic for ICP trajectory snaps back toward the initial pose."""

from __future__ import annotations

import argparse
import math
import time
from collections import Counter
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


@dataclass
class Pose2:
    x: float
    y: float
    yaw: float


def dist(a: Pose2, b: Pose2) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def pose_from_odom(msg: Odometry) -> Pose2:
    p = msg.pose.pose.position
    return Pose2(float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation))


def pose_from_tf(tf: TransformStamped) -> Pose2:
    t = tf.transform.translation
    return Pose2(float(t.x), float(t.y), yaw_from_quat(tf.transform.rotation))


def compose(a: Pose2, b: Pose2) -> Pose2:
    ca = math.cos(a.yaw)
    sa = math.sin(a.yaw)
    return Pose2(
        a.x + ca * b.x - sa * b.y,
        a.y + sa * b.x + ca * b.y,
        wrap_pi(a.yaw + b.yaw),
    )


def fmt_pose(p: Optional[Pose2]) -> str:
    if p is None:
        return "unavailable"
    return f"({p.x:+.3f},{p.y:+.3f},{math.degrees(p.yaw):+.1f}deg)"


class OriginSnapDiag(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("origin_snap_diag")
        self.args = args
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Odometry, args.icp_topic, self._on_icp, qos)

        self.initial_icp: Optional[Pose2] = None
        self.last_icp: Optional[Pose2] = None
        self.last_odom_tf: Optional[Pose2] = None
        self.last_map_odom_tf: Optional[Pose2] = None
        self.last_predicted: Optional[Pose2] = None
        self.static_checked = False
        self.samples = 0
        self.jumps = 0
        self.origin_snaps = 0
        self.sources: Counter[str] = Counter()

        print("=" * 88)
        print("DIRECTED ORIGIN-SNAP DIAGNOSTIC")
        print(f"icp_topic={args.icp_topic}")
        print(f"frames: map={args.map_frame} odom={args.odom_frame} base={args.base_frame}")
        print(f"jump_thresh={args.jump_thresh:.2f}m origin_drop={args.origin_drop:.2f}m")
        print("=" * 88)

    def _lookup_pose(self, target: str, source: str, stamp_msg) -> Optional[Pose2]:
        try:
            tf = self.tf_buffer.lookup_transform(
                target,
                source,
                Time.from_msg(stamp_msg),
                timeout=Duration(seconds=self.args.tf_timeout),
            )
            return pose_from_tf(tf)
        except TransformException:
            return None

    def _check_static_tf(self) -> None:
        if self.static_checked:
            return
        try:
            base_hesai = pose_from_tf(
                self.tf_buffer.lookup_transform(
                    self.args.base_frame,
                    self.args.sensor_frame,
                    Time(),
                    timeout=Duration(seconds=0.25),
                )
            )
            base_link_hesai = pose_from_tf(
                self.tf_buffer.lookup_transform(
                    "base_link",
                    self.args.sensor_frame,
                    Time(),
                    timeout=Duration(seconds=0.25),
                )
            )
            print(
                "[STATIC TF] "
                f"{self.args.base_frame}->{self.args.sensor_frame} {fmt_pose(base_hesai)} | "
                f"base_link->{self.args.sensor_frame} {fmt_pose(base_link_hesai)}"
            )
            print("            Expected yaw is about +90deg for base/base_link to hesai_lidar.")
            self.static_checked = True
        except TransformException as exc:
            print(f"[STATIC TF] unavailable yet: {exc}")

    def _classify(
        self,
        icp: Pose2,
        odom_tf: Optional[Pose2],
        map_odom_tf: Optional[Pose2],
        predicted: Optional[Pose2],
    ) -> str:
        raw_step = dist(odom_tf, self.last_odom_tf) if odom_tf and self.last_odom_tf else 0.0
        map_odom_step = (
            dist(map_odom_tf, self.last_map_odom_tf)
            if map_odom_tf and self.last_map_odom_tf
            else 0.0
        )
        pred_step = dist(predicted, self.last_predicted) if predicted and self.last_predicted else 0.0
        icp_pred_delta = dist(icp, predicted) if predicted else float("inf")

        if raw_step > self.args.jump_thresh:
            return "odom_to_base_tf"
        if icp_pred_delta > 0.25:
            return "published_pose_not_equal_tf_chain"
        if map_odom_step > 0.25 and raw_step < 0.25:
            return "map_to_odom_icp_correction"
        if pred_step > self.args.jump_thresh and raw_step < 0.25:
            return "map_to_odom_or_prediction"
        return "unknown_or_registration"

    def _on_icp(self, msg: Odometry) -> None:
        self._check_static_tf()
        self.samples += 1
        icp = pose_from_odom(msg)
        if self.initial_icp is None:
            self.initial_icp = icp

        odom_tf = self._lookup_pose(self.args.odom_frame, self.args.base_frame, msg.header.stamp)
        map_odom_tf = self._lookup_pose(self.args.map_frame, self.args.odom_frame, msg.header.stamp)
        predicted = compose(map_odom_tf, odom_tf) if odom_tf and map_odom_tf else None

        jump = self.last_icp is not None and dist(icp, self.last_icp) > self.args.jump_thresh
        origin_snap = False
        if self.initial_icp is not None and self.last_icp is not None:
            last_d0 = dist(self.last_icp, self.initial_icp)
            curr_d0 = dist(icp, self.initial_icp)
            origin_snap = (
                last_d0 > self.args.origin_min_dist
                and curr_d0 + self.args.origin_drop < last_d0
            )

        if jump or origin_snap:
            self.jumps += int(jump)
            self.origin_snaps += int(origin_snap)
            source = self._classify(icp, odom_tf, map_odom_tf, predicted)
            self.sources[source] += 1

            raw_step = dist(odom_tf, self.last_odom_tf) if odom_tf and self.last_odom_tf else float("nan")
            map_odom_step = (
                dist(map_odom_tf, self.last_map_odom_tf)
                if map_odom_tf and self.last_map_odom_tf
                else float("nan")
            )
            pred_step = (
                dist(predicted, self.last_predicted)
                if predicted and self.last_predicted
                else float("nan")
            )
            icp_pred_delta = dist(icp, predicted) if predicted else float("nan")
            d0 = dist(icp, self.initial_icp) if self.initial_icp else float("nan")
            last_d0 = dist(self.last_icp, self.initial_icp) if self.last_icp and self.initial_icp else float("nan")

            print("\n[ANOMALY]")
            print(f"  stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
            print(f"  flags: jump={jump} origin_snap={origin_snap} likely_source={source}")
            print(f"  icp_pose       {fmt_pose(icp)} step={dist(icp, self.last_icp):.3f}m d_initial={d0:.3f}m last_d_initial={last_d0:.3f}m")
            print(f"  odom->base     {fmt_pose(odom_tf)} step={raw_step:.3f}m")
            print(f"  map->odom      {fmt_pose(map_odom_tf)} step={map_odom_step:.3f}m")
            print(f"  tf_predicted   {fmt_pose(predicted)} step={pred_step:.3f}m icp_minus_pred={icp_pred_delta:.3f}m")

        self.last_icp = icp
        self.last_odom_tf = odom_tf
        self.last_map_odom_tf = map_odom_tf
        self.last_predicted = predicted

    def print_summary(self) -> None:
        print("\n" + "=" * 88)
        print("SUMMARY")
        print(f"samples={self.samples} jumps={self.jumps} origin_snaps={self.origin_snaps}")
        for source, count in self.sources.most_common():
            print(f"  {source}: {count}")
        print("=" * 88)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--icp-topic", default="/mapping/icp_odom")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--sensor-frame", default="hesai_lidar")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--jump-thresh", type=float, default=0.50)
    parser.add_argument("--origin-min-dist", type=float, default=0.75)
    parser.add_argument("--origin-drop", type=float, default=0.40)
    parser.add_argument("--tf-timeout", type=float, default=0.05)
    args = parser.parse_args()

    rclpy.init()
    node = OriginSnapDiag(args)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.print_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
