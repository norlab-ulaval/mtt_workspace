#!/usr/bin/env python3
"""Rear hazard monitor for the reverse/auto-reverse experiment path.

The existing mtt_front_obstacle_monitor (src/mtt_core/mtt_bringup) only looks
forward -- it provides zero protection while the robot is backing up (normal
C_reverse_arcs segments, or the conductor's auto-reverse boundary recovery).
This node closes that gap with the SAME fail-safe pattern (stop=True if the
cloud is missing/stale or TF fails), but naive and 2D by construction, per
explicit request (2026-08-03): a narrow angular sector behind the trailer,
a narrow elevation slice (mimicking a 2D lidar sweep instead of scanning the
full 3D volume), out to a few meters past the trailer rear.

Geometry: same hitch pivot / trailer-rear constants as trailer_pose_node
V4.0 (the proven reference, see CLAUDE.md "Key geometry constants"), and the
identical yaw_prior = pi - theta convention, theta read live from
/trailer/articulation_angle (the KF-fused, authoritative source -- NOT the
raw hardware topic). The "behind" sector is anchored at the trailer rear
(hitch + 1.90m along u_rear), not the robot origin, so it correctly follows
the trailer as it articulates instead of assuming straight-behind.

This node only ever PUBLISHES a stop-request boolean; like the front monitor
and the conductor's geofence, it is consumed as one more AND-ed disengage
condition and never opens a new command-authority path.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Float64, String
import tf2_ros

# Same base_link-frame constants as trailer_pose_node / mtt_trailer_estimator_node
# (CLAUDE.md "Key geometry constants" -- must stay consistent across all consumers).
HITCH_XYZ = (-1.45, -0.085, 0.35)
TRAILER_REAR_LENGTH_M = 1.90


def _quat_to_rotation(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]
    )


@dataclass
class RearScanResult:
    hit_points: int
    sampled_points: int
    min_clearance_m: float
    theta_rad: float
    theta_stale: bool


class MttRearObstacleMonitor(Node):
    def __init__(self) -> None:
        super().__init__("mtt_rear_obstacle_monitor")

        self.declare_parameter("cloud_topic", "/hesai_lidar/points")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("articulation_angle_topic", "/trailer/articulation_angle")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("cloud_timeout_s", 0.5)
        self.declare_parameter("tf_timeout_s", 0.08)
        self.declare_parameter("theta_timeout_s", 1.0)  # stale encoder -> assume straight-behind, just note it
        # Naive 2D scan: narrow elevation slice in the SENSOR's own frame (mimics a 2D
        # lidar instead of scanning the full 3D volume -- cheap prefilter before the TF
        # transform, so fewer points ever reach the (slower) transform step).
        self.declare_parameter("elevation_half_deg", 3.0)
        # Angular sector + range window, anchored at the TRAILER REAR (hitch + 1.90m),
        # not the robot origin -- the ask was explicitly "prendre en compte la longueur
        # de la remorque". half_angle_deg=20 -> 40 deg total sector, mid-range of the
        # requested 30-50 deg.
        self.declare_parameter("sector_half_angle_deg", 20.0)
        self.declare_parameter("min_range_m", 0.3)  # skip the trailer body itself
        self.declare_parameter("max_range_m", 5.0)  # "quatre ou cinq metres" behind the trailer
        self.declare_parameter("min_hit_points", 5)
        self.declare_parameter("stop_confirm_frames", 2)
        self.declare_parameter("clear_confirm_frames", 5)
        self.declare_parameter("max_points", 60000)
        self.declare_parameter("stop_topic", "/mtt_rear_obstacle/stop_requested")
        self.declare_parameter("clearance_topic", "/mtt_rear_obstacle/clearance_m")
        self.declare_parameter("status_topic", "/mtt_rear_obstacle/status")

        self._cloud_topic = str(self.get_parameter("cloud_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._theta_topic = str(self.get_parameter("articulation_angle_topic").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._cloud_timeout_s = float(self.get_parameter("cloud_timeout_s").value)
        self._tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)
        self._theta_timeout_s = float(self.get_parameter("theta_timeout_s").value)
        self._elevation_half_rad = math.radians(float(self.get_parameter("elevation_half_deg").value))
        self._sector_half_rad = math.radians(float(self.get_parameter("sector_half_angle_deg").value))
        self._min_range_m = float(self.get_parameter("min_range_m").value)
        self._max_range_m = float(self.get_parameter("max_range_m").value)
        self._min_hit_points = int(self.get_parameter("min_hit_points").value)
        self._stop_confirm_frames = int(self.get_parameter("stop_confirm_frames").value)
        self._clear_confirm_frames = int(self.get_parameter("clear_confirm_frames").value)
        self._max_points = max(1000, int(self.get_parameter("max_points").value))

        self._lock = threading.Lock()
        self._latest_cloud: Optional[PointCloud2] = None
        self._latest_cloud_time: Optional[float] = None
        self._theta_rad = 0.0
        self._theta_stamp: Optional[float] = None
        self._stop_hits = 0
        self._clear_hits = self._clear_confirm_frames
        self._stop_active = False

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._cloud_sub = self.create_subscription(PointCloud2, self._cloud_topic, self._on_cloud, qos)
        self._theta_sub = self.create_subscription(Float64, self._theta_topic, self._on_theta, 10)

        self._stop_pub = self.create_publisher(Bool, str(self.get_parameter("stop_topic").value), 10)
        self._clearance_pub = self.create_publisher(Float32, str(self.get_parameter("clearance_topic").value), 10)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        period = 1.0 / max(self._publish_rate_hz, 1.0)
        self.create_timer(period, self._timer)
        self.get_logger().info(
            f"rear obstacle monitor: cloud={self._cloud_topic} target_frame={self._target_frame} "
            f"sector=+/-{math.degrees(self._sector_half_rad):.0f}deg "
            f"elevation=+/-{math.degrees(self._elevation_half_rad):.0f}deg "
            f"range={self._min_range_m:.1f}..{self._max_range_m:.1f}m behind trailer rear "
            f"(trailer_rear_length={TRAILER_REAR_LENGTH_M:.2f}m)"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cloud(self, msg: PointCloud2) -> None:
        with self._lock:
            self._latest_cloud = msg
            self._latest_cloud_time = self._now_s()

    def _on_theta(self, msg: Float64) -> None:
        self._theta_rad = float(msg.data)
        self._theta_stamp = self._now_s()

    def _transform_for_cloud(self, cloud: PointCloud2):
        source_frame = cloud.header.frame_id.strip().lstrip("/")
        target_frame = self._target_frame.strip().lstrip("/")
        if not source_frame:
            raise RuntimeError("cloud header frame_id is empty")
        if source_frame == target_frame:
            return None
        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, cloud.header.stamp, timeout=Duration(seconds=self._tf_timeout_s)
            )
        except Exception:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=self._tf_timeout_s)
            )

    def _analyze(self, cloud: PointCloud2) -> RearScanResult:
        pts = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
        raw_n = pts.shape[0]
        if raw_n > self._max_points:
            stride = max(1, raw_n // self._max_points)
            pts = pts[::stride]
        sampled_n = pts.shape[0]

        if sampled_n == 0:
            return RearScanResult(0, 0, math.inf, self._theta_rad, self._theta_is_stale())

        x, y, z = pts[:, 0].astype(np.float64), pts[:, 1].astype(np.float64), pts[:, 2].astype(np.float64)

        # Stage 1 (cheap, in the sensor's own frame): narrow elevation slice, mimics a
        # 2D lidar instead of scanning the full 3D volume -- fewer points reach the TF
        # transform below.
        planar_r = np.sqrt(x * x + y * y)
        elev = np.arctan2(z, np.maximum(planar_r, 1e-6))
        keep = np.abs(elev) <= self._elevation_half_rad
        x, y, z = x[keep], y[keep], z[keep]
        if x.size == 0:
            return RearScanResult(0, sampled_n, math.inf, self._theta_rad, self._theta_is_stale())

        # Stage 2: transform survivors into base_link.
        transform = self._transform_for_cloud(cloud)
        if transform is not None:
            t = transform.transform.translation
            q = transform.transform.rotation
            rot = _quat_to_rotation(q.x, q.y, q.z, q.w)
            pts_b = np.stack([x, y, z], axis=1) @ rot.T + np.array([t.x, t.y, t.z])
            x, y, z = pts_b[:, 0], pts_b[:, 1], pts_b[:, 2]

        # Stage 3: angular sector + range, anchored at the trailer rear (follows
        # articulation via theta -> yaw_prior = pi - theta, same convention as
        # trailer_pose_node.cpp:334).
        theta = self._theta_rad
        yaw_prior = math.pi - theta
        rear_x = HITCH_XYZ[0] + TRAILER_REAR_LENGTH_M * math.cos(yaw_prior)
        rear_y = HITCH_XYZ[1] + TRAILER_REAR_LENGTH_M * math.sin(yaw_prior)
        dx = x - rear_x
        dy = y - rear_y
        along = dx * math.cos(yaw_prior) + dy * math.sin(yaw_prior)
        across = -dx * math.sin(yaw_prior) + dy * math.cos(yaw_prior)
        rng = np.sqrt(along * along + across * across)
        angle = np.arctan2(across, np.maximum(along, 1e-6))

        in_sector = (
            (along > 0.0)
            & (rng >= self._min_range_m)
            & (rng <= self._max_range_m)
            & (np.abs(angle) <= self._sector_half_rad)
        )
        hit_rng = rng[in_sector]
        min_clearance = float(hit_rng.min()) if hit_rng.size else math.inf

        return RearScanResult(
            hit_points=int(in_sector.sum()),
            sampled_points=sampled_n,
            min_clearance_m=min_clearance,
            theta_rad=theta,
            theta_stale=self._theta_is_stale(),
        )

    def _theta_is_stale(self) -> bool:
        return self._theta_stamp is None or (self._now_s() - self._theta_stamp) > self._theta_timeout_s

    def _set_stop_state(self, stop_detected: bool) -> None:
        if stop_detected:
            self._stop_hits += 1
            self._clear_hits = 0
        else:
            self._clear_hits += 1
            self._stop_hits = 0
        if self._stop_hits >= self._stop_confirm_frames:
            self._stop_active = True
        if self._clear_hits >= self._clear_confirm_frames:
            self._stop_active = False

    def _publish(self, stop: bool, clearance: float, status: str) -> None:
        stop_msg = Bool()
        stop_msg.data = stop
        self._stop_pub.publish(stop_msg)
        clearance_msg = Float32()
        clearance_msg.data = float(clearance) if math.isfinite(clearance) else -1.0
        self._clearance_pub.publish(clearance_msg)
        status_msg = String()
        status_msg.data = status
        self._status_pub.publish(status_msg)

    def _timer(self) -> None:
        with self._lock:
            cloud = self._latest_cloud
            cloud_time = self._latest_cloud_time
        if cloud is None or cloud_time is None:
            self._stop_active = True
            self._publish(True, math.inf, "cloud missing: rear check has no data yet")
            return
        age = self._now_s() - cloud_time
        if age > self._cloud_timeout_s:
            self._stop_active = True
            self._publish(True, math.inf, f"cloud stale: age={age:.2f}s")
            return

        try:
            result = self._analyze(cloud)
        except Exception as exc:
            self._stop_active = True
            self._publish(True, math.inf, f"tf/filter unavailable: {exc}")
            return

        stop_detected = result.hit_points >= self._min_hit_points
        self._set_stop_state(stop_detected)
        clearance_text = f"{result.min_clearance_m:.2f}m" if math.isfinite(result.min_clearance_m) else "clear"
        theta_note = " theta_stale=assuming_straight" if result.theta_stale else ""
        status = (
            f"stop={self._stop_active} clearance={clearance_text} "
            f"hits={result.hit_points}/{self._min_hit_points} sampled={result.sampled_points} "
            f"theta={math.degrees(result.theta_rad):+.1f}deg{theta_note}"
        )
        self._publish(self._stop_active, result.min_clearance_m, status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MttRearObstacleMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
