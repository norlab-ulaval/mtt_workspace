#!/usr/bin/env python3
"""Bridges Isaac ROS Visual SLAM's SLAM frame into the main TF tree without
conflicting with either the URDF or Isaac VSLAM's own TF publishes.

The constraint that shapes this whole script: a TF frame can have exactly
ONE parent-broadcaster, static or dynamic, full stop. Two candidate designs
were tried and empirically failed for that exact reason before this one:

1. base_frame: zed_camera_link + publish_odom_to_base_tf: true -- conflicts
   with the URDF's static hesai_lidar -> zed_camera_link (isaac_vslam would
   ALSO broadcast odom -> zed_camera_link). Confirmed on the robot
   2026-07-26: "Tf has two or more unconnected trees".
2. base_frame: isaac_base_link (a frame nothing else names) + a static
   bridge publishing hesai_lidar -> isaac_base_link + publish_odom_to_base_tf:
   true -- looks unclaimed, but isaac_vslam publishes isaac_odom ->
   isaac_base_link on the SAME frame, so isaac_base_link now has two
   parent-broadcasters (the static bridge AND isaac_vslam). Confirmed via
   `ros2 run tf2_tools view_frames` on the robot 2026-07-26: isaac_base_link
   only ever shows ONE parent (isaac_odom -- the dynamic one wins), and
   hesai_lidar/isaac_map stayed in separate trees. Any base_frame isaac_vslam
   is told to use, with publish_odom_to_base_tf true, gets claimed by
   isaac_vslam -- there is no name that dodges this.

Working design: leave isaac_vslam's TF output alone (base_frame:
zed_camera_link, publish_odom_to_base_tf: FALSE -- isaac_vslam then never
broadcasts anything with zed_camera_link as a child, so the URDF keeps sole
ownership of hesai_lidar -> zed_camera_link). Isaac_vslam's tracked pose
(isaac_odom -> zed_camera_link) is instead read from the /isaac/vslam/odometry
TOPIC, not TF. This script combines three independently-obtained rigid
transforms with plain matrix algebra (no tf2 multi-hop composition, which
proved unreliable when mixing a real TransformListener with a
manually-injected sample on the same buffer -- see git history for the
abandoned attempt) to get hesai_lidar -> isaac_map, and broadcasts that as
ONE new dynamic edge:

    T(hesai_lidar->isaac_map)
      = T(hesai_lidar->zed_camera_link)         # URDF static, TF lookup
      . inv(T(isaac_odom->zed_camera_link))     # /isaac/vslam/odometry topic
      . inv(T(isaac_map->isaac_odom))           # isaac_vslam's own TF publish

isaac_map is isaac_vslam's tree ROOT -- nothing else ever publishes a parent
for it, so this broadcast is the only parent-claimant isaac_map ever gets.
No conflict in either direction.
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import TransformBroadcaster

STATIC_PARENT_FRAME = "hesai_lidar"
STATIC_CHILD_FRAME = "zed_camera_link"
SLAM_PARENT_FRAME = "isaac_map"
SLAM_CHILD_FRAME = "isaac_odom"
OUTPUT_PARENT_FRAME = "hesai_lidar"
OUTPUT_CHILD_FRAME = "isaac_map"
ODOMETRY_TOPIC = "/isaac/vslam/odometry"
STATIC_LOOKUP_TIMEOUT_S = 30.0
PUBLISH_PERIOD_S = 0.1  # 10 Hz -- plenty for an ICP prior, cheap on CPU/GPU


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0.0 else 0.0
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def _rot_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def _to_matrix(tx: float, ty: float, tz: float, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = _quat_to_rot(qx, qy, qz, qw)
    m[:3, 3] = (tx, ty, tz)
    return m


def _invert(m: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    r_t = m[:3, :3].T
    out[:3, :3] = r_t
    out[:3, 3] = -r_t @ m[:3, 3]
    return out


class IsaacVslamFrameBridge(Node):
    def __init__(self) -> None:
        super().__init__("isaac_vslam_frame_bridge")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._t_hesai_zed: np.ndarray | None = None
        self._last_publish_s = 0.0
        self.get_logger().info(
            f"Waiting for static {STATIC_PARENT_FRAME} -> {STATIC_CHILD_FRAME}..."
        )
        self.create_subscription(Odometry, ODOMETRY_TOPIC, self._on_odometry, 10)

    def _ensure_static_piece(self) -> bool:
        if self._t_hesai_zed is not None:
            return True
        try:
            tr = self.tf_buffer.lookup_transform(
                STATIC_PARENT_FRAME, STATIC_CHILD_FRAME, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False
        t, q = tr.transform.translation, tr.transform.rotation
        self._t_hesai_zed = _to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f"Got static {STATIC_PARENT_FRAME} -> {STATIC_CHILD_FRAME}, bridging live."
        )
        return True

    def _on_odometry(self, msg: Odometry) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s - self._last_publish_s < PUBLISH_PERIOD_S:
            return
        if not self._ensure_static_piece():
            return
        try:
            slam_tr = self.tf_buffer.lookup_transform(
                SLAM_PARENT_FRAME, SLAM_CHILD_FRAME, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f"No {SLAM_PARENT_FRAME} -> {SLAM_CHILD_FRAME} yet: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t_isaacodom_zed = _to_matrix(p.x, p.y, p.z, o.x, o.y, o.z, o.w)

        st = slam_tr.transform.translation
        sq = slam_tr.transform.rotation
        t_isaacmap_isaacodom = _to_matrix(st.x, st.y, st.z, sq.x, sq.y, sq.z, sq.w)

        t_hesai_isaacmap = (
            self._t_hesai_zed @ _invert(t_isaacodom_zed) @ _invert(t_isaacmap_isaacodom)
        )
        qx, qy, qz, qw = _rot_to_quat(t_hesai_isaacmap[:3, :3])

        out = TransformStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = OUTPUT_PARENT_FRAME
        out.child_frame_id = OUTPUT_CHILD_FRAME
        out.transform.translation.x = float(t_hesai_isaacmap[0, 3])
        out.transform.translation.y = float(t_hesai_isaacmap[1, 3])
        out.transform.translation.z = float(t_hesai_isaacmap[2, 3])
        out.transform.rotation.x = qx
        out.transform.rotation.y = qy
        out.transform.rotation.z = qz
        out.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(out)
        self._last_publish_s = now_s


def main() -> None:
    rclpy.init()
    node = IsaacVslamFrameBridge()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
