#!/usr/bin/env python3
"""Republish ZED visual-inertial odometry as the odom → base_footprint TF.

Replay-only odom prior for the ICP mapper when wheel odometry is unusable
(track slip downhill makes the translation prior diverge). The ZED odometry
in the bag (odom_zed → zed_camera_link) tracked those segments correctly.

T_odom_base = T_odomzed_zedcam · T_zedcam_base

T_zedcam_base is looked up once from /tf_static (URDF via the description
service). Requires the bag player to keep /zed/zed_node/odom
(REPLAY_KEEP_ZED_ODOM=true).

Do NOT run together with runtime_odometry or imu_odom — all of them publish
the odom → base_footprint TF.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def quat_to_mat(x, y, z, w):
    n = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return n


def mat_to_quat(m):
    t = np.trace(m)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    else:
        i = np.argmax(np.diag(m))
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
    return x, y, z, w


class ZedOdomToTf(Node):
    def __init__(self):
        super().__init__("zed_odom_to_tf")
        self.declare_parameter("odom_topic", "/zed/zed_node/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("camera_frame", "zed_camera_link")

        self.odom_frame = self.get_parameter("odom_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.t_cam_base = None  # 4x4, zed_camera_link → base_footprint
        self.msg_count = 0

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, qos)
        self.get_logger().info(
            f"ZED odom → TF bridge: {self.get_parameter('odom_topic').value} "
            f"→ {self.odom_frame} → {self.robot_frame}")

    def lookup_extrinsic(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                self.camera_frame, self.robot_frame, rclpy.time.Time())
        except Exception:
            return None
        t = np.eye(4)
        q = tr.transform.rotation
        t[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
        t[:3, 3] = [tr.transform.translation.x,
                    tr.transform.translation.y,
                    tr.transform.translation.z]
        return t

    def on_odom(self, msg: Odometry):
        if self.t_cam_base is None:
            self.t_cam_base = self.lookup_extrinsic()
            if self.t_cam_base is None:
                return  # static TF not available yet
            self.get_logger().info(
                f"Extrinsic {self.camera_frame} → {self.robot_frame} acquired: "
                f"t={self.t_cam_base[:3, 3]}")

        t_odom_cam = np.eye(4)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t_odom_cam[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
        t_odom_cam[:3, 3] = [p.x, p.y, p.z]

        t_odom_base = t_odom_cam @ self.t_cam_base

        out = TransformStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.robot_frame
        out.transform.translation.x = t_odom_base[0, 3]
        out.transform.translation.y = t_odom_base[1, 3]
        out.transform.translation.z = t_odom_base[2, 3]
        x, y, z, w = mat_to_quat(t_odom_base[:3, :3])
        out.transform.rotation.x = x
        out.transform.rotation.y = y
        out.transform.rotation.z = z
        out.transform.rotation.w = w
        self.broadcaster.sendTransform(out)

        self.msg_count += 1
        if self.msg_count % 1500 == 0:
            self.get_logger().info(
                f"odom TF #{self.msg_count}: pos=({t_odom_base[0,3]:.2f}, "
                f"{t_odom_base[1,3]:.2f}, {t_odom_base[2,3]:.2f})")


def main():
    rclpy.init()
    node = ZedOdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
