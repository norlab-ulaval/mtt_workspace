#!/usr/bin/env python3
import math
from copy import deepcopy
from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
    )


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class SimOdomAligner(Node):
    def __init__(self) -> None:
        super().__init__("sim_model_odom_aligner")
        self.declare_parameter("model_odom_topic", "/mtt_odometry")
        self.declare_parameter("ground_truth_topic", "/mtt_odometry/ground_truth")
        self.declare_parameter("aligned_odom_topic", "/mtt_odometry/aligned")

        self.model_origin: Optional[Tuple[float, float, float]] = None
        self.gt_origin: Optional[Tuple[float, float, float]] = None
        self.last_gt: Optional[Odometry] = None

        self.pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("aligned_odom_topic").value),
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("ground_truth_topic").value),
            self._on_ground_truth,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("model_odom_topic").value),
            self._on_model_odom,
            20,
        )
        self.model_topic = str(self.get_parameter("model_odom_topic").value)
        self.gt_topic = str(self.get_parameter("ground_truth_topic").value)
        self.aligned_topic = str(self.get_parameter("aligned_odom_topic").value)
        self.get_logger().info(
            f"Sim odom aligner ready: {self.model_topic} aligned on {self.gt_topic} -> {self.aligned_topic}"
        )

    def _pose_tuple(self, msg: Odometry) -> Tuple[float, float, float]:
        pose = msg.pose.pose
        return (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

    def _on_ground_truth(self, msg: Odometry) -> None:
        self.last_gt = msg
        if self.gt_origin is None:
            self.gt_origin = self._pose_tuple(msg)

    def _on_model_odom(self, msg: Odometry) -> None:
        if self.model_origin is None:
            self.model_origin = self._pose_tuple(msg)
        if self.gt_origin is None:
            return

        model_x, model_y, model_yaw = self._pose_tuple(msg)
        origin_x, origin_y, origin_yaw = self.model_origin
        gt_x, gt_y, gt_yaw = self.gt_origin

        dx = model_x - origin_x
        dy = model_y - origin_y
        c = math.cos(gt_yaw - origin_yaw)
        s = math.sin(gt_yaw - origin_yaw)

        aligned = Odometry()
        aligned.header = msg.header
        aligned.header.frame_id = "odom"
        aligned.child_frame_id = msg.child_frame_id or "base_footprint"
        aligned.pose = deepcopy(msg.pose)
        aligned.twist = deepcopy(msg.twist)
        aligned.pose.pose.position.x = gt_x + c * dx - s * dy
        aligned.pose.pose.position.y = gt_y + s * dx + c * dy
        aligned.pose.pose.position.z = 0.0
        aligned.pose.pose.orientation = yaw_to_quaternion(wrap_to_pi(gt_yaw + model_yaw - origin_yaw))
        self.pub.publish(aligned)


def main() -> None:
    rclpy.init()
    node = SimOdomAligner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
