#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, String


def yaw_from_quaternion(q) -> float:
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


class DistanceTracker:
    def __init__(self) -> None:
        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.distance_m = 0.0

    def update(self, odom: Odometry) -> None:
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        if self.last_x is not None and self.last_y is not None:
            step = math.hypot(x - self.last_x, y - self.last_y)
            if step < 2.0:
                self.distance_m += step
        self.last_x = x
        self.last_y = y


class RelativePoseTracker:
    def __init__(self) -> None:
        self.origin_x: Optional[float] = None
        self.origin_y: Optional[float] = None
        self.origin_yaw: Optional[float] = None

    def relative_pose(self, odom: Odometry):
        pose = odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        if self.origin_x is None or self.origin_y is None or self.origin_yaw is None:
            self.origin_x = pose.position.x
            self.origin_y = pose.position.y
            self.origin_yaw = yaw

        dx = pose.position.x - self.origin_x
        dy = pose.position.y - self.origin_y
        c = math.cos(-self.origin_yaw)
        s = math.sin(-self.origin_yaw)
        return (
            c * dx - s * dy,
            s * dx + c * dy,
            wrap_to_pi(yaw - self.origin_yaw),
        )


class SimModelMonitor(Node):
    def __init__(self) -> None:
        super().__init__("sim_model_monitor")
        self.declare_parameter("ground_truth_topic", "/mtt_odometry/ground_truth")
        self.declare_parameter("model_odom_topic", "/mtt_odometry")
        self.declare_parameter("publish_period_s", 0.5)

        self.ground_truth: Optional[Odometry] = None
        self.model_odom: Optional[Odometry] = None
        self.gt_tracker = DistanceTracker()
        self.model_tracker = DistanceTracker()
        self.gt_relative = RelativePoseTracker()
        self.model_relative = RelativePoseTracker()

        self.gt_topic = str(self.get_parameter("ground_truth_topic").value)
        self.model_topic = str(self.get_parameter("model_odom_topic").value)

        self.create_subscription(
            Odometry,
            self.gt_topic,
            self._on_ground_truth,
            20,
        )
        self.create_subscription(
            Odometry,
            self.model_topic,
            self._on_model_odom,
            20,
        )

        self.position_error_pub = self.create_publisher(Float64, "~/position_error_m", 10)
        self.heading_error_pub = self.create_publisher(Float64, "~/heading_error_rad", 10)
        self.distance_ratio_pub = self.create_publisher(Float64, "~/distance_ratio_model_over_gt", 10)
        self.ground_truth_distance_pub = self.create_publisher(Float64, "~/ground_truth_distance_m", 10)
        self.model_distance_pub = self.create_publisher(Float64, "~/model_distance_m", 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)

        period = float(self.get_parameter("publish_period_s").value)
        self.create_timer(period, self._publish)

        self.get_logger().info(
            f"Sim model monitor ready: comparing {self.model_topic} with {self.gt_topic}"
        )

    def _on_ground_truth(self, msg: Odometry) -> None:
        self.ground_truth = msg
        self.gt_tracker.update(msg)

    def _on_model_odom(self, msg: Odometry) -> None:
        self.model_odom = msg
        self.model_tracker.update(msg)

    def _publish_float(self, publisher, value: float) -> None:
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

    def _publish(self) -> None:
        if self.ground_truth is None or self.model_odom is None:
            return

        gt_x, gt_y, gt_yaw = self.gt_relative.relative_pose(self.ground_truth)
        model_x, model_y, model_yaw = self.model_relative.relative_pose(self.model_odom)
        position_error = math.hypot(model_x - gt_x, model_y - gt_y)
        heading_error = wrap_to_pi(model_yaw - gt_yaw)
        ratio = self.model_tracker.distance_m / max(self.gt_tracker.distance_m, 1e-6)

        self._publish_float(self.position_error_pub, position_error)
        self._publish_float(self.heading_error_pub, heading_error)
        self._publish_float(self.distance_ratio_pub, ratio)
        self._publish_float(self.ground_truth_distance_pub, self.gt_tracker.distance_m)
        self._publish_float(self.model_distance_pub, self.model_tracker.distance_m)

        status = String()
        status.data = (
            f"model_error={position_error:.2f} m heading={math.degrees(heading_error):.1f} deg "
            f"distance_ratio={ratio:.2f} model={self.model_tracker.distance_m:.2f} m "
            f"gt={self.gt_tracker.distance_m:.2f} m"
        )
        self.status_pub.publish(status)
        self.get_logger().info(status.data, throttle_duration_sec=2.0)


def main() -> None:
    rclpy.init()
    node = SimModelMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
