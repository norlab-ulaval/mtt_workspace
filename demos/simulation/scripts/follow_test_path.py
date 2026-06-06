#!/usr/bin/env python3
import argparse
import math
import sys
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from norlab_controllers_msgs.action import FollowPath
from norlab_controllers_msgs.msg import DirectionalPath
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
    )


def build_straight(length: float, points: int) -> List[Tuple[float, float, float]]:
    return [(length * i / max(points - 1, 1), 0.0, 0.0) for i in range(points)]


def build_arc(radius: float, angle_rad: float, points: int) -> List[Tuple[float, float, float]]:
    poses = []
    for i in range(points):
        theta = angle_rad * i / max(points - 1, 1)
        x = radius * math.sin(theta)
        y = radius * (1.0 - math.cos(theta))
        yaw = theta
        poses.append((x, y, yaw))
    return poses


def build_s_curve(length: float, amplitude: float, cycles: float, points: int) -> List[Tuple[float, float, float]]:
    poses = []
    omega = 2.0 * math.pi * cycles / max(length, 1e-6)
    for i in range(points):
        x = length * i / max(points - 1, 1)
        y = amplitude * math.sin(omega * x)
        dy_dx = amplitude * omega * math.cos(omega * x)
        yaw = math.atan2(dy_dx, 1.0)
        poses.append((x, y, yaw))
    return poses


def make_pose(frame_id: str, stamp, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = yaw_to_quaternion(yaw)
    return pose


def apply_local_offset(
    samples: List[Tuple[float, float, float]],
    offset_x: float,
    offset_y: float,
    offset_yaw: float,
) -> List[Tuple[float, float, float]]:
    if abs(offset_x) < 1e-12 and abs(offset_y) < 1e-12 and abs(offset_yaw) < 1e-12:
        return samples
    c = math.cos(offset_yaw)
    s = math.sin(offset_yaw)
    return [
        (
            offset_x + c * x - s * y,
            offset_y + s * x + c * y,
            yaw + offset_yaw,
        )
        for x, y, yaw in samples
    ]


class FollowTestPath(Node):
    def __init__(self, args):
        super().__init__("sim_follow_test_path")
        self.args = args
        self.action_client = ActionClient(self, FollowPath, args.action)
        self.auto_client = self.create_client(Trigger, args.auto_service)
        self.path_pub = self.create_publisher(Path, args.preview_topic, 1)
        self.odom = None
        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def wait_for_odom(self) -> bool:
        if self.args.absolute:
            return True
        end_time = self.get_clock().now().nanoseconds / 1e9 + 5.0
        while rclpy.ok() and self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds / 1e9 > end_time:
                self.get_logger().error(f"no odometry received on {self.args.odom_topic}")
                return False
        return True

    def apply_current_pose(self, samples: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        if self.args.absolute:
            return samples
        pose = self.odom.pose.pose
        x0 = pose.position.x
        y0 = pose.position.y
        yaw0 = yaw_from_quaternion(pose.orientation)
        c = math.cos(yaw0)
        s = math.sin(yaw0)
        transformed = []
        for x, y, yaw in samples:
            transformed.append((x0 + c * x - s * y, y0 + s * x + c * y, yaw0 + yaw))
        return transformed

    def build_path(self) -> DirectionalPath:
        if self.args.shape == "straight":
            samples = build_straight(self.args.length, self.args.points)
        elif self.args.shape == "arc":
            samples = build_arc(self.args.radius, math.radians(self.args.angle_deg), self.args.points)
        elif self.args.shape == "s_curve":
            samples = build_s_curve(self.args.length, self.args.amplitude, self.args.cycles, self.args.points)
        else:
            raise ValueError(f"unsupported shape: {self.args.shape}")
        samples = apply_local_offset(
            samples,
            self.args.offset_x,
            self.args.offset_y,
            math.radians(self.args.offset_yaw_deg),
        )
        samples = self.apply_current_pose(samples)

        stamp = self.get_clock().now().to_msg()
        path = DirectionalPath()
        path.header.frame_id = self.args.frame
        path.header.stamp = stamp
        path.forward = not self.args.reverse
        path.poses = [make_pose(self.args.frame, stamp, x, y, yaw) for x, y, yaw in samples]
        return path

    def publish_preview(self, directional_path: DirectionalPath) -> None:
        msg = Path()
        msg.header = directional_path.header
        msg.poses = list(directional_path.poses)
        self.path_pub.publish(msg)

    def request_auto(self) -> bool:
        if self.args.no_auto:
            return True
        if not self.auto_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"service not available: {self.args.auto_service}")
            return False
        future = self.auto_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is None:
            self.get_logger().error("auto request timed out")
            return False
        result = future.result()
        if not result.success:
            self.get_logger().error(f"auto refused: {result.message}")
            return False
        self.get_logger().info(f"auto enabled: {result.message}")
        return True

    def send_goal(self, directional_path: DirectionalPath) -> int:
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"action server not available: {self.args.action}")
            return 2

        goal = FollowPath.Goal()
        goal.follower_options.velocity.data = float(self.args.speed)
        goal.path.header = directional_path.header
        goal.path.paths.append(directional_path)

        self.get_logger().info(
            f"sending {self.args.shape} path: poses={len(directional_path.poses)} speed={self.args.speed:.2f} m/s"
        )
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("follow path goal rejected")
            return 3

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"follow path result_status={result.result_status.data}")
        return 0

    def run(self) -> int:
        if not self.wait_for_odom():
            return 1
        directional_path = self.build_path()
        self.publish_preview(directional_path)
        if self.args.preview_only:
            return 0
        if not self.request_auto():
            return 1
        return self.send_goal(directional_path)


def parse_args():
    parser = argparse.ArgumentParser(description="Send an analytical test path to /follow_path in MTT simulation.")
    parser.add_argument("--shape", choices=["straight", "arc", "s_curve"], default="s_curve")
    parser.add_argument("--speed", type=float, default=0.6)
    parser.add_argument("--length", type=float, default=12.0)
    parser.add_argument("--radius", type=float, default=4.0)
    parser.add_argument("--angle-deg", type=float, default=120.0)
    parser.add_argument("--amplitude", type=float, default=1.2)
    parser.add_argument("--cycles", type=float, default=1.0)
    parser.add_argument("--points", type=int, default=160)
    parser.add_argument("--frame", default="odom")
    parser.add_argument("--odom-topic", default="/mtt_odometry/ground_truth")
    parser.add_argument("--action", default="/follow_path")
    parser.add_argument("--auto-service", default="/mtt_control/request_auto")
    parser.add_argument("--preview-topic", default="/sim_motion_model/test_path")
    parser.add_argument("--offset-x", type=float, default=0.0)
    parser.add_argument("--offset-y", type=float, default=0.0)
    parser.add_argument("--offset-yaw-deg", type=float, default=0.0)
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--absolute", action="store_true")
    parser.add_argument("--no-auto", action="store_true")
    parser.add_argument("--preview-only", action="store_true")
    return parser.parse_args()


def main() -> int:
    rclpy.init()
    node = FollowTestPath(parse_args())
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
