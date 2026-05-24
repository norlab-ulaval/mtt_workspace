#!/usr/bin/env python3
"""Convert ZED Pose/Odometry to Path and Markers for visualization."""

import argparse
import math
import collections
from typing import Deque, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Quaternion
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


class ZedPoseVisualizer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("zed_pose_visualizer")
        self.map_frame = args.map_frame
        self.target_frame = args.target_frame # e.g. base_link or base_footprint
        self.history_limit = args.history_limit
        
        self.path = Path()
        self.path.header.frame_id = self.map_frame
        
        self.initial_offset: Optional[TransformStamped] = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.path_pub = self.create_publisher(Path, "zed/path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "zed/markers", 10)

        self.create_subscription(PoseStamped, args.pose_topic, self.on_pose, qos)
        self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info(f"Visualizing ZED pose from {args.pose_topic} in {self.map_frame}")

    def on_pose(self, msg: PoseStamped) -> None:
        # If the pose is already in the map frame, we just add it to the path
        # If it's in another frame (like 'odom_zed'), we might need to align it.
        # However, for now we assume we want to show it RELATIVE to where it started
        # OR we use the current TF to place it in the map if it's in base_link.
        
        # Add to path
        pose_in_path = PoseStamped()
        pose_in_path.header = msg.header
        pose_in_path.header.frame_id = self.map_frame # We want to visualize it in map
        pose_in_path.pose = msg.pose
        
        self.path.poses.append(pose_in_path)
        if len(self.path.poses) > self.history_limit:
            self.path.poses.pop(0)
            
        self.path.header.stamp = msg.header.stamp
        self.path_pub.publish(self.path)

    def publish_markers(self) -> None:
        if not self.path.poses:
            return
            
        latest_pose = self.path.poses[-1]
        
        markers = MarkerArray()
        
        # Ghost robot (Arrow)
        arrow = Marker()
        arrow.header = latest_pose.header
        arrow.ns = "zed_ghost"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = latest_pose.pose
        arrow.scale.x = 0.5
        arrow.scale.y = 0.1
        arrow.scale.z = 0.1
        arrow.color.r = 0.0
        arrow.color.g = 0.8
        arrow.color.b = 1.0
        arrow.color.a = 0.5
        markers.markers.append(arrow)
        
        # Label
        label = Marker()
        label.header = latest_pose.header
        label.ns = "zed_ghost"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose = latest_pose.pose
        label.pose.position.z += 0.5
        label.scale.z = 0.2
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = "ZED Pose"
        markers.markers.append(label)
        
        self.marker_pub.publish(markers)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-topic", default="/zed/zed_node/pose")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--history-limit", type=int, default=1000)
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = ZedPoseVisualizer(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
