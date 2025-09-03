#!/usr/bin/env python3
"""MTT-154 Odometry Node: Dedicated composable node for odometry calculations."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mtt_msgs.msg import MttTachometerData


class MttOdometryNode(Node):
    """
    Composable ROS2 node for MTT-154 odometry calculations.
    
    Subscribes to:
        - /mtt_tachometer: Raw tachometer data from wrapper
    
    Publishes:
        - /mtt_odometry: Standard ROS odometry message
    """

    def __init__(self):
        super().__init__("mtt_odometry_node")
        
        # Odometry state tracking using absolute distance from driver
        self.ros_position_x = 0.0
        self.last_absolute_distance_m = 0.0
        
        # Publisher - standard ROS odometry topic
        self.odom_publisher = self.create_publisher(Odometry, "/mtt_odometry", 10)
        
        # Subscriber to tachometer data from wrapper
        self.tachometer_subscriber = self.create_subscription(
            MttTachometerData,
            "/mtt_tachometer",
            self.tachometer_callback,
            10
        )
        
        self.get_logger().info("MTT Odometry Node initialized - subscribing to /mtt_tachometer")

    def tachometer_callback(self, msg: MttTachometerData):
        """
        Process tachometer data and publish odometry.
        
        Uses absolute distance from driver hardware with incremental ROS position tracking.
        """
        # Calculate and publish odometry directly
        self._publish_odometry(msg)

    def _publish_odometry(self, tachometer_msg: MttTachometerData):
        """
        Publish standard ROS2 odometry message.
        
        Uses incremental calculation based on absolute distance from hardware.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "mtt_base_link"

        # Convert distance back to meters and calculate increment 
        # (driver provides absolute distance, we need incremental for ROS position)
        current_absolute_distance_m = tachometer_msg.distance_km * 1000.0
        distance_increment = current_absolute_distance_m - self.last_absolute_distance_m
        
        # Apply direction-based movement to ROS position
        if tachometer_msg.direction == "Forward":
            self.ros_position_x += distance_increment
        else:  # Reverse
            self.ros_position_x -= distance_increment
        self.last_absolute_distance_m = current_absolute_distance_m

        # Position (X-axis movement only for ground vehicle)
        odom_msg.pose.pose.position.x = self.ros_position_x
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (no rotation for this implementation)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        # Velocity from hardware speed calculation
        odom_msg.twist.twist.linear.x = tachometer_msg.speed_ms
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Publish to standard odometry topic
        self.odom_publisher.publish(odom_msg)


def main(args=None):
    """Main entry point for standalone execution."""
    rclpy.init(args=args)
    
    try:
        odometry_node = MttOdometryNode()
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'odometry_node' in locals():
            odometry_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
