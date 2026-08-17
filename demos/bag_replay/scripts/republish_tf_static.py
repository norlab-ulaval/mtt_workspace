#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
import time

class TFStaticRepublisher(Node):
    def __init__(self):
        super().__init__('tf_static_republisher')
        self.get_logger().info('Waiting for /clock before republishing /tf_static...')

        # Wait for /clock
        clock_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.clock_sub = self.create_subscription(String, '/clock', self.clock_cb, clock_qos)
        self.clock_received = False

    def clock_cb(self, msg):
        if self.clock_received:
            return
        self.clock_received = True
        self.get_logger().info('/clock received, republishing /tf_static with sim time...')

        # Give /clock time to propagate
        time.sleep(1.0)

        # Get current tf_static by subscribing (transient_local)
        self.tf_static_data = None
        tf_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.tf_sub = self.create_subscription(TFMessage, '/tf_static', self.tf_cb, tf_qos)
        # Wait for the latched message
        time.sleep(2.0)

        if self.tf_static_data is not None:
            # Republish with current sim time
            now = self.get_clock().now()
            for transform in self.tf_static_data.transforms:
                transform.header.stamp = now.to_msg()
            pub = self.create_publisher(TFMessage, '/tf_static', tf_qos)
            pub.publish(self.tf_static_data)
            self.get_logger().info(
                f'Republished {len(self.tf_static_data.transforms)} transforms '
                f'at sim time {now.to_msg().sec}.{now.to_msg().nanosec}'
            )
        else:
            self.get_logger().error('Could not get /tf_static message')

        # Keep spinning
        self.tf_sub = None  # allow cleanup

    def tf_cb(self, msg):
        if self.tf_static_data is None:
            self.tf_static_data = msg
            self.get_logger().info(f'Got /tf_static with {len(msg.transforms)} transforms')


def main():
    rclpy.init()
    node = TFStaticRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
