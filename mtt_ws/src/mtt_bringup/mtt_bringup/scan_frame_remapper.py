#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class FrameRemapper(Node):
    def __init__(self):
        super().__init__('scan_frame_remapper')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.sub = self.create_subscription(LaserScan, '/gz_scan', self.callback, 10)

    def callback(self, msg):
        # When receiving the scans from gazebo, the frame_id is not correct
        msg.header.frame_id = 'lidar_link'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrameRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
