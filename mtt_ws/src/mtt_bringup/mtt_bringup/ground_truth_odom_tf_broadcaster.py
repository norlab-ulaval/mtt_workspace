#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry as NavOdometry


class GroundTruthOdomTF(Node):
    def __init__(self):
        super().__init__('ground_truth_odom_tf_broadcaster')
        self.declare_parameter('odom_topic', '/ground_truth_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self.odom_cb, 10)
        # Publisher to /odom so other nodes that expect /odom receive the ground truth
        self._odom_pub = self.create_publisher(NavOdometry, '/odom', 10)
        self.get_logger().info(f'Publishing TF {self._odom_frame} -> {self._base_frame} from {topic}')

    def odom_cb(self, msg: Odometry):
        # Debug: log first few messages to confirm reception
        if not hasattr(self, '_msg_count'):
            self._msg_count = 0
        if self._msg_count < 5:
            self.get_logger().info(
                f"Received odom seq {self._msg_count} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec} frame_id={msg.header.frame_id} child={msg.child_frame_id}"
            )
        self._msg_count += 1

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)

        # Republish odometry to /odom with expected frame IDs
        odom_out = NavOdometry()
        odom_out.header = msg.header
        odom_out.header.frame_id = self._odom_frame
        odom_out.child_frame_id = self._base_frame
        odom_out.pose = msg.pose
        odom_out.twist = msg.twist
        self._odom_pub.publish(odom_out)


def main():
    rclpy.init()
    rclpy.spin(GroundTruthOdomTF())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

