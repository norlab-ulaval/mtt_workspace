#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class MttControllerInterface(Node):

    def __init__(self):
        super().__init__('mtt_controller_interface')
        self.wheel_publisher = self.create_publisher(Float64MultiArray, '/wheel_group_controller/commands', 10)
        self.yaw_publisher = self.create_publisher(Float64MultiArray, '/yaw_controller/commands', 10)

        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # self.timer = self.create_timer(0.1, self.publish_message)  # Publish every second


    def cmd_vel_callback(self, cmd_vel_msg: Twist):

        # negative sign to have the right direction
        # *20 to make an array for each of the 20 wheels
        wheel_speeds = [-cmd_vel_msg.linear.x] * 20

        wheel_cmd_msg = Float64MultiArray()
        wheel_cmd_msg.data = wheel_speeds
        self.wheel_publisher.publish(wheel_cmd_msg)

        yaw_cmd_msg = Float64MultiArray()
       
        yaw = [-cmd_vel_msg.angular.z]
        yaw_cmd_msg.data = yaw
        self.yaw_publisher.publish(yaw_cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MttControllerInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()