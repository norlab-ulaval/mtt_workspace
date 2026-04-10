import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from mtt_msgs.msg import MttAuxCommand
import pyinotify
import os
from ament_index_python.packages import get_package_share_directory
import yaml


class MTTTeleopJoy(Node):
    """Translates /joy messages to /cmd_vel and /mtt_aux_cmd, including light toggle and winch safety."""

    def __init__(self):
        super().__init__("mtt_teleop_joy")


        self.maximmum_linear_speed = 0.1  # m/s
        self.maximum_angular_speed = 0.1  # rad/s
        self.declare_parameter("max_linear_speed", self.maximmum_linear_speed)
        self.declare_parameter("max_angular_speed", self.maximum_angular_speed) 
        self.maximmum_linear_speed = self.get_parameter("max_linear_speed").value
        self.maximum_angular_speed = self.get_parameter("max_angular_speed").value  


        self.cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel_raw", 10)
        self.aux_cmd_pub = self.create_publisher(MttAuxCommand, "mtt_aux_cmd", 10)

        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.get_logger().info("MTT Teleop Node started.")


        self.axis_map = {"linear_speed": None, "rotation_speed": None, "brake": None, "winch": None}
        self.button_map = {"deadman": None, "light_toggle": None, "winch_safety": None}
        self.prev_light_btn = 0
        self.light_state = False
        self.safety_state = False

    def joy_callback(self, msg: Joy):
        
        twist_msg = TwistStamped()
        aux_msg = MttAuxCommand()


        if msg.buttons[5]:

            if msg.axes[4] < -0.05:
                twist_msg.twist.linear.x = - 0.3 + msg.axes[4] * 0.3
            elif msg.axes[4] > 0.05:
                twist_msg.twist.linear.x = 0.3 + msg.axes[4] * 0.3
            else:
                twist_msg.twist.linear.x = 0.0
                
            if msg.axes[3] < -0.05 or msg.axes[3] > 0.05:
                twist_msg.twist.angular.z = msg.axes[3]
            else:
                twist_msg.twist.angular.z = 0.0

            aux_msg.brake = (-msg.axes[5] + 1 )/2
            
            
            # Winch command from D-pad
            # dpad_val = msg.axes[7]
            # if dpad_val > 0.5:
            #     aux_msg.winch_command = 1
            # elif dpad_val < -0.5:
            #     aux_msg.winch_command = 2
            # else:
            #     aux_msg.winch_command = 0

            # Light toggle logic (rising edge detection)
            light_btn = msg.buttons[2]
            if light_btn == 1 and self.prev_light_btn == 0:
                self.light_state = not self.light_state
            self.prev_light_btn = light_btn

            # Set light state if supported
            aux_msg.light_state = self.light_state

        else: 
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist_msg)
        self.aux_cmd_pub.publish(aux_msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = MTTTeleopJoy()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
