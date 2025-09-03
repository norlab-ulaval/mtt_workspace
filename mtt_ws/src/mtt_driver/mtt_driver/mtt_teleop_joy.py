import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from mtt_msgs.msg import MttAuxCommand


class MTTTeleopJoy(Node):
    """Translates /joy messages to /cmd_vel and /mtt_aux_cmd, including light toggle."""

    def __init__(self):
        super().__init__("mtt_teleop_joy")
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.aux_cmd_pub = self.create_publisher(MttAuxCommand, "mtt_aux_cmd", 10)
        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.get_logger().info("MTT Teleop Node started.")

        self.axis_map = {"left_v": 1, "right_h": 3, "brake": 5, "dpad_v": 7}
        self.button_map = {"dead_man": 5, "light_toggle": 4}
        self.prev_light_btn = 0
        self.light_state = False

    def joy_callback(self, msg: Joy):
        twist_msg = Twist()
        twist_msg.linear.x = msg.axes[self.axis_map["left_v"]]
        twist_msg.angular.z = msg.axes[self.axis_map["right_h"]]
        self.cmd_vel_pub.publish(twist_msg)

        aux_msg = MttAuxCommand()
        aux_msg.dead_man_switch = msg.buttons[self.button_map["dead_man"]] == 1
        aux_msg.brake = (1.0 - msg.axes[self.axis_map["brake"]]) / 2.0
        dpad_val = msg.axes[self.axis_map["dpad_v"]]
        if dpad_val > 0.5:
            aux_msg.winch_command = MttAuxCommand.WINCH_IN
        elif dpad_val < -0.5:
            aux_msg.winch_command = MttAuxCommand.WINCH_OUT
        else:
            aux_msg.winch_command = MttAuxCommand.WINCH_NEUTRAL

        # Light toggle logic (rising edge detection)
        light_btn = msg.buttons[self.button_map["light_toggle"]]
        if light_btn == 1 and self.prev_light_btn == 0:
            self.light_state = not self.light_state
        self.prev_light_btn = light_btn

        # Set light state if supported
        if hasattr(aux_msg, "light_state"):
            aux_msg.light_state = int(self.light_state)

        self.aux_cmd_pub.publish(aux_msg)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = MTTTeleopJoy()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
