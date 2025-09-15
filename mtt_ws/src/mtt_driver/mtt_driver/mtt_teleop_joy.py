import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from mtt_msgs.msg import MttAuxCommand
from enum import Enum
from rcl_interfaces.msg import Log
import pyinotify
import os
from ament_index_python.packages import get_package_share_directory
import yaml


class MTTTeleopJoy(Node):
    """Translates /joy messages to /cmd_vel and /mtt_aux_cmd, including light toggle and winch safety."""

    def __init__(self):
        super().__init__("mtt_teleop_joy")
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.aux_cmd_pub = self.create_publisher(MttAuxCommand, "mtt_aux_cmd", 10)

        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.get_logger().info("MTT Teleop Node started.")

        self.axis_map = {"linear_speed": None, "rotation_speed": None, "brake": None, "winch": None}
        self.button_map = {"nav_safety": None, "light_toggle": None, "winch_safety": None}
        self.prev_light_btn = 0
        self.light_state = False

        self.current_joystick_controller = None
        self.is_safety_locked = True

        self.last_toggle_light_command = 0

        self.joystick_name = self._get_joystick_name("/dev/input/js0")
        self.get_logger().info(f"Initial joystick: {self.joystick_name}")
        self.set_joystick_mapper(self.joystick_name)

        # pyinotify setup
        self.wm = pyinotify.WatchManager()
        mask = pyinotify.IN_CREATE | pyinotify.IN_DELETE | pyinotify.IN_ATTRIB | pyinotify.IN_MOVED_TO | pyinotify.IN_MOVED_FROM

        # Add watch to /dev/input
        self.notifier = pyinotify.ThreadedNotifier(self.wm, self.event_handler)
        self.wm.add_watch("/dev/input", mask)
        self.notifier.start()

    def _get_joystick_name(self, js_dev="/dev/input/js0"):

        try:
            with open(f"/sys/class/input/{js_dev.split('/')[-1]}/device/name") as f:
                return f.read().strip()
        except Exception as e:
            self.get_logger().error(f"Could not read joystick name: {e}")
            return "Unknown"
        
    def set_joystick_mapper(self, received_string):

        if "Logitech Gamepad F310" in received_string:
            self.load_params_from_file("logitech_gamepad_310_mapper.yaml")

        elif "8BitDo Ultimate Wireless" in received_string:
            self.load_params_from_file("8BitDo_ultimate_wireless.yaml")

        else:
            self.get_logger().warn("Unsupported Joystick controller detected")
            self.current_joystick_controller = None

        self.is_safety_locked = True


    def load_params_from_file(self, yaml_file: str):
        pkg_share = get_package_share_directory("mtt_driver")
        yaml_path = os.path.join(pkg_share, "config", yaml_file)
    
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        self.get_logger().debug(f"Loaded parameters from {yaml_file}")

        params = data["mtt_teleop_joy"]
        self.axis_map["linear_speed"] = params["axis"]["linear_speed"]
        self.axis_map["rotation_speed"] = params["axis"]["rotation_speed"]
        self.axis_map["brake"] = params["axis"]["brake"]
        self.axis_map["winch"] = params["axis"]["winch"]
        
        self.button_map["nav_safety"] = params["buttons"]["nav_safety"]
        self.button_map["winch_safety"] = params["buttons"]["winch_safety"]
        self.button_map["light_toggle"] = params["buttons"]["light_toggle"]

        # self.linear

    def event_handler(self, event):
        # Check if the event concerns js0
        if os.path.basename(event.pathname) == "js0":
            # self.get_logger().info(f"Event on {event.pathname}: {event.maskname}")
            new_name = self._get_joystick_name()
            if new_name != self.joystick_name:
                self.get_logger().info(f"Joystick changed: {self.joystick_name} → {new_name}")
                self.joystick_name = new_name
                self.set_joystick_mapper(new_name)

    def joy_callback(self, msg: Joy):

        twist_msg = Twist()
        twist_msg.linear.x = msg.axes[self.axis_map["linear_speed"]]
        twist_msg.angular.z = msg.axes[self.axis_map["rotation_speed"]]
        self.cmd_vel_pub.publish(twist_msg)

        aux_msg = MttAuxCommand()
        aux_msg.dead_man_switch = bool(msg.buttons[self.button_map["nav_safety"]])
        aux_msg.brake = (1.0 - msg.axes[self.axis_map["brake"]]) / 2.0
        
        # Winch safety button
        aux_msg.winch_safety_button = bool(msg.buttons[self.button_map["winch_safety"]])
         
        # Winch command from D-pad
        dpad_val = msg.axes[self.axis_map["winch"]]
        if dpad_val > 0.5:
            aux_msg.winch_command = MttAuxCommand.WINCH_IN
        elif dpad_val < -0.5:
            aux_msg.winch_command = MttAuxCommand.WINCH_OUT
        else:
            aux_msg.winch_command = MttAuxCommand.WINCH_NEUTRAL

        # Light toggle logic (rising edge detection)
        light_btn = bool(msg.buttons[self.button_map["light_toggle"]])
        if light_btn == 1 and self.prev_light_btn == 0:
            self.light_state = not self.light_state
        self.prev_light_btn = light_btn

        # Set light state if supported
        if hasattr(aux_msg, "light_state"):
            aux_msg.light_state = self.light_state

        self.get_logger().info(str(self.light_state))
        self.aux_cmd_pub.publish(aux_msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = MTTTeleopJoy()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
