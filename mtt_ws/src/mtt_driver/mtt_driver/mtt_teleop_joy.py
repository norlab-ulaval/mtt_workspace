import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from mtt_msgs.msg import MttAuxCommand
import pyinotify
import os
from ament_index_python.packages import get_package_share_directory
import yaml


class MTTTeleopJoy(Node):
    """Translates /joy messages to /cmd_vel and /mtt_aux_cmd, including light toggle and winch safety."""

    def __init__(self):
        super().__init__("mtt_teleop_joy_node")
        # Publish to the correct topic for twist_mux
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel/teleop", 10)
        self.aux_cmd_pub = self.create_publisher(MttAuxCommand, "mtt_aux_cmd", 10)

        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.get_logger().info("MTT Teleop Node started.")

        self.mapping_registry = self.load_mapping_registry()

        self.axis_map = {"linear_speed": None, "rotation_speed": None, "brake": None, "winch": None}
        self.button_map = {"nav_safety": None, "light_toggle": None, "winch_safety": None}
        self.prev_light_btn = 0
        self.light_state = False

        self.is_initialized = False
        
        # Add debouncing for joystick detection to prevent rapid reconnections
        self.last_joystick_check = 0.0
        self.joystick_check_interval = 2.0  # Only check every 2 seconds
        self.last_known_good_name = None

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

        pkg_share = get_package_share_directory("mtt_driver")
        config_dir = os.path.join(pkg_share, "config")
        self.wm.add_watch(config_dir, mask, rec=False)

    def _get_joystick_name(self, js_dev="/dev/input/js0"):

        try:
            with open(f"/sys/class/input/{js_dev.split('/')[-1]}/device/name") as f:
                return f.read().strip()
        except Exception as e:
            self.get_logger().error(f"Could not read joystick name: {e}")
            return "Unknown"
        
    def set_joystick_mapper(self, received_string):
        for name, yaml_file in self.mapping_registry.items():
            if name in received_string:
                self.load_params_from_file(yaml_file)
                return
        self.get_logger().warn(f"No mapping found for joystick: {received_string}")

    def load_mapping_registry(self):
        pkg_share = get_package_share_directory("mtt_driver")
        mapping_path = os.path.join(pkg_share, "config", "joystick_mappings.yaml")
        try:
            with open(mapping_path, 'r') as f:
                data = yaml.safe_load(f)
            return data.get("joystick_mappings", {})
        except Exception as e:
            self.get_logger().error(f"Failed to load joystick mapping registry: {e}")
            return {}
    
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

        self.is_initialized = True

    def _get_button(self, msg: Joy, name: str, default: bool = False):
        """Return a button as bool; safe if mapping is missing or out of range."""
        idx = self.button_map.get(name)
        if idx is None:
            return default
        try:
            return bool(msg.buttons[idx])
        except (IndexError, TypeError):
            return default

    def _get_axis(self, msg: Joy, name: str, default: float = 0.0):
        """Return an axis as float; safe if mapping is missing or out of range."""
        idx = self.axis_map.get(name)
        if idx is None:
            return default
        try:
            return float(msg.axes[idx])
        except (IndexError, TypeError, ValueError):
            return default

    def event_handler(self, event):
        basename = os.path.basename(event.pathname)

        # Check if the event concerns js0 with debouncing
        if basename == "js0":
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Only check joystick changes every few seconds to prevent rapid cycling
            if current_time - self.last_joystick_check < self.joystick_check_interval:
                return
                
            self.last_joystick_check = current_time
            new_name = self._get_joystick_name()
            
            # Only update if we have a meaningful change
            if new_name != "Unknown" and new_name != self.joystick_name:
                self.is_initialized = False
                self.get_logger().info(f"Joystick changed: {self.joystick_name} → {new_name}")
                self.joystick_name = new_name
                self.last_known_good_name = new_name
                self.set_joystick_mapper(new_name)
            elif new_name == "Unknown" and self.last_known_good_name and self.last_known_good_name != self.joystick_name:
                # Fall back to last known good joystick if current detection fails
                self.get_logger().info(f"Joystick detection failed, using last known: {self.last_known_good_name}")
                self.joystick_name = self.last_known_good_name
                self.set_joystick_mapper(self.last_known_good_name)

    def joy_callback(self, msg: Joy):

        # During transitions some timing error can occur where the params are not yet loaded and the cb is called
        if not self.is_initialized:
            return

        twist_msg = Twist()
        twist_msg.linear.x = self._get_axis(msg, "linear_speed")
        twist_msg.angular.z = self._get_axis(msg, "rotation_speed")
        self.cmd_vel_pub.publish(twist_msg)

        aux_msg = MttAuxCommand()
        aux_msg.dead_man_switch = self._get_button(msg, "nav_safety")
        aux_msg.brake = abs(self._get_axis(msg, "brake"))
        
        # Winch safety button
        aux_msg.winch_safety_button = self._get_button(msg, "winch_safety")
         
        # Winch command from D-pad
        dpad_val = self._get_axis(msg, "winch")
        if dpad_val > 0.5:
            aux_msg.winch_command = MttAuxCommand.WINCH_IN
        elif dpad_val < -0.5:
            aux_msg.winch_command = MttAuxCommand.WINCH_OUT
        else:
            aux_msg.winch_command = MttAuxCommand.WINCH_NEUTRAL

        # Light toggle logic (rising edge detection)
        light_btn = self._get_button(msg, "light_toggle")
        if light_btn == 1 and self.prev_light_btn == 0:
            self.light_state = not self.light_state
        self.prev_light_btn = light_btn

        # Set light state if supported
        if hasattr(aux_msg, "light_state"):
            aux_msg.light_state = self.light_state

        self.aux_cmd_pub.publish(aux_msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = MTTTeleopJoy()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
