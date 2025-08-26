#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from rcl_interfaces.msg import Log
from geometry_msgs.msg import Twist
from enum import Enum


class JoystickMode(Enum):
    LogitechDualAction = 0
    LogitechGamepadF310 = 1


class JoyMapper(Node):
    def __init__(self):
        super().__init__('mtt_joy_mapper')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rosout_sub = self.create_subscription(Log, '/rosout', self.log_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # float numbers are rarely exactly equal to 0
        self.float_number_threshold = 1e-6

        self.current_mode = None
        self.is_safety_locked = True

        joystick_name = self.get_initial_joystick_name("/dev/input/js0")
        
        self.get_logger().info(f"Detected joystick: {joystick_name}")
        self.set_joystick_controller(joystick_name)

        self.wheel_speed_multiplier = 1
        self.last_wheel_speed_multiplier_command = 0
        self.yaw_speed_multiplier = 1
        self.last_yaw_speed_multiplier_command = 0
        self.wheel_speed_threshold = 0.2
        self.yaw_speed_threshold = 0.2




    def get_initial_joystick_name(self, js_dev="/dev/input/js0"):

        try:
            with open(f"/sys/class/input/{js_dev.split('/')[-1]}/device/name") as f:
                return f.read().strip()
        except Exception as e:
            self.get_logger().error(f"Could not read joystick name: {e}")
            return "Unknown"


    def set_joystick_controller(self, received_string):

        if "Logitech Dual Action" in received_string:
            self.current_mode = JoystickMode.LogitechDualAction

        elif "Logitech Gamepad F310" in received_string:
            self.current_mode = JoystickMode.LogitechGamepadF310

        else:
            self.get_logger().warn("Unsupported Joystick controller detected")
            self.current_mode = None

        self.is_safety_locked = True


    def log_callback(self, msg: Log):
        # Look for joy_node "Opened joystick" message
        if msg.name.endswith('joy_node') and 'Opened joystick' in msg.msg:
            self.get_logger().warn(f"Joystick remap detected: {msg.msg}")
            self.set_joystick_controller(msg.msg)
            

    def joy_callback(self, msg: Joy):

        self.check_safety(msg)

        if self.current_mode == None:
            return
        
        if self.is_safety_locked:
            return
        
        if not self.message_match_mode_verification(msg):
            return
        
        # the length of msg.buttons changes depending on the controller's mode
        if self.current_mode == JoystickMode.LogitechDualAction:
            self.logitech_dual_action_mapping(msg)
        else:
            self.get_logger().warn("No mapping function for input - unsupported Joystick controller")


    def check_safety(self, msg: Joy):

        # TODO: some controllers do have some default values that are effectively 1 at rest

        # When switching mode, some joystick values are received as 1 when they are at rest which can be dangerous
        if self.is_safety_locked:


            is_axe_at_rest_list = [False] * len(msg.axes)
            is_button_at_rest_list = [False] * len(msg.buttons)

            for index, axe in enumerate(msg.axes):
                if axe < self.float_number_threshold and axe > -self.float_number_threshold:
                    is_axe_at_rest_list[index] = True

            for index, button in enumerate(msg.buttons):
                if button < self.float_number_threshold and button > -self.float_number_threshold:
                    is_button_at_rest_list[index] = True

            if all(is_axe_at_rest_list) and all(is_button_at_rest_list):
                self.is_safety_locked = False


    def message_match_mode_verification(self, msg: Joy):

        if self.current_mode == JoystickMode.LogitechDualAction:
            axe_array_length = 6
            button_array_length = 12
        elif self.current_mode == JoystickMode.LogitechGamepadF310:
            axe_array_length = 8
            button_array_length = 11
        else:
            self.get_logger().warn("No implementation for Joy message verification for current joystick controller")
            return False
        

        if len(msg.axes) == axe_array_length and len(msg.buttons) == button_array_length:
            return True
        else:
            self.get_logger().warn("Mismatch array size for Joy message verification for current joystick controller")
            return False



    def logitech_dual_action_mapping(self, msg: Joy):

        # print("####")
        # print("joy_cb")

        # Left joystick X axis (horizontal)
        yaw_speed = msg.axes[0] 

        # Left joystick Y axis (vertical)
        wheel_speed = msg.axes[1]

        # Right joystick X axis (horizontal)
        msg.axes[2] 

        # Right joystick Y axis (vertical)
        msg.axes[3]

        # D-pad (arrows) x axis (horizontal)
        msg.axes[4]

        # D-pad (arrows) y axis (vertical)
        winch = msg.axes[5]

        # X
        msg.buttons[0]

        # A
        nav_deadman_switch = msg.buttons[1]

        # B
        winch_deadman_switch = msg.buttons[2]

        # Y
        toggle_light = msg.buttons[3]

        # LB
        decrease_wheel_speed = msg.buttons[4]

        # RB
        increase_wheel_speed = msg.buttons[5]

        # LT
        decrease_yaw_speed = msg.buttons[6]

        # RT
        increase_yaw_speed = msg.buttons[7]

        # Back
        msg.buttons[8]

        # Start
        msg.buttons[9]

        # appears but haven't found it
        msg.buttons[10]

        # appears but haven't found it
        msg.buttons[11]

        # The following part could eventually be a function on its own

        # ignoring small values to prevent unwillingly moving the robot 
        # in a direction due joystick not being perfectly oriented on an axis
        if abs(wheel_speed) < self.wheel_speed_threshold:
            wheel_speed = 0.0

        if abs(yaw_speed) < self.yaw_speed_threshold:
            yaw_speed = 0.0


        cmd_vel_msg = Twist()

        if nav_deadman_switch:

            cmd_vel_msg.linear.x = wheel_speed * self.wheel_speed_multiplier
            cmd_vel_msg.angular.z = yaw_speed * self.yaw_speed_multiplier

        else:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0


        self.cmd_vel_publisher.publish(cmd_vel_msg)
        

        if winch_deadman_switch:
            if winch > 1 - self.float_number_threshold:
                pass
                # winch out

            elif winch < -1 + self.float_number_threshold:
                pass
                # winch in

        

        if increase_wheel_speed or decrease_wheel_speed:
            if self.last_wheel_speed_multiplier_command == 0:
                self.last_wheel_speed_multiplier_command = 1

                if increase_wheel_speed:
                    self.wheel_speed_multiplier +=1
                elif decrease_wheel_speed:
                    self.wheel_speed_multiplier -=1
        else:
            self.last_wheel_speed_multiplier_command = 0


        if increase_yaw_speed or decrease_yaw_speed:
            if self.last_yaw_speed_multiplier_command == 0:
                self.last_yaw_speed_multiplier_command = 1

                if increase_yaw_speed:
                    self.yaw_speed_multiplier +=1
                elif decrease_yaw_speed:
                    self.yaw_speed_multiplier -=1
        else:
            self.last_yaw_speed_multiplier_command = 0


        if self.wheel_speed_multiplier < 1:
            self.wheel_speed_multiplier = 1

        if self.yaw_speed_multiplier < 1:
            self.yaw_speed_multiplier = 1

        if self.wheel_speed_multiplier > 10:
            self.wheel_speed_multiplier = 10

        if self.yaw_speed_multiplier > 10:
            self.yaw_speed_multiplier = 10






def main():
    rclpy.init()
    node = JoyMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()