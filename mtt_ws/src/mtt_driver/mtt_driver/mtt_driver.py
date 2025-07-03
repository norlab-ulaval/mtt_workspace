#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import can
from enum import Enum
import time
import socket
import fcntl
import struct


MTT_SWITCHES_VEHICLE_TYPE = 0
MTT_SWITCHES_GLOBAL = 1
MTT_ANALOG_THROTTLE = 2
MTT_ANALOG_WINCH = 3
MTT_ANALOG_BRAKE = 4
MTT_ANALOG_STEER = 5
MTT_SWITCHES_DIRECTION_MODE = 6

class DirectionMode(Enum):
    OpenLoop = 0
    CloseLoop = 1

class DirectionState(Enum):
    Reverse = 0x00
    Forward = 0x01

class WinchState(Enum):
    WinchNeutral = 0x7f # 127
    WinchIn = 0xe5 # 229
    WinchOut = 0x18 # 24

class SecuritySwitchState(Enum):
    SafetyLocked = 0x00
    SafetyUnlocked = 0x01

class VehicleType(Enum):
    VehicleSingleTrack = 0x00
    VehicleSbsLeft = 0x01
    VehicleSbsRight = 0x02

class LightState(Enum):
    Off = 0x00
    On = 0x01

def interface_exists(ifname):
    """Check if a network interface exists (e.g. 'can0')."""
    SIOCGIFINDEX = 0x8933
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        fcntl.ioctl(s.fileno(), SIOCGIFINDEX, struct.pack('256s', ifname.encode('utf-8')[:15]))
        return True
    except OSError:
        return False
    
class MttDriver(Node):
    def __init__(self):
        super().__init__("mtt_driver")

        # to validate
        self.node_id = 0x001 # remote controller id
        # self.node_id = 0x100 # is supposed to work according to documentation but it doesn't

        # can bus initialization
        interface_name = "can0"
        timeout_seconds = 10
        check_interval = 0.5
        elapsed = 0.0

        self.get_logger().info(f"Waiting for CAN interface '{interface_name}'...")

        while not interface_exists(interface_name) and elapsed < timeout_seconds:
            time.sleep(check_interval)
            elapsed += check_interval

        if not interface_exists(interface_name):
            self.get_logger().error(f"CAN interface '{interface_name}' not found after {timeout_seconds} seconds.")
            rclpy.shutdown()
            return

        try:
            self.bus = can.interface.Bus(interface_name, bustype="socketcan")
            self.get_logger().info(f"Successfully initialized CAN bus on '{interface_name}'")
        except OSError as e:
            self.get_logger().error(f"Failed to initialize CAN bus: {e}")
            rclpy.shutdown()


        self.can_array = [0] * 8

        # initializing the default values for the configuration of the mtt
        self.vehicle_type = None
        self.steer_value = None
        self.throttle_value = None
        self.brake_value = None
        self.winch_state = None
        self.security_switch_state = None
        self.direction_state = None
        self.direction_mode = None
        self.light_state = None

        # can_array = [MTT_SWITCHES_VEHICLE_TYPE = 0
        # MTT_SWITCHES_GLOBAL = 1
        # MTT_ANALOG_THROTTLE = 2
        # MTT_ANALOG_WINCH = 3
        # MTT_ANALOG_BRAKE = 4
        # MTT_ANALOG_STEER = 5
        # MTT_SWITCHES_DIRECTION_MODE = 6]

        # MTT_SWITCHES_VEHICLE_TYPE
        self.set_vehicle_type(VehicleType.VehicleSingleTrack)

        # MTT_SWITCHES_GLOBAL
        self.set_security_switch(SecuritySwitchState.SafetyLocked)
        # self.set_security_switch(SecuritySwitchState.SafetyUnlocked)

        self.set_direction(DirectionState.Forward)
        # self.set_direction(DirectionState.Reverse)
        
        self.set_light_state(LightState.On)
        # self.set_light_state(LightState.On)

        # MTT_ANALOG_THROTTLE
        self.set_throttle(0)
        # self.set_throttle(60)

        # MTT_ANALOG_WINCH
        self.set_winch_state(WinchState.WinchNeutral)

        # MTT_ANALOG_BRAKE
        self.set_brake(0)

        # MTT_ANALOG_STEER
        # self.set_steer(0)
        self.set_steer(128)

        # MTT_SWITCHES_DIRECTION_MODE
        self.set_direction_mode(DirectionMode.OpenLoop)
        # self.set_direction_mode(DirectionMode.CloseLoop)

        self.send_frame_period = 0.1
        self.count = 0

        self.timer = self.create_timer(self.send_frame_period, self.send_can_frame)


    def __del__(self):
        if hasattr(self, "bus") and self.bus:
            self.bus.shutdown()


        # TODO: iterative error 


    def set_steer(self, steer_value):

        if not isinstance(steer_value, int):
            print("ERROR: steer_value is not an integer: " + str(steer_value))
            return

        # out of bound values are set to the closest bound
        # TODO: ignore out of bound values instead
        if steer_value > 255:
            print("WARNING: out of bound value " + str(steer_value) + " for steer_value")
            steer_value = 255

        if steer_value < 0:
            print("WARNING: out of bound value " + str(steer_value) + " for steer_value")
            steer_value = 0

        if steer_value >= 0 and steer_value <= 255:  
            self.steer_value = steer_value
            self.can_array[MTT_ANALOG_STEER] = steer_value


    def set_throttle(self, throttle_value):

        if not isinstance(throttle_value, int):
            print("ERROR: throttle_value is not an integer: " + str(throttle_value))
            return

        # TODO: ignore out of bound values instead
        if throttle_value > 230:
            print("WARNING: out of bound value " + str(throttle_value) + " for throttle_value")
            throttle_value = 230

        if throttle_value < 0:
            print("WARNING: out of bound value " + str(throttle_value) + " for throttle_value")
            throttle_value = 0

        if throttle_value >= 0 and throttle_value <= 230:
            self.throttle_value = throttle_value  
            self.can_array[MTT_ANALOG_THROTTLE] = throttle_value


    def set_brake(self, brake_value):
        
        if not isinstance(brake_value, int):
            print("ERROR: brake_value is not an integer: " + str(brake_value))
            return
            
        if brake_value > 255:
            print("WARNING: out of bound value " + str(brake_value) + " for brake_value")
            brake_value = 255

        if brake_value < 0:
            print("WARNING: out of bound value " + str(brake_value) + " for brake_value")
            brake_value = 0

        if brake_value >= 0 and brake_value <= 255:
            self.brake_value = brake_value
            self.can_array[MTT_ANALOG_BRAKE] = brake_value


    def set_winch_state(self, winch_state):

        if winch_state == WinchState.WinchNeutral:
            self.can_array[MTT_ANALOG_WINCH] = winch_state.value
            self.winch_state = WinchState.WinchNeutral

        elif winch_state == WinchState.WinchIn:
            self.can_array[MTT_ANALOG_WINCH] = winch_state.value
            self.winch_state = WinchState.WinchIn
             
        elif winch_state == WinchState.WinchOut:
            self.can_array[MTT_ANALOG_WINCH] = winch_state.value
            self.winch_state = WinchState.WinchOut

        else:
            print("ERROR: invalid value for winch_state: " + str(winch_state))
            return 


    def set_security_switch(self, switch_value):

        if switch_value == SecuritySwitchState.SafetyLocked:
            self.can_array[MTT_SWITCHES_GLOBAL] &= 0b11110111
            self.security_switch_state = SecuritySwitchState.SafetyLocked

        elif switch_value == SecuritySwitchState.SafetyUnlocked:
            self.can_array[MTT_SWITCHES_GLOBAL] |= 0b00001000
            self.security_switch_state = SecuritySwitchState.SafetyUnlocked

        else:
            print("ERROR: invalid value for switch_value: " + str(switch_value))
            return


    def set_direction(self, direction):

        # semble être inversé forward et reverse
        if direction == DirectionState.Forward:
            self.can_array[MTT_SWITCHES_GLOBAL] &= 0b11011111
            self.direction_state = DirectionState.Forward

        elif direction == DirectionState.Reverse:
            self.can_array[MTT_SWITCHES_GLOBAL] |= 0b00100000
            self.direction_state = DirectionState.Reverse

        else:
            print("ERROR: invalid value for direction: " + str(direction))
            return


    def set_light_state(self, light_state):

        # Semble être inversé on et off
        if light_state == LightState.Off:
            self.light_state = LightState.Off
            self.can_array[MTT_SWITCHES_GLOBAL] |= 0b01000000

        elif light_state == LightState.On:
            self.light_state = LightState.On
            self.can_array[MTT_SWITCHES_GLOBAL] &= 0b10111111
        else:
            print("ERROR: invalid value for light_state: " + str(light_state))
            return


    def set_direction_mode(self, direction_mode):

        if direction_mode == DirectionMode.CloseLoop:
            self.can_array[MTT_SWITCHES_DIRECTION_MODE] |= 0b00000001
            self.direction_mode = DirectionMode.CloseLoop

        elif direction_mode == DirectionMode.OpenLoop:
            self.can_array[MTT_SWITCHES_DIRECTION_MODE] &= 0b11111110
            self.direction_mode = DirectionMode.OpenLoop
        else:
            print("ERROR: invalid value for direction_mode: " + str(direction_mode))
            return


    def set_vehicle_type(self, vehicle_type):

        if vehicle_type == VehicleType.VehicleSingleTrack:
            self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
            self.vehicle_type = VehicleType.VehicleSingleTrack

        elif vehicle_type == VehicleType.VehicleSbsLeft:
            self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
            self.vehicle_type = VehicleType.VehicleSbsLeft
             
        elif vehicle_type == VehicleType.VehicleSbsRight:
            self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
            self.vehicle_type = VehicleType.VehicleSbsRight

        else:
            print("ERROR: invalid value for vehicle_type: " + str(vehicle_type))
            return


    def send_can_frame(self):
        
        # only for testing purpose
        self.count += self.send_frame_period

        if self.count >= 5:
            if self.light_state == LightState.On:
                new_state = LightState.Off
            else:
                new_state = LightState.On

            self.set_light_state(new_state)
            self.count = 0

        print(self.can_array)

        self.bus.send(can.Message(arbitration_id=self.node_id, data=self.can_array, is_extended_id=False))



def main(args=None):

    rclpy.init(args=args)
    mtt_driver = MttDriver()
    rclpy.spin(mtt_driver)

    rclpy.shutdown()


if __name__ == '__main__':
    main()