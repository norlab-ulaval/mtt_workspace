import can
from enum import Enum
import logging
import time
import socket
import fcntl
import struct

log = logging.getLogger('MTTCanDriver')
log.setLevel(logging.INFO)
handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
if not log.handlers:
    log.addHandler(handler)

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
    WinchNeutral = 0x7f  # 127
    WinchIn = 0xe5       # 229
    WinchOut = 0x18      # 24

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

class MTTCanDriver:
    """Non-ROS driver for controlling the MTT-154 via CAN bus.
    
    Provides low-level CAN communication interface for the MTT-154 vehicle.
    Handles throttle, steering, brake, winch, and safety system control.
    """

    def __init__(self, can_interface='can0'):
        # CAN node configuration
        self.node_id = 0x100  # rf remote controller id 0x001 and software controller id 0x100
        self.can_array = [0] * 8
        
        # CAN bus initialization with interface checking
        timeout_seconds = 10
        check_interval = 0.5
        elapsed = 0.0

        log.info(f"Waiting for CAN interface '{can_interface}'...")

        while not interface_exists(can_interface) and elapsed < timeout_seconds:
            time.sleep(check_interval)
            elapsed += check_interval

        if not interface_exists(can_interface):
            log.error(f"CAN interface '{can_interface}' not found after {timeout_seconds} seconds.")
            raise Exception(f"CAN interface '{can_interface}' not available")

        try:
            self.bus = can.interface.Bus(can_interface, bustype="socketcan")
            log.info(f"Successfully initialized CAN bus on '{can_interface}'")
        except OSError as e:
            log.error(f"Failed to initialize CAN bus: {e}")
            raise

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
        self.set_steer(128) # Center position

        # MTT_SWITCHES_DIRECTION_MODE
        self.set_direction_mode(DirectionMode.OpenLoop)
        # self.set_direction_mode(DirectionMode.CloseLoop)

    def reset_motion_commands(self):
        """Resets all motion commands to a safe, stopped state."""
        self.set_throttle(0)
        self.set_steer(128)  # Center position
        self.set_brake(255)  # Full brake
        self.set_winch_state(WinchState.WinchNeutral)
        self.set_direction(DirectionState.Forward)

    def send_can_frame(self):
        """Sends the current command frame to the CAN bus."""
        if not self.bus: 
            return
        try:
            message = can.Message(arbitration_id=self.node_id, data=self.can_array, is_extended_id=False)
            self.bus.send(message)
        except can.CanError as e:
            log.error(f"Error sending CAN frame: {e}")

    # --- Control Methods ---
    def set_steer(self, steer_value):

        if not isinstance(steer_value, int):
            log.error(f"steer_value is not an integer: {steer_value}")
            return

        # out of bound values are set to the closest bound
        # TODO: ignore out of bound values instead
        if steer_value > 255:
            log.warning(f"out of bound value {steer_value} for steer_value")
            steer_value = 255

        if steer_value < 0:
            log.warning(f"out of bound value {steer_value} for steer_value")
            steer_value = 0

        if steer_value >= 0 and steer_value <= 255:
            self.steer_value = steer_value
            self.can_array[MTT_ANALOG_STEER] = steer_value


    def set_throttle(self, throttle_value):

        if not isinstance(throttle_value, int):
            log.error(f"throttle_value is not an integer: {throttle_value}")
            return

        # TODO: ignore out of bound values instead
        if throttle_value > 230:
            log.warning(f"out of bound value {throttle_value} for throttle_value")
            throttle_value = 230

        if throttle_value < 0:
            log.warning(f"out of bound value {throttle_value} for throttle_value")
            throttle_value = 0

        if throttle_value >= 0 and throttle_value <= 230:
            self.throttle_value = throttle_value
            self.can_array[MTT_ANALOG_THROTTLE] = throttle_value


    def set_brake(self, brake_value):
        
        if not isinstance(brake_value, int):
            log.error(f"brake_value is not an integer: {brake_value}")
            return
            
        if brake_value > 255:
            log.warning(f"out of bound value {brake_value} for brake_value")
            brake_value = 255

        if brake_value < 0:
            log.warning(f"out of bound value {brake_value} for brake_value")
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
            log.error(f"invalid value for winch_state: {winch_state}")
            return
        
    def set_security_switch(self, switch_value):

        if switch_value == SecuritySwitchState.SafetyLocked:
            self.can_array[MTT_SWITCHES_GLOBAL] &= 0b11110111
            self.security_switch_state = SecuritySwitchState.SafetyLocked

        elif switch_value == SecuritySwitchState.SafetyUnlocked:
            self.can_array[MTT_SWITCHES_GLOBAL] |= 0b00001000
            self.security_switch_state = SecuritySwitchState.SafetyUnlocked

        else:
            log.error(f"invalid value for switch_value: {switch_value}")
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
            log.error(f"invalid value for direction: {direction}")
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
            log.error(f"invalid value for light_state: {light_state}")
            return


    def set_direction_mode(self, direction_mode):

        if direction_mode == DirectionMode.CloseLoop:
            self.can_array[MTT_SWITCHES_DIRECTION_MODE] |= 0b00000001
            self.direction_mode = DirectionMode.CloseLoop

        elif direction_mode == DirectionMode.OpenLoop:
            self.can_array[MTT_SWITCHES_DIRECTION_MODE] &= 0b11111110
            self.direction_mode = DirectionMode.OpenLoop
        else:
            log.error(f"invalid value for direction_mode: {direction_mode}")
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
            log.error(f"invalid value for vehicle_type: {vehicle_type}")
            return

    def shutdown(self):
        if self.bus: 
            self.bus.shutdown()
