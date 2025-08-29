#!/usr/bin/env python3
"""
MTT-154 CAN Driver

Pure Python interface for controlling the MTT-154 via CAN bus.
Implements CANBus_Specification.md v1.1 (2025-07-03).

This module provides low-level CAN communication without ROS dependencies.
"""

import can
from enum import Enum
import logging
import time
import socket
import fcntl
import struct
import threading
from dataclasses import dataclass


log = logging.getLogger('MTTCanDriver')
# TODO: set logging as parameter through launch
log.setLevel(logging.INFO)
handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
if not log.handlers:
    log.addHandler(handler)


# MTT-154 Encoder and Odometry Constants
# Reference: CANBus_Specification.md Section 5.1 - Gearing Constants
MTT_GEAR1 = 16
MTT_GEAR2 = 36
MTT_GEAR3 = 15
MTT_GEAR4 = 32
MTT_GEAR_DRIVE = 8
MTT_GEAR_TRACK = 54
# Option A fix for 2x overscale: double encoder teeth from 5 -> 10 so FinalRatio doubles (324 -> 648)
MTT_ENCODER_TEETH = 10  # was 5
MTT_TRACK_LENGTH_CM = 393
MTT_TRACK_LENGTH_KM = MTT_TRACK_LENGTH_CM / 100000.0  # Firmware: MTT_TRACK_LENGTH_KM = MTT_TRACK_LENGTH_CM / 100000
MTT_TRACK_LENGTH_M = MTT_TRACK_LENGTH_CM / 100.0  # For convenience in Python

# CAN IDs - Reference: CANBus_Specification.md Section 3.0
CAN_MAIN_TELEMETRY = 0x2FF  # Main controller data (encoder, temperature)

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
    Forward = 0x00
    Reverse = 0x01

class WinchState(Enum):
    WinchNeutral = 0x7F  # 127 - matches RF remote idle frame
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


@dataclass
class TachometerData:
    """Data structure for tachometer information from CAN 0x2FF
    
    Reference: CANBus_Specification.md Section 3.2.1 - MSB-first byte order
    """
    main_sensor_temp_a: float = 0.0  # Temperature sensor A in °C
    main_sensor_temp_b: float = 0.0  # Temperature sensor B in °C
    tachometer_instant: int = 0      # Instantaneous speed in raw encoder ticks per second (RPS)
    tachometer_cumulative: int = 0   # Cumulative distance in raw encoder ticks
    timestamp: float = 0.0           # Time when data was received
    new_data_available: bool = False # Flag indicating fresh data


    def get_speed_ms(self, final_ratio: float) -> float:
        """Calculate speed in m/s from encoder data (firmware-compatible)"""
        if final_ratio == 0.0:
            return 0.0
        # Firmware RPS_to_KMh: return ((float)RPS / FinalRatio) * (float)MTT_TRACK_LENGTH_KM *3600;
        # Convert to m/s: ((RPS / FinalRatio) * MTT_TRACK_LENGTH_KM * 3600) / 3.6
        speed_kmh = (self.tachometer_instant / final_ratio) * MTT_TRACK_LENGTH_KM * 3600
        speed_ms = speed_kmh / 3.6
        return speed_ms


    def get_speed_kmh(self, final_ratio: float) -> float:
        """Calculate speed in km/h from encoder data (exact firmware implementation)"""
        if final_ratio == 0.0:
            return 0.0
        # Firmware RPS_to_KMh: return ((float)RPS / FinalRatio) * (float)MTT_TRACK_LENGTH_KM *3600;
        return (self.tachometer_instant / final_ratio) * MTT_TRACK_LENGTH_KM * 3600


    def __str__(self):
        return (f"TachometerData(TempA={self.main_sensor_temp_a}°C, TempB={self.main_sensor_temp_b}°C, "
                f"Speed={self.tachometer_instant} RPS, Cumulative={self.tachometer_cumulative}, "
                f"Timestamp={self.timestamp:.3f}, NewData={self.new_data_available})")



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
    
    Implements CANBus_Specification.md v1.1 for complete vehicle control.
    
    CAN Communication:
    - 0x001: Joystick/remote controller
    - 0x100: Auxiliary control (this driver) - overrides 0x001 when active
    - 0x2ff: Tachometer data (receive only)
    
    Safety Requirements:
    - Security switch (bit 7) must be unlocked for operation
    - Light state acts as emergency stop (temporary firmware behavior)
    - Direction is controlled by master system
    """

    def __init__(self, can_interface='can0'):

        self.node_id = 0x001 # remote controller id
        # self.node_id = 0x100 # is supposed to work according to documentation but it doesn't
        
        # can bus initialization
        self.can_interface = can_interface
        self.send_frame_period = 0.05
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


        self.frame_lock = threading.Lock()

        # TODO: Initialize CAN frame with safe starting values
        # Template frame for reference - not used in actual operation
        # can_array = [MTT_SWITCHES_VEHICLE_TYPE = 0
        # MTT_SWITCHES_GLOBAL = 1
        # MTT_ANALOG_THROTTLE = 2
        # MTT_ANALOG_WINCH = 3
        # MTT_ANALOG_BRAKE = 4
        # MTT_ANALOG_STEER = 5
        # MTT_SWITCHES_DIRECTION_MODE = 6]
        self.can_array = [0] * 8

        # Initialize state variables
        self.vehicle_type = None
        self.steer_value = None
        self.throttle_value = None
        self.brake_value = None
        self.winch_state = None
        self.security_switch_state = None
        self.direction_state = None
        self.direction_mode = None
        self.light_state = None
        
        self.set_vehicle_type(VehicleType.VehicleSingleTrack)
        self.set_security_switch(SecuritySwitchState.SafetyLocked)
        self.set_direction(DirectionState.Forward)
        self.set_light_state(LightState.Off)
        self.set_throttle(0)
        self.set_winch_state(WinchState.WinchNeutral)
        self.set_brake(0)
        self.set_steer(127)
        self.set_direction_mode(DirectionMode.OpenLoop)

        # Odometry and tachometer data
        self.tachometer_data = TachometerData()
        self.encoder_final_ratio = self._calculate_gear_ratio()
        self.last_cumulative_ticks = 0
        self.total_distance = 0.0
        self.current_direction = DirectionState.Forward
        
        # Start CAN threads
        self.can_listener_thread = threading.Thread(target=self._can_listener, daemon=True)
        self.can_listener_running = True
        self.can_listener_thread.start()
        
        self.sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self.sender_running = True
        self.sender_thread.start()


    def __del__(self):
        self.cleanup()

    def reset_motion_commands(self):
        """Resets all motion commands to a safe, stopped state."""
        self.set_throttle(0)
        self.set_steer(127)  # Center position
        self.set_brake(255)  # Full brake
        self.set_direction(DirectionState.Forward)
        self.set_winch_state(WinchState.WinchNeutral)


    def _calculate_gear_ratio(self):
        """Calculate the final gear ratio for speed conversion (exact firmware implementation)"""
        # Firmware RPS_to_KMh_Precalc() function:
        # ratio1 = ((float)MTT_GEAR2 / (float)MTT_GEAR1) * (float)MTT_Encoder_TEET;
        # ratio2 = ((float)MTT_GEAR4 / (float)MTT_GEAR3) * ratio1;
        # FinalRatio = (((float)MTT_GEAR_TRACK / (float)MTT_GEAR_DRIVE) * ratio2) * 2;
        
        # NOTE: This ratio is used for SPEED calculation only (RPS to km/h)
        # For DISTANCE calculation, we use empirical measurement: 15160 ticks = 1 track turn = 3.93m
        
        ratio1 = (MTT_GEAR2 / MTT_GEAR1) * MTT_ENCODER_TEETH
        ratio2 = (MTT_GEAR4 / MTT_GEAR3) * ratio1
        final_ratio = ((MTT_GEAR_TRACK / MTT_GEAR_DRIVE) * ratio2) * 2
        
        log.info(f"Encoder final ratio calculated: {final_ratio} (firmware-exact calculation, used for speed only)")
        return final_ratio


    def _can_listener(self):
        """CAN listener thread for tachometer data (0x2FF frames)"""
        while self.can_listener_running:
            try:
                message = self.bus.recv(timeout=0.1)
                if message and message.arbitration_id == CAN_MAIN_TELEMETRY:
                    self._process_tachometer_data(message.data)
            except Exception as e:
                if self.can_listener_running:  # Only log if we're still supposed to be running
                    log.debug(f"CAN listener error: {e}")
                    time.sleep(0.1)


    def _process_tachometer_data(self, data):
        """Process tachometer data from 0x2FF frame (MSB-first byte order)"""
        if len(data) != 8:
            return

        # Parse data according to specification (MSB-first byte order)
        # Bytes 0-1: Temperature sensors (signed bytes)
        temp_a = struct.unpack('b', data[0:1])[0]  # signed byte
        temp_b = struct.unpack('b', data[1:2])[0]  # signed byte
        
        # Bytes 2-3: Instantaneous tachometer (RPS) - MSB first (Big Endian)
        tachimeter_instant = struct.unpack('>H', data[2:4])[0]  # big-endian uint16
        
        # Bytes 4-7: Cumulative RPS (distance in ticks) - MSB first (Big Endian)
        # Firmware: RxData_3[4]<<24 | RxData_3[5]<<16 | RxData_3[6]<<8 | RxData_3[7]
        tachimeter_cumulative = struct.unpack('>I', data[4:8])[0]  # big-endian uint32

        # Update tachometer data structure
        self.tachometer_data.main_sensor_temp_a = float(temp_a)
        self.tachometer_data.main_sensor_temp_b = float(temp_b)
        self.tachometer_data.tachometer_instant = tachimeter_instant  # Fixed typo here
        self.tachometer_data.tachometer_cumulative = tachimeter_cumulative
        self.tachometer_data.timestamp = time.time()
        self.tachometer_data.new_data_available = True


        # Calculate distance traveled since last reading
        if self.last_cumulative_ticks > 0:
            tick_diff = tachimeter_cumulative - self.last_cumulative_ticks
            if tick_diff > 0:  # Avoid negative values due to overflow
                # Distance calculation: ticks * (track_length / gear_ratio)
                # The firmware gear ratio includes the *2 factor for speed (RPS to km/h conversion)
                # For distance, we need to use the gear ratio without the *2 factor
                gear_ratio_for_distance = self.encoder_final_ratio / 2
                distance_increment = (tick_diff / gear_ratio_for_distance) * MTT_TRACK_LENGTH_M
                
                # Odometry always accumulates positive distance regardless of direction
                # Direction is only used for speed calculation, not odometry
                self.total_distance += distance_increment

        self.last_cumulative_ticks = tachimeter_cumulative

        # Log periodically (every 50 messages to avoid spam)
        if tachimeter_cumulative % 50 == 0:
            speed_ms = self.get_current_speed_ms()
            log.info(
                f"Tachometer data - Speed: {speed_ms:.2f} m/s, "
                f"Cumulative: {tachimeter_cumulative}, "
                f"Temp A: {temp_a}°C, Temp B: {temp_b}°C"
            )


    def emergency_stop(self):
        """Emergency stop - sets all motion controls to safe values"""
        self.set_throttle(0)
        self.set_brake(255)
        self.set_steer(127)
        self.set_winch_state(WinchState.WinchNeutral)
        log.warning("EMERGENCY STOP ACTIVATED")


    def is_system_ready(self):
        """Check if system is ready for operation"""
        if self.security_switch_state != SecuritySwitchState.SafetyUnlocked:
            log.warning("System not ready: Security switch is locked")
            return False
        
        if not hasattr(self, 'bus') or not self.bus:
            log.warning("System not ready: CAN bus not initialized")
            return False
            
        return True

    
    def get_direction_for_calculation(self):
        """Get direction for distance/speed calculations"""
        # Since tachometer doesn't provide direction, we use our control direction
        return 1 if self.direction_state == DirectionState.Forward else -1
    

    def get_current_speed_ms(self) -> float:
        """Get current speed in m/s, considering direction"""
        if not self.tachometer_data.new_data_available:
            return 0.0
        
        speed = self.tachometer_data.get_speed_ms(self.encoder_final_ratio)
        
        # Apply direction (negative for reverse)
        if self.current_direction == DirectionState.Reverse:
            speed = -speed
        
        return speed
    

    def get_current_speed_kmh(self) -> float:
        """Get current speed in km/h, considering direction"""
        if not self.tachometer_data.new_data_available:
            return 0.0
        
        speed = self.tachometer_data.get_speed_kmh(self.encoder_final_ratio)
        
        # Apply direction (negative for reverse)
        if self.current_direction == DirectionState.Reverse:
            speed = -speed
        
        return speed
    

    def get_tachometer_data(self) -> TachometerData:
        """Get complete tachometer data structure"""
        return self.tachometer_data


    def get_odometry_data(self) -> dict:
        """Get complete odometry data dictionary"""
        # TODO: Add steering angle data for higher-level kinematic models
        # Single track (VehicleSingleTrack): Reverse bicycle model
        # Side-by-side (VehicleSbsLeft/Right): Skid-steer drive model
        # Implementation will be done at ROS controller level using ros_controllers
        return {
            'speed_ms': self.get_current_speed_ms(),
            'speed_kmh': self.get_current_speed_kmh(),
            'cumulative_ticks': self.tachometer_data.tachometer_cumulative,
            'total_distance_m': self.total_distance,
            'timestamp': self.tachometer_data.timestamp,
            'temperature_a': self.tachometer_data.main_sensor_temp_a,
            'temperature_b': self.tachometer_data.main_sensor_temp_b,
            'direction': self.current_direction.name,
            'data_age_ms': (time.time() - self.tachometer_data.timestamp) * 1000 if self.tachometer_data.timestamp > 0 else 0,
            # TODO: Add steering_angle_rad and vehicle_type for kinematic models
        }
    

    # TODO check if fucntion is used - also, funciton should use the setters
    def set_deadman_switch(self, pressed=True):
        """Set deadman switch state"""
        with self.frame_lock:
            if pressed:
                self.can_array[MTT_SWITCHES_GLOBAL] |= 0b00001000  # Set bit 3
            else:
                self.can_array[MTT_SWITCHES_GLOBAL] &= 0b11110111  # Clear bit 3


    def set_steer(self, steer_value):
        """Set steering value (0-255, 127 is center)"""

        if not isinstance(steer_value, int):
            print("ERROR: steer_value is not an integer: " + str(steer_value))
            return False

        # out of bound values are ignored
        if steer_value > 255:
            print("WARNING: out of bound value " + str(steer_value) + " for steer_value")
            return False

        if steer_value < 0:
            print("WARNING: out of bound value " + str(steer_value) + " for steer_value")
            return False

        with self.frame_lock:
            self.steer_value = steer_value
            self.can_array[MTT_ANALOG_STEER] = steer_value

            return True


    def set_throttle(self, throttle_value):
        """Set throttle value (0-230)"""
        if not isinstance(throttle_value, int):
            log.error(f"throttle_value is not an integer: {throttle_value}")
            return


        # ignoring out of bounds values instead of truncking
        if throttle_value > 230:
            print("WARNING: out of bound value " + str(throttle_value) + " for throttle_value")
            return False

        if throttle_value < 0:
            print("WARNING: out of bound value " + str(throttle_value) + " for throttle_value")
            return False

        if throttle_value >= 0 and throttle_value <= 230:
            with self.frame_lock:
                self.throttle_value = throttle_value  
                self.can_array[MTT_ANALOG_THROTTLE] = throttle_value

            return True


    def set_brake(self, brake_value):
        """Set brake value (0-255)"""
        if not isinstance(brake_value, int):
            log.error(f"brake_value is not an integer: {brake_value}")
            return False
            
        # ignoring out of bounds values instead of truncking
        if brake_value > 255:
            print("WARNING: out of bound value " + str(brake_value) + " for brake_value")
            return False

        if brake_value < 0:
            print("WARNING: out of bound value " + str(brake_value) + " for brake_value")
            return False

        if brake_value >= 0 and brake_value <= 255:
            with self.frame_lock:
                self.brake_value = brake_value
                self.can_array[MTT_ANALOG_BRAKE] = brake_value

            return True


    def set_winch_state(self, winch_state):
        """Set winch state - holds the state until explicitly changed"""
        if winch_state == WinchState.WinchNeutral:
            with self.frame_lock:
                self.can_array[MTT_ANALOG_WINCH] = WinchState.WinchNeutral.value
                self.winch_state = WinchState.WinchNeutral
                    
        elif winch_state == WinchState.WinchIn:
            with self.frame_lock:
                self.can_array[MTT_ANALOG_WINCH] = WinchState.WinchIn.value
                self.winch_state = WinchState.WinchIn
                    
        elif winch_state == WinchState.WinchOut:
            with self.frame_lock:
                self.can_array[MTT_ANALOG_WINCH] = WinchState.WinchOut.value
                self.winch_state = WinchState.WinchOut
                    
        else:
            log.error(f"invalid value for winch_state: {winch_state}")
            return
    

    def set_security_switch(self, switch_value):
        """Set security switch state"""
        if switch_value == SecuritySwitchState.SafetyLocked:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] &= 0b01111111  # Clear bit 7
                self.security_switch_state = SecuritySwitchState.SafetyLocked

            return True

        elif switch_value == SecuritySwitchState.SafetyUnlocked:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] |= 0b10000000  # Set bit 7
                self.security_switch_state = SecuritySwitchState.SafetyUnlocked

            return True

        else:
            log.error(f"invalid value for switch_value: {switch_value}")
            return False
        

    def set_direction(self, direction):
        """Set vehicle direction"""
        if direction == DirectionState.Forward:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] |= 0b00100000  # Set bit 5
                self.direction_state = DirectionState.Forward
                self.current_direction = DirectionState.Forward

            return True

        elif direction == DirectionState.Reverse:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] &= 0b11011111  # Clear bit 5
                self.direction_state = DirectionState.Reverse
                self.current_direction = DirectionState.Reverse

            return True

        else:
            log.error(f"invalid value for direction: {direction}")
            return


    def set_light_state(self, light_state):
        """Set light state"""
        if light_state == LightState.Off:
            with self.frame_lock:
                self.light_state = LightState.Off
                self.can_array[MTT_SWITCHES_GLOBAL] |= 0b01000000  # Set bit 6

            return True

        elif light_state == LightState.On:
            with self.frame_lock:
                self.light_state = LightState.On
                self.can_array[MTT_SWITCHES_GLOBAL] &= 0b10111111  # Clear bit 6
        else:
            log.error(f"invalid value for light_state: {light_state}")
            return


    def set_direction_mode(self, direction_mode):
        """Set direction mode"""
        if direction_mode == DirectionMode.CloseLoop:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_DIRECTION_MODE] |= 0b00000001
                self.direction_mode = DirectionMode.CloseLoop

            return True

        elif direction_mode == DirectionMode.OpenLoop:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_DIRECTION_MODE] &= 0b11111110
                self.direction_mode = DirectionMode.OpenLoop
        else:
            log.error(f"invalid value for direction_mode: {direction_mode}")
            return


    def set_vehicle_type(self, vehicle_type):
        """Set vehicle type"""
        if vehicle_type == VehicleType.VehicleSingleTrack:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
                self.vehicle_type = VehicleType.VehicleSingleTrack

            return True

        elif vehicle_type == VehicleType.VehicleSbsLeft:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
                self.vehicle_type = VehicleType.VehicleSbsLeft
             
        elif vehicle_type == VehicleType.VehicleSbsRight:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
                self.vehicle_type = VehicleType.VehicleSbsRight

            return True

        else:
            log.error(f"invalid value for vehicle_type: {vehicle_type}")
            return False


    def cleanup(self):
        """Clean up resources before shutting down"""
        self.sender_running = False
        if hasattr(self, 'sender_thread'):
            self.sender_thread.join(timeout=1.0)
        
        self.can_listener_running = False
        if hasattr(self, 'can_listener_thread'):
            self.can_listener_thread.join(timeout=1.0)
        
        if hasattr(self, 'bus') and self.bus:
            self.bus.shutdown()
            log.info("CAN driver cleaned up")


    def get_current_frame_hex(self):
        """Get the current CAN frame as hex string for debugging"""
        return " ".join([f"{b:02X}" for b in self.can_array])


    def _sender_loop(self):
        """Continuously sends keepalive frames at 20Hz to maintain communication"""

        if self.bus is None:
            log.warning("CAN bus not initialized")
            return

        while self.sender_running:
            try:
                with self.frame_lock:
                    # Send current frame state as keepalive
                    frame_data = self.can_array.copy()
                
                message = can.Message(arbitration_id=self.node_id, data=frame_data, is_extended_id=False)
                # self.bus.send(message)
                print("ok")
                print("here ", frame_data)
                # log.debug(f"Keepalive frame: {' '.join([f'{b:02X}' for b in frame_data])}")
                
            except (OSError, can.CanOperationError) as e:
                log.error(f"CAN send failed: {e}")
                # attempt to reinitialize the CAN bus
                try:
                    self.bus.shutdown()
                except Exception:
                    pass
                try:
                    self.bus = can.interface.Bus(self.can_interface, bustype="socketcan")
                    log.info("Reinitialized CAN bus.")
                except Exception as init_e:
                    log.error(f"Failed to reinitialize CAN bus: {init_e}")
                    self.bus = None
            
            time.sleep(self.send_frame_period)  # Send at 20 Hz (50ms interval) for reliable keepalive



def main(args=None):

    mtt_driver = MTTCanDriver()


if __name__ == '__main__':
    main()