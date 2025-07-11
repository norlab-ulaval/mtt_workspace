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
from typing import Optional

log = logging.getLogger('MTTCanDriver')
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
MTT_ENCODER_TEETH = 5
MTT_TRACK_LENGTH_CM = 393
MTT_TRACK_LENGTH_KM = MTT_TRACK_LENGTH_CM / 100000.0

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
    Reverse = 0x00
    Forward = 0x01

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
        """Calculate speed in m/s from encoder data"""
        if final_ratio == 0.0:
            return 0.0
        # Convert RPS to actual speed using gear ratios
        speed_ms = (self.tachometer_instant / final_ratio) * MTT_TRACK_LENGTH_KM * 1000.0  # Convert km to m
        return speed_ms

    def get_speed_kmh(self, final_ratio: float) -> float:
        """Calculate speed in km/h from encoder data"""
        return self.get_speed_ms(final_ratio) * 3.6

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

    # --- Real-World Frame Examples (From RF Remote Logs) ---
    # These are examples of frames observed, not fixed templates:
    # Idle frame when deadman NOT pressed: 00 60 19 7F 29 7F 00 7F
    # Idle frame when deadman IS pressed:  00 68 19 7F 29 7F 00 7F
    # Light toggle action:                 00 28 19 7F 29 7F 00 7F
    # Winch in action:                     00 68 19 E5 29 7F 00 7F
    # Winch out action:                    00 68 19 18 29 7F 00 7F
    # Note: 0x19 (throttle) and 0x29 (brake) are just the idle positions, not fixed values
    idle_frame_template = [
        0x00,  # [0] Vehicle type: Single Track
        0x68,  # [1] Global switches: deadman pressed (bit 3=1), security unlocked (bit 7=1), light off (bit 6=0)
        0x19,  # [2] Throttle: idle value (25 decimal) - THIS IS NOT FIXED, just RF remote idle position
        0x7F,  # [3] Winch: neutral (127 decimal)
        0x29,  # [4] Brake: idle value (41 decimal) - THIS IS NOT FIXED, just RF remote idle position
        0x7F,  # [5] Steer: center (127 decimal)
        0x00,  # [6] Direction mode: open loop
        0x7F   # [7] Reserved (matches RF remote)
    ]

    def __init__(self, can_interface='can0'):
        # CAN ID Configuration
        # Reference: CANBus_Specification.md Section 3.1.2
        # Using 0x001 instead of 0x100 due to corruption issues
        self.node_id = 0x001
        
        # Initialize can_array with proper starting values
        self.can_array = [
            0x00,  # [0] Vehicle type: Single Track
            0x68,  # [1] Global switches: deadman pressed, security unlocked, light OFF (bit 6=1)
            0x00,  # [2] Throttle: minimum (0)
            0x7F,  # [3] Winch: neutral (127 decimal)
            0x00,  # [4] Brake: minimum (0)
            0x7F,  # [5] Steer: center (127 decimal)
            0x00,  # [6] Direction mode: open loop
            0x7F   # [7] Reserved
        ]
        
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
        
        # Winch auto-reset control
        self.winch_timer = None
        self.winch_timer_lock = threading.Lock()

        # Odometry and Tachometer Data
        self.tachometer_data = TachometerData()
        self.encoder_final_ratio = 0.0
        self.last_cumulative_ticks = 0
        self.total_distance = 0.0  # in meters
        self.current_direction = DirectionState.Forward  # Track current direction for odometry
        
        # Calculate gear ratio once at startup
        self.encoder_final_ratio = self._calculate_gear_ratio()
        
        # CAN listener thread for tachometer data (0x2FF frames)
        self.can_listener_thread = threading.Thread(target=self._can_listener, daemon=True)
        self.can_listener_running = True
        self.can_listener_thread.start()

        # Safety Configuration
        # Reference: CANBus_Specification.md Section 3.1 - Critical safety patches
        # - Security switch MUST be unlocked (MTT_DEF_SAFETY_UNLOCKED) for operation
        # - Light state acts as emergency stop due to test modifications
        # - One light state = stop, other = operational (test both to determine)
        # - Direction is controlled by master system (us) via direction bit

        # MTT_SWITCHES_VEHICLE_TYPE
        self.set_vehicle_type(VehicleType.VehicleSingleTrack)

        # MTT_SWITCHES_GLOBAL - SAFETY CRITICAL
        # Security switch must be unlocked for system to function
        self.set_security_switch(SecuritySwitchState.SafetyLocked)  # Start locked for safety
        
        # Set initial direction (controlled by master system)
        self.set_direction(DirectionState.Forward)
        
        # Light state - emergency stop mechanism added for testing
        # Start with light OFF (safer initial state) 
        self.set_light_state(LightState.Off)  # This sets bit 6=1 (light OFF)

        # MTT_ANALOG_THROTTLE
        self.set_throttle(0)
        # self.set_throttle(60)

        # MTT_ANALOG_WINCH - Set to proper neutral state
        self.set_winch_state(WinchState.WinchNeutral)

        # MTT_ANALOG_BRAKE
        self.set_brake(0)

        # MTT_ANALOG_STEER - Use same center value as RF remote
        self.set_steer(127)  # Center position (0x7F, same as RF remote)

        # MTT_SWITCHES_DIRECTION_MODE
        self.set_direction_mode(DirectionMode.OpenLoop)
        # self.set_direction_mode(DirectionMode.CloseLoop)

    def reset_motion_commands(self):
        """Resets all motion commands to a safe, stopped state (RF idle template)."""
        self.set_throttle(0)
        self.set_steer(127)  # Center position (same as RF remote)
        self.set_brake(255)  # Full brake
        self.set_direction(DirectionState.Forward)
        self.set_winch_state(WinchState.WinchNeutral)  # Always set winch to neutral for idle
        # Optionally, set other fields to match RF idle if needed

    def build_can_frame(self, **kwargs):
        """
        Build a CAN frame starting from the idle template, only altering fields specified in kwargs.
        Example: build_can_frame(throttle=60, winch=WinchState.WinchIn.value)
        """
        frame = self.idle_frame_template.copy()
        if 'vehicle_type' in kwargs:
            frame[MTT_SWITCHES_VEHICLE_TYPE] = kwargs['vehicle_type']
        if 'global_switches' in kwargs:
            frame[MTT_SWITCHES_GLOBAL] = kwargs['global_switches']
        if 'throttle' in kwargs:
            frame[MTT_ANALOG_THROTTLE] = kwargs['throttle']
        if 'winch' in kwargs:
            frame[MTT_ANALOG_WINCH] = kwargs['winch']
        if 'brake' in kwargs:
            frame[MTT_ANALOG_BRAKE] = kwargs['brake']
        if 'steer' in kwargs:
            frame[MTT_ANALOG_STEER] = kwargs['steer']
        if 'direction_mode' in kwargs:
            frame[MTT_SWITCHES_DIRECTION_MODE] = kwargs['direction_mode']
        # Reserved byte (7) can be set if needed
        return frame

    def send_can_frame(self, **kwargs):
        """
        Sends a CAN frame using the current can_array state, with only specified fields altered.
        Usage: send_can_frame(throttle=60, winch=WinchState.WinchIn.value)
        """
        if not self.bus:
            return
        
        # Start with current state
        frame = self.can_array.copy()
        
        # Apply any overrides from kwargs
        if 'vehicle_type' in kwargs:
            frame[MTT_SWITCHES_VEHICLE_TYPE] = kwargs['vehicle_type']
        if 'global_switches' in kwargs:
            frame[MTT_SWITCHES_GLOBAL] = kwargs['global_switches']
        if 'throttle' in kwargs:
            frame[MTT_ANALOG_THROTTLE] = kwargs['throttle']
        if 'winch' in kwargs:
            frame[MTT_ANALOG_WINCH] = kwargs['winch']
        if 'brake' in kwargs:
            frame[MTT_ANALOG_BRAKE] = kwargs['brake']
        if 'steer' in kwargs:
            frame[MTT_ANALOG_STEER] = kwargs['steer']
        if 'direction_mode' in kwargs:
            frame[MTT_SWITCHES_DIRECTION_MODE] = kwargs['direction_mode']
        
        try:
            message = can.Message(arbitration_id=self.node_id, data=frame, is_extended_id=False)
            self.bus.send(message)
            log.debug(f"Sent CAN frame: {' '.join([f'{b:02X}' for b in frame])}")
        except can.CanError as e:
            log.error(f"Error sending CAN frame: {e}")

    def _calculate_gear_ratio(self):
        """Calculate the final gear ratio for speed conversion"""
        ratio1 = (MTT_GEAR2 / MTT_GEAR1) * MTT_ENCODER_TEETH
        ratio2 = (MTT_GEAR4 / MTT_GEAR3) * ratio1
        final_ratio = ((MTT_GEAR_TRACK / MTT_GEAR_DRIVE) * ratio2) * 2
        log.info(f"Encoder final ratio calculated: {final_ratio}")
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
                distance_increment = (tick_diff / self.encoder_final_ratio) * MTT_TRACK_LENGTH_KM * 1000.0  # Convert to meters
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

    def get_speed_kmh(self):
        """Convert tachometer instant reading to km/h"""
        if self.tachometer_data['tachimeter_instant'] > 0:
            rps = float(self.tachometer_data['tachimeter_instant'])
            return (rps / self.final_ratio) * self.MTT_TRACK_LENGTH_KM * 3600
        return 0.0

    def get_cumulative_distance_km(self):
        """Get cumulative distance in kilometers from tachometer data"""
        if self.tachometer_data['tachimeter_cumulative'] > 0:
            ticks = float(self.tachometer_data['tachimeter_cumulative'])
            return (ticks / self.final_ratio) * self.MTT_TRACK_LENGTH_KM
        return 0.0

    def get_tachometer_data(self):
        """Get all tachometer data"""
        return self.tachometer_data.copy()

    def emergency_stop(self):
        """Emergency stop function - sets all motion controls to safe values"""
        self.set_throttle(0)
        self.set_brake(255)  # Maximum brake
        self.set_steer(127)  # Center steering (same as RF remote)
        self.set_winch_state(WinchState.WinchNeutral)  # Standard neutral
        log.warn("EMERGENCY STOP ACTIVATED")

    def is_system_ready(self):
        """Check if system is ready for operation"""
        if self.security_switch_state != SecuritySwitchState.SafetyUnlocked:
            log.warn("System not ready: Security switch is locked")
            return False
        
        if not hasattr(self, 'bus') or not self.bus:
            log.warn("System not ready: CAN bus not initialized")
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

    def set_deadman_switch(self, pressed=True):
        """
        Set deadman switch state based on real RF remote behavior
        
        From RF remote logs:
        - Deadman NOT pressed: Global switches = 0x60 (bit 3=0)
        - Deadman pressed: Global switches = 0x68 (bit 3=1)
        
        Bit 3 (0x08) controls deadman switch:
        - 0 = not pressed (idle state)
        - 1 = pressed (active state)
        """
        if pressed:
            self.can_array[MTT_SWITCHES_GLOBAL] |= 0b00001000  # Set bit 3 (0x08)
        else:
            self.can_array[MTT_SWITCHES_GLOBAL] &= 0b11110111  # Clear bit 3 (0x08)
        
        # Send frame with updated state
        self.send_can_frame()

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
            # Send frame with updated state
            self.send_can_frame()

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
            # Send frame with updated state
            self.send_can_frame()

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
            # Send frame with updated state
            self.send_can_frame()

    def set_winch_state(self, winch_state):
        """
        Set the winch state for the next CAN frame. Always returns to neutral (0x7F) after IN/OUT, matching RF remote idle behavior.
        """
        with self.winch_timer_lock:
            # Cancel any existing timer
            if self.winch_timer is not None:
                self.winch_timer.cancel()
                self.winch_timer = None
            
            if winch_state == WinchState.WinchNeutral:
                self.can_array[MTT_ANALOG_WINCH] = WinchState.WinchNeutral.value
                self.winch_state = WinchState.WinchNeutral
                # Send frame with updated state
                self.send_can_frame()
            elif winch_state == WinchState.WinchIn:
                self.can_array[MTT_ANALOG_WINCH] = WinchState.WinchIn.value
                self.winch_state = WinchState.WinchIn
                # Send frame with winch in
                self.send_can_frame()
                # Schedule return to neutral after 50ms
                self.winch_timer = threading.Timer(0.05, self._winch_auto_neutral)
                self.winch_timer.start()
            elif winch_state == WinchState.WinchOut:
                self.can_array[MTT_ANALOG_WINCH] = WinchState.WinchOut.value
                self.winch_state = WinchState.WinchOut
                # Send frame with winch out
                self.send_can_frame()
                # Schedule return to neutral after 50ms
                self.winch_timer = threading.Timer(0.05, self._winch_auto_neutral)
                self.winch_timer.start()
            else:
                log.error(f"invalid value for winch_state: {winch_state}")
                return
    
    def _winch_auto_neutral(self):
        """Internal method to automatically return winch to neutral (called by timer)"""
        with self.winch_timer_lock:
            if self.winch_timer is not None:
                self.winch_timer = None
                self.can_array[MTT_ANALOG_WINCH] = WinchState.WinchNeutral.value
                self.winch_state = WinchState.WinchNeutral
                self.send_can_frame()
                log.debug("Winch auto-returned to neutral")

    def set_security_switch(self, switch_value):
        """
        CRITICAL: Security switch must be unlocked for system to function
        
        From CANBus_Specification.md v1.1:
        void MTT_SetSecuritySwitch(uint8_t SwitchValue)
        - MTT_DEF_SAFETY_LOCKED (0x00): MttControl.CAN_Array[MTT_SWITCHES_GLOBAL] &= 0b01111111
        - MTT_DEF_SAFETY_UNLOCKED (0x01): MttControl.CAN_Array[MTT_SWITCHES_GLOBAL] |= 0b10000000
        
        Bit 7 (0x80) controls security switch:
        - 0 = locked (system cannot function)
        - 1 = unlocked (system can function)
        
        Real-world behavior from RF remote logs:
        - Security locked: Global switches = 0x60 (bit 7=0)
        - Security unlocked: Global switches = 0x68 (bit 7=0, but bit 3=1 for deadman)
        """
        if switch_value == SecuritySwitchState.SafetyLocked:
            self.can_array[MTT_SWITCHES_GLOBAL] &= 0b01111111  # Clear bit 7 (0x80)
            self.security_switch_state = SecuritySwitchState.SafetyLocked
            # Send frame with updated state
            self.send_can_frame()

        elif switch_value == SecuritySwitchState.SafetyUnlocked:
            self.can_array[MTT_SWITCHES_GLOBAL] |= 0b10000000  # Set bit 7 (0x80)
            self.security_switch_state = SecuritySwitchState.SafetyUnlocked
            # Send frame with updated state
            self.send_can_frame()

        else:
            log.error(f"invalid value for switch_value: {switch_value}")
            return


    def set_direction(self, direction):
        """Set vehicle direction and track for odometry (INVERTED: Forward/Reverse logic swapped)"""
        # Inverted logic: Forward sets reverse bit, Reverse clears it
        if direction == DirectionState.Forward:
            self.can_array[MTT_SWITCHES_GLOBAL] |= 0b00100000  # Set bit 5 (reverse)
            self.direction_state = DirectionState.Forward
            self.current_direction = DirectionState.Forward  # Track for odometry
            # Send frame with updated state
            self.send_can_frame()

        elif direction == DirectionState.Reverse:
            self.can_array[MTT_SWITCHES_GLOBAL] &= 0b11011111  # Clear bit 5 (forward)
            self.direction_state = DirectionState.Reverse
            self.current_direction = DirectionState.Reverse  # Track for odometry
            # Send frame with updated state
            self.send_can_frame()

        else:
            log.error(f"invalid value for direction: {direction}")
            return


    def set_light_state(self, light_state):
        """
        Set light state based on real RF remote behavior
        
        From RF remote logs:
        - Light OFF: Global switches = 0x68 (bit 6=1)
        - Light ON: Global switches = 0x28 (bit 6=0)
        
        Note: The behavior appears inverted from spec - bit 6=1 means light OFF
        """
        if light_state == LightState.Off:
            self.light_state = LightState.Off
            self.can_array[MTT_SWITCHES_GLOBAL] |= 0b01000000  # Set bit 6 (light OFF)
            # Send frame with updated state
            self.send_can_frame()

        elif light_state == LightState.On:
            self.light_state = LightState.On
            self.can_array[MTT_SWITCHES_GLOBAL] &= 0b10111111  # Clear bit 6 (light ON)
            # Send frame with updated state
            self.send_can_frame()
        else:
            log.error(f"invalid value for light_state: {light_state}")
            return


    def set_direction_mode(self, direction_mode):
        if direction_mode == DirectionMode.CloseLoop:
            self.can_array[MTT_SWITCHES_DIRECTION_MODE] |= 0b00000001
            self.direction_mode = DirectionMode.CloseLoop
            # Send frame with updated state
            self.send_can_frame()

        elif direction_mode == DirectionMode.OpenLoop:
            self.can_array[MTT_SWITCHES_DIRECTION_MODE] &= 0b11111110
            self.direction_mode = DirectionMode.OpenLoop
            # Send frame with updated state
            self.send_can_frame()
        else:
            log.error(f"invalid value for direction_mode: {direction_mode}")
            return


    def set_vehicle_type(self, vehicle_type):
        if vehicle_type == VehicleType.VehicleSingleTrack:
            self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
            self.vehicle_type = VehicleType.VehicleSingleTrack
            # Send frame with updated state
            self.send_can_frame()

        elif vehicle_type == VehicleType.VehicleSbsLeft:
            self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
            self.vehicle_type = VehicleType.VehicleSbsLeft
            # Send frame with updated state
            self.send_can_frame()
             
        elif vehicle_type == VehicleType.VehicleSbsRight:
            self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
            self.vehicle_type = VehicleType.VehicleSbsRight
            # Send frame with updated state
            self.send_can_frame()

        else:
            log.error(f"invalid value for vehicle_type: {vehicle_type}")
            return

    def test_real_world_behavior(self):
        """
        Test method that demonstrates all real-world behaviors matching RF remote logs
        
        This method sends the exact same CAN frames as observed from the RF remote:
        - Idle (deadman NOT pressed): 00 60 19 7F 29 7F 00 7F
        - Idle (deadman pressed):  00 68 19 7F 29 7F 00 7F
        - Light toggle: 00 28 19 7F 29 7F 00 7F
        - Winch in: 00 68 19 E5 29 7F 00 7F
        - Winch out: 00 68 19 18 29 7F 00 7F
        """
        log.info("Testing real-world behavior patterns...")
        
        # Test 1: Idle with deadman NOT pressed (RF remote idle)
        log.info("Test 1: Idle with deadman NOT pressed")
        self.set_deadman_switch(pressed=False)
        time.sleep(0.1)
        
        # Test 2: Idle with deadman pressed (RF remote active idle)
        log.info("Test 2: Idle with deadman pressed")
        self.set_deadman_switch(pressed=True)
        time.sleep(0.1)
        
        # Test 3: Light toggle action
        log.info("Test 3: Light toggle")
        self.set_light_state(LightState.On)  # This should send 0x28 for global switches
        time.sleep(0.1)
        self.set_light_state(LightState.Off)  # This should send 0x68 for global switches
        time.sleep(0.1)
        
        # Test 4: Winch in action
        log.info("Test 4: Winch in")
        self.set_winch_state(WinchState.WinchIn)  # Should send 0xE5 for winch, then auto-neutral
        time.sleep(0.2)
        
        # Test 5: Winch out action
        log.info("Test 5: Winch out")
        self.set_winch_state(WinchState.WinchOut)  # Should send 0x18 for winch, then auto-neutral
        time.sleep(0.2)
        
        # Test 6: Throttle variation
        log.info("Test 6: Throttle variation")
        self.set_throttle(50)  # Should change from 0x19 to 0x32
        time.sleep(0.1)
        self.set_throttle(25)  # Back to RF remote default 0x19
        time.sleep(0.1)
        
        # Test 7: Brake variation
        log.info("Test 7: Brake variation")
        self.set_brake(100)  # Should change from 0x29 to 0x64
        time.sleep(0.1)
        self.set_brake(41)  # Back to RF remote default 0x29
        time.sleep(0.1)
        
        log.info("Real-world behavior test completed")

    def cleanup(self):
        """Clean up resources before shutting down"""
        # Cancel any pending winch timer
        with self.winch_timer_lock:
            if self.winch_timer is not None:
                self.winch_timer.cancel()
                self.winch_timer = None
        
        # Stop CAN listener
        self.can_listener_running = False
        if hasattr(self, 'can_listener_thread'):
            self.can_listener_thread.join(timeout=1.0)
        
        # Close CAN bus
        if hasattr(self, 'bus') and self.bus:
            self.bus.shutdown()
            log.info("CAN driver cleaned up")

    def get_current_frame_hex(self):
        """Get the current CAN frame as hex string for debugging"""
        return " ".join([f"{b:02X}" for b in self.can_array])

