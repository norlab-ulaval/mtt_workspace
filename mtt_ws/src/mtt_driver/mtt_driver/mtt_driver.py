#!/usr/bin/env python3
"""MTT-154 CAN driver: low-level CAN control + tachometer parsing (no ROS deps)."""

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

# Gearing / track constants
MTT_GEAR1 = 16
MTT_GEAR2 = 36
MTT_GEAR3 = 15
MTT_GEAR4 = 32
MTT_GEAR_DRIVE = 8
MTT_GEAR_TRACK = 54
MTT_ENCODER_TEETH = 10
MTT_TRACK_LENGTH_CM = 393
MTT_TRACK_LENGTH_KM = MTT_TRACK_LENGTH_CM / 100000.0  # Firmware: MTT_TRACK_LENGTH_KM = MTT_TRACK_LENGTH_CM / 100000
MTT_TRACK_LENGTH_M = MTT_TRACK_LENGTH_CM / 100.0  # For convenience in Python

CAN_MAIN_TELEMETRY = 0x2FF

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
    """Tachometer state (temps, raw ticks, timestamp)."""
    main_sensor_temp_a: float = 0.0
    main_sensor_temp_b: float = 0.0
    tachometer_instant: int = 0
    tachometer_cumulative: int = 0
    timestamp: float = 0.0
    new_data_available: bool = False

    def get_speed_ms(self, final_ratio: float) -> float:
        """Speed m/s from raw RPS."""
        if final_ratio == 0.0:
            return 0.0
        # Firmware RPS_to_KMh: return ((float)RPS / FinalRatio) * (float)MTT_TRACK_LENGTH_KM *3600;
        # Convert to m/s: ((RPS / FinalRatio) * MTT_TRACK_LENGTH_KM * 3600) / 3.6
        speed_kmh = (self.tachometer_instant / final_ratio) * MTT_TRACK_LENGTH_KM * 3600
        speed_ms = speed_kmh / 3.6
        return speed_ms

    def get_speed_kmh(self, final_ratio: float) -> float:
        """Speed km/h from raw RPS."""
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
    """CAN driver (frame build, send, tachometer parse, safety helpers)."""

    # Reference template (not mutated directly)
    idle_frame_template = [
        0x00,  # [0] Vehicle type: Single Track
        0x68,  # [1] Global switches: deadman pressed, security unlocked, light off
        0x19,  # [2] Throttle: idle value (25 decimal)
        0x7F,  # [3] Winch: neutral (127 decimal)
        0x29,  # [4] Brake: idle value (41 decimal)
        0x7F,  # [5] Steer: center (127 decimal)
        0x00,  # [6] Direction mode: open loop
        0x7F   # [7] Reserved
    ]

    def __init__(self, can_interface='can0', *,
                 winch_auto_neutral_ms: int = 300,
                 start_forward: bool = True,
                 steer_center: int = 128,
                 initial_global_switches: int = 0x40,
                 reserved_byte: int = 0x00):
        """Construct driver.
        Args:
            can_interface: socketcan interface name.
            winch_auto_neutral_ms: auto-neutral timeout for winch (0 disables feature).
            start_forward: choose initial direction bit (bit5) if True.
            steer_center: value used as steering center (firmware tolerates 127/128).
            initial_global_switches: baseline global switch byte (wrapper used 0x40).
            reserved_byte: value forced into byte[7].
        """
        self.node_id = 0x001
        self.winch_auto_neutral_ms = winch_auto_neutral_ms
        self.last_winch_command_time = 0.0
        self.estop_active = False

        # Prepare locking early (we build frame before any threads)
        self.frame_lock = threading.Lock()

    # Deterministic baseline frame
        self.can_array = [
            0x00,
            initial_global_switches & 0xFF,
            0x00,
            WinchState.WinchNeutral.value,
            0x00,
            max(0, min(255, steer_center)),
            0x00,
            reserved_byte & 0xFF
        ]

    # CAN init / wait
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

        # State variables
        self.vehicle_type = VehicleType.VehicleSingleTrack
        self.steer_value = self.can_array[MTT_ANALOG_STEER]
        self.throttle_value = 0
        self.brake_value = 0
        self.winch_state = WinchState.WinchNeutral
        self.security_switch_state = SecuritySwitchState.SafetyLocked  # bit7 cleared in 0x40
        self.direction_state = DirectionState.Forward if start_forward else DirectionState.Reverse
        self.direction_mode = DirectionMode.OpenLoop
        self.light_state = LightState.Off

    # Initial direction bit
        if start_forward:
            self._set_global_bit(5, True)
        else:
            self._set_global_bit(5, False)

    # Odometry state
        self.tachometer_data = TachometerData()
        self.encoder_final_ratio = self._calculate_gear_ratio()
        self.last_cumulative_ticks = 0
        self.total_distance = 0.0
        self.current_direction = self.direction_state

        # Threads (start only after full state prepared)
        self.can_listener_running = True
        self.can_listener_thread = threading.Thread(target=self._can_listener, daemon=True)
        self.can_listener_thread.start()
        self.sender_running = True
        self.sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self.sender_thread.start()
    log.info("MTTCanDriver ready")

    # ---------------- Internal helpers -----------------
    def _set_global_bit(self, bit_index: int, value: bool):
        with self.frame_lock:
            if value:
                self.can_array[MTT_SWITCHES_GLOBAL] |= (1 << bit_index)
            else:
                self.can_array[MTT_SWITCHES_GLOBAL] &= ~(1 << bit_index)

    def apply_estop(self):
        """Latch E-stop."""
        with self.frame_lock:
            self.estop_active = True
        self.set_security_switch(SecuritySwitchState.SafetyLocked)
        self.set_throttle(0)
        self.set_brake(255)
        self.set_winch_state(WinchState.WinchNeutral)
        self.set_steer(self.steer_value if self.steer_value is not None else 128)
        log.warning("Local E-STOP applied in driver")

    def release_estop(self):
        with self.frame_lock:
            self.estop_active = False
        self.set_security_switch(SecuritySwitchState.SafetyUnlocked)
        log.info("Local E-STOP released in driver")

    def set_reserved(self, value: int):
        with self.frame_lock:
            self.can_array[7] = max(0, min(255, int(value)))

    def reset_motion_commands(self):
        """Set motion to safe stop."""
        self.set_throttle(0)
        self.set_steer(127)  # Center position
        self.set_brake(255)  # Full brake
        self.set_direction(DirectionState.Forward)
        self.set_winch_state(WinchState.WinchNeutral)

    def build_can_frame(self, **kwargs):
        """Return new frame from template with provided overrides."""
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
        """Send one-off frame override (doesn't change internal can_array)."""
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
        """Compute final gear ratio (matches firmware)."""
        
        ratio1 = (MTT_GEAR2 / MTT_GEAR1) * MTT_ENCODER_TEETH
        ratio2 = (MTT_GEAR4 / MTT_GEAR3) * ratio1
        final_ratio = ((MTT_GEAR_TRACK / MTT_GEAR_DRIVE) * ratio2) * 2
        
        log.info(f"Encoder final ratio calculated: {final_ratio} (firmware-exact calculation)")
        return final_ratio

    def _can_listener(self):
        """Listener thread: processes 0x2FF frames."""
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
        """Decode 0x2FF frame into tachometer state."""
        if len(data) != 8:
            return
        temp_a = struct.unpack('b', data[0:1])[0]
        temp_b = struct.unpack('b', data[1:2])[0]
        tachimeter_instant = struct.unpack('>H', data[2:4])[0]
        tachimeter_cumulative = struct.unpack('>I', data[4:8])[0]

        # Update tachometer data structure
        self.tachometer_data.main_sensor_temp_a = float(temp_a)
        self.tachometer_data.main_sensor_temp_b = float(temp_b)
        self.tachometer_data.tachometer_instant = tachimeter_instant  # Fixed typo here
        self.tachometer_data.tachometer_cumulative = tachimeter_cumulative
        self.tachometer_data.timestamp = time.time()
        self.tachometer_data.new_data_available = True

        if self.last_cumulative_ticks > 0:
            tick_diff = tachimeter_cumulative - self.last_cumulative_ticks
            if tick_diff > 0:
                gear_ratio_for_distance = self.encoder_final_ratio / 2
                distance_increment = (tick_diff / gear_ratio_for_distance) * MTT_TRACK_LENGTH_M
                self.total_distance += distance_increment

        self.last_cumulative_ticks = tachimeter_cumulative

        if tachimeter_cumulative % 50 == 0:
            speed_ms = self.get_current_speed_ms()
            log.info(
                f"Tachometer data - Speed: {speed_ms:.2f} m/s, "
                f"Cumulative: {tachimeter_cumulative}, "
                f"Temp A: {temp_a}°C, Temp B: {temp_b}°C"
            )

    def emergency_stop(self):
        """Immediate stop (max brake, zero throttle/winch)."""
        self.set_throttle(0)
        self.set_brake(255)
        self.set_steer(127)
        self.set_winch_state(WinchState.WinchNeutral)
        log.warn("EMERGENCY STOP ACTIVATED")

    def is_system_ready(self):
        """Return True if security unlocked and bus OK."""
        if self.security_switch_state != SecuritySwitchState.SafetyUnlocked:
            log.warn("System not ready: Security switch is locked")
            return False
        
        if not hasattr(self, 'bus') or not self.bus:
            log.warn("System not ready: CAN bus not initialized")
            return False
            
        return True

    def get_direction_for_calculation(self):
        """Numeric direction multiplier."""
        return 1 if self.direction_state == DirectionState.Forward else -1

    def get_current_speed_ms(self) -> float:
        """Signed speed (m/s)."""
        if not self.tachometer_data.new_data_available:
            return 0.0
        
        speed = self.tachometer_data.get_speed_ms(self.encoder_final_ratio)
        
        # Apply direction (negative for reverse)
        if self.current_direction == DirectionState.Reverse:
            speed = -speed
        
        return speed
    
    def get_current_speed_kmh(self) -> float:
        """Signed speed (km/h)."""
        if not self.tachometer_data.new_data_available:
            return 0.0
        
        speed = self.tachometer_data.get_speed_kmh(self.encoder_final_ratio)
        
        # Apply direction (negative for reverse)
        if self.current_direction == DirectionState.Reverse:
            speed = -speed
        
        return speed
    
    def get_tachometer_data(self) -> TachometerData:
        """Return tachometer snapshot."""
        return self.tachometer_data
    
    def get_odometry_data(self) -> dict:
        """Return odometry dict (distance always positive)."""
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
        """Set deadman bit."""
        with self.frame_lock:
            if pressed:
                self.can_array[MTT_SWITCHES_GLOBAL] |= 0b00001000  # Set bit 3
            else:
                self.can_array[MTT_SWITCHES_GLOBAL] &= 0b11110111  # Clear bit 3

    # --- Control Methods ---
    def set_steer(self, steer_value):
        """Steer 0-255 (center ~127/128)."""
        if not isinstance(steer_value, int):
            log.error(f"steer_value is not an integer: {steer_value}")
            return

        steer_value = max(0, min(255, steer_value))
        with self.frame_lock:
            self.steer_value = steer_value
            self.can_array[MTT_ANALOG_STEER] = steer_value

    def set_throttle(self, throttle_value):
        """Throttle 0-230."""
        if not isinstance(throttle_value, int):
            log.error(f"throttle_value is not an integer: {throttle_value}")
            return

        throttle_value = max(0, min(230, throttle_value))
        with self.frame_lock:
            self.throttle_value = throttle_value
            self.can_array[MTT_ANALOG_THROTTLE] = throttle_value

    def set_brake(self, brake_value):
        """Brake 0-255."""
        if not isinstance(brake_value, int):
            log.error(f"brake_value is not an integer: {brake_value}")
            return
            
        brake_value = max(0, min(255, brake_value))
        with self.frame_lock:
            self.brake_value = brake_value
            self.can_array[MTT_ANALOG_BRAKE] = brake_value

    def set_winch_state(self, winch_state):
        """Set winch state (auto-neutral watchdog optional)."""
        # Accept raw int or Enum for robustness
        if isinstance(winch_state, int):
            try:
                # Map int to enum if matches value else error
                winch_state = WinchState(winch_state)
            except ValueError:
                log.error(f"invalid raw int for winch_state: {winch_state}")
                return
        if not isinstance(winch_state, WinchState):
            log.error(f"invalid type for winch_state: {winch_state}")
            return
        with self.frame_lock:
            self.can_array[MTT_ANALOG_WINCH] = winch_state.value
            self.winch_state = winch_state
            self.last_winch_command_time = time.time()
    
    def set_security_switch(self, switch_value):
        """Set security switch (bit7)."""
        if switch_value == SecuritySwitchState.SafetyLocked:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] &= 0b01111111  # Clear bit 7
                self.security_switch_state = SecuritySwitchState.SafetyLocked

        elif switch_value == SecuritySwitchState.SafetyUnlocked:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] |= 0b10000000  # Set bit 7
                self.security_switch_state = SecuritySwitchState.SafetyUnlocked

        else:
            log.error(f"invalid value for switch_value: {switch_value}")
            return


    def set_direction(self, direction):
        """Set direction bit."""
        if direction == DirectionState.Forward:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] |= 0b00100000  # Set bit 5
                self.direction_state = DirectionState.Forward
                self.current_direction = DirectionState.Forward

        elif direction == DirectionState.Reverse:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_GLOBAL] &= 0b11011111  # Clear bit 5
                self.direction_state = DirectionState.Reverse
                self.current_direction = DirectionState.Reverse

        else:
            log.error(f"invalid value for direction: {direction}")
            return


    def set_light_state(self, light_state):
        """Set light bit."""
        if light_state == LightState.Off:
            with self.frame_lock:
                self.light_state = LightState.Off
                self.can_array[MTT_SWITCHES_GLOBAL] |= 0b01000000  # Set bit 6

        elif light_state == LightState.On:
            with self.frame_lock:
                self.light_state = LightState.On
                self.can_array[MTT_SWITCHES_GLOBAL] &= 0b10111111  # Clear bit 6
        else:
            log.error(f"invalid value for light_state: {light_state}")
            return


    def set_direction_mode(self, direction_mode):
        """Set open/close loop bit0 of byte6."""
        if direction_mode == DirectionMode.CloseLoop:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_DIRECTION_MODE] |= 0b00000001
                self.direction_mode = DirectionMode.CloseLoop

        elif direction_mode == DirectionMode.OpenLoop:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_DIRECTION_MODE] &= 0b11111110
                self.direction_mode = DirectionMode.OpenLoop
        else:
            log.error(f"invalid value for direction_mode: {direction_mode}")
            return


    def set_vehicle_type(self, vehicle_type):
        """Set vehicle type byte0."""
        if vehicle_type == VehicleType.VehicleSingleTrack:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
                self.vehicle_type = VehicleType.VehicleSingleTrack

        elif vehicle_type == VehicleType.VehicleSbsLeft:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
                self.vehicle_type = VehicleType.VehicleSbsLeft
             
        elif vehicle_type == VehicleType.VehicleSbsRight:
            with self.frame_lock:
                self.can_array[MTT_SWITCHES_VEHICLE_TYPE] = vehicle_type.value
                self.vehicle_type = VehicleType.VehicleSbsRight

        else:
            log.error(f"invalid value for vehicle_type: {vehicle_type}")
            return

    def cleanup(self):
        """Stop threads + shutdown bus."""
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
        """Hex dump of current frame."""
        return " ".join([f"{b:02X}" for b in self.can_array])

    def _sender_loop(self):
        """Keepalive thread (20Hz)."""
        while self.sender_running:
            try:
                with self.frame_lock:
                    # Auto-neutral winch after timeout if feature enabled
                    if self.winch_auto_neutral_ms > 0 and self.winch_state != WinchState.WinchNeutral:
                        if (time.time() - self.last_winch_command_time) * 1000.0 > self.winch_auto_neutral_ms:
                            self.can_array[MTT_ANALOG_WINCH] = WinchState.WinchNeutral.value
                            self.winch_state = WinchState.WinchNeutral
                            log.debug("Winch auto-neutral applied by driver watchdog")
                    frame_data = self.can_array.copy()

                message = can.Message(arbitration_id=self.node_id, data=frame_data, is_extended_id=False)
                self.bus.send(message)
                log.debug(f"Keepalive frame: {' '.join([f'{b:02X}' for b in frame_data])}")
                
            except can.CanError as e:
                if self.sender_running:
                    log.error(f"Keepalive send failed: {e}")
            except Exception as e:
                if self.sender_running:
                    log.error(f"Keepalive error: {e}")
            
            time.sleep(0.05)

