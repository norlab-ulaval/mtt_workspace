#!/usr/bin/env python3
"""MTT-154 ROS2 Wrapper: bridges ROS topics to low-level CAN driver."""

import rclpy
import time
import threading
import logging
from enum import Enum
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from mtt_msgs.msg import MttTachometerData, MttVehicleStatus, MttAuxCommand, MttDrivingMode
from mtt_interfaces.srv import SetVehiculeTypeSrv, GetVehiculeTypeSrv
from .mtt_driver import (
    MTTCanDriver,
    WinchState,
    DirectionState
)


class SafetyState(Enum):
    """Safety state machine for the MTT vehicle."""

    ESTOPPED = "estopped"  # System estopped (start, wrapper closed, remote lost, deadman released)
    READY = "ready"  # Remote detected, waiting for deadman
    ACTIVE = "active"  # Remote detected AND deadman pressed


class SafetyStateMachine:
    """Manages safety state transitions with proper locking."""

    def __init__(self, initial_state=SafetyState.ESTOPPED):
        self.state = initial_state
        self.lock = threading.Lock()
        self.remote_present = False
        self.deadman_active = False

    def transition(self, *, remote_present=None, deadman_active=None):
        """Atomically update inputs and return (prev, new) state."""
        with self.lock:
            prev = self.state
            if remote_present is not None:
                self.remote_present = bool(remote_present)
            if deadman_active is not None:
                self.deadman_active = bool(deadman_active)
            self._update_state()
            return prev, self.state

    def get_state(self):
        """Get current state (thread-safe)."""
        with self.lock:
            return self.state

    def _update_state(self):
        """Internal state update logic (must be called with lock held)."""
        # Communication lost = immediate E-STOP (alive safety)
        if not self.remote_present:
            self.state = SafetyState.ESTOPPED
        # Deadman released = immediate E-STOP (emergency safety)  
        elif not self.deadman_active:
            self.state = SafetyState.READY  # Still E-STOPPED but ready for deadman
        # Both communication and deadman OK = ACTIVE
        elif self.remote_present and self.deadman_active:
            self.state = SafetyState.ACTIVE


class MTTRosWrapper(Node):
    """ROS2 node exposing cmd_vel + aux command control and tachometer/odometry outputs."""

    def __init__(self):
        super().__init__("mtt_ros_wrapper")
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("test_mode", False)
        self.declare_parameter("driver_log_level", "INFO")
        self.declare_parameter("control_frequency_hz", 50.0)
        self.declare_parameter("base_frame", "mtt_base_link")  # added param

        can_interface = self.get_parameter("can_interface").get_parameter_value().string_value
        test_mode = self.get_parameter("test_mode").get_parameter_value().bool_value
        driver_log_level_str = self.get_parameter("driver_log_level").get_parameter_value().string_value
        control_frequency_hz = self.get_parameter("control_frequency_hz").get_parameter_value().double_value
        control_period = 1.0 / control_frequency_hz
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value  # store
        
        self.get_logger().info(f"Control frequency: {control_frequency_hz}Hz (period: {control_period:.6f}s)")

        # Convert string log level to logging constant
        driver_log_level = getattr(logging, driver_log_level_str.upper(), logging.INFO)

        if test_mode:
            can_interface = "vcan0"
            self.get_logger().info("TEST MODE: vcan0")
        else:
            self.get_logger().info(f"CAN interface: {can_interface}")

        try:
            self.driver = MTTCanDriver(can_interface, log_level=driver_log_level)
            self.get_logger().info("MTT Driver initialized")
        except Exception as e:
            self.get_logger().fatal(f"Could not start driver: {e}")
            return

        # Initialize safety state machine
        self.safety_state_machine = SafetyStateMachine(SafetyState.ESTOPPED)

        # Driver serialization lock
        self.driver_lock = threading.RLock()

        # Ensure driver starts in estopped state to match safety state machine
        self._apply_safety_state(SafetyState.ESTOPPED)

        self.send_frame_period = control_period
        # Deadbands to prevent oscillating idle frames
        self.throttle_deadband = 0.01
        self.steer_deadband = 0.01

        # Remote controller detection
        self.remote_timeout_seconds = 0.5  # Remote considered lost after 500ms
        self.last_remote_command_time = None

        # Store timer references for proper shutdown
        self.frame_timer = None
        self.remote_timer = None
        self.ctrl_timer = None

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(MttAuxCommand, "mtt_aux_cmd", self.aux_cmd_callback, 10)
        
        # Single aggregated publisher for all vehicle status
        # Publishers for telemetry data
        self.tachometer_pub = self.create_publisher(MttTachometerData, "mtt_tachometer", 10)  # Pure odometry data
        self.vehicle_status_pub = self.create_publisher(MttVehicleStatus, "mtt_status", 10)   # High-level monitoring
        
        # Publisher for driving mode changes
        self.driving_mode_pub = self.create_publisher(MttDrivingMode, "mtt_driving_mode", 10)
        
        # Services for driving mode control
        self.set_mode_srv = self.create_service(SetVehiculeTypeSrv, "/mtt/set_driving_mode", self._srv_set_mode)
        self.get_mode_srv = self.create_service(GetVehiculeTypeSrv, "/mtt/get_driving_mode", self._srv_get_mode)
        
        # Current driving mode (default: single trailer)
        self.current_driving_mode = 0  # SINGLE_TRAILER
        
        # Keep only essential command feedback
        self.steer_pub = self.create_publisher(UInt8, "mtt_steer_cmd", 10)

        self.ctrl_timer = self.create_timer(control_period, self.control_loop)
        self.get_logger().info("Wrapper ready (E-stop active - waiting for remote controller).")

        self.frame_timer = self.create_timer(self.send_frame_period, self.send_can_frame)
        # Timer to check remote controller presence
        self.remote_timer = self.create_timer(0.1, self.check_remote_presence)

    def send_can_frame(self):
        with self.driver_lock:
            self.driver.send_can_frame()

    def check_remote_presence(self):
        """Check if remote controller is still present based on command timeout."""
        now = time.monotonic()
        lrt = self.last_remote_command_time
        remote_present = (lrt is not None) and (now - lrt < self.remote_timeout_seconds)

        prev, new = self.safety_state_machine.transition(remote_present=remote_present)

        if prev != new:
            if not remote_present:
                self.get_logger().warn(f"Remote controller timeout ({(now - lrt) if lrt else 'never'})")
            else:
                self.get_logger().info("Remote controller detected")
            self.get_logger().info(f"Safety state transition: {prev.value} -> {new.value}")
            self._apply_safety_state(new)

    def _apply_safety_state(self, state):
        """Apply the safety state to the driver."""
        with self.driver_lock:
            if state == SafetyState.ESTOPPED:
                if not self.driver.estop_active:
                    self.driver.emergency_stop()
                    self.get_logger().warn("E-STOP: Communication lost")

            elif state == SafetyState.READY:
                # READY = E-STOPPED waiting for deadman (communication OK)
                if not self.driver.estop_active:
                    self.driver.emergency_stop()
                    self.get_logger().info("E-STOP: Waiting for deadman (remote OK)")

            elif state == SafetyState.ACTIVE:
                if self.driver.estop_active:
                    self.driver.release_estop()
                    self.get_logger().info("System ACTIVE (remote + deadman)")

    def cmd_vel_callback(self, msg: Twist):
        """Handle standard ROS velocity commands."""
        # Mark that we received a remote command
        self.last_remote_command_time = time.monotonic()
        prev, new = self.safety_state_machine.transition(remote_present=True)

        # Handle state transitions (same logic as aux_cmd_callback)
        if prev != new:
            if prev == SafetyState.ESTOPPED and new != SafetyState.ESTOPPED:
                self.get_logger().info("Remote controller detected")
            self.get_logger().info(f"Safety state transition: {prev.value} -> {new.value}")

        # Apply safety state
        self._apply_safety_state(new)

        # Only apply commands if we're in ACTIVE state
        if new != SafetyState.ACTIVE:
            return

        lin = float(msg.linear.x)
        ang = float(msg.angular.z)

        if abs(lin) < self.throttle_deadband:
            lin = 0.0
        if abs(ang) < self.steer_deadband:
            ang = 0.0

        throttle_percent = min(1.0, abs(lin))

        with self.driver_lock:
            self.driver.set_throttle_percent(throttle_percent)
            steer_raw = self.driver.set_steer_normalized(max(-1.0, min(1.0, ang)))
            direction = DirectionState.Forward if lin >= 0 else DirectionState.Reverse
            self.driver.set_direction(direction)

        steer_msg = UInt8()
        steer_msg.data = steer_raw if steer_raw is not None else 0
        self.steer_pub.publish(steer_msg)

    def aux_cmd_callback(self, msg: MttAuxCommand):
        """Handle auxiliary commands (brake, winch, dead man's switch, light toggle)."""
        self.get_logger().debug(f"AUX MSG: deadman={msg.dead_man_switch}, brake={msg.brake:.2f}")

        # Mark that we received a remote command
        self.last_remote_command_time = time.monotonic()

        # Atomic state transition
        prev, new = self.safety_state_machine.transition(remote_present=True, deadman_active=msg.dead_man_switch)

        # Handle state transitions (same logic as check_remote_presence)
        if prev != new:
            if prev == SafetyState.ESTOPPED and new != SafetyState.ESTOPPED:
                self.get_logger().info("Remote controller detected")
            self.get_logger().info(f"Safety state transition: {prev.value} -> {new.value}")

        # Always apply safety state (even if unchanged)
        self._apply_safety_state(new)

        # Brake commands only allowed when ACTIVE (deadman pressed)
        if new == SafetyState.ACTIVE:
            with self.driver_lock:
                self.driver.set_brake_percent(msg.brake)

        # Winch and light commands allowed in both READY and ACTIVE states
        if new in (SafetyState.READY, SafetyState.ACTIVE):
            with self.driver_lock:
                # Winch commands
                if msg.winch_command == MttAuxCommand.WINCH_IN:
                    self.driver.set_winch_state(WinchState.WinchIn)
                elif msg.winch_command == MttAuxCommand.WINCH_OUT:
                    self.driver.set_winch_state(WinchState.WinchOut)
                else:
                    self.driver.set_winch_state(WinchState.WinchNeutral)

                # Light commands
                if hasattr(msg, "light_state"):
                    from .mtt_driver import LightState

                    if msg.light_state == 1:
                        self.driver.set_light_state(LightState.On)
                    else:
                        self.driver.set_light_state(LightState.Off)

    def control_loop(self):
        """Main control loop - publishes tachometer data."""
        with self.driver_lock:
            self._publish_vehicle_data()

    def set_driving_mode(self, mode: int):
        """
        Change driving mode and notify odometry manager.
        
        Args:
            mode: Driving mode (0=SINGLE_TRAILER, 1=DUAL_DIFFERENTIAL, 2=DUAL_SERPENTINE)
        """
        if mode != self.current_driving_mode:
            self.get_logger().info(f"Changing driving mode from {self.current_driving_mode} to {mode}")
            
            # Update internal state
            self.current_driving_mode = mode
            
            # Publish the command so the odometry manager switches too
            mode_msg = MttDrivingMode()
            mode_msg.mode = mode
            # Optional: pass parameters here, e.g. "track=1.2 wheelbase=2.4"
            mode_msg.mode_parameters = ""
            self.driving_mode_pub.publish(mode_msg)
            
            # Mode-specific driver configuration can be added here if needed
            
            self.get_logger().info(f"Successfully changed to driving mode {mode}")
            return True
        return False

    def _srv_set_mode(self, request, response):
        """Service handler for setting driving mode using SetVehiculeTypeSrv."""
        try:
            # 1) Switch internal state
            ok = self.set_driving_mode(int(request.vehicule_type))
            
            # 2) Publish the topic for the Odometry Manager
            msg = MttDrivingMode()
            msg.mode = int(request.vehicule_type)
            msg.mode_parameters = ""  # Could be extended to support parameters
            self.driving_mode_pub.publish(msg)

            response.success = bool(ok)
            return response
        except Exception as e:
            self.get_logger().error(f"set_mode failed: {e}")
            response.success = False
            return response

    def _srv_get_mode(self, request, response):
        """Service handler for getting current driving mode using GetVehiculeTypeSrv."""
        try:
            cur = int(getattr(self, "current_driving_mode", 0))
            # Optional: mapping for human-readable names
            names = {0: "SINGLE_TRAILER", 1: "DUAL_DIFFERENTIAL", 2: "DUAL_SERPENTINE"}
            response.vehicule_type = cur
            response.type_name = names.get(cur, "UNKNOWN")
            return response
        except Exception as e:
            self.get_logger().error(f"get_mode failed: {e}")
            response.vehicule_type = 255  # Invalid value
            response.type_name = "ERROR"
            return response

    def _publish_vehicle_data(self):
        """Publish both tachometer data (for odometry) and vehicle status (for monitoring)."""
        # Use snapshot methods to avoid torn reads
        tach_data = self.driver.get_tachometer_snapshot()
        odometry_data = self.driver.get_odometry_snapshot()
        
        # Always publish vehicle status for safety monitoring, regardless of tachometer data
        status_msg = MttVehicleStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.header.frame_id = self.base_frame
        
        # Safety and connection status must always be published
        status_msg.emergency_stop_active = (self.safety_state_machine.get_state() == SafetyState.ESTOPPED)
        status_msg.remote_connected = (self.last_remote_command_time is not None and 
                                     (time.monotonic() - self.last_remote_command_time) < self.remote_timeout_seconds)
        
        # Motion data only when tachometer data is available
        if tach_data.new_data_available:
            status_msg.speed_ms = odometry_data["speed_ms"]
            status_msg.speed_kmh = odometry_data["speed_kmh"]
            status_msg.distance_km = odometry_data["absolute_distance_m"] / 1000.0
            status_msg.direction = odometry_data["direction"]
            status_msg.temperature_a = odometry_data["temperature_a"]
            status_msg.temperature_b = odometry_data["temperature_b"]
            status_msg.tachometer_instant = tach_data.tachometer_instant
            status_msg.tachometer_cumulative = tach_data.tachometer_cumulative
            
            # Publish pure tachometer data for odometry node
            tachometer_msg = MttTachometerData()
            tachometer_msg.header.stamp = self.get_clock().now().to_msg()
            tachometer_msg.header.frame_id = self.base_frame
            tachometer_msg.tachometer_instant = tach_data.tachometer_instant
            tachometer_msg.tachometer_cumulative = tach_data.tachometer_cumulative
            tachometer_msg.speed_ms = odometry_data["speed_ms"]
            tachometer_msg.speed_kmh = odometry_data["speed_kmh"]
            tachometer_msg.distance_km = odometry_data["absolute_distance_m"] / 1000.0
            tachometer_msg.direction = odometry_data["direction"]
            tachometer_msg.main_sensor_temp_a = odometry_data["temperature_a"]
            tachometer_msg.main_sensor_temp_b = odometry_data["temperature_b"]
            self.tachometer_pub.publish(tachometer_msg)
        else:
            # Set default values when no motion data is available
            status_msg.speed_ms = 0.0
            status_msg.speed_kmh = 0.0
            status_msg.distance_km = 0.0
            status_msg.direction = "Unknown"
            status_msg.temperature_a = 0.0
            status_msg.temperature_b = 0.0
            status_msg.tachometer_instant = 0
            status_msg.tachometer_cumulative = 0
        
        status_msg.steer_position = 0          # Will be enhanced when steering is implemented
        self.vehicle_status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean shutdown with emergency stop."""
        self.get_logger().info("Shutting down MTT driver - applying emergency stop")

        # Cancel timers first to prevent race conditions
        for timer in (
            getattr(self, "frame_timer", None),
            getattr(self, "remote_timer", None),
            getattr(self, "ctrl_timer", None),
        ):
            if timer:
                timer.cancel()

        # Quiesce driver with lock
        if hasattr(self, "driver") and self.driver:
            with self.driver_lock:
                self.driver.emergency_stop()
                self.driver.send_can_frame()
                self.driver.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    wrapper_node = MTTRosWrapper()
    try:
        rclpy.spin(wrapper_node)
    except KeyboardInterrupt:
        pass
    finally:
        wrapper_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
