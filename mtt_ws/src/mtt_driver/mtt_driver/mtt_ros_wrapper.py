#!/usr/bin/env python3
"""MTT-154 ROS2 Wrapper: bridges ROS topics to low-level CAN driver."""

import rclpy
import time
import threading
import logging
from enum import Enum
from typing import Optional
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from mtt_msgs.msg import MttTachometerData, MttVehicleStatus, MttAuxCommand, MttDrivingMode
from mtt_interfaces.srv import SetVehiculeTypeSrv, GetVehiculeTypeSrv
from .mtt_driver import MTTCanDriver, WinchState, DirectionState, SecuritySwitchState, STEER_CENTER, STEER_MAX
from .mtt_articulated_model import ArticulatedVehicleParams


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
        # Parameters
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("test_mode", False)
        self.declare_parameter("driver_log_level", "INFO")
        self.declare_parameter("control_frequency_hz", 50.0)
        self.declare_parameter("can_frame_frequency_hz", 20.0)
        self.declare_parameter("base_frame", "mtt_base_link")
        self.declare_parameter("can_id", 0x001)
        # Command integration parameters
        self.declare_parameter("use_external_mux", False)  # Use twist_mux to feed unified /cmd_vel
        # control_source: 'auto' | 'teleop' | 'auto_if_no_deadman'
        self.declare_parameter("control_source", "auto_if_no_deadman")
        self.declare_parameter("command_timeout", 0.5)
        # Direction switch hysteresis threshold to avoid rapid FWD/REV flipping
        self.declare_parameter("direction_switch_threshold", 0.05)
        # Topic parameters to support alternate control pipelines
        self.declare_parameter("teleop_input_topic", "cmd_vel/teleop_smoothed")
        self.declare_parameter("teleop_raw_topic", "cmd_vel/teleop")

        # Resolve parameters
        can_interface = self.get_parameter("can_interface").get_parameter_value().string_value
        test_mode = self.get_parameter("test_mode").get_parameter_value().bool_value
        driver_log_level_str = self.get_parameter("driver_log_level").get_parameter_value().string_value
        control_frequency_hz = self.get_parameter("control_frequency_hz").get_parameter_value().double_value
        can_frame_period_hz = self.get_parameter("can_frame_frequency_hz").get_parameter_value().double_value
        self.control_period = 1.0 / max(1e-3, control_frequency_hz)
        self.can_frame_period = 1.0 / max(1e-3, can_frame_period_hz)
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.can_id = self.get_parameter("can_id").get_parameter_value().integer_value
        self.use_external_mux = self.get_parameter("use_external_mux").get_parameter_value().bool_value
        self.control_source = self.get_parameter("control_source").get_parameter_value().string_value
        self.command_timeout = self.get_parameter("command_timeout").get_parameter_value().double_value
        self.teleop_input_topic = self.get_parameter("teleop_input_topic").get_parameter_value().string_value
        self.teleop_raw_topic = self.get_parameter("teleop_raw_topic").get_parameter_value().string_value
        self.direction_switch_threshold = (
            self.get_parameter("direction_switch_threshold").get_parameter_value().double_value
        )

        # If running in test mode and the default interface is used, switch to vcan0
        if test_mode and can_interface == "can0":
            self.get_logger().info("test_mode enabled: using vcan0 instead of can0")
            can_interface = "vcan0"

        driver_log_level = getattr(logging, driver_log_level_str.upper(), logging.INFO)
        self.get_logger().info(f"CAN interface: {can_interface}")

        # Driver init
        try:
            self.driver = MTTCanDriver(can_interface, log_level=driver_log_level, can_id=self.can_id)
            self.get_logger().info("MTT Driver initialized")
        except Exception as e:
            self.get_logger().fatal(f"Could not start driver: {e}")
            raise

        # Safety machines
        self.safety_state_machine_cmd_vel = SafetyStateMachine(SafetyState.ESTOPPED)
        self.safety_state_machine_winch = SafetyStateMachine(SafetyState.ESTOPPED)

        # Locks and thresholds
        self.driver_lock = threading.RLock()
        self.throttle_deadband = 0.01
        self.steer_deadband = 0.01

        # Remote presence detection
        self.remote_timeout_seconds = 0.5
        self.last_remote_command_time = None

        # Subscriptions: unified + internal mux inputs
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        # Teleop raw (for presence/freshness) and smoothed (for command)
        self.teleop_last_msg = None
        self.teleop_last_time = None
        self.teleop_raw_last_msg = None
        self.teleop_raw_last_time = None
        self.create_subscription(Twist, self.teleop_input_topic, self._teleop_smoothed_cb, 10)
        self.create_subscription(Twist, self.teleop_raw_topic, self._teleop_raw_cb, 10)
        # Auto
        self.auto_last_msg = None
        self.auto_last_time = None
        self.create_subscription(Twist, "cmd_vel/auto", self._auto_cmd_cb, 10)
        self.create_subscription(MttAuxCommand, "mtt_aux_cmd", self.aux_cmd_callback, 10)

        # Publishers and services
        self.tachometer_pub = self.create_publisher(MttTachometerData, "mtt_tachometer", 10)
        self.vehicle_status_pub = self.create_publisher(MttVehicleStatus, "mtt_status", 10)
        self.driving_mode_pub = self.create_publisher(MttDrivingMode, "mtt_driving_mode", 10)
        self.set_mode_srv = self.create_service(SetVehiculeTypeSrv, "/mtt/set_driving_mode", self._srv_set_mode)
        self.get_mode_srv = self.create_service(GetVehiculeTypeSrv, "/mtt/get_driving_mode", self._srv_get_mode)

        # Mode state and feedback
        self.current_driving_mode = 0  # SINGLE_TRAILER
        self.steer_pub = self.create_publisher(UInt8, "mtt_steer_cmd", 10)

        # Timers
        self.ctrl_timer = self.create_timer(self.control_period, self.control_loop)
        self.cmd_select_timer = self.create_timer(self.control_period, self._select_and_apply_command)
        self.can_frame_timer = self.create_timer(self.can_frame_period, self.send_can_frame)
        self.remote_timer = self.create_timer(0.1, self.check_remote_presence)

        self.get_logger().info("Wrapper ready (E-stop active - waiting for remote controller).")
        # Internal mux auxiliary state
        self._zero_twist = Twist()
        self._last_cmd_applied_time = 0.0
        self._last_zero_sent = False
        # Track last direction to apply hysteresis around zero
        self._last_direction_state = DirectionState.Forward

    # -------------------- Services -------------------- #
    def _srv_set_mode(self, request: SetVehiculeTypeSrv.Request, response: SetVehiculeTypeSrv.Response):
        """Set vehicle/driving mode via low-level driver then notify odometry manager.

        request.vehicule_type: uint8 (0=SINGLE_TRAILER, 1=DUAL_DIFFERENTIAL, 2=DUAL_SERPENTINE)
        """
        try:
            mode = int(request.vehicule_type)
            self._set_driving_mode(mode)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to set driving mode: {e}")
            response.success = False
        return response

    def _srv_get_mode(self, request: GetVehiculeTypeSrv.Request, response: GetVehiculeTypeSrv.Response):
        """Return current driving mode as numeric code and name."""
        try:
            response.vehicule_type = int(self.current_driving_mode)
            # Provide a human-readable name consistent with odometry manager
            names = {0: "SINGLE_TRAILER", 1: "DUAL_DIFFERENTIAL", 2: "DUAL_SERPENTINE"}
            response.type_name = names.get(int(self.current_driving_mode), "UNKNOWN")
        except Exception as e:
            self.get_logger().error(f"Failed to get driving mode: {e}")
            response.vehicule_type = 0
            response.type_name = "UNKNOWN"
        return response

    # -------------------- Internal mux input callbacks -------------------- #
    def _teleop_raw_cb(self, msg: Twist):
        # Raw teleop updates presence and freshness
        self.teleop_raw_last_msg = msg
        self.teleop_raw_last_time = time.monotonic()
        self.last_remote_command_time = self.teleop_raw_last_time

    def _teleop_smoothed_cb(self, msg: Twist):
        # Smoothed teleop is used for command but not for presence
        self.teleop_last_msg = msg
        self.teleop_last_time = time.monotonic()

    def _auto_cmd_cb(self, msg: Twist):
        self.auto_last_msg = msg
        self.auto_last_time = time.monotonic()
        # Auto also counts as command activity for presence
        self.last_remote_command_time = self.auto_last_time

    def _is_fresh(self, t: Optional[float]) -> bool:
        if t is None:
            return False
        return (time.monotonic() - t) < self.command_timeout

    def _select_and_apply_command(self):
        """If using internal mux, choose the active command and apply it."""
        if self.use_external_mux:
            return  # External mux will feed /cmd_vel -> cmd_vel_callback

        # Choose source based on configured policy
        source = self.control_source
        chosen: Optional[Twist] = None

        # Gate teleop by deadman when using teleop path
        deadman_active = self.safety_state_machine_cmd_vel.deadman_active
        # Freshness is evaluated on RAW input; smoothed may continue briefly
        teleop_fresh = self._is_fresh(self.teleop_raw_last_time)
        auto_fresh = self._is_fresh(self.auto_last_time)

        if source == "teleop":
            if teleop_fresh and deadman_active:
                chosen = self.teleop_last_msg
        elif source == "auto":
            if auto_fresh:
                chosen = self.auto_last_msg
        else:  # auto_if_no_deadman (default)
            if deadman_active and teleop_fresh:
                chosen = self.teleop_last_msg
            elif auto_fresh:
                chosen = self.auto_last_msg

        # Apply only when the system is ACTIVE; otherwise e-stop logic handles quiescing
        if self.safety_state_machine_cmd_vel.get_state() == SafetyState.ACTIVE:
            now = time.monotonic()
            if chosen is not None:
                # Reuse the same handling as unified /cmd_vel
                self._handle_cmd_vel(chosen)
                self._last_cmd_applied_time = now
                self._last_zero_sent = False
            else:
                # No fresh command: enforce safe stop (zero) to avoid holding last speed
                # Limit zero spam by remembering last zero was sent
                if not self._last_zero_sent or (now - self._last_cmd_applied_time) >= self.command_timeout:
                    self._handle_cmd_vel(self._zero_twist)
                    self._last_zero_sent = True

    def send_can_frame(self):
        with self.driver_lock:
            self.driver.send_can_frame()

    def check_remote_presence(self):
        """Check if remote controller is still present based on command timeout."""
        now = time.monotonic()
        remote_present = (
            self.last_remote_command_time is not None
            and (now - self.last_remote_command_time) < self.remote_timeout_seconds
        )

        # Update both safety state machines for remote presence
        prev_state_cmd, new_state_cmd = self.safety_state_machine_cmd_vel.transition(remote_present=remote_present)
        prev_state_winch, new_state_winch = self.safety_state_machine_winch.transition(remote_present=remote_present)

        if prev_state_cmd != new_state_cmd:
            if not remote_present:
                self.get_logger().warning(
                    f"Remote controller timeout ({(now - self.last_remote_command_time) if self.last_remote_command_time else 'never'})"
                )
            else:
                self.get_logger().info("Remote controller detected")
            self.get_logger().info(f"Cmd_vel safety state transition: {prev_state_cmd.value} -> {new_state_cmd.value}")
            self._apply_safety_state_cmd_vel(new_state_cmd)

        if prev_state_winch != new_state_winch:
            self.get_logger().info(
                f"Winch safety state transition: {prev_state_winch.value} -> {new_state_winch.value}"
            )
            self._apply_safety_state_winch(new_state_winch)

    def _apply_safety_state_cmd_vel(self, state):
        """Apply the safety state to the driver for cmd_vel operations."""
        with self.driver_lock:
            if state == SafetyState.ESTOPPED:
                if self.driver.get_security_switch_state() != SecuritySwitchState.SafetyLocked:
                    self.driver.emergency_stop()
                    self.get_logger().warn("E-STOP: Communication lost")

            elif state == SafetyState.READY:
                # READY = E-STOPPED waiting for deadman (communication OK)
                if self.driver.get_security_switch_state() != SecuritySwitchState.SafetyLocked:
                    self.driver.emergency_stop()
                    self.get_logger().info("E-STOP: Waiting for deadman (remote OK)")

            elif state == SafetyState.ACTIVE:
                if self.driver.get_security_switch_state() == SecuritySwitchState.SafetyLocked:
                    self.driver.release_estop()
                    self.get_logger().info("System ACTIVE (remote + deadman)")

    def _apply_safety_state_winch(self, state):
        """Apply the safety state to the winch operations only."""
        with self.driver_lock:
            if state in (SafetyState.ESTOPPED, SafetyState.READY):
                # Force winch to neutral when not in ACTIVE state
                self.driver.set_winch_state(WinchState.WinchNeutral)
                if state == SafetyState.ESTOPPED:
                    self.get_logger().debug("Winch safety: Communication lost - winch neutral")
                else:
                    self.get_logger().debug("Winch safety: Winch button released - winch neutral")

    def cmd_vel_callback(self, msg: Twist):
        """Handle unified ROS velocity commands (external mux or legacy)."""
        # Mark that we received a remote command
        self.last_remote_command_time = time.monotonic()
        prev, new = self.safety_state_machine_cmd_vel.transition(remote_present=True)

        # Handle state transitions (same logic as aux_cmd_callback)
        if prev != new:
            if prev == SafetyState.ESTOPPED and new != SafetyState.ESTOPPED:
                self.get_logger().info("Remote controller detected")
            self.get_logger().info(f"Safety state transition: {prev.value} -> {new.value}")
            # Apply safety state only when it changes
            self._apply_safety_state_cmd_vel(new)

        # Only apply commands if we're in ACTIVE state
        if new != SafetyState.ACTIVE:
            return

        self._handle_cmd_vel(msg)

    def _handle_cmd_vel(self, msg: Twist):
        lin = float(msg.linear.x)
        ang = float(msg.angular.z)

        if abs(lin) < self.throttle_deadband:
            lin = 0.0
        if abs(ang) < self.steer_deadband:
            ang = 0.0

        throttle_percent = min(1.0, abs(lin))

        # Direction hysteresis: only switch when beyond +/- threshold, else keep last
        thr = max(0.0, float(self.direction_switch_threshold))
        if lin > thr:
            desired_dir = DirectionState.Forward
        elif lin < -thr:
            desired_dir = DirectionState.Reverse
        else:
            desired_dir = self._last_direction_state

        with self.driver_lock:
            self.driver.set_throttle(throttle_percent)
            steer_raw = self.driver.set_steer(max(-1.0, min(1.0, ang)))
            if desired_dir != self._last_direction_state:
                self.driver.set_direction(desired_dir)
                self._last_direction_state = desired_dir

        steer_msg = UInt8()
        steer_msg.data = steer_raw if steer_raw is not None else 0
        self.steer_pub.publish(steer_msg)

    def aux_cmd_callback(self, msg: MttAuxCommand):
        """Handle auxiliary commands (brake, winch, dead man's switch, light toggle)."""
        self.get_logger().debug(f"AUX MSG: deadman={msg.dead_man_switch}, brake={msg.brake:.2f}")

        # Mark that we received a remote command
        self.last_remote_command_time = time.monotonic()

        # Handle cmd_vel safety state machine (deadman switch)
        prev_cmd, new_cmd = self.safety_state_machine_cmd_vel.transition(
            remote_present=True,
            deadman_active=msg.dead_man_switch,
        )

        # Handle winch safety state machine using the new winch_safety_button field
        winch_safety_active = msg.winch_safety_button
        prev_winch, new_winch = self.safety_state_machine_winch.transition(
            remote_present=True,
            deadman_active=winch_safety_active,
        )

        # Handle cmd_vel state transitions
        if prev_cmd != new_cmd:
            if prev_cmd == SafetyState.ESTOPPED and new_cmd != SafetyState.ESTOPPED:
                self.get_logger().info("Remote controller detected")
            self.get_logger().info(f"Cmd_vel safety state transition: {prev_cmd.value} -> {new_cmd.value}")

        # Handle winch state transitions (logging only, no e-stop)
        if prev_winch != new_winch:
            self.get_logger().info(f"Winch safety state transition: {prev_winch.value} -> {new_winch.value}")

        # Only apply safety states when they actually change
        if prev_cmd != new_cmd:
            self._apply_safety_state_cmd_vel(new_cmd)
        if prev_winch != new_winch:
            self._apply_safety_state_winch(new_winch)

        # Brake commands only allowed when cmd_vel is ACTIVE (deadman pressed)
        if new_cmd == SafetyState.ACTIVE:
            with self.driver_lock:
                self.driver.set_brake(msg.brake)

        # Light commands allowed in both READY and ACTIVE states (for cmd_vel state machine)
        if new_cmd in (SafetyState.READY, SafetyState.ACTIVE):
            with self.driver_lock:
                # Light commands
                if hasattr(msg, "light_state"):
                    from .mtt_driver import LightState

                    if msg.light_state == 1:
                        self.driver.set_light_state(LightState.On)
                    else:
                        self.driver.set_light_state(LightState.Off)

        # Winch commands only allowed when winch safety is ACTIVE
        if new_winch == SafetyState.ACTIVE:
            with self.driver_lock:
                # Winch commands
                if msg.winch_command == MttAuxCommand.WINCH_IN:
                    self.driver.set_winch_state(WinchState.WinchIn)
                elif msg.winch_command == MttAuxCommand.WINCH_OUT:
                    self.driver.set_winch_state(WinchState.WinchOut)
                else:
                    self.driver.set_winch_state(WinchState.WinchNeutral)
        # Note: winch neutral enforcement is handled in _apply_safety_state_winch

    def control_loop(self):
        """Main control loop - publishes tachometer data."""
        with self.driver_lock:
            self._publish_vehicle_data()

    def _publish_vehicle_data(self):
        """Publish tachometer and vehicle status snapshots from the driver."""
        try:
            tacho = self.driver.get_tachometer_snapshot()
            # Tachometer message
            tmsg = MttTachometerData()
            tmsg.header.stamp = self.get_clock().now().to_msg()
            tmsg.main_sensor_temp_a = float(tacho.main_sensor_temp_a)
            tmsg.main_sensor_temp_b = float(tacho.main_sensor_temp_b)
            tmsg.tachometer_instant = int(tacho.tachometer_instant)
            tmsg.tachometer_cumulative = int(tacho.tachometer_cumulative)
            tmsg.speed_ms = float(self.driver._get_current_speed_ms())
            tmsg.speed_kmh = float(self.driver._get_current_speed_kmh())
            # Distance in km from driver snapshot (convert meters to km)
            tmsg.distance_km = 0.0
            try:
                if hasattr(self.driver, "get_odometry_snapshot"):
                    odom = self.driver.get_odometry_snapshot()
                    abs_m = float(odom.get("absolute_distance_m", 0.0))
                    if abs_m > 0.0:
                        tmsg.distance_km = abs_m / 1000.0
            except Exception:
                # Leave as 0.0 if snapshot not available
                pass
            # Direction
            if hasattr(self.driver, "current_direction") and self.driver.current_direction is not None:
                tmsg.direction = self.driver.current_direction.name
            else:
                tmsg.direction = "Unknown"
            # Articulation angle (if modeled) - currently unavailable from driver; set 0.0
            tmsg.articulation_angle_rad = 0.0
            self.tachometer_pub.publish(tmsg)

            # Vehicle status aggregate
            vmsg = MttVehicleStatus()
            vmsg.header = tmsg.header
            vmsg.speed_ms = tmsg.speed_ms
            vmsg.speed_kmh = tmsg.speed_kmh
            vmsg.distance_km = tmsg.distance_km
            vmsg.direction = tmsg.direction
            vmsg.temperature_a = tmsg.main_sensor_temp_a
            vmsg.temperature_b = tmsg.main_sensor_temp_b
            vmsg.steer_position = int(self.driver.steer_value or 0)
            vmsg.tachometer_instant = tmsg.tachometer_instant
            vmsg.tachometer_cumulative = tmsg.tachometer_cumulative
            # System status flags
            try:
                vmsg.emergency_stop_active = self.driver.get_security_switch_state() == SecuritySwitchState.SafetyLocked
            except Exception:
                vmsg.emergency_stop_active = False
            vmsg.remote_connected = (
                self.last_remote_command_time is not None
                and (time.monotonic() - self.last_remote_command_time) < self.remote_timeout_seconds
            )
            self.vehicle_status_pub.publish(vmsg)
        except Exception as e:
            self.get_logger().debug(f"Publish vehicle data failed: {e}")

    def _set_driving_mode(self, mode: int):
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
            mode_msg.mode = int(mode)
            # Optionally include parameters string for geometry updates (left blank by default)
            mode_msg.mode_parameters = ""
            try:
                self.driving_mode_pub.publish(mode_msg)
                self.get_logger().info(f"Published driving mode command: {mode_msg.mode}")
            except Exception as e:
                self.get_logger().error(f"Failed to publish driving mode: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MTTRosWrapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Process terminated externally (e.g., SIGTERM); ignore
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Context may already be shut down
            pass
