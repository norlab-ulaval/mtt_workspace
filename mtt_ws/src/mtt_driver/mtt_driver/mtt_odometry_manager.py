#!/usr/bin/env python3

from __future__ import annotations
from abc import ABC, abstractmethod
from enum import IntEnum
from typing import Optional, Dict, Any
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from mtt_msgs.msg import MttTachometerData, MttDrivingMode
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

"""
MTT-154 Multi-Mode Odometry Manager

Provides three driving modes with live switching:
- Single Trailer: 1D odometry for basic tractor+trailer
- Dual Differential: 2D skid-steer when left/right sensors available
- Dual Serpentine: 2D bicycle model with articulation angle

Features:
- State preservation across mode switches (no odometry jumps)
- First-sample initialization eliminates startup discontinuities
- Parametric geometry configuration (track width, wheelbase)
- Sensor-optimized QoS and odometer wrap/reset handling
- Service interface for odometry reset
"""


# --------------------------- Modes --------------------------- #
class DrivingMode(IntEnum):
    SINGLE_TRAILER = 0  # Single tractor with trailer (slip + central joint)
    DUAL_DIFFERENTIAL = 1  # Two tractors side-by-side (skid steer)
    DUAL_SERPENTINE = 2  # Two tractors front/back (articulated)


# ---------------------- Interface & Utils -------------------- #
class OdometryInterface(ABC):
    @abstractmethod
    def calculate_odometry(
        self,
        msg: MttTachometerData,
        *,
        odom_frame: str,
        base_frame: str,
        distance_multiplier: float,
        wrap_reset_threshold_m: float,
        angular_velocity: float = 0.0,
    ) -> Odometry:
        pass

    @abstractmethod
    def export_state(self) -> Dict[str, Any]:
        """Return a serializable state for transfer across mode switches."""
        pass

    @abstractmethod
    def import_state(self, state: Dict[str, Any]) -> None:
        pass

    @abstractmethod
    def reset_odometry(self) -> None:
        pass

    @abstractmethod
    def get_mode_name(self) -> str:
        pass

    # --- helpers --- #
    @staticmethod
    def _odom_with_stamp(msg: MttTachometerData, odom_frame: str, base_frame: str) -> Odometry:
        odom = Odometry()
        # Prefer sensor time if available
        try:
            odom.header.stamp = msg.header.stamp
        except Exception:
            # Fallback is set by caller if needed
            pass
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        return odom

    @staticmethod
    def _apply_default_covariances(odom: Odometry) -> None:
        # Small variance on x position & vx; large elsewhere (uninformative)
        big = 1e6
        small = 1e-3
        # pose: [x, y, z, roll, pitch, yaw]
        cov_p = [big] * 36
        cov_p[0] = small  # x
        cov_p[7] = big  # y
        cov_p[14] = big  # z
        cov_p[35] = big  # yaw
        odom.pose.covariance = cov_p

        cov_t = [big] * 36
        cov_t[0] = small  # vx
        cov_t[7] = big  # vy
        cov_t[14] = big  # vz
        cov_t[35] = big  # wyaw
        odom.twist.covariance = cov_t

    @staticmethod
    def _norm_direction(direction_field: Optional[str]) -> int:
        """Return +1 for forward, -1 for reverse (default forward if unknown)."""
        if not direction_field:
            return +1
        d = str(direction_field).strip().lower()
        if d in ("reverse", "backward", "rev"):
            return -1
        return +1


# ----------------------- Implementations --------------------- #
class SingleTrailerOdometry(OdometryInterface):
    """Simple 1D odometry that relays absolute distance with direction and steering."""

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_abs_m: Optional[float] = None  # first-sample sentinel
        self.last_time: Optional[float] = None

    def calculate_odometry(
        self,
        msg: MttTachometerData,
        *,
        odom_frame: str,
        base_frame: str,
        distance_multiplier: float,
        wrap_reset_threshold_m: float,
        angular_velocity: float = 0.0,
    ) -> Odometry:
        odom = self._odom_with_stamp(msg, odom_frame, base_frame)
        # Current absolute distance after unit+scale adjustment
        cur_abs = float(getattr(msg, "distance_km", 0.0)) * distance_multiplier
        
        # Get current time for dt calculation
        current_time = time.time()

        if self.last_abs_m is None:
            # Seed, no initial jump
            self.last_abs_m = cur_abs
            self.last_time = current_time
            delta = 0.0
            dt = 0.02  # Default dt for first sample
        else:
            delta = cur_abs - self.last_abs_m
            dt = current_time - self.last_time if self.last_time else 0.02
            dt = max(dt, 0.001)  # Prevent division by zero
            
            # Detect wrap/reset (large negative)
            if delta < -abs(wrap_reset_threshold_m):
                # Re-seed without moving
                self.last_abs_m = cur_abs
                self.last_time = current_time
                delta = 0.0

        sign = self._norm_direction(getattr(msg, "direction", None))
        linear_distance = sign * delta
        
        # Update yaw with angular velocity
        self.yaw += angular_velocity * dt
        
        # Update position using current yaw
        self.x += linear_distance * math.cos(self.yaw)
        self.y += linear_distance * math.sin(self.yaw)
        
        self.last_abs_m = cur_abs
        self.last_time = current_time

        # Pose (2D with rotation)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        quat = self._yaw_to_quaternion(self.yaw)
        odom.pose.pose.orientation = quat

        # Velocity (with angular)
        speed_ms = float(getattr(msg, "speed_ms", 0.0))
        odom.twist.twist.linear.x = sign * abs(speed_ms)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        self._apply_default_covariances(odom)
        return odom

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

    def export_state(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y, 
            "yaw": self.yaw,
            "last_abs_m": self.last_abs_m,
            "last_time": self.last_time
        }

    def import_state(self, state: Dict[str, Any]) -> None:
        self.x = float(state.get("x", 0.0))
        self.y = float(state.get("y", 0.0))
        self.yaw = float(state.get("yaw", 0.0))
        last = state.get("last_abs_m", None)
        self.last_abs_m = float(last) if last is not None else None
        last_time = state.get("last_time", None)
        self.last_time = float(last_time) if last_time is not None else None

    def reset_odometry(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_abs_m = None
        self.last_time = None

    def get_mode_name(self) -> str:
        return "Single Trailer"


class DualDifferentialOdometry(OdometryInterface):
    """Skid-steer style. If left/right signals are present, integrates (x,y,theta);
    otherwise falls back to 1D like SingleTrailer.

    Optional message fields used if available:
      - left_distance_km, right_distance_km  (absolute)
      - left_speed_ms, right_speed_ms        (instantaneous)
    Params used (provided by manager): track_width_m
    """

    def __init__(self, *, track_width_m: float, **kwargs) -> None:
        super().__init__()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_L_m: Optional[float] = None
        self.last_R_m: Optional[float] = None
        self.track_width_m = float(track_width_m)
        # Fallback 1D state
        self.last_abs_m: Optional[float] = None

    def calculate_odometry(
        self,
        msg: MttTachometerData,
        *,
        odom_frame: str,
        base_frame: str,
        distance_multiplier: float,
        wrap_reset_threshold_m: float,
        angular_velocity: float = 0.0,
    ) -> Odometry:
        # For dual differential, we'd need separate left/right data
        # Since MttTachometerData only has single tachometer, fallback to 1D mode
        abs_distance_m = msg.distance_km * 1000.0 * distance_multiplier
        
        # Handle wrap-around
        if self.last_abs_m is not None and abs(abs_distance_m - self.last_abs_m) > wrap_reset_threshold_m:
            self.last_abs_m = None
        
        if self.last_abs_m is None:
            self.last_abs_m = abs_distance_m
            return self._create_simple_odometry_msg(odom_frame, base_frame, msg.header.stamp)
        
        # Calculate distance delta
        delta_distance = abs_distance_m - self.last_abs_m
        self.last_abs_m = abs_distance_m
        
        # Update position using angular velocity for steering
        self.theta += angular_velocity * (1.0 / 50.0)  # Assuming 50Hz update rate
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)
        
        return self._create_odometry_msg(odom_frame, base_frame, msg.header.stamp)
    
    def _create_odometry_msg(self, odom_frame: str, base_frame: str, stamp) -> Odometry:
        """Create odometry message with current pose"""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation
        quat = self._yaw_to_quaternion(self.theta)
        odom.pose.pose.orientation.x = quat.x
        odom.pose.pose.orientation.y = quat.y 
        odom.pose.pose.orientation.z = quat.z
        odom.pose.pose.orientation.w = quat.w
        
        return odom
    
    def _create_simple_odometry_msg(self, odom_frame: str, base_frame: str, stamp) -> Odometry:
        """Create simple odometry message for initialization"""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.orientation.w = 1.0  # Identity quaternion
        return odom
    
    def get_mode_name(self) -> str:
        return "Dual Differential"
    
    def export_state(self) -> dict:
        return {
            'x': self.x,
            'y': self.y, 
            'theta': self.theta,
            'last_abs_m': self.last_abs_m,
            'last_L_m': self.last_L_m,
            'last_R_m': self.last_R_m
        }
    
    def import_state(self, state: dict) -> None:
        self.x = state.get('x', 0.0)
        self.y = state.get('y', 0.0)
        self.theta = state.get('theta', 0.0)
        self.last_abs_m = state.get('last_abs_m')
        self.last_L_m = state.get('last_L_m')
        self.last_R_m = state.get('last_R_m')
    
    def reset_odometry(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_abs_m = None
        self.last_L_m = None
        self.last_R_m = None
    
    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat


class DualSerpentineOdometry(OdometryInterface):
    """Articulated front-back pair. If an articulation angle is available in the msg (e.g.,
    `articulation_angle_rad`), integrate like a simple car-like model. Otherwise, 1D fallback.
    """

    def __init__(self, wheelbase_m: float = 2.0) -> None:
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.wheelbase_m = float(wheelbase_m)
        self.last_abs_m: Optional[float] = None

    def calculate_odometry(
        self,
        msg: MttTachometerData,
        *,
        odom_frame: str,
        base_frame: str,
        distance_multiplier: float,
        wrap_reset_threshold_m: float,
    ) -> Odometry:
        odom = self._odom_with_stamp(msg, odom_frame, base_frame)

        # Distance delta (1D along the articulation frame)
        cur_abs = float(getattr(msg, "distance_km", 0.0)) * distance_multiplier
        if self.last_abs_m is None:
            self.last_abs_m = cur_abs
            ds = 0.0
        else:
            ds = cur_abs - self.last_abs_m
            if ds < -abs(wrap_reset_threshold_m):
                self.last_abs_m = cur_abs
                ds = 0.0
        self.last_abs_m = cur_abs

        sign = self._norm_direction(getattr(msg, "direction", None))
        ds *= sign

        # Use articulation angle if available; treat as steer angle surrogate
        steer = float(getattr(msg, "articulation_angle_rad", 0.0)) if hasattr(msg, "articulation_angle_rad") else 0.0

        if abs(steer) < 1e-9 or self.wheelbase_m < 1e-9:
            # Straight
            self.x += ds * math.cos(self.th)
            self.y += ds * math.sin(self.th)
        else:
            # Bicycle model integration
            dth = math.tan(steer) / self.wheelbase_m * ds
            R = ds / dth if abs(dth) > 1e-9 else float("inf")
            if math.isfinite(R):
                self.x += R * (math.sin(self.th + dth) - math.sin(self.th))
                self.y -= R * (math.cos(self.th + dth) - math.cos(self.th))
            else:
                self.x += ds * math.cos(self.th)
                self.y += ds * math.sin(self.th)
            self.th += dth

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        half = 0.5 * self.th
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)

        # Velocity (signed forward speed if available)
        v = sign * abs(float(getattr(msg, "speed_ms", 0.0)))
        odom.twist.twist.linear.x = v
        # yaw rate if steer present
        odom.twist.twist.angular.z = (v / max(self.wheelbase_m, 1e-6)) * math.tan(steer) if abs(steer) > 0 else 0.0

        self._apply_default_covariances(odom)
        return odom

    def export_state(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "th": self.th,
            "wheelbase_m": self.wheelbase_m,
            "last_abs_m": self.last_abs_m,
        }

    def import_state(self, state: Dict[str, Any]) -> None:
        self.x = float(state.get("x", 0.0))
        self.y = float(state.get("y", 0.0))
        self.th = float(state.get("th", 0.0))
        self.wheelbase_m = float(state.get("wheelbase_m", self.wheelbase_m))
        self.last_abs_m = float(state["last_abs_m"]) if state.get("last_abs_m") is not None else None

    def reset_odometry(self) -> None:
        self.x = self.y = self.th = 0.0
        self.last_abs_m = None

    def get_mode_name(self) -> str:
        return "Dual Serpentine"


# --------------------------- Factory ------------------------- #
class OdometryFactory:
    @staticmethod
    def create_odometry(mode: DrivingMode, *, track_width_m: float, wheelbase_m: float) -> OdometryInterface:
        if mode == DrivingMode.SINGLE_TRAILER:
            return SingleTrailerOdometry()
        if mode == DrivingMode.DUAL_DIFFERENTIAL:
            return DualDifferentialOdometry(track_width_m=track_width_m)
        if mode == DrivingMode.DUAL_SERPENTINE:
            return DualSerpentineOdometry(wheelbase_m=wheelbase_m)
        raise ValueError(f"Unsupported driving mode: {mode}")


# ----------------------------- Node -------------------------- #
class MttOdometryManager(Node):
    def __init__(self) -> None:
        super().__init__("mtt_odometry_manager")

        # Parameters
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "mtt_base_link")
        self.declare_parameter("tachometer_topic", "/mtt_tachometer")
        self.declare_parameter("odometry_topic", "/mtt_odometry")
        self.declare_parameter("mode_topic", "/mtt_driving_mode")
        self.declare_parameter("wrap_reset_threshold_m", 1000.0)
        self.declare_parameter("track_width_m", 1.0)
        self.declare_parameter("wheelbase_m", 2.0)
        # New: distance unit & scaling
        self.declare_parameter("distance_unit", "km")  # 'km' or 'm'
        self.declare_parameter("distance_scale", 1.0)  # additional multiplicative scaling

        self.odom_frame: str = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.base_frame: str = self.get_parameter("base_frame").get_parameter_value().string_value
        self.tachometer_topic: str = self.get_parameter("tachometer_topic").get_parameter_value().string_value
        self.odometry_topic: str = self.get_parameter("odometry_topic").get_parameter_value().string_value
        self.mode_topic: str = self.get_parameter("mode_topic").get_parameter_value().string_value
        self.wrap_reset_threshold_m: float = (
            self.get_parameter("wrap_reset_threshold_m").get_parameter_value().double_value
        )
        self.track_width_m: float = self.get_parameter("track_width_m").get_parameter_value().double_value
        self.wheelbase_m: float = self.get_parameter("wheelbase_m").get_parameter_value().double_value
        distance_unit: str = self.get_parameter("distance_unit").get_parameter_value().string_value.lower()
        distance_scale: float = self.get_parameter("distance_scale").get_parameter_value().double_value
        base_multiplier = 1000.0 if distance_unit == "km" else 1.0
        self.distance_multiplier = base_multiplier * distance_scale
        self.get_logger().info(
            f"Distance conversion: unit={distance_unit} scale={distance_scale} -> multiplier={self.distance_multiplier}"
        )

        # Mode
        self.current_mode = DrivingMode.SINGLE_TRAILER
        self.odometry_calculator: OdometryInterface = OdometryFactory.create_odometry(
            self.current_mode, track_width_m=self.track_width_m, wheelbase_m=self.wheelbase_m
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, 10)

        # Subscriber with sensor-like QoS
        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = HistoryPolicy.KEEP_LAST
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        self.tacho_sub = self.create_subscription(
            MttTachometerData,
            self.tachometer_topic,
            self.tachometer_callback,
            sensor_qos,
        )

        # Driving mode subscriber (reliable)
        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.history = HistoryPolicy.KEEP_LAST
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        self.mode_sub = self.create_subscription(
            MttDrivingMode,
            self.mode_topic,
            self.mode_change_callback,
            reliable_qos,
        )

        # Services
        self.reset_srv = self.create_service(Trigger, "/mtt/reset_odometry", self.reset_odometry_cb)

        # Angular velocity from cmd_vel for steering odometry
        self.current_angular_vel = 0.0
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10  # Standard QoS for commands
        )

        self.get_logger().info(
            f"MTT Odometry Manager initialized - Mode: {self.odometry_calculator.get_mode_name()} | "
            f"odom_frame={self.odom_frame}, base_frame={self.base_frame}, pub={self.odometry_topic}, sub={self.tachometer_topic}, mode_sub={self.mode_topic}"
        )
        self.tf_broadcaster = TransformBroadcaster(self)  # added

    # ----------------- Callbacks ----------------- #
    def tacho_sub_failed_time_fallback(self, odom: Odometry) -> None:
        # Fallback to node time if msg had no timestamp (rare)
        if not odom.header.stamp.sec and not odom.header.stamp.nanosec:
            odom.header.stamp = self.get_clock().now().to_msg()

    def tachometer_callback(self, msg: MttTachometerData) -> None:
        try:
            odom = self.odometry_calculator.calculate_odometry(
                msg,
                odom_frame=self.odom_frame,
                base_frame=self.base_frame,
                distance_multiplier=self.distance_multiplier,
                wrap_reset_threshold_m=self.wrap_reset_threshold_m,
                angular_velocity=self.current_angular_vel,
            )
            self.tacho_sub_failed_time_fallback(odom)
            self.odom_pub.publish(odom)
            # broadcast TF transform odom->base_frame
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = odom.pose.pose.position.z
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f"Odometry calculation failed: {e}")

    def reset_odometry_cb(self, request, response):
        try:
            self.odometry_calculator.reset_odometry()
            response.success = True
            response.message = f"Odometry reset in mode: {self.odometry_calculator.get_mode_name()}"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Store current angular velocity for steering odometry calculations"""
        self.current_angular_vel = msg.angular.z

    # --------------- Mode switching --------------- #
    def _transfer_state(self, src: OdometryInterface, dst: OdometryInterface) -> None:
        try:
            state = src.export_state()
            dst.import_state(state)
        except Exception as e:
            self.get_logger().warn(f"State transfer failed ({e}); continuing with fresh state")

    def _parse_mode_parameters(self, params: str) -> Dict[str, float]:
        """Parse key=value pairs (comma/space separated). Example: "track=1.25 wheelbase=2.3"""
        out: Dict[str, float] = {}
        if not params:
            return out
        for token in params.replace(",", " ").split():
            if "=" in token:
                k, v = token.split("=", 1)
                try:
                    out[k.strip().lower()] = float(v)
                except ValueError:
                    pass
        return out

    def mode_change_callback(self, msg: MttDrivingMode) -> None:
        """Handle driving mode changes from the wrapper.
        Accepts optional `mode_parameters` such as:
          - "track=1.1" to update track_width_m (DualDifferential)
          - "wheelbase=2.3" to update wheelbase_m (DualSerpentine)
        """
        try:
            new_mode = DrivingMode(int(msg.mode))
        except Exception:
            self.get_logger().error(f"Invalid mode value: {msg.mode}")
            return

        # Update geometry parameters if provided
        kv = self._parse_mode_parameters(getattr(msg, "mode_parameters", ""))
        if "track" in kv:
            self.track_width_m = float(kv["track"])
        if "wheelbase" in kv:
            self.wheelbase_m = float(kv["wheelbase"])

        if new_mode == self.current_mode:
            # Mode unchanged; just update calculator params if relevant
            if isinstance(self.odometry_calculator, DualDifferentialOdometry):
                self.odometry_calculator.track_width_m = self.track_width_m
            if isinstance(self.odometry_calculator, DualSerpentineOdometry):
                self.odometry_calculator.wheelbase_m = self.wheelbase_m
            return

        self.get_logger().info(
            f"Mode change requested: {self.current_mode.name} → {new_mode.name} (track={self.track_width_m:.3f}, wheelbase={self.wheelbase_m:.3f})"
        )

        try:
            new_calc = OdometryFactory.create_odometry(
                new_mode, track_width_m=self.track_width_m, wheelbase_m=self.wheelbase_m
            )
            self._transfer_state(self.odometry_calculator, new_calc)
            self.current_mode = new_mode
            self.odometry_calculator = new_calc
            self.get_logger().info(f"Switched to {self.odometry_calculator.get_mode_name()} mode (state preserved)")
        except Exception as e:
            self.get_logger().error(f"Mode change failed: {e}")

    def manual_mode_change(self, mode: int) -> bool:
        try:
            new_mode = DrivingMode(mode)
            if new_mode != self.current_mode:
                self.get_logger().info(
                    f"Manual mode switch: {self.odometry_calculator.get_mode_name()} → {new_mode.name}"
                )
                new_calc = OdometryFactory.create_odometry(
                    new_mode, track_width_m=self.track_width_m, wheelbase_m=self.wheelbase_m
                )
                self._transfer_state(self.odometry_calculator, new_calc)
                self.current_mode = new_mode
                self.odometry_calculator = new_calc
                self.get_logger().info(f"Switched to {self.odometry_calculator.get_mode_name()} mode (state preserved)")
                return True
            return False
        except ValueError as e:
            self.get_logger().error(f"Invalid driving mode: {mode} - {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"Mode change failed: {e}")
            return False

    # TF broadcasting can be added here if coordinate frame transforms are needed


# ----------------------------- Main -------------------------- #
def main(args=None) -> None:
    rclpy.init(args=args)
    node = MttOdometryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
