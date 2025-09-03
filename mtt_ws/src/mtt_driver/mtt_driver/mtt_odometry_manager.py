#!/usr/bin/env python3
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
from __future__ import annotations

from abc import ABC, abstractmethod
from enum import IntEnum
from typing import Optional, Dict, Any
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from mtt_msgs.msg import MttTachometerData, MttDrivingMode
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster  # added
from geometry_msgs.msg import TransformStamped  # added


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
    """Simple 1D odometry that relays absolute distance with direction."""

    def __init__(self) -> None:
        self.x = 0.0
        self.last_abs_m: Optional[float] = None  # first-sample sentinel

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
        # Current absolute distance after unit+scale adjustment
        cur_abs = float(getattr(msg, "distance_km", 0.0)) * distance_multiplier

        if self.last_abs_m is None:
            # Seed, no initial jump
            self.last_abs_m = cur_abs
            delta = 0.0
        else:
            delta = cur_abs - self.last_abs_m
            # Detect wrap/reset (large negative)
            if delta < -abs(wrap_reset_threshold_m):
                # Re-seed without moving
                self.last_abs_m = cur_abs
                delta = 0.0

        sign = self._norm_direction(getattr(msg, "direction", None))
        self.x += sign * delta
        self.last_abs_m = cur_abs

        # Pose (1D along x)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        # Velocity (signed)
        speed_ms = float(getattr(msg, "speed_ms", 0.0))
        odom.twist.twist.linear.x = sign * abs(speed_ms)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        self._apply_default_covariances(odom)
        return odom

    def export_state(self) -> Dict[str, Any]:
        return {"x": self.x, "last_abs_m": self.last_abs_m}

    def import_state(self, state: Dict[str, Any]) -> None:
        self.x = float(state.get("x", 0.0))
        last = state.get("last_abs_m", None)
        self.last_abs_m = float(last) if last is not None else None

    def reset_odometry(self) -> None:
        self.x = 0.0
        self.last_abs_m = None

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

    def __init__(self, track_width_m: float = 1.0) -> None:
        # 2D pose
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0  # yaw
        # Absolute distance per side (m)
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
    ) -> Odometry:
        odom = self._odom_with_stamp(msg, odom_frame, base_frame)

        has_L = hasattr(msg, "left_distance_km") or hasattr(msg, "left_speed_ms")
        has_R = hasattr(msg, "right_distance_km") or hasattr(msg, "right_speed_ms")

        if has_L and has_R:
            # Prefer distance integration if absolute counters exist
            if hasattr(msg, "left_distance_km") and hasattr(msg, "right_distance_km"):
                cur_L = float(getattr(msg, "left_distance_km", 0.0)) * distance_multiplier
                cur_R = float(getattr(msg, "right_distance_km", 0.0)) * distance_multiplier

                if self.last_L_m is None or self.last_R_m is None:
                    self.last_L_m, self.last_R_m = cur_L, cur_R
                    dL = dR = 0.0
                else:
                    dL = cur_L - self.last_L_m
                    dR = cur_R - self.last_R_m
                    # Wrap/reset detection per side
                    if dL < -abs(wrap_reset_threshold_m):
                        dL = 0.0
                        self.last_L_m = cur_L
                    if dR < -abs(wrap_reset_threshold_m):
                        dR = 0.0
                        self.last_R_m = cur_R

                self.last_L_m, self.last_R_m = cur_L, cur_R

            else:
                # Instantaneous speeds only; estimate dt from header stamps is non-trivial here.
                # We keep positions as-is and only set twist from speeds.
                dL = dR = 0.0

            # Integrate differential kinematics (if any delta exists)
            dS = 0.5 * (dL + dR)
            dTh = (dR - dL) / max(self.track_width_m, 1e-6)

            if abs(dTh) < 1e-9:
                # Straight-ish
                self.x += dS * math.cos(self.th)
                self.y += dS * math.sin(self.th)
            else:
                # Arc integration
                R = dS / dTh  # instantaneous curvature radius
                self.x += R * (math.sin(self.th + dTh) - math.sin(self.th))
                self.y -= R * (math.cos(self.th + dTh) - math.cos(self.th))
                self.th = self.th + dTh  # keep unbounded yaw

            # Pose
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            # Convert yaw to quaternion (no roll/pitch)
            half = 0.5 * self.th
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = math.sin(half)
            odom.pose.pose.orientation.w = math.cos(half)

            # Twist: if instantaneous speeds exist, compute signed linear & yaw rate
            if hasattr(msg, "left_speed_ms") and hasattr(msg, "right_speed_ms"):
                vL = float(getattr(msg, "left_speed_ms", 0.0))
                vR = float(getattr(msg, "right_speed_ms", 0.0))
                v = 0.5 * (vL + vR)
                w = (vR - vL) / max(self.track_width_m, 1e-6)
                odom.twist.twist.linear.x = v
                odom.twist.twist.linear.y = 0.0
                odom.twist.twist.angular.z = w
            else:
                # Fallback: signed speed if only aggregate is available
                sign = self._norm_direction(getattr(msg, "direction", None))
                v = sign * abs(float(getattr(msg, "speed_ms", 0.0)))
                odom.twist.twist.linear.x = v
                odom.twist.twist.angular.z = 0.0

            self._apply_default_covariances(odom)
            return odom

        # ---- Fallback: behave like SingleTrailer ---- #
        cur_abs = float(getattr(msg, "distance_km", 0.0)) * distance_multiplier
        if self.last_abs_m is None:
            self.last_abs_m = cur_abs
            delta = 0.0
        else:
            delta = cur_abs - self.last_abs_m
            if delta < -abs(wrap_reset_threshold_m):
                self.last_abs_m = cur_abs
                delta = 0.0
        sign = self._norm_direction(getattr(msg, "direction", None))
        self.x += sign * delta
        self.last_abs_m = cur_abs

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        speed_ms = float(getattr(msg, "speed_ms", 0.0))
        odom.twist.twist.linear.x = sign * abs(speed_ms)
        odom.twist.twist.angular.z = 0.0
        self._apply_default_covariances(odom)
        return odom

    def export_state(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "th": self.th,
            "last_L_m": self.last_L_m,
            "last_R_m": self.last_R_m,
            "track_width_m": self.track_width_m,
            "last_abs_m": self.last_abs_m,
        }

    def import_state(self, state: Dict[str, Any]) -> None:
        self.x = float(state.get("x", 0.0))
        self.y = float(state.get("y", 0.0))
        self.th = float(state.get("th", 0.0))
        self.track_width_m = float(state.get("track_width_m", self.track_width_m))
        self.last_L_m = float(state["last_L_m"]) if state.get("last_L_m") is not None else None
        self.last_R_m = float(state["last_R_m"]) if state.get("last_R_m") is not None else None
        self.last_abs_m = float(state["last_abs_m"]) if state.get("last_abs_m") is not None else None

    def reset_odometry(self) -> None:
        self.x = self.y = self.th = 0.0
        self.last_L_m = self.last_R_m = None
        self.last_abs_m = None

    def get_mode_name(self) -> str:
        return "Dual Differential"


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
