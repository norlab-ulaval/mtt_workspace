#!/usr/bin/env python3
"""Real-time coverage & quality monitor for the ice-rink M0-M5 identification
session (companion to mtt_experiment_conductor.py).

Why this exists: the conductor plays a command profile, but only live sensor
feedback can confirm the (speed x articulation x direction) grid was actually
achieved (commands are not motion) and that the ground-truth chain is trust-
worthy enough to support the paper's claims. This node reuses the gating
thresholds and sign conventions of the frozen command-only benchmark
(artifacts/m5_research_review_2026-07-13/command_only/all_models_reanalysis.py)
so a fitted result from this session stays consistent with the existing
identification code, without importing that offline-analysis tree directly.

Measurement choices (deliberate, see plan review):
  - kappa = yaw_rate / ground_speed is computed from IMU yaw-rate and a
    pose-source-derived ground speed, NOT from a position track directly: a
    flat 60x26 m rink with distant boards is a poor aperture for lidar-ICP
    position accuracy along the long axis, while IMU yaw is robust.
  - Pose source for real-time ground_speed/yaw_rate is Isaac ROS Visual SLAM
    (/isaac/vslam/odometry), PRIMARY as of 2026-07-28, with the Norlab ICP
    mapper (/mapping/icp_odom) kept as a live fallback + cross-check, NOT
    removed. Rationale: cuVSLAM (GPU-accelerated stereo/RGBD VO) publishes
    at camera rate with much lower latency than the ICP mapper's point-cloud
    registration, which matters for a live dashboard driving go/no-go calls
    mid-session; ICP remains the trusted, slower, globally-consistent source
    used by scripts/analyze_ice_session.py for the actual offline fit
    (deliberately UNCHANGED there -- this file only affects the live
    monitor). If /isaac/vslam/odometry is stale or never publishes (e.g. the
    live_robot isaac_vslam compose service wasn't started alongside this
    session), _select_pose_source() falls back to ICP automatically -- no
    crash, no silent bad data, just a coarser live picture until the next
    offline pass corrects it.
  - Ground speed (not tachometer) is preferred for kappa, because belt slip
    on ice makes the tachometer over-read speed, which would fabricate a
    curvature deficit. Tachometer vs pose-source speed slip is monitored
    live (the "belt slip" flag) using the same slip_ratio construction as
    the frozen benchmark's summarize_signals().
  - The standstill split ratio (dtheta1/dphi at v=0) bypasses the |v|>=0.15
    speed gate entirely, since kappa is undefined there but the split ratio
    is not.
"""

from __future__ import annotations

import json
import math
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, Optional, Tuple

import numpy as np

import rclpy
from mtt_msgs.msg import MttTachometerData
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Float64, String

try:
    from zone_map import ZoneMap  # scripts/zone_map.py, same directory
except ImportError:
    ZoneMap = None  # zone display stays unavailable; never blocks the rest of the monitor


RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
    )


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class SegmentRecord:
    kind: str = ""
    tier: str = ""
    label: str = ""
    meta: dict = field(default_factory=dict)
    first_seen_wall: float = 0.0
    last_seen_wall: float = 0.0
    max_elapsed: float = 0.0
    duration: float = 0.0
    completed: bool = False


# Canonical grouping mirroring the paper's series (A/B/B-split/D/speed-sweep/E).
KIND_TO_SERIES = {
    "phi_step": "A - actuator",
    "speed_step": "A - actuator",
    "hold_arc": "B - phi->kappa map",
    "phi_ramp": "D/B-split - ramp & standstill split",
    "figure8": "E - held-out",
    "circle": "G - trajectory library (extended)",
    "line": "G - trajectory library (extended)",
    "prbs_phi": "E - held-out",
    "slalom_sine": "E - held-out (extended)",
    "combined_v_phi": "E - held-out (extended)",
    "stop_and_go": "E - held-out (extended)",
    "pause": "(pause)",
}


class MttExperimentMonitor(Node):
    def __init__(self) -> None:
        super().__init__("mtt_experiment_monitor")

        self.declare_parameter("tacho_topic", "mtt_tachometer")
        self.declare_parameter("articulation_topic", "/hardware/articulation_angle")
        self.declare_parameter("icp_odom_topic", "/mapping/icp_odom")
        self.declare_parameter("vslam_odom_topic", "/isaac/vslam/odometry")
        self.declare_parameter("vslam_staleness_s", 0.3)
        # Grace period before flagging "never published" -- ZED/isaac_vslam can
        # legitimately take several seconds to open the camera (see health_check.py's
        # camera group hint). Below this, silence is normal startup, not an alarm.
        self.declare_parameter("pose_source_startup_grace_s", 20.0)
        # Threshold to flag "was alive, then went silent" mid-session (distinct from
        # vslam_staleness_s, which is the much shorter 0.3s used to decide fallback
        # routing every tick -- that's normal operation, not an alarm by itself).
        self.declare_parameter("pose_source_dead_s", 8.0)
        self.declare_parameter("imu_topic", "/mti100/data")
        self.declare_parameter("segment_topic", "/mtt_experiment/segment")
        self.declare_parameter("coverage_topic", "/mtt_experiment/coverage")
        self.declare_parameter("coverage_json_path", "")
        self.declare_parameter("dashboard_rate_hz", 1.0)
        # Default +1: verified 2026-07-23 against the conductor's own commanded setpoint
        # (commanded +X deg tracks to measured +X deg on this pipeline). The -1 used in
        # the frozen offline benchmark reconciled a DIFFERENT pair of sources (July
        # hardware vs June lidar); it does not apply to this self-consistent
        # conductor+monitor pipeline. See analyze_ice_session.py's PHI_SIGN comment.
        self.declare_parameter("phi_sign", 1.0)
        self.declare_parameter("speed_gate_kappa_ms", 0.15)
        self.declare_parameter("speed_gate_rate_ms", 0.20)
        self.declare_parameter("standstill_speed_ms", 0.05)
        self.declare_parameter("kappa_abs_max_m_inv", 1.5)
        self.declare_parameter("icp_gap_max_s", 0.2)
        self.declare_parameter("icp_jump_max_m", 1.0)
        self.declare_parameter("slip_ratio_warn", 0.2)
        self.declare_parameter("v_bin_edges_ms", [-1.6, -1.0, -0.55, -0.15, 0.15, 0.55, 1.0, 1.6])
        # Extended 2026-07-27 to cover the ICE_ID_FULL/ASPHALT_ID_FULL protocol's
        # +-40 deg cells (was clipped at +-25 deg, silently folding the +-30/+-40
        # deg arcs into the edge bin -- audit finding).
        self.declare_parameter(
            "phi_bin_edges_deg",
            [-40.0, -35.0, -25.0, -17.5, -12.5, -7.5, -4.5, -1.5, 1.5, 4.5, 7.5,
             12.5, 17.5, 25.0, 35.0, 40.0],
        )
        self.declare_parameter("target_reps_per_cell", 2)
        self.declare_parameter("obstacle_stop_topic", "/mtt_obstacle/stop_requested")
        self.declare_parameter("zone_map_path", "")
        self.declare_parameter("zone_display_warn_margin_m", 2.0)
        self.declare_parameter("obstacle_status_topic", "/mtt_obstacle/hazard_status")
        self.declare_parameter("obstacle_clearance_topic", "/mtt_obstacle/front_clearance_m")

        self._phi_sign = float(self.get_parameter("phi_sign").value)
        self._v_gate_kappa = float(self.get_parameter("speed_gate_kappa_ms").value)
        self._v_gate_rate = float(self.get_parameter("speed_gate_rate_ms").value)
        self._standstill_v = float(self.get_parameter("standstill_speed_ms").value)
        self._kappa_abs_max = float(self.get_parameter("kappa_abs_max_m_inv").value)
        self._vslam_staleness_s = float(self.get_parameter("vslam_staleness_s").value)
        self._pose_source_startup_grace_s = float(self.get_parameter("pose_source_startup_grace_s").value)
        self._pose_source_dead_s = float(self.get_parameter("pose_source_dead_s").value)
        self._icp_gap_max_s = float(self.get_parameter("icp_gap_max_s").value)
        self._icp_jump_max_m = float(self.get_parameter("icp_jump_max_m").value)
        self._slip_warn = float(self.get_parameter("slip_ratio_warn").value)
        self._v_edges = np.array(self.get_parameter("v_bin_edges_ms").value, dtype=float)
        self._phi_edges = np.array(self.get_parameter("phi_bin_edges_deg").value, dtype=float)
        self._target_reps = int(self.get_parameter("target_reps_per_cell").value)
        self._zone_warn_margin_m = float(self.get_parameter("zone_display_warn_margin_m").value)
        self._zone_map: Optional[ZoneMap] = None
        zone_map_path_str = str(self.get_parameter("zone_map_path").value).strip()
        if zone_map_path_str:
            if ZoneMap is None:
                self.get_logger().error("zone_map_path set but zone_map module failed to import -- zone display DISABLED.")
            else:
                try:
                    self._zone_map = ZoneMap.load(Path(zone_map_path_str))
                    self.get_logger().info(f"Zone display ACTIVE: {zone_map_path_str}")
                except Exception as exc:
                    self.get_logger().error(f"Failed to load zone_map_path={zone_map_path_str!r}: {exc} -- zone display DISABLED.")

        # --- live raw state ---
        self._tach_v_signed: Optional[float] = None
        self._tach_stamp: Optional[float] = None
        self._phi_raw: Optional[float] = None
        self._phi_prev: Optional[float] = None
        self._phi_prev_stamp: Optional[float] = None
        self._phi_stamp: Optional[float] = None
        self._imu_yaw_rate: Optional[float] = None
        self._imu_stamp: Optional[float] = None
        self._odom_prev_pose = None
        self._odom_prev_stamp: Optional[float] = None
        self._odom_twist_populated: Optional[bool] = None
        self._ground_speed: Optional[float] = None
        self._icp_yaw_rate: Optional[float] = None
        self._icp_stamp: Optional[float] = None
        self._icp_gap_flag = False
        self._icp_jump_flag = False

        # --- Isaac VSLAM state (primary real-time pose source, see module docstring) ---
        self._vslam_prev_pose = None
        self._vslam_prev_stamp: Optional[float] = None
        self._vslam_twist_populated: Optional[bool] = None
        self._vslam_ground_speed: Optional[float] = None
        self._vslam_yaw_rate: Optional[float] = None
        self._vslam_stamp: Optional[float] = None
        self._vslam_gap_flag = False
        self._vslam_jump_flag = False
        self._active_pose_source = "none"
        self._active_pose_gap_flag = False
        self._active_pose_jump_flag = False
        self._obstacle_stop_active: Optional[bool] = None
        self._obstacle_status_text = ""
        self._obstacle_clearance_m: Optional[float] = None
        self._obstacle_stamp: Optional[float] = None
        self._node_start_wall = self._now_s()
        self._vslam_ever_seen = False
        self._icp_ever_seen = False

        # --- current active segment (from conductor) ---
        self._active_uid: Optional[str] = None
        self._segments: Dict[str, SegmentRecord] = {}

        # --- coverage grid: (v_bin, phi_bin) -> {"dwell_s": float, "uids": set} ---
        self._grid: Dict[Tuple[int, int], dict] = defaultdict(lambda: {"dwell_s": 0.0, "uids": set()})
        self._grid_last_update: Optional[float] = None

        # --- quality buffers ---
        self._yawrate_pairs: Deque[Tuple[float, float]] = deque(maxlen=1000)
        self._vslam_vs_icp_pairs: Deque[Tuple[float, float]] = deque(maxlen=1000)
        self._slip_samples: Deque[float] = deque(maxlen=2000)
        self._split_ratio_samples: Deque[float] = deque(maxlen=200)

        # --- QoS ---
        volatile_qos = QoSProfile(
            depth=20,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            MttTachometerData, str(self.get_parameter("tacho_topic").value), self._on_tacho, volatile_qos
        )
        self.create_subscription(
            Float64, str(self.get_parameter("articulation_topic").value), self._on_articulation, volatile_qos
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("icp_odom_topic").value), self._on_icp_odom, volatile_qos
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("vslam_odom_topic").value), self._on_vslam_odom, sensor_qos
        )
        self.create_subscription(Imu, str(self.get_parameter("imu_topic").value), self._on_imu, sensor_qos)
        self.create_subscription(
            String, str(self.get_parameter("segment_topic").value), self._on_segment, volatile_qos
        )
        self.create_subscription(
            Bool, str(self.get_parameter("obstacle_stop_topic").value), self._on_obstacle_stop, volatile_qos
        )
        self.create_subscription(
            String, str(self.get_parameter("obstacle_status_topic").value), self._on_obstacle_status, volatile_qos
        )
        self.create_subscription(
            Float32, str(self.get_parameter("obstacle_clearance_topic").value), self._on_obstacle_clearance, volatile_qos
        )

        self._coverage_pub = self.create_publisher(String, str(self.get_parameter("coverage_topic").value), 5)

        json_path_param = str(self.get_parameter("coverage_json_path").value)
        if json_path_param:
            self._json_path = Path(json_path_param)
        else:
            ts = int(time.time())
            self._json_path = Path.cwd() / "data" / "ice_segment_logs" / f"coverage_live_{ts}.json"
        self._json_path.parent.mkdir(parents=True, exist_ok=True)

        dashboard_hz = float(self.get_parameter("dashboard_rate_hz").value)
        self.create_timer(1.0 / max(dashboard_hz, 0.1), self._on_dashboard_tick)
        self.create_timer(0.05, self._on_fast_tick)  # 20 Hz grid/quality accumulation

        self.get_logger().info(f"Monitor ready. Coverage snapshot: {self._json_path}")

    # -- callbacks -----------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tacho(self, msg: MttTachometerData) -> None:
        sign = 1.0 if msg.direction == "Forward" else -1.0
        self._tach_v_signed = sign * msg.speed_ms
        self._tach_stamp = self._now_s()

    def _on_articulation(self, msg: Float64) -> None:
        now = self._now_s()
        if self._phi_raw is not None and self._phi_stamp is not None:
            self._phi_prev = self._phi_raw
            self._phi_prev_stamp = self._phi_stamp
        self._phi_raw = msg.data
        self._phi_stamp = now

    def _on_imu(self, msg: Imu) -> None:
        self._imu_yaw_rate = msg.angular_velocity.z
        self._imu_stamp = self._now_s()

    def _on_icp_odom(self, msg: Odometry) -> None:
        if not self._icp_ever_seen:
            self._icp_ever_seen = True
            self.get_logger().info("ICP odom (/mapping/icp_odom) first message received.")
        now = self._now_s()
        twist_v = msg.twist.twist.linear.x
        twist_w = msg.twist.twist.angular.z
        if self._odom_twist_populated is None:
            self._odom_twist_populated = abs(twist_v) > 1e-3 or abs(twist_w) > 1e-4

        fd_v = None
        fd_w = None
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self._odom_prev_pose is not None and self._odom_prev_stamp is not None:
            dt = now - self._odom_prev_stamp
            if dt > 1e-3:
                dx = pos.x - self._odom_prev_pose[0]
                dy = pos.y - self._odom_prev_pose[1]
                dist = math.hypot(dx, dy)
                if dist > self._icp_jump_max_m:
                    self._icp_jump_flag = True
                else:
                    self._icp_jump_flag = False
                heading = math.atan2(dy, dx) if dist > 1e-4 else self._odom_prev_pose[2]
                sign = 1.0 if abs(wrap_to_pi(heading - self._odom_prev_pose[2])) < math.pi / 2 else -1.0
                fd_v = sign * dist / dt
                fd_w = wrap_to_pi(yaw - self._odom_prev_pose[2]) / dt
                self._icp_gap_flag = dt > self._icp_gap_max_s
        self._odom_prev_pose = (pos.x, pos.y, yaw)
        self._odom_prev_stamp = now
        self._icp_stamp = now

        self._ground_speed = twist_v if self._odom_twist_populated else fd_v
        self._icp_yaw_rate = twist_w if self._odom_twist_populated else fd_w

    def _on_vslam_odom(self, msg: Odometry) -> None:
        """Isaac ROS Visual SLAM (/isaac/vslam/odometry) -- see module docstring.
        Structurally identical to _on_icp_odom (same nav_msgs/Odometry contract,
        same twist-populated-vs-finite-difference logic) but keeps fully separate
        state so a stale/absent VSLAM feed can never contaminate the ICP fallback."""
        if not self._vslam_ever_seen:
            self._vslam_ever_seen = True
            self.get_logger().info("VSLAM odom (/isaac/vslam/odometry) first message received.")
        now = self._now_s()
        twist_v = msg.twist.twist.linear.x
        twist_w = msg.twist.twist.angular.z
        if self._vslam_twist_populated is None:
            self._vslam_twist_populated = abs(twist_v) > 1e-3 or abs(twist_w) > 1e-4

        fd_v = None
        fd_w = None
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self._vslam_prev_pose is not None and self._vslam_prev_stamp is not None:
            dt = now - self._vslam_prev_stamp
            if dt > 1e-3:
                dx = pos.x - self._vslam_prev_pose[0]
                dy = pos.y - self._vslam_prev_pose[1]
                dist = math.hypot(dx, dy)
                self._vslam_jump_flag = dist > self._icp_jump_max_m
                heading = math.atan2(dy, dx) if dist > 1e-4 else self._vslam_prev_pose[2]
                sign = 1.0 if abs(wrap_to_pi(heading - self._vslam_prev_pose[2])) < math.pi / 2 else -1.0
                fd_v = sign * dist / dt
                fd_w = wrap_to_pi(yaw - self._vslam_prev_pose[2]) / dt
                self._vslam_gap_flag = dt > self._icp_gap_max_s
        self._vslam_prev_pose = (pos.x, pos.y, yaw)
        self._vslam_prev_stamp = now
        self._vslam_stamp = now

        self._vslam_ground_speed = twist_v if self._vslam_twist_populated else fd_v
        self._vslam_yaw_rate = twist_w if self._vslam_twist_populated else fd_w

    def _select_pose_source(self, now: float) -> Tuple[Optional[float], Optional[float], str, bool, bool]:
        """Real-time ground_speed/yaw_rate for the grid/slip/dashboard computations
        below. VSLAM is primary when fresh; falls back to ICP otherwise -- see
        module docstring for why. Returns (ground_speed, yaw_rate, source_name,
        gap_flag, jump_flag)."""
        if self._vslam_stamp is not None and (now - self._vslam_stamp) < self._vslam_staleness_s:
            return self._vslam_ground_speed, self._vslam_yaw_rate, "vslam", self._vslam_gap_flag, self._vslam_jump_flag
        return self._ground_speed, self._icp_yaw_rate, "icp_fallback", self._icp_gap_flag, self._icp_jump_flag

    def _active_position(self, now: float):
        """Mirrors _select_pose_source's VSLAM-primary/ICP-fallback freshness logic,
        for the (x, y) needed by the zone map. Returns (x, y, source) or (None, None,
        'none') if neither is fresh -- 2026-07-30, added after a field incident where
        manual reverse recovery near an obstacle had no distance/direction feedback
        and made things worse."""
        if self._vslam_stamp is not None and (now - self._vslam_stamp) < self._vslam_staleness_s and self._vslam_prev_pose is not None:
            x, y, _yaw = self._vslam_prev_pose
            return x, y, "vslam"
        if self._odom_prev_pose is not None:
            x, y, _yaw = self._odom_prev_pose
            return x, y, "icp_fallback"
        return None, None, "none"

    def _on_obstacle_stop(self, msg: Bool) -> None:
        self._obstacle_stop_active = bool(msg.data)
        self._obstacle_stamp = self._now_s()

    def _on_obstacle_status(self, msg: String) -> None:
        self._obstacle_status_text = msg.data

    def _on_obstacle_clearance(self, msg: Float32) -> None:
        self._obstacle_clearance_m = float(msg.data)

    def _on_segment(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        uid = payload["uid"]
        self._active_uid = uid if payload.get("engaged") else None
        rec = self._segments.setdefault(uid, SegmentRecord(first_seen_wall=self._now_s()))
        rec.kind = payload.get("kind", rec.kind)
        rec.tier = payload.get("tier", rec.tier)
        rec.label = payload.get("label", rec.label)
        rec.meta = payload.get("meta", rec.meta)
        rec.last_seen_wall = self._now_s()
        rec.max_elapsed = max(rec.max_elapsed, float(payload.get("elapsed_s", 0.0)))
        rec.duration = float(payload.get("duration_s", rec.duration))
        if rec.duration > 0 and rec.max_elapsed >= rec.duration - 0.05:
            rec.completed = True

    # -- fast accumulation (grid, slip, split ratio, imu/icp cross-check) ---

    def _on_fast_tick(self) -> None:
        now = self._now_s()

        canonical_phi = self._phi_sign * self._phi_raw if self._phi_raw is not None else None
        pose_v, pose_yaw_rate, pose_source, pose_gap_flag, pose_jump_flag = self._select_pose_source(now)
        self._active_pose_source = pose_source
        self._active_pose_gap_flag = pose_gap_flag
        self._active_pose_jump_flag = pose_jump_flag
        # Note: self._icp_gap_flag / self._icp_jump_flag are maintained independently
        # inside _on_icp_odom and always reflect ICP's own health, regardless of
        # whether ICP or VSLAM is the currently-active source -- kept separate so the
        # dashboard can show "ICP is fine, we're just using VSLAM right now" or vice versa.

        # Belt-slip flag: tachometer vs the active (VSLAM-primary, ICP-fallback) ground speed.
        if self._tach_v_signed is not None and pose_v is not None:
            denom = max(abs(self._tach_v_signed), 0.2)
            slip = abs(self._tach_v_signed - pose_v) / denom
            self._slip_samples.append(slip)

        # IMU-vs-pose-source yaw-rate cross-check buffer.
        if self._imu_yaw_rate is not None and pose_yaw_rate is not None:
            if self._imu_stamp is not None and (now - self._imu_stamp) < 0.2:
                self._yawrate_pairs.append((self._imu_yaw_rate, pose_yaw_rate))

        # VSLAM-vs-ICP yaw-rate consistency: live check that the fast primary source
        # (VSLAM) and the trusted offline source (ICP) agree, independent of which one
        # is currently selected -- catches VSLAM drift/tracking loss before it silently
        # corrupts the live grid.
        if (
            self._vslam_yaw_rate is not None and self._icp_yaw_rate is not None
            and self._vslam_stamp is not None and (now - self._vslam_stamp) < self._vslam_staleness_s
            and self._icp_stamp is not None and (now - self._icp_stamp) < self._icp_gap_max_s
        ):
            self._vslam_vs_icp_pairs.append((self._vslam_yaw_rate, self._icp_yaw_rate))

        # Standstill split ratio dtheta1/dphi (bypasses the kappa speed gate entirely).
        if (
            pose_v is not None
            and abs(pose_v) < self._standstill_v
            and self._imu_yaw_rate is not None
            and self._phi_prev is not None
            and self._phi_prev_stamp is not None
            and self._phi_stamp is not None
        ):
            dt_phi = self._phi_stamp - self._phi_prev_stamp
            if dt_phi > 1e-3:
                phi_rate = self._phi_sign * (self._phi_raw - self._phi_prev) / dt_phi
                if abs(phi_rate) > 0.01:  # rad/s, ignore near-zero (undefined ratio)
                    self._split_ratio_samples.append(self._imu_yaw_rate / phi_rate)

        # Joint (v, phi) occupancy grid, gated like the frozen benchmark.
        if (
            pose_v is not None
            and canonical_phi is not None
            and self._imu_stamp is not None
            and (now - self._imu_stamp) < 0.2
        ):
            v = pose_v
            if abs(v) >= self._v_gate_kappa:
                kappa = self._imu_yaw_rate / v if self._imu_yaw_rate is not None else None
                if kappa is not None and abs(kappa) < self._kappa_abs_max:
                    v_bin = int(np.clip(np.digitize([v], self._v_edges)[0] - 1, 0, len(self._v_edges) - 2))
                    phi_deg = math.degrees(canonical_phi)
                    phi_bin = int(np.clip(np.digitize([phi_deg], self._phi_edges)[0] - 1, 0, len(self._phi_edges) - 2))
                    cell = self._grid[(v_bin, phi_bin)]
                    if self._grid_last_update is not None:
                        cell["dwell_s"] += min(now - self._grid_last_update, 0.5)
                    if self._active_uid is not None:
                        cell["uids"].add(self._active_uid)
        self._grid_last_update = now

    # -- dashboard -----------------------------------------------------------

    def _series_checklist(self) -> Dict[str, Tuple[int, int]]:
        seen: Dict[str, int] = defaultdict(int)
        done: Dict[str, int] = defaultdict(int)
        for rec in self._segments.values():
            series = KIND_TO_SERIES.get(rec.kind, rec.kind)
            seen[series] += 1
            if rec.completed:
                done[series] += 1
        return {k: (done[k], seen[k]) for k in seen}

    def _quality_report(self) -> dict:
        report = {}
        if self._yawrate_pairs:
            arr = np.array(self._yawrate_pairs)
            imu, icp = arr[:, 0], arr[:, 1]
            if len(arr) >= 5 and np.std(imu) > 1e-6 and np.std(icp) > 1e-6:
                corr = float(np.corrcoef(imu, icp)[0, 1])
            else:
                corr = float("nan")
            rmse = float(np.sqrt(np.mean((imu - icp) ** 2)))
            report["imu_vs_active_pose_yaw_rate"] = {"n": len(arr), "corr": corr, "rmse_rad_s": rmse}
        if self._slip_samples:
            arr = np.array(self._slip_samples)
            report["belt_slip"] = {
                "median": float(np.median(arr)),
                "p95": float(np.percentile(arr, 95)),
                "fraction_above_warn": float(np.mean(arr > self._slip_warn)),
            }
        if self._split_ratio_samples:
            arr = np.array(self._split_ratio_samples)
            report["standstill_split_ratio"] = {
                "n": len(arr),
                "median": float(np.median(arr)),
                "p10_p90": [float(np.percentile(arr, 10)), float(np.percentile(arr, 90))],
            }
        if self._vslam_vs_icp_pairs:
            arr = np.array(self._vslam_vs_icp_pairs)
            vs, icp = arr[:, 0], arr[:, 1]
            if len(arr) >= 5 and np.std(vs) > 1e-6 and np.std(icp) > 1e-6:
                corr = float(np.corrcoef(vs, icp)[0, 1])
            else:
                corr = float("nan")
            report["vslam_vs_icp_yaw_rate"] = {
                "n": len(arr), "corr": corr, "rmse_rad_s": float(np.sqrt(np.mean((vs - icp) ** 2))),
            }
        report["active_pose_source"] = self._active_pose_source
        report["active_pose_gap_flag"] = self._active_pose_gap_flag
        report["active_pose_jump_flag"] = self._active_pose_jump_flag
        report["icp_gap_flag"] = self._icp_gap_flag
        report["icp_jump_flag"] = self._icp_jump_flag
        report["obstacle_stop_active"] = self._obstacle_stop_active
        report["obstacle_clearance_m"] = self._obstacle_clearance_m
        report["obstacle_status_text"] = self._obstacle_status_text
        return report

    def _grid_ascii(self) -> str:
        rows = []
        header = "phi\\v(m/s) " + " ".join(f"{self._v_edges[i]:5.2f}" for i in range(len(self._v_edges) - 1))
        rows.append(header)
        for pb in range(len(self._phi_edges) - 1):
            phi_mid = (self._phi_edges[pb] + self._phi_edges[pb + 1]) / 2.0
            cells = []
            for vb in range(len(self._v_edges) - 1):
                cell = self._grid.get((vb, pb))
                if cell is None or cell["dwell_s"] < 0.5:
                    cells.append("  .  ")
                else:
                    n_uid = len(cell["uids"])
                    mark = "OK" if n_uid >= self._target_reps else str(n_uid)
                    cells.append(f"{mark:>3}({cell['dwell_s']:>3.0f}s)"[:5])
            rows.append(f"{phi_mid:8.1f}  " + " ".join(cells))
        return "\n".join(rows)

    def _pose_source_alerts(self) -> list[str]:
        """Loud, hard-to-miss alerts for the two failure modes that went unnoticed
        for a full session on grass (2026-07-28): a pose source with zero messages
        for the whole run, and a source that was alive and then went silent. Both
        degrade gracefully (see _select_pose_source) instead of crashing, which is
        exactly why they need a SEPARATE loud signal -- silent degradation is safe
        for the robot but easy to miss on a scrolling terminal."""
        now = self._now_s()
        since_start = now - self._node_start_wall
        alerts: list[str] = []

        if not self._vslam_ever_seen:
            if since_start > self._pose_source_startup_grace_s:
                alerts.append(
                    f"VSLAM: NEVER received a message ({since_start:.0f}s since startup) -- "
                    "check the ZED camera (cable/USB/power) and 'docker compose logs isaac_vslam'. "
                    "Running on ICP only for the whole session so far."
                )
        elif self._vslam_stamp is not None:
            age = now - self._vslam_stamp
            if age > self._pose_source_dead_s:
                alerts.append(f"VSLAM: silent for {age:.0f}s (was alive) -- may have crashed or lost tracking.")

        if not self._icp_ever_seen:
            if since_start > self._pose_source_startup_grace_s:
                alerts.append(
                    f"ICP: NEVER received a message ({since_start:.0f}s since startup) -- "
                    "check the 'mapping' service (docker compose logs mapping)."
                    + (" VSLAM is ALSO down -- NO usable pose source at all right now." if not self._vslam_ever_seen else "")
                )
        elif self._icp_stamp is not None:
            age = now - self._icp_stamp
            if age > self._pose_source_dead_s:
                alerts.append(f"ICP: silent for {age:.0f}s (was alive) -- mapping node may have stalled or crashed.")

        return alerts

    def _obstacle_alerts(self) -> list[str]:
        """Mirrors _pose_source_alerts: mtt_front_obstacle_monitor is a default
        service in this stack and fails safe on its own, but if THIS node hasn't
        heard from it at all, that's worth a loud flag too -- same philosophy as
        the pose-source alerts, purely informational here (the conductor is the
        one that actually gates on it, via the same _is_engaged() AND condition)."""
        now = self._now_s()
        since_start = now - self._node_start_wall
        if self._obstacle_stamp is None:
            if since_start > self._pose_source_startup_grace_s:
                return [f"OBSTACLE MONITOR: NEVER received a message ({since_start:.0f}s since startup) -- "
                        "check the 'localization' service (mtt_front_obstacle_monitor)."]
            return []
        age = now - self._obstacle_stamp
        if age > self._pose_source_dead_s:
            return [f"OBSTACLE MONITOR: silent for {age:.0f}s (was alive) -- conductor is failing closed (disengaged) on this."]
        return []

    def _zone_alerts(self) -> list[str]:
        """Loud version of the zone-map distance line: fires when close enough
        that the operator should actively be looking at safe_direction_deg before
        moving, not just glancing at the normal dashboard line."""
        if self._zone_map is None:
            return []
        x, y, src = self._active_position(self._now_s())
        if x is None:
            return []
        d = self._zone_map.distance_to_boundary_m(x, y)
        if d < self._zone_warn_margin_m:
            direction = self._zone_map.safe_direction_deg(x, y)
            dir_str = f"{direction:.0f} deg" if direction == direction else "unknown (at/beyond mapped boundary)"
            return [
                f"ZONE MAP [{src}]: only {d:.2f}m from the wall -- safe direction to move: {dir_str} "
                "(recenter articulation to ~0 before reversing on this vehicle)."
            ]
        return []

    def _on_dashboard_tick(self) -> None:
        checklist = self._series_checklist()
        quality = self._quality_report()
        alerts = self._pose_source_alerts() + self._obstacle_alerts() + self._zone_alerts()

        if alerts:
            banner = ["!" * 78, f"{RED}{BOLD}  POSE SOURCE ALERT -- DATA QUALITY AT RISK{RESET}"]
            for a in alerts:
                banner.append(f"{RED}{BOLD}  ! {a}{RESET}")
            banner.append("!" * 78)
            print("\n".join(banner), flush=True)

        lines = ["=" * 78, "MTT ICE-RINK EXPERIMENT MONITOR", "-" * 78]
        lines.append("Segment checklist (completed/seen):")
        for series, (done, seen) in sorted(checklist.items()):
            flag = "OK " if done >= seen and seen > 0 else "..."
            lines.append(f"  [{flag}] {series:40s} {done}/{seen}")
        lines.append("-" * 78)
        lines.append(f"Coverage grid (cell = independent-segment-count(dwell_s), target={self._target_reps}):")
        lines.append(self._grid_ascii())
        lines.append("-" * 78)
        lines.append("Live quality:")
        lines.append(
            f"  Active pose source: {quality['active_pose_source']}  "
            f"(gap={quality['active_pose_gap_flag']} jump={quality['active_pose_jump_flag']})"
        )
        yr = quality.get("imu_vs_active_pose_yaw_rate")
        if yr:
            lines.append(f"  IMU vs {quality['active_pose_source']} yaw-rate: n={yr['n']} corr={yr['corr']:.3f} rmse={yr['rmse_rad_s']:.3f} rad/s")
        vsicp = quality.get("vslam_vs_icp_yaw_rate")
        if vsicp:
            lines.append(f"  VSLAM vs ICP yaw-rate: n={vsicp['n']} corr={vsicp['corr']:.3f} rmse={vsicp['rmse_rad_s']:.3f} rad/s")
        slip = quality.get("belt_slip")
        if slip:
            lines.append(
                f"  Belt slip ratio: median={slip['median']:.2f} p95={slip['p95']:.2f} "
                f"frac>{self._slip_warn:.1f}={slip['fraction_above_warn']:.2f}"
            )
        split = quality.get("standstill_split_ratio")
        if split:
            lines.append(
                f"  Standstill split dtheta1/dphi: n={split['n']} median={split['median']:.2f} "
                f"p10-p90={split['p10_p90'][0]:.2f}..{split['p10_p90'][1]:.2f} (nominal ~0.62)"
            )
        lines.append(f"  ICP gap flag={quality['icp_gap_flag']}  ICP jump flag={quality['icp_jump_flag']}")
        clearance_str = f"{self._obstacle_clearance_m:.2f}m" if self._obstacle_clearance_m is not None else "n/a"
        lines.append(
            f"  Obstacle monitor: stop={self._obstacle_stop_active}  clearance={clearance_str}  "
            f"({self._obstacle_status_text or 'no status yet'})"
        )
        if self._zone_map is not None:
            zx, zy, zsrc = self._active_position(self._now_s())
            if zx is None:
                lines.append("  Zone map: no fused position yet")
            else:
                zd = self._zone_map.distance_to_boundary_m(zx, zy)
                zdir = self._zone_map.safe_direction_deg(zx, zy)
                dir_str = f"{zdir:.0f}deg (away from nearest wall)" if zd == zd else "n/a"  # zd==zd: not nan
                lines.append(
                    f"  Zone map [{zsrc}]: distance_to_wall={zd:.2f}m  safe_direction={dir_str}"
                    + ("  << CLOSE, use safe_direction before reversing" if zd < self._zone_warn_margin_m else "")
                )
        active = self._active_uid or "(disengaged / pause)"
        lines.append(f"Active segment: {active}")
        lines.append("=" * 78)
        text = "\n".join(lines)
        print(text, flush=True)

        snapshot = {
            "wall_time": time.time(),
            "checklist": checklist,
            "quality": quality,
            "pose_source_alerts": alerts,
            "active_uid": self._active_uid,
            "grid": {
                f"{vb}_{pb}": {"dwell_s": c["dwell_s"], "n_independent": len(c["uids"])}
                for (vb, pb), c in self._grid.items()
            },
        }
        try:
            with open(self._json_path, "w") as fh:
                json.dump(snapshot, fh, indent=2)
        except OSError as exc:
            self.get_logger().warn(f"Could not write coverage snapshot: {exc}")
        msg = String()
        msg.data = json.dumps(snapshot)
        self._coverage_pub.publish(msg)

    def destroy_node(self) -> None:
        checklist = self._series_checklist()
        print("\n" + "=" * 78)
        print("SESSION EXIT SUMMARY")
        for series, (done, seen) in sorted(checklist.items()):
            verdict = "SUFFICIENT" if done >= seen and seen > 0 else "INCOMPLETE"
            print(f"  {series:40s} {done}/{seen}  -> {verdict}")
        print(f"  VSLAM ever seen a message: {self._vslam_ever_seen}")
        print(f"  ICP ever seen a message:   {self._icp_ever_seen}")
        alerts = self._pose_source_alerts()
        if alerts:
            print(f"  {RED}{BOLD}POSE SOURCE ALERTS STILL ACTIVE AT EXIT:{RESET}")
            for a in alerts:
                print(f"  {RED}{BOLD}  ! {a}{RESET}")
        print(f"Coverage snapshot last written to: {self._json_path}")
        print("=" * 78)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MttExperimentMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
