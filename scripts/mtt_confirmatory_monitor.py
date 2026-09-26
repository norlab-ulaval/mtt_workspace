#!/usr/bin/env python3
"""Live advisory monitor for the Paper-1 Session-A confirmatory acquisition
(companion to mtt_experiment_conductor.py, sibling to mtt_experiment_monitor.py).

Why a separate node instead of extending mtt_experiment_monitor.py: that node is
purpose-built for the ice-rink M0-M5 (v, phi) occupancy grid -- kept exactly as-is
for that protocol (it is field-tested and safety-relevant; this task does not touch
it beyond one additive field on the conductor's /mtt_experiment/segment payload).
Session A needs a different live view entirely: progress against the frozen
20-cell x 3-repetition confirmatory matrix, keyed by the ledger's own `attempt_uid`
(e.g. "P01-R1"), not a kappa grid.

WHAT THIS NODE MUST NEVER DO
-----------------------------
Per FIRST_PAPER_CONFIRMATORY_EXPERIMENT.md's reveal sequence, no reference
lateral/yaw/pose outcome, translation/yaw RPE, CSRE, S_Q0, or model
(Classical-M0/Ours-Q) comparison may be displayed before the named external
reviewer's one-time reveal. This node satisfies that by construction: it only
ever subscribes to LIVE topics (tachometer, hardware articulation, IMU, VSLAM
odometry, ICP odometry, the conductor's own segment state) -- the offline
qualified reference does not exist live, it is produced post-hoc by
demos/bag_replay/scripts/offline_icp.py + build_session_dataset.py in the
research repo. assert_dashboard_payload_is_clean() is a defense-in-depth
self-check on top of that structural guarantee, so a future edit that
accidentally imports or hardcodes a forbidden field fails loudly instead of
silently leaking.

WHAT "ADVISORY" MEANS HERE
----------------------------
The per-attempt binary gate (10s uninterrupted, >=5m traveled, forward,
|v_b|>=0.5 m/s except <=0.1s total, timestamp gaps <=0.02s, one continuity
block) is evaluated LIVE using the tachometer channel only, as a fast proxy for
the eventual qualified v_b. This is NEVER the authoritative model-blind
qualification -- that runs post-hoc offline against the full canonical
multi-topic dataset (Gate B, research repo). The live verdict exists purely so
an operator gets an immediate "this cell needs a repeat" signal for the
model-blind operational reasons the protocol already allows (missing topic,
insufficient distance/duration, wrong cell, hardware fault) -- never to decide
whether a cell "looks scientifically good."

SINGLE-BAG POST-PROCESSING
----------------------------
This node does not require one bag per attempt. It publishes
/mtt_experiment/confirmatory_status as BOTH a ~1Hz full-state tick AND a
discrete "event" message on every meaningful transition (attempt armed,
attempt advisory-pass/advisory-incomplete, pause/checkpoint entered/exited,
checkpoint ACK received) -- both message kinds carry the same identifying
fields (role, attempt_uid, plan_id, repetition, event_seq). A single
continuous recording session spanning many attempts and checkpoints (e.g. one
whole Day-3 block) can be sliced into clean per-attempt time ranges after the
fact by filtering this one topic for `"message_kind": "event"`, without
needing per-attempt bag files.
"""

from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

try:
    from mtt_msgs.msg import MttTachometerData
except ImportError:  # pragma: no cover -- exercised only outside a built workspace
    MttTachometerData = None


# Mirrors the philosophy (not the exact list) of
# scripts/seal_first_paper_confirmatory.py's FORBIDDEN_OUTCOME_TOKENS in the
# research repo. Deliberately duplicated rather than imported: this node has no
# import path to that file (separate repo, no rclpy in the research .venv) and
# must fail closed even if that file is mid-edit or unavailable.
FORBIDDEN_DASHBOARD_TOKENS = (
    "vy_ref", "omega_ref", "yaw_ref", "pose_ref", "reference_vy", "reference_omega",
    "gt_track", "gt_yaw", "trajectory_error", "rpe", "csre", "s_q0", "winner",
    "classical_m0", "ours_q", "alpha",
)


def assert_dashboard_payload_is_clean(payload: dict) -> None:
    """Defense-in-depth self-check: this node structurally never touches the
    offline qualified reference or the frozen model predictions (neither exists
    live), so this should always pass trivially. It exists to make that
    invariant testable and to fail loudly the moment anyone adds a forbidden
    field, rather than relying on nobody ever importing the wrong thing."""
    flat = json.dumps(payload).lower()
    for token in FORBIDDEN_DASHBOARD_TOKENS:
        if token in flat:
            raise RuntimeError(
                f"forbidden pre-reveal token {token!r} found in confirmatory monitor "
                "dashboard payload -- this node must never surface reference/model data"
            )


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
    )


_ATTEMPT_UID_RE = re.compile(r"^P(\d{2})-R(\d)$")


def parse_attempt_uid(uid: str) -> Optional[tuple]:
    """Returns (plan_id, repetition) for a counted Session-A attempt uid like
    'P01-R1', or None for anything else (pauses, checkpoints, auto-reverse
    synthetic segments, extended/other profiles) -- those are not gate-tracked
    or matrix-tracked, but are still displayed and event-flagged."""
    match = _ATTEMPT_UID_RE.match(uid)
    if not match:
        return None
    plan, repetition = int(match.group(1)), int(match.group(2))
    return (plan, repetition) if 1 <= plan <= 20 and 1 <= repetition <= 3 else None


@dataclass
class ConfirmatoryGates:
    required_continuous_motion_s: float
    required_minimum_distance_m: float
    minimum_abs_speed_mps: float
    maximum_below_speed_duration_s: float
    maximum_timestamp_gap_s: float

    def __post_init__(self) -> None:
        if any(not math.isfinite(v) or v <= 0 for v in (
                self.required_continuous_motion_s, self.required_minimum_distance_m,
                self.minimum_abs_speed_mps, self.maximum_timestamp_gap_s)):
            raise ValueError("confirmatory gate thresholds must be finite and positive")
        if (not math.isfinite(self.maximum_below_speed_duration_s)
                or self.maximum_below_speed_duration_s < 0):
            raise ValueError("below-speed allowance must be finite and nonnegative")

    @classmethod
    def from_profile_yaml(cls, path: Path) -> "ConfirmatoryGates":
        doc = yaml.safe_load(path.read_text())
        gates = (doc.get("meta") or {}).get("confirmatory_gates")
        if not gates:
            raise RuntimeError(
                f"{path} has no meta.confirmatory_gates block -- was it generated by "
                "scripts/build_session_a_conductor_profile.py in the research repo?"
            )
        return cls(**{k: float(v) for k, v in gates.items()})


@dataclass
class LiveGateTracker:
    """Advisory, live approximation of the per-attempt binary gate. NEVER
    authoritative -- see the module docstring's "WHAT ADVISORY MEANS HERE".
    Pure logic, deliberately ROS-free so it is unit-testable in isolation.

    Tracks one contiguous "continuity block" of tachometer samples: a block
    resets whenever the tachometer message-to-message gap exceeds
    maximum_timestamp_gap_s, whenever cumulative below-speed time exceeds
    maximum_below_speed_duration_s, or whenever a (small, implementation-only,
    not a frozen protocol value) reverse-direction tolerance is exceeded. The
    block advisory-passes the instant it has accumulated both
    required_continuous_motion_s of elapsed time and required_minimum_distance_m
    of forward travel.

    NOTE on gates.maximum_timestamp_gap_s (0.02s in the frozen ledger): that
    value is calibrated for gaps in the fused ~100Hz canonical grid built
    offline from multiple topics -- it is NOT reused here to detect a "gap" in
    raw tachometer message arrivals, because the tachometer's own native rate
    is itself ~50Hz (nominal ~0.02s spacing): applying the same number as a
    hard threshold against its own nominal period would spuriously reset the
    block on ordinary publisher jitter, defeating the tracker's purpose. Tach
    silence is instead detected with TACH_SILENCE_RESET_S, a separate,
    deliberately generous, implementation-only constant (NOT a frozen protocol
    parameter) -- this is exactly the kind of approximation that keeps this
    tracker advisory rather than authoritative.
    """

    TACH_SILENCE_RESET_S = 0.15  # ~7x the tachometer's nominal 0.02s period

    gates: ConfirmatoryGates
    forward_violation_epsilon_s: float = 0.05

    _block_start_t: Optional[float] = field(default=None, init=False)
    _below_speed_accum_s: float = field(default=0.0, init=False)
    _distance_m: float = field(default=0.0, init=False)
    _forward_violation_s: float = field(default=0.0, init=False)
    _last_sample_t: Optional[float] = field(default=None, init=False)
    _last_msg_t: Optional[float] = field(default=None, init=False)
    passed: bool = field(default=False, init=False)

    def reset(self) -> None:
        self._block_start_t = None
        self._below_speed_accum_s = 0.0
        self._distance_m = 0.0
        self._forward_violation_s = 0.0
        self._last_sample_t = None
        self._last_msg_t = None
        self.passed = False

    def _start_block(self, t: float) -> None:
        self._block_start_t = t
        self._last_sample_t = t
        self._below_speed_accum_s = 0.0
        self._distance_m = 0.0
        self._forward_violation_s = 0.0

    def on_tach_sample(self, t: float, v_signed: float) -> None:
        if self.passed:
            return
        if not math.isfinite(t) or not math.isfinite(v_signed):
            self.reset()
            return
        gap = None if self._last_msg_t is None else (t - self._last_msg_t)
        self._last_msg_t = t
        if gap is not None and (gap <= 0 or gap > self.TACH_SILENCE_RESET_S):
            self._start_block(t)
            return
        if self._block_start_t is None:
            self._start_block(t)
            return
        dt = t - self._last_sample_t
        self._last_sample_t = t
        if dt <= 0:
            return
        if v_signed < 0:
            self._forward_violation_s += dt
            if self._forward_violation_s > self.forward_violation_epsilon_s:
                self._start_block(t)
                return
        else:
            self._forward_violation_s = 0.0
        if abs(v_signed) < self.gates.minimum_abs_speed_mps:
            self._below_speed_accum_s += dt
            if self._below_speed_accum_s > self.gates.maximum_below_speed_duration_s:
                self._start_block(t)
                return
        elif v_signed > 0:
            self._distance_m += v_signed * dt
        elapsed = t - self._block_start_t
        if (
            elapsed >= self.gates.required_continuous_motion_s
            and self._distance_m >= self.gates.required_minimum_distance_m
        ):
            self.passed = True

    def status(self) -> str:
        if self.passed:
            return "advisory_pass"
        if self._block_start_t is None:
            return "not_started"
        return "advisory_in_progress"

    def progress(self) -> dict:
        elapsed = 0.0 if self._block_start_t is None or self._last_sample_t is None else (
            self._last_sample_t - self._block_start_t
        )
        return {
            "status": self.status(),
            "block_elapsed_s": round(elapsed, 2),
            "block_distance_m": round(self._distance_m, 2),
            "below_speed_accum_s": round(self._below_speed_accum_s, 2),
        }


class MttConfirmatoryMonitor(Node):
    def __init__(self) -> None:
        super().__init__("mtt_confirmatory_monitor")

        self.declare_parameter("tacho_topic", "mtt_tachometer")
        self.declare_parameter("articulation_topic", "/hardware/articulation_angle")
        self.declare_parameter("icp_odom_topic", "/mapping/icp_odom")
        self.declare_parameter("vslam_odom_topic", "/isaac/vslam/odometry")
        self.declare_parameter("vslam_staleness_s", 0.3)
        self.declare_parameter("pose_source_startup_grace_s", 20.0)
        self.declare_parameter("pose_source_dead_s", 8.0)
        self.declare_parameter("imu_topic", "/mti100/data")
        self.declare_parameter("segment_topic", "/mtt_experiment/segment")
        self.declare_parameter("checkpoint_ack_topic", "/mtt_experiment/checkpoint_ack")
        self.declare_parameter("confirmatory_status_topic", "/mtt_experiment/confirmatory_status")
        self.declare_parameter("confirmatory_profile_path", "")
        # "session_a_confirmatory" = counted, sealed acquisition. "non_counted_pilot"
        # = Day-2 hardware pilot (LAST_WEEK_QUEBEC_EXECUTION_PLAN.md) -- purely a
        # label attached to every event/snapshot/log filename so a pilot run can
        # never be mistaken for, or merged into, the sealed 60. This node never
        # writes the sealed ledger either way; the label is for post-processing.
        self.declare_parameter("role", "session_a_confirmatory")
        self.declare_parameter("dashboard_rate_hz", 1.0)
        self.declare_parameter("snapshot_json_path", "")

        profile_path_str = str(self.get_parameter("confirmatory_profile_path").value)
        if not profile_path_str:
            raise RuntimeError(
                "confirmatory_profile_path parameter is required -- point it at the "
                "profile generated by build_session_a_conductor_profile.py so the "
                "five gate thresholds are read, not hardcoded a second time."
            )
        self._gates = ConfirmatoryGates.from_profile_yaml(Path(profile_path_str))
        self.get_logger().info(f"Confirmatory gates loaded: {self._gates}")

        self._role = str(self.get_parameter("role").value)
        if self._role not in ("session_a_confirmatory", "non_counted_pilot"):
            raise RuntimeError(f"unknown role {self._role!r}")

        self._vslam_staleness_s = float(self.get_parameter("vslam_staleness_s").value)
        self._pose_source_startup_grace_s = float(self.get_parameter("pose_source_startup_grace_s").value)
        self._pose_source_dead_s = float(self.get_parameter("pose_source_dead_s").value)

        # --- live raw state (telemetry / situational awareness only -- never
        # feeds the gate tracker, see class docstring) ---
        self._icp_stamp: Optional[float] = None
        self._icp_pose_xy: Optional[tuple] = None
        self._vslam_stamp: Optional[float] = None
        self._vslam_pose_xy: Optional[tuple] = None
        self._vslam_ever_seen = False
        self._icp_ever_seen = False
        self._imu_stamp: Optional[float] = None
        self._phi_raw: Optional[float] = None
        self._phi_stamp: Optional[float] = None
        self._node_start_wall = self._now_s()

        # --- active segment / gate tracking ---
        self._active_uid: Optional[str] = None
        self._active_kind: str = ""
        self._active_is_pause = False
        self._active_requires_ack = False
        self._active_pause_message = ""
        self._tracker: Optional[LiveGateTracker] = None
        self._matrix: dict = {}  # (plan_id, repetition) -> tracker.status()
        self._event_seq = 0
        self._checkpoint_ack_seen_for: Optional[str] = None

        ts = int(time.time())
        json_path_param = str(self.get_parameter("snapshot_json_path").value)
        if json_path_param:
            self._json_path = Path(json_path_param)
        else:
            self._json_path = Path.cwd() / "data" / "confirmatory_logs" / f"{self._role}_{ts}.json"
        self._json_path.parent.mkdir(parents=True, exist_ok=True)

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

        if MttTachometerData is not None:
            self.create_subscription(
                MttTachometerData, str(self.get_parameter("tacho_topic").value), self._on_tacho, volatile_qos
            )
        self.create_subscription(Imu, str(self.get_parameter("imu_topic").value), self._on_imu, sensor_qos)
        self.create_subscription(
            Odometry, str(self.get_parameter("icp_odom_topic").value), self._on_icp_odom, volatile_qos
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("vslam_odom_topic").value), self._on_vslam_odom, sensor_qos
        )
        self.create_subscription(
            String, str(self.get_parameter("segment_topic").value), self._on_segment, volatile_qos
        )
        self.create_subscription(
            String, str(self.get_parameter("checkpoint_ack_topic").value), self._on_checkpoint_ack, volatile_qos
        )

        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("confirmatory_status_topic").value), 20
        )

        dashboard_hz = float(self.get_parameter("dashboard_rate_hz").value)
        self.create_timer(1.0 / max(dashboard_hz, 0.1), self._on_dashboard_tick)

        self.get_logger().warn(
            f"Confirmatory monitor ready. role={self._role!r}  gates={self._gates}  "
            f"snapshot={self._json_path}  ADVISORY ONLY -- never authoritative."
        )

    # -- callbacks -----------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tacho(self, msg) -> None:
        if (not msg.telemetry_fresh or msg.tachometer_is_synthetic
                or msg.direction not in ("Forward", "Reverse")
                or not math.isfinite(msg.speed_ms) or msg.speed_ms < 0):
            if self._tracker is not None and not self._tracker.passed:
                self._tracker.reset()
            return
        sign = 1.0 if msg.direction == "Forward" else -1.0
        v_signed = sign * msg.speed_ms
        if self._tracker is not None:
            self._tracker.on_tach_sample(self._now_s(), v_signed)
            self._maybe_emit_pass_event()

    def _on_imu(self, msg: Imu) -> None:
        self._imu_stamp = self._now_s()

    def _on_icp_odom(self, msg: Odometry) -> None:
        if not self._icp_ever_seen:
            self._icp_ever_seen = True
            self.get_logger().info("ICP odom (/mapping/icp_odom) first message received.")
        self._icp_stamp = self._now_s()
        p = msg.pose.pose.position
        self._icp_pose_xy = (p.x, p.y)

    def _on_vslam_odom(self, msg: Odometry) -> None:
        if not self._vslam_ever_seen:
            self._vslam_ever_seen = True
            self.get_logger().info("VSLAM odom (/isaac/vslam/odometry) first message received.")
        self._vslam_stamp = self._now_s()
        p = msg.pose.pose.position
        self._vslam_pose_xy = (p.x, p.y)

    def _on_checkpoint_ack(self, msg: String) -> None:
        if msg.data.strip() and self._active_uid is not None:
            self._checkpoint_ack_seen_for = self._active_uid
            self._emit_event("checkpoint_ack_received", note=msg.data.strip())

    def _on_segment(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        uid = payload.get("uid")
        if uid != self._active_uid:
            self._on_segment_changed(uid, payload)
        self._active_kind = payload.get("kind", self._active_kind)
        self._active_is_pause = bool(payload.get("is_pause", False))
        self._active_requires_ack = bool(payload.get("requires_ack", False))
        self._active_pause_message = payload.get("pause_message", "")

    def _on_segment_changed(self, new_uid: Optional[str], payload: dict) -> None:
        # Falling edge: close out the previous attempt/pause before arming the next.
        if self._active_uid is not None:
            parsed = parse_attempt_uid(self._active_uid)
            if parsed is not None and self._tracker is not None:
                self._matrix[parsed] = self._tracker.status()
                if not self._tracker.passed:
                    self._emit_event(
                        "attempt_ended_advisory_incomplete", note=self._tracker.progress()
                    )
            elif self._active_is_pause:
                self._emit_event("pause_exit")

        self._active_uid = new_uid
        # Set new segment fields before emitting its transition event.
        self._active_kind = payload.get("kind", "")
        self._active_is_pause = bool(payload.get("is_pause", False))
        self._active_requires_ack = bool(payload.get("requires_ack", False))
        self._active_pause_message = payload.get("pause_message", "")
        self._tracker = None
        self._checkpoint_ack_seen_for = None
        if new_uid is None:
            return

        parsed = parse_attempt_uid(new_uid)
        if parsed is not None:
            self._tracker = LiveGateTracker(gates=self._gates)
            self._matrix.setdefault(parsed, "not_started")
            self._emit_event("attempt_armed")
        elif payload.get("is_pause"):
            self._emit_event("pause_enter")
        else:
            self._emit_event("segment_armed")

    def _maybe_emit_pass_event(self) -> None:
        if self._tracker is not None and self._tracker.passed:
            parsed = parse_attempt_uid(self._active_uid or "")
            if parsed is not None and self._matrix.get(parsed) != "advisory_pass":
                self._matrix[parsed] = "advisory_pass"
                self._emit_event("attempt_advisory_pass")

    # -- pose-source health (situational awareness only) ---------------------

    def _pose_source_alerts(self) -> list:
        now = self._now_s()
        since_start = now - self._node_start_wall
        alerts = []
        if not self._vslam_ever_seen and since_start > self._pose_source_startup_grace_s:
            alerts.append(f"VSLAM: NEVER received a message ({since_start:.0f}s since startup).")
        elif self._vslam_stamp is not None and (now - self._vslam_stamp) > self._pose_source_dead_s:
            alerts.append(f"VSLAM: silent for {now - self._vslam_stamp:.0f}s (was alive).")
        if not self._icp_ever_seen and since_start > self._pose_source_startup_grace_s:
            alerts.append(f"ICP: NEVER received a message ({since_start:.0f}s since startup).")
        elif self._icp_stamp is not None and (now - self._icp_stamp) > self._pose_source_dead_s:
            alerts.append(f"ICP: silent for {now - self._icp_stamp:.0f}s (was alive).")
        return alerts

    # -- event/snapshot emission ----------------------------------------------

    def _base_fields(self) -> dict:
        parsed = parse_attempt_uid(self._active_uid or "")
        return {
            "role": self._role,
            "wall_time": time.time(),
            "active_uid": self._active_uid,
            "active_kind": self._active_kind,
            "is_pause": self._active_is_pause,
            "requires_ack": self._active_requires_ack,
            "pause_message": self._active_pause_message,
            "plan_id": parsed[0] if parsed else None,
            "repetition": parsed[1] if parsed else None,
        }

    def _emit_event(self, event: str, note=None) -> None:
        self._event_seq += 1
        payload = self._base_fields()
        payload.update({"message_kind": "event", "event": event, "event_seq": self._event_seq, "note": note})
        if self._tracker is not None:
            payload["gate_progress"] = self._tracker.progress()
        assert_dashboard_payload_is_clean(payload)
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)

    def _on_dashboard_tick(self) -> None:
        alerts = self._pose_source_alerts()
        payload = self._base_fields()
        payload.update(
            {
                "message_kind": "tick",
                "event_seq": self._event_seq,
                "gate_progress": self._tracker.progress() if self._tracker is not None else None,
                "matrix_completed": sum(1 for v in self._matrix.values() if v == "advisory_pass"),
                "matrix_total_seen": len(self._matrix),
                "pose_source_alerts": alerts,
            }
        )
        assert_dashboard_payload_is_clean(payload)
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)

        if alerts:
            print("!" * 78)
            for a in alerts:
                print(f"  ! {a}")
            print("!" * 78)

        lines = ["=" * 78, f"SESSION-A CONFIRMATORY MONITOR [{self._role}] (ADVISORY ONLY)", "-" * 78]
        lines.append(f"Active: {self._active_uid or '(none)'}  kind={self._active_kind}")
        if self._active_is_pause:
            tag = "CHECKPOINT (awaiting ACK)" if self._active_requires_ack else "PAUSE"
            lines.append(f"  {tag}: {self._active_pause_message}")
        if self._tracker is not None:
            lines.append(f"  Gate (advisory): {self._tracker.progress()}")
        lines.append(
            f"Matrix: {payload['matrix_completed']}/60 advisory-passed, "
            f"{payload['matrix_total_seen']} cells seen so far"
        )
        lines.append("=" * 78)
        print("\n".join(lines), flush=True)

        try:
            with open(self._json_path, "w") as fh:
                json.dump(payload, fh, indent=2, default=str)
        except OSError as exc:
            self.get_logger().warn(f"Could not write snapshot: {exc}")

    def destroy_node(self) -> None:
        # Bug fixed 2026-08-27: previously computed "passed" as 60 - len(incomplete),
        # silently assuming every one of the 60 cells had been SEEN. With zero
        # segments ever received (e.g. the conductor never ran), self._matrix was
        # empty, incomplete was [], and this printed "60/60 passed" -- a false
        # success claim on a session that tested nothing. Caught by a live smoke
        # test inside the container, not by the pure-logic unit tests.
        seen = len(self._matrix)
        passed = sum(1 for v in self._matrix.values() if v == "advisory_pass")
        incomplete_cells = sorted(k for k, v in self._matrix.items() if v != "advisory_pass")
        never_attempted = 60 - seen
        print("\n" + "=" * 78)
        print(f"SESSION-A CONFIRMATORY MONITOR EXIT SUMMARY [{self._role}]")
        print(
            f"  cells advisory-passed: {passed}/60  "
            f"seen-but-incomplete: {len(incomplete_cells)}  "
            f"never attempted this session: {never_attempted}/60"
        )
        if incomplete_cells:
            print(f"  incomplete cells: {incomplete_cells}")
        print(f"  snapshot: {self._json_path}")
        print("=" * 78)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MttConfirmatoryMonitor()
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
