#!/usr/bin/env python3
"""Autonomous ice-rink experiment conductor for the MTT-154.

Plays a declarative sequence of command "segments" (defined in a YAML profile
file) that sweep the (speed x articulation x direction) grid the M0-M5 motion
model ladder needs to be identified -- see
Paper/mtt_m5_theory/main.tex ("Falsifiable predictions and identification
map") and Paper/Publi_ICRA27_MohamedOUNALLY/main.tex (Sec. Model Hierarchy),
and the pre-registered decisive experiments in
Paper/Publi_ICRA27_MohamedOUNALLY/M5_VL_PAPER_PATCH_PROPOSAL.tex.

SAFETY -- read before running on ice
-------------------------------------
Verified in mtt_control / mtt_driver (2026-07-23): neither
`mtt_cmd_arbiter_node` nor `mtt_can_node` gates AUTO commands on the deadman.
Both experiment command channels (speed, articulation) route through the
autonomy mux's "experiment" owner (see mtt_autonomy_mux_node.cpp), which the
arbiter then gates on mode+freshness for speed -- but the articulation
channel downstream of the mux (`/mtt_articulation_setpoint` ->
`mtt_articulation_servo_node` -> `articulation_servo/steer_cmd` ->
`mtt_can_node`) has *no* mode gate anywhere (`mtt_can_node.cpp`: "Always
allow servo override if a fresh message is received"). This conductor's own
AUTO+deadman interlock, plus the explicit mux-ownership request/release, is
therefore the SOLE protection for steering.

On disengage: (1) speed is explicitly zeroed for a short burst
(`_publish_zero_speed_only`, always safe -- zero speed never induces
motion), (2) articulation is NOT published at all during or after the
burst -- it is deliberately left to go stale so the servo's own
`command_timeout_s` (~0.25 s) silences it, rather than this node actively
commanding a return to phi=0, which would fight the operator's regained
manual control with an unrequested steering motion (fixed 2026-07-27; a
prior version wrongly published an explicit articulation=0 during the
burst). `_publish_zero()` (speed AND articulation to 0) is reserved for
deliberate, non-emergency parking only: profile-complete and node shutdown.
The node also calls `mtt_control/request_manual` on disengage so mode falls
out of AUTO -- resuming then requires a fresh press of the AUTO button,
matching the requested "hold deadman + press A" handoff model. The
both-trigger ESTOP and the STOP button remain the only *hard* stops; losing
the deadman is a soft stop with sub-second latency. Operators must be briefed
on this distinction before driving on ice.

The AUTO speed limit is capped at startup via the arbiter's
`/mtt_control/set_auto_speed_limit` service (default 4.2 m/s is not a safe
ice-rink speed); the node refuses to command any segment until that cap is
confirmed applied.
"""

from __future__ import annotations

import bisect
import csv
import json
import math
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional

import yaml

import rclpy
from geometry_msgs.msg import TwistStamped
from mtt_interfaces.srv import SetSpeedLimit
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def deg2rad(deg: float) -> float:
    return math.radians(deg)


# --------------------------------------------------------------------------
# Segment model: every segment type reduces to (duration_s, speed_fn(t),
# phi_fn(t)) so the control loop only needs one code path regardless of
# whether the segment is a constant hold, a triangular ramp, a sine slalom or
# a PRBS burst. Step-like segments (holds, phi steps) get their physical
# ramp-up "for free" from the slew limiter applied at publish time.
# --------------------------------------------------------------------------


@dataclass
class AtomicSegment:
    uid: str
    label: str
    kind: str
    tier: str
    duration_s: float
    speed_fn: Callable[[float], float]
    phi_fn: Callable[[float], float]
    is_pause: bool = False
    pause_message: str = ""
    requires_ack: bool = False  # manual_checkpoint: needs an explicit operator ACK, not just elapsed time
    meta: dict = field(default_factory=dict)


def _piecewise_constant(schedule: List[tuple]) -> Callable[[float], float]:
    """schedule: sorted [(t_start, value), ...]. Returns value active at t."""
    starts = [s for s, _ in schedule]
    values = [v for _, v in schedule]

    def fn(t: float) -> float:
        idx = bisect.bisect_right(starts, t) - 1
        idx = clamp(idx, 0, len(values) - 1)
        return values[idx]

    return fn


def _const(value: float) -> Callable[[float], float]:
    return lambda _t: value


def _resolve_speed(value, speeds: dict) -> float:
    """Resolve a speed field that may be a literal number or a named alias
    (e.g. 'V1') defined under the profile's meta.speeds table."""
    if isinstance(value, str):
        if value not in speeds:
            raise KeyError(f"unknown named speed '{value}' (known: {sorted(speeds)})")
        return float(speeds[value])
    return float(value)


def build_phi_steps(cfg: dict, speeds: dict) -> List[AtomicSegment]:
    speed_ms = _resolve_speed(cfg.get("speed_ms", 0.0), speeds)
    steps_deg = cfg["steps_deg"]
    hold_s = cfg.get("hold_s", 4.0)
    hold_list = hold_s if isinstance(hold_s, list) else [hold_s] * len(steps_deg)
    segments = []
    for i, deg in enumerate(steps_deg):
        segments.append(
            AtomicSegment(
                uid=f"{cfg['id']}#step{i:02d}_{deg:+.0f}deg",
                label=cfg.get("label", cfg["id"]),
                kind="phi_step",
                tier=cfg.get("tier", "core"),
                duration_s=float(hold_list[i]),
                speed_fn=_const(speed_ms),
                phi_fn=_const(deg2rad(deg)),
                meta={"speed_ms": speed_ms, "phi_deg": deg, "repeat": 1},
            )
        )
    return segments


def build_speed_steps(cfg: dict, speeds: dict) -> List[AtomicSegment]:
    phi_deg = float(cfg.get("phi_deg", 0.0))
    steps_ms = [_resolve_speed(v, speeds) for v in cfg["steps_ms"]]
    hold_s = cfg.get("hold_s", 4.0)
    hold_list = hold_s if isinstance(hold_s, list) else [hold_s] * len(steps_ms)
    segments = []
    for i, v in enumerate(steps_ms):
        segments.append(
            AtomicSegment(
                uid=f"{cfg['id']}#step{i:02d}_{v:.2f}ms",
                label=cfg.get("label", cfg["id"]),
                kind="speed_step",
                tier=cfg.get("tier", "core"),
                duration_s=float(hold_list[i]),
                speed_fn=_const(v),
                phi_fn=_const(deg2rad(phi_deg)),
                meta={"speed_ms": v, "phi_deg": phi_deg, "repeat": 1},
            )
        )
    return segments


def build_hold_arc_grid(cfg: dict, speeds: dict) -> List[AtomicSegment]:
    phi_deg_list = cfg["phi_deg_list"]
    segments = []
    for block in cfg["speeds"]:
        speed_ms = _resolve_speed(block["speed_ms"], speeds)
        reps = int(block.get("reps", 1))
        hold_s = float(block.get("hold_s", 10.0))
        for rep in range(1, reps + 1):
            for deg in phi_deg_list:
                segments.append(
                    AtomicSegment(
                        uid=f"{cfg['id']}#v{speed_ms:.2f}#phi{deg:+.0f}#rep{rep}",
                        label=cfg.get("label", cfg["id"]),
                        kind="hold_arc",
                        tier=cfg.get("tier", "core"),
                        duration_s=hold_s,
                        speed_fn=_const(speed_ms),
                        phi_fn=_const(deg2rad(deg)),
                        meta={"speed_ms": speed_ms, "phi_deg": deg, "repeat": rep},
                    )
                )
    return segments


def build_phi_ramp(cfg: dict, speeds: dict) -> List[AtomicSegment]:
    speed_ms = _resolve_speed(cfg.get("speed_ms", 0.0), speeds)
    waypoints_deg = list(cfg["waypoints_deg"])
    rate_deg_s = float(cfg["rate_deg_s"])
    repeats = int(cfg.get("repeats", 1))
    full_waypoints = []
    for r in range(repeats):
        full_waypoints.extend(waypoints_deg)
    schedule = [(0.0, deg2rad(full_waypoints[0]))]
    t = 0.0
    for a, b in zip(full_waypoints[:-1], full_waypoints[1:]):
        leg_s = abs(b - a) / max(rate_deg_s, 1e-6)
        t += leg_s
        schedule.append((t, deg2rad(b)))
    total_duration = t

    def phi_fn(elapsed: float) -> float:
        # Piecewise-linear interpolation between waypoints.
        if elapsed <= 0.0:
            return schedule[0][1]
        if elapsed >= total_duration:
            return schedule[-1][1]
        idx = bisect.bisect_right([s for s, _ in schedule], elapsed) - 1
        idx = clamp(idx, 0, len(schedule) - 2)
        t0, v0 = schedule[idx]
        t1, v1 = schedule[idx + 1]
        if t1 <= t0:
            return v1
        frac = (elapsed - t0) / (t1 - t0)
        return v0 + frac * (v1 - v0)

    return [
        AtomicSegment(
            uid=f"{cfg['id']}",
            label=cfg.get("label", cfg["id"]),
            kind="phi_ramp",
            tier=cfg.get("tier", "core"),
            duration_s=total_duration,
            speed_fn=_const(speed_ms),
            phi_fn=phi_fn,
            meta={"speed_ms": speed_ms, "phi_rate_deg_s": rate_deg_s},
        )
    ]


def build_figure8(cfg: dict, speeds: dict) -> List[AtomicSegment]:
    speed_ms = _resolve_speed(cfg.get("speed_ms", 0.3), speeds)
    amp = float(cfg["phi_amp_deg"])
    hold_s = float(cfg.get("hold_s", 6.0))
    lobes = int(cfg.get("repeats", 3)) * 2
    segments = []
    for i in range(lobes):
        deg = amp if i % 2 == 0 else -amp
        segments.append(
            AtomicSegment(
                uid=f"{cfg['id']}#lobe{i:02d}_{deg:+.0f}deg",
                label=cfg.get("label", cfg["id"]),
                kind="figure8",
                tier=cfg.get("tier", "core"),
                duration_s=hold_s,
                speed_fn=_const(speed_ms),
                phi_fn=_const(deg2rad(deg)),
                meta={"speed_ms": speed_ms, "phi_deg": deg, "repeat": i // 2 + 1},
            )
        )
    return segments


def build_prbs_phi(cfg: dict, speeds: dict) -> List[AtomicSegment]:
    speed_ms = _resolve_speed(cfg.get("speed_ms", 0.5), speeds)
    choices_deg = list(cfg["phi_choices_deg"])
    dwell_min = float(cfg.get("dwell_min_s", 2.0))
    dwell_max = float(cfg.get("dwell_max_s", 5.0))
    total_s = float(cfg.get("total_duration_s", 40.0))
    seed = int(cfg.get("seed", 42))
    rng = random.Random(seed)
    schedule = []
    t = 0.0
    while t < total_s:
        deg = rng.choice(choices_deg)
        schedule.append((t, deg2rad(deg)))
        t += rng.uniform(dwell_min, dwell_max)
    phi_fn = _piecewise_constant(schedule)
    return [
        AtomicSegment(
            uid=f"{cfg['id']}",
            label=cfg.get("label", cfg["id"]),
            kind="prbs_phi",
            tier=cfg.get("tier", "core"),
            duration_s=total_s,
            speed_fn=_const(speed_ms),
            phi_fn=phi_fn,
            meta={"speed_ms": speed_ms, "seed": seed},
        )
    ]


def build_slalom_sine(cfg: dict, speeds: dict) -> List[AtomicSegment]:
    speed_ms = _resolve_speed(cfg.get("speed_ms", 0.5), speeds)
    amp = deg2rad(float(cfg["phi_amp_deg"]))
    freq = float(cfg["freq_hz"])
    duration_s = float(cfg["duration_s"])

    def phi_fn(t: float) -> float:
        return amp * math.sin(2.0 * math.pi * freq * t)

    return [
        AtomicSegment(
            uid=f"{cfg['id']}",
            label=cfg.get("label", cfg["id"]),
            kind="slalom_sine",
            tier=cfg.get("tier", "extended"),
            duration_s=duration_s,
            speed_fn=_const(speed_ms),
            phi_fn=phi_fn,
            meta={"speed_ms": speed_ms, "amp_deg": cfg["phi_amp_deg"], "freq_hz": freq},
        )
    ]


def build_combined_v_phi(cfg: dict, speeds: dict) -> List[AtomicSegment]:
    speed0 = _resolve_speed(cfg.get("speed0_ms", 0.5), speeds)
    speed_amp = float(cfg.get("speed_amp_ms", 0.2))
    fv = float(cfg.get("speed_freq_hz", 0.05))
    phi_amp = deg2rad(float(cfg.get("phi_amp_deg", 10.0)))
    fphi = float(cfg.get("phi_freq_hz", 0.08))
    duration_s = float(cfg["duration_s"])

    def speed_fn(t: float) -> float:
        return speed0 + speed_amp * math.sin(2.0 * math.pi * fv * t)

    def phi_fn(t: float) -> float:
        return phi_amp * math.sin(2.0 * math.pi * fphi * t)

    return [
        AtomicSegment(
            uid=f"{cfg['id']}",
            label=cfg.get("label", cfg["id"]),
            kind="combined_v_phi",
            tier=cfg.get("tier", "extended"),
            duration_s=duration_s,
            speed_fn=speed_fn,
            phi_fn=phi_fn,
            meta={"speed0_ms": speed0, "fv": fv, "fphi": fphi},
        )
    ]


def build_stop_and_go(cfg: dict, speeds: dict) -> List[AtomicSegment]:
    speed_ms = _resolve_speed(cfg.get("speed_ms", 0.4), speeds)
    phi_deg = float(cfg.get("phi_deg", 10.0))
    on_s = float(cfg.get("on_s", 4.0))
    off_s = float(cfg.get("off_s", 3.0))
    cycles = int(cfg.get("cycles", 4))
    schedule = []
    t = 0.0
    for _ in range(cycles):
        schedule.append((t, speed_ms))
        t += on_s
        schedule.append((t, 0.0))
        t += off_s
    speed_fn = _piecewise_constant(schedule)
    return [
        AtomicSegment(
            uid=f"{cfg['id']}",
            label=cfg.get("label", cfg["id"]),
            kind="stop_and_go",
            tier=cfg.get("tier", "extended"),
            duration_s=t,
            speed_fn=speed_fn,
            phi_fn=_const(deg2rad(phi_deg)),
            meta={"speed_ms": speed_ms, "phi_deg": phi_deg},
        )
    ]


def build_pause(cfg: dict, speeds: dict) -> List[AtomicSegment]:
    return [
        AtomicSegment(
            uid=f"{cfg['id']}",
            label=cfg.get("label", cfg["id"]),
            kind="pause",
            tier=cfg.get("tier", "core"),
            duration_s=float(cfg.get("duration_s", 15.0)),
            speed_fn=_const(0.0),
            phi_fn=_const(0.0),
            is_pause=True,
            pause_message=cfg.get("message", cfg.get("label", "operator pause")),
            meta={},
        )
    ]


def build_checkpoint(cfg: dict, speeds: dict) -> List[AtomicSegment]:
    """Like a pause, but does NOT auto-advance once duration_s elapses -- it
    additionally requires an explicit operator ACK (see conductor's
    /mtt_experiment/checkpoint_ack handling) before completing. duration_s is
    only a MINIMUM dwell (time to physically do the manual step), not a
    timeout that silently waves the checkpoint through unconfirmed."""
    return [
        AtomicSegment(
            uid=f"{cfg['id']}",
            label=cfg.get("label", cfg["id"]),
            kind="checkpoint",
            tier=cfg.get("tier", "core"),
            duration_s=float(cfg.get("duration_s", 15.0)),
            speed_fn=_const(0.0),
            phi_fn=_const(0.0),
            is_pause=True,
            requires_ack=True,
            pause_message=cfg.get("message", cfg.get("label", "operator checkpoint")),
            meta={},
        )
    ]


BUILDERS = {
    "phi_steps": build_phi_steps,
    "speed_steps": build_speed_steps,
    "hold_arc_grid": build_hold_arc_grid,
    "phi_ramp": build_phi_ramp,
    "figure8": build_figure8,
    "prbs_phi": build_prbs_phi,
    "slalom_sine": build_slalom_sine,
    "combined_v_phi": build_combined_v_phi,
    "stop_and_go_hold_phi": build_stop_and_go,
    "pause": build_pause,
    "manual_checkpoint": build_checkpoint,
}


def load_profile(path: Path, include_extended: bool) -> List[AtomicSegment]:
    with open(path, "r") as fh:
        doc = yaml.safe_load(fh)
    speeds = (doc.get("meta") or {}).get("speeds") or {}
    out: List[AtomicSegment] = []
    for cfg in doc["segments"]:
        tier = cfg.get("tier", "core")
        if tier == "extended" and not include_extended:
            continue
        builder = BUILDERS.get(cfg["type"])
        if builder is None:
            raise ValueError(f"unknown segment type '{cfg['type']}' in id={cfg.get('id')}")
        out.extend(builder(cfg, speeds))
    return out


# --------------------------------------------------------------------------
# Node
# --------------------------------------------------------------------------


class MttExperimentConductor(Node):
    def __init__(self) -> None:
        super().__init__("mtt_experiment_conductor")

        self.declare_parameter("profile_path", "")
        self.declare_parameter("include_extended", False)
        self.declare_parameter("command_rate_hz", 25.0)
        self.declare_parameter("auto_speed_cap_ms", 1.2)
        self.declare_parameter("speed_accel_limit_mps2", 0.4)
        self.declare_parameter("articulation_rate_limit_rad_s", 0.45)
        self.declare_parameter("max_articulation_rad", 0.733)
        self.declare_parameter("mode_topic", "mtt_control/selected_mode")
        self.declare_parameter("auto_enabled_topic", "mtt_control/auto_mode_enabled")
        self.declare_parameter("deadman_topic", "mtt_control/teleop_deadman")
        self.declare_parameter("estop_topic", "mtt_control/teleop_estop")
        self.declare_parameter("cmd_vel_topic", "/mtt_control/auto/experiment/cmd_vel")
        self.declare_parameter(
            "articulation_topic", "/mtt_control/auto/experiment/articulation_setpoint"
        )
        self.declare_parameter("autonomy_owner", "experiment")
        self.declare_parameter("autonomy_request_topic", "/mtt_control/autonomy/request")
        self.declare_parameter("autonomy_release_topic", "/mtt_control/autonomy/release")
        self.declare_parameter("autonomy_selected_topic", "/mtt_control/autonomy/selected")
        self.declare_parameter("ownership_retry_s", 0.5)
        self.declare_parameter("segment_topic", "/mtt_experiment/segment")
        self.declare_parameter("request_manual_service", "mtt_control/request_manual")
        self.declare_parameter("set_speed_limit_service", "/mtt_control/set_auto_speed_limit")
        self.declare_parameter("state_staleness_timeout_s", 0.3)
        self.declare_parameter("disengage_zero_burst_count", 3)
        self.declare_parameter("segment_log_path", "")
        self.declare_parameter("skip_pause_topic", "/mtt_experiment/skip_pause")
        self.declare_parameter("checkpoint_ack_topic", "/mtt_experiment/checkpoint_ack")

        profile_path_str = str(self.get_parameter("profile_path").value)
        if not profile_path_str:
            raise RuntimeError("profile_path parameter is required (YAML segment file)")
        include_extended = bool(self.get_parameter("include_extended").value)
        self._command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self._auto_speed_cap_ms = float(self.get_parameter("auto_speed_cap_ms").value)
        self._speed_accel_limit = float(self.get_parameter("speed_accel_limit_mps2").value)
        self._phi_rate_limit = float(self.get_parameter("articulation_rate_limit_rad_s").value)
        self._max_phi = float(self.get_parameter("max_articulation_rad").value)
        self._state_staleness_timeout_s = float(self.get_parameter("state_staleness_timeout_s").value)
        self._disengage_zero_burst = int(self.get_parameter("disengage_zero_burst_count").value)

        self._segments = load_profile(Path(profile_path_str), include_extended)
        if not self._segments:
            raise RuntimeError(f"profile '{profile_path_str}' produced zero segments")
        n_core = sum(1 for s in self._segments if s.tier == "core")
        n_ext = sum(1 for s in self._segments if s.tier == "extended")
        self.get_logger().info(
            f"Loaded {len(self._segments)} segments from {profile_path_str} "
            f"(core={n_core}, extended={n_ext}, include_extended={include_extended})"
        )

        # --- state ---
        self._segment_index = 0
        self._segment_started_at: Optional[float] = None
        self._segment_attempt_zero_at: Optional[float] = None
        self._published_speed = 0.0
        self._published_phi = 0.0
        self._was_engaged = False
        self._speed_cap_confirmed = False
        self._pause_skip_requested = False
        self._checkpoint_ack_received = False
        self._checkpoint_ack_payload = ""
        self._zero_burst_remaining = 0
        self._autonomy_selected = ""
        self._last_ownership_request_at: Optional[float] = None
        self._last_ownership_release_at: Optional[float] = None
        self._autonomy_owner = str(self.get_parameter("autonomy_owner").value).strip().lower()
        self._ownership_retry_s = max(0.1, float(self.get_parameter("ownership_retry_s").value))
        if not self._autonomy_owner:
            raise RuntimeError("autonomy_owner must not be empty")

        self._selected_mode = ""
        self._mode_stamp: Optional[float] = None
        self._auto_enabled = False
        self._deadman_active = False
        self._deadman_stamp: Optional[float] = None
        self._estop_active = False

        # --- log file ---
        log_path_param = str(self.get_parameter("segment_log_path").value)
        if log_path_param:
            self._log_path = Path(log_path_param)
        else:
            ts = self.get_clock().now().to_msg().sec
            self._log_path = Path.cwd() / "data" / "ice_segment_logs" / f"segment_log_{ts}.csv"
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = open(self._log_path, "w", newline="")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow(
            [
                "segment_id", "label", "kind", "tier", "speed_ms", "phi_deg", "phi_rate_deg_s",
                "repeat", "attempt_start_wall", "attempt_end_wall", "engaged_duration_s",
                "target_duration_s", "completed", "notes",
            ]
        )
        self._log_file.flush()
        self.get_logger().info(f"Segment log: {self._log_path}")

        # --- QoS ---
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        volatile_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        mode_topic = str(self.get_parameter("mode_topic").value)
        auto_enabled_topic = str(self.get_parameter("auto_enabled_topic").value)
        deadman_topic = str(self.get_parameter("deadman_topic").value)
        estop_topic = str(self.get_parameter("estop_topic").value)

        self.create_subscription(String, mode_topic, self._on_mode, latched_qos)
        self.create_subscription(Bool, auto_enabled_topic, self._on_auto_enabled, latched_qos)
        self.create_subscription(Bool, deadman_topic, self._on_deadman, volatile_qos)
        self.create_subscription(Bool, estop_topic, self._on_estop, volatile_qos)
        self.create_subscription(
            String,
            str(self.get_parameter("autonomy_selected_topic").value),
            self._on_autonomy_selected,
            latched_qos,
        )
        self.create_subscription(
            Bool, str(self.get_parameter("skip_pause_topic").value), self._on_skip_pause, volatile_qos
        )
        self.create_subscription(
            String,
            str(self.get_parameter("checkpoint_ack_topic").value),
            self._on_checkpoint_ack,
            volatile_qos,
        )

        self._cmd_pub = self.create_publisher(TwistStamped, str(self.get_parameter("cmd_vel_topic").value), 20)
        self._articulation_pub = self.create_publisher(
            Float64, str(self.get_parameter("articulation_topic").value), 20
        )
        self._segment_pub = self.create_publisher(String, str(self.get_parameter("segment_topic").value), 20)
        self._autonomy_request_pub = self.create_publisher(
            String, str(self.get_parameter("autonomy_request_topic").value), volatile_qos
        )
        self._autonomy_release_pub = self.create_publisher(
            String, str(self.get_parameter("autonomy_release_topic").value), volatile_qos
        )

        self._request_manual_client = self.create_client(
            Trigger, str(self.get_parameter("request_manual_service").value)
        )
        self._speed_limit_client = self.create_client(
            SetSpeedLimit, str(self.get_parameter("set_speed_limit_service").value)
        )

        self._apply_speed_cap()

        period = 1.0 / max(self._command_rate_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().warn(
            f"Conductor ready: speed_cap_confirmed={self._speed_cap_confirmed}  "
            f"segments={len(self._segments)} autonomy_owner={self._autonomy_owner!r}  "
            "publishing ONLY while "
            "selected_mode==AUTO && deadman && !estop. Operator: hold deadman, "
            "press AUTO button to (re)arm; releasing deadman or STOP hands control "
            "back and requires a fresh AUTO press to resume."
        )

    # -- startup: hard precondition on the AUTO speed cap ------------------

    def _apply_speed_cap(self) -> None:
        if not self._speed_limit_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().fatal(
                "set_auto_speed_limit service unavailable after 5s; refusing to arm. "
                "The arbiter default AUTO cap (4.2 m/s) is unsafe on ice."
            )
            self._speed_cap_confirmed = False
            return
        request = SetSpeedLimit.Request()
        request.max_speed_ms = self._auto_speed_cap_ms
        future = self._speed_limit_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result() is not None and future.result().success:
            self.get_logger().warn(f"AUTO speed cap applied: {future.result().applied_max_speed_ms:.2f} m/s")
            self._speed_cap_confirmed = True
        else:
            self.get_logger().fatal(
                f"Failed to apply AUTO speed cap ({self._auto_speed_cap_ms} m/s); refusing to arm."
            )
            self._speed_cap_confirmed = False

    # -- subscriptions -------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_mode(self, msg: String) -> None:
        self._selected_mode = msg.data
        self._mode_stamp = self._now_s()

    def _on_auto_enabled(self, msg: Bool) -> None:
        self._auto_enabled = bool(msg.data)

    def _on_deadman(self, msg: Bool) -> None:
        self._deadman_active = bool(msg.data)
        self._deadman_stamp = self._now_s()

    def _on_estop(self, msg: Bool) -> None:
        self._estop_active = bool(msg.data)

    def _on_autonomy_selected(self, msg: String) -> None:
        self._autonomy_selected = msg.data.strip().lower()

    def _on_skip_pause(self, msg: Bool) -> None:
        if msg.data:
            self._pause_skip_requested = True

    def _on_checkpoint_ack(self, msg: String) -> None:
        # Any non-empty payload counts as an ACK; operators are encouraged to
        # publish JSON with at least {"ack": true} and ideally metadata
        # (mass_kg, position, operator) -- it is logged verbatim into the
        # segment log's notes column for traceability, but the content is not
        # validated (a bare "ack" string is accepted).
        if msg.data.strip():
            self._checkpoint_ack_received = True
            self._checkpoint_ack_payload = msg.data.strip()

    def _request_ownership(self) -> None:
        now = self._now_s()
        if (
            self._last_ownership_request_at is not None
            and (now - self._last_ownership_request_at) < self._ownership_retry_s
        ):
            return
        msg = String()
        msg.data = self._autonomy_owner
        self._autonomy_request_pub.publish(msg)
        self._last_ownership_request_at = now
        self.get_logger().info(
            f"Requesting autonomy ownership '{self._autonomy_owner}'",
            throttle_duration_sec=self._ownership_retry_s,
        )

    def _release_ownership(self) -> None:
        now = self._now_s()
        if (
            self._last_ownership_release_at is not None
            and (now - self._last_ownership_release_at) < self._ownership_retry_s
        ):
            return
        msg = String()
        msg.data = self._autonomy_owner
        self._autonomy_release_pub.publish(msg)
        self._last_ownership_release_at = now

    # -- engagement ------------------------------------------------------

    def _is_engaged(self) -> bool:
        if not self._speed_cap_confirmed:
            return False
        now = self._now_s()
        if self._mode_stamp is None or (now - self._mode_stamp) > self._state_staleness_timeout_s:
            return False
        if self._deadman_stamp is None or (now - self._deadman_stamp) > self._state_staleness_timeout_s:
            return False
        if self._estop_active:
            return False
        if self._selected_mode != "AUTO" or not self._auto_enabled:
            return False
        return self._deadman_active

    def _publish_zero(self) -> None:
        """Speed AND articulation to zero. Only for a deliberate, non-emergency
        park (profile complete, node shutdown) -- articulation=0 is an ACTIVE
        recentering command, not a "stop", so this must never be used on an
        operator-triggered disengage (see _publish_zero_speed_only)."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self._cmd_pub.publish(cmd)
        art = Float64()
        art.data = 0.0
        self._articulation_pub.publish(art)

    def _publish_zero_speed_only(self) -> None:
        """Speed to zero (always safe, never induces motion); articulation is
        NOT published at all, so the servo's own command_timeout_s (~0.25s)
        is what silences steering -- matches the documented disengage
        behavior (self-gate stops commanding, no active re-centering) instead
        of fighting the operator's regained manual control with a forced
        return to phi=0."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self._cmd_pub.publish(cmd)

    def _log_attempt(
        self, segment: AtomicSegment, start_wall: float, end_wall: float,
        engaged_duration_s: float, completed: bool, notes: str,
    ) -> None:
        self._log_writer.writerow(
            [
                segment.uid, segment.label, segment.kind, segment.tier,
                segment.meta.get("speed_ms", ""), segment.meta.get("phi_deg", ""),
                segment.meta.get("phi_rate_deg_s", ""), segment.meta.get("repeat", ""),
                f"{start_wall:.3f}", f"{end_wall:.3f}", f"{engaged_duration_s:.3f}",
                f"{segment.duration_s:.3f}", completed, notes,
            ]
        )
        self._log_file.flush()

    def _publish_segment_state(self, segment: AtomicSegment, elapsed: float, engaged: bool) -> None:
        payload = {
            "index": self._segment_index,
            "uid": segment.uid,
            "label": segment.label,
            "kind": segment.kind,
            "tier": segment.tier,
            "elapsed_s": round(elapsed, 3),
            "duration_s": round(segment.duration_s, 3),
            "engaged": engaged,
            "meta": segment.meta,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._segment_pub.publish(msg)

    # -- main loop ---------------------------------------------------------

    def _on_timer(self) -> None:
        if self._segment_index >= len(self._segments):
            if self._autonomy_selected == self._autonomy_owner:
                self._release_ownership()
            return  # profile complete; stay alive so the bag captures a clean tail

        segment = self._segments[self._segment_index]
        operator_engaged = self._is_engaged()
        if operator_engaged and self._autonomy_selected != self._autonomy_owner:
            self._request_ownership()
        elif not operator_engaged and self._autonomy_selected == self._autonomy_owner:
            self._release_ownership()
        engaged = operator_engaged and self._autonomy_selected == self._autonomy_owner
        dt = 1.0 / max(self._command_rate_hz, 1.0)

        # --- falling edge: disengage ---
        if self._was_engaged and not engaged:
            self.get_logger().warn(
                f"DISENGAGED during '{segment.uid}' (mode={self._selected_mode!r}, "
                f"deadman={self._deadman_active}, estop={self._estop_active}) -> "
                "stopping commands, requesting MANUAL, segment will RESTART on resume."
            )
            if self._segment_started_at is not None:
                elapsed = self._now_s() - self._segment_started_at
                self._log_attempt(
                    segment, self._segment_started_at, self._now_s(), elapsed, completed=False,
                    notes="aborted: disengaged",
                )
            self._zero_burst_remaining = self._disengage_zero_burst
            self._published_speed = 0.0
            self._published_phi = 0.0
            self._segment_started_at = None
            if self._request_manual_client.service_is_ready():
                self._request_manual_client.call_async(Trigger.Request())
            else:
                self.get_logger().warn("request_manual service not ready; relying on self-gate only.")

        if self._zero_burst_remaining > 0:
            self._publish_zero_speed_only()
            self._zero_burst_remaining -= 1
            self._was_engaged = engaged
            return

        # --- rising edge: (re)arm current segment from its beginning ---
        if engaged and self._segment_started_at is None:
            self._segment_started_at = self._now_s()
            self._pause_skip_requested = False
            if segment.requires_ack:
                self._checkpoint_ack_received = False
                self._checkpoint_ack_payload = ""
            self.get_logger().info(f"ARMED '{segment.uid}' ({segment.label}) duration={segment.duration_s:.1f}s")

        self._was_engaged = engaged

        if not engaged:
            # Not publishing anything: this is the intentional soft-stop behavior.
            return

        elapsed = self._now_s() - self._segment_started_at
        self._publish_segment_state(segment, elapsed, engaged=True)

        done = elapsed >= segment.duration_s
        if segment.requires_ack:
            # duration_s is a MINIMUM dwell only -- never auto-advances on
            # elapsed time alone, and skip_pause does NOT bypass it (that
            # shortcut is for repositioning pauses, not safety-relevant
            # confirmations like "the load is actually strapped").
            done = done and self._checkpoint_ack_received
        elif segment.is_pause and self._pause_skip_requested:
            done = True
        if segment.is_pause:
            if not done:
                if segment.requires_ack:
                    self.get_logger().warn(
                        f"CHECKPOINT (waiting for explicit ACK): {segment.pause_message} "
                        f"-- publish a non-empty std_msgs/String on "
                        f"{self.get_parameter('checkpoint_ack_topic').value} "
                        '(e.g. \'{"ack": true, "mass_kg": 45, "position": "centered"}\') '
                        "to continue. This segment will NOT auto-advance.",
                        throttle_duration_sec=8.0,
                    )
                else:
                    self.get_logger().info(
                        f"PAUSE: {segment.pause_message} "
                        f"({segment.duration_s - elapsed:.0f}s remaining, or publish "
                        f"Bool(true) on {self.get_parameter('skip_pause_topic').value} to continue)",
                        throttle_duration_sec=5.0,
                    )
            self._published_speed = 0.0
            self._published_phi = 0.0
        else:
            target_speed = clamp(segment.speed_fn(elapsed), -self._auto_speed_cap_ms, self._auto_speed_cap_ms)
            target_phi = clamp(segment.phi_fn(elapsed), -self._max_phi, self._max_phi)
            max_speed_step = self._speed_accel_limit * dt
            max_phi_step = self._phi_rate_limit * dt
            self._published_speed += clamp(
                target_speed - self._published_speed, -max_speed_step, max_speed_step
            )
            self._published_phi += clamp(target_phi - self._published_phi, -max_phi_step, max_phi_step)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = self._published_speed
        cmd.twist.angular.z = 0.0
        self._cmd_pub.publish(cmd)
        art = Float64()
        art.data = self._published_phi
        self._articulation_pub.publish(art)

        if done:
            notes = f"checkpoint_ack: {self._checkpoint_ack_payload}" if segment.requires_ack else ""
            self._log_attempt(
                segment, self._segment_started_at, self._now_s(), elapsed, completed=True, notes=notes,
            )
            self.get_logger().info(f"COMPLETE '{segment.uid}' ({elapsed:.1f}s)")
            self._segment_index += 1
            self._segment_started_at = None
            if self._segment_index >= len(self._segments):
                self.get_logger().warn("Profile complete. Holding zero; safe to end the session.")
                self._publish_zero()
                self._release_ownership()
                self._was_engaged = False

    def destroy_node(self) -> None:
        try:
            self._publish_zero()
            self._release_ownership()
            self._log_file.close()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MttExperimentConductor()
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
