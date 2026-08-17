#!/usr/bin/env python3
"""Offline analysis of a recorded ice-rink M0-M5 identification session.

Bridges the bag produced by demos/data_collection's `record` service (played
alongside mtt_experiment_conductor.py / mtt_experiment_monitor.py) to the
figures and tables the paper needs
(Paper/Publi_ICRA27_MohamedOUNALLY/main.tex, Sec. Model Hierarchy /
Dissipative Model). Self-contained: reads the mcap directly with the same
lightweight `mcap` + `mcap_ros2` approach as
artifacts/.../command_only/extract_command_topics.py, and reimplements the
small set of pure math functions it needs (M0 curvature law, M5-R deadband
law) locally rather than importing the artifacts tree, for the same reason
the live monitor does: no path-fragility across directories that may move.

Two tiers of output, matching the session plan:
  PRIMARY   (robust on ice; does not depend on ICP trajectory quality)
    - phi -> kappa table per steady-hold segment (Series B / speed-sweep),
      with the M0 overlay and a locally fit M5-R (gain + deadband + bias).
      Each row also carries a segment-median body-frame (vx, vy) from ICP
      (best-effort, degrades to null without ICP) -- vy is the priority-1
      fit target per the session's own header note (GT median |vy|=0.638 vs
      M5-predicted 0.024), since M0 imposes vy=0 exactly and every other
      model in the hierarchy predicts otherwise -- and an epsilon (quasi-
      static validity ratio) band across a range of candidate mu_y values,
      since mu_y itself is not yet a validated measurement.
    - M5-R fit identifiability diagnostic: bootstrap CIs + search-grid
      boundary-hit rate for gamma/phi_dead/phi0 (audit doc Sec.25.4).
    - deadband/hysteresis loop time series for each D1 ramp segment.
    - same-path speed-sweep comparison (rate-independence check).
  CONTINGENT (needs decent ICP continuity; degrades gracefully)
    - open-loop SE(2) rollout RPE (position + yaw error) at 1/2/5/10 s
      horizons for M0 vs the fitted M5-R, evaluated on the held-out
      figure-8 / PRBS segments only (never on the segments used to fit).

Usage:
    python3 analyze_ice_session.py <session_dir_or_bag_dir> [--out DIR]

<session_dir_or_bag_dir> follows the same convention as
scripts/audit_bag_timing.py: either the session directory (containing
`bag/metadata.yaml`) or the `bag/` directory itself.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from mcap.reader import NonSeekingReader, make_reader
from mcap_ros2.decoder import DecoderFactory

# --------------------------------------------------------------------------
# Constants shared with the conductor/monitor (kept in sync manually --
# these three are geometry/convention facts about the MTT-154, not tunables).
# --------------------------------------------------------------------------
# Measured geometry (corrected 2026-07-27 per
# artifacts/msa_canonical_mtt_calibration_test_garage_2026-06-02_09-01-53/README.md;
# the old CLAUDE.md/paper value L1=0.9 was ~65.8mm too large, per that audit).
L1_M = 0.834227   # tractor contact-center-to-hitch distance, measured
L2_M = 1.5135     # hitch-to-trailer-axle-pair-center distance, measured
L_M = L1_M + L2_M
PHI_SIGN = 1.0  # measured /hardware/articulation_angle already shares the conductor's raw
# commanded-setpoint sign convention (verified 2026-07-23: commanded +X deg tracks to
# measured +X deg on this pipeline). The all_models_reanalysis.py:80 convention of -1
# was for reconciling the July session's hardware interface against a DIFFERENT
# (lidar-derived) canonical source across two different data-collection pipelines; it
# does not apply here, where the conductor's command and this measurement are the only
# two signals and already agree.
#
# Global sign vs. the paper's theta2-theta1 / M0 convention: CONFIRMED 2026-07-28
# against M5_POSITION_ETENDUE_AUDIT_THEORIQUE_2026-07-26.md Sec.3.1, which fixes
# phi=theta2-theta1 (CCW positive yaw) and derives "une articulation positive produit
# une courbure negative en marche avant" (forward driving). Checked directly against
# every completed forward hold_arc row in both existing bags
# (artifacts/m5_field_sessions_2026-07-23/{ice_rink,open_terrain_asphalt}/analysis/
# phi_kappa_table.json, ~160 rows combined): phi>0 <-> kappa<0 and phi<0 <-> kappa>0
# holds with zero exceptions on both ice and asphalt. nominal_curvature_m0()'s leading
# minus (below) already encodes this, so no further global flip is needed anywhere in
# this file. /sensor/speed's direction sign remains unverified (see BagStreams docstring)
# — check it against a known-forward A2 speed-step segment on the next session.
SPEED_GATE_KAPPA_MS = 0.15
SPEED_GATE_TACH_MS = 0.20  # matches the frozen benchmark's tacho gate (all_models_reanalysis.py:237)
KAPPA_ABS_MAX = 1.5

# Quasi-static validity ratio (paper Sec. Extended Dynamics / audit doc Sec.19.2):
# epsilon = v^2 |kappa| / (mu_y * g). mu_y is NOT well known: the only measured proxy
# (garage audit's mu_2x=0.01242, mu_2y/mu_2x=56.36 => mu_2y~0.70) is a *trailer rolling*
# coefficient product the audit doc itself flags as a possible fixed-patch artifact
# (Sec.21), not a validated ground lateral-friction value, and ice/asphalt/grass differ
# hugely in real mu_y. Never quote epsilon as a single number -- always as a band over
# this candidate range, surface-agnostic until a dedicated friction measurement exists.
MU_Y_BAND = (0.08, 0.15, 0.35, 0.7)  # ice-low .. asphalt/grass-high, illustrative not measured
G_MS2 = 9.81

WANTED_TOPICS = {
    "/mtt_experiment/segment",
    "/hardware/articulation_angle",
    "mtt_tachometer",
    "/mtt_tachometer",
    "/mti100/data",
    "/mapping/icp_odom",
    "/mtt_odometry",
    "/sensor/speed",
}


def wrap_to_pi(a: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(a), np.cos(a))


def nominal_curvature_m0(phi_rad: np.ndarray) -> np.ndarray:
    """Exact M0 steady curvature, Paper/.../main.tex eq. (m0):
    kappa_M0(phi) = -sin(phi) / (l1*cos(phi) + l2) -- the leading minus is geometry, not
    a fitted choice (paper Sec. Platform, "Conventions"). Confirmed 2026-07-23 against
    real (phi, kappa) pairs: kappa_measured and this formula must share sign for the
    same phi; fit_m5r()/eval_m5r() already bake in this same leading minus via dz(...),
    so only this standalone display/overlay function needed the correction.
    """
    denom = L1_M * np.cos(phi_rad) + L2_M
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(denom > 1e-6, -np.sin(phi_rad) / denom, np.nan)


def dz(x: np.ndarray, r: float) -> np.ndarray:
    """Symmetric dead-zone, Paper/.../main.tex eq. (m5r)."""
    return np.sign(x) * np.maximum(np.abs(x) - r, 0.0)


def resample_hold(times: np.ndarray, values: np.ndarray, grid_t: np.ndarray, max_staleness_s: float = 1.0) -> np.ndarray:
    """Zero-order hold: value active at each grid time (previous sample), but only
    while that sample is fresher than max_staleness_s -- a topic that genuinely stops
    publishing mid-session must produce NaN (so downstream fallback chains can react),
    not silently keep serving an arbitrarily old value forever."""
    if len(times) == 0:
        return np.full_like(grid_t, np.nan)
    idx = np.searchsorted(times, grid_t, side="right") - 1
    out = np.full_like(grid_t, np.nan, dtype=float)
    valid = idx >= 0
    out[valid] = values[idx[valid]]
    staleness = grid_t - np.where(valid, times[np.clip(idx, 0, len(times) - 1)], -np.inf)
    out[staleness > max_staleness_s] = np.nan
    return out


def resample_linear(times: np.ndarray, values: np.ndarray, grid_t: np.ndarray, max_gap_s: float = 0.5) -> np.ndarray:
    """Linear interpolation, NaN outside the sample range or across large gaps."""
    if len(times) < 2:
        return np.full_like(grid_t, np.nan)
    out = np.interp(grid_t, times, values, left=np.nan, right=np.nan)
    gap_idx = np.searchsorted(times, grid_t, side="right")
    gap_idx = np.clip(gap_idx, 1, len(times) - 1)
    gaps = times[gap_idx] - times[gap_idx - 1]
    out[gaps > max_gap_s] = np.nan
    return out


# --------------------------------------------------------------------------
# Bag reading
# --------------------------------------------------------------------------


def resolve_mcap(path: Path) -> Path:
    bag_dir = path if (path / "metadata.yaml").exists() else path / "bag"
    if not (bag_dir / "metadata.yaml").exists():
        raise FileNotFoundError(f"no metadata.yaml under {path} or {path / 'bag'}")
    mcap_files = sorted(bag_dir.glob("*.mcap"))
    if len(mcap_files) != 1:
        raise RuntimeError(f"expected exactly one .mcap in {bag_dir}, found {len(mcap_files)}")
    return mcap_files[0]


def stamp_or_log_time(msg, log_time_s: float) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and (stamp.sec or stamp.nanosec):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return log_time_s


@dataclass
class BagStreams:
    segment_events: List[dict] = field(default_factory=list)  # decoded JSON payloads + t
    phi_t: List[float] = field(default_factory=list)
    phi_deg: List[float] = field(default_factory=list)  # canonical (PHI_SIGN applied)
    tach_t: List[float] = field(default_factory=list)
    tach_v: List[float] = field(default_factory=list)  # signed m/s
    # /sensor/speed (badger_driver canbus_driver.py): encoder-derived speed, m/s,
    # std_msgs/Float32, no direction field. SIGN CONVENTION NOT YET EMPIRICALLY
    # VERIFIED (2026-07-23: the robot was offline when this was added) -- assumed
    # already signed (matching mtt_tachometer's convention) since the publisher
    # applies no separate direction correction; sanity-check against a known-forward
    # A2 speed-step segment on the next session before fully trusting it, the same
    # way PHI_SIGN and nominal_curvature_m0's sign were confirmed against real data.
    sensor_speed_t: List[float] = field(default_factory=list)
    sensor_speed_v: List[float] = field(default_factory=list)
    imu_t: List[float] = field(default_factory=list)
    imu_yawrate: List[float] = field(default_factory=list)
    icp_t: List[float] = field(default_factory=list)
    icp_x: List[float] = field(default_factory=list)
    icp_y: List[float] = field(default_factory=list)
    icp_yaw: List[float] = field(default_factory=list)
    # Last-resort ground-speed fallback only (see build_grid). Deliberately NOT used
    # for yaw-rate or pose: /mtt_odometry is plausibly derived from the same M0
    # kinematic model this pipeline is trying to validate, so using its angular
    # rate or position as a reference would be circular. Its forward speed is closer
    # in nature to the tachometer (a direct wheel-speed quantity) and is fine as a
    # third redundancy layer for ground_speed only.
    odom_t: List[float] = field(default_factory=list)
    odom_v: List[float] = field(default_factory=list)


def _consume_record(streams: BagStreams, record) -> None:
    topic = record.channel.topic
    msg = record.decoded_message
    t = stamp_or_log_time(msg, record.message.log_time * 1e-9)
    if topic == "/mtt_experiment/segment":
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, AttributeError):
            return
        payload["_t"] = t
        streams.segment_events.append(payload)
    elif topic == "/hardware/articulation_angle":
        streams.phi_t.append(t)
        streams.phi_deg.append(math.degrees(PHI_SIGN * msg.data))
    elif topic in ("mtt_tachometer", "/mtt_tachometer"):
        sign = 1.0 if msg.direction == "Forward" else -1.0
        streams.tach_t.append(t)
        streams.tach_v.append(sign * msg.speed_ms)
    elif topic == "/sensor/speed":
        streams.sensor_speed_t.append(t)
        streams.sensor_speed_v.append(float(msg.data))
    elif topic == "/mti100/data":
        streams.imu_t.append(t)
        streams.imu_yawrate.append(msg.angular_velocity.z)
    elif topic == "/mapping/icp_odom":
        streams.icp_t.append(t)
        streams.icp_x.append(msg.pose.pose.position.x)
        streams.icp_y.append(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        streams.icp_yaw.append(
            math.atan2(2.0 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
        )
    elif topic == "/mtt_odometry":
        streams.odom_t.append(t)
        streams.odom_v.append(msg.twist.twist.linear.x)


def read_bag(mcap_path: Path) -> BagStreams:
    """Indexed read first (fast, uses the MCAP summary/footer). Falls back to a
    pure sequential (non-seeking) read if the summary is unreadable -- this is
    the normal signature of a session that ended without a clean shutdown
    (container killed, power/network loss): the MCAP writer never got to write
    a valid closing Footer/summary section, but the MESSAGES it already wrote
    are intact. `ros2 bag reindex` fixes the separate rosbag2 metadata.yaml
    sidecar but does NOT repair the MCAP file's own internal footer, so this
    fallback is still needed even after reindexing. Confirmed against a real
    corrupted bag 2026-08-02 (RecordLengthLimitExceeded on a garbage footer
    length -- the fallback recovered the full session)."""
    streams = BagStreams()
    try:
        with mcap_path.open("rb") as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for record in reader.iter_decoded_messages(topics=sorted(WANTED_TOPICS)):
                _consume_record(streams, record)
        return streams
    except Exception as exc:
        print(
            f"  Indexed read failed ({exc!r}) -- likely a corrupted MCAP footer from an "
            "unclean session end. Falling back to a sequential (non-seeking) read of the "
            "same file; this recovers all messages, just slower."
        )

    streams = BagStreams()  # discard any partial state from the failed indexed attempt
    n_recovered = 0
    with mcap_path.open("rb") as fh:
        reader = NonSeekingReader(fh, decoder_factories=[DecoderFactory()])
        try:
            # log_time_order=False is essential here, not an optimization: with no chunk
            # index to seek by, the default (True) has to buffer messages to sort them --
            # for a 15M-message/84GB bag that means trying to hold the whole file in RAM,
            # which OOM-killed this exact process with zero progress (confirmed 2026-08-02,
            # no exception raised, just SIGKILL). File order is fine for our purposes:
            # each topic's OWN stream is still written in the order its publisher produced
            # it, and every consumer below (resample_hold/resample_linear, group_segments)
            # only assumes per-topic monotonicity, never a global cross-topic ordering.
            for record in reader.iter_decoded_messages(topics=sorted(WANTED_TOPICS), log_time_order=False):
                _consume_record(streams, record)
                n_recovered += 1
        except Exception as exc:
            # A plain mid-record truncation (not just a garbage footer length) can make
            # the sequential reader itself choke on the final, incomplete record -- e.g.
            # a raw struct.error from a header that's cut off mid-field. Recovering
            # everything up to that point is still far better than raising and getting
            # nothing; only the last (at most one) partial record at the truncation
            # point is lost, matching the physical reality of an abrupt kill.
            print(
                f"  Sequential read also hit an error after recovering {n_recovered} messages "
                f"({exc!r}) -- likely the trailing record was mid-write when the process was "
                "killed. Keeping everything recovered up to that point."
            )
    return streams


# --------------------------------------------------------------------------
# Segment grouping (mirrors the conductor's restart-on-resume semantics:
# an aborted attempt resets elapsed_s to ~0, so we keep only the LAST
# contiguous run per uid -- the one that reached completion, if any).
# --------------------------------------------------------------------------


@dataclass
class SegmentInterval:
    uid: str
    kind: str
    tier: str
    label: str
    meta: dict
    t_start: float
    t_end: float
    max_elapsed: float
    duration: float
    completed: bool


def group_segments(events: List[dict]) -> List[SegmentInterval]:
    by_uid: Dict[str, List[dict]] = {}
    for e in events:
        by_uid.setdefault(e["uid"], []).append(e)

    intervals: List[SegmentInterval] = []
    for uid, evs in by_uid.items():
        evs.sort(key=lambda e: e["_t"])
        runs: List[List[dict]] = [[]]
        prev_elapsed = -1.0
        for e in evs:
            elapsed = float(e.get("elapsed_s", 0.0))
            if elapsed < prev_elapsed - 0.5:  # reset -> new attempt
                runs.append([])
            runs[-1].append(e)
            prev_elapsed = elapsed
        best = max(runs, key=lambda r: r[-1]["elapsed_s"] if r else -1.0)
        if not best:
            continue
        last = best[-1]
        duration = float(last.get("duration_s", 0.0))
        max_elapsed = float(last.get("elapsed_s", 0.0))
        intervals.append(
            SegmentInterval(
                uid=uid,
                kind=last.get("kind", ""),
                tier=last.get("tier", ""),
                label=last.get("label", ""),
                meta=last.get("meta", {}),
                t_start=best[0]["_t"],
                t_end=last["_t"],
                max_elapsed=max_elapsed,
                duration=duration,
                completed=duration > 0 and max_elapsed >= duration - 0.05,
            )
        )
    intervals.sort(key=lambda s: s.t_start)
    return intervals


# --------------------------------------------------------------------------
# Aligned grid + derived signals
# --------------------------------------------------------------------------


@dataclass
class AlignedGrid:
    t: np.ndarray
    phi_deg: np.ndarray
    phi_rad: np.ndarray
    tach_v: np.ndarray
    ground_speed: np.ndarray
    imu_yawrate: np.ndarray
    icp_yawrate: np.ndarray
    kappa: np.ndarray
    kappa_m0: np.ndarray
    icp_x: np.ndarray
    icp_y: np.ndarray
    icp_yaw: np.ndarray
    odom_v: np.ndarray
    sensor_speed_v: np.ndarray
    kappa_source: np.ndarray  # per-sample "icp"|"sensor_speed"|"tach"|"odom"|"none"
    vx_body_icp: np.ndarray  # ICP world velocity rotated into the body frame (forward)
    vy_body_icp: np.ndarray  # same, lateral -- the paper's priority-1 fit target (Sec.18/28.1
    # of the theory audit: M0 imposes vy=0 exactly, so a nonzero median vy on a steady hold
    # is direct, model-discriminating evidence M0 alone cannot produce).


def build_grid(streams: BagStreams, dt: float = 0.1) -> Optional[AlignedGrid]:
    all_t = streams.phi_t + streams.tach_t + streams.imu_t + streams.icp_t
    if not all_t:
        return None
    t0, t1 = min(all_t), max(all_t)
    grid_t = np.arange(t0, t1, dt)
    if len(grid_t) < 2:
        return None

    phi_deg = resample_hold(np.array(streams.phi_t), np.array(streams.phi_deg), grid_t)
    tach_v = resample_hold(np.array(streams.tach_t), np.array(streams.tach_v), grid_t)
    odom_v = resample_hold(np.array(streams.odom_t), np.array(streams.odom_v), grid_t)
    sensor_speed_v = resample_hold(np.array(streams.sensor_speed_t), np.array(streams.sensor_speed_v), grid_t)
    imu_yawrate = resample_linear(np.array(streams.imu_t), np.array(streams.imu_yawrate), grid_t, max_gap_s=0.3)

    icp_x = resample_linear(np.array(streams.icp_t), np.array(streams.icp_x), grid_t, max_gap_s=0.5)
    icp_y = resample_linear(np.array(streams.icp_t), np.array(streams.icp_y), grid_t, max_gap_s=0.5)
    icp_yaw_unwrapped = np.unwrap(np.array(streams.icp_yaw)) if streams.icp_yaw else np.array([])
    icp_yaw = resample_linear(np.array(streams.icp_t), icp_yaw_unwrapped, grid_t, max_gap_s=0.5)
    icp_yaw = wrap_to_pi(icp_yaw)

    dx = np.diff(icp_x, prepend=np.nan)
    dy = np.diff(icp_y, prepend=np.nan)
    dist = np.hypot(dx, dy)
    heading = np.arctan2(dy, dx)
    prev_yaw = np.roll(icp_yaw, 1)
    prev_yaw[0] = np.nan
    sign = np.where(np.abs(wrap_to_pi(heading - prev_yaw)) < math.pi / 2, 1.0, -1.0)
    ground_speed = sign * dist / dt
    icp_yawrate = np.diff(np.unwrap(np.nan_to_num(icp_yaw)), prepend=np.nan) / dt

    # Body-frame twist from ICP: rotate the world-frame finite-difference velocity
    # (vx_world, vy_world) = (dx/dt, dy/dt) into the body frame using yaw at the START
    # of each step (prev_yaw) -- consistent with a causal, no-relookahead estimate.
    # Per-sample values are noisy (known from the mathis dense-ICP cross-check: sample-
    # level vx/vy splits are unreliable even when hypot(vx,vy) matches GT speed) -- this
    # is why main() takes a segment-median over a steady hold rather than trusting any
    # single sample here.
    vx_world = dx / dt
    vy_world = dy / dt
    cos_y, sin_y = np.cos(prev_yaw), np.sin(prev_yaw)
    vx_body_icp = vx_world * cos_y + vy_world * sin_y
    vy_body_icp = -vx_world * sin_y + vy_world * cos_y

    # Ground-speed source priority: icp (slip-immune) > sensor_speed (precise encoder,
    # still wheel-based so not slip-immune -- sign unverified, see BagStreams docstring)
    # > tach (existing wheel speed, coarser) > mtt_odometry (last resort; NOT used for
    # yaw-rate/pose, see BagStreams docstring, only its forward speed). Every row is
    # tagged with which source produced it so slip-suspect fallback points can be
    # filtered out downstream.
    with np.errstate(invalid="ignore"):
        kappa_icp = np.where(np.abs(ground_speed) >= SPEED_GATE_KAPPA_MS, imu_yawrate / ground_speed, np.nan)
        kappa_icp = np.where(np.abs(kappa_icp) < KAPPA_ABS_MAX, kappa_icp, np.nan)
        kappa_sensor = np.where(np.abs(sensor_speed_v) >= SPEED_GATE_TACH_MS, imu_yawrate / sensor_speed_v, np.nan)
        kappa_sensor = np.where(np.abs(kappa_sensor) < KAPPA_ABS_MAX, kappa_sensor, np.nan)
        kappa_tach = np.where(np.abs(tach_v) >= SPEED_GATE_TACH_MS, imu_yawrate / tach_v, np.nan)
        kappa_tach = np.where(np.abs(kappa_tach) < KAPPA_ABS_MAX, kappa_tach, np.nan)
        kappa_odom = np.where(np.abs(odom_v) >= SPEED_GATE_TACH_MS, imu_yawrate / odom_v, np.nan)
        kappa_odom = np.where(np.abs(kappa_odom) < KAPPA_ABS_MAX, kappa_odom, np.nan)

    use_icp = np.isfinite(kappa_icp)
    use_sensor = ~use_icp & np.isfinite(kappa_sensor)
    use_tach = ~use_icp & ~use_sensor & np.isfinite(kappa_tach)
    use_odom = ~use_icp & ~use_sensor & ~use_tach & np.isfinite(kappa_odom)
    kappa = np.where(
        use_icp, kappa_icp,
        np.where(use_sensor, kappa_sensor, np.where(use_tach, kappa_tach, kappa_odom)),
    )
    kappa_source = np.where(
        use_icp, "icp",
        np.where(use_sensor, "sensor_speed", np.where(use_tach, "tach", np.where(use_odom, "odom", "none"))),
    )

    phi_rad = np.radians(phi_deg)
    kappa_m0 = nominal_curvature_m0(phi_rad)

    return AlignedGrid(
        t=grid_t, phi_deg=phi_deg, phi_rad=phi_rad, tach_v=tach_v, ground_speed=ground_speed,
        imu_yawrate=imu_yawrate, icp_yawrate=icp_yawrate, kappa=kappa, kappa_m0=kappa_m0,
        icp_x=icp_x, icp_y=icp_y, icp_yaw=icp_yaw, odom_v=odom_v, sensor_speed_v=sensor_speed_v,
        kappa_source=kappa_source, vx_body_icp=vx_body_icp, vy_body_icp=vy_body_icp,
    )


def epsilon_band(speed_ms: float, kappa: float) -> Dict[str, float]:
    """Quasi-static validity ratio eps = v^2|kappa| / (mu_y g), reported across
    MU_Y_BAND since mu_y itself is not reliably known yet -- see the module-level
    comment. Small eps everywhere in the band = safely quasi-static; if eps
    approaches or exceeds 1 even at the HIGH end of the band, the point is worth
    flagging as a fast/tight window per the audit doc Sec.19.2, to be scored
    separately from the quasi-static fit rather than folded into it."""
    if not (math.isfinite(speed_ms) and math.isfinite(kappa)):
        return {}
    return {f"mu_y_{mu:.2f}": (speed_ms ** 2) * abs(kappa) / (mu * G_MS2) for mu in MU_Y_BAND}


def window_mask(grid: AlignedGrid, t_start: float, t_end: float, tail_fraction: float = 0.5) -> np.ndarray:
    span = t_end - t_start
    window_start = t_start + (1.0 - tail_fraction) * span
    return (grid.t >= window_start) & (grid.t <= t_end)


# --------------------------------------------------------------------------
# M5-R fit: gamma * dz(phi - phi0; phi_dead) / L, coarse grid search over
# (dead, bias) with a closed-form least-squares gamma at each grid point.
# --------------------------------------------------------------------------


# Grid bounds for the (dead, bias) search. Widened 2026-07-28 from the original
# 0-15deg / +-6deg: the current protocol reaches +-40deg steady holds, so a true
# deadband anywhere near the old 15deg ceiling would have silently pinned to the
# grid edge and reported a confident-looking wrong number (advisor-flagged risk).
# Kept as module constants so fit_m5r_bootstrap can detect edge-pinned replicates
# using the exact same bounds the point estimate used.
DEAD_DEG_BOUNDS = (0.0, 22.0)
BIAS_DEG_BOUNDS = (-8.0, 8.0)


def fit_m5r(phi_deg: np.ndarray, kappa: np.ndarray) -> dict:
    valid = np.isfinite(phi_deg) & np.isfinite(kappa)
    phi_rad = np.radians(phi_deg[valid])
    y = kappa[valid]
    if len(y) < 5:
        return {"ok": False, "reason": "too few valid (phi, kappa) points"}

    best = None
    for dead_deg in np.linspace(DEAD_DEG_BOUNDS[0], DEAD_DEG_BOUNDS[1], 45):
        dead = math.radians(dead_deg)
        for bias_deg in np.linspace(BIAS_DEG_BOUNDS[0], BIAS_DEG_BOUNDS[1], 33):
            bias = math.radians(bias_deg)
            x = -dz(phi_rad - bias, dead) / L_M  # sign per eq.(m5r): kappa = -(gamma/L) dz(phi - phi0; phi_d)
            denom = float(np.dot(x, x))
            if denom < 1e-9:
                continue
            gamma = float(np.dot(x, y) / denom)
            resid = y - gamma * x
            sse = float(np.dot(resid, resid))
            if best is None or sse < best["sse"]:
                best = {"sse": sse, "gamma": gamma, "phi_dead_deg": dead_deg, "phi0_deg": bias_deg}
    if best is None:
        return {"ok": False, "reason": "grid search found no valid point"}
    best["ok"] = True
    best["rmse"] = math.sqrt(best["sse"] / len(y))
    best["n"] = len(y)
    return best


def fit_m5r_bootstrap(phi_deg: np.ndarray, kappa: np.ndarray, n_boot: int = 500, seed: int = 0) -> dict:
    """Identifiability diagnostic for the M5-R fit (audit doc Sec.25.4: singular
    values / condition number / bootstrap CIs / boundary behaviour before trusting
    published parameters). Resamples (phi, kappa) rows with replacement and refits
    each time. Reports CIs AND, more importantly with n in the 20-40 range on a
    quantized grid, the fraction of replicates that pin against a search-grid
    boundary -- a high boundary-hit rate means "not identifiable from this data",
    which a narrow-looking CI alone would hide."""
    valid = np.isfinite(phi_deg) & np.isfinite(kappa)
    phi_v, kappa_v = phi_deg[valid], kappa[valid]
    n = len(phi_v)
    if n < 5:
        return {"ok": False, "reason": "too few valid (phi, kappa) points", "n": n}

    rng = np.random.default_rng(seed)
    gammas, deads, biases = [], [], []
    dead_lo_hits = dead_hi_hits = bias_lo_hits = bias_hi_hits = 0
    for _ in range(n_boot):
        idx = rng.integers(0, n, size=n)
        fit = fit_m5r(phi_v[idx], kappa_v[idx])
        if not fit.get("ok"):
            continue
        gammas.append(fit["gamma"])
        deads.append(fit["phi_dead_deg"])
        biases.append(fit["phi0_deg"])
        if fit["phi_dead_deg"] <= DEAD_DEG_BOUNDS[0] + 1e-6:
            dead_lo_hits += 1
        if fit["phi_dead_deg"] >= DEAD_DEG_BOUNDS[1] - 1e-6:
            dead_hi_hits += 1
        if fit["phi0_deg"] <= BIAS_DEG_BOUNDS[0] + 1e-6:
            bias_lo_hits += 1
        if fit["phi0_deg"] >= BIAS_DEG_BOUNDS[1] - 1e-6:
            bias_hi_hits += 1
    n_ok = len(gammas)
    if n_ok == 0:
        return {"ok": False, "reason": "no bootstrap replicate converged", "n": n}

    def ci(vals: List[float]) -> Dict[str, float]:
        arr = np.array(vals)
        return {"p16": float(np.percentile(arr, 16)), "median": float(np.percentile(arr, 50)), "p84": float(np.percentile(arr, 84))}

    return {
        "ok": True,
        "n_points": n,
        "n_boot_requested": n_boot,
        "n_boot_converged": n_ok,
        "gamma_ci": ci(gammas),
        "phi_dead_deg_ci": ci(deads),
        "phi0_deg_ci": ci(biases),
        "boundary_hit_fraction": {
            "dead_at_0": dead_lo_hits / n_ok,
            "dead_at_max": dead_hi_hits / n_ok,
            "bias_at_min": bias_lo_hits / n_ok,
            "bias_at_max": bias_hi_hits / n_ok,
        },
        "search_bounds": {"dead_deg": DEAD_DEG_BOUNDS, "bias_deg": BIAS_DEG_BOUNDS},
        "warning": (
            "high boundary-hit fraction (>0.1 on any edge) means that parameter is "
            "not well identified from this data -- widen the search bounds and/or add "
            "dedicated trials, do not just trust the point estimate"
        ),
    }


def eval_m5r(phi_deg: np.ndarray, fit: dict) -> np.ndarray:
    phi_rad = np.radians(phi_deg)
    dead = math.radians(fit["phi_dead_deg"])
    bias = math.radians(fit["phi0_deg"])
    return -fit["gamma"] * dz(phi_rad - bias, dead) / L_M


# --------------------------------------------------------------------------
# SE(2) open-loop rollout RPE (contingent tier)
# --------------------------------------------------------------------------


def rollout_rpe(grid: AlignedGrid, t_start: float, t_end: float, kappa_pred: np.ndarray, horizons_s=(1, 2, 5, 10)) -> dict:
    mask = (grid.t >= t_start) & (grid.t <= t_end)
    idx = np.where(mask)[0]
    if len(idx) < 3:
        return {}
    t = grid.t[idx]
    # ground_speed (ICP-derived, slip-immune), not tach_v: tach over-reads on ice/loose
    # terrain when the belt spins faster than the ground, which would bias this rollout's
    # forward distance identically to the kappa fallback chain it was built to avoid
    # biasing (advisor-flagged inconsistency, fixed 2026-07-28).
    v = grid.ground_speed[idx]
    kp = kappa_pred[idx]
    x0, y0, yaw0 = grid.icp_x[idx[0]], grid.icp_y[idx[0]], grid.icp_yaw[idx[0]]
    if not (np.isfinite(x0) and np.isfinite(y0) and np.isfinite(yaw0)):
        return {}

    x, y, yaw = x0, y0, yaw0
    dt = np.diff(t, prepend=t[0])
    results = {}
    for h in horizons_s:
        target_t = t_start + h
        if target_t > t_end:
            continue
        results[h] = None  # filled below once we integrate that far

    for i in range(len(t)):
        if i > 0:
            step_dt = dt[i]
            vv = v[i] if np.isfinite(v[i]) else 0.0
            kk = kp[i] if np.isfinite(kp[i]) else 0.0
            yaw_mid = yaw + 0.5 * vv * kk * step_dt
            x += vv * math.cos(yaw_mid) * step_dt
            y += vv * math.sin(yaw_mid) * step_dt
            yaw = wrap_to_pi(yaw + vv * kk * step_dt)
        elapsed = t[i] - t_start
        for h in list(results.keys()):
            if results[h] is None and elapsed >= h:
                ix, iy, iyaw = grid.icp_x[idx[i]], grid.icp_y[idx[i]], grid.icp_yaw[idx[i]]
                if np.isfinite(ix) and np.isfinite(iy) and np.isfinite(iyaw):
                    pos_err = math.hypot(x - ix, y - iy)
                    yaw_err = abs(wrap_to_pi(yaw - iyaw))
                    results[h] = {"pos_err_m": pos_err, "yaw_err_rad": yaw_err}
    return {h: r for h, r in results.items() if r is not None}


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path, help="session dir or bag/ dir")
    parser.add_argument("--out", type=Path, default=None, help="output dir (default: <session>/analysis)")
    args = parser.parse_args()

    mcap_path = resolve_mcap(args.session)
    out_dir = args.out or (args.session / "analysis")
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"Reading {mcap_path} ...")
    streams = read_bag(mcap_path)
    print(
        f"  segment events={len(streams.segment_events)}  phi={len(streams.phi_t)}  "
        f"tach={len(streams.tach_t)}  imu={len(streams.imu_t)}  icp={len(streams.icp_t)}"
    )

    segments = group_segments(streams.segment_events)
    print(f"Grouped into {len(segments)} segment attempts (last-completed-run per uid).")

    grid = build_grid(streams)
    if grid is None:
        print("FATAL: could not build an aligned time grid (missing phi/tach/imu/icp streams). Aborting.")
        return

    # --- PRIMARY: phi -> kappa table (Series B + speed-sweep) ---
    phi_kappa_rows = []
    for seg in segments:
        if seg.kind != "hold_arc":
            continue
        mask = window_mask(grid, seg.t_start, seg.t_end, tail_fraction=0.5)
        if not np.any(mask):
            continue
        phi_deg = float(np.nanmedian(grid.phi_deg[mask]))
        kappa_finite = np.isfinite(grid.kappa[mask])
        kappa = float(np.nanmedian(grid.kappa[mask])) if np.any(kappa_finite) else float("nan")
        kappa_m0 = float(nominal_curvature_m0(np.radians(np.array([phi_deg])))[0])
        sources = grid.kappa_source[mask][kappa_finite]
        if len(sources) == 0:
            source = "none"
        else:
            values, counts = np.unique(sources, return_counts=True)
            source = str(values[np.argmax(counts)])

        # Priority-1 fit target (see AlignedGrid.vy_body_icp docstring): segment-median
        # body-frame twist from ICP, not a single sample. Needs decent ICP coverage in
        # the window -- degrades to null gracefully like the rest of the ICP-based tier.
        vxb = grid.vx_body_icp[mask]
        vyb = grid.vy_body_icp[mask]
        vxb_finite, vyb_finite = np.isfinite(vxb), np.isfinite(vyb)
        vx_body_med = float(np.nanmedian(vxb)) if np.any(vxb_finite) else None
        vy_body_med = float(np.nanmedian(vyb)) if np.any(vyb_finite) else None

        ground_speed_med = float(np.nanmedian(grid.ground_speed[mask])) if np.any(np.isfinite(grid.ground_speed[mask])) else float("nan")
        speed_for_eps = ground_speed_med if math.isfinite(ground_speed_med) else float(seg.meta.get("speed_ms") or float("nan"))
        eps = epsilon_band(abs(speed_for_eps), kappa) if math.isfinite(kappa) else {}

        phi_kappa_rows.append(
            {
                "uid": seg.uid, "speed_ms_target": seg.meta.get("speed_ms"), "phi_deg_measured": phi_deg,
                "kappa_measured": kappa, "kappa_m0": kappa_m0,
                "residual": (kappa - kappa_m0) if math.isfinite(kappa) else None,
                "kappa_source": source, "n_valid_samples": int(np.sum(kappa_finite)),
                "completed": seg.completed,
                "vx_body_icp_med": vx_body_med, "vy_body_icp_med": vy_body_med,
                "vy_nonzero_evidence_vs_m0": (
                    abs(vy_body_med) if vy_body_med is not None else None
                ),  # M0 imposes vy=0 exactly -- this IS the residual against M0 for this axis
                "epsilon_band": eps,  # quasi-static ratio across MU_Y_BAND; see module docstring
            }
        )
    with open(out_dir / "phi_kappa_table.json", "w") as fh:
        json.dump(phi_kappa_rows, fh, indent=2)
    print(f"Wrote {len(phi_kappa_rows)} rows to phi_kappa_table.json")

    # Require BOTH a finite kappa AND completed=True: an aborted hold (segment
    # cut short by a disengage) may not have reached steady state even if its
    # partial window happened to yield a finite median -- fitting on it would
    # silently mix transient and steady-state samples (audit finding, 2026-07-27).
    valid_rows = [
        r for r in phi_kappa_rows
        if r["kappa_measured"] == r["kappa_measured"] and r["completed"]  # not NaN, completed
    ]
    valid_phi = np.array([r["phi_deg_measured"] for r in valid_rows])
    valid_kappa = np.array([r["kappa_measured"] for r in valid_rows])
    fit = fit_m5r(valid_phi, valid_kappa)
    with open(out_dir / "m5r_fit.json", "w") as fh:
        json.dump(fit, fh, indent=2)
    if fit.get("ok"):
        print(
            f"M5-R fit: gamma={fit['gamma']:.3f}  phi_dead={fit['phi_dead_deg']:.1f}deg  "
            f"phi0={fit['phi0_deg']:.1f}deg  rmse={fit['rmse']:.4f}  n={fit['n']}"
        )
    else:
        print(f"M5-R fit FAILED: {fit.get('reason')}")

    # --- Identifiability diagnostic (audit doc Sec.25.4 / "ultra redondant" ask) ---
    boot = fit_m5r_bootstrap(valid_phi, valid_kappa)
    with open(out_dir / "m5r_fit_identifiability.json", "w") as fh:
        json.dump(boot, fh, indent=2)
    if boot.get("ok"):
        bh = boot["boundary_hit_fraction"]
        print(
            f"M5-R identifiability: n={boot['n_points']} boot={boot['n_boot_converged']}/{boot['n_boot_requested']}  "
            f"gamma CI=[{boot['gamma_ci']['p16']:.3f},{boot['gamma_ci']['p84']:.3f}]  "
            f"dead CI=[{boot['phi_dead_deg_ci']['p16']:.1f},{boot['phi_dead_deg_ci']['p84']:.1f}]deg  "
            f"boundary hits: dead0={bh['dead_at_0']:.2f} deadmax={bh['dead_at_max']:.2f} "
            f"biasmin={bh['bias_at_min']:.2f} biasmax={bh['bias_at_max']:.2f}"
        )
        if max(bh.values()) > 0.1:
            print("  WARNING: >10% of bootstrap replicates pin a search-grid boundary -- treat that parameter as NOT identified yet.")
    else:
        print(f"M5-R identifiability diagnostic unavailable: {boot.get('reason')}")

    # --- vy evidence summary (the paper's stated priority metric, Sec.18/28.1) ---
    vy_rows = [r for r in valid_rows if r["vy_body_icp_med"] is not None]
    if vy_rows:
        vy_abs_med = float(np.median([abs(r["vy_body_icp_med"]) for r in vy_rows]))
        print(f"vy evidence: median |vy_body_icp| over {len(vy_rows)} completed hold segments = {vy_abs_med:.3f} m/s (M0 imposes vy=0 exactly)")

    # --- PRIMARY: deadband/hysteresis loop time series (D1 ramps) ---
    for seg in segments:
        if seg.kind != "phi_ramp" or not seg.uid.startswith("D1"):
            continue
        mask = (grid.t >= seg.t_start) & (grid.t <= seg.t_end)
        rows = [
            {"t": float(grid.t[i] - seg.t_start), "phi_deg": float(grid.phi_deg[i]), "kappa": float(grid.kappa[i])}
            for i in np.where(mask)[0]
            if math.isfinite(grid.kappa[i])
        ]
        with open(out_dir / f"deadband_loop_{seg.uid}.json", "w") as fh:
            json.dump(rows, fh, indent=2)
        print(f"Wrote {len(rows)} samples to deadband_loop_{seg.uid}.json")

    # --- PRIMARY: same-path speed sweep ---
    sweep_rows = [r for r in phi_kappa_rows if r["uid"].startswith("speedsweep_arc")]
    if sweep_rows:
        kappas = [r["kappa_measured"] for r in sweep_rows if r["kappa_measured"] == r["kappa_measured"]]
        spread = (max(kappas) - min(kappas)) / max(abs(np.mean(kappas)), 1e-6) if kappas else float("nan")
        with open(out_dir / "speed_sweep.json", "w") as fh:
            json.dump({"rows": sweep_rows, "relative_spread": spread}, fh, indent=2)
        print(f"Speed sweep: {len(sweep_rows)} points, relative spread={spread:.3f} (rate-independence check; want small)")

    # --- CONTINGENT: RPE on held-out segments (figure8, prbs_phi) ---
    if fit.get("ok"):
        rpe_rows = []
        for seg in segments:
            if seg.kind not in ("figure8", "prbs_phi"):
                continue
            mask = (grid.t >= seg.t_start) & (grid.t <= seg.t_end)
            icp_ok = np.any(mask) and np.mean(np.isfinite(grid.icp_x[mask])) > 0.5
            if not icp_ok:
                print(f"  skip RPE for {seg.uid}: insufficient ICP coverage (contingent tier degraded)")
                continue
            kappa_m0_series = grid.kappa_m0
            kappa_m5r_series = eval_m5r(grid.phi_deg, fit)
            rpe_m0 = rollout_rpe(grid, seg.t_start, seg.t_end, kappa_m0_series)
            rpe_m5r = rollout_rpe(grid, seg.t_start, seg.t_end, kappa_m5r_series)
            rpe_rows.append({"uid": seg.uid, "kind": seg.kind, "M0": rpe_m0, "M5R": rpe_m5r})
        with open(out_dir / "rpe_table.json", "w") as fh:
            json.dump(rpe_rows, fh, indent=2)
        print(f"Wrote RPE table for {len(rpe_rows)} held-out segments to rpe_table.json")
    else:
        print("Skipping RPE table: M5-R fit unavailable.")

    print(f"\nAnalysis outputs in: {out_dir}")


if __name__ == "__main__":
    main()
