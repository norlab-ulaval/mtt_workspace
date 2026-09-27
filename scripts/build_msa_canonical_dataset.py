#!/usr/bin/env python3
"""Build the canonical MSA reference datasets for one MTT session.

This builder deliberately does not read or play the raw ROS bag.  It combines:

* the two 100 Hz MSA pose/twist CSV files;
* the already extracted 50 Hz ``aligned_samples.csv``;
* the recorded V1 Hesai extrinsic and the audited physical track-contact point.

Two gzip-compressed CSV files are produced:

``msa_track_100hz.csv.gz``
    MSA pose and twist expressed at ``track_contact_center``.  This is the
    operational ground-truth stream used for model/odometry evaluation.

``canonical_fused_50hz.csv.gz``
    The MSA reference interpolated at the timestamps of the existing aligned
    dataset, alongside commands, legacy articulation, tachometer, MTT odometry,
    and historical ICP.  Missing hardware articulation remains missing; it is
    never replaced by the legacy +/-60 degree-derived signal.

The output directory also contains a manifest, schema, and numerical quality
summary.  All transforms follow the convention ``T_A_B``: coordinates in B
are mapped into A.
"""

from __future__ import annotations

import argparse
import hashlib
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml
from scipy.spatial.transform import Rotation, Slerp
from scipy.signal import savgol_filter


SESSION_NAME = "mtt_calibration_test_garage_2026-06-02_09-01-53"

# Audited physical/CAD geometry, expressed in the recorded base_link.
TRACK_IN_BASE_M = np.array([-0.216961, -0.040672, 0.003815], dtype=float)
HESAI_IN_BASE_M = np.array(
    [-0.09748083811785774, -0.05002168970356846, 0.8783010371621075],
    dtype=float,
)
HESAI_IN_BASE_RPY_RAD = np.array([0.0, 0.0, math.pi / 2.0], dtype=float)

L_FRONT_M = 0.834227
L_REAR_M = 1.513500
TRACK_CONTACT_LENGTH_M = 1.225
TRACK_LOOP_LENGTH_NOMINAL_M = 3.93
ARTICULATION_LIMIT_PHYSICAL_DEG = 45.0
ARTICULATION_LIMIT_RECORDED_DEG = 60.0
TACHO_GLOBAL_SCALE_CANDIDATE = 0.993080

MSA_COLUMNS = [
    "time_s",
    "px_m",
    "py_m",
    "pz_m",
    "rx_rad",
    "ry_rad",
    "rz_rad",
    "vx_mps",
    "vy_mps",
    "vz_mps",
    "wx_rps",
    "wy_rps",
    "wz_rps",
]


@dataclass(frozen=True)
class RigidTransform:
    rotation: Rotation
    translation: np.ndarray

    def inverse(self) -> "RigidTransform":
        rinv = self.rotation.inv()
        return RigidTransform(rinv, -rinv.apply(self.translation))

    def compose(self, other: "RigidTransform") -> "RigidTransform":
        """Return self * other, i.e. T_A_B * T_B_C = T_A_C."""
        return RigidTransform(
            self.rotation * other.rotation,
            self.translation + self.rotation.apply(other.translation),
        )


def parse_args() -> argparse.Namespace:
    workspace = Path(__file__).resolve().parents[1]
    default_session = workspace / "data" / SESSION_NAME
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, default=default_session)
    parser.add_argument(
        "--msa-dir",
        type=Path,
        default=Path("/home/mohamed/Téléchargements/pose.norlab.bag_zstd_max_0"),
    )
    parser.add_argument(
        "--aligned-csv",
        type=Path,
        default=None,
        help="Default: <session>/motion_model_validation/aligned_samples.csv",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Default: <workspace>/artifacts/msa_canonical_<session>",
    )
    parser.add_argument("--filter-window-s", type=float, default=0.21)
    parser.add_argument("--filter-polyorder", type=int, default=3)
    parser.add_argument("--curvature-speed-gate-ms", type=float, default=0.10)
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_record(path: Path) -> dict[str, Any]:
    return {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def read_msa_csv(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path)
    if list(frame.columns) != MSA_COLUMNS:
        raise ValueError(f"Unexpected MSA schema in {path}: {list(frame.columns)}")
    values = frame.to_numpy(dtype=float)
    if not np.isfinite(values).all():
        raise ValueError(f"MSA file contains non-finite values: {path}")
    t = frame["time_s"].to_numpy()
    if not np.all(np.diff(t) > 0):
        raise ValueError(f"MSA timestamps are not strictly increasing: {path}")
    return frame


def poses(frame: pd.DataFrame) -> tuple[np.ndarray, Rotation]:
    p = frame[["px_m", "py_m", "pz_m"]].to_numpy(dtype=float)
    r = Rotation.from_rotvec(
        frame[["rx_rad", "ry_rad", "rz_rad"]].to_numpy(dtype=float)
    )
    return p, r


def estimate_robot_to_hesai(
    robot: pd.DataFrame, hesai: pd.DataFrame
) -> tuple[RigidTransform, dict[str, float]]:
    tr = robot["time_s"].to_numpy()
    th = hesai["time_s"].to_numpy()
    if robot.shape != hesai.shape or not np.array_equal(tr, th):
        raise ValueError("MSA robot and Hesai CSVs are not exactly timestamp-paired")

    p_wr, r_wr = poses(robot)
    p_wh, r_wh = poses(hesai)
    r_rh_all = r_wr.inv() * r_wh
    p_rh_all = r_wr.inv().apply(p_wh - p_wr)

    # Internal rigidity is near machine precision.  Rotation.mean() and the
    # componentwise translation median make the estimate insensitive to any
    # accidental isolated export outlier.
    r_rh = r_rh_all.mean()
    p_rh = np.median(p_rh_all, axis=0)
    rot_residual = (r_rh.inv() * r_rh_all).magnitude()
    trans_residual = np.linalg.norm(p_rh_all - p_rh, axis=1)
    quality = {
        "translation_residual_rms_m": float(
            np.sqrt(np.mean(trans_residual**2))
        ),
        "translation_residual_max_m": float(np.max(trans_residual)),
        "rotation_residual_rms_deg": float(
            np.degrees(np.sqrt(np.mean(rot_residual**2)))
        ),
        "rotation_residual_max_deg": float(np.degrees(np.max(rot_residual))),
    }
    return RigidTransform(r_rh, p_rh), quality


def transform_to_track(
    robot: pd.DataFrame,
    hesai: pd.DataFrame,
    t_track_robot: RigidTransform,
    t_track_hesai: RigidTransform,
) -> tuple[pd.DataFrame, dict[str, float]]:
    p_wr, r_wr = poses(robot)
    p_wh, r_wh = poses(hesai)

    t_robot_track = t_track_robot.inverse()
    t_hesai_track = t_track_hesai.inverse()

    r_wt = r_wr * t_robot_track.rotation
    p_wt = p_wr + r_wr.apply(t_robot_track.translation)
    r_wt_h = r_wh * t_hesai_track.rotation
    p_wt_h = p_wh + r_wh.apply(t_hesai_track.translation)

    position_crosscheck = np.linalg.norm(p_wt - p_wt_h, axis=1)
    rotation_crosscheck = (r_wt.inv() * r_wt_h).magnitude()

    v_r = robot[["vx_mps", "vy_mps", "vz_mps"]].to_numpy(dtype=float)
    w_r = robot[["wx_rps", "wy_rps", "wz_rps"]].to_numpy(dtype=float)
    w_t = t_track_robot.rotation.apply(w_r)
    v_t = t_track_robot.rotation.apply(v_r) + np.cross(
        np.broadcast_to(t_track_robot.translation, w_t.shape), w_t
    )

    quaternion = r_wt.as_quat()
    rpy = r_wt.as_euler("xyz")
    rotvec = r_wt.as_rotvec()
    result = pd.DataFrame(
        {
            "time_s": robot["time_s"].to_numpy(dtype=float),
            "gt_track_x_m": p_wt[:, 0],
            "gt_track_y_m": p_wt[:, 1],
            "gt_track_z_m": p_wt[:, 2],
            "gt_track_qx": quaternion[:, 0],
            "gt_track_qy": quaternion[:, 1],
            "gt_track_qz": quaternion[:, 2],
            "gt_track_qw": quaternion[:, 3],
            "gt_track_roll_rad": rpy[:, 0],
            "gt_track_pitch_rad": rpy[:, 1],
            "gt_track_yaw_rad": rpy[:, 2],
            "gt_track_rotvec_x_rad": rotvec[:, 0],
            "gt_track_rotvec_y_rad": rotvec[:, 1],
            "gt_track_rotvec_z_rad": rotvec[:, 2],
            "gt_track_vx_raw_ms": v_t[:, 0],
            "gt_track_vy_raw_ms": v_t[:, 1],
            "gt_track_vz_raw_ms": v_t[:, 2],
            "gt_track_wx_raw_rad_s": w_t[:, 0],
            "gt_track_wy_raw_rad_s": w_t[:, 1],
            "gt_track_wz_raw_rad_s": w_t[:, 2],
        }
    )
    quality = {
        "robot_vs_hesai_track_position_rms_m": float(
            np.sqrt(np.mean(position_crosscheck**2))
        ),
        "robot_vs_hesai_track_position_max_m": float(
            np.max(position_crosscheck)
        ),
        "robot_vs_hesai_track_rotation_rms_deg": float(
            np.degrees(np.sqrt(np.mean(rotation_crosscheck**2)))
        ),
        "robot_vs_hesai_track_rotation_max_deg": float(
            np.degrees(np.max(rotation_crosscheck))
        ),
    }
    return result, quality


def odd_window_samples(window_s: float, dt: float, polyorder: int) -> int:
    count = max(polyorder + 2, int(round(window_s / dt)))
    if count % 2 == 0:
        count += 1
    return count


def add_derived_reference(
    frame: pd.DataFrame,
    filter_window_s: float,
    polyorder: int,
    speed_gate_ms: float,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    result = frame.copy()
    t = result["time_s"].to_numpy(dtype=float)
    dt = float(np.median(np.diff(t)))
    window = odd_window_samples(filter_window_s, dt, polyorder)

    raw_names = [
        "gt_track_vx_raw_ms",
        "gt_track_vy_raw_ms",
        "gt_track_vz_raw_ms",
        "gt_track_wx_raw_rad_s",
        "gt_track_wy_raw_rad_s",
        "gt_track_wz_raw_rad_s",
    ]
    filtered: dict[str, np.ndarray] = {}
    derivatives: dict[str, np.ndarray] = {}
    for name in raw_names:
        x = result[name].to_numpy(dtype=float)
        filtered[name] = savgol_filter(
            x, window_length=window, polyorder=polyorder, mode="interp"
        )
        derivatives[name] = savgol_filter(
            x,
            window_length=window,
            polyorder=polyorder,
            deriv=1,
            delta=dt,
            mode="interp",
        )

    result["gt_track_vx_filt_ms"] = filtered["gt_track_vx_raw_ms"]
    result["gt_track_vy_filt_ms"] = filtered["gt_track_vy_raw_ms"]
    result["gt_track_vz_filt_ms"] = filtered["gt_track_vz_raw_ms"]
    result["gt_track_wx_filt_rad_s"] = filtered["gt_track_wx_raw_rad_s"]
    result["gt_track_wy_filt_rad_s"] = filtered["gt_track_wy_raw_rad_s"]
    result["gt_track_wz_filt_rad_s"] = filtered["gt_track_wz_raw_rad_s"]
    result["gt_track_dvx_dt_ms2"] = derivatives["gt_track_vx_raw_ms"]
    result["gt_track_dvy_dt_ms2"] = derivatives["gt_track_vy_raw_ms"]
    result["gt_track_dvz_dt_ms2"] = derivatives["gt_track_vz_raw_ms"]
    result["gt_track_yaw_accel_rad_s2"] = derivatives[
        "gt_track_wz_raw_rad_s"
    ]

    velocity = np.column_stack(
        [
            result["gt_track_vx_filt_ms"],
            result["gt_track_vy_filt_ms"],
            result["gt_track_vz_filt_ms"],
        ]
    )
    omega = np.column_stack(
        [
            result["gt_track_wx_filt_rad_s"],
            result["gt_track_wy_filt_rad_s"],
            result["gt_track_wz_filt_rad_s"],
        ]
    )
    dv_components = np.column_stack(
        [
            result["gt_track_dvx_dt_ms2"],
            result["gt_track_dvy_dt_ms2"],
            result["gt_track_dvz_dt_ms2"],
        ]
    )
    inertial_accel_body = dv_components + np.cross(omega, velocity)
    result["gt_track_ax_body_ms2"] = inertial_accel_body[:, 0]
    result["gt_track_ay_body_ms2"] = inertial_accel_body[:, 1]
    result["gt_track_az_body_ms2"] = inertial_accel_body[:, 2]

    vx = result["gt_track_vx_filt_ms"].to_numpy(dtype=float)
    vy = result["gt_track_vy_filt_ms"].to_numpy(dtype=float)
    wz = result["gt_track_wz_filt_rad_s"].to_numpy(dtype=float)
    planar_speed = np.hypot(vx, vy)
    signed_path_speed = np.sign(vx) * planar_speed
    moving = np.abs(vx) >= speed_gate_ms
    result["gt_track_planar_speed_ms"] = planar_speed
    result["gt_track_signed_path_speed_ms"] = signed_path_speed
    result["gt_track_sideslip_rad"] = np.arctan2(vy, np.maximum(np.abs(vx), 1e-12))
    result["gt_track_curvature_body_1pm"] = np.where(moving, wz / vx, np.nan)
    result["gt_track_curvature_path_1pm"] = np.where(
        np.abs(signed_path_speed) >= speed_gate_ms,
        wz / signed_path_speed,
        np.nan,
    )
    result["valid_gt_curvature"] = moving

    xy = result[["gt_track_x_m", "gt_track_y_m"]].to_numpy(dtype=float)
    ds = np.r_[0.0, np.linalg.norm(np.diff(xy, axis=0), axis=1)]
    result["gt_track_step_distance_m"] = ds
    result["gt_track_path_s_m"] = np.cumsum(ds)
    result["gt_track_yaw_unwrapped_rad"] = np.unwrap(
        result["gt_track_yaw_rad"].to_numpy(dtype=float)
    )
    result["time_from_start_s"] = t - t[0]
    result["valid_gt_pose"] = True
    result["reference_frame"] = "track_contact_center"
    result["reference_source"] = "MSA_offline_batch"
    result["calibration_profile"] = "v1_recorded+msa_v1_transform"

    # One-session exploratory split.  Windows are accepted only if their start
    # and end carry the same label; this prevents horizon leakage at boundaries.
    span = t[-1] - t[0]
    train_end = t[0] + 0.50 * span
    validation_end = t[0] + 0.75 * span
    split = np.where(
        t < train_end,
        "train",
        np.where(t < validation_end, "validation", "test"),
    )
    result["split_contiguous_v1"] = split
    result["split_boundary_distance_s"] = np.minimum.reduce(
        [
            np.abs(t - t[0]),
            np.abs(t - train_end),
            np.abs(t - validation_end),
            np.abs(t - t[-1]),
        ]
    )

    filter_info = {
        "type": "Savitzky-Golay",
        "phase": "zero_phase_symmetric_offline",
        "window_requested_s": filter_window_s,
        "window_samples": window,
        "window_effective_s": window * dt,
        "polyorder": polyorder,
        "sample_period_median_s": dt,
        "curvature_speed_gate_ms": speed_gate_ms,
        "curvature_primary_definition": "wz_filtered / vx_filtered",
        "acceleration_definition": "d(v_body)/dt + omega_body cross v_body",
    }
    return result, filter_info


def interpolate_reference(
    reference: pd.DataFrame, target_t: np.ndarray
) -> pd.DataFrame:
    source_t = reference["time_s"].to_numpy(dtype=float)
    valid = (target_t >= source_t[0]) & (target_t <= source_t[-1])
    out = pd.DataFrame({"time_s": target_t})

    scalar_columns = [
        c
        for c in reference.columns
        if c != "time_s"
        and pd.api.types.is_numeric_dtype(reference[c])
        and c
        not in {
            "gt_track_qx",
            "gt_track_qy",
            "gt_track_qz",
            "gt_track_qw",
            "gt_track_roll_rad",
            "gt_track_pitch_rad",
            "gt_track_yaw_rad",
            "gt_track_yaw_unwrapped_rad",
            "valid_gt_pose",
            "valid_gt_curvature",
        }
    ]
    for column in scalar_columns:
        values = reference[column].to_numpy(dtype=float)
        out[column] = np.interp(target_t, source_t, values, left=np.nan, right=np.nan)

    quaternion = reference[
        ["gt_track_qx", "gt_track_qy", "gt_track_qz", "gt_track_qw"]
    ].to_numpy(dtype=float)
    q_target = np.full((len(target_t), 4), np.nan)
    if np.any(valid):
        slerp = Slerp(source_t, Rotation.from_quat(quaternion))
        q_target[valid] = slerp(target_t[valid]).as_quat()
    out[["gt_track_qx", "gt_track_qy", "gt_track_qz", "gt_track_qw"]] = q_target
    rpy = np.full((len(target_t), 3), np.nan)
    if np.any(valid):
        rpy[valid] = Rotation.from_quat(q_target[valid]).as_euler("xyz")
    out[["gt_track_roll_rad", "gt_track_pitch_rad", "gt_track_yaw_rad"]] = rpy
    yaw_unwrapped = np.full(len(target_t), np.nan)
    if np.any(valid):
        yaw_unwrapped[valid] = np.unwrap(rpy[valid, 2])
    out["gt_track_yaw_unwrapped_rad"] = yaw_unwrapped
    out["valid_gt_pose"] = valid

    vx = out["gt_track_vx_filt_ms"].to_numpy(dtype=float)
    out["valid_gt_curvature"] = valid & np.isfinite(vx) & (
        np.abs(vx) >= 0.10
    )
    return out


def build_fused(
    aligned_path: Path, reference: pd.DataFrame
) -> pd.DataFrame:
    aligned = pd.read_csv(aligned_path)
    if "t" not in aligned:
        raise ValueError(f"Missing t column in {aligned_path}")
    aligned_t = aligned["t"].to_numpy(dtype=float)
    if not np.all(np.diff(aligned_t) > 0):
        raise ValueError("aligned_samples.csv timestamps are not strictly increasing")
    gt = interpolate_reference(reference, aligned_t)

    rename = {
        "t": "time_s",
        "odom_x": "mtt_odom_x_m",
        "odom_y": "mtt_odom_y_m",
        "odom_heading": "mtt_odom_yaw_rad",
        "odom_linear_x": "mtt_odom_vx_ms",
        "odom_angular_z": "mtt_odom_wz_rad_s",
        "icp_x": "legacy_icp_hesai_x_m",
        "icp_y": "legacy_icp_hesai_y_m",
        "icp_heading": "legacy_icp_hesai_yaw_rad",
        "icp_linear_x": "legacy_icp_hesai_vx_ms",
        "icp_angular_z": "legacy_icp_hesai_wz_rad_s",
        "cmd_linear_x": "cmd_vel_linear_x_ms",
        "cmd_angular_z": "cmd_vel_angular_z_normalized",
        "teleop_linear_x": "cmd_teleop_linear_x_ms",
        "teleop_angular_z": "cmd_teleop_angular_z_normalized",
        "controller_linear_x": "cmd_controller_linear_x_ms",
        "controller_angular_z": "cmd_controller_angular_z_normalized",
        "articulation_rad": "articulation_legacy_60deg_rad",
        "status_speed_ms": "status_measured_speed_ms",
        "status_steer_normalized": "status_steer_normalized",
        "status_command_linear_speed_ms": "status_command_speed_ms",
        "status_effective_linear_speed_command_ms": "status_effective_speed_command_ms",
        "tach_speed_ms": "tacho_speed_magnitude_raw_ms",
        "tach_model_speed_ms": "tacho_model_speed_legacy_ms",
        "tach_direction": "tacho_direction",
        "tach_steer_cmd": "tacho_steer_cmd_normalized",
        "tach_model_yaw_rate_rad_s": "tacho_model_yaw_rate_legacy_rad_s",
        "tach_model_articulation_rad": "tacho_model_articulation_legacy_rad",
        "tach_model_state_valid": "tacho_model_state_valid",
        "tachometer_is_synthetic": "tacho_is_synthetic",
        "tachometer_source": "tacho_source",
        "model_x": "legacy_model_x_m",
        "model_y": "legacy_model_y_m",
        "model_heading": "legacy_model_yaw_rad",
    }
    keep = [column for column in rename if column in aligned.columns]
    signals = aligned[keep].rename(columns={k: v for k, v in rename.items() if k in keep})
    # ``time_s`` is already the join key in the interpolated GT frame.
    signals = signals.drop(columns=["time_s"], errors="ignore")
    fused = pd.concat([gt, signals.reset_index(drop=True)], axis=1)

    direction = fused.get("tacho_direction", pd.Series("", index=fused.index)).astype(str)
    magnitude = pd.to_numeric(
        fused.get("tacho_speed_magnitude_raw_ms", np.nan), errors="coerce"
    )
    direction_sign = np.where(
        direction.str.casefold().eq("reverse"),
        -1.0,
        np.where(direction.str.casefold().eq("forward"), 1.0, np.nan),
    )
    fused["tacho_speed_signed_raw_ms"] = magnitude * direction_sign
    fused["tacho_speed_signed_global_candidate_ms"] = (
        TACHO_GLOBAL_SCALE_CANDIDATE * fused["tacho_speed_signed_raw_ms"]
    )
    fused["tacho_global_scale_candidate"] = TACHO_GLOBAL_SCALE_CANDIDATE
    fused["tacho_global_scale_is_final"] = False

    # Explicitly reserve the authoritative hardware channel.  It will be
    # populated by a raw-topic extraction after the concurrent replay/mapper
    # job is finished.
    fused["articulation_hardware_rad"] = np.nan
    fused["articulation_hardware_available"] = False
    fused["articulation_hardware_source"] = "not_extracted_yet"
    fused["articulation_legacy_is_hardware"] = False
    fused["articulation_limit_physical_deg"] = ARTICULATION_LIMIT_PHYSICAL_DEG
    fused["articulation_limit_recorded_deg"] = ARTICULATION_LIMIT_RECORDED_DEG
    fused["legacy_icp_is_ground_truth"] = False
    fused["gt_source"] = "MSA_offline_batch"
    fused["gt_frame"] = "track_contact_center"

    source_t = reference["time_s"].to_numpy(dtype=float)
    split_edges = [
        source_t[0] + 0.50 * (source_t[-1] - source_t[0]),
        source_t[0] + 0.75 * (source_t[-1] - source_t[0]),
    ]
    fused["split_contiguous_v1"] = np.where(
        aligned_t < split_edges[0],
        "train",
        np.where(aligned_t < split_edges[1], "validation", "test"),
    )
    fused["time_from_start_s"] = aligned_t - source_t[0]
    return fused


def finite_stats(values: np.ndarray) -> dict[str, float | int]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return {"count": 0}
    return {
        "count": int(finite.size),
        "min": float(np.min(finite)),
        "median": float(np.median(finite)),
        "p95": float(np.percentile(finite, 95)),
        "max": float(np.max(finite)),
        "mean": float(np.mean(finite)),
        "rmse": float(np.sqrt(np.mean(finite**2))),
    }


def transform_record(transform: RigidTransform) -> dict[str, Any]:
    return {
        "translation_m": [float(x) for x in transform.translation],
        "quaternion_xyzw": [float(x) for x in transform.rotation.as_quat()],
        "rpy_deg": [
            float(x) for x in np.degrees(transform.rotation.as_euler("xyz"))
        ],
        "convention": "T_A_B maps coordinates from B into A",
    }


def write_yaml(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    session = args.session.resolve()
    msa_dir = args.msa_dir.resolve()
    aligned_path = (
        args.aligned_csv.resolve()
        if args.aligned_csv
        else session / "motion_model_validation" / "aligned_samples.csv"
    )
    out_dir = (
        args.output_dir.resolve()
        if args.output_dir
        else (
            Path(__file__).resolve().parents[1]
            / "artifacts"
            / f"msa_canonical_{session.name}"
        )
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    robot_path = msa_dir / "pose_robot.local.csv"
    hesai_path = msa_dir / "pose_hesai_lidar.local.csv"
    for required in (robot_path, hesai_path, aligned_path):
        if not required.is_file():
            raise FileNotFoundError(required)

    robot = read_msa_csv(robot_path)
    hesai = read_msa_csv(hesai_path)
    t_robot_hesai, rigidity_quality = estimate_robot_to_hesai(robot, hesai)

    t_base_track = RigidTransform(Rotation.identity(), TRACK_IN_BASE_M)
    t_base_hesai = RigidTransform(
        Rotation.from_euler("xyz", HESAI_IN_BASE_RPY_RAD), HESAI_IN_BASE_M
    )
    t_track_hesai = t_base_track.inverse().compose(t_base_hesai)
    t_track_robot = t_track_hesai.compose(t_robot_hesai.inverse())

    reference, crosscheck_quality = transform_to_track(
        robot, hesai, t_track_robot, t_track_hesai
    )
    reference, filter_info = add_derived_reference(
        reference,
        filter_window_s=args.filter_window_s,
        polyorder=args.filter_polyorder,
        speed_gate_ms=args.curvature_speed_gate_ms,
    )
    fused = build_fused(aligned_path, reference)

    reference_path = out_dir / "msa_track_100hz.csv.gz"
    fused_path = out_dir / "canonical_fused_50hz.csv.gz"
    reference.to_csv(reference_path, index=False, compression="gzip")
    fused.to_csv(fused_path, index=False, compression="gzip")

    dt100 = np.diff(reference["time_s"].to_numpy(dtype=float))
    dt50 = np.diff(fused["time_s"].to_numpy(dtype=float))
    yaw_pose_rate = np.gradient(
        reference["gt_track_yaw_unwrapped_rad"].to_numpy(dtype=float),
        reference["time_s"].to_numpy(dtype=float),
    )
    yaw_twist = reference["gt_track_wz_raw_rad_s"].to_numpy(dtype=float)
    speed = reference["gt_track_planar_speed_ms"].to_numpy(dtype=float)
    quality_summary = {
        "session": session.name,
        "reference_rows": len(reference),
        "fused_rows": len(fused),
        "reference_duration_s": float(
            reference["time_s"].iloc[-1] - reference["time_s"].iloc[0]
        ),
        "reference_sample_period_s": finite_stats(dt100),
        "fused_sample_period_s": finite_stats(dt50),
        "rigid_pair_quality": rigidity_quality,
        "track_pose_crosscheck_quality": crosscheck_quality,
        "pose_vs_twist_yaw_rate_residual_rad_s": finite_stats(
            yaw_pose_rate - yaw_twist
        ),
        "planar_speed_ms": finite_stats(speed),
        "path_length_m": float(reference["gt_track_path_s_m"].iloc[-1]),
        "moving_fraction_abs_vx_gt_0p1": float(
            np.mean(np.abs(reference["gt_track_vx_filt_ms"]) >= 0.1)
        ),
        "reverse_fraction_while_moving": float(
            np.mean(
                reference.loc[
                    np.abs(reference["gt_track_vx_filt_ms"]) >= 0.1,
                    "gt_track_vx_filt_ms",
                ]
                < 0
            )
        ),
        "hardware_articulation_available": False,
        "warnings": [
            "The fused non-MSA signals come from the historical aligned_samples.csv, not a fresh raw-topic extraction.",
            "articulation_legacy_60deg_rad is not the physical hardware angle and must not be used as ground truth.",
            "The global tachometer scale 0.993080 is a candidate only; regime-specific calibration remains required.",
            "Historical ICP remains an estimator in the Hesai frame and is not the MSA ground truth.",
            "The one-session contiguous split is exploratory; it does not create independent experimental sessions.",
        ],
    }
    write_yaml(out_dir / "quality_summary.yaml", quality_summary)

    schema = {
        "reference_csv": {
            "path": reference_path.name,
            "row_rate": "100 Hz nominal",
            "primary_pose_columns": [
                "gt_track_x_m",
                "gt_track_y_m",
                "gt_track_yaw_rad",
            ],
            "primary_twist_columns": [
                "gt_track_vx_filt_ms",
                "gt_track_vy_filt_ms",
                "gt_track_wz_filt_rad_s",
            ],
            "primary_curvature_column": "gt_track_curvature_body_1pm",
        },
        "fused_csv": {
            "path": fused_path.name,
            "row_rate": "approximately 50 Hz, inherited from aligned_samples.csv",
            "authoritative_articulation_column": "articulation_hardware_rad",
            "authoritative_articulation_status": "missing until raw topic extraction",
            "legacy_articulation_column": "articulation_legacy_60deg_rad",
            "command_columns": [
                "status_effective_speed_command_ms",
                "status_steer_normalized",
                "cmd_vel_linear_x_ms",
                "cmd_vel_angular_z_normalized",
            ],
            "tachometer_columns": [
                "tacho_speed_signed_raw_ms",
                "tacho_speed_signed_global_candidate_ms",
            ],
        },
        "angle_convention": {
            "yaw": "positive counter-clockwise",
            "paper_articulation": "phi = theta_trailer - theta_tractor; positive raw sensor sign still requires verification against hardware angle and MSA yaw response",
        },
    }
    write_yaml(out_dir / "schema.yaml", schema)

    calibration_zip = msa_dir / "msa_cal.bot0.norlab.norlab.bag_zstd_max_0.zip"
    metadata_path = session / "bag" / "metadata.yaml"
    inputs = {
        "msa_robot": file_record(robot_path),
        "msa_hesai": file_record(hesai_path),
        "aligned_samples": file_record(aligned_path),
    }
    if calibration_zip.is_file():
        inputs["msa_calibration_archive"] = file_record(calibration_zip)
    if metadata_path.is_file():
        inputs["raw_bag_metadata"] = file_record(metadata_path)
    manifest = {
        "manifest_version": 1,
        "session": session.name,
        "terrain": "garage",
        "calibration_recorded": "v1",
        "ground_truth_operational": "MSA offline batch",
        "raw_bag_read_by_this_builder": False,
        "inputs": inputs,
        "geometry": {
            "track_contact_center_in_base_link_m": TRACK_IN_BASE_M.tolist(),
            "l_front_m": L_FRONT_M,
            "l_rear_m": L_REAR_M,
            "wheelbase_equivalent_m": L_FRONT_M + L_REAR_M,
            "track_ground_contact_length_m": TRACK_CONTACT_LENGTH_M,
            "track_developed_loop_nominal_m": TRACK_LOOP_LENGTH_NOMINAL_M,
            "articulation_limit_physical_deg": ARTICULATION_LIMIT_PHYSICAL_DEG,
            "articulation_limit_recorded_legacy_deg": ARTICULATION_LIMIT_RECORDED_DEG,
        },
        "transforms": {
            "T_robot_MSA_hesai_MSA_estimated": transform_record(t_robot_hesai),
            "T_track_contact_center_hesai_v1": transform_record(t_track_hesai),
            "T_track_contact_center_robot_MSA_inferred": transform_record(
                t_track_robot
            ),
        },
        "filter": filter_info,
        "split": {
            "column": "split_contiguous_v1",
            "definition": "first 50% train, next 25% validation, final 25% test in time",
            "window_rule": "start and end must have the same split; no rollout may cross a split boundary",
            "status": "exploratory within one session, not an independent-session split",
        },
        "outputs": {
            "msa_track_100hz": file_record(reference_path),
            "canonical_fused_50hz": file_record(fused_path),
            "quality_summary": str((out_dir / "quality_summary.yaml").resolve()),
            "schema": str((out_dir / "schema.yaml").resolve()),
        },
    }
    write_yaml(out_dir / "manifest.yaml", manifest)

    print(
        yaml.safe_dump(
            {
                "status": "ok",
                "output_dir": str(out_dir),
                "reference_rows": len(reference),
                "fused_rows": len(fused),
                "path_length_m": quality_summary["path_length_m"],
                "hardware_articulation_available": False,
            },
            sort_keys=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
