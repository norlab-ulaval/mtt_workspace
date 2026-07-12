#!/usr/bin/env python3
"""Evaluate command-only M0-M4 motion-model progression against ICP and odom."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from lib.mtt_motion_research import (
    assign_segments,
    clamp,
    derive_research_rows,
    find_motion_csv,
    finite_stats,
    nominal_curvature,
    parse_bool,
    parse_float,
    percentile,
    read_csv_rows,
    resolve_sessions,
    rmse,
    se2_log_error,
    solve_least_squares,
    wrap_angle,
    write_csv_rows,
)


@dataclass
class CommandModel:
    name: str
    level: str
    description: str
    tau_v_s: float = 0.0
    tau_phi_s: float = 0.0
    speed_scale: float = 1.0
    yaw_gain: float = 1.0
    phi_scale: float = 1.0
    phi_bias_rad: float = 0.0
    speed_deadband_ms: float = 0.0
    phi_deadband_rad: float = 0.0
    speed_residual_features: list[str] = field(default_factory=list)
    speed_residual_coefficients: list[float] = field(default_factory=list)
    kappa_residual_features: list[str] = field(default_factory=list)
    kappa_residual_coefficients: list[float] = field(default_factory=list)


def dataset_csv_for_session(session: Path) -> Path:
    candidates = [
        session / "motion_research" / "dataset.csv",
        session / "motion_model_validation" / "motion_research" / "datasets" / session.name / "dataset.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return find_motion_csv(session)


def load_rows(sessions: list[Path], force_rebuild: bool) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for session in sessions:
        csv_path = dataset_csv_for_session(session)
        session_rows = read_csv_rows(csv_path)
        if force_rebuild or "research_quality_ok" not in session_rows[0]:
            session_rows = assign_segments(derive_research_rows(read_csv_rows(find_motion_csv(session)), session_name=session.name))
        rows.extend(session_rows)
    rows.sort(key=lambda row: (str(row.get("session", "")), float(row.get("t", 0.0))))
    augment_command_and_terrain_proxies(rows)
    return rows


def median(values: list[float]) -> float:
    vals = sorted(v for v in values if math.isfinite(v))
    if not vals:
        return 0.0
    mid = len(vals) // 2
    return vals[mid] if len(vals) % 2 else 0.5 * (vals[mid - 1] + vals[mid])


def augment_command_and_terrain_proxies(rows: list[dict[str, Any]]) -> None:
    by_session: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        by_session.setdefault(str(row.get("session", "")), []).append(row)
    for session_rows in by_session.values():
        session_rows.sort(key=lambda row: float(row.get("t", 0.0)))
        roll_med = median([parse_float(row.get("imu_roll_rad")) or math.nan for row in session_rows])
        pitch_med = median([parse_float(row.get("imu_pitch_rad")) or math.nan for row in session_rows])
        acc_med = median([parse_float(row.get("imu_acc_norm_ms2")) or math.nan for row in session_rows])
        z_med = median([parse_float(row.get("icp_z")) or math.nan for row in session_rows])
        prev: dict[str, Any] | None = None
        for row in session_rows:
            row["imu_roll_centered_rad"] = wrap_angle((parse_float(row.get("imu_roll_rad")) or roll_med) - roll_med)
            row["imu_pitch_centered_rad"] = wrap_angle((parse_float(row.get("imu_pitch_rad")) or pitch_med) - pitch_med)
            row["imu_acc_dynamic_ms2"] = (parse_float(row.get("imu_acc_norm_ms2")) or acc_med) - acc_med
            row["icp_z_centered_m"] = (parse_float(row.get("icp_z")) or z_med) - z_med
            if prev is None:
                row["cmd_accel_ms2"] = 0.0
                row["cmd_phi_rate_rad_s"] = 0.0
                row["icp_z_rate_m_s"] = 0.0
            else:
                dt = max(0.0, float(row["t"]) - float(prev["t"]))
                cmd_v = parse_float(row.get("cmd_speed_ms"))
                prev_cmd_v = parse_float(prev.get("cmd_speed_ms"))
                cmd_phi = parse_float(row.get("cmd_phi_rad"))
                prev_cmd_phi = parse_float(prev.get("cmd_phi_rad"))
                z = parse_float(row.get("icp_z_centered_m"))
                prev_z = parse_float(prev.get("icp_z_centered_m"))
                row["cmd_accel_ms2"] = (cmd_v - prev_cmd_v) / dt if dt > 1e-6 and cmd_v is not None and prev_cmd_v is not None else 0.0
                row["cmd_phi_rate_rad_s"] = (cmd_phi - prev_cmd_phi) / dt if dt > 1e-6 and cmd_phi is not None and prev_cmd_phi is not None else 0.0
                row["icp_z_rate_m_s"] = (z - prev_z) / dt if dt > 1e-6 and z is not None and prev_z is not None else 0.0
            prev = row


def split_folds(rows: list[dict[str, Any]], k: int, mode: str) -> list[tuple[list[dict[str, Any]], list[dict[str, Any]]]]:
    if mode == "session":
        keys = sorted({str(row.get("session", "")) for row in rows})
        if not keys:
            return [(rows, rows)]
        k = max(2, min(k, len(keys)))
        fold_keys = [set(keys[i::k]) for i in range(k)]
        return [
            (
                [row for row in rows if str(row.get("session", "")) not in keys_i],
                [row for row in rows if str(row.get("session", "")) in keys_i],
            )
            for keys_i in fold_keys
        ]

    keys = sorted({(str(row.get("session", "")), str(row.get("segment_id", ""))) for row in rows})
    if not keys:
        return [(rows, rows)]
    k = max(2, min(k, len(keys)))
    fold_keys = [set(keys[i::k]) for i in range(k)]
    return [
        (
            [row for row in rows if (str(row.get("session", "")), str(row.get("segment_id", ""))) not in keys_i],
            [row for row in rows if (str(row.get("session", "")), str(row.get("segment_id", ""))) in keys_i],
        )
        for keys_i in fold_keys
    ]


def ref_pose(row: dict[str, Any], reference: str) -> tuple[float, float, float] | None:
    if reference == "icp":
        keys = ("icp_x", "icp_y", "icp_yaw")
    elif reference == "odom":
        keys = ("odom_x", "odom_y", "odom_yaw")
    else:
        raise ValueError(reference)
    vals = [parse_float(row.get(key)) for key in keys]
    if any(v is None for v in vals):
        return None
    assert vals[0] is not None and vals[1] is not None and vals[2] is not None
    return vals[0], vals[1], vals[2]


def ref_speed(row: dict[str, Any], reference: str) -> float | None:
    return parse_float(row.get(f"{reference}_speed_ms"))


def ref_yaw_rate(row: dict[str, Any], reference: str) -> float | None:
    return parse_float(row.get(f"{reference}_yaw_rate_rad_s_derived"))


def command_raw(row: dict[str, Any]) -> tuple[float, float]:
    v = parse_float(row.get("cmd_speed_ms"))
    phi = parse_float(row.get("cmd_phi_rad"))
    return (v if v is not None else 0.0), (phi if phi is not None else 0.0)


def command_feature_vector(state: dict[str, float], features: list[str]) -> list[float]:
    v = state["v"]
    phi = state["phi"]
    vdot = state["vdot"]
    phidot = state["phidot"]
    out = []
    for feature in features:
        if feature == "bias":
            out.append(1.0)
        elif feature == "v":
            out.append(v)
        elif feature == "abs_v":
            out.append(abs(v))
        elif feature == "phi":
            out.append(phi)
        elif feature == "abs_phi":
            out.append(abs(phi))
        elif feature == "vdot":
            out.append(vdot)
        elif feature == "abs_vdot":
            out.append(abs(vdot))
        elif feature == "phidot":
            out.append(phidot)
        elif feature == "abs_phidot":
            out.append(abs(phidot))
        elif feature == "v_phi":
            out.append(v * phi)
        elif feature == "v_abs_phi":
            out.append(v * abs(phi))
        elif feature == "phi2":
            out.append(phi * phi)
        else:
            out.append(0.0)
    return out


def update_command_state(
    row: dict[str, Any],
    prev_t: float,
    state: dict[str, float],
    model: CommandModel,
) -> tuple[float, dict[str, float]]:
    t = float(row["t"])
    dt = max(0.0, min(t - prev_t, 0.5))
    raw_v, raw_phi = command_raw(row)
    prev_v = state["v"]
    prev_phi = state["phi"]
    if model.tau_v_s > 1e-6:
        state["v"] += (raw_v - state["v"]) * clamp(dt / model.tau_v_s, 0.0, 1.0)
    else:
        state["v"] = raw_v
    if model.tau_phi_s > 1e-6:
        state["phi"] += (raw_phi - state["phi"]) * clamp(dt / model.tau_phi_s, 0.0, 1.0)
    else:
        state["phi"] = raw_phi
    state["vdot"] = (state["v"] - prev_v) / dt if dt > 1e-6 else 0.0
    state["phidot"] = (state["phi"] - prev_phi) / dt if dt > 1e-6 else 0.0
    return dt, state


def command_speed_and_kappa(state: dict[str, float], model: CommandModel) -> tuple[float, float]:
    speed = state["v"]
    phi = state["phi"]
    if abs(speed) < model.speed_deadband_ms:
        speed = 0.0
    if abs(phi) < model.phi_deadband_rad:
        phi = 0.0
    phi_eff = model.phi_scale * phi + model.phi_bias_rad
    speed = model.speed_scale * speed
    if model.speed_residual_coefficients and model.speed_residual_features:
        speed += sum(c * f for c, f in zip(model.speed_residual_coefficients, command_feature_vector(state, model.speed_residual_features)))
    kappa = model.yaw_gain * nominal_curvature(phi_eff, "exact")
    if model.kappa_residual_coefficients and model.kappa_residual_features:
        kappa += sum(c * f for c, f in zip(model.kappa_residual_coefficients, command_feature_vector(state, model.kappa_residual_features)))
    return speed, kappa


def initial_command_state(row: dict[str, Any]) -> dict[str, float]:
    v, phi = command_raw(row)
    return {"v": v, "phi": phi, "vdot": 0.0, "phidot": 0.0}


def simulate_command_window(
    rows: list[dict[str, Any]],
    start_idx: int,
    end_idx: int,
    model: CommandModel,
    reference: str,
) -> tuple[float, float, float] | None:
    start_pose = ref_pose(rows[start_idx], reference)
    if start_pose is None:
        return None
    x, y, yaw = start_pose
    state = initial_command_state(rows[start_idx])
    prev_t = float(rows[start_idx]["t"])
    for row in rows[start_idx + 1 : end_idx + 1]:
        dt, state = update_command_state(row, prev_t, state, model)
        prev_t = float(row["t"])
        if dt <= 0.0:
            continue
        speed, kappa = command_speed_and_kappa(state, model)
        yaw_rate = speed * kappa if abs(speed) >= 0.05 else 0.0
        dyaw = yaw_rate * dt
        heading_mid = yaw + 0.5 * dyaw
        x += speed * math.cos(heading_mid) * dt
        y += speed * math.sin(heading_mid) * dt
        yaw = wrap_angle(yaw + dyaw)
    return x, y, yaw


def score_pointwise(rows: list[dict[str, Any]], model: CommandModel, reference: str) -> float:
    speed_errors = []
    yaw_errors = []
    state_by_session: dict[str, dict[str, float]] = {}
    prev_t_by_session: dict[str, float] = {}
    for row in rows:
        if not parse_bool(row.get("research_quality_ok")):
            continue
        session = str(row.get("session", ""))
        if session not in state_by_session:
            state_by_session[session] = initial_command_state(row)
            prev_t_by_session[session] = float(row["t"])
            continue
        _, state = update_command_state(row, prev_t_by_session[session], state_by_session[session], model)
        prev_t_by_session[session] = float(row["t"])
        speed_pred, kappa_pred = command_speed_and_kappa(state, model)
        speed_ref = ref_speed(row, reference)
        yaw_ref = ref_yaw_rate(row, reference)
        if speed_ref is not None:
            speed_errors.append(speed_pred - speed_ref)
        if yaw_ref is not None:
            yaw_errors.append(speed_pred * kappa_pred - yaw_ref)
    speed_rmse = rmse(speed_errors) or 0.0
    yaw_rmse = rmse(yaw_errors) or 0.0
    return speed_rmse + 3.0 * yaw_rmse


def train_residuals(rows: list[dict[str, Any]], model: CommandModel, reference: str, *, fit_speed: bool, fit_kappa: bool) -> CommandModel:
    speed_features = ["bias", "v", "abs_v", "vdot", "abs_vdot", "v_abs_phi"]
    kappa_features = ["bias", "phi", "abs_phi", "phidot", "abs_phidot", "v_phi", "phi2"]
    state_by_session: dict[str, dict[str, float]] = {}
    prev_t_by_session: dict[str, float] = {}
    speed_x: list[list[float]] = []
    speed_y: list[float] = []
    kappa_x: list[list[float]] = []
    kappa_y: list[float] = []
    for row in rows:
        if not parse_bool(row.get("research_quality_ok")):
            continue
        session = str(row.get("session", ""))
        if session not in state_by_session:
            state_by_session[session] = initial_command_state(row)
            prev_t_by_session[session] = float(row["t"])
            continue
        _, state = update_command_state(row, prev_t_by_session[session], state_by_session[session], model)
        prev_t_by_session[session] = float(row["t"])
        base_speed, base_kappa = command_speed_and_kappa(state, model)
        speed_reference = ref_speed(row, reference)
        yaw_reference = ref_yaw_rate(row, reference)
        if fit_speed and speed_reference is not None:
            speed_x.append(command_feature_vector(state, speed_features))
            speed_y.append(speed_reference - base_speed)
        speed_for_kappa = speed_reference if speed_reference is not None else base_speed
        if fit_kappa and yaw_reference is not None and speed_for_kappa is not None and abs(speed_for_kappa) > 0.15:
            kappa_x.append(command_feature_vector(state, kappa_features))
            kappa_y.append(yaw_reference / speed_for_kappa - base_kappa)
    trained = CommandModel(**model.__dict__)
    if fit_speed:
        trained.speed_residual_features = speed_features
        trained.speed_residual_coefficients = solve_least_squares(speed_x, speed_y, ridge=1e-4)
    if fit_kappa:
        trained.kappa_residual_features = kappa_features
        trained.kappa_residual_coefficients = solve_least_squares(kappa_x, kappa_y, ridge=1e-4)
    return trained


def tune_m1(train_rows: list[dict[str, Any]], reference: str, tau_grid: list[float]) -> CommandModel:
    best: tuple[float, CommandModel] | None = None
    for tau_v in tau_grid:
        for tau_phi in tau_grid:
            model = CommandModel(
                "M1_command_lag",
                "M1",
                "command speed/articulation with first-order actuator lag",
                tau_v_s=tau_v,
                tau_phi_s=tau_phi,
            )
            score = score_pointwise(train_rows, model, reference)
            if best is None or score < best[0]:
                best = (score, model)
    assert best is not None
    return best[1]


def tune_m2(train_rows: list[dict[str, Any]], reference: str, base: CommandModel) -> CommandModel:
    best: tuple[float, CommandModel] | None = None
    for speed_scale in (0.75, 0.9, 1.0, 1.1, 1.25):
        for yaw_gain in (0.5, 0.75, 1.0, 1.25, 1.5):
            for phi_scale in (0.6, 0.8, 1.0, 1.2, 1.4):
                model = CommandModel(
                    "M2_command_calibrated",
                    "M2",
                    "M1 plus command-to-state scale and curvature gain calibration",
                    tau_v_s=base.tau_v_s,
                    tau_phi_s=base.tau_phi_s,
                    speed_scale=speed_scale,
                    yaw_gain=yaw_gain,
                    phi_scale=phi_scale,
                )
                score = score_pointwise(train_rows, model, reference)
                if best is None or score < best[0]:
                    best = (score, model)
    assert best is not None
    return best[1]


def train_progression(
    train_rows: list[dict[str, Any]],
    reference: str,
    tau_grid: list[float],
    tuning_stride: int = 1,
) -> list[CommandModel]:
    tuning_rows = train_rows[:: max(1, tuning_stride)]
    m0 = CommandModel("M0_command_exact", "M0", "raw command speed/articulation with exact 2D curvature")
    m1 = tune_m1(tuning_rows, reference, tau_grid)
    m2 = tune_m2(tuning_rows, reference, m1)
    m3 = train_residuals(
        train_rows,
        CommandModel(
            "M3_command_curvature_residual",
            "M3",
            "M2 plus command-only curvature residual",
            tau_v_s=m2.tau_v_s,
            tau_phi_s=m2.tau_phi_s,
            speed_scale=m2.speed_scale,
            yaw_gain=m2.yaw_gain,
            phi_scale=m2.phi_scale,
        ),
        reference,
        fit_speed=False,
        fit_kappa=True,
    )
    m4 = train_residuals(
        train_rows,
        CommandModel(
            "M4_command_twist_residual",
            "M4",
            "M2 plus command-only speed and curvature residuals",
            tau_v_s=m2.tau_v_s,
            tau_phi_s=m2.tau_phi_s,
            speed_scale=m2.speed_scale,
            yaw_gain=m2.yaw_gain,
            phi_scale=m2.phi_scale,
        ),
        reference,
        fit_speed=True,
        fit_kappa=True,
    )
    return [m0, m1, m2, m3, m4]


def prediction_errors_reference(
    rows: list[dict[str, Any]],
    model: CommandModel,
    reference: str,
    horizons_s: list[float],
    sample_period_s: float,
    include_weak: bool,
) -> list[dict[str, Any]]:
    eval_rows = list(rows) if include_weak else [row for row in rows if parse_bool(row.get("research_quality_ok"))]
    eval_rows = [row for row in eval_rows if ref_pose(row, reference) is not None and parse_float(row.get("cmd_speed_ms")) is not None]
    times = [float(row["t"]) for row in eval_rows]
    out: list[dict[str, Any]] = []
    next_start_t = -math.inf
    for start_idx, start in enumerate(eval_rows):
        start_t = float(start["t"])
        if start_t < next_start_t:
            continue
        next_start_t = start_t + sample_period_s
        for horizon in horizons_s:
            target_t = start_t + horizon
            end_idx = min(range(start_idx, len(eval_rows)), key=lambda idx: abs(times[idx] - target_t))
            if end_idx <= start_idx or abs(times[end_idx] - target_t) > 0.25:
                continue
            window = eval_rows[start_idx : end_idx + 1]
            if not include_weak and not all(parse_bool(row.get("research_quality_ok")) for row in window):
                continue
            pred = simulate_command_window(eval_rows, start_idx, end_idx, model, reference)
            end_pose = ref_pose(eval_rows[end_idx], reference)
            if pred is None or end_pose is None:
                continue
            ex, ey, eyaw = se2_log_error(pred[0], pred[1], pred[2], end_pose[0], end_pose[1], end_pose[2])
            out.append(
                {
                    "model": model.name,
                    "level": model.level,
                    "reference": reference,
                    "session": start.get("session", ""),
                    "segment_id": start.get("segment_id", ""),
                    "motion_class": start.get("motion_class", ""),
                    "quality_class": start.get("quality_class", ""),
                    "start_t": start_t,
                    "end_t": float(eval_rows[end_idx]["t"]),
                    "horizon_s": horizon,
                    "position_error_m": math.hypot(ex, ey),
                    "longitudinal_error_m": ex,
                    "lateral_error_m": ey,
                    "yaw_error_rad": abs(eyaw),
                    "max_abs_phi_cmd_rad": max(abs(parse_float(row.get("cmd_phi_rad")) or 0.0) for row in window),
                    "max_abs_roll_centered_rad": max(abs(parse_float(row.get("imu_roll_centered_rad")) or 0.0) for row in window),
                    "max_abs_pitch_centered_rad": max(abs(parse_float(row.get("imu_pitch_centered_rad")) or 0.0) for row in window),
                    "max_abs_imu_acc_dynamic_ms2": max(abs(parse_float(row.get("imu_acc_dynamic_ms2")) or 0.0) for row in window),
                    "max_abs_cmd_accel_ms2": max(abs(parse_float(row.get("cmd_accel_ms2")) or 0.0) for row in window),
                    "max_abs_cmd_phi_rate_rad_s": max(abs(parse_float(row.get("cmd_phi_rate_rad_s")) or 0.0) for row in window),
                    "max_abs_icp_z_centered_m": max(abs(parse_float(row.get("icp_z_centered_m")) or 0.0) for row in window),
                    "max_abs_icp_z_rate_m_s": max(abs(parse_float(row.get("icp_z_rate_m_s")) or 0.0) for row in window),
                }
            )
    return out


def summarize_errors(errors: list[dict[str, Any]], horizon_for_gain: float) -> dict[str, Any]:
    summary: dict[str, Any] = {"rows": len(errors), "by_reference_model_horizon": {}, "gain_at_horizon": {}}
    for reference in sorted({str(row["reference"]) for row in errors}):
        summary["by_reference_model_horizon"][reference] = {}
        ref_rows = [row for row in errors if row["reference"] == reference]
        for model in sorted({str(row["model"]) for row in ref_rows}):
            summary["by_reference_model_horizon"][reference][model] = {}
            model_rows = [row for row in ref_rows if row["model"] == model]
            for horizon in sorted({float(row["horizon_s"]) for row in model_rows}):
                rows_h = [row for row in model_rows if abs(float(row["horizon_s"]) - horizon) < 1e-9]
                summary["by_reference_model_horizon"][reference][model][f"{horizon:g}s"] = {
                    "samples": len(rows_h),
                    "position_m": finite_stats(float(row["position_error_m"]) for row in rows_h),
                    "yaw_rad": finite_stats(float(row["yaw_error_rad"]) for row in rows_h),
                }
        rows_gain = [row for row in ref_rows if abs(float(row["horizon_s"]) - horizon_for_gain) < 1e-9]
        ordered_models = ["M0_command_exact", "M1_command_lag", "M2_command_calibrated", "M3_command_curvature_residual", "M4_command_twist_residual"]
        previous_rmse = None
        gain_rows = []
        for model in ordered_models:
            vals = [float(row["position_error_m"]) for row in rows_gain if row["model"] == model]
            model_rmse = rmse(vals)
            if model_rmse is None:
                continue
            gain_rows.append(
                {
                    "model": model,
                    "rmse_m": model_rmse,
                    "gain_vs_previous_pct": (100.0 * (previous_rmse - model_rmse) / previous_rmse) if previous_rmse else None,
                }
            )
            previous_rmse = model_rmse
        summary["gain_at_horizon"][reference] = gain_rows
    return summary


def write_plots(out_dir: Path, errors: list[dict[str, Any]], horizon_for_gain: float) -> dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}
    outputs: dict[str, str] = {}
    models = ["M0_command_exact", "M1_command_lag", "M2_command_calibrated", "M3_command_curvature_residual", "M4_command_twist_residual"]
    labels = ["M0\ncmd", "M1\nlag", "M2\ncalib", "M3\ncurv", "M4\ntwist"]
    horizons = sorted({float(row["horizon_s"]) for row in errors})

    def save(name: str) -> None:
        path = out_dir / name
        plt.tight_layout()
        plt.savefig(path, dpi=170)
        plt.close()
        outputs[name] = str(path)

    for reference in sorted({str(row["reference"]) for row in errors}):
        rows_ref = [row for row in errors if row["reference"] == reference]
        plt.figure(figsize=(12, 6))
        for model in models:
            ys = []
            for horizon in horizons:
                vals = [float(row["position_error_m"]) for row in rows_ref if row["model"] == model and abs(float(row["horizon_s"]) - horizon) < 1e-9]
                ys.append(rmse(vals) or math.nan)
            plt.plot(horizons, ys, marker="o", label=model)
        plt.xlabel("prediction horizon [s]")
        plt.ylabel("position RMSE [m]")
        plt.title(f"Command-only progression vs {reference.upper()}")
        plt.grid(True, alpha=0.35)
        plt.legend(fontsize=8)
        save(f"command_progression_rmse_vs_{reference}.png")

        plt.figure(figsize=(12, 6))
        for model in models:
            q1 = []
            med = []
            q3 = []
            for horizon in horizons:
                vals = [float(row["position_error_m"]) for row in rows_ref if row["model"] == model and abs(float(row["horizon_s"]) - horizon) < 1e-9]
                q1.append(percentile(vals, 25.0) or math.nan)
                med.append(percentile(vals, 50.0) or math.nan)
                q3.append(percentile(vals, 75.0) or math.nan)
            line = plt.plot(horizons, med, marker="o", label=model)[0]
            plt.fill_between(horizons, q1, q3, color=line.get_color(), alpha=0.14)
        plt.xlabel("prediction horizon [s]")
        plt.ylabel("position error [m]")
        plt.title(f"Median + IQR command-only error vs {reference.upper()}")
        plt.grid(True, alpha=0.35)
        plt.legend(fontsize=8)
        save(f"command_progression_median_iqr_vs_{reference}.png")

        rows_h = [row for row in rows_ref if abs(float(row["horizon_s"]) - horizon_for_gain) < 1e-9]
        data = [[float(row["position_error_m"]) for row in rows_h if row["model"] == model] or [0.0] for model in models]
        plt.figure(figsize=(11, 6))
        plt.boxplot(data, labels=labels, showmeans=True, showfliers=False)
        plt.yscale("symlog", linthresh=0.1)
        plt.ylabel(f"{horizon_for_gain:g}s position error [m]")
        plt.title(f"Command-only error distribution vs {reference.upper()}")
        plt.grid(True, axis="y", alpha=0.35)
        save(f"command_progression_h{horizon_for_gain:g}_boxplot_vs_{reference}.png")

        plt.figure(figsize=(11, 6))
        parts = plt.violinplot(data, showmedians=True, showextrema=False)
        for body in parts["bodies"]:
            body.set_alpha(0.55)
        plt.xticks(range(1, len(labels) + 1), labels)
        plt.yscale("symlog", linthresh=0.1)
        plt.ylabel(f"{horizon_for_gain:g}s position error [m]")
        plt.title(f"Command-only error density vs {reference.upper()}")
        plt.grid(True, axis="y", alpha=0.35)
        save(f"command_progression_h{horizon_for_gain:g}_violin_vs_{reference}.png")

        rmses = [rmse(vals) or math.nan for vals in data]
        gains = [0.0]
        for prev, curr in zip(rmses, rmses[1:]):
            gains.append(100.0 * (prev - curr) / prev if math.isfinite(prev) and prev > 1e-9 else 0.0)
        plt.figure(figsize=(11, 5))
        plt.bar(labels, gains)
        plt.axhline(0.0, color="black", linewidth=1.0)
        plt.ylabel("gain vs previous model [% RMSE]")
        plt.title(f"Incremental complexity gain at {horizon_for_gain:g}s vs {reference.upper()}")
        plt.grid(True, axis="y", alpha=0.35)
        save(f"command_progression_h{horizon_for_gain:g}_gain_vs_{reference}.png")

        best_model = "M4_command_twist_residual"
        rows_best = [row for row in rows_h if row["model"] == best_model]
        classes = sorted({str(row.get("motion_class", "")) for row in rows_best})
        if classes:
            class_data = [[float(row["position_error_m"]) for row in rows_best if row.get("motion_class") == klass] or [0.0] for klass in classes]
            plt.figure(figsize=(12, 6))
            plt.boxplot(class_data, labels=classes, showmeans=True, showfliers=False)
            plt.yscale("symlog", linthresh=0.1)
            plt.ylabel(f"{horizon_for_gain:g}s position error [m]")
            plt.title(f"Best command model failures by motion class vs {reference.upper()}")
            plt.grid(True, axis="y", alpha=0.35)
            plt.xticks(rotation=30, ha="right")
            save(f"command_m4_h{horizon_for_gain:g}_by_motion_class_vs_{reference}.png")

        for key, xlabel in (
            ("max_abs_phi_cmd_rad", "max |command articulation| [rad]"),
            ("max_abs_roll_centered_rad", "max |roll - median roll| [rad]"),
            ("max_abs_pitch_centered_rad", "max |pitch - median pitch| [rad]"),
            ("max_abs_imu_acc_dynamic_ms2", "max |IMU acceleration norm - median| [m/s2]"),
            ("max_abs_cmd_accel_ms2", "max |command acceleration| [m/s2]"),
            ("max_abs_cmd_phi_rate_rad_s", "max |command articulation rate| [rad/s]"),
            ("max_abs_icp_z_centered_m", "max |ICP z - median z| [m]"),
            ("max_abs_icp_z_rate_m_s", "max |ICP z rate| [m/s]"),
        ):
            if not rows_best:
                continue
            plt.figure(figsize=(8, 5))
            plt.scatter([float(row.get(key, 0.0)) for row in rows_best], [float(row["position_error_m"]) for row in rows_best], s=10, alpha=0.45)
            plt.xlabel(xlabel)
            plt.ylabel(f"{horizon_for_gain:g}s position error [m]")
            plt.title(f"M4 failures vs {key} ({reference.upper()})")
            plt.grid(True, alpha=0.35)
            save(f"command_m4_h{horizon_for_gain:g}_error_vs_{key}_{reference}.png")

    return outputs


def top_failure_rows(errors: list[dict[str, Any]], horizon_s: float, reference: str, model: str, limit: int = 80) -> list[dict[str, Any]]:
    rows = [
        row
        for row in errors
        if row["reference"] == reference and row["model"] == model and abs(float(row["horizon_s"]) - horizon_s) < 1e-9
    ]
    rows.sort(key=lambda row: float(row["position_error_m"]), reverse=True)
    return rows[:limit]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", type=Path, nargs="+")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--fit-reference", choices=["icp", "odom"], default="icp")
    parser.add_argument("--horizons", default="0.5,1,2,3,5,10")
    parser.add_argument("--sample-period-s", type=float, default=0.5)
    parser.add_argument("--k", type=int, default=5)
    parser.add_argument("--fold-mode", choices=["segment", "session"], default="segment")
    parser.add_argument("--horizon-for-gain", type=float, default=3.0)
    parser.add_argument("--tau-grid", default="0,0.2,0.5,0.8,1.2")
    parser.add_argument("--tuning-stride", type=int, default=1, help="Subsample rows only for parameter search; final fit/eval still use full data.")
    parser.add_argument("--include-weak", action="store_true")
    parser.add_argument("--force-rebuild", action="store_true")
    return parser.parse_args()


def resolve_all_sessions(paths: list[Path]) -> list[Path]:
    sessions: list[Path] = []
    seen: set[Path] = set()
    for path in paths:
        for session in resolve_sessions(path):
            resolved = session.resolve()
            if resolved not in seen:
                sessions.append(resolved)
                seen.add(resolved)
    return sessions


def main() -> int:
    args = parse_args()
    sessions = resolve_all_sessions(args.paths)
    rows_all = load_rows(sessions, args.force_rebuild)
    rows_trainable = [row for row in rows_all if parse_bool(row.get("research_quality_ok"))]
    tau_grid = [float(x) for x in args.tau_grid.split(",") if x.strip()]
    horizons = [float(x) for x in args.horizons.split(",") if x.strip()]
    out_dir = args.output_dir or (sessions[0] / "motion_research" / "command_progression" if len(sessions) == 1 else Path("artifacts/results/motion_research/command_progression"))
    out_dir.mkdir(parents=True, exist_ok=True)

    full_models = train_progression(rows_trainable, args.fit_reference, tau_grid, args.tuning_stride)
    all_errors: list[dict[str, Any]] = []
    for reference in ("icp", "odom"):
        for model in full_models:
            all_errors.extend(
                prediction_errors_reference(
                    rows_all,
                    model,
                    reference,
                    horizons,
                    args.sample_period_s,
                    args.include_weak,
                )
            )
    write_csv_rows(out_dir / "command_prediction_errors.csv", all_errors)

    kfold_rows = []
    for fold_idx, (train, test) in enumerate(split_folds(rows_trainable, args.k, args.fold_mode)):
        models = train_progression(train, args.fit_reference, tau_grid, args.tuning_stride)
        for model in models:
            errors = prediction_errors_reference(test, model, args.fit_reference, [args.horizon_for_gain], args.sample_period_s, False)
            vals = [float(row["position_error_m"]) for row in errors]
            kfold_rows.append(
                {
                    "fold": fold_idx,
                    "model": model.name,
                    "level": model.level,
                    "reference": args.fit_reference,
                    "horizon_s": args.horizon_for_gain,
                    "test_windows": len(vals),
                    "position_rmse_m": rmse(vals),
                    "position_median_m": percentile(vals, 50.0),
                    "position_p95_m": percentile(vals, 95.0),
                }
            )
    write_csv_rows(out_dir / "command_kfold_results.csv", kfold_rows)

    summary = {
        "sessions": [session.name for session in sessions],
        "rows": len(rows_all),
        "trainable_rows": len(rows_trainable),
        "fit_reference": args.fit_reference,
        "fold_mode": args.fold_mode,
        "tuning_stride": args.tuning_stride,
        "include_weak": bool(args.include_weak),
        "horizons_s": horizons,
        "command_only_contract": {
            "prediction_inputs": ["cmd_speed_ms", "cmd_phi_rad", "t"],
            "allowed_training_targets": ["icp pose/speed/yaw_rate", "odom pose/speed/yaw_rate"],
            "diagnostic_only_signals": ["imu", "roll/pitch", "icp_z", "motion_class", "quality_class"],
            "not_used_as_prediction_inputs": ["tachometer", "measured articulation", "trailer pose", "imu"],
        },
        "models": [model.__dict__ for model in full_models],
        "summary": summarize_errors(all_errors, args.horizon_for_gain),
        "outputs": {
            "prediction_errors_csv": str(out_dir / "command_prediction_errors.csv"),
            "kfold_csv": str(out_dir / "command_kfold_results.csv"),
        },
    }
    summary["outputs"].update(write_plots(out_dir, all_errors, args.horizon_for_gain))
    write_csv_rows(out_dir / "top_failures_m4_vs_icp.csv", top_failure_rows(all_errors, args.horizon_for_gain, "icp", "M4_command_twist_residual"))
    write_csv_rows(out_dir / "top_failures_m4_vs_odom.csv", top_failure_rows(all_errors, args.horizon_for_gain, "odom", "M4_command_twist_residual"))
    (out_dir / "command_progression_summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump({"output_dir": str(out_dir), "models": [model.name for model in full_models], "rows": len(all_errors)}, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
