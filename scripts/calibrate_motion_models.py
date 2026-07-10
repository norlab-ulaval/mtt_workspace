#!/usr/bin/env python3
"""Calibrate measured and command MTT motion models on local RPE windows."""

from __future__ import annotations

import argparse
import bisect
import csv
import itertools
import math
import statistics
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable

import yaml


L_F_M = 0.9
L_R_M = 1.5
MAX_ARTICULATION_RAD = math.radians(60.0)
MIN_TURN_SPEED_MS = 0.1
MOVING_MIN_DISTANCE_M = 0.5
MOVING_MIN_YAW_RAD = math.radians(5.0)


def parse_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return math.isfinite(float(value)) and abs(float(value)) > 0.5
    return str(value).strip().lower() in {"1", "true", "yes", "ok", "accepted"}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def nominal_curvature(phi_rad: float) -> float:
    phi = clamp(phi_rad, -MAX_ARTICULATION_RAD, MAX_ARTICULATION_RAD)
    return math.sin(phi) / (L_F_M * math.cos(phi) + L_R_M)


def rmse(values: Iterable[float]) -> float | None:
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return None
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def percentile(values: Iterable[float], pct: float) -> float | None:
    vals = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not vals:
        return None
    idx = int(round((len(vals) - 1) * pct / 100.0))
    return vals[max(0, min(len(vals) - 1, idx))]


def finite_stats(values: Iterable[float]) -> dict[str, float | int | None]:
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return {"count": 0, "min": None, "mean": None, "median": None, "p95": None, "max": None, "rmse": None}
    return {
        "count": len(vals),
        "min": min(vals),
        "mean": statistics.mean(vals),
        "median": statistics.median(vals),
        "p95": percentile(vals, 95.0),
        "max": max(vals),
        "rmse": rmse(vals),
    }


def tach_direction_sign(direction: Any) -> float:
    text = str(direction or "").strip().lower()
    return -1.0 if text in {"reverse", "backward", "rev", "-1"} else 1.0


def se2_error(pred: tuple[float, float, float], ref: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = pred[0] - ref[0]
    dy = pred[1] - ref[1]
    c = math.cos(ref[2])
    s = math.sin(ref[2])
    longitudinal = c * dx + s * dy
    lateral = -s * dx + c * dy
    return longitudinal, lateral, wrap_angle(pred[2] - ref[2])


def compose_delta(
    start_ref: tuple[float, float, float],
    start_delta_source: tuple[float, float, float],
    end_delta_source: tuple[float, float, float],
) -> tuple[float, float, float]:
    sx, sy, syaw = start_delta_source
    ex, ey, eyaw = end_delta_source
    c0 = math.cos(syaw)
    s0 = math.sin(syaw)
    dx_body = c0 * (ex - sx) + s0 * (ey - sy)
    dy_body = -s0 * (ex - sx) + c0 * (ey - sy)
    dyaw = wrap_angle(eyaw - syaw)
    cr = math.cos(start_ref[2])
    sr = math.sin(start_ref[2])
    return (
        start_ref[0] + cr * dx_body - sr * dy_body,
        start_ref[1] + sr * dx_body + cr * dy_body,
        wrap_angle(start_ref[2] + dyaw),
    )


@dataclass
class MotionModel:
    name: str
    family: str
    level: str
    description: str
    speed_source: str
    phi_source: str
    tau_v_s: float = 0.0
    tau_phi_s: float = 0.0
    speed_scale: float = 1.0
    yaw_gain: float = 1.0
    phi_scale: float = 1.0
    phi_bias_rad: float = 0.0
    phi_delay_s: float = 0.0
    speed_deadband_ms: float = 0.0
    phi_deadband_rad: float = 0.0
    speed_residual_features: list[str] = field(default_factory=list)
    speed_residual_coefficients: list[float] = field(default_factory=list)
    kappa_residual_features: list[str] = field(default_factory=list)
    kappa_residual_coefficients: list[float] = field(default_factory=list)


class Series:
    def __init__(self, rows: list[dict[str, Any]]):
        self.rows = rows
        self.times = [float(row["t"]) for row in rows]

    def nearest(self, t: float, tolerance_s: float) -> dict[str, Any] | None:
        if not self.rows:
            return None
        idx = bisect.bisect_left(self.times, t)
        candidates = []
        if idx < len(self.rows):
            candidates.append(self.rows[idx])
        if idx:
            candidates.append(self.rows[idx - 1])
        best = min(candidates, key=lambda row: abs(float(row["t"]) - t))
        return best if abs(float(best["t"]) - t) <= tolerance_s else None


def read_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for raw in csv.DictReader(stream):
            row: dict[str, Any] = dict(raw)
            for key, value in raw.items():
                parsed = parse_float(value)
                if parsed is not None:
                    row[key] = parsed
            if "tacho_signed_ms" not in row:
                speed = parse_float(row.get("tach_speed_ms"))
                row["tacho_signed_ms"] = speed * tach_direction_sign(row.get("tach_direction")) if speed is not None else None
            if "phi_rad" not in row:
                row["phi_rad"] = parse_float(row.get("mtt_articulation_angle")) or parse_float(row.get("trailer_articulation_angle"))
            rows.append(row)
    rows.sort(key=lambda row: (str(row.get("session", "")), float(row.get("t", 0.0))))
    return rows


def dataset_csv_for_session(session: Path) -> Path:
    candidates = [
        session / "motion_model_validation" / "motion_research" / "datasets" / session.name / "dataset.csv",
        session / "motion_research" / "dataset.csv",
        session / "postprocess_dataset" / "motion_model_dataset.csv",
        session / "motion_model_validation" / "aligned_samples.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"no motion research dataset found under {session}")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = sorted({key for row in rows for key in row.keys()})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def good_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in rows if parse_bool(row.get("research_quality_ok"))]


def split_folds(rows: list[dict[str, Any]], k: int, mode: str) -> list[tuple[list[dict[str, Any]], list[dict[str, Any]]]]:
    if mode == "session":
        keys = sorted({str(row.get("session", "")) for row in rows})
    else:
        keys = sorted({f"{row.get('session', '')}/{row.get('segment_id', '')}" for row in rows})
    if len(keys) < 2:
        return [(rows, rows)]
    k = max(2, min(k, len(keys)))
    fold_keys = [set(keys[i::k]) for i in range(k)]

    def key(row: dict[str, Any]) -> str:
        return str(row.get("session", "")) if mode == "session" else f"{row.get('session', '')}/{row.get('segment_id', '')}"

    return [
        ([row for row in rows if key(row) not in fold], [row for row in rows if key(row) in fold])
        for fold in fold_keys
    ]


def grouped_series(rows: list[dict[str, Any]]) -> dict[str, Series]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(str(row.get("session", "")), []).append(row)
    return {session: Series(sorted(session_rows, key=lambda row: float(row["t"]))) for session, session_rows in grouped.items()}


def grouped_segments(rows: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault((str(row.get("session", "")), str(row.get("segment_id", ""))), []).append(row)
    return [sorted(group, key=lambda row: float(row["t"])) for _, group in sorted(grouped.items())]


def pose(row: dict[str, Any], reference: str) -> tuple[float, float, float] | None:
    vals = [parse_float(row.get(f"{reference}_x")), parse_float(row.get(f"{reference}_y")), parse_float(row.get(f"{reference}_yaw"))]
    if any(value is None for value in vals):
        return None
    assert vals[0] is not None and vals[1] is not None and vals[2] is not None
    return vals[0], vals[1], vals[2]


def speed_ref(row: dict[str, Any], reference: str) -> float | None:
    return parse_float(row.get(f"{reference}_speed_ms"))


def yaw_rate_ref(row: dict[str, Any], reference: str) -> float | None:
    return parse_float(row.get(f"{reference}_yaw_rate_rad_s_derived")) or parse_float(row.get(f"{reference}_yaw_rate_rad_s"))


def source_speed(row: dict[str, Any], model: MotionModel) -> float:
    if model.speed_source == "tacho":
        return parse_float(row.get("tacho_signed_ms")) or 0.0
    if model.speed_source == "cmd":
        return parse_float(row.get("cmd_speed_ms")) or parse_float(row.get("cmd_linear_x")) or 0.0
    return 0.0


def source_phi(row: dict[str, Any], model: MotionModel) -> float:
    if model.phi_source == "measured":
        return parse_float(row.get("phi_rad")) or parse_float(row.get("mtt_articulation_angle")) or 0.0
    if model.phi_source == "cmd":
        return parse_float(row.get("cmd_phi_rad")) or 0.0
    return 0.0


def feature_vector(state: dict[str, float], features: list[str]) -> list[float]:
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


def initial_state(row: dict[str, Any], model: MotionModel, series: Series | None = None) -> dict[str, float]:
    phi_row = row
    if series is not None and abs(model.phi_delay_s) > 1e-9:
        phi_row = series.nearest(float(row["t"]) - model.phi_delay_s, 0.10) or row
    return {
        "v": source_speed(row, model),
        "phi": source_phi(phi_row, model),
        "vdot": 0.0,
        "phidot": 0.0,
    }


def update_state(
    row: dict[str, Any],
    prev_t: float,
    state: dict[str, float],
    model: MotionModel,
    series: Series | None,
) -> tuple[float, dict[str, float]]:
    t = float(row["t"])
    dt = max(0.0, min(t - prev_t, 0.5))
    phi_row = row
    if series is not None and abs(model.phi_delay_s) > 1e-9:
        phi_row = series.nearest(t - model.phi_delay_s, 0.10) or row
    raw_v = source_speed(row, model)
    raw_phi = source_phi(phi_row, model)
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


def speed_and_kappa(state: dict[str, float], model: MotionModel) -> tuple[float, float]:
    speed = model.speed_scale * state["v"]
    if abs(speed) < model.speed_deadband_ms:
        speed = 0.0
    phi = model.phi_scale * state["phi"] + model.phi_bias_rad
    if abs(phi) < model.phi_deadband_rad:
        phi = 0.0
    speed += sum(
        coeff * value
        for coeff, value in zip(model.speed_residual_coefficients, feature_vector(state, model.speed_residual_features))
    )
    kappa = model.yaw_gain * nominal_curvature(phi)
    kappa += sum(
        coeff * value
        for coeff, value in zip(model.kappa_residual_coefficients, feature_vector(state, model.kappa_residual_features))
    )
    return speed, kappa


def simulate_window(
    rows: list[dict[str, Any]],
    start_idx: int,
    end_idx: int,
    model: MotionModel,
    reference: str,
    series_by_session: dict[str, Series],
) -> tuple[float, float, float] | None:
    start_pose = pose(rows[start_idx], reference)
    if start_pose is None:
        return None
    x, y, yaw = start_pose
    session = str(rows[start_idx].get("session", ""))
    series = series_by_session.get(session)
    state = initial_state(rows[start_idx], model, series)
    prev_t = float(rows[start_idx]["t"])
    for row in rows[start_idx + 1 : end_idx + 1]:
        dt, state = update_state(row, prev_t, state, model, series)
        prev_t = float(row["t"])
        if dt <= 0.0:
            continue
        speed, kappa = speed_and_kappa(state, model)
        yaw_rate = speed * kappa if abs(speed) >= MIN_TURN_SPEED_MS else 0.0
        dyaw = yaw_rate * dt
        heading_mid = yaw + 0.5 * dyaw
        x += speed * math.cos(heading_mid) * dt
        y += speed * math.sin(heading_mid) * dt
        yaw = wrap_angle(yaw + dyaw)
    return x, y, yaw


def prediction_errors(
    rows: list[dict[str, Any]],
    model: MotionModel,
    reference: str,
    horizons_s: list[float],
    sample_period_s: float,
    require_quality: bool = True,
) -> list[dict[str, Any]]:
    if require_quality:
        eval_rows = good_rows(rows)
    else:
        eval_rows = list(rows)
    eval_rows = [row for row in eval_rows if pose(row, reference) is not None]
    eval_rows.sort(key=lambda row: (str(row.get("session", "")), str(row.get("segment_id", "")), float(row["t"])))
    series_by_session = grouped_series(eval_rows)
    out: list[dict[str, Any]] = []

    for group in grouped_segments(eval_rows):
        if len(group) < 2:
            continue
        times = [float(row["t"]) for row in group]
        next_start_t = -math.inf
        for start_idx, start in enumerate(group):
            start_t = float(start["t"])
            if start_t < next_start_t:
                continue
            next_start_t = start_t + sample_period_s
            for horizon in horizons_s:
                target_t = start_t + horizon
                idx = bisect.bisect_left(times, target_t)
                candidates = []
                if idx < len(group):
                    candidates.append(idx)
                if idx:
                    candidates.append(idx - 1)
                if not candidates:
                    continue
                end_idx = min(candidates, key=lambda i: abs(times[i] - target_t))
                if end_idx <= start_idx or abs(times[end_idx] - target_t) > 0.25:
                    continue
                pred = simulate_window(group, start_idx, end_idx, model, reference, series_by_session)
                start_pose = pose(group[start_idx], reference)
                end_pose = pose(group[end_idx], reference)
                if pred is None or start_pose is None or end_pose is None:
                    continue
                ex, ey, eyaw = se2_error(pred, end_pose)
                ref_distance = math.hypot(end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
                ref_yaw_delta = abs(wrap_angle(end_pose[2] - start_pose[2]))
                window = group[start_idx : end_idx + 1]
                out.append(
                    {
                        "model": model.name,
                        "family": model.family,
                        "level": model.level,
                        "reference": reference,
                        "session": start.get("session", ""),
                        "segment_id": start.get("segment_id", ""),
                        "motion_class": start.get("motion_class", ""),
                        "quality_class": start.get("quality_class", ""),
                        "start_t": start_t,
                        "end_t": float(group[end_idx]["t"]),
                        "horizon_s": horizon,
                        "longitudinal_error_m": ex,
                        "lateral_error_m": ey,
                        "position_error_m": math.hypot(ex, ey),
                        "yaw_error_rad": abs(eyaw),
                        "ref_distance_m": ref_distance,
                        "ref_yaw_delta_rad": ref_yaw_delta,
                        "moving_window": ref_distance >= MOVING_MIN_DISTANCE_M or ref_yaw_delta >= MOVING_MIN_YAW_RAD,
                        "max_abs_phi_rad": max(abs(parse_float(row.get("phi_rad")) or 0.0) for row in window),
                        "max_abs_cmd_phi_rad": max(abs(parse_float(row.get("cmd_phi_rad")) or 0.0) for row in window),
                        "max_abs_speed_ms": max(abs(parse_float(row.get("tacho_signed_ms")) or 0.0) for row in window),
                        "max_abs_cmd_speed_ms": max(abs(parse_float(row.get("cmd_speed_ms")) or 0.0) for row in window),
                        "max_abs_yaw_rate_ref_rad_s": max(abs(yaw_rate_ref(row, reference) or 0.0) for row in window),
                    }
                )
    return out


def odom_delta_errors(rows: list[dict[str, Any]], reference: str, horizons_s: list[float], sample_period_s: float) -> list[dict[str, Any]]:
    eval_rows = [row for row in good_rows(rows) if pose(row, reference) is not None and pose(row, "odom") is not None]
    eval_rows.sort(key=lambda row: (str(row.get("session", "")), str(row.get("segment_id", "")), float(row["t"])))
    out: list[dict[str, Any]] = []
    for group in grouped_segments(eval_rows):
        times = [float(row["t"]) for row in group]
        next_start_t = -math.inf
        for start_idx, start in enumerate(group):
            start_t = float(start["t"])
            if start_t < next_start_t:
                continue
            next_start_t = start_t + sample_period_s
            for horizon in horizons_s:
                target_t = start_t + horizon
                idx = bisect.bisect_left(times, target_t)
                candidates = []
                if idx < len(group):
                    candidates.append(idx)
                if idx:
                    candidates.append(idx - 1)
                if not candidates:
                    continue
                end_idx = min(candidates, key=lambda i: abs(times[i] - target_t))
                if end_idx <= start_idx or abs(times[end_idx] - target_t) > 0.25:
                    continue
                start_ref = pose(group[start_idx], reference)
                start_odom = pose(group[start_idx], "odom")
                end_odom = pose(group[end_idx], "odom")
                end_ref = pose(group[end_idx], reference)
                if start_ref is None or start_odom is None or end_odom is None or end_ref is None:
                    continue
                pred = compose_delta(start_ref, start_odom, end_odom)
                ex, ey, eyaw = se2_error(pred, end_ref)
                ref_distance = math.hypot(end_ref[0] - start_ref[0], end_ref[1] - start_ref[1])
                ref_yaw_delta = abs(wrap_angle(end_ref[2] - start_ref[2]))
                out.append(
                    {
                        "model": "odom_delta",
                        "family": "baseline",
                        "level": "baseline",
                        "reference": reference,
                        "session": start.get("session", ""),
                        "segment_id": start.get("segment_id", ""),
                        "motion_class": start.get("motion_class", ""),
                        "quality_class": start.get("quality_class", ""),
                        "start_t": start_t,
                        "end_t": float(group[end_idx]["t"]),
                        "horizon_s": horizon,
                        "longitudinal_error_m": ex,
                        "lateral_error_m": ey,
                        "position_error_m": math.hypot(ex, ey),
                        "yaw_error_rad": abs(eyaw),
                        "ref_distance_m": ref_distance,
                        "ref_yaw_delta_rad": ref_yaw_delta,
                        "moving_window": ref_distance >= MOVING_MIN_DISTANCE_M or ref_yaw_delta >= MOVING_MIN_YAW_RAD,
                    }
                )
    return out


def is_moving_error(row: dict[str, Any]) -> bool:
    return parse_bool(row.get("moving_window"))


def score_model(rows: list[dict[str, Any]], model: MotionModel, reference: str, horizon_s: float, sample_period_s: float) -> tuple[float, float, int]:
    errors = prediction_errors(rows, model, reference, [horizon_s], sample_period_s)
    moving = [row for row in errors if is_moving_error(row)]
    if moving:
        errors = moving
    pos = [float(row["position_error_m"]) for row in errors]
    yaw = [float(row["yaw_error_rad"]) for row in errors]
    pos_rmse = rmse(pos)
    yaw_rmse = rmse(yaw)
    if pos_rmse is None:
        return math.inf, math.inf, 0
    return pos_rmse + 2.0 * (yaw_rmse or 0.0), pos_rmse, len(pos)


def score_model_pointwise(rows: list[dict[str, Any]], model: MotionModel, reference: str) -> tuple[float, float, int]:
    rows_quality = good_rows(rows)
    series_by_session = grouped_series(rows_quality)
    state_by_session: dict[str, dict[str, float]] = {}
    prev_t_by_session: dict[str, float] = {}
    speed_errors: list[float] = []
    yaw_rate_errors: list[float] = []
    for row in rows_quality:
        session = str(row.get("session", ""))
        series = series_by_session.get(session)
        if session not in state_by_session:
            state_by_session[session] = initial_state(row, model, series)
            prev_t_by_session[session] = float(row["t"])
            continue
        _, state = update_state(row, prev_t_by_session[session], state_by_session[session], model, series)
        prev_t_by_session[session] = float(row["t"])
        pred_speed, pred_kappa = speed_and_kappa(state, model)
        ref_v = speed_ref(row, reference)
        ref_w = yaw_rate_ref(row, reference)
        if ref_v is not None:
            speed_errors.append(pred_speed - ref_v)
        if ref_w is not None:
            yaw_rate_errors.append(pred_speed * pred_kappa - ref_w)
    speed_score = rmse(speed_errors) or 0.0
    yaw_score = rmse(yaw_rate_errors) or 0.0
    return speed_score + 3.0 * yaw_score, yaw_score, min(len(speed_errors), len(yaw_rate_errors))


def clone_model(model: MotionModel, **updates: Any) -> MotionModel:
    data = asdict(model)
    data.update(updates)
    return MotionModel(**data)


def tune_grid(
    rows: list[dict[str, Any]],
    base: MotionModel,
    reference: str,
    horizon_s: float,
    sample_period_s: float,
    grid: dict[str, list[float]],
) -> MotionModel:
    best_score = math.inf
    best_model = base
    keys = list(grid.keys())
    for values in itertools.product(*(grid[key] for key in keys)):
        candidate = clone_model(base, **dict(zip(keys, values)))
        score, _, count = score_model(rows, candidate, reference, horizon_s, sample_period_s)
        if count and score < best_score:
            best_score = score
            best_model = candidate
    return best_model


def solve_ridge(xs: list[list[float]], ys: list[float], ridge: float = 1e-4) -> list[float]:
    if not xs or not ys:
        return []
    try:
        import numpy as np
    except ImportError:
        return [0.0 for _ in xs[0]]
    x = np.asarray(xs, dtype=float)
    y = np.asarray(ys, dtype=float)
    lhs = x.T @ x + ridge * np.eye(x.shape[1])
    rhs = x.T @ y
    try:
        coeff = np.linalg.solve(lhs, rhs)
    except np.linalg.LinAlgError:
        coeff = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
    return [float(v) for v in coeff]


def train_residuals(rows: list[dict[str, Any]], model: MotionModel, reference: str, *, fit_speed: bool, fit_kappa: bool) -> MotionModel:
    speed_features = ["bias", "v", "abs_v", "vdot", "abs_vdot", "v_abs_phi"]
    kappa_features = ["bias", "phi", "abs_phi", "phidot", "abs_phidot", "v_phi", "phi2"]
    if model.family == "measured":
        speed_features = ["bias", "v", "abs_v"]
        kappa_features = ["bias", "abs_v", "phi", "abs_phi", "v_abs_phi", "phi2"]
    series_by_session = grouped_series(rows)
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
        series = series_by_session.get(session)
        if session not in state_by_session:
            state_by_session[session] = initial_state(row, model, series)
            prev_t_by_session[session] = float(row["t"])
            continue
        _, state = update_state(row, prev_t_by_session[session], state_by_session[session], model, series)
        prev_t_by_session[session] = float(row["t"])
        base_speed, base_kappa = speed_and_kappa(state, model)
        ref_v = speed_ref(row, reference)
        ref_w = yaw_rate_ref(row, reference)
        if fit_speed and ref_v is not None:
            speed_x.append(feature_vector(state, speed_features))
            speed_y.append(ref_v - base_speed)
        speed_for_kappa = ref_v if ref_v is not None else base_speed
        if fit_kappa and ref_w is not None and abs(speed_for_kappa) > 0.15:
            kappa_x.append(feature_vector(state, kappa_features))
            kappa_y.append(ref_w / speed_for_kappa - base_kappa)
    trained = clone_model(model)
    if fit_speed:
        trained.speed_residual_features = speed_features
        trained.speed_residual_coefficients = solve_ridge(speed_x, speed_y, ridge=1e-3)
    if fit_kappa:
        trained.kappa_residual_features = kappa_features
        trained.kappa_residual_coefficients = solve_ridge(kappa_x, kappa_y, ridge=1e-3)
    return trained


def train_measured_progression(rows: list[dict[str, Any]], reference: str, horizon_s: float, sample_period_s: float) -> list[MotionModel]:
    m0 = MotionModel(
        "M0_measured_exact",
        "measured",
        "M0",
        "tachometer speed + measured articulation + exact articulated curvature",
        "tacho",
        "measured",
    )
    m1 = tune_grid(
        rows,
        clone_model(m0, name="M1_measured_speed_yaw_scale", level="M1", description="M0 plus speed scale and yaw gain"),
        reference,
        horizon_s,
        sample_period_s,
        {"speed_scale": [0.8, 0.9, 1.0, 1.1, 1.2], "yaw_gain": [0.6, 0.75, 0.9, 1.0, 1.1, 1.25, 1.4]},
    )
    m2 = tune_grid(
        rows,
        clone_model(m1, name="M2_measured_phi_calib", level="M2", description="M1 plus articulation scale and bias"),
        reference,
        horizon_s,
        sample_period_s,
        {"phi_scale": [0.75, 0.9, 1.0, 1.1, 1.25], "phi_bias_rad": [-0.08, -0.04, 0.0, 0.04, 0.08]},
    )
    m3 = tune_grid(
        rows,
        clone_model(m2, name="M3_measured_phi_delay", level="M3", description="M2 plus articulation delay"),
        reference,
        horizon_s,
        sample_period_s,
        {"phi_delay_s": [-0.4, -0.25, -0.1, 0.0, 0.1, 0.25, 0.4]},
    )
    m4 = train_residuals(
        rows,
        clone_model(m3, name="M4_measured_slip_residual", level="M4", description="M3 plus learned curvature/slip residual"),
        reference,
        fit_speed=False,
        fit_kappa=True,
    )
    return [m0, m1, m2, m3, m4]


def train_command_progression(rows: list[dict[str, Any]], reference: str, horizon_s: float, sample_period_s: float) -> list[MotionModel]:
    m0 = MotionModel(
        "M0_command_exact",
        "command",
        "M0",
        "raw command speed/articulation + exact articulated curvature",
        "cmd",
        "cmd",
    )
    m1 = tune_grid(
        rows,
        clone_model(m0, name="M1_command_lag", level="M1", description="M0 plus first-order speed/articulation lag"),
        reference,
        horizon_s,
        sample_period_s,
        {"tau_v_s": [0.0, 0.2, 0.5, 0.8, 1.2, 1.8, 2.5], "tau_phi_s": [0.0, 0.2, 0.5, 0.8, 1.2, 1.8, 2.5]},
    )
    m2 = tune_grid(
        rows,
        clone_model(m1, name="M2_command_calibrated", level="M2", description="M1 plus command-state scale and yaw gain"),
        reference,
        horizon_s,
        sample_period_s,
        {
            "speed_scale": [0.6, 0.75, 0.9, 1.0, 1.1, 1.25],
            "yaw_gain": [0.5, 0.75, 1.0, 1.25, 1.5],
            "phi_scale": [0.6, 0.8, 1.0, 1.2, 1.4],
        },
    )
    m3 = train_residuals(
        rows,
        clone_model(m2, name="M3_command_curvature_residual", level="M3", description="M2 plus command-only curvature residual"),
        reference,
        fit_speed=False,
        fit_kappa=True,
    )
    m4 = train_residuals(
        rows,
        clone_model(m2, name="M4_command_twist_residual", level="M4", description="M2 plus command-only speed and curvature residuals"),
        reference,
        fit_speed=True,
        fit_kappa=True,
    )
    return [m0, m1, m2, m3, m4]


def summarize_errors(errors: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for reference in sorted({str(row["reference"]) for row in errors}):
        summary[reference] = {}
        ref_rows = [row for row in errors if row["reference"] == reference]
        for family in sorted({str(row["family"]) for row in ref_rows}):
            summary[reference][family] = {}
            family_rows = [row for row in ref_rows if row["family"] == family]
            for model in sorted({str(row["model"]) for row in family_rows}):
                summary[reference][family][model] = {}
                model_rows = [row for row in family_rows if row["model"] == model]
                for horizon in sorted({float(row["horizon_s"]) for row in model_rows}):
                    rows_h = [row for row in model_rows if abs(float(row["horizon_s"]) - horizon) < 1e-9]
                    summary[reference][family][model][f"{horizon:g}s"] = {
                        "position_m": finite_stats(float(row["position_error_m"]) for row in rows_h),
                        "yaw_deg": finite_stats(math.degrees(float(row["yaw_error_rad"])) for row in rows_h),
                    }
    return summary


def gain_table(errors: list[dict[str, Any]], horizon_s: float, reference: str) -> list[dict[str, Any]]:
    order = [
        "odom_delta",
        "M0_measured_exact",
        "M1_measured_speed_yaw_scale",
        "M2_measured_phi_calib",
        "M3_measured_phi_delay",
        "M4_measured_slip_residual",
        "M0_command_exact",
        "M1_command_lag",
        "M2_command_calibrated",
        "M3_command_curvature_residual",
        "M4_command_twist_residual",
    ]
    rows = []
    previous_by_family: dict[str, float] = {}
    for model in order:
        vals = [
            float(row["position_error_m"])
            for row in errors
            if row["reference"] == reference and row["model"] == model and abs(float(row["horizon_s"]) - horizon_s) < 1e-9
        ]
        score = rmse(vals)
        if score is None:
            continue
        family = next((str(row["family"]) for row in errors if row["model"] == model), "")
        previous = previous_by_family.get(family)
        rows.append(
            {
                "reference": reference,
                "family": family,
                "model": model,
                "horizon_s": horizon_s,
                "samples": len(vals),
                "position_rmse_m": score,
                "gain_vs_previous_family_model_pct": 100.0 * (previous - score) / previous if previous and previous > 1e-9 else None,
            }
        )
        previous_by_family[family] = score
    return rows


def kfold_evaluate(
    rows: list[dict[str, Any]],
    reference: str,
    k: int,
    fold_mode: str,
    horizon_s: float,
    sample_period_s: float,
) -> tuple[list[dict[str, Any]], list[MotionModel]]:
    trainable = good_rows(rows)
    folds = split_folds(trainable, k, fold_mode)
    kfold_rows: list[dict[str, Any]] = []
    full_models = train_measured_progression(trainable, reference, horizon_s, sample_period_s) + train_command_progression(
        trainable, reference, horizon_s, sample_period_s
    )
    for fold_idx, (train, test) in enumerate(folds):
        models = train_measured_progression(train, reference, horizon_s, sample_period_s) + train_command_progression(
            train, reference, horizon_s, sample_period_s
        )
        for model in models:
            errs = prediction_errors(test, model, reference, [horizon_s], sample_period_s)
            for window_set, rows_eval in (("all", errs), ("moving", [row for row in errs if is_moving_error(row)])):
                vals = [float(row["position_error_m"]) for row in rows_eval]
                yaws = [math.degrees(float(row["yaw_error_rad"])) for row in rows_eval]
                kfold_rows.append(
                    {
                        "fold": fold_idx,
                        "window_set": window_set,
                        "reference": reference,
                        "family": model.family,
                        "level": model.level,
                        "model": model.name,
                        "horizon_s": horizon_s,
                        "test_windows": len(vals),
                        "position_rmse_m": rmse(vals),
                        "position_median_m": percentile(vals, 50.0),
                        "position_p95_m": percentile(vals, 95.0),
                        "yaw_rmse_deg": rmse(yaws),
                    }
                )
    return kfold_rows, full_models


def write_plots(out_dir: Path, errors: list[dict[str, Any]], horizon_for_boxplot: float) -> dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}

    outputs: dict[str, str] = {}
    out_dir.mkdir(parents=True, exist_ok=True)
    model_order = [
        "odom_delta",
        "M0_measured_exact",
        "M1_measured_speed_yaw_scale",
        "M2_measured_phi_calib",
        "M3_measured_phi_delay",
        "M4_measured_slip_residual",
        "M0_command_exact",
        "M1_command_lag",
        "M2_command_calibrated",
        "M3_command_curvature_residual",
        "M4_command_twist_residual",
    ]
    horizons = sorted({float(row["horizon_s"]) for row in errors})

    def save(name: str) -> None:
        path = out_dir / name
        plt.tight_layout()
        plt.savefig(path, dpi=170)
        plt.close()
        outputs[name] = str(path)

    def vals(reference: str, model: str, horizon: float, key: str) -> list[float]:
        return [
            float(row[key])
            for row in errors
            if row["reference"] == reference and row["model"] == model and abs(float(row["horizon_s"]) - horizon) < 1e-9
        ]

    for reference in sorted({str(row["reference"]) for row in errors}):
        for metric, ylabel, filename in (
            ("position_error_m", "position RMSE [m]", f"calibrated_position_rmse_vs_horizon_{reference}.png"),
            ("yaw_error_rad", "yaw RMSE [deg]", f"calibrated_yaw_rmse_vs_horizon_{reference}.png"),
        ):
            plt.figure(figsize=(13, 6))
            for model in model_order:
                if not any(row["reference"] == reference and row["model"] == model for row in errors):
                    continue
                ys = []
                for horizon in horizons:
                    data = vals(reference, model, horizon, metric)
                    if metric == "yaw_error_rad":
                        data = [math.degrees(v) for v in data]
                    ys.append(rmse(data) or math.nan)
                plt.plot(horizons, ys, marker="o", linewidth=1.5, label=model)
            plt.xlabel("prediction horizon [s]")
            plt.ylabel(ylabel)
            plt.title(f"Calibrated model progression vs {reference.upper()}")
            plt.grid(True, alpha=0.35)
            plt.legend(fontsize=7, ncol=2)
            save(filename)

        plt.figure(figsize=(13, 6))
        for model in model_order:
            if not any(row["reference"] == reference and row["model"] == model for row in errors):
                continue
            med = []
            q1 = []
            q3 = []
            for horizon in horizons:
                data = vals(reference, model, horizon, "position_error_m")
                med.append(percentile(data, 50.0) or math.nan)
                q1.append(percentile(data, 25.0) or math.nan)
                q3.append(percentile(data, 75.0) or math.nan)
            line = plt.plot(horizons, med, marker="o", linewidth=1.4, label=model)[0]
            plt.fill_between(horizons, q1, q3, color=line.get_color(), alpha=0.12)
        plt.xlabel("prediction horizon [s]")
        plt.ylabel("position error median + IQR [m]")
        plt.title(f"Median/IQR calibrated position error vs {reference.upper()}")
        plt.grid(True, alpha=0.35)
        plt.legend(fontsize=7, ncol=2)
        save(f"calibrated_position_median_iqr_{reference}.png")

        rows_h = [row for row in errors if row["reference"] == reference and abs(float(row["horizon_s"]) - horizon_for_boxplot) < 1e-9]
        present = [model for model in model_order if any(row["model"] == model for row in rows_h)]
        data = [[float(row["position_error_m"]) for row in rows_h if row["model"] == model] or [0.0] for model in present]
        labels = [model.replace("_", "\n", 2) for model in present]
        plt.figure(figsize=(14, 6))
        plt.boxplot(data, labels=labels, showmeans=True, showfliers=False, whis=(5, 95))
        plt.yscale("symlog", linthresh=0.1)
        plt.ylabel(f"{horizon_for_boxplot:g}s position error [m]")
        plt.title(f"Calibrated RPE boxplot vs {reference.upper()}")
        plt.xticks(rotation=25, ha="right")
        plt.grid(True, axis="y", alpha=0.35)
        save(f"calibrated_h{horizon_for_boxplot:g}_boxplot_{reference}.png")

        for model in ("M4_measured_slip_residual", "M4_command_twist_residual"):
            model_rows = [row for row in rows_h if row["model"] == model]
            if not model_rows:
                continue
            errors_m = [float(row["position_error_m"]) for row in model_rows]
            plt.figure(figsize=(8, 5))
            plt.hist(errors_m, bins=60, alpha=0.75)
            plt.xlabel(f"{horizon_for_boxplot:g}s position error [m]")
            plt.ylabel("count")
            plt.title(f"{model} histogram vs {reference.upper()}")
            plt.grid(True, alpha=0.25)
            save(f"{model}_h{horizon_for_boxplot:g}_hist_{reference}.png")

            sorted_err = sorted(errors_m)
            cdf = [(i + 1) / len(sorted_err) for i in range(len(sorted_err))]
            plt.figure(figsize=(8, 5))
            plt.plot(sorted_err, cdf)
            plt.xlabel(f"{horizon_for_boxplot:g}s position error [m]")
            plt.ylabel("CDF")
            plt.title(f"{model} CDF vs {reference.upper()}")
            plt.grid(True, alpha=0.35)
            save(f"{model}_h{horizon_for_boxplot:g}_cdf_{reference}.png")

        best_rows = [row for row in rows_h if row["model"] in {"M4_measured_slip_residual", "M4_command_twist_residual"}]
        for key, xlabel in (
            ("max_abs_phi_rad", "max |measured articulation| [rad]"),
            ("max_abs_cmd_phi_rad", "max |command articulation| [rad]"),
            ("max_abs_speed_ms", "max |measured speed| [m/s]"),
            ("max_abs_cmd_speed_ms", "max |command speed| [m/s]"),
            ("max_abs_yaw_rate_ref_rad_s", "max |reference yaw rate| [rad/s]"),
        ):
            if not best_rows:
                continue
            plt.figure(figsize=(8, 5))
            for model in ("M4_measured_slip_residual", "M4_command_twist_residual"):
                rows_model = [row for row in best_rows if row["model"] == model and key in row]
                if not rows_model:
                    continue
                plt.scatter(
                    [float(row.get(key, 0.0) or 0.0) for row in rows_model],
                    [float(row["position_error_m"]) for row in rows_model],
                    s=9,
                    alpha=0.4,
                    label=model,
                )
            plt.xlabel(xlabel)
            plt.ylabel(f"{horizon_for_boxplot:g}s position error [m]")
            plt.title(f"Failure correlation vs {reference.upper()}")
            plt.grid(True, alpha=0.35)
            plt.legend(fontsize=8)
            save(f"calibrated_h{horizon_for_boxplot:g}_error_vs_{key}_{reference}.png")

    return outputs


def top_failures(errors: list[dict[str, Any]], reference: str, horizon_s: float, limit: int = 120) -> list[dict[str, Any]]:
    rows = [
        row
        for row in errors
        if row["reference"] == reference
        and abs(float(row["horizon_s"]) - horizon_s) < 1e-9
        and row["model"] in {"M4_measured_slip_residual", "M4_command_twist_residual"}
    ]
    rows.sort(key=lambda row: float(row["position_error_m"]), reverse=True)
    return rows[:limit]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--fit-reference", choices=["icp", "odom"], default="icp")
    parser.add_argument("--eval-references", default="icp,odom")
    parser.add_argument("--horizons", default="0.5,1,2,3,5,10")
    parser.add_argument("--sample-period-s", type=float, default=0.5)
    parser.add_argument("--tune-sample-period-s", type=float, default=1.5)
    parser.add_argument("--tune-horizon-s", type=float, default=3.0)
    parser.add_argument("--k", type=int, default=5)
    parser.add_argument("--fold-mode", choices=["segment", "session"], default="segment")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = args.session.expanduser().resolve()
    dataset_csv = dataset_csv_for_session(session)
    out_dir = args.output_dir or (session / "motion_model_validation" / "motion_research" / "calibrated_models")
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = read_rows(dataset_csv)
    trainable = good_rows(rows)
    horizons = [float(item) for item in args.horizons.split(",") if item.strip()]
    eval_refs = [item.strip() for item in args.eval_references.split(",") if item.strip()]

    kfold_rows, models = kfold_evaluate(
        rows,
        args.fit_reference,
        args.k,
        args.fold_mode,
        args.tune_horizon_s,
        args.tune_sample_period_s,
    )

    all_errors: list[dict[str, Any]] = []
    for reference in eval_refs:
        all_errors.extend(odom_delta_errors(rows, reference, horizons, args.sample_period_s))
        for model in models:
            all_errors.extend(prediction_errors(rows, model, reference, horizons, args.sample_period_s))
    moving_errors = [row for row in all_errors if is_moving_error(row)]

    parameter_rows = [asdict(model) for model in models]
    gain_rows = []
    gain_rows_moving = []
    for reference in eval_refs:
        gain_rows.extend(gain_table(all_errors, args.tune_horizon_s, reference))
        gain_rows_moving.extend(gain_table(moving_errors, args.tune_horizon_s, reference))
    failure_rows = []
    for reference in eval_refs:
        failure_rows.extend(top_failures(all_errors, reference, args.tune_horizon_s))

    write_csv(out_dir / "calibrated_prediction_errors.csv", all_errors)
    write_csv(out_dir / "calibrated_prediction_errors_moving_windows.csv", moving_errors)
    write_csv(out_dir / "calibrated_kfold_results.csv", kfold_rows)
    write_csv(out_dir / "calibrated_parameters.csv", parameter_rows)
    write_csv(out_dir / "calibrated_gain_table.csv", gain_rows)
    write_csv(out_dir / "calibrated_gain_table_moving_windows.csv", gain_rows_moving)
    write_csv(out_dir / "top_failures_by_model.csv", failure_rows)
    plots = write_plots(out_dir, all_errors, args.tune_horizon_s)
    moving_plots = write_plots(out_dir / "moving_windows", moving_errors, args.tune_horizon_s) if moving_errors else {}

    summary = {
        "session": session.name,
        "dataset_csv": str(dataset_csv),
        "rows": len(rows),
        "trainable_rows": len(trainable),
        "moving_prediction_rows": len(moving_errors),
        "moving_window_definition": {
            "min_reference_distance_m": MOVING_MIN_DISTANCE_M,
            "min_reference_yaw_delta_deg": math.degrees(MOVING_MIN_YAW_RAD),
        },
        "fit_reference": args.fit_reference,
        "eval_references": eval_refs,
        "fold_mode": args.fold_mode,
        "k": args.k,
        "horizons_s": horizons,
        "tune_horizon_s": args.tune_horizon_s,
        "models": parameter_rows,
        "gain_table": gain_rows,
        "gain_table_moving_windows": gain_rows_moving,
        "summary": summarize_errors(all_errors),
        "summary_moving_windows": summarize_errors(moving_errors),
        "interpretation_contract": {
            "odom_delta": "strong local baseline using measured odom relative motion, not a deployable open-loop model",
            "measured_models": "best kinematic potential using measured tachometer and articulation",
            "command_models": "open-loop command-only prediction potential",
            "non_monotonic_test_error": "expected if added parameters overfit train folds or if command signals are less informative than measured states",
        },
        "outputs": {
            "prediction_errors_csv": str(out_dir / "calibrated_prediction_errors.csv"),
            "prediction_errors_moving_csv": str(out_dir / "calibrated_prediction_errors_moving_windows.csv"),
            "kfold_csv": str(out_dir / "calibrated_kfold_results.csv"),
            "parameters_csv": str(out_dir / "calibrated_parameters.csv"),
            "gain_csv": str(out_dir / "calibrated_gain_table.csv"),
            "gain_moving_csv": str(out_dir / "calibrated_gain_table_moving_windows.csv"),
            "top_failures_csv": str(out_dir / "top_failures_by_model.csv"),
            "plots": plots,
            "moving_plots": moving_plots,
        },
    }
    (out_dir / "calibrated_summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump({"summary": {k: summary[k] for k in ("session", "rows", "trainable_rows", "outputs")}}, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
