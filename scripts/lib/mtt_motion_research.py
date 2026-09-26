#!/usr/bin/env python3
"""Shared utilities for MTT motion-model research scripts.

The helpers in this file intentionally avoid ROS runtime dependencies.  They
operate on the postprocess CSV files produced by the bag replay pipeline.
"""

from __future__ import annotations

import bisect
import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


L_F_M = 0.9
L_R_M = 1.5
WHEELBASE_M = L_F_M + L_R_M
MAX_ARTICULATION_RAD = math.radians(60.0)
MIN_CURVATURE_SPEED_MS = 0.10


def parse_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def parse_bool(value: Any) -> bool:
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value) != 0.0
    return str(value).strip().lower() in {"1", "true", "yes", "y", "ok", "pass"}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def direction_sign(value: Any) -> float:
    text = str(value or "").strip().lower()
    return -1.0 if text in {"reverse", "backward", "rev", "rear", "-1"} else 1.0


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_to_roll_pitch(x: float, y: float, z: float, w: float) -> tuple[float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


def nominal_curvature(phi: float, model: str = "exact", lf_m: float = L_F_M, lr_m: float = L_R_M) -> float:
    phi = clamp(phi, -MAX_ARTICULATION_RAD, MAX_ARTICULATION_RAD)
    if model == "tangent":
        return math.tan(phi) / max(lf_m + lr_m, 1e-9)
    return math.sin(phi) / max(lf_m * math.cos(phi) + lr_m, 1e-9)


def se2_log_error(
    pred_x: float,
    pred_y: float,
    pred_yaw: float,
    ref_x: float,
    ref_y: float,
    ref_yaw: float,
) -> tuple[float, float, float]:
    """Return reference-minus-prediction error expressed in prediction body frame."""
    dx = ref_x - pred_x
    dy = ref_y - pred_y
    c = math.cos(pred_yaw)
    s = math.sin(pred_yaw)
    return c * dx + s * dy, -s * dx + c * dy, wrap_angle(ref_yaw - pred_yaw)


def rmse(values: Iterable[float]) -> float | None:
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return None
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def percentile(values: Iterable[float], pct: float) -> float | None:
    vals = sorted(v for v in values if math.isfinite(v))
    if not vals:
        return None
    idx = int(round((pct / 100.0) * (len(vals) - 1)))
    return vals[clamp(idx, 0, len(vals) - 1)]  # type: ignore[arg-type]


def finite_stats(values: Iterable[float]) -> dict[str, float | int | None]:
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return {"count": 0, "min": None, "mean": None, "median": None, "p95": None, "max": None, "rmse": None}
    return {
        "count": len(vals),
        "min": min(vals),
        "mean": sum(vals) / len(vals),
        "median": statistics.median(vals),
        "p95": percentile(vals, 95.0),
        "max": max(vals),
        "rmse": rmse(vals),
    }


def read_csv_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for raw in csv.DictReader(stream):
            row: dict[str, Any] = dict(raw)
            for key, value in list(row.items()):
                parsed = parse_float(value)
                if parsed is not None:
                    row[key] = parsed
            rows.append(row)
    rows.sort(key=lambda row: float(row.get("t", 0.0)))
    return rows


def write_csv_rows(path: Path, rows: list[dict[str, Any]], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        keys: list[str] = []
        for row in rows:
            for key in row:
                if key not in keys:
                    keys.append(key)
        fieldnames = keys
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def resolve_sessions(path_value: str | Path) -> list[Path]:
    path = Path(path_value).expanduser().resolve()
    if (path / "bag" / "metadata.yaml").exists() or (path / "postprocess_dataset").exists():
        return [path]
    sessions = sorted(p.parent.parent for p in path.glob("*/bag/metadata.yaml"))
    if sessions:
        return sessions
    sessions = sorted(p.parent.parent for p in path.glob("*/postprocess_dataset/motion_model_dataset.csv"))
    if sessions:
        return sessions
    raise SystemExit(f"Could not resolve sessions from {path}")


def find_motion_csv(session_dir: Path) -> Path:
    candidates = [
        session_dir / "postprocess_dataset" / "motion_model_dataset.csv",
        session_dir / "postprocess_dataset" / "dataset.csv",
        session_dir / "motion_model_validation" / "aligned_samples.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"No motion dataset CSV found for {session_dir}")


def _series_variation(rows: list[dict[str, Any]], key: str) -> float:
    vals = [float(v) for row in rows if (v := parse_float(row.get(key))) is not None]
    if len(vals) < 5:
        return 0.0
    return max(vals) - min(vals)


def choose_articulation_column(rows: list[dict[str, Any]], preference: str = "auto") -> str | None:
    candidates = {
        "mtt": ["mtt_articulation_angle", "mtt_articulation_rad"],
        "trailer": ["trailer_articulation_angle", "trailer_articulation_rad", "articulation_rad"],
        "hardware": ["hardware_articulation_rad"],
    }
    if preference != "auto":
        for key in candidates.get(preference, [preference]):
            if any(parse_float(row.get(key)) is not None for row in rows):
                return key
        return None
    scored: list[tuple[float, str]] = []
    for keys in candidates.values():
        for key in keys:
            if any(parse_float(row.get(key)) is not None for row in rows):
                scored.append((_series_variation(rows, key), key))
    if not scored:
        return None
    scored.sort(reverse=True)
    return scored[0][1]


def signed_tacho(row: dict[str, Any]) -> float | None:
    speed = parse_float(row.get("tach_speed_ms"))
    if speed is None:
        return None
    return speed * direction_sign(row.get("tach_direction"))


def bool_field(row: dict[str, Any], key: str) -> bool:
    return parse_bool(row.get(key))


def derive_research_rows(
    raw_rows: list[dict[str, Any]],
    *,
    session_name: str,
    articulation_preference: str = "auto",
    max_icp_gap_s: float = 0.35,
    max_icp_step_m: float = 1.0,
    max_icp_speed_ms: float = 8.0,
    max_icp_yaw_rate_rad_s: float = 2.5,
) -> list[dict[str, Any]]:
    if not raw_rows:
        return []
    articulation_key = choose_articulation_column(raw_rows, articulation_preference)
    rows: list[dict[str, Any]] = []
    prev: dict[str, Any] | None = None
    for raw in raw_rows:
        t = parse_float(raw.get("t"))
        ix = parse_float(raw.get("icp_x"))
        iy = parse_float(raw.get("icp_y"))
        iyaw = parse_float(raw.get("icp_yaw", raw.get("icp_heading")))
        if None in (t, ix, iy, iyaw):
            continue
        assert t is not None and ix is not None and iy is not None and iyaw is not None
        row = dict(raw)
        # Normalize the legacy heading alias before this row becomes `prev`.
        row["icp_yaw"] = iyaw
        row["session"] = session_name
        row["articulation_selected_source"] = articulation_key or ""
        phi = parse_float(raw.get(articulation_key)) if articulation_key else None
        row["phi_rad"] = phi if phi is not None else math.nan
        row["tacho_signed_ms"] = signed_tacho(raw) if signed_tacho(raw) is not None else math.nan
        row["cmd_speed_ms"] = parse_float(raw.get("cmd_linear_x")) if parse_float(raw.get("cmd_linear_x")) is not None else math.nan
        cmd_az = parse_float(raw.get("cmd_angular_z"))
        row["cmd_phi_rad"] = clamp(cmd_az, -1.0, 1.0) * MAX_ARTICULATION_RAD if cmd_az is not None else math.nan

        q = [parse_float(raw.get(k)) for k in ("imu_orientation_x", "imu_orientation_y", "imu_orientation_z", "imu_orientation_w")]
        if all(v is not None for v in q):
            assert all(v is not None for v in q)
            roll, pitch = quat_to_roll_pitch(q[0], q[1], q[2], q[3])  # type: ignore[arg-type]
            row["imu_roll_rad"] = roll
            row["imu_pitch_rad"] = pitch
        else:
            row["imu_roll_rad"] = math.nan
            row["imu_pitch_rad"] = math.nan
        imu_yaw_rate = parse_float(raw.get("imu_angular_velocity_z"))
        row["imu_yaw_rate_rad_s"] = imu_yaw_rate if imu_yaw_rate is not None else math.nan
        ax = parse_float(raw.get("imu_linear_acceleration_x")) or 0.0
        ay = parse_float(raw.get("imu_linear_acceleration_y")) or 0.0
        az = parse_float(raw.get("imu_linear_acceleration_z")) or 0.0
        row["imu_acc_norm_ms2"] = math.sqrt(ax * ax + ay * ay + az * az)

        if prev is None:
            row["dt_s"] = 0.0
            row["icp_speed_ms"] = 0.0
            row["icp_yaw_rate_rad_s_derived"] = 0.0
            row["icp_step_m_derived"] = 0.0
            row["odom_speed_ms"] = 0.0
            row["odom_yaw_rate_rad_s_derived"] = 0.0
            row["odom_step_m_derived"] = 0.0
            row["tacho_accel_ms2"] = 0.0
            row["phi_rate_rad_s"] = 0.0
            row["cmd_accel_ms2"] = 0.0
            row["cmd_phi_rate_rad_s"] = 0.0
        else:
            dt = max(0.0, t - float(prev["t"]))
            dx = ix - float(prev["icp_x"])
            dy = iy - float(prev["icp_y"])
            dyaw = wrap_angle(iyaw - float(prev["icp_yaw"]))
            heading_mid = wrap_angle(float(prev["icp_yaw"]) + 0.5 * dyaw)
            row["dt_s"] = dt
            row["icp_step_m_derived"] = math.hypot(dx, dy)
            row["icp_speed_ms"] = ((dx * math.cos(heading_mid) + dy * math.sin(heading_mid)) / dt) if dt > 1e-6 else 0.0
            row["icp_yaw_rate_rad_s_derived"] = dyaw / dt if dt > 1e-6 else 0.0
            ox = parse_float(row.get("odom_x"))
            oy = parse_float(row.get("odom_y"))
            oyaw = parse_float(row.get("odom_yaw"))
            prev_ox = parse_float(prev.get("odom_x"))
            prev_oy = parse_float(prev.get("odom_y"))
            prev_oyaw = parse_float(prev.get("odom_yaw"))
            if None not in (ox, oy, oyaw, prev_ox, prev_oy, prev_oyaw) and dt > 1e-6:
                assert ox is not None and oy is not None and oyaw is not None
                assert prev_ox is not None and prev_oy is not None and prev_oyaw is not None
                odx = ox - prev_ox
                ody = oy - prev_oy
                odyaw = wrap_angle(oyaw - prev_oyaw)
                odom_heading_mid = wrap_angle(prev_oyaw + 0.5 * odyaw)
                row["odom_step_m_derived"] = math.hypot(odx, ody)
                row["odom_speed_ms"] = (odx * math.cos(odom_heading_mid) + ody * math.sin(odom_heading_mid)) / dt
                row["odom_yaw_rate_rad_s_derived"] = odyaw / dt
            else:
                row["odom_step_m_derived"] = math.nan
                row["odom_speed_ms"] = math.nan
                row["odom_yaw_rate_rad_s_derived"] = math.nan
            prev_tacho = parse_float(prev.get("tacho_signed_ms"))
            curr_tacho = parse_float(row.get("tacho_signed_ms"))
            row["tacho_accel_ms2"] = (curr_tacho - prev_tacho) / dt if dt > 1e-6 and prev_tacho is not None and curr_tacho is not None else 0.0
            prev_phi = parse_float(prev.get("phi_rad"))
            curr_phi = parse_float(row.get("phi_rad"))
            row["phi_rate_rad_s"] = (curr_phi - prev_phi) / dt if dt > 1e-6 and prev_phi is not None and curr_phi is not None else 0.0
            prev_cmd_v = parse_float(prev.get("cmd_speed_ms"))
            curr_cmd_v = parse_float(row.get("cmd_speed_ms"))
            row["cmd_accel_ms2"] = (curr_cmd_v - prev_cmd_v) / dt if dt > 1e-6 and prev_cmd_v is not None and curr_cmd_v is not None else 0.0
            prev_cmd_phi = parse_float(prev.get("cmd_phi_rad"))
            curr_cmd_phi = parse_float(row.get("cmd_phi_rad"))
            row["cmd_phi_rate_rad_s"] = (curr_cmd_phi - prev_cmd_phi) / dt if dt > 1e-6 and prev_cmd_phi is not None and curr_cmd_phi is not None else 0.0

        v_for_curv = parse_float(row.get("tacho_signed_ms"))
        if v_for_curv is None or abs(v_for_curv) < MIN_CURVATURE_SPEED_MS:
            v_for_curv = parse_float(row.get("icp_speed_ms"))
        yaw_rate = parse_float(row.get("icp_yaw_rate_rad_s"))
        if yaw_rate is None:
            yaw_rate = parse_float(row.get("icp_yaw_rate_rad_s_derived"))
        if v_for_curv is not None and yaw_rate is not None and abs(v_for_curv) >= MIN_CURVATURE_SPEED_MS:
            row["kappa_real"] = yaw_rate / v_for_curv
        else:
            row["kappa_real"] = math.nan
        if phi is not None:
            row["kappa_nom_exact"] = nominal_curvature(phi, "exact")
            row["kappa_nom_tangent"] = nominal_curvature(phi, "tangent")
            row["delta_kappa_exact"] = row["kappa_real"] - row["kappa_nom_exact"] if math.isfinite(row["kappa_real"]) else math.nan
            row["gamma_exact"] = row["kappa_real"] / row["kappa_nom_exact"] if abs(row["kappa_nom_exact"]) > 1e-6 and math.isfinite(row["kappa_real"]) else math.nan
        else:
            row["kappa_nom_exact"] = math.nan
            row["kappa_nom_tangent"] = math.nan
            row["delta_kappa_exact"] = math.nan
            row["gamma_exact"] = math.nan

        has_icp = bool_field(raw, "has_icp") or ix is not None
        icp_quality = bool_field(raw, "icp_quality_ok") or bool_field(raw, "icp_ground_truth_ok")
        if str(raw.get("icp_validation_state", "")).strip().lower() in {"ok", "accepted", "pass"}:
            icp_quality = True
        # Missing provenance is not proof of a physical tachometer. An explicit
        # false flag takes precedence over the legacy synthetic-source alias.
        if raw.get("has_real_tacho") not in (None, ""):
            has_real_tacho = bool_field(raw, "has_real_tacho")
        else:
            has_real_tacho = raw.get("tach_is_synthetic") not in (None, "")
        has_real_tacho = has_real_tacho and not parse_bool(raw.get("tach_is_synthetic"))
        step = parse_float(row.get("icp_step_m_derived")) or 0.0
        gap = parse_float(row.get("dt_s")) or 0.0
        speed_abs = abs(parse_float(row.get("icp_speed_ms")) or 0.0)
        yaw_abs = abs(parse_float(row.get("icp_yaw_rate_rad_s_derived")) or 0.0)
        row["has_real_tacho_research"] = int(has_real_tacho)
        row["quality_reason"] = ""
        sample_valid = has_icp and icp_quality and 1e-6 < gap <= max_icp_gap_s and step <= max_icp_step_m and speed_abs <= max_icp_speed_ms and yaw_abs <= max_icp_yaw_rate_rad_s
        if not sample_valid:
            reasons = []
            if not icp_quality:
                reasons.append("weak_icp")
            if gap <= 1e-6:
                reasons.append("invalid_dt")
            if gap > max_icp_gap_s:
                reasons.append("icp_gap")
            if step > max_icp_step_m:
                reasons.append("icp_step")
            if speed_abs > max_icp_speed_ms:
                reasons.append("speed_outlier")
            if yaw_abs > max_icp_yaw_rate_rad_s:
                reasons.append("yaw_outlier")
            row["quality_reason"] = ";".join(reasons)
        row["research_quality_ok"] = int(sample_valid)
        rows.append(row)
        prev = row
    return rows


def classify_motion(row: dict[str, Any]) -> str:
    v = abs(parse_float(row.get("tacho_signed_ms")) or parse_float(row.get("icp_speed_ms")) or 0.0)
    a = abs(parse_float(row.get("tacho_accel_ms2")) or 0.0)
    phi = abs(parse_float(row.get("phi_rad")) or 0.0)
    phidot = abs(parse_float(row.get("phi_rate_rad_s")) or 0.0)
    yaw_rate = abs(parse_float(row.get("icp_yaw_rate_rad_s_derived")) or 0.0)
    if v < 0.08:
        return "stop"
    if a > 0.8:
        return "high_accel"
    if phi < 0.04 and yaw_rate < 0.04:
        return "straight"
    if phidot > 0.25 or yaw_rate > 0.45:
        return "aggressive_turn"
    if phi > 0.18:
        return "constant_turn"
    if abs(parse_float(row.get("phi_rate_rad_s")) or 0.0) > 0.08:
        return "s_curve"
    return "mild_motion"


def assign_segments(rows: list[dict[str, Any]], *, min_segment_s: float = 1.0, max_gap_s: float = 0.5) -> list[dict[str, Any]]:
    current_id = -1
    prev_t: float | None = None
    prev_class = ""
    start_t = 0.0
    for row in rows:
        t = float(row["t"])
        motion_class = classify_motion(row)
        row["quality_class"] = "pass" if parse_bool(row.get("research_quality_ok")) else "weak_icp"
        new_segment = (
            prev_t is None
            or t - prev_t > max_gap_s
            or motion_class != prev_class
            or (t - start_t >= min_segment_s and motion_class in {"s_curve", "aggressive_turn", "high_accel"})
        )
        if new_segment:
            current_id += 1
            start_t = t
        row["motion_class"] = motion_class
        row["segment_id"] = current_id
        prev_t = t
        prev_class = motion_class
    return rows


def segment_summary(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_id: dict[int, list[dict[str, Any]]] = {}
    for row in rows:
        by_id.setdefault(int(row["segment_id"]), []).append(row)
    out = []
    for seg_id, seg_rows in sorted(by_id.items()):
        valid = [row for row in seg_rows if parse_bool(row.get("research_quality_ok"))]
        out.append(
            {
                "session": seg_rows[0].get("session", ""),
                "segment_id": seg_id,
                "motion_class": seg_rows[0].get("motion_class", ""),
                "start_t": float(seg_rows[0]["t"]),
                "end_t": float(seg_rows[-1]["t"]),
                "duration_s": float(seg_rows[-1]["t"]) - float(seg_rows[0]["t"]),
                "rows": len(seg_rows),
                "valid_rows": len(valid),
                "valid_ratio": len(valid) / max(len(seg_rows), 1),
                "distance_m": sum(abs(parse_float(row.get("icp_speed_ms")) or 0.0) * max(parse_float(row.get("dt_s")) or 0.0, 0.0) for row in valid),
                "max_abs_phi_rad": max((abs(parse_float(row.get("phi_rad")) or 0.0) for row in valid), default=0.0),
                "max_abs_roll_rad": max((abs(parse_float(row.get("imu_roll_rad")) or 0.0) for row in valid), default=0.0),
                "max_abs_pitch_rad": max((abs(parse_float(row.get("imu_pitch_rad")) or 0.0) for row in valid), default=0.0),
            }
        )
    return out


@dataclass
class ModelParams:
    name: str
    kind: str
    speed_source: str = "tacho"
    phi_source: str = "measured"
    curvature_model: str = "exact"
    speed_scale: float = 1.0
    yaw_gain: float = 1.0
    tau_v_s: float = 0.5
    tau_phi_s: float = 0.5
    speed_deadband_ms: float = 0.0
    phi_deadband_rad: float = 0.0
    residual_coefficients: list[float] | None = None
    residual_features: list[str] | None = None


class RowSeries:
    def __init__(self, rows: list[dict[str, Any]]):
        self.rows = sorted(rows, key=lambda row: float(row["t"]))
        self.times = [float(row["t"]) for row in self.rows]

    def nearest_index(self, t: float) -> int:
        idx = bisect.bisect_left(self.times, t)
        if idx <= 0:
            return 0
        if idx >= len(self.times):
            return len(self.times) - 1
        return idx if abs(self.times[idx] - t) < abs(self.times[idx - 1] - t) else idx - 1

    def nearest(self, t: float, tolerance_s: float = 0.08) -> dict[str, Any] | None:
        if not self.rows:
            return None
        idx = self.nearest_index(t)
        row = self.rows[idx]
        return row if abs(float(row["t"]) - t) <= tolerance_s else None


def input_speed(row: dict[str, Any], source: str) -> float | None:
    if source == "tacho":
        return parse_float(row.get("tacho_signed_ms"))
    if source == "cmd":
        return parse_float(row.get("cmd_speed_ms"))
    if source == "icp":
        return parse_float(row.get("icp_speed_ms"))
    if source == "odom":
        return parse_float(row.get("odom_speed_ms"))
    raise ValueError(f"Unsupported speed source {source}")


def input_phi(row: dict[str, Any], source: str) -> float | None:
    if source == "measured":
        return parse_float(row.get("phi_rad"))
    if source == "mtt":
        return parse_float(row.get("mtt_articulation_angle", row.get("mtt_articulation_rad")))
    if source == "trailer":
        return parse_float(row.get("trailer_articulation_angle", row.get("trailer_articulation_rad")))
    if source == "hardware":
        return parse_float(row.get("hardware_articulation_rad"))
    if source == "cmd":
        return parse_float(row.get("cmd_phi_rad"))
    raise ValueError(f"Unsupported phi source {source}")


def residual_feature_vector(row: dict[str, Any], features: list[str]) -> list[float]:
    out: list[float] = []
    for feature in features:
        if feature == "bias":
            out.append(1.0)
        elif feature == "abs_v":
            out.append(abs(parse_float(row.get("tacho_signed_ms")) or parse_float(row.get("icp_speed_ms")) or 0.0))
        elif feature == "v":
            out.append(parse_float(row.get("tacho_signed_ms")) or parse_float(row.get("icp_speed_ms")) or 0.0)
        elif feature == "phi":
            out.append(parse_float(row.get("phi_rad")) or 0.0)
        elif feature == "abs_phi":
            out.append(abs(parse_float(row.get("phi_rad")) or 0.0))
        elif feature == "phi_dot":
            out.append(parse_float(row.get("phi_rate_rad_s")) or 0.0)
        elif feature == "abs_phi_dot":
            out.append(abs(parse_float(row.get("phi_rate_rad_s")) or 0.0))
        elif feature == "roll":
            out.append(parse_float(row.get("imu_roll_rad")) or 0.0)
        elif feature == "pitch":
            out.append(parse_float(row.get("imu_pitch_rad")) or 0.0)
        elif feature == "accel":
            out.append(parse_float(row.get("tacho_accel_ms2")) or 0.0)
        elif feature == "imu_yaw_rate":
            out.append(parse_float(row.get("imu_yaw_rate_rad_s")) or 0.0)
        else:
            out.append(parse_float(row.get(feature)) or 0.0)
    return out


def solve_least_squares(x_rows: list[list[float]], y_vals: list[float], ridge: float = 1e-6) -> list[float]:
    if not x_rows:
        return []
    n = len(x_rows[0])
    ata = [[0.0 for _ in range(n)] for _ in range(n)]
    aty = [0.0 for _ in range(n)]
    for x, y in zip(x_rows, y_vals):
        for i in range(n):
            aty[i] += x[i] * y
            for j in range(n):
                ata[i][j] += x[i] * x[j]
    for i in range(n):
        ata[i][i] += ridge
    # Gaussian elimination with partial pivoting.
    a = [row[:] + [aty_i] for row, aty_i in zip(ata, aty)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-12:
            continue
        a[col], a[pivot] = a[pivot], a[col]
        div = a[col][col]
        for k in range(col, n + 1):
            a[col][k] /= div
        for r in range(n):
            if r == col:
                continue
            factor = a[r][col]
            for k in range(col, n + 1):
                a[r][k] -= factor * a[col][k]
    return [a[i][n] for i in range(n)]


def fit_residual_model(rows: list[dict[str, Any]], features: list[str], *, min_abs_speed_ms: float = MIN_CURVATURE_SPEED_MS) -> list[float]:
    x_rows: list[list[float]] = []
    y_vals: list[float] = []
    for row in rows:
        if not parse_bool(row.get("research_quality_ok")):
            continue
        v = parse_float(row.get("tacho_signed_ms")) or parse_float(row.get("icp_speed_ms")) or 0.0
        y = parse_float(row.get("delta_kappa_exact"))
        if y is None or not math.isfinite(y) or abs(v) < min_abs_speed_ms:
            continue
        x_rows.append(residual_feature_vector(row, features))
        y_vals.append(y)
    return solve_least_squares(x_rows, y_vals)


def simulate_window(
    rows: list[dict[str, Any]],
    start_idx: int,
    end_idx: int,
    params: ModelParams,
) -> tuple[float, float, float]:
    start = rows[start_idx]
    x = float(start["icp_x"])
    y = float(start["icp_y"])
    yaw = float(start.get("icp_yaw", start.get("icp_heading")))
    prev_t = float(start["t"])
    v_state = input_speed(start, params.speed_source) or 0.0
    phi_state = input_phi(start, params.phi_source) or 0.0

    if params.kind == "odom_delta":
        sx = parse_float(start.get("odom_x")) or 0.0
        sy = parse_float(start.get("odom_y")) or 0.0
        syaw = parse_float(start.get("odom_yaw")) or 0.0
        end = rows[end_idx]
        ex = parse_float(end.get("odom_x")) or sx
        ey = parse_float(end.get("odom_y")) or sy
        eyaw = parse_float(end.get("odom_yaw")) or syaw
        dx = ex - sx
        dy = ey - sy
        dyaw = wrap_angle(eyaw - syaw)
        c = math.cos(yaw - syaw)
        s = math.sin(yaw - syaw)
        return x + c * dx - s * dy, y + s * dx + c * dy, wrap_angle(yaw + dyaw)

    for row in rows[start_idx + 1 : end_idx + 1]:
        t = float(row["t"])
        dt = max(0.0, min(t - prev_t, 0.5))
        prev_t = t
        if dt <= 0.0:
            continue
        speed_in = input_speed(row, params.speed_source)
        phi_in = input_phi(row, params.phi_source)
        if speed_in is None or phi_in is None:
            continue
        if abs(speed_in) < params.speed_deadband_ms:
            speed_in = 0.0
        if abs(phi_in) < params.phi_deadband_rad:
            phi_in = 0.0
        if params.kind in {"m1", "m2", "m3"}:
            alpha_v = clamp(dt / max(params.tau_v_s, 1e-6), 0.0, 1.0)
            alpha_phi = clamp(dt / max(params.tau_phi_s, 1e-6), 0.0, 1.0)
            v_state += (speed_in - v_state) * alpha_v
            phi_state += (phi_in - phi_state) * alpha_phi
            speed = v_state
            phi = phi_state
        else:
            speed = speed_in
            phi = phi_in
        speed *= params.speed_scale
        kappa = nominal_curvature(phi, params.curvature_model) * params.yaw_gain
        if params.kind in {"m2", "m3"} and params.residual_coefficients and params.residual_features:
            feats = residual_feature_vector(row, params.residual_features)
            kappa += sum(c * f for c, f in zip(params.residual_coefficients, feats))
        yaw_rate = speed * kappa if abs(speed) >= MIN_CURVATURE_SPEED_MS else 0.0
        dyaw = yaw_rate * dt
        heading_mid = yaw + 0.5 * dyaw
        x += speed * math.cos(heading_mid) * dt
        y += speed * math.sin(heading_mid) * dt
        yaw = wrap_angle(yaw + dyaw)
    return x, y, yaw


def prediction_errors(
    rows: list[dict[str, Any]],
    params: ModelParams,
    *,
    horizons_s: list[float],
    sample_period_s: float = 0.5,
    require_real_tacho: bool = False,
    include_weak: bool = False,
) -> list[dict[str, Any]]:
    valid_rows = list(rows) if include_weak else [row for row in rows if parse_bool(row.get("research_quality_ok"))]
    valid_rows = [
        row
        for row in valid_rows
        if parse_float(row.get("t")) is not None
        and parse_float(row.get("icp_x")) is not None
        and parse_float(row.get("icp_y")) is not None
        and parse_float(row.get("icp_yaw", row.get("icp_heading"))) is not None
    ]
    if require_real_tacho:
        valid_rows = [row for row in valid_rows if parse_bool(row.get("has_real_tacho_research"))]
    if len(valid_rows) < 3:
        return []
    series = RowSeries(valid_rows)
    errors: list[dict[str, Any]] = []
    last_start_t = -1e99
    for start_idx, start in enumerate(valid_rows):
        t0 = float(start["t"])
        if t0 - last_start_t < sample_period_s:
            continue
        last_start_t = t0
        for horizon in horizons_s:
            target_t = t0 + horizon
            end_idx = series.nearest_index(target_t)
            end = valid_rows[end_idx]
            if abs(float(end["t"]) - target_t) > 0.25 or end_idx <= start_idx:
                continue
            pred_x, pred_y, pred_yaw = simulate_window(valid_rows, start_idx, end_idx, params)
            ex, ey, etheta = se2_log_error(
                pred_x,
                pred_y,
                pred_yaw,
                float(end["icp_x"]),
                float(end["icp_y"]),
                float(end.get("icp_yaw", end.get("icp_heading"))),
            )
            errors.append(
                {
                    "model": params.name,
                    "session": start.get("session", ""),
                    "segment_id": start.get("segment_id", ""),
                    "motion_class": start.get("motion_class", ""),
                    "quality_class": start.get("quality_class", "pass" if parse_bool(start.get("research_quality_ok")) else "weak_icp"),
                    "start_t": t0,
                    "end_t": float(end["t"]),
                    "horizon_s": horizon,
                    "error_x_body_m": ex,
                    "error_y_body_m": ey,
                    "position_error_m": math.hypot(ex, ey),
                    "yaw_error_rad": etheta,
                    "max_abs_phi_rad": max(abs(parse_float(r.get("phi_rad")) or 0.0) for r in valid_rows[start_idx : end_idx + 1]),
                    "max_abs_roll_rad": max(abs(parse_float(r.get("imu_roll_rad")) or 0.0) for r in valid_rows[start_idx : end_idx + 1]),
                    "max_abs_pitch_rad": max(abs(parse_float(r.get("imu_pitch_rad")) or 0.0) for r in valid_rows[start_idx : end_idx + 1]),
                    "max_abs_accel_ms2": max(abs(parse_float(r.get("tacho_accel_ms2")) or 0.0) for r in valid_rows[start_idx : end_idx + 1]),
                }
            )
    return errors


def summarize_errors(errors: list[dict[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {"rows": len(errors), "by_horizon_s": {}, "by_motion_class": {}}
    for horizon in sorted({float(row["horizon_s"]) for row in errors}):
        rows_h = [row for row in errors if abs(float(row["horizon_s"]) - horizon) < 1e-9]
        out["by_horizon_s"][horizon] = {
            "samples": len(rows_h),
            "position_m": finite_stats(float(row["position_error_m"]) for row in rows_h),
            "yaw_rad": finite_stats(abs(float(row["yaw_error_rad"])) for row in rows_h),
        }
    for klass in sorted({str(row.get("motion_class", "")) for row in errors}):
        rows_k = [row for row in errors if str(row.get("motion_class", "")) == klass]
        out["by_motion_class"][klass] = {
            "samples": len(rows_k),
            "position_m": finite_stats(float(row["position_error_m"]) for row in rows_k),
            "yaw_rad": finite_stats(abs(float(row["yaw_error_rad"])) for row in rows_k),
        }
    return out
