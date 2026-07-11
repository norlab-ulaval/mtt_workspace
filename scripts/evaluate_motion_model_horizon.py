#!/usr/bin/env python3
"""Evaluate short-horizon MTT motion-model prediction errors against ICP."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Any

import yaml


L_F_M = 0.9
L_R_M = 1.5
WHEELBASE_M = L_F_M + L_R_M
MAX_ARTICULATION_RAD = math.radians(60.0)
MIN_TURN_SPEED_MS = 0.1
SLIP_BASE = 0.10
SLIP_SPEED_GAIN = 0.05
SLIP_ARTICULATION_GAIN = 0.15
SLIP_MIN_SCALE = 0.55


def parse_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def tach_direction_sign(direction: Any) -> float:
    text = str(direction or "").strip().lower()
    return -1.0 if text in {"reverse", "backward", "rev", "-1"} else 1.0


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def nominal_curvature(articulation_rad: float, model: str) -> float:
    phi = clamp(articulation_rad, -MAX_ARTICULATION_RAD, MAX_ARTICULATION_RAD)
    if model == "tangent":
        return math.tan(phi) / WHEELBASE_M
    return math.sin(phi) / (L_F_M * math.cos(phi) + L_R_M)


def slip_scale(speed_ms: float, articulation_rad: float) -> float:
    normalized_articulation = abs(articulation_rad) / max(MAX_ARTICULATION_RAD, 1e-9)
    scale = 1.0 - SLIP_BASE - SLIP_SPEED_GAIN * abs(speed_ms) - SLIP_ARTICULATION_GAIN * normalized_articulation
    return clamp(scale, SLIP_MIN_SCALE, 1.0)


def select_speed(row: dict[str, Any], source: str) -> float | None:
    if source == "tach":
        speed = parse_float(row.get("tach_speed_ms"))
        return speed * tach_direction_sign(row.get("tach_direction")) if speed is not None else None
    if source == "status":
        return parse_float(row.get("status_effective_linear_speed_command_ms"))
    if source == "cmd":
        return parse_float(row.get("cmd_linear_x"))
    raise ValueError(f"Unsupported speed source: {source}")


def select_articulation(row: dict[str, Any], source: str) -> float | None:
    if source == "measured":
        return parse_float(row.get("articulation_rad"))
    if source == "status":
        steer = parse_float(row.get("status_steer_normalized"))
    elif source == "cmd":
        steer = parse_float(row.get("cmd_angular_z"))
    else:
        raise ValueError(f"Unsupported articulation source: {source}")
    return clamp(steer, -1.0, 1.0) * MAX_ARTICULATION_RAD if steer is not None else None


def model_speed_yaw_rate(
    row: dict[str, Any],
    *,
    speed_source: str,
    articulation_source: str,
    curvature_model: str,
    yaw_gain: float = 1.0,
) -> tuple[float | None, float | None]:
    signed_speed = select_speed(row, speed_source)
    articulation = select_articulation(row, articulation_source)
    if signed_speed is None or articulation is None:
        return None, None
    yaw_rate = 0.0
    if abs(signed_speed) >= MIN_TURN_SPEED_MS:
        yaw_rate = yaw_gain * signed_speed * nominal_curvature(articulation, curvature_model)
    return signed_speed, yaw_rate


def model_inputs(
    row: dict[str, Any],
    *,
    speed_source: str,
    articulation_source: str,
) -> tuple[float | None, float | None]:
    return select_speed(row, speed_source), select_articulation(row, articulation_source)


def read_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    numeric = {
        "t",
        "icp_x",
        "icp_y",
        "icp_heading",
        "icp_linear_x",
        "icp_angular_z",
        "odom_x",
        "odom_y",
        "odom_heading",
        "mtt_articulation_rad",
        "hardware_articulation_rad",
        "trailer_articulation_rad",
        "tach_speed_ms",
        "articulation_rad",
        "status_effective_linear_speed_command_ms",
        "status_steer_normalized",
        "cmd_linear_x",
        "cmd_angular_z",
    }
    with path.open("r", encoding="utf-8", newline="") as stream:
        for raw in csv.DictReader(stream):
            row: dict[str, Any] = dict(raw)
            for key in numeric:
                value = parse_float(raw.get(key))
                if value is not None:
                    row[key] = value
            if all(parse_float(row.get(k)) is not None for k in ("t", "icp_x", "icp_y", "icp_heading")):
                rows.append(row)
    rows.sort(key=lambda row: float(row["t"]))
    return rows


def split_valid_segments(
    rows: list[dict[str, Any]],
    *,
    max_icp_step_m: float,
    max_icp_gap_s: float,
    max_icp_speed_ms: float,
    max_icp_yaw_rate_rad_s: float,
    speed_source: str,
    articulation_source: str,
    curvature_model: str,
) -> tuple[list[list[dict[str, Any]]], dict[str, int]]:
    segments: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    prev: dict[str, Any] | None = None
    rejected = {"step_or_gap": 0, "speed": 0, "yaw_rate": 0, "missing_model": 0}
    for row in rows:
        good = True
        if prev is not None:
            dt = float(row["t"]) - float(prev["t"])
            step = math.hypot(float(row["icp_x"]) - float(prev["icp_x"]), float(row["icp_y"]) - float(prev["icp_y"]))
            if dt > max_icp_gap_s or step > max_icp_step_m:
                good = False
                rejected["step_or_gap"] += 1
        icp_v = parse_float(row.get("icp_linear_x"))
        icp_w = parse_float(row.get("icp_angular_z"))
        if icp_v is not None and abs(icp_v) > max_icp_speed_ms:
            good = False
            rejected["speed"] += 1
        if icp_w is not None and abs(icp_w) > max_icp_yaw_rate_rad_s:
            good = False
            rejected["yaw_rate"] += 1
        speed, yaw_rate = model_speed_yaw_rate(
            row,
            speed_source=speed_source,
            articulation_source=articulation_source,
            curvature_model=curvature_model,
        )
        if speed is None or yaw_rate is None:
            good = False
            rejected["missing_model"] += 1
        if good:
            current.append(row)
        else:
            if current:
                segments.append(current)
            current = []
        prev = row
    if current:
        segments.append(current)
    return segments, rejected


def nearest_index(times: list[float], target: float) -> int:
    idx = bisect.bisect_left(times, target)
    if idx <= 0:
        return 0
    if idx >= len(times):
        return len(times) - 1
    return idx if abs(times[idx] - target) < abs(times[idx - 1] - target) else idx - 1


def integrate_prediction(
    segment: list[dict[str, Any]],
    start_idx: int,
    end_idx: int,
    *,
    speed_source: str,
    articulation_source: str,
    curvature_model: str,
    model_kind: str,
    tau_v_s: float,
    tau_r_s: float,
    tau_phi_s: float,
) -> tuple[float, float, float]:
    x = float(segment[start_idx]["icp_x"])
    y = float(segment[start_idx]["icp_y"])
    yaw = float(segment[start_idx]["icp_heading"])
    prev_t = float(segment[start_idx]["t"])

    start_speed, start_articulation = model_inputs(
        segment[start_idx],
        speed_source=speed_source,
        articulation_source=articulation_source,
    )
    v_state = start_speed if start_speed is not None else 0.0
    phi_state = start_articulation if start_articulation is not None else 0.0
    r_state = v_state * nominal_curvature(phi_state, curvature_model) if abs(v_state) >= MIN_TURN_SPEED_MS else 0.0

    for row in segment[start_idx + 1 : end_idx + 1]:
        t = float(row["t"])
        dt = max(0.0, min(t - prev_t, 1.0))
        speed_cmd, articulation_cmd = model_inputs(
            row,
            speed_source=speed_source,
            articulation_source=articulation_source,
        )
        if speed_cmd is None or articulation_cmd is None or dt <= 0.0:
            prev_t = t
            continue

        if model_kind == "direct":
            speed = speed_cmd
            curvature = nominal_curvature(articulation_cmd, curvature_model)
            yaw_rate = speed * curvature if abs(speed) >= MIN_TURN_SPEED_MS else 0.0
        elif model_kind == "slip_heuristic":
            speed = speed_cmd
            curvature = nominal_curvature(articulation_cmd, curvature_model)
            curvature *= slip_scale(speed, articulation_cmd)
            yaw_rate = speed * curvature if abs(speed) >= MIN_TURN_SPEED_MS else 0.0
        elif model_kind == "m1_phi_lag":
            alpha_v = clamp(dt / max(tau_v_s, 1e-6), 0.0, 1.0)
            alpha_phi = clamp(dt / max(tau_phi_s, 1e-6), 0.0, 1.0)
            v_state += (speed_cmd - v_state) * alpha_v
            phi_state += (articulation_cmd - phi_state) * alpha_phi
            phi_state = clamp(phi_state, -MAX_ARTICULATION_RAD, MAX_ARTICULATION_RAD)
            speed = v_state
            yaw_rate = speed * nominal_curvature(phi_state, curvature_model) if abs(speed) >= MIN_TURN_SPEED_MS else 0.0
        elif model_kind == "m1_yaw_lag":
            alpha_v = clamp(dt / max(tau_v_s, 1e-6), 0.0, 1.0)
            alpha_r = clamp(dt / max(tau_r_s, 1e-6), 0.0, 1.0)
            v_state += (speed_cmd - v_state) * alpha_v
            target_r = (
                v_state * nominal_curvature(articulation_cmd, curvature_model)
                if abs(v_state) >= MIN_TURN_SPEED_MS
                else 0.0
            )
            r_state += (target_r - r_state) * alpha_r
            speed = v_state
            yaw_rate = r_state
        else:
            raise ValueError(f"Unsupported model kind: {model_kind}")

        if dt > 0.0:
            dtheta = yaw_rate * dt
            heading_mid = yaw + 0.5 * dtheta
            x += speed * dt * math.cos(heading_mid)
            y += speed * dt * math.sin(heading_mid)
            yaw = wrap_angle(yaw + dtheta)
        prev_t = t
    return x, y, yaw


def integrate_segment_rollout(
    segment: list[dict[str, Any]],
    *,
    segment_id: int,
    speed_source: str,
    articulation_source: str,
    curvature_model: str,
    model_kind: str,
    tau_v_s: float,
    tau_r_s: float,
    tau_phi_s: float,
) -> list[dict[str, float]]:
    if not segment:
        return []

    x = float(segment[0]["icp_x"])
    y = float(segment[0]["icp_y"])
    yaw = float(segment[0]["icp_heading"])
    prev_t = float(segment[0]["t"])
    start_speed, start_articulation = model_inputs(
        segment[0],
        speed_source=speed_source,
        articulation_source=articulation_source,
    )
    v_state = start_speed if start_speed is not None else 0.0
    phi_state = start_articulation if start_articulation is not None else 0.0
    r_state = v_state * nominal_curvature(phi_state, curvature_model) if abs(v_state) >= MIN_TURN_SPEED_MS else 0.0

    traces: list[dict[str, float]] = []
    for row in segment:
        t = float(row["t"])
        dt = max(0.0, min(t - prev_t, 1.0))
        speed_cmd, articulation_cmd = model_inputs(
            row,
            speed_source=speed_source,
            articulation_source=articulation_source,
        )
        speed = 0.0
        yaw_rate = 0.0
        if speed_cmd is not None and articulation_cmd is not None:
            if model_kind == "direct":
                speed = speed_cmd
                curvature = nominal_curvature(articulation_cmd, curvature_model)
                yaw_rate = speed * curvature if abs(speed) >= MIN_TURN_SPEED_MS else 0.0
            elif model_kind == "slip_heuristic":
                speed = speed_cmd
                curvature = nominal_curvature(articulation_cmd, curvature_model)
                curvature *= slip_scale(speed, articulation_cmd)
                yaw_rate = speed * curvature if abs(speed) >= MIN_TURN_SPEED_MS else 0.0
            elif model_kind == "m1_phi_lag":
                alpha_v = clamp(dt / max(tau_v_s, 1e-6), 0.0, 1.0)
                alpha_phi = clamp(dt / max(tau_phi_s, 1e-6), 0.0, 1.0)
                v_state += (speed_cmd - v_state) * alpha_v
                phi_state += (articulation_cmd - phi_state) * alpha_phi
                phi_state = clamp(phi_state, -MAX_ARTICULATION_RAD, MAX_ARTICULATION_RAD)
                speed = v_state
                yaw_rate = speed * nominal_curvature(phi_state, curvature_model) if abs(speed) >= MIN_TURN_SPEED_MS else 0.0
            elif model_kind == "m1_yaw_lag":
                alpha_v = clamp(dt / max(tau_v_s, 1e-6), 0.0, 1.0)
                alpha_r = clamp(dt / max(tau_r_s, 1e-6), 0.0, 1.0)
                v_state += (speed_cmd - v_state) * alpha_v
                target_r = (
                    v_state * nominal_curvature(articulation_cmd, curvature_model)
                    if abs(v_state) >= MIN_TURN_SPEED_MS
                    else 0.0
                )
                r_state += (target_r - r_state) * alpha_r
                speed = v_state
                yaw_rate = r_state
            else:
                raise ValueError(f"Unsupported model kind: {model_kind}")

        if dt > 0.0:
            dtheta = yaw_rate * dt
            heading_mid = yaw + 0.5 * dtheta
            x += speed * dt * math.cos(heading_mid)
            y += speed * dt * math.sin(heading_mid)
            yaw = wrap_angle(yaw + dtheta)
        traces.append(
            {
                "segment": float(segment_id),
                "t": t,
                "icp_x": float(row["icp_x"]),
                "icp_y": float(row["icp_y"]),
                "icp_heading": float(row["icp_heading"]),
                "model_x": x,
                "model_y": y,
                "model_heading": yaw,
                "model_speed_ms": speed,
                "model_yaw_rate_rad_s": yaw_rate,
            }
        )
        prev_t = t
    return traces


def global_model_rollout(
    segments: list[list[dict[str, Any]]],
    *,
    speed_source: str,
    articulation_source: str,
    curvature_model: str,
    model_kind: str,
    tau_v_s: float,
    tau_r_s: float,
    tau_phi_s: float,
) -> list[dict[str, float]]:
    traces: list[dict[str, float]] = []
    for segment_id, segment in enumerate(segments):
        traces.extend(
            integrate_segment_rollout(
                segment,
                segment_id=segment_id,
                speed_source=speed_source,
                articulation_source=articulation_source,
                curvature_model=curvature_model,
                model_kind=model_kind,
                tau_v_s=tau_v_s,
                tau_r_s=tau_r_s,
                tau_phi_s=tau_phi_s,
            )
        )
    return traces


def aligned_xy_to_icp(
    rows: list[dict[str, Any]],
    *,
    x_key: str,
    y_key: str,
    yaw_key: str,
) -> tuple[list[float], list[float], list[float], list[float]]:
    anchor: tuple[float, float, float, float, float, float] | None = None
    for row in rows:
        ix = parse_float(row.get("icp_x"))
        iy = parse_float(row.get("icp_y"))
        ih = parse_float(row.get("icp_heading"))
        ox = parse_float(row.get(x_key))
        oy = parse_float(row.get(y_key))
        oh = parse_float(row.get(yaw_key))
        if None not in (ix, iy, ih, ox, oy, oh):
            assert ix is not None and iy is not None and ih is not None
            assert ox is not None and oy is not None and oh is not None
            anchor = (ix, iy, ih, ox, oy, oh)
            break
    if anchor is None:
        return [], [], [], []

    ix0, iy0, ih0, ox0, oy0, oh0 = anchor
    dyaw = wrap_angle(ih0 - oh0)
    c = math.cos(dyaw)
    s = math.sin(dyaw)
    icp_xs: list[float] = []
    icp_ys: list[float] = []
    aligned_xs: list[float] = []
    aligned_ys: list[float] = []
    for row in rows:
        ix = parse_float(row.get("icp_x"))
        iy = parse_float(row.get("icp_y"))
        ox = parse_float(row.get(x_key))
        oy = parse_float(row.get(y_key))
        if None in (ix, iy, ox, oy):
            continue
        assert ix is not None and iy is not None and ox is not None and oy is not None
        dx = ox - ox0
        dy = oy - oy0
        icp_xs.append(ix)
        icp_ys.append(iy)
        aligned_xs.append(ix0 + c * dx - s * dy)
        aligned_ys.append(iy0 + s * dx + c * dy)
    return icp_xs, icp_ys, aligned_xs, aligned_ys


def summarize(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "mae": None, "rmse": None, "median_abs": None, "p95_abs": None, "max_abs": None}
    abs_values = sorted(abs(v) for v in values)
    p95_idx = min(len(abs_values) - 1, max(0, int(round(0.95 * (len(abs_values) - 1)))))
    median_idx = len(abs_values) // 2
    return {
        "count": len(values),
        "mean": sum(values) / len(values),
        "mae": sum(abs(v) for v in values) / len(values),
        "rmse": math.sqrt(sum(v * v for v in values) / len(values)),
        "median_abs": abs_values[median_idx],
        "p95_abs": abs_values[p95_idx],
        "max_abs": abs_values[-1],
    }


def write_plots(out_dir: Path, errors: list[dict[str, Any]], summary: dict[str, Any], prefix: str) -> dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}

    outputs: dict[str, str] = {}
    horizons = sorted(summary["by_horizon_s"])
    pos_rmse = [summary["by_horizon_s"][h]["position_m"]["rmse"] or 0.0 for h in horizons]
    pos_p95 = [summary["by_horizon_s"][h]["position_m"]["p95_abs"] or 0.0 for h in horizons]
    yaw_rmse = [summary["by_horizon_s"][h]["yaw_rad"]["rmse"] or 0.0 for h in horizons]

    plt.figure(figsize=(9, 5))
    plt.plot(horizons, pos_rmse, marker="o", label="position RMSE")
    plt.plot(horizons, pos_p95, marker="o", label="position P95")
    plt.xlabel("prediction horizon [s]")
    plt.ylabel("position error [m]")
    plt.title("M0/M1 short-horizon position prediction")
    plt.grid(True, alpha=0.4)
    plt.legend()
    plt.tight_layout()
    path = out_dir / f"{prefix}_position_error.png"
    plt.savefig(path, dpi=140)
    plt.close()
    outputs["position_error"] = str(path)

    plt.figure(figsize=(9, 5))
    plt.plot(horizons, yaw_rmse, marker="o", label="yaw RMSE")
    plt.xlabel("prediction horizon [s]")
    plt.ylabel("yaw error [rad]")
    plt.title("M0/M1 short-horizon yaw prediction")
    plt.grid(True, alpha=0.4)
    plt.legend()
    plt.tight_layout()
    path = out_dir / f"{prefix}_yaw_error.png"
    plt.savefig(path, dpi=140)
    plt.close()
    outputs["yaw_error"] = str(path)

    target_h = 3.0 if 3.0 in horizons else horizons[min(len(horizons) - 1, 2)]
    rows = [row for row in errors if abs(float(row["horizon_s"]) - target_h) < 1e-6]
    if rows:
        plt.figure(figsize=(11, 5))
        plt.scatter([float(r["start_offset_s"]) for r in rows], [float(r["position_error_m"]) for r in rows], s=8)
        plt.xlabel("start time [s]")
        plt.ylabel(f"{target_h:.1f}s position error [m]")
        plt.title("Short-horizon prediction error over time")
        plt.grid(True, alpha=0.4)
        plt.tight_layout()
        path = out_dir / f"{prefix}_error_timeline.png"
        plt.savefig(path, dpi=140)
        plt.close()
        outputs["error_timeline"] = str(path)

        plt.figure(figsize=(10, 5))
        plt.scatter([float(r["max_speed_ms"]) for r in rows], [float(r["position_error_m"]) for r in rows], s=8, label="speed")
        plt.xlabel("max |speed| in horizon [m/s]")
        plt.ylabel(f"{target_h:.1f}s position error [m]")
        plt.title("Short-horizon error vs speed")
        plt.grid(True, alpha=0.4)
        plt.tight_layout()
        path = out_dir / f"{prefix}_error_vs_speed.png"
        plt.savefig(path, dpi=140)
        plt.close()
        outputs["error_vs_speed"] = str(path)

        plt.figure(figsize=(10, 5))
        plt.scatter([float(r["max_articulation_rad"]) for r in rows], [float(r["position_error_m"]) for r in rows], s=8)
        plt.xlabel("max |articulation| in horizon [rad]")
        plt.ylabel(f"{target_h:.1f}s position error [m]")
        plt.title("Short-horizon error vs articulation")
        plt.grid(True, alpha=0.4)
        plt.tight_layout()
        path = out_dir / f"{prefix}_error_vs_articulation.png"
        plt.savefig(path, dpi=140)
        plt.close()
        outputs["error_vs_articulation"] = str(path)

    return outputs


def write_global_plots(
    out_dir: Path,
    rows: list[dict[str, Any]],
    segments: list[list[dict[str, Any]]],
    *,
    prefix: str,
    speed_source: str,
    articulation_source: str,
    curvature_model: str,
    model_kind: str,
    tau_v_s: float,
    tau_r_s: float,
    tau_phi_s: float,
) -> dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}

    outputs: dict[str, str] = {}
    traces = global_model_rollout(
        segments,
        speed_source=speed_source,
        articulation_source=articulation_source,
        curvature_model=curvature_model,
        model_kind=model_kind,
        tau_v_s=tau_v_s,
        tau_r_s=tau_r_s,
        tau_phi_s=tau_phi_s,
    )

    if traces:
        plt.figure(figsize=(10, 8))
        traces_by_segment: dict[int, list[dict[str, float]]] = {}
        for trace in traces:
            traces_by_segment.setdefault(int(trace["segment"]), []).append(trace)
        for segment_id, segment_rows in sorted(traces_by_segment.items()):
            label_icp = "ICP" if segment_id == 0 else None
            label_model = "motion model" if segment_id == 0 else None
            plt.plot(
                [trace["icp_x"] for trace in segment_rows],
                [trace["icp_y"] for trace in segment_rows],
                color="tab:orange",
                linewidth=1.2,
                alpha=0.85,
                label=label_icp,
            )
            plt.plot(
                [trace["model_x"] for trace in segment_rows],
                [trace["model_y"] for trace in segment_rows],
                color="tab:green",
                linewidth=1.1,
                alpha=0.85,
                label=label_model,
            )
        plt.plot(traces[0]["icp_x"], traces[0]["icp_y"], "ko", markersize=5, label="start")
        plt.axis("equal")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title(f"{prefix} global ICP vs motion-model rollout")
        plt.grid(True, alpha=0.4)
        plt.legend()
        plt.tight_layout()
        path = out_dir / f"{prefix}_global_icp_vs_motion_model.png"
        plt.savefig(path, dpi=150)
        plt.close()
        outputs["global_icp_vs_motion_model"] = str(path)

    icp_xs, icp_ys, odom_xs, odom_ys = aligned_xy_to_icp(rows, x_key="odom_x", y_key="odom_y", yaw_key="odom_heading")
    if icp_xs and odom_xs:
        plt.figure(figsize=(10, 8))
        plt.plot(icp_xs, icp_ys, label="ICP", color="tab:orange", linewidth=1.3)
        plt.plot(odom_xs, odom_ys, label="MTT odom aligned to ICP start", color="tab:blue", linewidth=1.15)
        plt.plot(icp_xs[0], icp_ys[0], "ko", markersize=5, label="start")
        plt.axis("equal")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title(f"{prefix} global MTT odom vs ICP")
        plt.grid(True, alpha=0.4)
        plt.legend()
        plt.tight_layout()
        path = out_dir / f"{prefix}_global_mtt_odom_vs_icp.png"
        plt.savefig(path, dpi=150)
        plt.close()
        outputs["global_mtt_odom_vs_icp"] = str(path)

    return outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--horizons", default="0.5,1,2,3,5,10")
    parser.add_argument("--sample-period-s", type=float, default=0.5)
    parser.add_argument("--max-icp-step-m", type=float, default=1.0)
    parser.add_argument("--max-icp-gap-s", type=float, default=0.5)
    parser.add_argument("--max-icp-speed-ms", type=float, default=8.0)
    parser.add_argument("--max-icp-yaw-rate-rad-s", type=float, default=2.5)
    parser.add_argument("--speed-source", choices=["tach", "status", "cmd"], default="tach")
    parser.add_argument("--articulation-source", choices=["measured", "status", "cmd"], default="measured")
    parser.add_argument("--curvature-model", choices=["exact", "tangent"], default="exact")
    parser.add_argument(
        "--model-kind",
        choices=["direct", "m1_yaw_lag", "m1_phi_lag", "slip_heuristic"],
        default="direct",
    )
    parser.add_argument("--tau-v-s", type=float, default=0.2)
    parser.add_argument("--tau-r-s", type=float, default=0.2)
    parser.add_argument("--tau-phi-s", type=float, default=0.2)
    parser.add_argument("--output-prefix", default="horizon")
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = args.session_dir.expanduser()
    input_dir = session / "motion_model_validation"
    out_dir = args.output_dir.expanduser() if args.output_dir else input_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = read_rows(input_dir / "aligned_samples.csv")
    horizons = [float(item) for item in args.horizons.split(",") if item.strip()]
    segments, rejected = split_valid_segments(
        rows,
        max_icp_step_m=args.max_icp_step_m,
        max_icp_gap_s=args.max_icp_gap_s,
        max_icp_speed_ms=args.max_icp_speed_ms,
        max_icp_yaw_rate_rad_s=args.max_icp_yaw_rate_rad_s,
        speed_source=args.speed_source,
        articulation_source=args.articulation_source,
        curvature_model=args.curvature_model,
    )

    errors: list[dict[str, Any]] = []
    t0 = float(rows[0]["t"]) if rows else 0.0
    for segment_id, segment in enumerate(segments):
        if len(segment) < 2:
            continue
        times = [float(row["t"]) for row in segment]
        next_start_time = times[0]
        for i, row in enumerate(segment):
            if times[i] + min(horizons) > times[-1]:
                break
            if times[i] + 1e-9 < next_start_time:
                continue
            next_start_time = times[i] + args.sample_period_s
            for horizon in horizons:
                target = times[i] + horizon
                if target > times[-1]:
                    continue
                j = nearest_index(times, target)
                if j <= i:
                    continue
                px, py, pyaw = integrate_prediction(
                    segment,
                    i,
                    j,
                    speed_source=args.speed_source,
                    articulation_source=args.articulation_source,
                    curvature_model=args.curvature_model,
                    model_kind=args.model_kind,
                    tau_v_s=args.tau_v_s,
                    tau_r_s=args.tau_r_s,
                    tau_phi_s=args.tau_phi_s,
                )
                tx = float(segment[j]["icp_x"])
                ty = float(segment[j]["icp_y"])
                tyaw = float(segment[j]["icp_heading"])
                max_speed = 0.0
                max_art = 0.0
                for local in segment[i : j + 1]:
                    speed, _ = model_speed_yaw_rate(
                        local,
                        speed_source=args.speed_source,
                        articulation_source=args.articulation_source,
                        curvature_model=args.curvature_model,
                    )
                    art = select_articulation(local, args.articulation_source)
                    if speed is not None:
                        max_speed = max(max_speed, abs(speed))
                    if art is not None:
                        max_art = max(max_art, abs(art))
                errors.append(
                    {
                        "segment": segment_id,
                        "start_offset_s": times[i] - t0,
                        "horizon_s": horizon,
                        "actual_dt_s": times[j] - times[i],
                        "position_error_m": math.hypot(px - tx, py - ty),
                        "yaw_error_rad": wrap_angle(pyaw - tyaw),
                        "max_speed_ms": max_speed,
                        "max_articulation_rad": max_art,
                        "start_icp_x": float(row["icp_x"]),
                        "start_icp_y": float(row["icp_y"]),
                    }
                )

    by_horizon: dict[float, dict[str, Any]] = {}
    for horizon in horizons:
        rows_h = [row for row in errors if abs(float(row["horizon_s"]) - horizon) < 1e-6]
        by_horizon[horizon] = {
            "samples": len(rows_h),
            "position_m": summarize([float(row["position_error_m"]) for row in rows_h]),
            "yaw_rad": summarize([float(row["yaw_error_rad"]) for row in rows_h]),
        }

    out_csv = out_dir / f"{args.output_prefix}_prediction_errors.csv"
    if errors:
        with out_csv.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(errors[0].keys()))
            writer.writeheader()
            writer.writerows(errors)

    summary = {
        "session": session.name,
        "settings": {
            "horizons_s": horizons,
            "sample_period_s": args.sample_period_s,
            "max_icp_step_m": args.max_icp_step_m,
            "max_icp_gap_s": args.max_icp_gap_s,
            "max_icp_speed_ms": args.max_icp_speed_ms,
            "max_icp_yaw_rate_rad_s": args.max_icp_yaw_rate_rad_s,
            "speed_source": args.speed_source,
            "articulation_source": args.articulation_source,
            "curvature_model": args.curvature_model,
            "model_kind": args.model_kind,
            "tau_v_s": args.tau_v_s,
            "tau_r_s": args.tau_r_s,
            "tau_phi_s": args.tau_phi_s,
            "slip_heuristic": {
                "base": SLIP_BASE,
                "speed_gain": SLIP_SPEED_GAIN,
                "articulation_gain": SLIP_ARTICULATION_GAIN,
                "min_scale": SLIP_MIN_SCALE,
            },
            "geometry": {
                "L_f_m": L_F_M,
                "L_r_m": L_R_M,
                "max_articulation_rad": MAX_ARTICULATION_RAD,
            },
        },
        "input_rows": len(rows),
        "segments": len(segments),
        "rejected": rejected,
        "prediction_rows": len(errors),
        "by_horizon_s": by_horizon,
        "outputs": {
            "csv": str(out_csv),
        },
    }
    summary["outputs"].update(write_plots(out_dir, errors, summary, args.output_prefix))
    summary["outputs"].update(
        write_global_plots(
            out_dir,
            rows,
            segments,
            prefix=args.output_prefix,
            speed_source=args.speed_source,
            articulation_source=args.articulation_source,
            curvature_model=args.curvature_model,
            model_kind=args.model_kind,
            tau_v_s=args.tau_v_s,
            tau_r_s=args.tau_r_s,
            tau_phi_s=args.tau_phi_s,
        )
    )
    out_summary = out_dir / f"{args.output_prefix}_prediction_summary.yaml"
    summary["outputs"]["summary"] = str(out_summary)
    out_summary.write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
