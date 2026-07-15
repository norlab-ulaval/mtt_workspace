#!/usr/bin/env python3
"""M5 contribution diagnostics: tune, compare, and plot real-bag evidence."""

from __future__ import annotations

import argparse
import bisect
import csv
import itertools
import math
import sys
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


L1_M = 0.9
L2_M = 1.5
L_M = L1_M + L2_M
MAX_ARTICULATION_RAD = math.radians(60.0)
MOVING_MIN_DISTANCE_M = 0.5
MOVING_MIN_YAW_RAD = math.radians(5.0)
MIN_TURN_SPEED_MS = 0.1


MODEL_ORDER = [
    "odom_delta",
    "M0_measured_exact",
    "M3_measured_phi_delay",
    "M5_pointfit",
    "M5_rpe_tuned",
    "M0_command_exact",
    "M3_command_curvature_residual",
]


LABELS = {
    "odom_delta": "odom\nbase",
    "M0_measured_exact": "M0\nmeas",
    "M3_measured_phi_delay": "M3\nmeas",
    "M5_pointfit": "M5\npoint",
    "M5_rpe_tuned": "M5\nRPE",
    "M0_command_exact": "M0\ncmd",
    "M3_command_curvature_residual": "M3\ncmd",
}


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


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def dz(value: float, threshold: float) -> float:
    return math.copysign(max(abs(value) - threshold, 0.0), value) if value != 0.0 else 0.0


def kappa_nom(phi: float) -> float:
    phi_c = clamp(phi, -MAX_ARTICULATION_RAD, MAX_ARTICULATION_RAD)
    return math.sin(phi_c) / (L1_M * math.cos(phi_c) + L2_M)


def kappa_m5(phi: float, gamma_slope: float, phi_dead_rad: float, phi_bias_rad: float) -> float:
    return (gamma_slope / L_M) * dz(phi - phi_bias_rad, phi_dead_rad)


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
        return {"count": 0, "mean": None, "median": None, "p95": None, "rmse": None}
    return {
        "count": len(vals),
        "mean": statistics.mean(vals),
        "median": statistics.median(vals),
        "p95": percentile(vals, 95.0),
        "rmse": rmse(vals),
    }


@dataclass(frozen=True)
class M5Params:
    name: str
    gamma_slope: float
    phi_dead_rad: float
    phi_bias_rad: float
    phi_delay_s: float = 0.0
    speed_scale: float = 1.0
    objective: str = ""

    @property
    def c2_eff_m2(self) -> float:
        return L_M * L_M * (1.0 / max(self.gamma_slope, 1e-9) - 1.0)


class Series:
    def __init__(self, rows: list[dict[str, Any]]):
        self.rows = rows
        self.times = [float(row["t"]) for row in rows]

    def nearest(self, t: float, tolerance_s: float) -> dict[str, Any] | None:
        idx = bisect.bisect_left(self.times, t)
        candidates = []
        if idx < len(self.rows):
            candidates.append(self.rows[idx])
        if idx:
            candidates.append(self.rows[idx - 1])
        if not candidates:
            return None
        best = min(candidates, key=lambda row: abs(float(row["t"]) - t))
        return best if abs(float(best["t"]) - t) <= tolerance_s else None


def read_csv(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for raw in csv.DictReader(stream):
            row: dict[str, Any] = dict(raw)
            for key, value in raw.items():
                parsed = parse_float(value)
                if parsed is not None:
                    row[key] = parsed
            rows.append(row)
    return rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = sorted({key for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def dataset_csv_for_session(session: Path) -> Path:
    candidates = [
        session / "motion_model_validation" / "motion_research" / "datasets" / session.name / "dataset.csv",
        session / "motion_research" / "dataset.csv",
        session / "postprocess_dataset" / "motion_model_dataset.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"no motion dataset found under {session}")


def calibrated_errors_csv(session: Path) -> Path:
    path = session / "motion_model_validation" / "motion_research" / "calibrated_models" / "calibrated_prediction_errors_moving_windows.csv"
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def good_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out = [row for row in rows if parse_bool(row.get("research_quality_ok"))]
    out.sort(key=lambda row: (str(row.get("session", "")), str(row.get("segment_id", "")), float(row.get("t", 0.0))))
    return out


def grouped_segments(rows: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault((str(row.get("session", "")), str(row.get("segment_id", ""))), []).append(row)
    return [sorted(group, key=lambda row: float(row["t"])) for _, group in sorted(groups.items())]


def grouped_series(rows: list[dict[str, Any]]) -> dict[str, Series]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault(str(row.get("session", "")), []).append(row)
    return {session: Series(sorted(group, key=lambda row: float(row["t"]))) for session, group in groups.items()}


def pose(row: dict[str, Any], reference: str) -> tuple[float, float, float] | None:
    vals = [parse_float(row.get(f"{reference}_x")), parse_float(row.get(f"{reference}_y")), parse_float(row.get(f"{reference}_yaw"))]
    if any(value is None for value in vals):
        return None
    assert vals[0] is not None and vals[1] is not None and vals[2] is not None
    return vals[0], vals[1], vals[2]


def se2_error(pred: tuple[float, float, float], ref: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = pred[0] - ref[0]
    dy = pred[1] - ref[1]
    c = math.cos(ref[2])
    s = math.sin(ref[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap_angle(pred[2] - ref[2])


def measured_speed(row: dict[str, Any]) -> float:
    return parse_float(row.get("tacho_signed_ms")) or 0.0


def measured_phi(row: dict[str, Any]) -> float:
    return parse_float(row.get("phi_rad")) or parse_float(row.get("mtt_articulation_angle")) or 0.0


def curvature_samples(rows: list[dict[str, Any]], reference: str) -> list[dict[str, float]]:
    samples = []
    for row in good_rows(rows):
        v = measured_speed(row)
        w = parse_float(row.get(f"{reference}_yaw_rate_rad_s_derived")) or parse_float(row.get(f"{reference}_yaw_rate_rad_s"))
        phi = measured_phi(row)
        if w is None or abs(v) < 0.25:
            continue
        kappa = w / v
        if abs(kappa) > 2.5:
            continue
        samples.append(
            {
                "phi": phi,
                "kappa": kappa,
                "speed": v,
                "phidot": parse_float(row.get("phi_rate_rad_s")) or 0.0,
                "motion_class": str(row.get("motion_class", "")),
            }
        )
    return samples


def fit_pointwise_m5(samples: list[dict[str, float]]) -> tuple[M5Params, dict[str, Any]]:
    gamma_grid = [0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.05]
    dead_grid = [math.radians(v) for v in (0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0)]
    bias_grid = [math.radians(v) for v in (-5.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 5.0)]
    best: tuple[float, M5Params] | None = None
    rows = []
    for gamma, dead, bias in itertools.product(gamma_grid, dead_grid, bias_grid):
        residuals = [kappa_m5(s["phi"], gamma, dead, bias) - s["kappa"] for s in samples]
        score = rmse(residuals)
        if score is None:
            continue
        rows.append({"gamma_slope": gamma, "phi_dead_deg": math.degrees(dead), "phi_bias_deg": math.degrees(bias), "rmse_1pm": score})
        params = M5Params("M5_pointfit", gamma, dead, bias, objective="pointwise_curvature")
        if best is None or score < best[0]:
            best = (score, params)
    if best is None:
        return M5Params("M5_pointfit", 0.75, math.radians(2.0), 0.0, objective="fallback"), {"grid": rows}
    return best[1], {"best_rmse_1pm": best[0], "grid": rows}


def make_windows(rows: list[dict[str, Any]], reference: str, horizon_s: float, sample_period_s: float) -> list[tuple[list[dict[str, Any]], int, int]]:
    windows = []
    for group in grouped_segments(good_rows(rows)):
        group = [row for row in group if pose(row, reference) is not None]
        times = [float(row["t"]) for row in group]
        next_start = -math.inf
        for start_idx, start in enumerate(group):
            start_t = float(start["t"])
            if start_t < next_start:
                continue
            next_start = start_t + sample_period_s
            target_t = start_t + horizon_s
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
            start_pose = pose(group[start_idx], reference)
            end_pose = pose(group[end_idx], reference)
            if start_pose is None or end_pose is None:
                continue
            dist = math.hypot(end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
            yaw_delta = abs(wrap_angle(end_pose[2] - start_pose[2]))
            if dist < MOVING_MIN_DISTANCE_M and yaw_delta < MOVING_MIN_YAW_RAD:
                continue
            windows.append((group, start_idx, end_idx))
    return windows


def simulate_m5_window(
    group: list[dict[str, Any]],
    start_idx: int,
    end_idx: int,
    params: M5Params,
    reference: str,
    series_by_session: dict[str, Series],
) -> dict[str, Any] | None:
    start = pose(group[start_idx], reference)
    end = pose(group[end_idx], reference)
    if start is None or end is None:
        return None
    x, y, yaw = start
    session = str(group[start_idx].get("session", ""))
    series = series_by_session.get(session)
    prev_t = float(group[start_idx]["t"])
    for row in group[start_idx + 1 : end_idx + 1]:
        t = float(row["t"])
        dt = max(0.0, min(t - prev_t, 0.5))
        prev_t = t
        if dt <= 0.0:
            continue
        phi_row = row
        if series is not None and abs(params.phi_delay_s) > 1e-9:
            phi_row = series.nearest(t - params.phi_delay_s, 0.10) or row
        speed = params.speed_scale * measured_speed(row)
        phi = measured_phi(phi_row)
        kappa = kappa_m5(phi, params.gamma_slope, params.phi_dead_rad, params.phi_bias_rad)
        yaw_rate = speed * kappa if abs(speed) >= MIN_TURN_SPEED_MS else 0.0
        dyaw = yaw_rate * dt
        mid = yaw + 0.5 * dyaw
        x += speed * math.cos(mid) * dt
        y += speed * math.sin(mid) * dt
        yaw = wrap_angle(yaw + dyaw)
    ex, ey, eyaw = se2_error((x, y, yaw), end)
    return {
        "model": params.name,
        "family": "m5",
        "level": "M5",
        "reference": reference,
        "horizon_s": float(group[end_idx]["t"]) - float(group[start_idx]["t"]),
        "start_t": float(group[start_idx]["t"]),
        "end_t": float(group[end_idx]["t"]),
        "segment_id": group[start_idx].get("segment_id", ""),
        "motion_class": group[start_idx].get("motion_class", ""),
        "longitudinal_error_m": ex,
        "lateral_error_m": ey,
        "position_error_m": math.hypot(ex, ey),
        "yaw_error_rad": abs(eyaw),
        "max_abs_phi_rad": max(abs(measured_phi(row)) for row in group[start_idx : end_idx + 1]),
        "max_abs_speed_ms": max(abs(measured_speed(row)) for row in group[start_idx : end_idx + 1]),
    }


def evaluate_m5_windows(rows: list[dict[str, Any]], params: M5Params, reference: str, horizon_s: float, sample_period_s: float) -> list[dict[str, Any]]:
    windows = make_windows(rows, reference, horizon_s, sample_period_s)
    series = grouped_series(good_rows(rows))
    out = []
    for group, start_idx, end_idx in windows:
        err = simulate_m5_window(group, start_idx, end_idx, params, reference, series)
        if err is not None:
            err["requested_horizon_s"] = horizon_s
            out.append(err)
    return out


def fit_rpe_m5(rows: list[dict[str, Any]], point_params: M5Params, reference: str, horizon_s: float, sample_period_s: float) -> tuple[M5Params, list[dict[str, Any]], list[dict[str, Any]]]:
    gamma_grid = [0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.05]
    dead_grid = [math.radians(v) for v in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)]
    bias_center = math.degrees(point_params.phi_bias_rad)
    bias_grid = [math.radians(v) for v in (bias_center - 2.0, bias_center, bias_center + 2.0)]
    delay_grid = [-0.4, -0.2, 0.0, 0.2]
    speed_grid = [0.9, 1.0, 1.1]
    windows = make_windows(rows, reference, horizon_s, sample_period_s)
    series = grouped_series(good_rows(rows))
    best: tuple[float, M5Params] | None = None
    sweep_rows = []
    for gamma, dead, bias, delay, speed_scale in itertools.product(gamma_grid, dead_grid, bias_grid, delay_grid, speed_grid):
        params = M5Params("M5_rpe_tuned", gamma, dead, bias, delay, speed_scale, "local_rpe")
        errors = []
        for group, start_idx, end_idx in windows:
            err = simulate_m5_window(group, start_idx, end_idx, params, reference, series)
            if err is not None:
                errors.append(float(err["position_error_m"]))
        score = rmse(errors)
        if score is None:
            continue
        row = {
            "gamma_slope": gamma,
            "phi_dead_deg": math.degrees(dead),
            "phi_bias_deg": math.degrees(bias),
            "phi_delay_s": delay,
            "speed_scale": speed_scale,
            "position_rmse_m": score,
            "windows": len(errors),
        }
        sweep_rows.append(row)
        if best is None or score < best[0]:
            best = (score, params)
    if best is None:
        return point_params, sweep_rows, []
    return best[1], sweep_rows, evaluate_m5_windows(rows, best[1], reference, horizon_s, sample_period_s)


def setup_matplotlib() -> Any:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "figure.dpi": 140,
            "savefig.dpi": 180,
            "font.size": 10,
            "axes.grid": True,
            "grid.alpha": 0.28,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )
    return plt


def savefig(plt: Any, path: Path, outputs: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path)
    plt.close()
    outputs[path.name] = str(path)


def plot_curvature_fit(plt: Any, samples: list[dict[str, float]], point_params: M5Params, rpe_params: M5Params, out_dir: Path, outputs: dict[str, str]) -> None:
    if not samples:
        return
    phis = [s["phi"] for s in samples]
    kappas = [s["kappa"] for s in samples]
    phi_grid = [math.radians(v) for v in [x * 0.5 for x in range(-60, 61)]]
    kn = [kappa_nom(phi) for phi in phis]
    denom = sum(v * v for v in kn)
    gamma_m2 = sum(a * b for a, b in zip(kn, kappas)) / denom if denom > 1e-12 else 1.0

    fig, ax = plt.subplots(figsize=(9, 6))
    ax.scatter([math.degrees(p) for p in phis], kappas, s=7, alpha=0.22, label="measured ICP curvature")
    bins = list(range(-30, 31, 2))
    bx, by, by1, by3 = [], [], [], []
    for lo, hi in zip(bins[:-1], bins[1:]):
        vals = [s["kappa"] for s in samples if lo <= math.degrees(s["phi"]) < hi]
        if len(vals) >= 5:
            bx.append(0.5 * (lo + hi))
            by.append(statistics.median(vals))
            by1.append(percentile(vals, 25.0) or statistics.median(vals))
            by3.append(percentile(vals, 75.0) or statistics.median(vals))
    if bx:
        ax.plot(bx, by, color="black", linewidth=2.0, label="binned median")
        ax.fill_between(bx, by1, by3, color="black", alpha=0.12, label="IQR")
    ax.plot([math.degrees(p) for p in phi_grid], [kappa_nom(p) for p in phi_grid], label="M0 exact", linewidth=2.0)
    ax.plot([math.degrees(p) for p in phi_grid], [gamma_m2 * kappa_nom(p) for p in phi_grid], label=f"M2 gain ({gamma_m2:.2f})", linewidth=2.0)
    ax.plot(
        [math.degrees(p) for p in phi_grid],
        [kappa_m5(p, point_params.gamma_slope, point_params.phi_dead_rad, point_params.phi_bias_rad) for p in phi_grid],
        label="M5 point fit",
        linewidth=2.4,
    )
    ax.plot(
        [math.degrees(p) for p in phi_grid],
        [kappa_m5(p, rpe_params.gamma_slope, rpe_params.phi_dead_rad, rpe_params.phi_bias_rad) for p in phi_grid],
        label="M5 RPE tuned",
        linewidth=2.4,
        linestyle="--",
    )
    ax.axvspan(
        math.degrees(rpe_params.phi_bias_rad - rpe_params.phi_dead_rad),
        math.degrees(rpe_params.phi_bias_rad + rpe_params.phi_dead_rad),
        color="gray",
        alpha=0.14,
        label="M5 deadband",
    )
    ax.set_xlabel("articulation phi [deg]")
    ax.set_ylabel("curvature kappa [1/m]")
    ax.set_title("M5 evidence: curvature deficit + deadband on real bag")
    ax.legend(fontsize=8, ncol=2)
    savefig(plt, out_dir / "m5_curvature_law_fit_real_data.png", outputs)


def plot_tuning_heatmap(plt: Any, sweep: list[dict[str, Any]], best: M5Params, out_dir: Path, outputs: dict[str, str]) -> None:
    if not sweep:
        return
    best_delay = best.phi_delay_s
    best_speed = best.speed_scale
    best_bias = min(sweep, key=lambda r: abs(float(r["phi_bias_deg"]) - math.degrees(best.phi_bias_rad)))["phi_bias_deg"]
    rows = [
        row
        for row in sweep
        if abs(float(row["phi_delay_s"]) - best_delay) < 1e-9
        and abs(float(row["speed_scale"]) - best_speed) < 1e-9
        and abs(float(row["phi_bias_deg"]) - float(best_bias)) < 1e-9
    ]
    gammas = sorted({float(row["gamma_slope"]) for row in rows})
    deads = sorted({float(row["phi_dead_deg"]) for row in rows})
    if not gammas or not deads:
        return
    z = []
    for dead in deads:
        zrow = []
        for gamma in gammas:
            match = [row for row in rows if abs(float(row["gamma_slope"]) - gamma) < 1e-9 and abs(float(row["phi_dead_deg"]) - dead) < 1e-9]
            zrow.append(float(match[0]["position_rmse_m"]) if match else math.nan)
        z.append(zrow)
    fig, ax = plt.subplots(figsize=(8, 6))
    mesh = ax.imshow(z, origin="lower", aspect="auto", extent=[min(gammas), max(gammas), min(deads), max(deads)], cmap="viridis")
    fig.colorbar(mesh, ax=ax, label="3s moving RPE RMSE [m]")
    ax.scatter([best.gamma_slope], [math.degrees(best.phi_dead_rad)], color="red", marker="x", s=90, label="best")
    ax.set_xlabel("M5 gamma_slope")
    ax.set_ylabel("M5 phi_dead [deg]")
    ax.set_title(f"M5 RPE tuning surface (bias={float(best_bias):.1f} deg, delay={best_delay:.1f}s)")
    ax.legend()
    savefig(plt, out_dir / "m5_rpe_tuning_surface_gamma_deadband.png", outputs)


def plot_rpe_comparison(plt: Any, calibrated_errors: list[dict[str, Any]], m5_errors: list[dict[str, Any]], out_dir: Path, outputs: dict[str, str], reference: str, horizon_s: float) -> None:
    rows = [
        row
        for row in calibrated_errors
        if row.get("reference") == reference and abs(float(row.get("horizon_s", -1.0)) - horizon_s) < 0.35
    ] + m5_errors
    present = [model for model in MODEL_ORDER if any(row.get("model") == model for row in rows)]
    if not present:
        return
    data = [[float(row["position_error_m"]) for row in rows if row.get("model") == model] for model in present]
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.violinplot([vals or [0.0] for vals in data], showmedians=True, showextrema=False)
    ax.boxplot([vals or [0.0] for vals in data], showmeans=True, showfliers=False, whis=(5, 95), widths=0.18)
    ax.set_xticks(range(1, len(present) + 1))
    ax.set_xticklabels([LABELS.get(model, model) for model in present])
    ax.set_yscale("symlog", linthresh=0.05)
    ax.set_ylabel(f"{horizon_s:g}s moving RPE position error [m]")
    ax.set_title("Does M5 improve local prediction? Real moving windows")
    savefig(plt, out_dir / "m5_vs_existing_models_rpe_violin.png", outputs)

    fig, ax = plt.subplots(figsize=(10, 5))
    rmses = [rmse(vals) or math.nan for vals in data]
    ax.bar([LABELS.get(model, model) for model in present], rmses, color=["#555555" if model == "odom_delta" else "#2a9d8f" if "measured" in model or "M5" in model else "#e76f51" for model in present])
    for i, val in enumerate(rmses):
        ax.text(i, val, f"{val:.2f}", ha="center", va="bottom", fontsize=9)
    ax.set_ylabel(f"{horizon_s:g}s position RMSE [m]")
    ax.set_title("M5 contribution: RMSE ladder on moving windows")
    savefig(plt, out_dir / "m5_vs_existing_models_rpe_rmse_bar.png", outputs)


def plot_hysteresis_proxy(plt: Any, samples: list[dict[str, float]], params: M5Params, out_dir: Path, outputs: dict[str, str]) -> None:
    if not samples:
        return
    up = [s for s in samples if s["phidot"] > 0.01]
    down = [s for s in samples if s["phidot"] < -0.01]
    amp = max(10.0, min(30.0, percentile([abs(math.degrees(s["phi"])) for s in samples], 95.0) or 20.0))
    phis = [math.radians(x) for x in [(-amp + 2 * amp * i / 600) for i in range(601)]]
    play_w = 0.0
    loop_phi = []
    loop_k = []
    tri = phis + list(reversed(phis)) + phis
    for phi in tri:
        centered = phi - params.phi_bias_rad
        r = params.phi_dead_rad
        if centered > play_w + r:
            play_w = centered - r
        elif centered < play_w - r:
            play_w = centered + r
        loop_phi.append(math.degrees(phi))
        loop_k.append((params.gamma_slope / L_M) * play_w)

    fig, ax = plt.subplots(figsize=(8.5, 6))
    if up:
        ax.scatter([math.degrees(s["phi"]) for s in up], [s["kappa"] for s in up], s=10, alpha=0.35, label="measured phidot > 0")
    if down:
        ax.scatter([math.degrees(s["phi"]) for s in down], [s["kappa"] for s in down], s=10, alpha=0.35, label="measured phidot < 0")
    ax.plot(loop_phi, loop_k, color="black", linewidth=1.4, label="M5 play-operator loop")
    ax.axvspan(
        math.degrees(params.phi_bias_rad - params.phi_dead_rad),
        math.degrees(params.phi_bias_rad + params.phi_dead_rad),
        color="gray",
        alpha=0.13,
        label="deadband",
    )
    ax.set_xlabel("articulation phi [deg]")
    ax.set_ylabel("curvature kappa [1/m]")
    ax.set_title("M5 hysteresis prediction/proxy: reversal branches")
    ax.legend(fontsize=8)
    savefig(plt, out_dir / "m5_hysteresis_proxy_phi_kappa_loop.png", outputs)


def plot_command_error_gain(plt: Any, calibrated_errors: list[dict[str, Any]], m5_errors: list[dict[str, Any]], out_dir: Path, outputs: dict[str, str], reference: str, horizon_s: float) -> None:
    base = {
        (str(row.get("segment_id")), float(row.get("start_t", 0.0))): float(row["position_error_m"])
        for row in calibrated_errors
        if row.get("reference") == reference and row.get("model") == "M0_measured_exact" and abs(float(row.get("horizon_s", -1.0)) - horizon_s) < 0.35
    }
    xs, ys, gains = [], [], []
    for row in m5_errors:
        key = (str(row.get("segment_id")), float(row.get("start_t", 0.0)))
        if key not in base:
            continue
        xs.append(float(row.get("max_abs_speed_ms", 0.0)))
        ys.append(float(row.get("max_abs_phi_rad", 0.0)))
        gains.append(base[key] - float(row["position_error_m"]))
    if not gains:
        return
    fig, ax = plt.subplots(figsize=(8, 6))
    hb = ax.hexbin(xs, ys, C=gains, reduce_C_function=statistics.mean, gridsize=18, mincnt=1, cmap="coolwarm")
    fig.colorbar(hb, ax=ax, label="mean improvement vs M0 measured [m]")
    ax.set_xlabel("max |measured speed| [m/s]")
    ax.set_ylabel("max |articulation| [rad]")
    ax.set_title("Where M5 helps: error reduction over command/state space")
    savefig(plt, out_dir / "m5_improvement_hexbin_vs_speed_articulation.png", outputs)


def plot_dgt_control_evidence(plt: Any, params: M5Params, out_dir: Path, outputs: dict[str, str]) -> dict[str, Any]:
    sys.path.insert(0, str(Path(__file__).resolve().parent / "mtt_motion_model"))
    try:
        from motion_model import VehicleParams, M5ContactParams
        from m5_control import DGTConfig, evaluate_tracking, speed_envelope
        from traj_tracking_sim import circular_arc_path, s_curve_path
    except Exception as exc:  # pragma: no cover - diagnostic best effort
        return {"available": False, "reason": str(exc)}

    p = VehicleParams.mtt154()
    m5 = M5ContactParams.calibrated(
        p,
        gamma_slope=params.gamma_slope,
        phi_dead_deg=math.degrees(params.phi_dead_rad),
    )
    control_summary: dict[str, Any] = {"available": True}

    cfg_on = DGTConfig(deadband_inverse=True, d_settle=9.0, v_ref=0.6)
    cfg_off = DGTConfig(deadband_inverse=False, d_settle=9.0, v_ref=0.6)
    res_on = evaluate_tracking(circular_arc_path(R=6.0), s_max=30.0, p=p, m5=m5, cfg=cfg_on)
    res_off = evaluate_tracking(circular_arc_path(R=6.0), s_max=30.0, p=p, m5=m5, cfg=cfg_off)
    fig, ax = plt.subplots(figsize=(8.5, 5))
    ax.plot(res_on["s"], [100.0 * abs(v) for v in res_on["e_y"]], label="deadband inverse ON", linewidth=2.2)
    ax.plot(res_off["s"], [100.0 * abs(v) for v in res_off["e_y"]], label="deadband inverse OFF", linewidth=2.2)
    ax.set_xlabel("arc length s [m]")
    ax.set_ylabel("|cross-track error| [cm]")
    ax.set_title("DGT ablation: deadband inverse avoids wasting steering authority")
    ax.legend()
    savefig(plt, out_dir / "m5_dgt_deadband_inverse_ablation.png", outputs)

    control_summary["deadband_inverse"] = {
        "on_rms_ey_m": float(res_on["tracking"].rms_ey),
        "off_rms_ey_m": float(res_off["tracking"].rms_ey),
        "gain_pct": 100.0 * (float(res_off["tracking"].rms_ey) - float(res_on["tracking"].rms_ey)) / max(float(res_off["tracking"].rms_ey), 1e-9),
    }

    fig, ax = plt.subplots(figsize=(8.5, 5))
    speed_rows = []
    for speed in (0.25, 0.45, 0.65):
        cfg = DGTConfig(deadband_inverse=True, d_settle=9.0, v_ref=speed)
        res = evaluate_tracking(s_curve_path(kappa_max=0.10, wavelength=18.0), s_max=36.0, p=p, m5=m5, cfg=cfg)
        ax.plot(res["s"], [100.0 * v for v in res["e_y"]], linewidth=1.8, label=f"v={speed:.2f} m/s")
        speed_rows.append(
            {
                "speed_ms": speed,
                "rms_ey_m": float(res["tracking"].rms_ey),
                "overshoot_m": float(res["tracking"].overshoot),
                "settle_distance_m": float(res["tracking"].settle_distance),
            }
        )
    ax.set_xlabel("arc length s [m]")
    ax.set_ylabel("cross-track error [cm]")
    ax.set_title("Distance-domain prediction: similar error vs distance across speeds")
    ax.legend()
    savefig(plt, out_dir / "m5_dgt_distance_domain_speed_invariance.png", outputs)
    control_summary["speed_invariance"] = speed_rows

    settle_rows = []
    for d_settle in (5.0, 7.0, 9.0, 12.0):
        cfg = DGTConfig(deadband_inverse=True, d_settle=d_settle, v_ref=0.6)
        res = evaluate_tracking(circular_arc_path(R=6.0), s_max=30.0, p=p, m5=m5, cfg=cfg)
        settle_rows.append(
            {
                "d_settle_cmd_m": d_settle,
                "rms_ey_m": float(res["tracking"].rms_ey),
                "overshoot_m": float(res["tracking"].overshoot),
                "settle_distance_m": float(res["tracking"].settle_distance),
                "speed_envelope_ms": speed_envelope(cfg, m5, p),
            }
        )
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
    xs = [row["d_settle_cmd_m"] for row in settle_rows]
    axes[0].plot(xs, [100.0 * row["rms_ey_m"] for row in settle_rows], marker="o")
    axes[0].set_ylabel("RMS cross-track [cm]")
    axes[0].set_xlabel("commanded settle distance [m]")
    axes[0].set_title("tracking error")
    axes[1].plot(xs, [row["settle_distance_m"] for row in settle_rows], marker="o")
    axes[1].set_ylabel("observed settle distance [m]")
    axes[1].set_xlabel("commanded settle distance [m]")
    axes[1].set_title("settling")
    axes[2].plot(xs, [row["speed_envelope_ms"] for row in settle_rows], marker="o")
    axes[2].set_ylabel("speed envelope [m/s]")
    axes[2].set_xlabel("commanded settle distance [m]")
    axes[2].set_title("servo feasibility")
    fig.suptitle("DGT tuning: aggressiveness vs error and speed envelope")
    savefig(plt, out_dir / "m5_dgt_dsettle_tuning_tradeoff.png", outputs)
    control_summary["dsettle_sweep"] = settle_rows
    return control_summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--reference", choices=["icp", "odom"], default="icp")
    parser.add_argument("--horizon-s", type=float, default=3.0)
    parser.add_argument("--sample-period-s", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = args.session.expanduser().resolve()
    out_dir = args.output_dir or session / "motion_model_validation" / "motion_research" / "calibrated_models" / "m5_contribution"
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = read_csv(dataset_csv_for_session(session))
    calibrated_errors = read_csv(calibrated_errors_csv(session))
    samples = curvature_samples(rows, args.reference)
    point_params, point_fit = fit_pointwise_m5(samples)
    rpe_params, sweep_rows, rpe_errors = fit_rpe_m5(rows, point_params, args.reference, args.horizon_s, args.sample_period_s)
    point_errors = evaluate_m5_windows(rows, point_params, args.reference, args.horizon_s, args.sample_period_s)
    for row in point_errors:
        row["model"] = "M5_pointfit"
    combined_m5_errors = point_errors + rpe_errors

    write_csv(out_dir / "m5_tuning_sweep.csv", sweep_rows)
    write_csv(out_dir / "m5_prediction_errors.csv", combined_m5_errors)
    write_csv(out_dir / "m5_pointwise_fit_grid.csv", point_fit.get("grid", []))

    plt = setup_matplotlib()
    outputs: dict[str, str] = {}
    plot_curvature_fit(plt, samples, point_params, rpe_params, out_dir, outputs)
    plot_tuning_heatmap(plt, sweep_rows, rpe_params, out_dir, outputs)
    plot_rpe_comparison(plt, calibrated_errors, combined_m5_errors, out_dir, outputs, args.reference, args.horizon_s)
    plot_hysteresis_proxy(plt, samples, rpe_params, out_dir, outputs)
    plot_command_error_gain(plt, calibrated_errors, rpe_errors, out_dir, outputs, args.reference, args.horizon_s)
    control_summary = plot_dgt_control_evidence(plt, rpe_params, out_dir, outputs)

    summary = {
        "session": session.name,
        "reference": args.reference,
        "horizon_s": args.horizon_s,
        "curvature_samples": len(samples),
        "point_fit": {
            **asdict(point_params),
            "phi_dead_deg": math.degrees(point_params.phi_dead_rad),
            "phi_bias_deg": math.degrees(point_params.phi_bias_rad),
            "best_rmse_1pm": point_fit.get("best_rmse_1pm"),
        },
        "rpe_fit": {
            **asdict(rpe_params),
            "phi_dead_deg": math.degrees(rpe_params.phi_dead_rad),
            "phi_bias_deg": math.degrees(rpe_params.phi_bias_rad),
            "position_error_m": finite_stats(float(row["position_error_m"]) for row in rpe_errors),
            "yaw_error_deg": finite_stats(math.degrees(float(row["yaw_error_rad"])) for row in rpe_errors),
        },
        "baselines_at_horizon": {
            model: finite_stats(
                float(row["position_error_m"])
                for row in calibrated_errors
                if row.get("reference") == args.reference
                and row.get("model") == model
                and abs(float(row.get("horizon_s", -1.0)) - args.horizon_s) < 0.35
            )
            for model in ["odom_delta", "M0_measured_exact", "M3_measured_phi_delay", "M0_command_exact", "M3_command_curvature_residual"]
        },
        "outputs": outputs,
        "control_evidence": control_summary,
        "notes": [
            "M5_pointfit optimizes instantaneous curvature kappa(phi).",
            "M5_rpe_tuned optimizes local moving-window RPE and can differ from pointwise curvature fit.",
            "M5 is successful only if it improves moving-window RPE without merely fitting stopped windows.",
        ],
    }
    (out_dir / "m5_contribution_summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
