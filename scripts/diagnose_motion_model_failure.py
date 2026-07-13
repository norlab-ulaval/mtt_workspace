#!/usr/bin/env python3
"""Diagnose why open-loop motion models diverge from ICP/odom references."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml

from lib.mtt_motion_research import (
    ModelParams,
    finite_stats,
    input_speed,
    nominal_curvature,
    parse_bool,
    parse_float,
    percentile,
    prediction_errors,
    read_csv_rows,
    se2_log_error,
    simulate_window,
    wrap_angle,
    write_csv_rows,
)


def dataset_csv_for_target(path: Path) -> Path:
    path = path.expanduser().resolve()
    candidates = [
        path / "motion_research" / "dataset.csv",
        path / "dataset.csv",
        path / "motion_model_validation" / "motion_research" / "datasets" / path.name / "dataset.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"No motion_research dataset.csv found under {path}")


def finite_pairs(rows: list[dict], key_a: str, key_b: str) -> list[tuple[float, float]]:
    pairs = []
    for row in rows:
        a = parse_float(row.get(key_a))
        b = parse_float(row.get(key_b))
        if a is not None and b is not None:
            pairs.append((a, b))
    return pairs


def error_stats(values: list[float]) -> dict[str, float | int | None]:
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return {"count": 0, "bias": None, "mae": None, "rmse": None, "p95_abs": None, "max_abs": None}
    abs_vals = [abs(v) for v in vals]
    return {
        "count": len(vals),
        "bias": sum(vals) / len(vals),
        "mae": sum(abs_vals) / len(abs_vals),
        "rmse": math.sqrt(sum(v * v for v in vals) / len(vals)),
        "p95_abs": percentile(abs_vals, 95.0),
        "max_abs": max(abs_vals),
    }


def add_pointwise_diagnostics(rows: list[dict]) -> None:
    for row in rows:
        v_tacho = parse_float(row.get("tacho_signed_ms"))
        v_icp = parse_float(row.get("icp_speed_ms"))
        v_cmd = parse_float(row.get("cmd_speed_ms"))
        phi = parse_float(row.get("phi_rad"))
        omega_icp = parse_float(row.get("icp_yaw_rate_rad_s_derived"))
        omega_imu = parse_float(row.get("imu_yaw_rate_rad_s"))
        if v_tacho is not None and phi is not None:
            row["omega_m0_tacho_exact"] = v_tacho * nominal_curvature(phi, "exact")
            row["omega_m0_tacho_tangent"] = v_tacho * nominal_curvature(phi, "tangent")
        else:
            row["omega_m0_tacho_exact"] = math.nan
            row["omega_m0_tacho_tangent"] = math.nan
        cmd_phi = parse_float(row.get("cmd_phi_rad"))
        if v_cmd is not None and cmd_phi is not None:
            row["omega_cmd_exact"] = v_cmd * nominal_curvature(cmd_phi, "exact")
        else:
            row["omega_cmd_exact"] = math.nan
        row["speed_error_tacho_minus_icp"] = v_tacho - v_icp if v_tacho is not None and v_icp is not None else math.nan
        row["speed_error_cmd_minus_icp"] = v_cmd - v_icp if v_cmd is not None and v_icp is not None else math.nan
        row["yaw_error_m0_minus_icp"] = (
            row["omega_m0_tacho_exact"] - omega_icp
            if math.isfinite(row["omega_m0_tacho_exact"]) and omega_icp is not None
            else math.nan
        )
        row["yaw_error_imu_minus_icp"] = omega_imu - omega_icp if omega_imu is not None and omega_icp is not None else math.nan
        mtt_phi = parse_float(row.get("mtt_articulation_angle", row.get("mtt_articulation_rad")))
        trailer_phi = parse_float(row.get("trailer_articulation_angle", row.get("trailer_articulation_rad")))
        row["articulation_mtt_minus_trailer"] = mtt_phi - trailer_phi if mtt_phi is not None and trailer_phi is not None else math.nan


def simulate_with_icp_yaw(rows: list[dict], start_idx: int, end_idx: int, speed_source: str) -> tuple[float, float, float]:
    start = rows[start_idx]
    x = float(start["icp_x"])
    y = float(start["icp_y"])
    prev_t = float(start["t"])
    prev_yaw = float(start["icp_yaw"])
    for row in rows[start_idx + 1 : end_idx + 1]:
        t = float(row["t"])
        dt = max(0.0, min(t - prev_t, 0.5))
        prev_t = t
        if dt <= 0.0:
            continue
        speed = input_speed(row, speed_source)
        if speed is None:
            continue
        yaw = float(row["icp_yaw"])
        heading_mid = wrap_angle(0.5 * (prev_yaw + yaw))
        x += speed * math.cos(heading_mid) * dt
        y += speed * math.sin(heading_mid) * dt
        prev_yaw = yaw
    return x, y, float(rows[end_idx]["icp_yaw"])


def ablation_errors(rows: list[dict], horizons_s: list[float], sample_period_s: float, include_weak: bool) -> list[dict]:
    models = [
        ModelParams("odom_delta", "odom_delta"),
        ModelParams("m0_tacho_phi", "direct", "tacho", "measured", "exact"),
        ModelParams("m0_icp_speed_phi", "direct", "icp", "measured", "exact"),
        ModelParams("m0_cmd", "direct", "cmd", "cmd", "exact"),
    ]
    out: list[dict] = []
    for model in models:
        for row in prediction_errors(rows, model, horizons_s=horizons_s, sample_period_s=sample_period_s, include_weak=include_weak):
            row["ablation_meaning"] = {
                "odom_delta": "recorded odom local delta; closed-loop/reference prior baseline",
                "m0_tacho_phi": "full measured-state M0; speed + articulation errors both active",
                "m0_icp_speed_phi": "ICP speed + measured articulation; isolates yaw/curvature/articulation error",
                "m0_cmd": "raw command-to-pose; exposes actuator/control dynamics gap",
            }[model.name]
            out.append(row)

    valid_indices = [idx for idx, row in enumerate(rows) if include_weak or parse_bool(row.get("research_quality_ok"))]
    times = [float(row["t"]) for row in rows]
    for speed_source, label in (("tacho", "tacho_speed_icp_yaw"), ("icp", "icp_speed_icp_yaw")):
        next_start_t = -math.inf
        for start_idx in valid_indices:
            start_t = times[start_idx]
            if start_t < next_start_t:
                continue
            next_start_t = start_t + sample_period_s
            for horizon in horizons_s:
                target_t = start_t + horizon
                end_idx = min(range(start_idx, len(rows)), key=lambda idx: abs(times[idx] - target_t))
                if abs(times[end_idx] - target_t) > 0.12 or end_idx <= start_idx:
                    continue
                window = rows[start_idx : end_idx + 1]
                if not include_weak and not all(parse_bool(row.get("research_quality_ok")) for row in window):
                    continue
                pred_x, pred_y, pred_yaw = simulate_with_icp_yaw(rows, start_idx, end_idx, speed_source)
                ref = rows[end_idx]
                ex, ey, eyaw = se2_log_error(pred_x, pred_y, pred_yaw, float(ref["icp_x"]), float(ref["icp_y"]), float(ref["icp_yaw"]))
                out.append(
                    {
                        "model": label,
                        "horizon_s": horizon,
                        "start_t": start_t,
                        "end_t": float(ref["t"]),
                        "position_error_m": math.hypot(ex, ey),
                        "longitudinal_error_m": ex,
                        "lateral_error_m": ey,
                        "yaw_error_rad": abs(eyaw),
                        "motion_class": rows[start_idx].get("motion_class", ""),
                        "quality_class": rows[start_idx].get("quality_class", ""),
                        "max_abs_phi_rad": max(abs(parse_float(row.get("phi_rad")) or 0.0) for row in window),
                        "ablation_meaning": {
                            "tacho_speed_icp_yaw": "tacho speed integrated on ICP heading; isolates longitudinal speed/source error",
                            "icp_speed_icp_yaw": "ICP speed integrated on ICP heading; numerical sanity check",
                        }[label],
                    }
                )
    return out


def summarize_ablation(errors: list[dict]) -> dict:
    summary: dict[str, dict] = {}
    for model in sorted({str(row["model"]) for row in errors}):
        summary[model] = {}
        for horizon in sorted({float(row["horizon_s"]) for row in errors if row["model"] == model}):
            rows_h = [row for row in errors if row["model"] == model and abs(float(row["horizon_s"]) - horizon) < 1e-9]
            vals = [float(row["position_error_m"]) for row in rows_h]
            summary[model][f"{horizon:g}s"] = {
                "count": len(vals),
                "median": percentile(vals, 50.0),
                "rmse": math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else None,
                "p95": percentile(vals, 95.0),
            }
    return summary


def write_plots(rows: list[dict], errors: list[dict], out_dir: Path) -> dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}

    outputs: dict[str, str] = {}
    t0 = float(rows[0]["t"]) if rows else 0.0
    t = [float(row["t"]) - t0 for row in rows]
    valid = [row for row in rows if parse_bool(row.get("research_quality_ok"))]

    def save(name: str) -> None:
        path = out_dir / name
        plt.tight_layout()
        plt.savefig(path, dpi=170)
        plt.close()
        outputs[name] = str(path)

    plt.figure(figsize=(13, 5))
    for key, label in (
        ("icp_speed_ms", "ICP speed"),
        ("odom_speed_ms", "odom speed"),
        ("tacho_signed_ms", "tacho speed"),
        ("cmd_speed_ms", "cmd speed"),
    ):
        vals = [parse_float(row.get(key)) or 0.0 for row in rows]
        plt.plot(t, vals, label=label, linewidth=1.0)
    plt.xlabel("time [s]")
    plt.ylabel("speed [m/s]")
    plt.title("Speed sources")
    plt.grid(True, alpha=0.3)
    plt.legend()
    save("speed_sources_time.png")

    plt.figure(figsize=(7, 7))
    pairs = finite_pairs(valid, "icp_speed_ms", "tacho_signed_ms")
    if pairs:
        xs, ys = zip(*pairs)
        plt.scatter(xs, ys, s=8, alpha=0.35)
        lim = max(max(abs(x) for x in xs), max(abs(y) for y in ys), 0.5)
        plt.plot([-lim, lim], [-lim, lim], "k--", linewidth=1.0)
        plt.xlim(-lim, lim)
        plt.ylim(-lim, lim)
    plt.xlabel("ICP speed [m/s]")
    plt.ylabel("tacho speed [m/s]")
    plt.title("Tachometer vs ICP speed")
    plt.grid(True, alpha=0.3)
    save("speed_tacho_vs_icp_scatter.png")

    plt.figure(figsize=(11, 5))
    for key, label in (
        ("speed_error_tacho_minus_icp", "tacho - ICP"),
        ("speed_error_cmd_minus_icp", "cmd - ICP"),
    ):
        vals = [float(row[key]) for row in valid if math.isfinite(float(row.get(key, math.nan)))]
        if vals:
            plt.hist(vals, bins=80, histtype="step", linewidth=1.8, label=label)
    plt.xlabel("speed error [m/s]")
    plt.ylabel("sample count")
    plt.title("Speed residuals")
    plt.grid(True, alpha=0.3)
    plt.legend()
    save("speed_error_hist.png")

    plt.figure(figsize=(13, 5))
    for key, label in (
        ("phi_rad", "selected phi"),
        ("mtt_articulation_angle", "MTT articulation"),
        ("trailer_articulation_angle", "trailer articulation"),
        ("cmd_phi_rad", "cmd phi"),
    ):
        vals = [parse_float(row.get(key)) or 0.0 for row in rows]
        plt.plot(t, vals, label=label, linewidth=1.0)
    plt.xlabel("time [s]")
    plt.ylabel("articulation [rad]")
    plt.title("Articulation sources")
    plt.grid(True, alpha=0.3)
    plt.legend()
    save("articulation_sources_time.png")

    plt.figure(figsize=(13, 5))
    for key, label in (
        ("icp_yaw_rate_rad_s_derived", "ICP yaw rate"),
        ("imu_yaw_rate_rad_s", "IMU yaw rate"),
        ("omega_m0_tacho_exact", "M0 tacho+phi yaw rate"),
        ("omega_cmd_exact", "command yaw rate"),
    ):
        vals = [parse_float(row.get(key)) or 0.0 for row in rows]
        plt.plot(t, vals, label=label, linewidth=1.0)
    plt.xlabel("time [s]")
    plt.ylabel("yaw rate [rad/s]")
    plt.title("Yaw-rate decomposition")
    plt.grid(True, alpha=0.3)
    plt.legend()
    save("yaw_rate_decomposition_time.png")

    plt.figure(figsize=(12, 6))
    classes = sorted({str(row.get("motion_class", "")) for row in valid})
    data = [
        [abs(float(row["yaw_error_m0_minus_icp"])) for row in valid if row.get("motion_class") == klass and math.isfinite(float(row.get("yaw_error_m0_minus_icp", math.nan)))]
        for klass in classes
    ]
    data = [vals if vals else [0.0] for vals in data]
    if classes:
        plt.boxplot(data, labels=classes, showfliers=True)
        plt.yscale("symlog", linthresh=0.01)
    plt.ylabel("|M0 yaw-rate - ICP yaw-rate| [rad/s]")
    plt.title("Yaw-rate residual by motion class")
    plt.grid(True, axis="y", alpha=0.3)
    plt.xticks(rotation=30, ha="right")
    save("yaw_rate_error_by_motion_class.png")

    plt.figure(figsize=(12, 6))
    models = sorted({str(row["model"]) for row in errors})
    horizons = sorted({float(row["horizon_s"]) for row in errors})
    for model in models:
        rmses = []
        for horizon in horizons:
            vals = [
                float(row["position_error_m"])
                for row in errors
                if row["model"] == model and abs(float(row["horizon_s"]) - horizon) < 1e-9
            ]
            rmses.append(math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else math.nan)
        plt.plot(horizons, rmses, marker="o", label=model)
    plt.xlabel("prediction horizon [s]")
    plt.ylabel("position RMSE [m]")
    plt.title("Causal ablation: what source explains the divergence?")
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=8)
    save("ablation_horizon_rmse.png")

    return outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--horizons", default="0.5,1,2,3,5,10")
    parser.add_argument("--sample-period-s", type=float, default=0.5)
    parser.add_argument("--include-weak", action="store_true", help="Keep weak ICP windows for diagnostic plots.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    dataset_csv = dataset_csv_for_target(args.path)
    rows = read_csv_rows(dataset_csv)
    add_pointwise_diagnostics(rows)
    horizons = [float(x) for x in args.horizons.split(",") if x.strip()]
    errors = ablation_errors(rows, horizons, args.sample_period_s, args.include_weak)
    out_dir = args.output_dir or dataset_csv.parent / "failure_modes"
    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv_rows(out_dir / "pointwise_diagnostics.csv", rows)
    write_csv_rows(out_dir / "ablation_prediction_errors.csv", errors)

    valid = [row for row in rows if parse_bool(row.get("research_quality_ok"))]
    summary = {
        "dataset_csv": str(dataset_csv),
        "rows": len(rows),
        "valid_rows": len(valid),
        "include_weak_windows": bool(args.include_weak),
        "speed_tacho_minus_icp": error_stats([float(row["speed_error_tacho_minus_icp"]) for row in valid if math.isfinite(float(row.get("speed_error_tacho_minus_icp", math.nan)))]),
        "speed_cmd_minus_icp": error_stats([float(row["speed_error_cmd_minus_icp"]) for row in valid if math.isfinite(float(row.get("speed_error_cmd_minus_icp", math.nan)))]),
        "yaw_m0_minus_icp": error_stats([float(row["yaw_error_m0_minus_icp"]) for row in valid if math.isfinite(float(row.get("yaw_error_m0_minus_icp", math.nan)))]),
        "yaw_imu_minus_icp": error_stats([float(row["yaw_error_imu_minus_icp"]) for row in valid if math.isfinite(float(row.get("yaw_error_imu_minus_icp", math.nan)))]),
        "articulation_mtt_minus_trailer": error_stats([float(row["articulation_mtt_minus_trailer"]) for row in valid if math.isfinite(float(row.get("articulation_mtt_minus_trailer", math.nan)))]),
        "speed_sources": {
            "icp_speed_ms": finite_stats(parse_float(row.get("icp_speed_ms")) or 0.0 for row in valid),
            "odom_speed_ms": finite_stats(parse_float(row.get("odom_speed_ms")) or 0.0 for row in valid),
            "tacho_signed_ms": finite_stats(parse_float(row.get("tacho_signed_ms")) or 0.0 for row in valid),
            "cmd_speed_ms": finite_stats(parse_float(row.get("cmd_speed_ms")) or 0.0 for row in valid),
        },
        "ablation_summary": summarize_ablation(errors),
    }
    summary["outputs"] = write_plots(rows, errors, out_dir)
    (out_dir / "failure_mode_summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump({"output_dir": str(out_dir), "summary": summary}, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
