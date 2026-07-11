#!/usr/bin/env python3
"""Evaluate M0-M4 candidate motion models on quality-labelled local windows."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml

from lib.mtt_motion_research import (
    ModelParams,
    assign_segments,
    derive_research_rows,
    find_motion_csv,
    fit_residual_model,
    parse_bool,
    percentile,
    prediction_errors,
    read_csv_rows,
    resolve_sessions,
    summarize_errors,
    write_csv_rows,
)


def load_research_rows(session: Path, args: argparse.Namespace) -> list[dict]:
    research_csv = session / "motion_research" / "dataset.csv"
    if research_csv.exists() and not args.force_rebuild:
        return read_csv_rows(research_csv)
    return assign_segments(
        derive_research_rows(
            read_csv_rows(find_motion_csv(session)),
            session_name=session.name,
            articulation_preference=args.articulation_source,
        )
    )


def write_suite_plots(out_dir: Path, all_errors: list[dict]) -> dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}
    outputs: dict[str, str] = {}
    models = sorted({str(row["model"]) for row in all_errors})
    horizons = sorted({float(row["horizon_s"]) for row in all_errors})

    def values(model: str, horizon: float, key: str) -> list[float]:
        return [
            float(row[key])
            for row in all_errors
            if row["model"] == model and abs(float(row["horizon_s"]) - horizon) < 1e-9
        ]

    def stat(model: str, horizon: float, key: str) -> float | None:
        vals = values(model, horizon, key)
        if not vals:
            return None
        return math.sqrt(sum(v * v for v in vals) / len(vals))

    def quantile_line(model: str, key: str, pct: float) -> list[float | None]:
        return [percentile(values(model, h, key), pct) for h in horizons]

    def plot_iqr(metric_key: str, ylabel: str, filename: str, title: str) -> None:
        plt.figure(figsize=(12, 6))
        for model in models:
            q1 = quantile_line(model, metric_key, 25.0)
            med = quantile_line(model, metric_key, 50.0)
            q3 = quantile_line(model, metric_key, 75.0)
            if any(v is None for v in q1 + med + q3):
                continue
            q1_vals = [float(v) for v in q1]
            med_vals = [float(v) for v in med]
            q3_vals = [float(v) for v in q3]
            line = plt.plot(horizons, med_vals, marker="o", linewidth=1.8, label=model)[0]
            plt.fill_between(horizons, q1_vals, q3_vals, color=line.get_color(), alpha=0.14)
        plt.xlabel("prediction horizon [s]")
        plt.ylabel(ylabel)
        plt.title(title)
        plt.grid(True, alpha=0.35)
        plt.legend(fontsize=8)
        plt.tight_layout()
        path = out_dir / filename
        plt.savefig(path, dpi=170)
        plt.close()
        outputs[filename] = str(path)

    def plot_distribution_by_model(horizon: float, filename_prefix: str) -> None:
        rows_h = [row for row in all_errors if abs(float(row["horizon_s"]) - horizon) < 1e-9]
        if not rows_h:
            return
        present_models = [model for model in models if any(row["model"] == model for row in rows_h)]
        data = [[float(row["position_error_m"]) for row in rows_h if row["model"] == model] for model in present_models]
        data = [vals if vals else [0.0] for vals in data]
        labels = [model.replace("/", "\n") for model in present_models]

        plt.figure(figsize=(13, 6))
        plt.boxplot(data, labels=labels, showmeans=True, showfliers=False)
        plt.yscale("symlog", linthresh=0.1)
        plt.ylabel(f"{horizon:g}s position error [m]")
        plt.title(f"Position-error distribution by model at {horizon:g}s")
        plt.grid(True, axis="y", alpha=0.35)
        plt.xticks(rotation=25, ha="right")
        plt.tight_layout()
        path = out_dir / f"{filename_prefix}_boxplot_by_model.png"
        plt.savefig(path, dpi=170)
        plt.close()
        outputs[path.name] = str(path)

        plt.figure(figsize=(13, 6))
        parts = plt.violinplot(data, showmeans=False, showmedians=True, showextrema=False)
        for body in parts["bodies"]:
            body.set_alpha(0.55)
        plt.xticks(range(1, len(labels) + 1), labels, rotation=25, ha="right")
        plt.yscale("symlog", linthresh=0.1)
        plt.ylabel(f"{horizon:g}s position error [m]")
        plt.title(f"Position-error density by model at {horizon:g}s")
        plt.grid(True, axis="y", alpha=0.35)
        plt.tight_layout()
        path = out_dir / f"{filename_prefix}_violin_by_model.png"
        plt.savefig(path, dpi=170)
        plt.close()
        outputs[path.name] = str(path)

    def plot_distribution_by_horizon(model: str, filename_prefix: str) -> None:
        if not any(row["model"] == model for row in all_errors):
            return
        data = [values(model, horizon, "position_error_m") for horizon in horizons]
        data = [vals if vals else [0.0] for vals in data]
        labels = [f"{h:g}s" for h in horizons]

        plt.figure(figsize=(11, 6))
        plt.boxplot(data, labels=labels, showmeans=True, showfliers=False)
        plt.yscale("symlog", linthresh=0.1)
        plt.xlabel("prediction horizon")
        plt.ylabel("position error [m]")
        plt.title(f"Position-error distribution by horizon ({model})")
        plt.grid(True, axis="y", alpha=0.35)
        plt.tight_layout()
        path = out_dir / f"{filename_prefix}_boxplot_by_horizon.png"
        plt.savefig(path, dpi=170)
        plt.close()
        outputs[path.name] = str(path)

        plt.figure(figsize=(11, 6))
        parts = plt.violinplot(data, showmeans=False, showmedians=True, showextrema=False)
        for body in parts["bodies"]:
            body.set_alpha(0.55)
        plt.xticks(range(1, len(labels) + 1), labels)
        plt.yscale("symlog", linthresh=0.1)
        plt.xlabel("prediction horizon")
        plt.ylabel("position error [m]")
        plt.title(f"Position-error density by horizon ({model})")
        plt.grid(True, axis="y", alpha=0.35)
        plt.tight_layout()
        path = out_dir / f"{filename_prefix}_violin_by_horizon.png"
        plt.savefig(path, dpi=170)
        plt.close()
        outputs[path.name] = str(path)

    for key, ylabel, filename in (
        ("position_error_m", "position RMSE [m]", "horizon_position_rmse.png"),
        ("yaw_error_rad", "yaw RMSE [rad]", "horizon_yaw_rmse.png"),
    ):
        plt.figure(figsize=(11, 6))
        for model in models:
            ys = [stat(model, h, key) for h in horizons]
            if any(v is None for v in ys):
                continue
            plt.plot(horizons, ys, marker="o", label=model)
        plt.xlabel("prediction horizon [s]")
        plt.ylabel(ylabel)
        plt.grid(True, alpha=0.35)
        plt.legend(fontsize=8)
        plt.tight_layout()
        path = out_dir / filename
        plt.savefig(path, dpi=160)
        plt.close()
        outputs[filename] = str(path)

    plot_iqr(
        "position_error_m",
        "position error [m]",
        "horizon_position_error_median_iqr.png",
        "Short-horizon position error: median with IQR",
    )
    plot_iqr(
        "yaw_error_rad",
        "yaw error [rad]",
        "horizon_yaw_error_median_iqr.png",
        "Short-horizon yaw error: median with IQR",
    )
    for horizon in (1.0, 3.0, 5.0, 10.0):
        plot_distribution_by_model(horizon, f"horizon{horizon:g}_position_error")
    plot_distribution_by_horizon("m0/measured_auto_exact", "measured_auto_position_error")
    plot_distribution_by_horizon("m0/odom_delta", "odom_delta_position_error")
    plot_distribution_by_horizon("m1/command_lag", "command_lag_position_error")

    plt.figure(figsize=(12, 6))
    for model in models:
        rows = [row for row in all_errors if row["model"] == model and abs(float(row["horizon_s"]) - 3.0) < 1e-9]
        if not rows:
            continue
        plt.scatter([float(r["max_abs_phi_rad"]) for r in rows], [float(r["position_error_m"]) for r in rows], s=8, alpha=0.45, label=model)
    plt.xlabel("max |articulation| in 3s window [rad]")
    plt.ylabel("3s position error [m]")
    plt.grid(True, alpha=0.35)
    plt.legend(fontsize=8)
    plt.tight_layout()
    path = out_dir / "horizon3_error_vs_articulation.png"
    plt.savefig(path, dpi=160)
    plt.close()
    outputs[path.name] = str(path)

    for horizon in (3.0, 5.0, 10.0):
        rows_h = [row for row in all_errors if abs(float(row["horizon_s"]) - horizon) < 1e-9]
        if not rows_h:
            continue
        plt.figure(figsize=(12, 6))
        bins = [0.0, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0]
        for model in models:
            vals = [float(row["position_error_m"]) for row in rows_h if row["model"] == model]
            if vals:
                plt.hist(vals, bins=bins, histtype="step", linewidth=1.8, label=model)
        plt.xscale("symlog", linthresh=0.1)
        plt.xlabel(f"{horizon:g}s position error [m]")
        plt.ylabel("window count")
        plt.title(f"Long-tail distribution at horizon {horizon:g}s")
        plt.grid(True, alpha=0.35)
        plt.legend(fontsize=8)
        plt.tight_layout()
        path = out_dir / f"horizon{horizon:g}_position_error_hist.png"
        plt.savefig(path, dpi=160)
        plt.close()
        outputs[path.name] = str(path)

        plt.figure(figsize=(12, 6))
        for model in models:
            vals = sorted(float(row["position_error_m"]) for row in rows_h if row["model"] == model)
            if not vals:
                continue
            ys = [(i + 1) / len(vals) for i in range(len(vals))]
            plt.plot(vals, ys, label=model)
        plt.xscale("symlog", linthresh=0.1)
        plt.xlabel(f"{horizon:g}s position error [m]")
        plt.ylabel("empirical CDF")
        plt.title(f"Error CDF / tail at horizon {horizon:g}s")
        plt.grid(True, alpha=0.35)
        plt.legend(fontsize=8)
        plt.tight_layout()
        path = out_dir / f"horizon{horizon:g}_position_error_cdf.png"
        plt.savefig(path, dpi=160)
        plt.close()
        outputs[path.name] = str(path)

    model_for_classes = "m1/command_lag" if any(row["model"] == "m1/command_lag" for row in all_errors) else (models[0] if models else "")
    rows_class = [
        row
        for row in all_errors
        if row["model"] == model_for_classes and abs(float(row["horizon_s"]) - 3.0) < 1e-9
    ]
    if rows_class:
        classes = sorted({str(row.get("motion_class", "")) for row in rows_class})
        data = [[float(row["position_error_m"]) for row in rows_class if row.get("motion_class") == klass] for klass in classes]
        plt.figure(figsize=(12, 6))
        plt.boxplot(data, labels=classes, showfliers=True)
        plt.yscale("symlog", linthresh=0.1)
        plt.ylabel("3s position error [m]")
        plt.title(f"Long-tail by motion class ({model_for_classes})")
        plt.grid(True, axis="y", alpha=0.35)
        plt.xticks(rotation=30, ha="right")
        plt.tight_layout()
        path = out_dir / "horizon3_error_by_motion_class_boxplot.png"
        plt.savefig(path, dpi=160)
        plt.close()
        outputs[path.name] = str(path)
    return outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--articulation-source", default="auto", choices=["auto", "mtt", "trailer", "hardware"])
    parser.add_argument("--horizons", default="0.5,1,2,3,5,10")
    parser.add_argument("--sample-period-s", type=float, default=0.5)
    parser.add_argument("--force-rebuild", action="store_true")
    parser.add_argument("--allow-synthetic-tacho", action="store_true")
    parser.add_argument("--include-weak", action="store_true", help="Diagnostic mode: keep weak-ICP samples in prediction windows.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sessions = resolve_sessions(args.path)
    out_dir = args.output_dir or (sessions[0] / "motion_research" / "model_suite" if len(sessions) == 1 else Path("artifacts/results/motion_research/model_suite"))
    out_dir.mkdir(parents=True, exist_ok=True)
    horizons = [float(x) for x in args.horizons.split(",") if x.strip()]
    rows: list[dict] = []
    for session in sessions:
        rows.extend(load_research_rows(session, args))
    train_rows = [row for row in rows if parse_bool(row.get("research_quality_ok"))]
    m2_features = ["bias", "abs_v", "phi", "abs_phi", "phi_dot", "abs_phi_dot"]
    m3_features = m2_features + ["roll", "pitch", "accel", "imu_yaw_rate"]
    m2_coeff = fit_residual_model(train_rows, m2_features)
    m3_coeff = fit_residual_model(train_rows, m3_features)
    models = [
        ModelParams("m0/odom_delta", "odom_delta"),
        ModelParams("m0/measured_auto_exact", "direct", "tacho", "measured", "exact"),
        ModelParams("m0/measured_auto_tangent", "direct", "tacho", "measured", "tangent"),
        ModelParams("m0/measured_mtt_exact", "direct", "tacho", "mtt", "exact"),
        ModelParams("m0/measured_trailer_exact", "direct", "tacho", "trailer", "exact"),
        ModelParams("m0/command_exact", "direct", "cmd", "cmd", "exact"),
        ModelParams("m1/command_lag", "m1", "cmd", "cmd", "exact", tau_v_s=0.8, tau_phi_s=0.6),
        ModelParams("m2/curvature_residual", "m2", "cmd", "cmd", "exact", tau_v_s=0.8, tau_phi_s=0.6, residual_features=m2_features, residual_coefficients=m2_coeff),
        ModelParams("m3/imu_3d_residual", "m3", "cmd", "cmd", "exact", tau_v_s=0.8, tau_phi_s=0.6, residual_features=m3_features, residual_coefficients=m3_coeff),
    ]
    all_errors: list[dict] = []
    summaries: dict[str, dict] = {}
    for model in models:
        require_real_tacho = model.speed_source == "tacho" and not args.allow_synthetic_tacho
        errors = prediction_errors(
            rows,
            model,
            horizons_s=horizons,
            sample_period_s=args.sample_period_s,
            require_real_tacho=require_real_tacho,
            include_weak=args.include_weak,
        )
        model_dir = out_dir / model.name
        write_csv_rows(model_dir / "prediction_errors.csv", errors)
        summary = summarize_errors(errors)
        summary["model"] = model.__dict__
        (model_dir / "summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
        summaries[model.name] = summary
        all_errors.extend(errors)
    write_csv_rows(out_dir / "all_prediction_errors.csv", all_errors)
    final_summary = {
        "sessions": [s.name for s in sessions],
        "rows": len(rows),
        "models": list(summaries.keys()),
        "m2_features": m2_features,
        "m2_coefficients": m2_coeff,
        "m3_features": m3_features,
        "m3_coefficients": m3_coeff,
        "summaries": summaries,
        "outputs": {"all_errors_csv": str(out_dir / "all_prediction_errors.csv")},
    }
    final_summary["outputs"].update(write_suite_plots(out_dir, all_errors))
    (out_dir / "suite_summary.yaml").write_text(yaml.safe_dump(final_summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump({"output_dir": str(out_dir), "models": list(summaries.keys()), "rows": len(all_errors)}, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
