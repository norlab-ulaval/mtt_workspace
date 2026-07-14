#!/usr/bin/env python3
"""Generate DRIVE-style motion-model figures from calibrated MTT outputs."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path
from typing import Any, Iterable

import yaml


L_F_M = 0.9
L_R_M = 1.5
MAX_ARTICULATION_RAD = math.radians(60.0)
MIN_DT_S = 0.03
MAX_DT_S = 0.75


MODEL_ORDER = [
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


SHORT_LABELS = {
    "odom_delta": "odom\nbase",
    "M0_measured_exact": "M0\nmeas",
    "M1_measured_speed_yaw_scale": "M1\nmeas",
    "M2_measured_phi_calib": "M2\nmeas",
    "M3_measured_phi_delay": "M3\nmeas",
    "M4_measured_slip_residual": "M4\nmeas",
    "M0_command_exact": "M0\ncmd",
    "M1_command_lag": "M1\ncmd",
    "M2_command_calibrated": "M2\ncmd",
    "M3_command_curvature_residual": "M3\ncmd",
    "M4_command_twist_residual": "M4\ncmd",
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


def nominal_curvature(phi_rad: float) -> float:
    phi = clamp(phi_rad, -MAX_ARTICULATION_RAD, MAX_ARTICULATION_RAD)
    return math.sin(phi) / (L_F_M * math.cos(phi) + L_R_M)


def read_csv(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = []
        for raw in csv.DictReader(stream):
            row: dict[str, Any] = dict(raw)
            for key, value in raw.items():
                parsed = parse_float(value)
                if parsed is not None:
                    row[key] = parsed
            rows.append(row)
        return rows


def percentile(values: Iterable[float], pct: float) -> float | None:
    vals = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not vals:
        return None
    idx = int(round((len(vals) - 1) * pct / 100.0))
    return vals[max(0, min(len(vals) - 1, idx))]


def rmse(values: Iterable[float]) -> float | None:
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return None
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def dataset_csv_for_session(session: Path) -> Path:
    candidates = [
        session / "motion_model_validation" / "motion_research" / "datasets" / session.name / "dataset.csv",
        session / "motion_research" / "dataset.csv",
        session / "postprocess_dataset" / "motion_model_dataset.csv",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"no motion research dataset found under {session}")


def errors_csv_for_session(session: Path, moving_only: bool) -> Path:
    out_dir = session / "motion_model_validation" / "motion_research" / "calibrated_models"
    name = "calibrated_prediction_errors_moving_windows.csv" if moving_only else "calibrated_prediction_errors.csv"
    path = out_dir / name
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def body_velocity_rows(rows: list[dict[str, Any]]) -> list[dict[str, float]]:
    quality = [row for row in rows if parse_bool(row.get("research_quality_ok"))]
    quality.sort(key=lambda row: (str(row.get("session", "")), str(row.get("segment_id", "")), float(row.get("t", 0.0))))
    out: list[dict[str, float]] = []
    prev_by_segment: dict[tuple[str, str], dict[str, Any]] = {}
    for row in quality:
        key = (str(row.get("session", "")), str(row.get("segment_id", "")))
        prev = prev_by_segment.get(key)
        prev_by_segment[key] = row
        if prev is None:
            continue
        t0 = parse_float(prev.get("t"))
        t1 = parse_float(row.get("t"))
        x0 = parse_float(prev.get("icp_x"))
        y0 = parse_float(prev.get("icp_y"))
        yaw0 = parse_float(prev.get("icp_yaw"))
        x1 = parse_float(row.get("icp_x"))
        y1 = parse_float(row.get("icp_y"))
        yaw1 = parse_float(row.get("icp_yaw"))
        if None in (t0, t1, x0, y0, yaw0, x1, y1, yaw1):
            continue
        assert t0 is not None and t1 is not None
        assert x0 is not None and y0 is not None and yaw0 is not None
        assert x1 is not None and y1 is not None and yaw1 is not None
        dt = t1 - t0
        if dt < MIN_DT_S or dt > MAX_DT_S:
            continue
        dx = x1 - x0
        dy = y1 - y0
        c = math.cos(yaw0)
        s = math.sin(yaw0)
        v_long = (c * dx + s * dy) / dt
        v_lat = (-s * dx + c * dy) / dt
        wz = wrap_angle(yaw1 - yaw0) / dt
        cmd_v = parse_float(row.get("cmd_speed_ms")) or parse_float(row.get("cmd_linear_x")) or 0.0
        cmd_phi = parse_float(row.get("cmd_phi_rad")) or 0.0
        cmd_wz = cmd_v * nominal_curvature(cmd_phi)
        slip_v_long = cmd_v - v_long
        slip_wz = cmd_wz - wz
        slip_angle = math.atan2(v_lat, abs(v_long)) if abs(v_long) > 0.05 or abs(v_lat) > 0.05 else 0.0
        out.append(
            {
                "t": t1,
                "v_long": v_long,
                "v_lat": v_lat,
                "wz": wz,
                "cmd_v": cmd_v,
                "cmd_phi": cmd_phi,
                "cmd_wz": cmd_wz,
                "slip_v_long": slip_v_long,
                "slip_wz": slip_wz,
                "slip_angle_rad": slip_angle,
                "abs_slip_angle_deg": abs(math.degrees(slip_angle)),
                "speed_error_abs": abs(slip_v_long),
                "yaw_rate_error_abs": abs(slip_wz),
            }
        )
    return out


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


def plot_violin_translation_rotation(plt: Any, errors: list[dict[str, Any]], out_dir: Path, outputs: dict[str, str], horizon: float, reference: str) -> None:
    rows = [row for row in errors if row.get("reference") == reference and abs(float(row.get("horizon_s", -1.0)) - horizon) < 1e-9]
    present = [model for model in MODEL_ORDER if any(row.get("model") == model for row in rows)]
    if not present:
        return
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    metrics = [
        ("position_error_m", "Translation error [m]", 1.0),
        ("yaw_error_rad", "Rotation error [deg]", 180.0 / math.pi),
    ]
    labels = [SHORT_LABELS.get(model, model) for model in present]
    for ax, (key, ylabel, scale) in zip(axes, metrics):
        data = []
        for model in present:
            vals = [float(row[key]) * scale for row in rows if row.get("model") == model and parse_float(row.get(key)) is not None]
            data.append(vals if vals else [0.0])
        parts = ax.violinplot(data, showmedians=True, showextrema=False)
        for body in parts["bodies"]:
            body.set_alpha(0.62)
        ax.boxplot(data, showmeans=True, showfliers=False, whis=(5, 95), widths=0.18)
        ax.set_ylabel(ylabel)
        ax.set_yscale("symlog", linthresh=0.05 if key == "position_error_m" else 0.5)
        ax.grid(True, axis="y", alpha=0.3)
    axes[-1].set_xticks(range(1, len(labels) + 1))
    axes[-1].set_xticklabels(labels)
    fig.suptitle(f"DRIVE-style local RPE distributions, {horizon:g}s moving windows vs {reference.upper()}")
    savefig(plt, out_dir / f"drive_style_translation_rotation_violin_h{horizon:g}_{reference}.png", outputs)


def plot_median_iqr_crosses(plt: Any, errors: list[dict[str, Any]], out_dir: Path, outputs: dict[str, str], horizon: float, reference: str) -> None:
    rows = [row for row in errors if row.get("reference") == reference and abs(float(row.get("horizon_s", -1.0)) - horizon) < 1e-9]
    present = [model for model in MODEL_ORDER if any(row.get("model") == model for row in rows)]
    if not present:
        return
    fig, ax = plt.subplots(figsize=(9, 7))
    families = {
        "baseline": "#222222",
        "measured": "#2a9d8f",
        "command": "#e76f51",
    }
    for model in present:
        model_rows = [row for row in rows if row.get("model") == model]
        tx = [float(row["position_error_m"]) for row in model_rows]
        ry = [math.degrees(float(row["yaw_error_rad"])) for row in model_rows]
        if not tx or not ry:
            continue
        x_med = statistics.median(tx)
        y_med = statistics.median(ry)
        x_q1 = percentile(tx, 25.0) or x_med
        x_q3 = percentile(tx, 75.0) or x_med
        y_q1 = percentile(ry, 25.0) or y_med
        y_q3 = percentile(ry, 75.0) or y_med
        family = str(model_rows[0].get("family", ""))
        color = families.get(family, "#555555")
        ax.errorbar(
            x_med,
            y_med,
            xerr=[[x_med - x_q1], [x_q3 - x_med]],
            yerr=[[y_med - y_q1], [y_q3 - y_med]],
            fmt="o",
            capsize=3,
            color=color,
            label=family,
        )
        ax.text(x_med, y_med, " " + SHORT_LABELS.get(model, model).replace("\n", " "), fontsize=8, va="center")
    ax.set_xlabel("Translation error median + IQR [m]")
    ax.set_ylabel("Rotation error median + IQR [deg]")
    ax.set_title(f"Median/IQR model tradeoff, {horizon:g}s moving windows vs {reference.upper()}")
    ax.grid(True, alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    dedup: dict[str, Any] = {}
    for handle, label in zip(handles, labels):
        dedup.setdefault(label, handle)
    ax.legend(dedup.values(), dedup.keys(), loc="best")
    savefig(plt, out_dir / f"drive_style_translation_rotation_median_iqr_h{horizon:g}_{reference}.png", outputs)


def plot_command_error_hexbin(plt: Any, errors: list[dict[str, Any]], out_dir: Path, outputs: dict[str, str], horizon: float, reference: str) -> None:
    model = "M3_command_curvature_residual"
    rows = [
        row
        for row in errors
        if row.get("reference") == reference and row.get("model") == model and abs(float(row.get("horizon_s", -1.0)) - horizon) < 1e-9
    ]
    if not rows:
        return
    x = [float(row.get("max_abs_cmd_speed_ms", 0.0) or 0.0) for row in rows]
    y = [float(row.get("max_abs_cmd_phi_rad", 0.0) or 0.0) for row in rows]
    c = [float(row["position_error_m"]) for row in rows]
    fig, ax = plt.subplots(figsize=(8, 6))
    hb = ax.hexbin(x, y, C=c, reduce_C_function=statistics.mean, gridsize=22, mincnt=1, cmap="magma")
    cb = fig.colorbar(hb, ax=ax)
    cb.set_label(f"Mean {horizon:g}s position error [m]")
    ax.set_xlabel("max |command speed| in window [m/s]")
    ax.set_ylabel("max |command articulation| in window [rad]")
    ax.set_title(f"Command-space error hexbin ({model})")
    savefig(plt, out_dir / f"drive_style_command_error_hexbin_h{horizon:g}_{reference}.png", outputs)


def plot_velocity_space(plt: Any, body_rows: list[dict[str, float]], out_dir: Path, outputs: dict[str, str]) -> None:
    moving = [row for row in body_rows if abs(row["v_long"]) > 0.05 or abs(row["wz"]) > 0.02]
    if not moving:
        return
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    hb0 = axes[0].hexbin(
        [row["cmd_v"] for row in moving],
        [row["cmd_wz"] for row in moving],
        gridsize=34,
        mincnt=1,
        cmap="Blues",
    )
    fig.colorbar(hb0, ax=axes[0], label="samples")
    axes[0].set_xlabel("command longitudinal velocity [m/s]")
    axes[0].set_ylabel("command angular velocity [rad/s]")
    axes[0].set_title("Commanded body velocity coverage")

    hb1 = axes[1].hexbin(
        [row["v_long"] for row in moving],
        [row["wz"] for row in moving],
        gridsize=34,
        mincnt=1,
        cmap="Greens",
    )
    fig.colorbar(hb1, ax=axes[1], label="samples")
    axes[1].set_xlabel("observed longitudinal velocity [m/s]")
    axes[1].set_ylabel("observed angular velocity [rad/s]")
    axes[1].set_title("Resulting body velocity coverage")
    savefig(plt, out_dir / "drive_style_command_vs_resulting_velocity_space.png", outputs)


def plot_longitudinal_vs_angular_slip(plt: Any, body_rows: list[dict[str, float]], out_dir: Path, outputs: dict[str, str]) -> None:
    moving = [row for row in body_rows if abs(row["v_long"]) > 0.05 or abs(row["wz"]) > 0.02]
    if not moving:
        return
    fig, ax = plt.subplots(figsize=(9, 6))
    hb = ax.hexbin(
        [row["wz"] for row in moving],
        [row["v_long"] for row in moving],
        C=[row["abs_slip_angle_deg"] for row in moving],
        reduce_C_function=statistics.mean,
        gridsize=36,
        mincnt=1,
        cmap="viridis",
    )
    cb = fig.colorbar(hb, ax=ax)
    cb.set_label("mean |slip angle| [deg]")
    ax.set_xlabel("observed angular velocity [rad/s]")
    ax.set_ylabel("observed longitudinal velocity [m/s]")
    ax.set_title("Longitudinal velocity vs angular velocity, colored by slip angle")
    savefig(plt, out_dir / "drive_style_vlong_vs_wz_slip_angle_hexbin.png", outputs)


def plot_slip_error_hexbin(plt: Any, body_rows: list[dict[str, float]], out_dir: Path, outputs: dict[str, str]) -> None:
    moving = [row for row in body_rows if abs(row["cmd_v"]) > 0.05 or abs(row["cmd_wz"]) > 0.02]
    if not moving:
        return
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    hb0 = axes[0].hexbin(
        [row["cmd_v"] for row in moving],
        [row["cmd_wz"] for row in moving],
        C=[row["speed_error_abs"] for row in moving],
        reduce_C_function=statistics.mean,
        gridsize=32,
        mincnt=1,
        cmap="inferno",
    )
    fig.colorbar(hb0, ax=axes[0], label="mean |v_cmd - v_obs| [m/s]")
    axes[0].set_xlabel("command longitudinal velocity [m/s]")
    axes[0].set_ylabel("command angular velocity [rad/s]")
    axes[0].set_title("Longitudinal slip/error over command space")

    hb1 = axes[1].hexbin(
        [row["cmd_v"] for row in moving],
        [row["cmd_wz"] for row in moving],
        C=[row["yaw_rate_error_abs"] for row in moving],
        reduce_C_function=statistics.mean,
        gridsize=32,
        mincnt=1,
        cmap="inferno",
    )
    fig.colorbar(hb1, ax=axes[1], label="mean |w_cmd - w_obs| [rad/s]")
    axes[1].set_xlabel("command longitudinal velocity [m/s]")
    axes[1].set_ylabel("command angular velocity [rad/s]")
    axes[1].set_title("Angular slip/error over command space")
    savefig(plt, out_dir / "drive_style_command_space_slip_error_hexbin.png", outputs)


def plot_slip_angle_histogram(plt: Any, body_rows: list[dict[str, float]], out_dir: Path, outputs: dict[str, str]) -> None:
    moving = [row for row in body_rows if abs(row["v_long"]) > 0.05 or abs(row["wz"]) > 0.02]
    if not moving:
        return
    fig, ax = plt.subplots(figsize=(8, 5))
    vals = [math.degrees(row["slip_angle_rad"]) for row in moving]
    ax.hist(vals, bins=70, color="#577590", alpha=0.82)
    ax.axvline(statistics.median(vals), color="black", linewidth=1.4, label="median")
    ax.set_xlabel("slip angle beta [deg]")
    ax.set_ylabel("samples")
    ax.set_title("Observed slip-angle distribution")
    ax.legend()
    savefig(plt, out_dir / "drive_style_slip_angle_histogram.png", outputs)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--horizon-s", type=float, default=3.0)
    parser.add_argument("--reference", choices=["icp", "odom"], default="icp")
    parser.add_argument("--all-windows", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = args.session.expanduser().resolve()
    out_dir = args.output_dir or (
        session / "motion_model_validation" / "motion_research" / "calibrated_models" / "drive_style_figures"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    dataset_rows = read_csv(dataset_csv_for_session(session))
    errors = read_csv(errors_csv_for_session(session, moving_only=not args.all_windows))
    body_rows = body_velocity_rows(dataset_rows)
    plt = setup_matplotlib()
    outputs: dict[str, str] = {}

    plot_violin_translation_rotation(plt, errors, out_dir, outputs, args.horizon_s, args.reference)
    plot_median_iqr_crosses(plt, errors, out_dir, outputs, args.horizon_s, args.reference)
    plot_command_error_hexbin(plt, errors, out_dir, outputs, args.horizon_s, args.reference)
    plot_velocity_space(plt, body_rows, out_dir, outputs)
    plot_longitudinal_vs_angular_slip(plt, body_rows, out_dir, outputs)
    plot_slip_error_hexbin(plt, body_rows, out_dir, outputs)
    plot_slip_angle_histogram(plt, body_rows, out_dir, outputs)

    slip_angles = [
        row["abs_slip_angle_deg"]
        for row in body_rows
        if math.hypot(row["v_long"], row["v_lat"]) > 0.2 or abs(row["wz"]) > 0.05
    ]
    summary = {
        "session": session.name,
        "reference": args.reference,
        "horizon_s": args.horizon_s,
        "window_set": "all" if args.all_windows else "moving",
        "dataset_rows": len(dataset_rows),
        "body_velocity_rows": len(body_rows),
        "moving_body_velocity_rows": len(slip_angles),
        "slip_angle_abs_deg": {
            "median": statistics.median(slip_angles) if slip_angles else None,
            "p95": percentile(slip_angles, 95.0),
            "rmse": rmse(slip_angles),
        },
        "outputs": outputs,
        "paper_style_notes": [
            "Translation and rotation errors are plotted separately, following the DRIVE paper's MRMSE split.",
            "Hexbin plots show command/input-space coverage and error concentration.",
            "Slip angle is approximated from ICP body-frame lateral and longitudinal velocity.",
        ],
    }
    (out_dir / "drive_style_summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
