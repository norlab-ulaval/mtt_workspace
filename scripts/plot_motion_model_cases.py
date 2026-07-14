#!/usr/bin/env python3
"""Plot representative local motion-model cases from horizon prediction errors."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any

import yaml

import evaluate_motion_model_horizon as horizon
import plot_motion_model_rollout as rollout_plot


def read_error_rows(path: Path, target_horizon_s: float) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for raw in csv.DictReader(stream):
            try:
                h = float(raw["horizon_s"])
            except (KeyError, ValueError):
                continue
            if abs(h - target_horizon_s) > 1e-6:
                continue
            row = dict(raw)
            for key in ("start_offset_s", "position_error_m", "yaw_error_rad", "max_speed_ms", "max_articulation_rad"):
                row[key] = float(row[key])
            rows.append(row)
    return rows


def diverse_pick(
    rows: list[dict[str, Any]],
    *,
    key: str,
    reverse: bool,
    count: int,
    label: str,
    min_spacing_s: float,
) -> list[dict[str, Any]]:
    picked: list[dict[str, Any]] = []
    for row in sorted(rows, key=lambda r: float(r[key]), reverse=reverse):
        t = float(row["start_offset_s"])
        if any(abs(t - float(prev["start_offset_s"])) < min_spacing_s for prev in picked):
            continue
        case = dict(row)
        case["case_type"] = label
        picked.append(case)
        if len(picked) >= count:
            break
    return picked


def select_cases(rows: list[dict[str, Any]], count_per_type: int, min_spacing_s: float) -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []
    cases.extend(diverse_pick(rows, key="position_error_m", reverse=True, count=count_per_type, label="worst_error", min_spacing_s=min_spacing_s))
    cases.extend(diverse_pick(rows, key="max_articulation_rad", reverse=True, count=count_per_type, label="high_articulation", min_spacing_s=min_spacing_s))
    cases.extend(diverse_pick(rows, key="max_speed_ms", reverse=True, count=count_per_type, label="high_speed", min_spacing_s=min_spacing_s))

    straight = [row for row in rows if float(row["max_articulation_rad"]) < 0.035 and float(row["max_speed_ms"]) > 0.4]
    cases.extend(diverse_pick(straight, key="position_error_m", reverse=True, count=count_per_type, label="straight_worst", min_spacing_s=min_spacing_s))

    turns_good = [
        row for row in rows
        if float(row["max_articulation_rad"]) > 0.25 and float(row["position_error_m"]) < 0.5
    ]
    cases.extend(diverse_pick(turns_good, key="max_articulation_rad", reverse=True, count=count_per_type, label="turn_good", min_spacing_s=min_spacing_s))

    unique: list[dict[str, Any]] = []
    seen: set[tuple[str, int]] = set()
    for case in cases:
        key = (str(case["case_type"]), int(round(float(case["start_offset_s"]) * 10.0)))
        if key in seen:
            continue
        seen.add(key)
        unique.append(case)
    return unique


def slice_rows(rows: list[dict[str, Any]], start_t: float, end_t: float) -> list[dict[str, Any]]:
    return [row for row in rows if start_t <= float(row["t"]) <= end_t]


def command_articulation(row: dict[str, Any]) -> float | None:
    steer = horizon.parse_float(row.get("status_steer_normalized"))
    return horizon.clamp(steer, -1.0, 1.0) * horizon.MAX_ARTICULATION_RAD if steer is not None else None


def tach_speed(row: dict[str, Any]) -> float | None:
    speed = horizon.parse_float(row.get("tach_speed_ms"))
    return speed * horizon.tach_direction_sign(row.get("tach_direction")) if speed is not None else None


def write_tracking_plot(case_dir: Path, case_prefix: str, rows: list[dict[str, Any]]) -> str | None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    if not rows:
        return None

    t0 = float(rows[0]["t"])
    ts = [float(row["t"]) - t0 for row in rows]
    cmd_v = [horizon.parse_float(row.get("status_effective_linear_speed_command_ms")) or 0.0 for row in rows]
    meas_v = [tach_speed(row) or 0.0 for row in rows]
    cmd_phi = [command_articulation(row) or 0.0 for row in rows]
    meas_phi = [horizon.parse_float(row.get("articulation_rad")) or 0.0 for row in rows]

    plt.figure(figsize=(11, 6))
    plt.subplot(2, 1, 1)
    plt.plot(ts, cmd_v, label="command speed")
    plt.plot(ts, meas_v, label="tach speed", alpha=0.8)
    plt.ylabel("speed [m/s]")
    plt.grid(True, alpha=0.4)
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.plot(ts, cmd_phi, label="command articulation")
    plt.plot(ts, meas_phi, label="measured articulation", alpha=0.8)
    plt.xlabel("time from case start [s]")
    plt.ylabel("articulation [rad]")
    plt.grid(True, alpha=0.4)
    plt.legend()
    plt.tight_layout()
    path = case_dir / f"{case_prefix}_command_tracking.png"
    plt.savefig(path, dpi=150)
    plt.close()
    return str(path)


def write_case_csv(path: Path, case_rows: list[dict[str, Any]]) -> None:
    if not case_rows:
        return
    fieldnames = sorted({key for row in case_rows for key in row.keys()})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(case_rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--prediction-errors", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--target-horizon-s", type=float, default=3.0)
    parser.add_argument("--count-per-type", type=int, default=3)
    parser.add_argument("--min-spacing-s", type=float, default=20.0)
    parser.add_argument("--speed-source", choices=["tach", "status", "cmd"], default="status")
    parser.add_argument("--articulation-source", choices=["measured", "status", "cmd"], default="status")
    parser.add_argument("--curvature-model", choices=["exact", "tangent"], default="exact")
    parser.add_argument("--model-kind", choices=["direct", "m1_yaw_lag", "m1_phi_lag", "slip_heuristic"], default="m1_phi_lag")
    parser.add_argument("--tau-v-s", type=float, default=1.08)
    parser.add_argument("--tau-r-s", type=float, default=0.64)
    parser.add_argument("--tau-phi-s", type=float, default=0.64)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = args.session_dir.expanduser()
    out_dir = args.output_dir.expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = horizon.read_rows(session / "motion_model_validation" / "aligned_samples.csv")
    t0 = float(rows[0]["t"]) if rows else 0.0
    errors = read_error_rows(args.prediction_errors.expanduser(), args.target_horizon_s)
    cases = select_cases(errors, args.count_per_type, args.min_spacing_s)
    summary_rows: list[dict[str, Any]] = []

    for idx, case in enumerate(cases, start=1):
        case_type = str(case["case_type"])
        start_offset = float(case["start_offset_s"])
        start_t = t0 + start_offset
        end_t = start_t + args.target_horizon_s
        case_rows = slice_rows(rows, start_t, end_t)
        if len(case_rows) < 2:
            continue
        case_prefix = f"case_{idx:02d}_{case_type}_t{start_offset:.1f}s".replace(".", "p")
        case_dir = out_dir / case_prefix
        case_dir.mkdir(parents=True, exist_ok=True)

        rollout_args = argparse.Namespace(
            speed_source=args.speed_source,
            articulation_source=args.articulation_source,
            curvature_model=args.curvature_model,
            model_kind=args.model_kind,
            tau_v_s=args.tau_v_s,
            tau_r_s=args.tau_r_s,
            tau_phi_s=args.tau_phi_s,
        )
        traces = rollout_plot.rollout(case_rows, rollout_args)
        rollout_csv = case_dir / f"{case_prefix}_rollout.csv"
        rollout_plot.write_csv(rollout_csv, traces)
        outputs = rollout_plot.write_plots(case_dir, case_prefix, traces)
        tracking_plot = write_tracking_plot(case_dir, case_prefix, case_rows)
        if tracking_plot:
            outputs["command_tracking"] = tracking_plot
        raw_csv = case_dir / f"{case_prefix}_aligned_samples.csv"
        write_case_csv(raw_csv, case_rows)

        summary_rows.append(
            {
                "case": case_prefix,
                "case_type": case_type,
                "start_offset_s": start_offset,
                "horizon_s": args.target_horizon_s,
                "position_error_m": float(case["position_error_m"]),
                "yaw_error_rad": float(case["yaw_error_rad"]),
                "max_speed_ms": float(case["max_speed_ms"]),
                "max_articulation_rad": float(case["max_articulation_rad"]),
                "rows": len(case_rows),
                "rollout_csv": str(rollout_csv),
                "aligned_samples_csv": str(raw_csv),
                "outputs": outputs,
            }
        )

    summary_path = out_dir / "case_summary.yaml"
    summary = {
        "session": session.name,
        "settings": vars(args) | {
            "session_dir": str(session),
            "prediction_errors": str(args.prediction_errors),
            "output_dir": str(out_dir),
        },
        "case_count": len(summary_rows),
        "cases": summary_rows,
    }
    summary_path.write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")

    csv_path = out_dir / "case_summary.csv"
    if summary_rows:
        flat_rows = [{key: value for key, value in row.items() if key != "outputs"} for row in summary_rows]
        with csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(flat_rows[0].keys()))
            writer.writeheader()
            writer.writerows(flat_rows)

    print(yaml.safe_dump({"summary": str(summary_path), "csv": str(csv_path), "case_count": len(summary_rows)}, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
