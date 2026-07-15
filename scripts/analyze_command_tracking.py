#!/usr/bin/env python3
"""Analyze command-to-state tracking from aligned motion-model samples."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any

import yaml


MAX_ARTICULATION_RAD = math.radians(60.0)


def parse_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def tach_direction_sign(direction: Any) -> float:
    text = str(direction or "").strip().lower()
    return -1.0 if text in {"reverse", "backward", "rev", "-1"} else 1.0


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def read_rows(path: Path) -> list[dict[str, Any]]:
    numeric = {
        "t",
        "status_effective_linear_speed_command_ms",
        "status_steer_normalized",
        "cmd_linear_x",
        "cmd_angular_z",
        "tach_speed_ms",
        "articulation_rad",
    }
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for raw in csv.DictReader(stream):
            row: dict[str, Any] = dict(raw)
            for key in numeric:
                value = parse_float(raw.get(key))
                if value is not None:
                    row[key] = value
            if parse_float(row.get("t")) is not None:
                rows.append(row)
    rows.sort(key=lambda row: float(row["t"]))
    return rows


def sample_vectors(rows: list[dict[str, Any]]) -> dict[str, list[float]]:
    t: list[float] = []
    v_cmd: list[float] = []
    v_meas: list[float] = []
    phi_cmd: list[float] = []
    phi_meas: list[float] = []
    for row in rows:
        tc = parse_float(row.get("t"))
        vc = parse_float(row.get("status_effective_linear_speed_command_ms"))
        vm = parse_float(row.get("tach_speed_ms"))
        steer = parse_float(row.get("status_steer_normalized"))
        pm = parse_float(row.get("articulation_rad"))
        if tc is None or vc is None or vm is None or steer is None or pm is None:
            continue
        t.append(tc)
        v_cmd.append(vc)
        v_meas.append(vm * tach_direction_sign(row.get("tach_direction")))
        phi_cmd.append(clamp(steer, -1.0, 1.0) * MAX_ARTICULATION_RAD)
        phi_meas.append(pm)
    return {"t": t, "v_cmd": v_cmd, "v_meas": v_meas, "phi_cmd": phi_cmd, "phi_meas": phi_meas}


def interp(times: list[float], values: list[float], target: float) -> float | None:
    if not times or target < times[0] or target > times[-1]:
        return None
    import bisect

    idx = bisect.bisect_left(times, target)
    if idx == 0:
        return values[0]
    if idx >= len(times):
        return values[-1]
    t0, t1 = times[idx - 1], times[idx]
    if t1 <= t0:
        return values[idx]
    alpha = (target - t0) / (t1 - t0)
    return values[idx - 1] * (1.0 - alpha) + values[idx] * alpha


def metrics(a: list[float], b: list[float]) -> dict[str, float | int | None]:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if not pairs:
        return {"count": 0, "bias": None, "mae": None, "rmse": None}
    residuals = [x - y for x, y in pairs]
    return {
        "count": len(pairs),
        "bias": sum(residuals) / len(residuals),
        "mae": sum(abs(r) for r in residuals) / len(residuals),
        "rmse": math.sqrt(sum(r * r for r in residuals) / len(residuals)),
    }


def delay_sweep(
    times: list[float],
    cmd: list[float],
    measured: list[float],
    *,
    min_abs_signal: float,
    max_delay_s: float,
    step_s: float,
) -> tuple[dict[str, float | int | None], list[dict[str, float]]]:
    sweep: list[dict[str, float]] = []
    n = int(round((2.0 * max_delay_s) / step_s)) + 1
    for idx in range(n):
        delay = -max_delay_s + idx * step_s
        pred: list[float] = []
        obs: list[float] = []
        for t, y in zip(times, measured):
            x = interp(times, cmd, t - delay)
            if x is None:
                continue
            if abs(x) < min_abs_signal and abs(y) < min_abs_signal:
                continue
            pred.append(x)
            obs.append(y)
        m = metrics(pred, obs)
        if m["rmse"] is not None:
            sweep.append({"delay_s": delay, "rmse": float(m["rmse"]), "mae": float(m["mae"]), "bias": float(m["bias"])})
    best = min(sweep, key=lambda row: row["rmse"]) if sweep else {}
    return best, sweep


def write_plots(out_dir: Path, data: dict[str, list[float]], speed_sweep: list[dict[str, float]], phi_sweep: list[dict[str, float]]) -> dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}

    outputs: dict[str, str] = {}
    t0 = data["t"][0] if data["t"] else 0.0
    rel_t = [t - t0 for t in data["t"]]

    plt.figure(figsize=(12, 5))
    plt.plot(rel_t, data["v_cmd"], label="command speed")
    plt.plot(rel_t, data["v_meas"], label="tach speed", alpha=0.8)
    plt.xlabel("time [s]")
    plt.ylabel("speed [m/s]")
    plt.title("Command speed vs tachometer speed")
    plt.grid(True, alpha=0.4)
    plt.legend()
    plt.tight_layout()
    path = out_dir / "speed_command_tracking.png"
    plt.savefig(path, dpi=150)
    plt.close()
    outputs["speed_tracking"] = str(path)

    plt.figure(figsize=(12, 5))
    plt.plot(rel_t, data["phi_cmd"], label="command articulation")
    plt.plot(rel_t, data["phi_meas"], label="measured articulation", alpha=0.8)
    plt.xlabel("time [s]")
    plt.ylabel("articulation [rad]")
    plt.title("Command articulation vs measured articulation")
    plt.grid(True, alpha=0.4)
    plt.legend()
    plt.tight_layout()
    path = out_dir / "articulation_command_tracking.png"
    plt.savefig(path, dpi=150)
    plt.close()
    outputs["articulation_tracking"] = str(path)

    for name, sweep in (("speed", speed_sweep), ("articulation", phi_sweep)):
        if not sweep:
            continue
        plt.figure(figsize=(9, 5))
        plt.plot([row["delay_s"] for row in sweep], [row["rmse"] for row in sweep], marker=".")
        plt.xlabel("command delay [s]")
        plt.ylabel("RMSE")
        plt.title(f"{name} command-to-state delay sweep")
        plt.grid(True, alpha=0.4)
        plt.tight_layout()
        path = out_dir / f"{name}_delay_sweep.png"
        plt.savefig(path, dpi=150)
        plt.close()
        outputs[f"{name}_delay_sweep"] = str(path)

    return outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--max-delay-s", type=float, default=2.0)
    parser.add_argument("--delay-step-s", type=float, default=0.02)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = args.session_dir.expanduser()
    input_csv = session / "motion_model_validation" / "aligned_samples.csv"
    out_dir = args.output_dir.expanduser() if args.output_dir else session / "motion_model_validation" / "model_suite" / "command_tracking"
    out_dir.mkdir(parents=True, exist_ok=True)
    data = sample_vectors(read_rows(input_csv))
    speed_metrics = metrics(data["v_cmd"], data["v_meas"])
    phi_metrics = metrics(data["phi_cmd"], data["phi_meas"])
    best_speed_delay, speed_sweep = delay_sweep(
        data["t"],
        data["v_cmd"],
        data["v_meas"],
        min_abs_signal=0.1,
        max_delay_s=args.max_delay_s,
        step_s=args.delay_step_s,
    )
    best_phi_delay, phi_sweep = delay_sweep(
        data["t"],
        data["phi_cmd"],
        data["phi_meas"],
        min_abs_signal=0.02,
        max_delay_s=args.max_delay_s,
        step_s=args.delay_step_s,
    )
    outputs = write_plots(out_dir, data, speed_sweep, phi_sweep)
    summary = {
        "session": session.name,
        "input_csv": str(input_csv),
        "rows": len(data["t"]),
        "speed_command_minus_tach": speed_metrics,
        "articulation_command_minus_measured": phi_metrics,
        "best_speed_delay_s": best_speed_delay,
        "best_articulation_delay_s": best_phi_delay,
        "outputs": outputs,
    }
    summary_path = out_dir / "command_tracking_summary.yaml"
    summary_path.write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
