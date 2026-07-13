#!/usr/bin/env python3
"""Tune simple local MTT motion-model parameters on validated ICP windows."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Any

import yaml


DEFAULT_WHEELBASE_M = 2.4
DEFAULT_MIN_TURN_SPEED_MS = 0.1


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


def tach_direction_sign(direction: str | None) -> float:
    text = str(direction or "").strip().lower()
    return -1.0 if text in {"reverse", "backward", "rev", "-1"} else 1.0


class RowSeries:
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
        for row in csv.DictReader(stream):
            parsed: dict[str, Any] = dict(row)
            for key in (
                "t",
                "icp_x",
                "icp_y",
                "icp_heading",
                "icp_linear_x",
                "icp_angular_z",
                "tach_speed_ms",
                "articulation_rad",
            ):
                value = parse_float(row.get(key))
                if value is not None:
                    parsed[key] = value
            rows.append(parsed)
    return rows


def valid_segments(
    rows: list[dict[str, Any]],
    *,
    max_icp_step_m: float,
    max_icp_gap_s: float,
    max_icp_speed_ms: float,
    max_icp_yaw_rate_rad_s: float,
) -> list[list[dict[str, Any]]]:
    segments: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    prev: tuple[float, float, float] | None = None
    for row in rows:
        t = parse_float(row.get("t"))
        ix = parse_float(row.get("icp_x"))
        iy = parse_float(row.get("icp_y"))
        ih = parse_float(row.get("icp_heading"))
        if None in (t, ix, iy, ih):
            continue
        assert t is not None and ix is not None and iy is not None
        good = True
        if prev is not None:
            pt, px, py = prev
            if t - pt > max_icp_gap_s or math.hypot(ix - px, iy - py) > max_icp_step_m:
                good = False
        v = parse_float(row.get("icp_linear_x"))
        w = parse_float(row.get("icp_angular_z"))
        if v is not None and abs(v) > max_icp_speed_ms:
            good = False
        if w is not None and abs(w) > max_icp_yaw_rate_rad_s:
            good = False
        if not good:
            if current:
                segments.append(current)
            current = []
        else:
            current.append(row)
        prev = (t, ix, iy)
    if current:
        segments.append(current)
    return segments


def make_windows(segments: list[list[dict[str, Any]]], window_s: float, min_duration_s: float) -> list[list[dict[str, Any]]]:
    windows: list[list[dict[str, Any]]] = []
    for segment in segments:
        start = 0
        while start < len(segment):
            t0 = float(segment[start]["t"])
            end = start
            while end < len(segment) and float(segment[end]["t"]) <= t0 + window_s:
                end += 1
            win = segment[start:end]
            if len(win) >= 10 and float(win[-1]["t"]) - float(win[0]["t"]) >= min_duration_s:
                distance = 0.0
                prev_xy = None
                for row in win:
                    xy = (float(row["icp_x"]), float(row["icp_y"]))
                    if prev_xy is not None:
                        distance += math.hypot(xy[0] - prev_xy[0], xy[1] - prev_xy[1])
                    prev_xy = xy
                if distance >= 3.0:
                    windows.append(win)
            start = max(end, start + 1)
    return windows


def simulate_window(
    window: list[dict[str, Any]],
    series: RowSeries,
    *,
    speed_scale: float,
    yaw_gain: float,
    articulation_delay_s: float,
    wheelbase_m: float,
) -> tuple[float, float, float]:
    x = float(window[0]["icp_x"])
    y = float(window[0]["icp_y"])
    yaw = float(window[0]["icp_heading"])
    prev_t = float(window[0]["t"])
    xy_errors: list[float] = []
    yaw_errors: list[float] = []
    yaw_rate_errors: list[float] = []
    for row in window:
        t = float(row["t"])
        dt = max(0.0, min(t - prev_t, 1.0))
        delayed = series.nearest(t - articulation_delay_s, 0.08) or row
        speed = parse_float(row.get("tach_speed_ms"))
        articulation = parse_float(delayed.get("articulation_rad"))
        if speed is not None and articulation is not None and dt > 0.0:
            signed_speed = speed_scale * speed * tach_direction_sign(str(row.get("tach_direction") or ""))
            yaw_rate = 0.0
            if abs(signed_speed) >= DEFAULT_MIN_TURN_SPEED_MS:
                yaw_rate = yaw_gain * signed_speed * math.tan(articulation) / wheelbase_m
            dtheta = yaw_rate * dt
            heading_mid = yaw + 0.5 * dtheta
            x += signed_speed * dt * math.cos(heading_mid)
            y += signed_speed * dt * math.sin(heading_mid)
            yaw = wrap_angle(yaw + dtheta)
            icp_yaw_rate = parse_float(row.get("icp_angular_z"))
            if icp_yaw_rate is not None:
                yaw_rate_errors.append(yaw_rate - icp_yaw_rate)
        xy_errors.append(math.hypot(x - float(row["icp_x"]), y - float(row["icp_y"])))
        yaw_errors.append(wrap_angle(yaw - float(row["icp_heading"])))
        prev_t = t
    pos_rmse = math.sqrt(sum(e * e for e in xy_errors) / len(xy_errors))
    yaw_rmse = math.sqrt(sum(e * e for e in yaw_errors) / len(yaw_errors))
    yaw_rate_rmse = (
        math.sqrt(sum(e * e for e in yaw_rate_errors) / len(yaw_rate_errors))
        if yaw_rate_errors
        else math.nan
    )
    return pos_rmse, yaw_rmse, yaw_rate_rmse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--window-s", type=float, default=30.0)
    parser.add_argument("--max-icp-step-m", type=float, default=1.0)
    parser.add_argument("--max-icp-gap-s", type=float, default=0.5)
    parser.add_argument("--max-icp-speed-ms", type=float, default=8.0)
    parser.add_argument("--max-icp-yaw-rate-rad-s", type=float, default=2.5)
    parser.add_argument("--wheelbase-m", type=float, default=DEFAULT_WHEELBASE_M)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = args.session_dir.expanduser().resolve()
    out_dir = session / "motion_model_validation"
    csv_path = out_dir / "aligned_samples.csv"
    rows = read_rows(csv_path)
    segments = valid_segments(
        rows,
        max_icp_step_m=args.max_icp_step_m,
        max_icp_gap_s=args.max_icp_gap_s,
        max_icp_speed_ms=args.max_icp_speed_ms,
        max_icp_yaw_rate_rad_s=args.max_icp_yaw_rate_rad_s,
    )
    windows = make_windows(segments, args.window_s, min_duration_s=5.0)
    series = RowSeries(rows)

    results = []
    for speed_scale in (0.80, 0.90, 1.00, 1.10, 1.20):
        for yaw_gain in (-1.5, -1.0, -0.75, -0.5, 0.0, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0):
            for delay_s in (-0.30, -0.20, -0.10, 0.0, 0.10, 0.20, 0.30, 0.50):
                pos = []
                yaw = []
                yaw_rate = []
                for window in windows:
                    p, y, w = simulate_window(
                        window,
                        series,
                        speed_scale=speed_scale,
                        yaw_gain=yaw_gain,
                        articulation_delay_s=delay_s,
                        wheelbase_m=args.wheelbase_m,
                    )
                    pos.append(p)
                    yaw.append(y)
                    if math.isfinite(w):
                        yaw_rate.append(w)
                if not pos:
                    continue
                results.append(
                    {
                        "speed_scale": speed_scale,
                        "yaw_gain": yaw_gain,
                        "articulation_delay_s": delay_s,
                        "window_count": len(pos),
                        "position_rmse_mean_m": sum(pos) / len(pos),
                        "position_rmse_p95_m": sorted(pos)[int(0.95 * (len(pos) - 1))],
                        "yaw_rmse_mean_rad": sum(yaw) / len(yaw),
                        "yaw_rate_rmse_mean_rad_s": sum(yaw_rate) / len(yaw_rate) if yaw_rate else None,
                    }
                )

    results.sort(key=lambda row: (row["position_rmse_mean_m"], row["yaw_rmse_mean_rad"]))
    sweep_csv = out_dir / "windowed_parameter_sweep.csv"
    if results:
        with sweep_csv.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(results[0].keys()))
            writer.writeheader()
            writer.writerows(results)
    summary = {
        "session": session.name,
        "input": str(csv_path),
        "settings": vars(args) | {"session_dir": str(session)},
        "segments": len(segments),
        "windows": len(windows),
        "best": results[:20],
        "outputs": {
            "csv": str(sweep_csv),
            "summary": str(out_dir / "windowed_parameter_sweep.yaml"),
        },
    }
    Path(summary["outputs"]["summary"]).write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
