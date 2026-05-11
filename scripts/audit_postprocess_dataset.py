#!/usr/bin/env python3
"""Audit postprocess_dataset CSV exports with physical consistency checks."""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
from pathlib import Path
from typing import Any

import yaml


REQUIRED_COLUMNS = {
    "t",
    "bag_time_offset_s",
    "icp_x",
    "icp_y",
    "icp_z",
    "icp_qx",
    "icp_qy",
    "icp_qz",
    "icp_qw",
    "odom_x",
    "odom_y",
    "odom_yaw",
    "imu_angular_velocity_z",
    "imu_linear_acceleration_x",
    "imu_linear_acceleration_y",
    "imu_linear_acceleration_z",
    "cmd_linear_x",
    "cmd_angular_z",
    "tach_speed_ms",
    "tach_source",
    "tach_is_synthetic",
    "mtt_articulation_angle",
    "trailer_articulation_angle",
    "trailer_pose_x",
    "trailer_pose_y",
    "trailer_confidence",
    "has_icp",
    "has_odom",
    "has_tacho",
    "has_real_tacho",
    "has_cmd_sim_tacho",
    "has_imu",
    "has_trailer_pose",
    "has_trailer_angle",
}

HEAVY_TOPICS = {
    "/merged_points",
    "/merged_points_raw",
    "/merged_points_filtered",
    "/hesai_lidar/points",
    "/rsairy_ns/points",
    "/trailer/trailer_roi_cloud",
    "/trailer/articulation_roi_cloud",
    "/mapping/map",
}


def infer_workspace_root() -> Path:
    env_workspace = os.environ.get("WORKSPACE")
    if env_workspace:
        return Path(env_workspace).resolve()
    script_path = Path(__file__).resolve()
    for candidate in [script_path.parent, *script_path.parents]:
        if (candidate / "src").exists() and (candidate / "demos").exists():
            return candidate
    return script_path.parent


def resolve_sessions(path_value: str) -> list[Path]:
    path = Path(path_value).expanduser().resolve()
    if (path / "postprocess_dataset" / "dataset.csv").exists():
        return [path]
    if (path / "bag" / "metadata.yaml").exists():
        return [path]
    sessions = sorted(p.parent.parent for p in path.glob("*/bag/metadata.yaml"))
    if sessions:
        return sessions
    csv_sessions = sorted(p.parent.parent for p in path.glob("*/postprocess_dataset/dataset.csv"))
    if csv_sessions:
        return csv_sessions
    raise SystemExit(f"Could not resolve sessions from {path}")


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception as exc:  # noqa: BLE001
        return {"_load_error": str(exc)}


def parse_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def parse_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def read_dataset(csv_path: Path) -> tuple[list[dict[str, Any]], list[str]]:
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = [dict(row) for row in reader]
        return rows, list(reader.fieldnames or [])


def quaternion_yaw(row: dict[str, Any], prefix: str) -> float | None:
    x = parse_float(row.get(f"{prefix}_qx"))
    y = parse_float(row.get(f"{prefix}_qy"))
    z = parse_float(row.get(f"{prefix}_qz"))
    w = parse_float(row.get(f"{prefix}_qw"))
    if None in (x, y, z, w):
        return None
    assert x is not None and y is not None and z is not None and w is not None
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((pct / 100.0) * (len(ordered) - 1)))))
    return ordered[index]


def rmse(errors: list[float]) -> float | None:
    if not errors:
        return None
    return math.sqrt(sum(error * error for error in errors) / len(errors))


def finite_stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"count": 0, "min": None, "mean": None, "median": None, "max": None}
    return {
        "count": len(values),
        "min": min(values),
        "mean": sum(values) / len(values),
        "median": statistics.median(values),
        "max": max(values),
    }


def derive_xy_kinematics(
    rows: list[dict[str, Any]],
    x_key: str,
    y_key: str,
    yaw_key: str | None,
) -> list[dict[str, float]]:
    out: list[dict[str, float]] = []
    previous: dict[str, float] | None = None
    for row in rows:
        t = parse_float(row.get("t"))
        x = parse_float(row.get(x_key))
        y = parse_float(row.get(y_key))
        yaw = parse_float(row.get(yaw_key)) if yaw_key else quaternion_yaw(row, "icp")
        if None in (t, x, y, yaw):
            continue
        assert t is not None and x is not None and y is not None and yaw is not None
        sample = {"t": t, "x": x, "y": y, "yaw": yaw, "speed": 0.0, "yaw_rate": 0.0}
        if previous is not None:
            dt = t - previous["t"]
            if dt > 1e-6:
                dx = x - previous["x"]
                dy = y - previous["y"]
                heading_mid = wrap_angle(0.5 * (yaw + previous["yaw"]))
                sample["speed"] = (dx * math.cos(heading_mid) + dy * math.sin(heading_mid)) / dt
                sample["yaw_rate"] = wrap_angle(yaw - previous["yaw"]) / dt
        out.append(sample)
        previous = sample
    return out


class Series:
    def __init__(self, rows: list[dict[str, float]]):
        self.rows = sorted(rows, key=lambda row: row["t"])
        self.times = [row["t"] for row in self.rows]

    def nearest(self, t: float, tolerance_s: float) -> dict[str, float] | None:
        if not self.rows:
            return None
        lo = 0
        hi = len(self.times)
        while lo < hi:
            mid = (lo + hi) // 2
            if self.times[mid] < t:
                lo = mid + 1
            else:
                hi = mid
        candidates = []
        if lo < len(self.rows):
            candidates.append(self.rows[lo])
        if lo:
            candidates.append(self.rows[lo - 1])
        best = min(candidates, key=lambda row: abs(row["t"] - t))
        return best if abs(best["t"] - t) <= tolerance_s else None


def coverage(rows: list[dict[str, Any]], key: str) -> float:
    if not rows:
        return 0.0
    return sum(1 for row in rows if parse_bool(row.get(key))) / len(rows)


def compute_audit(session_dir: Path, rows: list[dict[str, Any]], columns: list[str]) -> dict[str, Any]:
    summary = load_yaml(session_dir / "postprocess_dataset" / "summary.yaml")
    icp_summary = load_yaml(session_dir / "offline_icp" / "summary.yaml")
    enriched_meta = load_yaml(session_dir / "postprocess_dataset" / "enriched_bag" / "metadata.yaml")

    times = [parse_float(row.get("t")) for row in rows]
    times = [value for value in times if value is not None]
    deltas = [b - a for a, b in zip(times, times[1:])]
    non_monotonic = sum(1 for delta in deltas if delta < -1e-9)
    duplicate_or_zero = sum(1 for delta in deltas if abs(delta) <= 1e-9)
    large_gaps = [delta for delta in deltas if delta > 0.5]

    icp_kin = derive_xy_kinematics(rows, "icp_x", "icp_y", None)
    odom_kin = derive_xy_kinematics(rows, "odom_x", "odom_y", "odom_yaw")
    icp_series = Series(icp_kin)
    odom_series = Series(odom_kin)

    xy_errors: list[float] = []
    yaw_errors: list[float] = []
    speed_errors: list[float] = []
    yaw_rate_errors: list[float] = []
    imu_yaw_rate_errors: list[float] = []
    tach_speed_errors: list[float] = []
    cmd_speed_errors: list[float] = []
    sign_total = 0
    sign_matches = 0

    for row in rows:
        t = parse_float(row.get("t"))
        if t is None:
            continue
        icp = icp_series.nearest(t, 0.05)
        odom = odom_series.nearest(t, 0.05)
        if icp and odom:
            xy_errors.append(math.hypot(odom["x"] - icp["x"], odom["y"] - icp["y"]))
            yaw_errors.append(wrap_angle(odom["yaw"] - icp["yaw"]))
            speed_errors.append(odom["speed"] - icp["speed"])
            yaw_rate_errors.append(odom["yaw_rate"] - icp["yaw_rate"])
        if icp:
            imu_yaw = parse_float(row.get("imu_angular_velocity_z"))
            tach_speed = parse_float(row.get("tach_speed_ms"))
            cmd_speed = parse_float(row.get("cmd_linear_x"))
            if imu_yaw is not None:
                imu_yaw_rate_errors.append(imu_yaw - icp["yaw_rate"])
            if tach_speed is not None:
                tach_speed_errors.append(tach_speed - icp["speed"])
                if abs(tach_speed) > 0.05 and abs(icp["speed"]) > 0.05:
                    sign_total += 1
                    sign_matches += int(math.copysign(1.0, tach_speed) == math.copysign(1.0, icp["speed"]))
            if cmd_speed is not None:
                cmd_speed_errors.append(cmd_speed - icp["speed"])

    imu_acc_norms = []
    for row in rows:
        ax = parse_float(row.get("imu_linear_acceleration_x"))
        ay = parse_float(row.get("imu_linear_acceleration_y"))
        az = parse_float(row.get("imu_linear_acceleration_z"))
        if None not in (ax, ay, az):
            assert ax is not None and ay is not None and az is not None
            imu_acc_norms.append(math.sqrt(ax * ax + ay * ay + az * az))

    articulation_values = [
        value for value in (parse_float(row.get("mtt_articulation_angle")) for row in rows) if value is not None
    ]
    trailer_angle_values = [
        value for value in (parse_float(row.get("trailer_articulation_angle")) for row in rows) if value is not None
    ]
    trailer_angle_errors = []
    for row in rows:
        mtt_angle = parse_float(row.get("mtt_articulation_angle"))
        trailer_angle = parse_float(row.get("trailer_articulation_angle"))
        if mtt_angle is not None and trailer_angle is not None:
            trailer_angle_errors.append(wrap_angle(trailer_angle - mtt_angle))

    enriched_counts = {}
    for item in enriched_meta.get("rosbag2_bagfile_information", {}).get("topics_with_message_count", []):
        meta = item.get("topic_metadata", {})
        enriched_counts[str(meta.get("name", ""))] = int(item.get("message_count", 0))
    heavy_recorded = sorted(topic for topic in enriched_counts if topic in HEAVY_TOPICS and enriched_counts[topic] > 0)

    grade = grade_audit(
        rows=rows,
        non_monotonic=non_monotonic,
        missing_columns=sorted(REQUIRED_COLUMNS.difference(columns)),
        icp_ratio=coverage(rows, "has_icp"),
        tacho_sign_agreement=(sign_matches / sign_total) if sign_total else None,
        heavy_recorded=heavy_recorded,
    )

    return {
        "session": session_dir.name,
        "status": "ok" if rows else "missing_dataset_rows",
        "dataset_csv": str(session_dir / "postprocess_dataset" / "dataset.csv"),
        "row_count": len(rows),
        "column_count": len(columns),
        "missing_required_columns": sorted(REQUIRED_COLUMNS.difference(columns)),
        "time": {
            "start": times[0] if times else None,
            "end": times[-1] if times else None,
            "duration_s": (times[-1] - times[0]) if len(times) >= 2 else 0.0,
            "non_monotonic_steps": non_monotonic,
            "duplicate_or_zero_steps": duplicate_or_zero,
            "dt_mean_s": (sum(deltas) / len(deltas)) if deltas else None,
            "dt_p95_s": percentile(deltas, 95.0),
            "dt_max_s": max(deltas) if deltas else None,
            "large_gaps_over_0_5_s": len(large_gaps),
        },
        "coverage": {
            "icp": coverage(rows, "has_icp"),
            "odom": coverage(rows, "has_odom"),
            "tacho": coverage(rows, "has_tacho"),
            "real_tacho": coverage(rows, "has_real_tacho"),
            "cmd_sim_tacho": coverage(rows, "has_cmd_sim_tacho"),
            "imu": coverage(rows, "has_imu"),
            "trailer_pose": coverage(rows, "has_trailer_pose"),
            "trailer_angle": coverage(rows, "has_trailer_angle"),
        },
        "icp_vs_odom": {
            "paired_samples": len(xy_errors),
            "xy_rmse_m": rmse(xy_errors),
            "xy_median_m": statistics.median(xy_errors) if xy_errors else None,
            "xy_p95_m": percentile(xy_errors, 95.0),
            "yaw_rmse_rad": rmse(yaw_errors),
            "speed_rmse_ms": rmse(speed_errors),
            "yaw_rate_rmse_rad_s": rmse(yaw_rate_errors),
        },
        "imu": {
            "acc_norm_ms2": finite_stats(imu_acc_norms),
            "acc_norm_error_from_g_median_ms2": (
                statistics.median([abs(value - 9.81) for value in imu_acc_norms]) if imu_acc_norms else None
            ),
            "yaw_rate_vs_icp_rmse_rad_s": rmse(imu_yaw_rate_errors),
        },
        "tacho_cmd_vs_icp": {
            "tach_speed_rmse_ms": rmse(tach_speed_errors),
            "cmd_speed_rmse_ms": rmse(cmd_speed_errors),
            "tach_sign_agreement": (sign_matches / sign_total) if sign_total else None,
            "tach_sign_checks": sign_total,
        },
        "articulation_trailer": {
            "mtt_articulation_rad": finite_stats(articulation_values),
            "trailer_articulation_rad": finite_stats(trailer_angle_values),
            "trailer_minus_mtt_rmse_rad": rmse(trailer_angle_errors),
            "trailer_minus_mtt_median_rad": statistics.median(trailer_angle_errors) if trailer_angle_errors else None,
            "mtt_outside_70deg_count": sum(1 for value in articulation_values if abs(value) > math.radians(70.0)),
            "trailer_outside_70deg_count": sum(1 for value in trailer_angle_values if abs(value) > math.radians(70.0)),
        },
        "outputs": {
            "postprocess_summary_status": summary.get("status"),
            "postprocess_grade": (summary.get("quality") or {}).get("grade"),
            "offline_icp_status": icp_summary.get("status"),
            "offline_icp_mode": icp_summary.get("mode"),
            "map_size_bytes": icp_summary.get("map_size_bytes"),
            "trajectory_size_bytes": icp_summary.get("trajectory_size_bytes"),
            "enriched_heavy_topics_recorded": heavy_recorded,
            "enriched_topic_count": len(enriched_counts),
        },
        "quality": {
            "grade": grade,
            "notes": quality_notes(rows, non_monotonic, large_gaps, heavy_recorded, sign_total, sign_matches),
        },
    }


def grade_audit(
    rows: list[dict[str, Any]],
    non_monotonic: int,
    missing_columns: list[str],
    icp_ratio: float,
    tacho_sign_agreement: float | None,
    heavy_recorded: list[str],
) -> str:
    if not rows or non_monotonic or missing_columns or heavy_recorded:
        return "weak"
    odom_ratio = coverage(rows, "has_odom")
    tacho_ratio = coverage(rows, "has_tacho")
    imu_ratio = coverage(rows, "has_imu")
    trailer_ratio = max(coverage(rows, "has_trailer_pose"), coverage(rows, "has_trailer_angle"))
    sign_ok = tacho_sign_agreement is None or tacho_sign_agreement >= 0.85
    if icp_ratio > 0.80 and odom_ratio > 0.80 and tacho_ratio > 0.80 and imu_ratio > 0.60 and trailer_ratio > 0.30 and sign_ok:
        return "excellent"
    if icp_ratio > 0.50 and odom_ratio > 0.50 and tacho_ratio > 0.50 and sign_ok:
        return "good"
    if icp_ratio > 0.20 or odom_ratio > 0.20 or tacho_ratio > 0.20:
        return "usable"
    return "weak"


def quality_notes(
    rows: list[dict[str, Any]],
    non_monotonic: int,
    large_gaps: list[float],
    heavy_recorded: list[str],
    sign_total: int,
    sign_matches: int,
) -> list[str]:
    notes: list[str] = []
    if not rows:
        notes.append("dataset_csv_empty")
    if non_monotonic:
        notes.append("timestamps_not_monotonic")
    if large_gaps:
        notes.append("large_timestamp_gaps")
    if heavy_recorded:
        notes.append("heavy_topics_in_enriched_bag")
    if coverage(rows, "has_icp") <= 0.20:
        notes.append("low_icp_coverage")
    if coverage(rows, "has_imu") <= 0.20:
        notes.append("low_imu_coverage")
    if max(coverage(rows, "has_trailer_pose"), coverage(rows, "has_trailer_angle")) <= 0.10:
        notes.append("low_trailer_perception_coverage")
    if sign_total and sign_matches / sign_total < 0.85:
        notes.append("tach_icp_speed_sign_suspicious")
    return notes


def finite_xy(rows: list[dict[str, Any]], x_key: str, y_key: str) -> tuple[list[float], list[float]]:
    xs, ys = [], []
    for row in rows:
        x = parse_float(row.get(x_key))
        y = parse_float(row.get(y_key))
        if x is not None and y is not None:
            xs.append(x)
            ys.append(y)
    return xs, ys


def write_plots(session_dir: Path, rows: list[dict[str, Any]]) -> dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        print(f"warning: matplotlib unavailable, plots skipped: {exc}", file=sys.stderr)
        return {}

    plot_dir = session_dir / "postprocess_dataset" / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    written: dict[str, str] = {}

    icp_kin = derive_xy_kinematics(rows, "icp_x", "icp_y", None)
    odom_kin = derive_xy_kinematics(rows, "odom_x", "odom_y", "odom_yaw")

    fig, ax = plt.subplots(figsize=(8, 6))
    plotted = False
    for label, x_key, y_key in (("icp", "icp_x", "icp_y"), ("odom", "odom_x", "odom_y"), ("trailer", "trailer_pose_x", "trailer_pose_y")):
        xs, ys = finite_xy(rows, x_key, y_key)
        if xs:
            ax.plot(xs, ys, label=label, linewidth=1.2)
            plotted = True
    if plotted:
        ax.set_title(session_dir.name)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.axis("equal")
        ax.grid(True, linewidth=0.4, alpha=0.5)
        ax.legend()
        fig.tight_layout()
        path = plot_dir / "trajectory_xy.png"
        fig.savefig(path, dpi=140)
        written["trajectory_xy"] = str(path)
    plt.close(fig)

    if icp_kin or odom_kin:
        fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        for label, samples in (("icp", icp_kin), ("odom", odom_kin)):
            if not samples:
                continue
            ts = [sample["t"] - samples[0]["t"] for sample in samples]
            axes[0].plot(ts, [sample["speed"] for sample in samples], label=label, linewidth=1.0)
            axes[1].plot(ts, [sample["yaw_rate"] for sample in samples], label=label, linewidth=1.0)
        tach_ts = []
        tach_values = []
        cmd_ts = []
        cmd_values = []
        t0 = parse_float(rows[0].get("t")) if rows else None
        for row in rows:
            t = parse_float(row.get("t"))
            if t is None or t0 is None:
                continue
            tach = parse_float(row.get("tach_speed_ms"))
            cmd = parse_float(row.get("cmd_linear_x"))
            if tach is not None:
                tach_ts.append(t - t0)
                tach_values.append(tach)
            if cmd is not None:
                cmd_ts.append(t - t0)
                cmd_values.append(cmd)
        if tach_ts:
            axes[0].plot(tach_ts, tach_values, label="tach", linewidth=0.8, alpha=0.8)
        if cmd_ts:
            axes[0].plot(cmd_ts, cmd_values, label="cmd", linewidth=0.8, alpha=0.8)
        axes[0].set_ylabel("speed [m/s]")
        axes[1].set_ylabel("yaw rate [rad/s]")
        axes[1].set_xlabel("bag offset [s]")
        for ax in axes:
            ax.grid(True, linewidth=0.4, alpha=0.5)
            ax.legend()
        fig.tight_layout()
        path = plot_dir / "speed_yaw_rate.png"
        fig.savefig(path, dpi=140)
        written["speed_yaw_rate"] = str(path)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    t0 = parse_float(rows[0].get("t")) if rows else None
    plotted = False
    for label, key in (("mtt", "mtt_articulation_angle"), ("trailer", "trailer_articulation_angle"), ("confidence", "trailer_confidence")):
        ts = []
        values = []
        for row in rows:
            t = parse_float(row.get("t"))
            value = parse_float(row.get(key))
            if t is not None and t0 is not None and value is not None:
                ts.append(t - t0)
                values.append(value)
        if values:
            ax.plot(ts, values, label=label, linewidth=1.0)
            plotted = True
    if plotted:
        ax.set_xlabel("bag offset [s]")
        ax.set_ylabel("rad / confidence")
        ax.grid(True, linewidth=0.4, alpha=0.5)
        ax.legend()
        fig.tight_layout()
        path = plot_dir / "articulation_trailer.png"
        fig.savefig(path, dpi=140)
        written["articulation_trailer"] = str(path)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    keys = ["has_icp", "has_odom", "has_tacho", "has_imu", "has_trailer_pose", "has_trailer_angle"]
    ax.bar(keys, [coverage(rows, key) for key in keys])
    ax.set_ylim(0.0, 1.05)
    ax.set_ylabel("coverage ratio")
    ax.tick_params(axis="x", labelrotation=30)
    ax.grid(True, axis="y", linewidth=0.4, alpha=0.5)
    fig.tight_layout()
    path = plot_dir / "sensor_coverage.png"
    fig.savefig(path, dpi=140)
    written["sensor_coverage"] = str(path)
    plt.close(fig)

    return written


def write_global_summary(report: list[dict[str, Any]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "postprocess_dataset_audit_report.yaml").write_text(
        yaml.safe_dump(report, sort_keys=False),
        encoding="utf-8",
    )
    summary_csv = output_dir / "postprocess_dataset_audit_summary.csv"
    fields = [
        "session",
        "status",
        "grade",
        "rows",
        "icp_coverage",
        "odom_coverage",
        "tacho_coverage",
        "imu_coverage",
        "trailer_pose_coverage",
        "trailer_angle_coverage",
        "icp_odom_xy_rmse_m",
        "tach_sign_agreement",
        "notes",
    ]
    with summary_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for item in report:
            coverage_data = item.get("coverage") or {}
            writer.writerow(
                {
                    "session": item.get("session"),
                    "status": item.get("status"),
                    "grade": (item.get("quality") or {}).get("grade"),
                    "rows": item.get("row_count"),
                    "icp_coverage": coverage_data.get("icp"),
                    "odom_coverage": coverage_data.get("odom"),
                    "tacho_coverage": coverage_data.get("tacho"),
                    "imu_coverage": coverage_data.get("imu"),
                    "trailer_pose_coverage": coverage_data.get("trailer_pose"),
                    "trailer_angle_coverage": coverage_data.get("trailer_angle"),
                    "icp_odom_xy_rmse_m": (item.get("icp_vs_odom") or {}).get("xy_rmse_m"),
                    "tach_sign_agreement": (item.get("tacho_cmd_vs_icp") or {}).get("tach_sign_agreement"),
                    "notes": ";".join((item.get("quality") or {}).get("notes") or []),
                }
            )


def process_session(session_dir: Path, write_plot_files: bool) -> dict[str, Any]:
    csv_path = session_dir / "postprocess_dataset" / "dataset.csv"
    if not csv_path.exists():
        result = {
            "session": session_dir.name,
            "status": "missing_dataset_csv",
            "dataset_csv": str(csv_path),
            "quality": {"grade": "weak", "notes": ["missing_dataset_csv"]},
        }
    else:
        rows, columns = read_dataset(csv_path)
        result = compute_audit(session_dir, rows, columns)
        if write_plot_files and rows:
            result["plots"] = write_plots(session_dir, rows)

    audit_path = session_dir / "postprocess_dataset" / "audit.yaml"
    audit_path.parent.mkdir(parents=True, exist_ok=True)
    audit_path.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    return result


def parse_args() -> argparse.Namespace:
    workspace_root = infer_workspace_root()
    parser = argparse.ArgumentParser(description="Audit postprocess_dataset outputs and generate metrics/plots.")
    parser.add_argument("input_path", nargs="?", default=str(workspace_root / "data"))
    parser.add_argument("--no-plots", action="store_true", help="Skip PNG plot generation.")
    parser.add_argument("--strict", action="store_true", help="Return non-zero when a session audit is weak/missing.")
    return parser.parse_args()


def main() -> int:
    workspace_root = infer_workspace_root()
    args = parse_args()
    sessions = resolve_sessions(args.input_path)
    report: list[dict[str, Any]] = []
    failures = 0

    for index, session_dir in enumerate(sessions, start=1):
        print(f"[{index}/{len(sessions)}] {session_dir.name}")
        try:
            result = process_session(session_dir, not args.no_plots)
        except Exception as exc:  # noqa: BLE001
            result = {
                "session": session_dir.name,
                "status": "failed_exception",
                "error": str(exc),
                "quality": {"grade": "weak", "notes": ["audit_exception"]},
            }
            audit_path = session_dir / "postprocess_dataset" / "audit.yaml"
            audit_path.parent.mkdir(parents=True, exist_ok=True)
            audit_path.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
        report.append(result)
        grade = (result.get("quality") or {}).get("grade", "weak")
        print(f"  {result.get('status')} grade={grade}")
        if grade == "weak" or result.get("status") not in {"ok"}:
            failures += 1

    write_global_summary(report, workspace_root / "data")
    print(f"Report: {workspace_root / 'data' / 'postprocess_dataset_audit_report.yaml'}")
    print(f"CSV:    {workspace_root / 'data' / 'postprocess_dataset_audit_summary.csv'}")
    return 1 if args.strict and failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
