#!/usr/bin/env python3
"""Investigate ICP/mapping quality across MTT postprocess outputs.

This script is intentionally offline-first: it reads existing metadata,
postprocess CSV files, audit summaries, VTK headers, and optional ROS bag TF
messages when rosbag2_py is available. It does not replay bags.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
from bisect import bisect_left
from pathlib import Path
from typing import Any

import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:  # pragma: no cover - available in ROS containers only
    rosbag2_py = None
    deserialize_message = None
    get_message = None


LIDAR_TOPICS = ["/hesai_lidar/points", "/rsairy_ns/points", "/merged_points_filtered"]
ICP_TOPIC = "/mapping/icp_odom"
ODOM_TOPIC = "/mtt_odometry"
TACH_TOPIC = "/mtt_tachometer"
IMU_TOPICS = ["/mti100/data", "/mti10/data", "/imu/data", "/zed/zed_node/imu/data"]
PERCEPTION_TOPICS = ["/trailer/angle", "/trailer/articulation_angle", "/trailer/pose"]
EXPECTED_DENSE_CONFIG = "src/external/norlab_robot/config/mapping/_config_replay_dense.yaml"


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
    if (path / "bag" / "metadata.yaml").exists() or (path / "postprocess_dataset" / "dataset.csv").exists():
        return [path]
    sessions = sorted(p.parent.parent for p in path.glob("*/bag/metadata.yaml"))
    if sessions:
        return sessions
    sessions = sorted(p.parent.parent for p in path.glob("*/postprocess_dataset/dataset.csv"))
    if sessions:
        return sessions
    raise SystemExit(f"Could not resolve any sessions from {path}")


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
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rmse(values: list[float]) -> float | None:
    if not values:
        return None
    return math.sqrt(sum(v * v for v in values) / len(values))


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(round((pct / 100.0) * (len(ordered) - 1)))))
    return ordered[idx]


def finite_stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "min": None, "mean": None, "median": None, "p95": None, "max": None}
    return {
        "count": len(values),
        "min": min(values),
        "mean": sum(values) / len(values),
        "median": statistics.median(values),
        "p95": percentile(values, 95.0),
        "max": max(values),
    }


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_dataset(path: Path) -> tuple[list[dict[str, Any]], list[str]]:
    if not path.exists():
        return [], []
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        return [dict(row) for row in reader], list(reader.fieldnames or [])


def load_metadata_counts(session_dir: Path) -> tuple[dict[str, int], float]:
    metadata = load_yaml(session_dir / "bag" / "metadata.yaml")
    info = metadata.get("rosbag2_bagfile_information", metadata)
    counts = {}
    for item in info.get("topics_with_message_count", []):
        meta = item.get("topic_metadata", {})
        counts[str(meta.get("name", ""))] = int(item.get("message_count", 0))
    duration_s = float((info.get("duration") or {}).get("nanoseconds", 0)) / 1e9
    return counts, duration_s


def vtk_points(path: Path) -> int | None:
    if not path.exists():
        return None
    try:
        with path.open("r", encoding="utf-8", errors="ignore") as stream:
            for _ in range(20):
                line = stream.readline()
                if not line:
                    break
                if line.startswith("POINTS "):
                    parts = line.split()
                    return int(parts[1])
    except OSError:
        return None
    return None


def coverage(rows: list[dict[str, Any]], key: str) -> float:
    if not rows:
        return 0.0
    return sum(1 for row in rows if parse_bool(row.get(key))) / len(rows)


def pose_series(rows: list[dict[str, Any]], prefix: str, has_key: str | None, yaw_key: str | None = None) -> list[dict[str, float]]:
    out: list[dict[str, float]] = []
    prev: dict[str, float] | None = None
    for row in rows:
        if has_key and not parse_bool(row.get(has_key)):
            continue
        t = parse_float(row.get("t"))
        x = parse_float(row.get(f"{prefix}_x"))
        y = parse_float(row.get(f"{prefix}_y"))
        if yaw_key:
            yaw = parse_float(row.get(yaw_key))
        else:
            qx = parse_float(row.get(f"{prefix}_qx"))
            qy = parse_float(row.get(f"{prefix}_qy"))
            qz = parse_float(row.get(f"{prefix}_qz"))
            qw = parse_float(row.get(f"{prefix}_qw"))
            yaw = None if None in (qx, qy, qz, qw) else yaw_from_quat(qx, qy, qz, qw)  # type: ignore[arg-type]
        if None in (t, x, y, yaw):
            continue
        assert t is not None and x is not None and y is not None and yaw is not None
        sample = {"t": t, "x": x, "y": y, "yaw": yaw, "speed": 0.0, "yaw_rate": 0.0}
        if prev is not None:
            dt = t - prev["t"]
            if dt > 1e-6:
                dx = x - prev["x"]
                dy = y - prev["y"]
                heading_mid = wrap_angle(0.5 * (yaw + prev["yaw"]))
                sample["speed"] = (dx * math.cos(heading_mid) + dy * math.sin(heading_mid)) / dt
                sample["yaw_rate"] = wrap_angle(yaw - prev["yaw"]) / dt
        out.append(sample)
        prev = sample
    return out


class Series:
    def __init__(self, rows: list[dict[str, float]]):
        self.rows = sorted(rows, key=lambda row: row["t"])
        self.times = [row["t"] for row in self.rows]

    def nearest(self, t: float, tolerance_s: float) -> dict[str, float] | None:
        if not self.rows:
            return None
        idx = bisect_left(self.times, t)
        candidates = []
        if idx < len(self.rows):
            candidates.append(self.rows[idx])
        if idx:
            candidates.append(self.rows[idx - 1])
        best = min(candidates, key=lambda row: abs(row["t"] - t))
        return best if abs(best["t"] - t) <= tolerance_s else None


def trajectory_stats(samples: list[dict[str, float]]) -> dict[str, Any]:
    if len(samples) < 2:
        return {"count": len(samples), "path_length_m": None, "net_displacement_m": None}
    steps = [
        math.hypot(samples[i]["x"] - samples[i - 1]["x"], samples[i]["y"] - samples[i - 1]["y"])
        for i in range(1, len(samples))
    ]
    dts = [samples[i]["t"] - samples[i - 1]["t"] for i in range(1, len(samples))]
    jumps = [
        {
            "t_offset_s": samples[i]["t"] - samples[0]["t"],
            "dt_s": dts[i - 1],
            "distance_m": steps[i - 1],
            "speed_ms": steps[i - 1] / dts[i - 1] if dts[i - 1] > 1e-6 else None,
        }
        for i in range(1, len(samples))
        if steps[i - 1] > 1.0 or (dts[i - 1] > 1e-6 and steps[i - 1] / dts[i - 1] > 5.0)
    ]
    return {
        "count": len(samples),
        "start_t": samples[0]["t"],
        "end_t": samples[-1]["t"],
        "duration_s": samples[-1]["t"] - samples[0]["t"],
        "path_length_m": sum(steps),
        "net_displacement_m": math.hypot(samples[-1]["x"] - samples[0]["x"], samples[-1]["y"] - samples[0]["y"]),
        "max_step_m": max(steps),
        "dt_s": finite_stats(dts),
        "gaps_over_1s": sum(1 for dt in dts if dt > 1.0),
        "gaps_over_5s": sum(1 for dt in dts if dt > 5.0),
        "suspicious_jump_count": len(jumps),
        "first_suspicious_jumps": jumps[:20],
    }


def compare_series(rows: list[dict[str, Any]], icp: list[dict[str, float]], odom: list[dict[str, float]]) -> dict[str, Any]:
    icp_series = Series(icp)
    odom_series = Series(odom)
    xy_errors: list[float] = []
    yaw_errors: list[float] = []
    speed_errors: list[float] = []
    yaw_rate_errors: list[float] = []
    imu_yaw_errors: list[float] = []
    tach_speed_errors: list[float] = []
    cmd_speed_errors: list[float] = []
    sign_total = 0
    sign_matches = 0
    reverse_like = 0

    for row in rows:
        t = parse_float(row.get("t"))
        if t is None:
            continue
        i = icp_series.nearest(t, 0.05)
        o = odom_series.nearest(t, 0.05)
        if i and o:
            xy_errors.append(math.hypot(o["x"] - i["x"], o["y"] - i["y"]))
            yaw_errors.append(wrap_angle(o["yaw"] - i["yaw"]))
            speed_errors.append(o["speed"] - i["speed"])
            yaw_rate_errors.append(o["yaw_rate"] - i["yaw_rate"])
        if i:
            imu_wz = parse_float(row.get("imu_angular_velocity_z"))
            tach = parse_float(row.get("tach_speed_ms"))
            cmd = parse_float(row.get("cmd_linear_x"))
            if imu_wz is not None:
                imu_yaw_errors.append(imu_wz - i["yaw_rate"])
            if tach is not None:
                tach_speed_errors.append(tach - i["speed"])
                if abs(tach) > 0.05 and abs(i["speed"]) > 0.05:
                    sign_total += 1
                    same = math.copysign(1.0, tach) == math.copysign(1.0, i["speed"])
                    sign_matches += int(same)
                    reverse_like += int(not same)
            if cmd is not None:
                cmd_speed_errors.append(cmd - i["speed"])

    return {
        "paired_samples": len(xy_errors),
        "xy_rmse_m": rmse(xy_errors),
        "xy_median_m": statistics.median(xy_errors) if xy_errors else None,
        "xy_p95_m": percentile(xy_errors, 95.0),
        "yaw_rmse_rad": rmse(yaw_errors),
        "speed_rmse_ms": rmse(speed_errors),
        "yaw_rate_rmse_rad_s": rmse(yaw_rate_errors),
        "imu_yaw_rate_vs_icp_rmse_rad_s": rmse(imu_yaw_errors),
        "tach_speed_vs_icp_rmse_ms": rmse(tach_speed_errors),
        "cmd_speed_vs_icp_rmse_ms": rmse(cmd_speed_errors),
        "tach_sign_checks": sign_total,
        "tach_sign_agreement": sign_matches / sign_total if sign_total else None,
        "tach_opposite_sign_count": reverse_like,
    }


def integrate_imu_yaw(rows: list[dict[str, Any]], reference: list[dict[str, float]]) -> dict[str, Any]:
    if len(reference) < 2:
        return {"available": False}
    ref_series = Series(reference)
    samples = []
    for row in rows:
        t = parse_float(row.get("t"))
        wz = parse_float(row.get("imu_angular_velocity_z"))
        if t is not None and wz is not None:
            samples.append({"t": t, "wz": wz})
    if len(samples) < 2:
        return {"available": False}
    yaw = reference[0]["yaw"]
    prev_t = samples[0]["t"]
    errors = []
    for sample in samples[1:]:
        dt = sample["t"] - prev_t
        prev_t = sample["t"]
        if dt <= 0.0 or dt > 0.5:
            continue
        yaw = wrap_angle(yaw + sample["wz"] * dt)
        ref = ref_series.nearest(sample["t"], 0.05)
        if ref:
            errors.append(wrap_angle(ref["yaw"] - yaw))
    return {
        "available": bool(errors),
        "paired_samples": len(errors),
        "yaw_rmse_rad": rmse(errors),
        "yaw_median_abs_error_rad": statistics.median([abs(v) for v in errors]) if errors else None,
    }


def inspect_tf_bag(session_dir: Path, max_messages: int = 200000) -> dict[str, Any]:
    if rosbag2_py is None or deserialize_message is None or get_message is None:
        return {"available": False, "reason": "rosbag2_py_not_available"}
    bag_dir = session_dir / "bag"
    metadata = bag_dir / "metadata.yaml"
    if not metadata.exists():
        return {"available": False, "reason": "missing_metadata"}
    try:
        reader = rosbag2_py.SequentialReader()
        storage = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap")
        converter = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        reader.open(storage, converter)
        type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        tf_types = {topic: get_message(type_map[topic]) for topic in ("/tf", "/tf_static") if topic in type_map}
        pairs: dict[str, dict[str, Any]] = {}
        count = 0
        while reader.has_next() and count < max_messages:
            topic, data, _ = reader.read_next()
            if topic not in tf_types:
                continue
            msg = deserialize_message(data, tf_types[topic])
            count += 1
            for tf in msg.transforms:
                parent = tf.header.frame_id
                child = tf.child_frame_id
                key = f"{parent}->{child}"
                stamp = float(tf.header.stamp.sec) + 1e-9 * float(tf.header.stamp.nanosec)
                item = pairs.setdefault(key, {"count": 0, "first_t": stamp, "last_t": stamp})
                item["count"] += 1
                item["first_t"] = min(item["first_t"], stamp)
                item["last_t"] = max(item["last_t"], stamp)
        interesting = {
            key: value
            for key, value in pairs.items()
            if any(token in key for token in ("odom", "base", "hesai", "rsairy", "mti", "lidar"))
        }
        return {
            "available": True,
            "tf_messages_read": count,
            "pair_count": len(pairs),
            "interesting_pairs": dict(sorted(interesting.items())),
        }
    except Exception as exc:  # noqa: BLE001
        return {"available": False, "reason": str(exc)}


def pipeline_risks(session_dir: Path, icp_summary: dict[str, Any]) -> list[str]:
    risks = []
    if icp_summary.get("offline_quality") == "max":
        mapping_config = str(icp_summary.get("mapping_config") or "")
        notes = " ".join(str(v) for v in icp_summary.get("quality_profile_notes", []))
        if EXPECTED_DENSE_CONFIG not in mapping_config and "_config_replay_dense.yaml" not in notes:
            risks.append("offline_quality_max_did_not_record_dense_mapping_config")
    if icp_summary.get("mode") == "fused" and icp_summary.get("filter_trailer") is not True:
        risks.append("fused_mapping_without_confirmed_trailer_bbox_filter")
    if (session_dir / "offline_icp" / "summary.yaml").exists() and not (session_dir / "offline_icp" / "logs").exists():
        risks.append("offline_icp_has_no_logs_for_replay_debug")
    return risks


def root_cause_hypotheses(
    counts: dict[str, int],
    icp_summary: dict[str, Any],
    rows: list[dict[str, Any]],
    icp_stats: dict[str, Any],
    odom_stats: dict[str, Any],
    compare: dict[str, Any],
    risks: list[str],
) -> list[str]:
    hypotheses = list(risks)
    icp_cov = coverage(rows, "has_icp")
    real_tach_cov = coverage(rows, "has_real_tacho")
    cmd_sim_cov = coverage(rows, "has_cmd_sim_tacho")
    sign = compare.get("tach_sign_agreement")
    xy_rmse = compare.get("xy_rmse_m")
    jumps = int(icp_stats.get("suspicious_jump_count") or 0)
    gaps = int(icp_stats.get("gaps_over_1s") or 0)

    if icp_summary.get("mode") == "fused" and counts.get("/hesai_lidar/points", 0) and counts.get("/rsairy_ns/points", 0):
        hypotheses.append("fused_lidar_path_active_check_hesai_only_variant")
    if cmd_sim_cov > 0.5 and real_tach_cov < 0.1:
        hypotheses.append("odom_prior_is_cmd_sim_not_real_tachometer")
    if sign is not None and sign < 0.5:
        hypotheses.append("tach_or_cmd_speed_sign_disagrees_with_icp")
    if icp_cov < 0.6:
        hypotheses.append("icp_temporal_coverage_low_mapper_dropped_or_failed_registrations")
    if jumps > 20 or gaps > 20:
        hypotheses.append("icp_has_many_gaps_or_pose_jumps")
    if isinstance(xy_rmse, float) and xy_rmse > 10.0:
        hypotheses.append("icp_and_odom_global_paths_strongly_disagree")
    if counts.get("/merged_points_filtered", 0) and counts.get("/hesai_lidar/points", 0) and counts.get("/rsairy_ns/points", 0):
        hypotheses.append("compare_live_cloud_merger_vs_recorded_merged_cloud")
    if odom_stats.get("path_length_m") and icp_stats.get("path_length_m"):
        odom_len = float(odom_stats["path_length_m"])
        icp_len = float(icp_stats["path_length_m"])
        if odom_len > 1.0 and (icp_len / odom_len < 0.5 or icp_len / odom_len > 1.8):
            hypotheses.append("icp_path_length_inconsistent_with_odom_path_length")
    return sorted(set(hypotheses))


def write_markdown(path: Path, report: dict[str, Any]) -> None:
    lines = [
        f"# ICP investigation - {report['session']}",
        "",
        f"- grade: `{report['quality']['grade']}`",
        f"- ICP coverage: `{report['coverage']['icp']:.3f}`",
        f"- ICP/odom XY RMSE: `{report['comparison'].get('xy_rmse_m')}` m",
        f"- tach sign agreement: `{report['comparison'].get('tach_sign_agreement')}`",
        f"- ICP jumps: `{report['icp_trajectory'].get('suspicious_jump_count')}`",
        f"- ICP gaps > 1 s: `{report['icp_trajectory'].get('gaps_over_1s')}`",
        "",
        "## Suspects",
    ]
    for item in report["suspects"]:
        lines.append(f"- `{item}`")
    lines.extend(["", "## Recommended next experiments"])
    for item in report["recommended_experiments"]:
        lines.append(f"- {item}")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def write_plots(session_dir: Path, rows: list[dict[str, Any]], icp: list[dict[str, float]], odom: list[dict[str, float]]) -> dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}

    out_dir = session_dir / "icp_investigation" / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    written: dict[str, str] = {}

    fig, ax = plt.subplots(figsize=(8, 6))
    if icp:
        ax.plot([p["x"] for p in icp], [p["y"] for p in icp], label="icp", linewidth=1.2)
    if odom:
        ax.plot([p["x"] for p in odom], [p["y"] for p in odom], label="odom", linewidth=1.0)
    tx = [parse_float(row.get("trailer_pose_x")) for row in rows]
    ty = [parse_float(row.get("trailer_pose_y")) for row in rows]
    trailer_xy = [(x, y) for x, y in zip(tx, ty) if x is not None and y is not None]
    if trailer_xy:
        ax.plot([p[0] for p in trailer_xy], [p[1] for p in trailer_xy], label="trailer_pose", linewidth=0.8, alpha=0.8)
    ax.set_title(session_dir.name)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, linewidth=0.4, alpha=0.5)
    ax.legend()
    fig.tight_layout()
    path = out_dir / "trajectory_xy_aligned.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written["trajectory_xy_aligned"] = str(path)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    t0 = parse_float(rows[0].get("t")) if rows else None
    for label, samples in (("icp", icp), ("odom", odom)):
        if samples:
            axes[0].plot([s["t"] - samples[0]["t"] for s in samples], [s["speed"] for s in samples], label=label, linewidth=1.0)
            axes[1].plot([s["t"] - samples[0]["t"] for s in samples], [s["yaw_rate"] for s in samples], label=label, linewidth=1.0)
    if t0 is not None:
        for key, label, axis in [
            ("tach_speed_ms", "tach", axes[0]),
            ("cmd_linear_x", "cmd", axes[0]),
            ("imu_angular_velocity_z", "imu_wz", axes[1]),
        ]:
            pts = []
            for row in rows:
                t = parse_float(row.get("t"))
                value = parse_float(row.get(key))
                if t is not None and value is not None:
                    pts.append((t - t0, value))
            if pts:
                axis.plot([p[0] for p in pts], [p[1] for p in pts], label=label, linewidth=0.7, alpha=0.8)
    axes[0].set_ylabel("speed [m/s]")
    axes[1].set_ylabel("yaw rate [rad/s]")
    axes[1].set_xlabel("bag offset [s]")
    for ax in axes:
        ax.grid(True, linewidth=0.4, alpha=0.5)
        ax.legend()
    fig.tight_layout()
    path = out_dir / "icp_odom_speed_yaw_imu.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written["icp_odom_speed_yaw_imu"] = str(path)

    if len(icp) > 1:
        dts = [icp[i]["t"] - icp[i - 1]["t"] for i in range(1, len(icp))]
        steps = [math.hypot(icp[i]["x"] - icp[i - 1]["x"], icp[i]["y"] - icp[i - 1]["y"]) for i in range(1, len(icp))]
        offsets = [icp[i]["t"] - icp[0]["t"] for i in range(1, len(icp))]
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        axes[0].scatter(offsets, dts, s=4)
        axes[1].scatter(offsets, steps, s=4)
        axes[0].set_ylabel("ICP dt [s]")
        axes[1].set_ylabel("ICP step [m]")
        axes[1].set_xlabel("bag offset [s]")
        for ax in axes:
            ax.grid(True, linewidth=0.4, alpha=0.5)
        fig.tight_layout()
        path = out_dir / "icp_gaps_jumps.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        written["icp_gaps_jumps"] = str(path)

    return written


def recommended_experiments(suspects: list[str]) -> list[str]:
    experiments = []
    if any("fused" in item or "merged" in item for item in suspects):
        experiments.append("Run Hesai-only dense ICP to isolate LiDAR fusion/calibration.")
    if any("trailer" in item for item in suspects):
        experiments.append("Re-run fused dense ICP with trailer bbox filter forced on.")
    if any("odom_prior" in item or "cmd_sim" in item or "sign" in item for item in suspects):
        experiments.append("Run a diagnostic mapper variant with odom prior neutralized or rebuilt from a trusted source.")
    if any("coverage" in item or "jumps" in item for item in suspects):
        experiments.append("Enable mapper debug logs and compare maxDist/outlier ratio/updateCondition variants.")
    if not experiments:
        experiments.append("Use this bag as a baseline; compare VTK and CSV against the worst bags.")
    return experiments


def grade_report(report: dict[str, Any]) -> str:
    icp_cov = report["coverage"]["icp"]
    xy = report["comparison"].get("xy_rmse_m")
    jumps = report["icp_trajectory"].get("suspicious_jump_count") or 0
    sign = report["comparison"].get("tach_sign_agreement")
    if icp_cov > 0.85 and (xy is None or xy < 2.0) and jumps < 10 and (sign is None or sign > 0.8):
        return "good"
    if icp_cov > 0.5 and (xy is None or xy < 10.0) and jumps < 50:
        return "mixed"
    return "bad"


def process_session(session_dir: Path, inspect_tf: bool, write_plot_files: bool) -> dict[str, Any]:
    counts, duration_s = load_metadata_counts(session_dir)
    rows, columns = read_dataset(session_dir / "postprocess_dataset" / "dataset.csv")
    icp_summary = load_yaml(session_dir / "offline_icp" / "summary.yaml")
    post_summary = load_yaml(session_dir / "postprocess_dataset" / "summary.yaml")
    audit = load_yaml(session_dir / "postprocess_dataset" / "audit.yaml")
    icp = pose_series(rows, "icp", "has_icp")
    odom = pose_series(rows, "odom", "has_odom", "odom_yaw")
    risks = pipeline_risks(session_dir, icp_summary)
    icp_traj = trajectory_stats(icp)
    odom_traj = trajectory_stats(odom)
    comparison = compare_series(rows, icp, odom)
    suspects = root_cause_hypotheses(counts, icp_summary, rows, icp_traj, odom_traj, comparison, risks)
    report: dict[str, Any] = {
        "session": session_dir.name,
        "session_dir": str(session_dir),
        "duration_s": duration_s,
        "metadata_counts": {topic: counts.get(topic, 0) for topic in LIDAR_TOPICS + [ICP_TOPIC, ODOM_TOPIC, TACH_TOPIC] + IMU_TOPICS + PERCEPTION_TOPICS},
        "csv": {
            "path": str(session_dir / "postprocess_dataset" / "dataset.csv"),
            "rows": len(rows),
            "columns": columns,
            "missing": not bool(rows),
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
        "offline_icp": {
            "status": icp_summary.get("status"),
            "mode": icp_summary.get("mode"),
            "offline_quality": icp_summary.get("offline_quality"),
            "mapping_config": icp_summary.get("mapping_config"),
            "filter_trailer": icp_summary.get("filter_trailer"),
            "quality_profile_notes": icp_summary.get("quality_profile_notes"),
            "map_points": vtk_points(session_dir / "offline_icp" / "map.vtk"),
            "trajectory_points": vtk_points(session_dir / "offline_icp" / "trajectory.vtk"),
            "map_size_bytes": icp_summary.get("map_size_bytes"),
            "trajectory_size_bytes": icp_summary.get("trajectory_size_bytes"),
        },
        "postprocess_status": {
            "summary_status": post_summary.get("status"),
            "summary_grade": (post_summary.get("quality") or {}).get("grade"),
            "audit_grade": ((audit.get("quality") or {}).get("grade") if audit else None),
            "audit_notes": ((audit.get("quality") or {}).get("notes") if audit else None),
        },
        "icp_trajectory": icp_traj,
        "odom_trajectory": odom_traj,
        "comparison": comparison,
        "imu_integration_vs_icp": integrate_imu_yaw(rows, icp),
        "pipeline_risks": risks,
        "suspects": suspects,
        "recommended_experiments": recommended_experiments(suspects),
    }
    report["quality"] = {"grade": grade_report(report)}
    if inspect_tf:
        report["tf"] = inspect_tf_bag(session_dir)
    if write_plot_files and rows:
        report["plots"] = write_plots(session_dir, rows, icp, odom)

    out_dir = session_dir / "icp_investigation"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "report.yaml").write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    write_markdown(out_dir / "summary.md", report)
    return report


def write_global_report(reports: list[dict[str, Any]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "icp_investigation_report.yaml").write_text(yaml.safe_dump(reports, sort_keys=False), encoding="utf-8")
    fields = [
        "session",
        "grade",
        "icp_coverage",
        "icp_xy_rmse_m",
        "icp_path_m",
        "odom_path_m",
        "icp_jumps",
        "icp_gaps_over_1s",
        "tach_sign_agreement",
        "cmd_sim_tacho_coverage",
        "mode",
        "mapping_config",
        "suspects",
    ]
    with (output_dir / "icp_investigation_summary.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for item in reports:
            writer.writerow(
                {
                    "session": item.get("session"),
                    "grade": (item.get("quality") or {}).get("grade"),
                    "icp_coverage": (item.get("coverage") or {}).get("icp"),
                    "icp_xy_rmse_m": (item.get("comparison") or {}).get("xy_rmse_m"),
                    "icp_path_m": (item.get("icp_trajectory") or {}).get("path_length_m"),
                    "odom_path_m": (item.get("odom_trajectory") or {}).get("path_length_m"),
                    "icp_jumps": (item.get("icp_trajectory") or {}).get("suspicious_jump_count"),
                    "icp_gaps_over_1s": (item.get("icp_trajectory") or {}).get("gaps_over_1s"),
                    "tach_sign_agreement": (item.get("comparison") or {}).get("tach_sign_agreement"),
                    "cmd_sim_tacho_coverage": (item.get("coverage") or {}).get("cmd_sim_tacho"),
                    "mode": (item.get("offline_icp") or {}).get("mode"),
                    "mapping_config": (item.get("offline_icp") or {}).get("mapping_config"),
                    "suspects": ";".join(item.get("suspects") or []),
                }
            )


def parse_args() -> argparse.Namespace:
    workspace = infer_workspace_root()
    parser = argparse.ArgumentParser(description="Investigate MTT ICP/mapping outputs across bags.")
    parser.add_argument("input_path", nargs="?", default=str(workspace / "data"))
    parser.add_argument("--inspect-tf", action="store_true", help="Decode /tf and /tf_static from bags when ROS bag Python APIs are available.")
    parser.add_argument("--no-plots", action="store_true", help="Skip investigation PNG plots.")
    parser.add_argument("--strict", action="store_true", help="Return non-zero if any processed session is bad.")
    return parser.parse_args()


def main() -> int:
    workspace = infer_workspace_root()
    args = parse_args()
    sessions = resolve_sessions(args.input_path)
    reports = []
    bad = 0
    for index, session_dir in enumerate(sessions, start=1):
        print(f"[{index}/{len(sessions)}] {session_dir.name}")
        try:
            report = process_session(session_dir, args.inspect_tf, not args.no_plots)
        except Exception as exc:  # noqa: BLE001
            report = {
                "session": session_dir.name,
                "session_dir": str(session_dir),
                "quality": {"grade": "bad"},
                "error": str(exc),
                "suspects": ["investigation_exception"],
            }
            out_dir = session_dir / "icp_investigation"
            out_dir.mkdir(parents=True, exist_ok=True)
            (out_dir / "report.yaml").write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
        reports.append(report)
        grade = (report.get("quality") or {}).get("grade", "bad")
        bad += int(grade == "bad")
        print(f"  grade={grade} suspects={len(report.get('suspects') or [])}")
    write_global_report(reports, workspace / "data")
    print(f"Report: {workspace / 'data' / 'icp_investigation_report.yaml'}")
    print(f"CSV:    {workspace / 'data' / 'icp_investigation_summary.csv'}")
    return 1 if args.strict and bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
