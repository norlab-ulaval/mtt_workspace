#!/usr/bin/env python3
"""Build synchronized post-process CSV datasets for MTT bag sessions."""

from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from bisect import bisect_left
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # pragma: no cover - ROS runtime dependency
    rosbag2_py = None
    deserialize_message = None
    get_message = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


ORIGINAL_TOPICS = {
    "/mapping/icp_odom",
    "/mtt_odometry",
    "/mtt_tachometer",
    "/mtt_articulation_angle",
    "/mti100/data",
    "/mti10/data",
    "/imu/data",
    "/cmd_vel",
    "/cmd_vel/teleop",
    "/controller/cmd_vel",
}

TELEMETRY_SUMMARY_TOPICS = {
    "/mtt_status",
    "/mtt_health",
    "/mtt_battery/status",
}

PERCEPTION_TOPICS = {
    "/trailer/angle",
    "/trailer/articulation_angle",
    "/trailer/pose",
    "/trailer/pose_confidence",
}

ALL_TOPICS = ORIGINAL_TOPICS | PERCEPTION_TOPICS | TELEMETRY_SUMMARY_TOPICS

HEAVY_REPLAY_TOPICS = {
    "/merged_points",
    "/merged_points_raw",
    "/merged_points_filtered",
    "/hesai_lidar/points",
    "/rsairy_ns/points",
    "/trailer/trailer_roi_cloud",
    "/trailer/articulation_roi_cloud",
    "/mapping/map",
}

METADATA_GROUPS = {
    "lidar": ["/hesai_lidar/points", "/rsairy_ns/points", "/merged_points_filtered"],
    "icp": ["/mapping/icp_odom", "/mapping/map"],
    "robot_motion": ["/mtt_odometry", "/mtt_tachometer", "/mtt_articulation_angle"],
    "commands": ["/cmd_vel", "/cmd_vel/teleop", "/controller/cmd_vel"],
    "imu": ["/mti100/data", "/mti10/data", "/imu/data", "/zed/zed_node/imu/data"],
    "perception": ["/trailer/angle", "/trailer/articulation_angle", "/trailer/pose", "/trailer/pose_confidence"],
    "telemetry_summary": ["/mtt_status", "/mtt_health", "/mtt_battery/status"],
    "visual": [
        "/zed/zed_node/rgb/color/rect/image/compressed",
        "/zed/zed_node/depth/depth_registered/compressedDepth",
        "/oak/rgb/image_rect",
        "/oak/stereo/image_raw",
        "/oak/points",
    ],
}

PERCEPTION_RECORD_TOPICS = sorted(
    PERCEPTION_TOPICS
    | {
        "/trailer/body_markers",
        "/trailer/articulation_axis_marker",
        "/trailer/pose_prior",
        "/trailer/pose_raw",
        "/trailer/yaw_prior",
        "/trailer/yaw_raw",
        "/trailer/yaw_used",
        "/trailer/yaw_correction_raw",
        "/trailer/yaw_correction_used",
        "/trailer/pitch_raw",
        "/trailer/pitch_used",
        "/trailer/roll_used",
        "/trailer/roi_point_count",
        "/trailer/roi_point_count_after_ground",
        "/trailer/span_s",
        "/trailer/centerline_pca_ratio",
        "/trailer/measurement_valid",
        "/trailer/pitch_valid",
        "/trailer/roll_valid",
        "/trailer/command_residual",
    }
)

CSV_FIELDS = [
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
    "imu_orientation_x",
    "imu_orientation_y",
    "imu_orientation_z",
    "imu_orientation_w",
    "imu_angular_velocity_x",
    "imu_angular_velocity_y",
    "imu_angular_velocity_z",
    "imu_linear_acceleration_x",
    "imu_linear_acceleration_y",
    "imu_linear_acceleration_z",
    "cmd_linear_x",
    "cmd_angular_z",
    "tach_speed_ms",
    "tach_source",
    "tach_is_synthetic",
    "tach_direction",
    "mtt_articulation_angle",
    "trailer_articulation_angle",
    "trailer_pose_x",
    "trailer_pose_y",
    "trailer_pose_z",
    "trailer_pose_qx",
    "trailer_pose_qy",
    "trailer_pose_qz",
    "trailer_pose_qw",
    "trailer_confidence",
    "has_icp",
    "has_odom",
    "has_tacho",
    "has_real_tacho",
    "has_cmd_sim_tacho",
    "has_imu",
    "has_trailer_pose",
    "has_trailer_angle",
]


@dataclass
class ProcessHandle:
    process: subprocess.Popen
    log_file: Any


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
    if path.is_file() and path.suffix == ".mcap":
        return [path.parent.parent if path.parent.name == "bag" else path.parent]
    if (path / "bag" / "metadata.yaml").exists():
        return [path]
    if (path / "metadata.yaml").exists():
        return [path.parent]
    sessions = sorted(p.parent.parent for p in path.glob("*/bag/metadata.yaml"))
    if sessions:
        return sessions
    raise SystemExit(f"Could not resolve sessions from {path}")


def load_metadata(session_dir: Path) -> tuple[dict[str, int], float, int]:
    metadata_path = session_dir / "bag" / "metadata.yaml"
    return load_bag_metadata(metadata_path)


def load_bag_metadata(metadata_path: Path) -> tuple[dict[str, int], float, int]:
    if not metadata_path.exists():
        return {}, 0.0, 0
    data = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    info = data.get("rosbag2_bagfile_information", data)
    counts = {
        item["topic_metadata"]["name"]: int(item["message_count"])
        for item in info.get("topics_with_message_count", [])
    }
    duration_s = float(info.get("duration", {}).get("nanoseconds", 0)) / 1e9
    total = int(info.get("message_count", 0))
    return counts, duration_s, total


def directory_size_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    total = 0
    for item in path.rglob("*"):
        if item.is_file():
            try:
                total += item.stat().st_size
            except OSError:
                pass
    return total


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def msg_time(msg: Any, bag_time_s: float) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and (stamp.sec or stamp.nanosec):
        return stamp_to_sec(stamp)
    return bag_time_s


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def extract_sample(topic: str, msg: Any, bag_time_s: float) -> dict[str, Any]:
    if topic in {"/mapping/icp_odom", "/mtt_odometry"}:
        pose = msg.pose.pose
        return {
            "t": msg_time(msg, bag_time_s),
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "qx": float(pose.orientation.x),
            "qy": float(pose.orientation.y),
            "qz": float(pose.orientation.z),
            "qw": float(pose.orientation.w),
            "yaw": yaw_from_quaternion(pose.orientation),
        }

    if topic in {"/cmd_vel", "/cmd_vel/teleop", "/controller/cmd_vel"}:
        if hasattr(msg, "twist") and hasattr(msg.twist, "twist"):
            twist = msg.twist.twist
        elif hasattr(msg, "twist"):
            twist = msg.twist
        else:
            twist = msg
        return {
            "t": msg_time(msg, bag_time_s),
            "linear_x": float(twist.linear.x),
            "angular_z": float(twist.angular.z),
        }

    if topic == "/mtt_tachometer":
        return {
            "t": msg_time(msg, bag_time_s),
            "speed_ms": float(getattr(msg, "speed_ms", 0.0)),
            "source": str(getattr(msg, "tachometer_source", "")),
            "synthetic": bool(getattr(msg, "tachometer_is_synthetic", False)),
            "direction": str(getattr(msg, "direction", "")),
            "main_sensor_temp_a": float(getattr(msg, "main_sensor_temp_a", math.nan)),
            "main_sensor_temp_b": float(getattr(msg, "main_sensor_temp_b", math.nan)),
        }

    if topic == "/mtt_status":
        return {
            "t": msg_time(msg, bag_time_s),
            "speed_ms": float(getattr(msg, "speed_ms", 0.0)),
            "tachometer_source": str(getattr(msg, "tachometer_source", "")),
            "tachometer_is_synthetic": bool(getattr(msg, "tachometer_is_synthetic", False)),
            "temperature_a": float(getattr(msg, "temperature_a", math.nan)),
            "temperature_b": float(getattr(msg, "temperature_b", math.nan)),
            "telemetry_fresh": bool(getattr(msg, "telemetry_fresh", False)),
            "telemetry_age_ms": float(getattr(msg, "telemetry_age_ms", math.nan)),
            "emergency_stop_active": bool(getattr(msg, "emergency_stop_active", False)),
            "command_timeout_active": bool(getattr(msg, "command_timeout_active", False)),
            "safety_state": str(getattr(msg, "safety_state", "")),
        }

    if topic == "/mtt_health":
        warnings = getattr(msg, "warnings", [])
        return {
            "t": msg_time(msg, bag_time_s),
            "tachometer_source": str(getattr(msg, "tachometer_source", "")),
            "tachometer_is_synthetic": bool(getattr(msg, "tachometer_is_synthetic", False)),
            "telemetry_fresh": bool(getattr(msg, "telemetry_fresh", False)),
            "telemetry_age_ms": float(getattr(msg, "telemetry_age_ms", math.nan)),
            "main_sensor_temp_a_c": float(getattr(msg, "main_sensor_temp_a_c", math.nan)),
            "main_sensor_temp_b_c": float(getattr(msg, "main_sensor_temp_b_c", math.nan)),
            "battery_current_estimated_a": float(getattr(msg, "battery_current_estimated_a", math.nan)),
            "battery_current_estimated_valid": bool(getattr(msg, "battery_current_estimated_valid", False)),
            "battery_voltage_v": float(getattr(msg, "battery_voltage_v", math.nan)),
            "battery_voltage_valid": bool(getattr(msg, "battery_voltage_valid", False)),
            "power_watts": float(getattr(msg, "power_watts", math.nan)),
            "power_valid": bool(getattr(msg, "power_valid", False)),
            "soc_percent": int(getattr(msg, "soc_percent", 0)),
            "fallback_low_confidence": bool(getattr(msg, "fallback_low_confidence", False)),
            "health_summary": str(getattr(msg, "health_summary", "")),
            "warnings": list(warnings),
        }

    if topic == "/mtt_battery/status":
        return {
            "t": msg_time(msg, bag_time_s),
            "soc_percent": int(getattr(msg, "soc_percent", 0)),
            "battery_current_estimated_a": float(getattr(msg, "battery_current_estimated_a", math.nan)),
            "battery_current_estimated_valid": bool(getattr(msg, "battery_current_estimated_valid", False)),
            "battery_voltage_v": float(getattr(msg, "battery_voltage_v", math.nan)),
            "battery_voltage_valid": bool(getattr(msg, "battery_voltage_valid", False)),
            "power_watts": float(getattr(msg, "power_watts", math.nan)),
            "power_valid": bool(getattr(msg, "power_valid", False)),
            "cell_temp_1_c": float(getattr(msg, "cell_temp_1_c", math.nan)),
            "cell_temp_2_c": float(getattr(msg, "cell_temp_2_c", math.nan)),
            "cell_temp_3_c": float(getattr(msg, "cell_temp_3_c", math.nan)),
            "cell_temp_4_c": float(getattr(msg, "cell_temp_4_c", math.nan)),
            "ambient_temp_c": float(getattr(msg, "ambient_temp_c", math.nan)),
            "mosfet_temp_c": float(getattr(msg, "mosfet_temp_c", math.nan)),
        }

    if topic in {"/mtt_articulation_angle", "/trailer/angle", "/trailer/articulation_angle", "/trailer/pose_confidence"}:
        return {"t": bag_time_s, "value": float(msg.data)}

    if topic == "/trailer/pose":
        pose = msg.pose
        return {
            "t": msg_time(msg, bag_time_s),
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "qx": float(pose.orientation.x),
            "qy": float(pose.orientation.y),
            "qz": float(pose.orientation.z),
            "qw": float(pose.orientation.w),
        }

    if topic in {"/mti100/data", "/mti10/data", "/imu/data"}:
        return {
            "t": msg_time(msg, bag_time_s),
            "orientation_x": float(msg.orientation.x),
            "orientation_y": float(msg.orientation.y),
            "orientation_z": float(msg.orientation.z),
            "orientation_w": float(msg.orientation.w),
            "angular_velocity_x": float(msg.angular_velocity.x),
            "angular_velocity_y": float(msg.angular_velocity.y),
            "angular_velocity_z": float(msg.angular_velocity.z),
            "linear_acceleration_x": float(msg.linear_acceleration.x),
            "linear_acceleration_y": float(msg.linear_acceleration.y),
            "linear_acceleration_z": float(msg.linear_acceleration.z),
        }

    raise ValueError(topic)


def read_bag_samples(bag_dir: Path, wanted_topics: set[str]) -> tuple[dict[str, list[dict[str, Any]]], dict[str, str], dict[str, str]]:
    if IMPORT_ERROR is not None:
        raise RuntimeError(f"rosbag2_py is not available: {IMPORT_ERROR}")
    if not (bag_dir / "metadata.yaml").exists():
        return {}, {}, {}

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = sorted(wanted_topics.intersection(topic_types))
    if not selected:
        return {}, {}, topic_types

    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    msg_types = {topic: get_message(topic_types[topic]) for topic in selected}
    samples = {topic: [] for topic in selected}
    skipped: dict[str, str] = {}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic in skipped:
            continue
        try:
            msg = deserialize_message(data, msg_types[topic])
            samples[topic].append(extract_sample(topic, msg, timestamp_ns / 1e9))
        except Exception as exc:  # noqa: BLE001
            skipped[topic] = str(exc)
            samples.pop(topic, None)

    for rows in samples.values():
        rows.sort(key=lambda row: float(row["t"]))
    return samples, skipped, topic_types


def merge_samples(base: dict[str, list[dict[str, Any]]], extra: dict[str, list[dict[str, Any]]]) -> dict[str, list[dict[str, Any]]]:
    merged = {topic: list(rows) for topic, rows in base.items()}
    for topic, rows in extra.items():
        merged.setdefault(topic, []).extend(rows)
        merged[topic].sort(key=lambda row: float(row["t"]))
    return merged


def finite_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    values = []
    for row in rows:
        try:
            value = float(row.get(key, math.nan))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def numeric_stats(rows: list[dict[str, Any]], key: str) -> dict[str, float | None]:
    values = finite_values(rows, key)
    if not values:
        return {"min": None, "mean": None, "max": None}
    return {
        "min": min(values),
        "mean": sum(values) / len(values),
        "max": max(values),
    }


def metadata_audit(counts: dict[str, int], duration_s: float, total_messages: int) -> dict[str, Any]:
    groups: dict[str, Any] = {}
    for group, topics in METADATA_GROUPS.items():
        present = {topic: int(counts.get(topic, 0)) for topic in topics if topic in counts}
        nonzero = {topic: count for topic, count in present.items() if count > 0}
        groups[group] = {
            "present_topics": present,
            "nonzero_topics": nonzero,
            "status": "ok" if nonzero else ("present_zero" if present else "missing"),
        }

    has_two_lidars = counts.get("/hesai_lidar/points", 0) > 0 and counts.get("/rsairy_ns/points", 0) > 0
    has_recorded_icp = counts.get("/mapping/icp_odom", 0) > 0
    has_recorded_perception = any(counts.get(topic, 0) > 0 for topic in PERCEPTION_TOPICS)
    max_pipeline_candidate = has_two_lidars or counts.get("/merged_points_filtered", 0) > 0 or counts.get("/hesai_lidar/points", 0) > 0

    return {
        "duration_s": duration_s,
        "total_messages": total_messages,
        "topic_count": len(counts),
        "groups": groups,
        "pipeline_inputs": {
            "has_two_lidars": has_two_lidars,
            "has_recorded_icp": has_recorded_icp,
            "has_recorded_perception": has_recorded_perception,
            "has_imu": groups["imu"]["status"] == "ok",
            "has_robot_motion": groups["robot_motion"]["status"] == "ok",
            "has_telemetry_summary": groups["telemetry_summary"]["status"] == "ok",
            "max_pipeline_candidate": max_pipeline_candidate,
        },
    }


def telemetry_summary(samples: dict[str, list[dict[str, Any]]]) -> dict[str, Any]:
    status_rows = samples.get("/mtt_status", [])
    health_rows = samples.get("/mtt_health", [])
    battery_rows = samples.get("/mtt_battery/status", [])
    tacho_rows = samples.get("/mtt_tachometer", [])

    warning_counts: dict[str, int] = {}
    for row in health_rows:
        for warning in row.get("warnings", []):
            warning_counts[str(warning)] = warning_counts.get(str(warning), 0) + 1

    source_rows = health_rows or status_rows or tacho_rows
    sources: dict[str, int] = {}
    synthetic_count = 0
    for row in source_rows:
        source = str(row.get("tachometer_source") or row.get("source") or "")
        if source:
            sources[source] = sources.get(source, 0) + 1
        synthetic_count += int(bool(row.get("tachometer_is_synthetic") or row.get("synthetic", False)))

    if status_rows:
        main_temp_rows = status_rows
        temp_a_key = "temperature_a"
        temp_b_key = "temperature_b"
    elif health_rows:
        main_temp_rows = health_rows
        temp_a_key = "main_sensor_temp_a_c"
        temp_b_key = "main_sensor_temp_b_c"
    else:
        main_temp_rows = tacho_rows
        temp_a_key = "main_sensor_temp_a"
        temp_b_key = "main_sensor_temp_b"

    return {
        "topics": {
            "/mtt_status": len(status_rows),
            "/mtt_health": len(health_rows),
            "/mtt_battery/status": len(battery_rows),
        },
        "tachometer_sources": sources,
        "tachometer_synthetic_samples": synthetic_count,
        "temperatures_c": {
            "main_sensor_a": numeric_stats(main_temp_rows, temp_a_key),
            "main_sensor_b": numeric_stats(main_temp_rows, temp_b_key),
            "battery_cell_1": numeric_stats(battery_rows, "cell_temp_1_c"),
            "battery_cell_2": numeric_stats(battery_rows, "cell_temp_2_c"),
            "battery_cell_3": numeric_stats(battery_rows, "cell_temp_3_c"),
            "battery_cell_4": numeric_stats(battery_rows, "cell_temp_4_c"),
            "battery_ambient": numeric_stats(battery_rows, "ambient_temp_c"),
            "battery_mosfet": numeric_stats(battery_rows, "mosfet_temp_c"),
        },
        "battery": {
            "soc_percent": numeric_stats(health_rows or battery_rows, "soc_percent"),
            "current_a": numeric_stats(health_rows or battery_rows, "battery_current_estimated_a"),
            "voltage_v": numeric_stats(health_rows or battery_rows, "battery_voltage_v"),
            "power_watts": numeric_stats(health_rows or battery_rows, "power_watts"),
        },
        "health_warnings_top": sorted(
            warning_counts.items(),
            key=lambda item: item[1],
            reverse=True,
        )[:10],
    }


class Series:
    def __init__(self, rows: list[dict[str, Any]]):
        self.rows = sorted(rows, key=lambda row: float(row["t"]))
        self.times = [float(row["t"]) for row in self.rows]

    def nearest(self, t: float, tolerance_s: float) -> dict[str, Any] | None:
        if not self.rows:
            return None
        idx = bisect_left(self.times, t)
        candidates = []
        if idx < len(self.rows):
            candidates.append(self.rows[idx])
        if idx:
            candidates.append(self.rows[idx - 1])
        best = min(candidates, key=lambda row: abs(float(row["t"]) - t))
        return best if abs(float(best["t"]) - t) <= tolerance_s else None


def parse_vtk_points(path: Path, duration_s: float, start_time: float | None) -> list[dict[str, Any]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    try:
        lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    except OSError:
        return []
    rows: list[dict[str, Any]] = []
    point_count = 0
    data_start = None
    for i, line in enumerate(lines):
        parts = line.split()
        if len(parts) >= 3 and parts[0].upper() == "POINTS":
            try:
                point_count = int(parts[1])
                data_start = i + 1
            except ValueError:
                return []
            break
    if data_start is None or point_count <= 0:
        return []
    values: list[float] = []
    for line in lines[data_start:]:
        if len(values) >= point_count * 3:
            break
        parts = line.split()
        if parts and parts[0].isalpha() and values:
            break
        for part in parts:
            try:
                values.append(float(part))
            except ValueError:
                return []
    if len(values) < point_count * 3:
        return []
    t0 = 0.0 if start_time is None else start_time
    denom = max(1, point_count - 1)
    for i in range(point_count):
        frac = i / denom
        rows.append(
            {
                "t": t0 + frac * duration_s,
                "x": values[i * 3],
                "y": values[i * 3 + 1],
                "z": values[i * 3 + 2],
                "qx": "",
                "qy": "",
                "qz": "",
                "qw": "",
            }
        )
    return rows


def collect_reference_times(samples: dict[str, list[dict[str, Any]]], duration_s: float) -> list[float]:
    times: set[float] = set()
    for topic in ("/mapping/icp_odom", "/mtt_odometry", "/mtt_tachometer", "/cmd_vel", "/controller/cmd_vel"):
        times.update(round(float(row["t"]), 3) for row in samples.get(topic, []))
    if not times:
        for rows in samples.values():
            times.update(round(float(row["t"]), 3) for row in rows)
    ordered = sorted(times)
    if len(ordered) > 12000:
        step = max(1, len(ordered) // 12000)
        ordered = ordered[::step]
    return ordered


def first_time(samples: dict[str, list[dict[str, Any]]]) -> float | None:
    times = [float(row["t"]) for rows in samples.values() for row in rows]
    return min(times) if times else None


def truth(value: bool) -> int:
    return 1 if value else 0


def build_dataset_rows(samples: dict[str, list[dict[str, Any]]], duration_s: float) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    times = collect_reference_times(samples, duration_s)
    start = min(times) if times else first_time(samples)
    if not times and start is not None:
        times = [start]

    icp = Series(samples.get("/mapping/icp_odom", []))
    odom = Series(samples.get("/mtt_odometry", []))
    tacho = Series(samples.get("/mtt_tachometer", []))
    imu_topic = next((topic for topic in ("/mti100/data", "/mti10/data", "/imu/data") if samples.get(topic)), "")
    imu = Series(samples.get(imu_topic, []))
    cmd_topic = next((topic for topic in ("/cmd_vel", "/controller/cmd_vel", "/cmd_vel/teleop") if samples.get(topic)), "")
    cmd = Series(samples.get(cmd_topic, []))
    mtt_angle = Series(samples.get("/mtt_articulation_angle", []))
    trailer_angle = Series(samples.get("/trailer/articulation_angle") or samples.get("/trailer/angle", []))
    trailer_pose = Series(samples.get("/trailer/pose", []))
    confidence = Series(samples.get("/trailer/pose_confidence", []))

    rows: list[dict[str, Any]] = []
    for t in times:
        icp_row = icp.nearest(t, 0.20)
        odom_row = odom.nearest(t, 0.10)
        tacho_row = tacho.nearest(t, 0.10)
        imu_row = imu.nearest(t, 0.10)
        cmd_row = cmd.nearest(t, 0.20)
        mtt_angle_row = mtt_angle.nearest(t, 0.15)
        trailer_angle_row = trailer_angle.nearest(t, 0.15)
        trailer_pose_row = trailer_pose.nearest(t, 0.15)
        confidence_row = confidence.nearest(t, 0.15)

        tach_source = str(tacho_row.get("source", "")) if tacho_row else ""
        tach_is_synthetic = bool(tacho_row.get("synthetic", False)) if tacho_row else False

        row = {field: "" for field in CSV_FIELDS}
        row.update(
            {
                "t": t,
                "bag_time_offset_s": t - start if start is not None else "",
                "has_icp": truth(icp_row is not None),
                "has_odom": truth(odom_row is not None),
                "has_tacho": truth(tacho_row is not None),
                "has_real_tacho": truth(tach_source == "real"),
                "has_cmd_sim_tacho": truth(tach_source == "cmd_sim"),
                "has_imu": truth(imu_row is not None),
                "has_trailer_pose": truth(trailer_pose_row is not None),
                "has_trailer_angle": truth(trailer_angle_row is not None),
            }
        )
        if icp_row:
            row.update({f"icp_{key}": icp_row.get(key, "") for key in ("x", "y", "z", "qx", "qy", "qz", "qw")})
        if odom_row:
            row.update({"odom_x": odom_row["x"], "odom_y": odom_row["y"], "odom_yaw": odom_row["yaw"]})
        if imu_row:
            for key in (
                "orientation_x",
                "orientation_y",
                "orientation_z",
                "orientation_w",
                "angular_velocity_x",
                "angular_velocity_y",
                "angular_velocity_z",
                "linear_acceleration_x",
                "linear_acceleration_y",
                "linear_acceleration_z",
            ):
                row[f"imu_{key}"] = imu_row[key]
        if cmd_row:
            row.update({"cmd_linear_x": cmd_row["linear_x"], "cmd_angular_z": cmd_row["angular_z"]})
        if tacho_row:
            row.update(
                {
                    "tach_speed_ms": tacho_row["speed_ms"],
                    "tach_source": tach_source,
                    "tach_is_synthetic": truth(tach_is_synthetic),
                    "tach_direction": tacho_row["direction"],
                }
            )
        if mtt_angle_row:
            row["mtt_articulation_angle"] = mtt_angle_row["value"]
        if trailer_angle_row:
            row["trailer_articulation_angle"] = trailer_angle_row["value"]
        if trailer_pose_row:
            row.update({f"trailer_pose_{key}": trailer_pose_row.get(key, "") for key in ("x", "y", "z", "qx", "qy", "qz", "qw")})
        if confidence_row:
            row["trailer_confidence"] = confidence_row["value"]
        rows.append(row)

    stats = {
        "rows": len(rows),
        "start_time": start,
        "end_time": max(times) if times else None,
        "imu_topic": imu_topic or None,
        "cmd_topic": cmd_topic or None,
        "icp_rows": sum(int(row["has_icp"]) for row in rows),
        "odom_rows": sum(int(row["has_odom"]) for row in rows),
        "tacho_rows": sum(int(row["has_tacho"]) for row in rows),
        "real_tacho_rows": sum(int(row["has_real_tacho"]) for row in rows),
        "cmd_sim_tacho_rows": sum(int(row["has_cmd_sim_tacho"]) for row in rows),
        "imu_rows": sum(int(row["has_imu"]) for row in rows),
        "trailer_pose_rows": sum(int(row["has_trailer_pose"]) for row in rows),
        "trailer_angle_rows": sum(int(row["has_trailer_angle"]) for row in rows),
    }
    conf_values = [float(row["trailer_confidence"]) for row in rows if row["trailer_confidence"] != ""]
    stats["trailer_confidence_mean"] = sum(conf_values) / len(conf_values) if conf_values else None
    return rows, stats


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def start_process(command: list[str], log_path: Path) -> ProcessHandle:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    return ProcessHandle(process, log_file)


def terminate(handle: ProcessHandle | None, grace_s: float = 10.0) -> None:
    if handle is None:
        return
    process = handle.process
    if process.poll() is None:
        try:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=grace_s)
        except Exception:  # noqa: BLE001
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    handle.log_file.close()


def run_command(command: list[str], log_path: Path, cwd: Path) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log:
        completed = subprocess.run(command, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, text=True, check=False)
    return completed.returncode


def run_offline_icp(session_dir: Path, args: argparse.Namespace, workspace_root: Path, log_dir: Path) -> int:
    script = workspace_root / "demos" / "bag_replay" / "scripts" / "offline_icp.py"
    command = [
        sys.executable,
        str(script),
        str(session_dir),
        "--offline-quality",
        args.quality,
    ]
    if args.force_icp:
        command.append("--force")
    return run_command(command, log_dir / "offline_icp.log", workspace_root)


def run_enriched_bag(session_dir: Path, duration_s: float, args: argparse.Namespace, workspace_root: Path, log_dir: Path) -> dict[str, Any]:
    bag_out = session_dir / "postprocess_dataset" / "enriched_bag"
    if not args.force_perception and (bag_out / "metadata.yaml").exists():
        counts, _, total = load_bag_metadata(bag_out / "metadata.yaml")
        return {
            "status": "skipped_existing",
            "bag_dir": str(bag_out),
            "size_bytes": directory_size_bytes(bag_out),
            "topic_counts": counts,
            "total_messages": total,
            "heavy_topics_recorded": bool(HEAVY_REPLAY_TOPICS.intersection(counts)),
        }
    if args.force_perception and bag_out.exists():
        shutil.rmtree(bag_out)

    bag_out.parent.mkdir(parents=True, exist_ok=True)
    qos_override = workspace_root / "src" / "external" / "norlab_robot" / "config" / "rosbag_record" / "qos_replay_override.yaml"
    replay_rate = args.replay_rate
    timeout_s = max(60.0, duration_s / max(replay_rate, 1e-6) + 60.0)

    description = perception = recorder = player = None
    result: dict[str, Any] = {"status": "missing_output", "bag_dir": str(bag_out)}
    try:
        description = start_process(
            [
                "ros2", "launch", "norlab_robot", "live_robot.launch.py",
                "enable_description:=true", "enable_sensors:=false", "enable_mapping:=false",
                "enable_perception:=false", "enable_localization:=false", "setup_real_can:=false",
                "publish_runtime_joint_states:=false", "use_sim_time:=true",
            ],
            log_dir / "perception_description.log",
        )
        perception = start_process(
            [
                "ros2", "launch", "mtt_perception", "perception.launch.py",
                "use_sim_time:=true", "publish_filtered:=true",
                "cloud_merger_max_pair_dt:=0.120", "cloud_merger_tf_timeout:=0.100",
            ],
            log_dir / "perception_nodes.log",
        )
        time.sleep(args.perception_startup_seconds)
        recorder = start_process(
            [
                "ros2", "bag", "record",
                "--use-sim-time",
                "--storage", "mcap",
                "--output", str(bag_out),
                *PERCEPTION_RECORD_TOPICS,
            ],
            log_dir / "perception_record.log",
        )
        time.sleep(1.0)
        player = start_process(
            [
                "ros2", "bag", "play", "--input", str(session_dir / "bag"), "mcap",
                "--clock", "--rate", str(replay_rate), "--disable-keyboard-controls",
                "--qos-profile-overrides-path", str(qos_override),
            ],
            log_dir / "perception_bag_play.log",
        )
        code = player.process.wait(timeout=timeout_s)
        if code != 0:
            result = {"status": "failed", "returncode": code, "bag_dir": str(bag_out)}
        else:
            result = {"status": "playback_ok", "bag_dir": str(bag_out)}
        time.sleep(2.0)
    except Exception as exc:  # noqa: BLE001
        result = {"status": "failed_exception", "error": str(exc), "bag_dir": str(bag_out)}
    finally:
        terminate(player)
        terminate(recorder)
        terminate(perception)
        terminate(description)

    counts, _, total = load_bag_metadata(bag_out / "metadata.yaml")
    if counts and result["status"] == "playback_ok":
        result["status"] = "ok"
    elif not counts and result["status"] == "playback_ok":
        result["status"] = "missing_output"
    result.update(
        {
            "size_bytes": directory_size_bytes(bag_out),
            "topic_counts": counts,
            "total_messages": total,
            "recorded_topics": PERCEPTION_RECORD_TOPICS,
            "excluded_heavy_topics": sorted(HEAVY_REPLAY_TOPICS),
            "heavy_topics_recorded": bool(HEAVY_REPLAY_TOPICS.intersection(counts)),
        }
    )
    return result


def grade_summary(stats: dict[str, Any], icp_summary: dict[str, Any] | None) -> str:
    rows = int(stats.get("rows", 0))
    if rows == 0:
        return "weak"
    icp_ratio = float(stats.get("icp_rows", 0)) / rows
    odom_ratio = float(stats.get("odom_rows", 0)) / rows
    tacho_ratio = float(stats.get("tacho_rows", 0)) / rows
    imu_ratio = float(stats.get("imu_rows", 0)) / rows
    trailer_ratio = max(float(stats.get("trailer_pose_rows", 0)), float(stats.get("trailer_angle_rows", 0))) / rows
    icp_ok = bool(icp_summary and str(icp_summary.get("status")) in {"ok", "skipped_existing"}) or icp_ratio > 0.25
    if icp_ok and odom_ratio > 0.8 and tacho_ratio > 0.8 and imu_ratio > 0.5 and trailer_ratio > 0.3:
        return "excellent"
    if icp_ok and odom_ratio > 0.5 and tacho_ratio > 0.5:
        return "good"
    if odom_ratio > 0.2 or tacho_ratio > 0.2 or icp_ratio > 0.2:
        return "usable"
    return "weak"


def load_yaml(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception:
        return None


def process_session(session_dir: Path, args: argparse.Namespace, workspace_root: Path) -> dict[str, Any]:
    output_dir = session_dir / "postprocess_dataset"
    log_dir = output_dir / "logs"
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    counts, duration_s, total_messages = load_metadata(session_dir)

    result: dict[str, Any] = {
        "session": session_dir.name,
        "session_dir": str(session_dir),
        "bag_dir": str(session_dir / "bag"),
        "duration_s": duration_s,
        "total_messages": total_messages,
        "status": "failed",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "metadata_audit": metadata_audit(counts, duration_s, total_messages),
        "topic_counts": {topic: counts.get(topic, 0) for topic in sorted(ALL_TOPICS | {"/hesai_lidar/points", "/rsairy_ns/points", "/merged_points_filtered"})},
    }
    if not counts:
        result["status"] = "skipped_missing_metadata"
        return result

    if args.run_icp:
        result["offline_icp_returncode"] = run_offline_icp(session_dir, args, workspace_root, log_dir)
    icp_summary = load_yaml(session_dir / "offline_icp" / "summary.yaml")
    result["icp"] = icp_summary

    if args.run_perception:
        result["enriched_bag"] = run_enriched_bag(session_dir, duration_s, args, workspace_root, log_dir)

    try:
        samples, skipped, _ = read_bag_samples(session_dir / "bag", ALL_TOPICS)
    except Exception as exc:  # noqa: BLE001
        result["status"] = "failed_read_bag"
        result["error"] = str(exc)
        result["quality"] = {"grade": "weak"}
        return result

    enriched_bag = session_dir / "postprocess_dataset" / "enriched_bag"
    if (enriched_bag / "metadata.yaml").exists():
        try:
            perception_samples, perception_skipped, _ = read_bag_samples(enriched_bag, PERCEPTION_TOPICS)
            samples = merge_samples(samples, perception_samples)
            skipped.update({f"perception:{topic}": error for topic, error in perception_skipped.items()})
        except Exception as exc:  # noqa: BLE001
            skipped["enriched_bag"] = str(exc)

    if not samples.get("/mapping/icp_odom"):
        start = first_time(samples)
        vtk_rows = parse_vtk_points(session_dir / "offline_icp" / "trajectory.vtk", duration_s, start)
        if not vtk_rows:
            vtk_rows = parse_vtk_points(session_dir / "trajectory.vtk", duration_s, start)
        if vtk_rows:
            samples["/mapping/icp_odom"] = vtk_rows
            result["icp_trajectory_source"] = "trajectory_vtk"

    rows, stats = build_dataset_rows(samples, duration_s)
    dataset_csv = output_dir / "dataset.csv"
    write_csv(rows, dataset_csv)

    result["status"] = "ok" if rows else "skipped_no_rows"
    result["dataset_csv"] = str(dataset_csv)
    result["skipped_topics"] = skipped
    result["stats"] = stats
    result["telemetry_summary"] = telemetry_summary(samples)
    result["quality"] = {
        "grade": grade_summary(stats, icp_summary),
        "icp": {
            "mode": icp_summary.get("mode") if icp_summary else None,
            "status": icp_summary.get("status") if icp_summary else None,
            "map_size_bytes": icp_summary.get("map_size_bytes") if icp_summary else None,
            "trajectory_size_bytes": icp_summary.get("trajectory_size_bytes") if icp_summary else None,
        },
        "perception": {
            "trailer_angle_rows": stats["trailer_angle_rows"],
            "trailer_pose_rows": stats["trailer_pose_rows"],
            "confidence_mean": stats["trailer_confidence_mean"],
        },
        "robot_data": {
            "odom_present": stats["odom_rows"] > 0,
            "tacho_present": stats["tacho_rows"] > 0,
            "real_tacho_present": stats["real_tacho_rows"] > 0,
            "cmd_sim_tacho_present": stats["cmd_sim_tacho_rows"] > 0,
            "imu_present": stats["imu_rows"] > 0,
            "commands_present": stats["cmd_topic"] is not None,
        },
    }
    return result


def parse_args() -> argparse.Namespace:
    workspace_root = infer_workspace_root()
    parser = argparse.ArgumentParser(description="Build postprocess_dataset/dataset.csv for MTT bag sessions.")
    parser.add_argument("input_path", nargs="?", default=str(workspace_root / "data"))
    parser.add_argument("--run-icp", action="store_true", help="Run/reuse offline ICP before CSV export.")
    parser.add_argument("--force-icp", action="store_true", help="Force offline ICP rebuild.")
    parser.add_argument("--run-perception", "--run-enriched-bag", dest="run_perception", action="store_true", help="Replay perception and record a lightweight enriched bag before CSV export.")
    parser.add_argument("--force-perception", "--force-enriched-bag", dest="force_perception", action="store_true", help="Re-record the lightweight enriched bag.")
    parser.add_argument("--quality", default="standard", choices=["standard", "max"], help="Offline ICP quality profile.")
    parser.add_argument("--replay-rate", type=float, default=1.0, help="Replay rate for enriched bag generation.")
    parser.add_argument("--perception-startup-seconds", type=float, default=4.0)
    parser.add_argument("--strict", action="store_true", help="Return non-zero if any session fails.")
    parser.add_argument("--metadata-only", action="store_true", help="Only audit metadata and write summaries/report; do not read messages or run replay.")
    return parser.parse_args()


def main() -> int:
    workspace_root = infer_workspace_root()
    args = parse_args()
    sessions = resolve_sessions(args.input_path)
    report = []
    failures = 0

    for index, session_dir in enumerate(sessions, start=1):
        print(f"[{index}/{len(sessions)}] {session_dir.name}")
        try:
            if args.metadata_only:
                counts, duration_s, total_messages = load_metadata(session_dir)
                result = {
                    "session": session_dir.name,
                    "session_dir": str(session_dir),
                    "bag_dir": str(session_dir / "bag"),
                    "status": "metadata_ok" if counts else "skipped_missing_metadata",
                    "metadata_audit": metadata_audit(counts, duration_s, total_messages),
                    "created_at": datetime.now().isoformat(timespec="seconds"),
                }
            else:
                result = process_session(session_dir, args, workspace_root)
        except Exception as exc:  # noqa: BLE001
            result = {
                "session": session_dir.name,
                "session_dir": str(session_dir),
                "status": "failed_exception",
                "error": str(exc),
                "created_at": datetime.now().isoformat(timespec="seconds"),
            }
        output_dir = session_dir / "postprocess_dataset"
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "summary.yaml").write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
        report.append(result)
        print(f"  {result['status']}  grade={result.get('quality', {}).get('grade', 'n/a')}")
        if str(result["status"]).startswith("failed"):
            failures += 1

    report_path = workspace_root / "data" / "postprocess_dataset_report.yaml"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    print(f"Report: {report_path}")
    return 1 if args.strict and failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
