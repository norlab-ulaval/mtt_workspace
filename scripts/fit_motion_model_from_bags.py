#!/usr/bin/env python3
"""Fit simple MTT command-motion models from bag data using offline ICP as reference."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import statistics
import sys
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


MAX_THROTTLE_RAW = 230.0
MAX_BRAKE_RAW = 255.0
DEFAULT_MAX_LINEAR_SPEED_MS = 1.0
DEFAULT_WHEELBASE_M = 2.4
DEFAULT_MAX_ARTICULATION_RAD = math.radians(60.0)

TOPICS = {
    "/mapping/icp_odom",
    "/cmd_vel",
    "/cmd_vel/teleop",
    "/cmd_vel/teleop_raw",
    "/cmd_vel/manual",
    "/cmd_vel/manual_raw",
    "/controller/cmd_vel",
    "/mtt_status",
    "/mtt_health",
    "/mtt_tachometer",
    "/mtt_odometry",
    "/mtt_articulation_angle",
    "/mtt_steer_cmd",
    "/mtt_aux_cmd",
    "/mti100/data",
    "/mti10/data",
    "/imu/data",
}

CSV_FIELDS = [
    "session",
    "t",
    "bag_time_offset_s",
    "icp_x",
    "icp_y",
    "icp_yaw",
    "v_icp_ms",
    "yaw_rate_icp_rad_s",
    "icp_pose_step_m",
    "icp_quality_ok",
    "cmd_linear_x",
    "cmd_angular_z",
    "teleop_linear_x",
    "teleop_raw_linear_x",
    "controller_linear_x",
    "effective_cmd_linear_x",
    "throttle_raw",
    "throttle_norm",
    "throttle_source",
    "has_observed_command",
    "brake_raw",
    "brake_norm",
    "brake_source",
    "steer_norm",
    "steer_source",
    "articulation_rad",
    "articulation_source",
    "imu_yaw_rate_z",
    "imu_pitch_rad",
    "imu_roll_rad",
    "tach_speed_ms",
    "tach_model_speed_ms",
    "tachometer_source",
    "tachometer_is_synthetic",
    "command_timeout_active",
    "emergency_stop_active",
    "segment_label",
    "v_model_ms",
    "v_target_model_ms",
    "yaw_rate_model_rad_s",
    "model_x",
    "model_y",
    "model_yaw",
]


def infer_workspace_root(script_path: Path) -> Path:
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


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def msg_time(msg: Any, bag_time_s: float) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and (stamp.sec or stamp.nanosec):
        return stamp_to_sec(stamp)
    return bag_time_s


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_to_roll_pitch(q: Any) -> tuple[float, float]:
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


def twist_from_msg(msg: Any) -> Any:
    if hasattr(msg, "twist") and hasattr(msg.twist, "twist"):
        return msg.twist.twist
    if hasattr(msg, "twist"):
        return msg.twist
    return msg


def extract_sample(topic: str, msg: Any, bag_time_s: float) -> dict[str, Any]:
    t = msg_time(msg, bag_time_s)
    if topic in {
        "/cmd_vel",
        "/cmd_vel/teleop",
        "/cmd_vel/teleop_raw",
        "/cmd_vel/manual",
        "/cmd_vel/manual_raw",
        "/controller/cmd_vel",
    }:
        twist = twist_from_msg(msg)
        return {"t": t, "linear_x": float(twist.linear.x), "angular_z": float(twist.angular.z)}

    if topic in {"/mapping/icp_odom", "/mtt_odometry"}:
        pose = msg.pose.pose
        return {
            "t": t,
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": quaternion_to_yaw(pose.orientation),
            "linear_x": float(msg.twist.twist.linear.x),
            "angular_z": float(msg.twist.twist.angular.z),
        }

    if topic == "/mtt_tachometer":
        return {
            "t": t,
            "speed_ms": float(getattr(msg, "speed_ms", 0.0)),
            "model_speed_ms": float(getattr(msg, "model_speed_ms", 0.0)),
            "steer_cmd": float(getattr(msg, "steer_cmd", 0.0)),
            "model_articulation_rad": float(getattr(msg, "model_articulation_effective_rad", 0.0)),
            "model_yaw_rate_rad_s": float(getattr(msg, "model_yaw_rate_effective_rad_s", 0.0)),
            "source": str(getattr(msg, "tachometer_source", "")),
            "synthetic": bool(getattr(msg, "tachometer_is_synthetic", False)),
        }

    if topic == "/mtt_status":
        return {
            "t": t,
            "speed_ms": float(getattr(msg, "speed_ms", 0.0)),
            "steer_norm": float(getattr(msg, "steer_normalized", 0.0)),
            "throttle_raw": int(getattr(msg, "throttle_raw", 0)),
            "brake_raw": int(getattr(msg, "brake_raw", 0)),
            "command_linear_speed_ms": float(getattr(msg, "command_linear_speed_ms", 0.0)),
            "effective_linear_speed_command_ms": float(
                getattr(msg, "effective_linear_speed_command_ms", 0.0)
            ),
            "command_timeout_active": bool(getattr(msg, "command_timeout_active", False)),
            "emergency_stop_active": bool(getattr(msg, "emergency_stop_active", False)),
            "tachometer_source": str(getattr(msg, "tachometer_source", "")),
            "tachometer_is_synthetic": bool(getattr(msg, "tachometer_is_synthetic", False)),
        }

    if topic == "/mtt_health":
        return {
            "t": t,
            "throttle_raw": int(getattr(msg, "throttle_raw", 0)),
            "brake_raw": int(getattr(msg, "brake_raw", 0)),
            "tachometer_source": str(getattr(msg, "tachometer_source", "")),
            "tachometer_is_synthetic": bool(getattr(msg, "tachometer_is_synthetic", False)),
            "command_timeout_active": bool(getattr(msg, "command_timeout_active", False)),
            "emergency_stop_active": bool(getattr(msg, "emergency_stop_active", False)),
        }

    if topic == "/mtt_articulation_angle":
        return {"t": t, "value": float(msg.data)}

    if topic == "/mtt_steer_cmd":
        return {"t": t, "value": float(getattr(msg, "data", 0.0))}

    if topic == "/mtt_aux_cmd":
        return {
            "t": t,
            "brake_norm": float(getattr(msg, "brake", 0.0)),
            "winch_command": int(getattr(msg, "winch_command", 0)),
            "light_state": int(getattr(msg, "light_state", 0)),
        }

    if topic in {"/mti100/data", "/mti10/data", "/imu/data"}:
        roll, pitch = quaternion_to_roll_pitch(msg.orientation)
        return {
            "t": t,
            "yaw_rate_z": float(msg.angular_velocity.z),
            "pitch": pitch,
            "roll": roll,
        }

    raise ValueError(topic)


class Series:
    def __init__(self, rows: list[dict[str, Any]]):
        self.rows = sorted(rows, key=lambda row: float(row["t"]))
        self.times = [float(row["t"]) for row in self.rows]

    def nearest(self, t: float, tolerance_s: float) -> dict[str, Any] | None:
        if not self.rows:
            return None
        idx = bisect.bisect_left(self.times, t)
        candidates = []
        if idx < len(self.rows):
            candidates.append(self.rows[idx])
        if idx > 0:
            candidates.append(self.rows[idx - 1])
        best = min(candidates, key=lambda row: abs(float(row["t"]) - t))
        return best if abs(float(best["t"]) - t) <= tolerance_s else None


def read_bag_samples(bag_dir: Path, wanted_topics: set[str]) -> tuple[dict[str, list[dict[str, Any]]], dict[str, str]]:
    if IMPORT_ERROR is not None:
        raise SystemExit(f"rosbag2_py is not available: {IMPORT_ERROR}")
    if not (bag_dir / "metadata.yaml").exists():
        return {}, {"metadata": "missing"}

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    selected = sorted(wanted_topics.intersection(type_map))
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    msg_types = {topic: get_message(type_map[topic]) for topic in selected}
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
    return samples, skipped


def read_offline_icp(session_dir: Path) -> tuple[list[dict[str, Any]], str]:
    for label, replay in (
        ("offline_icp_canonical/icp_odom_replay", session_dir / "offline_icp_canonical" / "icp_odom_replay"),
        ("offline_icp/icp_odom_replay", session_dir / "offline_icp" / "icp_odom_replay"),
    ):
        if (replay / "metadata.yaml").exists():
            samples, _ = read_bag_samples(replay, {"/mapping/icp_odom"})
            rows = samples.get("/mapping/icp_odom", [])
            if rows:
                return rows, label
    return [], "missing"


def derive_icp_kinematics(rows: list[dict[str, Any]], smooth_window_s: float) -> list[dict[str, Any]]:
    if not rows:
        return []
    rows = sorted(rows, key=lambda row: float(row["t"]))
    ts = [float(row["t"]) for row in rows]
    xs = [float(row["x"]) for row in rows]
    ys = [float(row["y"]) for row in rows]
    yaws = [float(row["yaw"]) for row in rows]
    half = smooth_window_s / 2.0
    out: list[dict[str, Any]] = []
    prev_xy: tuple[float, float] | None = None

    for i, row in enumerate(rows):
        t = ts[i]
        j0 = bisect.bisect_left(ts, t - half)
        j1 = bisect.bisect_right(ts, t + half) - 1
        step = 0.0 if prev_xy is None else math.hypot(xs[i] - prev_xy[0], ys[i] - prev_xy[1])
        prev_xy = (xs[i], ys[i])
        if j1 > j0 and ts[j1] > ts[j0]:
            dt = ts[j1] - ts[j0]
            dx = xs[j1] - xs[j0]
            dy = ys[j1] - ys[j0]
            heading_mid = wrap_angle(0.5 * (yaws[j0] + yaws[j1]))
            v = (dx * math.cos(heading_mid) + dy * math.sin(heading_mid)) / dt
            yaw_rate = wrap_angle(yaws[j1] - yaws[j0]) / dt
        else:
            v = float(row.get("linear_x", 0.0))
            yaw_rate = float(row.get("angular_z", 0.0))
        out.append({**row, "v_icp_ms": v, "yaw_rate_icp_rad_s": yaw_rate, "pose_step_m": step})
    return out


def reference_times(samples: dict[str, list[dict[str, Any]]], icp_rows: list[dict[str, Any]]) -> list[float]:
    times: set[float] = set()
    for topic in (
        "/mtt_status",
        "/mtt_tachometer",
        "/mtt_odometry",
        "/cmd_vel",
        "/cmd_vel/teleop",
        "/controller/cmd_vel",
    ):
        times.update(round(float(row["t"]), 3) for row in samples.get(topic, []))
    if not times:
        times.update(round(float(row["t"]), 3) for row in icp_rows)
    return sorted(times)


def signed_cmd_linear(row: dict[str, Any]) -> float | None:
    for key in ("effective_cmd_linear_x", "cmd_linear_x", "teleop_linear_x", "controller_linear_x"):
        value = row.get(key)
        if value is not None and math.isfinite(float(value)):
            return float(value)
    return None


def finite(value: Any) -> bool:
    try:
        return value is not None and math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def build_rows(
    session_dir: Path,
    samples: dict[str, list[dict[str, Any]]],
    icp_rows: list[dict[str, Any]],
    *,
    max_linear_speed_ms: float,
    max_articulation_rad: float,
    icp_step_threshold_m: float,
) -> list[dict[str, Any]]:
    icp_series = Series(derive_icp_kinematics(icp_rows, smooth_window_s=1.0))
    series = {topic: Series(rows) for topic, rows in samples.items()}
    times = reference_times(samples, icp_rows)
    if not times:
        return []
    t0 = times[0]
    rows: list[dict[str, Any]] = []

    for t in times:
        icp = icp_series.nearest(t, 0.25)
        if icp is None:
            continue
        cmd = series.get("/cmd_vel", Series([])).nearest(t, 0.1)
        teleop = series.get("/cmd_vel/teleop", Series([])).nearest(t, 0.1)
        teleop_raw = series.get("/cmd_vel/teleop_raw", Series([])).nearest(t, 0.1)
        manual = series.get("/cmd_vel/manual", Series([])).nearest(t, 0.1)
        manual_raw = series.get("/cmd_vel/manual_raw", Series([])).nearest(t, 0.1)
        controller = series.get("/controller/cmd_vel", Series([])).nearest(t, 0.1)
        status = series.get("/mtt_status", Series([])).nearest(t, 0.1)
        health = series.get("/mtt_health", Series([])).nearest(t, 0.2)
        tacho = series.get("/mtt_tachometer", Series([])).nearest(t, 0.1)
        articulation = series.get("/mtt_articulation_angle", Series([])).nearest(t, 0.1)
        steer_cmd = series.get("/mtt_steer_cmd", Series([])).nearest(t, 0.1)
        aux = series.get("/mtt_aux_cmd", Series([])).nearest(t, 0.15)
        imu = (
            series.get("/mti100/data", Series([])).nearest(t, 0.05)
            or series.get("/mti10/data", Series([])).nearest(t, 0.05)
            or series.get("/imu/data", Series([])).nearest(t, 0.05)
        )

        primary_cmd = cmd or controller or teleop or teleop_raw or manual or manual_raw
        cmd_linear = float(primary_cmd["linear_x"]) if primary_cmd else None
        has_observed_command = (
            cmd is not None
            or teleop is not None
            or teleop_raw is not None
            or manual is not None
            or manual_raw is not None
            or controller is not None
        )
        effective_cmd = (
            float(status["effective_linear_speed_command_ms"])
            if status and finite(status.get("effective_linear_speed_command_ms"))
            else cmd_linear
        )

        throttle_raw = None
        throttle_source = "missing"
        if status and finite(status.get("throttle_raw")):
            throttle_raw = float(status["throttle_raw"])
            throttle_source = "mtt_status"
            has_observed_command = True
        elif health and finite(health.get("throttle_raw")) and float(health["throttle_raw"]) > 0.0:
            throttle_raw = float(health["throttle_raw"])
            throttle_source = "mtt_health"
            has_observed_command = True
        elif effective_cmd is not None:
            throttle_raw = min(MAX_THROTTLE_RAW, abs(effective_cmd) / max_linear_speed_ms * MAX_THROTTLE_RAW)
            throttle_source = "reconstructed_cmd_vel"
            has_observed_command = True

        brake_raw = None
        brake_source = "missing"
        if status and finite(status.get("brake_raw")):
            brake_raw = float(status["brake_raw"])
            brake_source = "mtt_status"
        elif health and finite(health.get("brake_raw")) and float(health["brake_raw"]) > 0.0:
            brake_raw = float(health["brake_raw"])
            brake_source = "mtt_health"
        elif aux and finite(aux.get("brake_norm")):
            brake_raw = float(aux["brake_norm"]) * MAX_BRAKE_RAW
            brake_source = "mtt_aux_cmd"

        steer_norm = None
        steer_source = "missing"
        if status and finite(status.get("steer_norm")):
            steer_norm = float(status["steer_norm"])
            steer_source = "mtt_status"
        elif tacho and finite(tacho.get("steer_cmd")):
            steer_norm = float(tacho["steer_cmd"])
            steer_source = "mtt_tachometer"
        elif steer_cmd and finite(steer_cmd.get("value")):
            steer_norm = float(steer_cmd["value"])
            steer_source = "mtt_steer_cmd"
        elif cmd and finite(cmd.get("angular_z")):
            steer_norm = max(-1.0, min(1.0, float(cmd["angular_z"])))
            steer_source = "cmd_angular_z"

        articulation_rad = None
        articulation_source = "missing"
        if articulation and finite(articulation.get("value")):
            articulation_rad = float(articulation["value"])
            articulation_source = "mtt_articulation_angle"
        elif tacho and finite(tacho.get("model_articulation_rad")):
            articulation_rad = float(tacho["model_articulation_rad"])
            articulation_source = "tacho_model"
        elif steer_norm is not None:
            articulation_rad = max(-1.0, min(1.0, steer_norm)) * max_articulation_rad
            articulation_source = f"{steer_source}_scaled"

        pose_step = float(icp.get("pose_step_m", 0.0))
        row = {
            "session": session_dir.name,
            "t": t,
            "bag_time_offset_s": t - t0,
            "icp_x": float(icp["x"]),
            "icp_y": float(icp["y"]),
            "icp_yaw": float(icp["yaw"]),
            "v_icp_ms": float(icp["v_icp_ms"]),
            "yaw_rate_icp_rad_s": float(icp["yaw_rate_icp_rad_s"]),
            "icp_pose_step_m": pose_step,
            "icp_quality_ok": pose_step <= icp_step_threshold_m,
            "cmd_linear_x": cmd_linear,
            "cmd_angular_z": float(cmd["angular_z"]) if cmd else None,
            "teleop_linear_x": float(teleop["linear_x"]) if teleop else None,
            "teleop_raw_linear_x": float(teleop_raw["linear_x"]) if teleop_raw else None,
            "controller_linear_x": float(controller["linear_x"]) if controller else None,
            "effective_cmd_linear_x": effective_cmd,
            "throttle_raw": throttle_raw,
            "throttle_norm": None if throttle_raw is None else throttle_raw / MAX_THROTTLE_RAW,
            "throttle_source": throttle_source,
            "has_observed_command": has_observed_command,
            "brake_raw": brake_raw,
            "brake_norm": None if brake_raw is None else brake_raw / MAX_BRAKE_RAW,
            "brake_source": brake_source,
            "steer_norm": steer_norm,
            "steer_source": steer_source,
            "articulation_rad": articulation_rad,
            "articulation_source": articulation_source,
            "imu_yaw_rate_z": float(imu["yaw_rate_z"]) if imu else None,
            "imu_pitch_rad": float(imu["pitch"]) if imu else None,
            "imu_roll_rad": float(imu["roll"]) if imu else None,
            "tach_speed_ms": float(tacho["speed_ms"]) if tacho else None,
            "tach_model_speed_ms": float(tacho["model_speed_ms"]) if tacho else None,
            "tachometer_source": (
                str(status["tachometer_source"])
                if status and status.get("tachometer_source")
                else (str(tacho["source"]) if tacho else None)
            ),
            "tachometer_is_synthetic": (
                bool(status["tachometer_is_synthetic"])
                if status and "tachometer_is_synthetic" in status
                else (bool(tacho["synthetic"]) if tacho else None)
            ),
            "command_timeout_active": bool(status["command_timeout_active"]) if status else False,
            "emergency_stop_active": bool(status["emergency_stop_active"]) if status else False,
        }
        row["segment_label"] = label_row(row)
        rows.append(row)

    return rows


def label_row(row: dict[str, Any]) -> str:
    if not row.get("icp_quality_ok"):
        return "rejected_icp_jump"
    if row.get("command_timeout_active") or row.get("emergency_stop_active"):
        return "rejected_safety"
    if not finite(row.get("v_icp_ms")):
        return "rejected_no_icp_speed"
    articulation = abs(float(row["articulation_rad"])) if finite(row.get("articulation_rad")) else 0.0
    speed = abs(float(row["v_icp_ms"]))
    has_command = bool(row.get("has_observed_command"))
    throttle = float(row["throttle_norm"]) if finite(row.get("throttle_norm")) else 0.0
    brake = float(row["brake_norm"]) if finite(row.get("brake_norm")) else 0.0
    if (not has_command or throttle < 0.03) and speed > 0.05:
        return "observed_motion_only_yaw" if articulation >= 0.08 else "observed_motion_only_longitudinal"
    if throttle < 0.03 and speed < 0.05:
        return "static"
    if articulation < 0.08 and speed > 0.05:
        return "longitudinal"
    if articulation >= 0.08 and speed > 0.05:
        return "yaw"
    if brake > 0.15 and row.get("brake_source") in {"mtt_status", "mtt_health"}:
        return "diagnostic_braking"
    return "diagnostic"


def fit_slope(xs: list[float], ys: list[float], through_zero: bool = True) -> tuple[float | None, float | None]:
    if len(xs) < 2:
        return None, None
    if through_zero:
        denom = sum(x * x for x in xs)
        if denom <= 1e-12:
            return None, None
        return sum(x * y for x, y in zip(xs, ys)) / denom, 0.0
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    denom = sum((x - x_mean) ** 2 for x in xs)
    if denom <= 1e-12:
        return None, None
    slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denom
    return slope, y_mean - slope * x_mean


def command_direction(row: dict[str, Any], command_sign: float | None) -> float:
    cmd = signed_cmd_linear(row)
    if command_sign is not None and cmd is not None:
        return math.copysign(1.0, command_sign * cmd) if abs(cmd) > 1e-6 else 0.0
    if finite(row.get("v_icp_ms")) and abs(float(row["v_icp_ms"])) > 0.05:
        return math.copysign(1.0, float(row["v_icp_ms"]))
    return 0.0


def simulate_unit_speed_response(
    rows: list[dict[str, Any]],
    *,
    tau_s: float,
    deadband: float,
    command_sign: float | None,
) -> list[float]:
    v_unit = 0.0
    prev_t = None
    values_out: list[float] = []
    for row in rows:
        t = float(row["t"])
        dt = 0.0 if prev_t is None else max(0.0, min(t - prev_t, 1.0))
        direction = command_direction(row, command_sign)
        throttle = float(row["throttle_norm"]) if finite(row.get("throttle_norm")) else 0.0
        throttle_eff = max(0.0, throttle - deadband)
        target_unit = throttle_eff * direction
        if dt > 0.0:
            alpha = 1.0 if tau_s <= 1e-6 else 1.0 - math.exp(-dt / tau_s)
            v_unit += (target_unit - v_unit) * max(0.0, min(1.0, alpha))
        values_out.append(v_unit)
        prev_t = t
    return values_out


def fit_dynamic_speed_model(
    rows: list[dict[str, Any]],
    command_sign: float | None,
) -> dict[str, float | None]:
    fit_rows = [
        row
        for row in rows
        if row["segment_label"] == "longitudinal"
        and bool(row.get("has_observed_command"))
        and finite(row.get("throttle_norm"))
        and finite(row.get("v_icp_ms"))
    ]
    if len(fit_rows) < 20:
        return {
            "dynamic_speed_gain_ms": None,
            "dynamic_speed_tau_s": None,
            "dynamic_speed_deadband": None,
            "dynamic_speed_rmse_ms": None,
        }

    targets = [float(row["v_icp_ms"]) for row in fit_rows]
    best: dict[str, float | None] = {
        "dynamic_speed_gain_ms": None,
        "dynamic_speed_tau_s": None,
        "dynamic_speed_deadband": None,
        "dynamic_speed_rmse_ms": None,
    }
    best_rmse = math.inf
    tau_grid = [0.05, 0.10, 0.15, 0.20, 0.30, 0.45, 0.60, 0.80, 1.00, 1.40, 1.80, 2.50, 3.50]
    deadband_grid = [0.00, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12]
    for tau_s in tau_grid:
        for deadband in deadband_grid:
            unit = simulate_unit_speed_response(
                fit_rows,
                tau_s=tau_s,
                deadband=deadband,
                command_sign=command_sign,
            )
            denom = sum(v * v for v in unit)
            if denom <= 1e-12:
                continue
            gain = sum(u * y for u, y in zip(unit, targets)) / denom
            pred = [gain * u for u in unit]
            err = rmse(pred, targets)
            if err is not None and err < best_rmse:
                best_rmse = err
                best = {
                    "dynamic_speed_gain_ms": gain,
                    "dynamic_speed_tau_s": tau_s,
                    "dynamic_speed_deadband": deadband,
                    "dynamic_speed_rmse_ms": err,
                }
    return best


def rmse(values_a: list[float], values_b: list[float]) -> float | None:
    if not values_a or len(values_a) != len(values_b):
        return None
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(values_a, values_b)) / len(values_a))


def fit_models(rows: list[dict[str, Any]], wheelbase_m: float) -> dict[str, Any]:
    good = [
        row
        for row in rows
        if row.get("icp_quality_ok")
        and not row.get("command_timeout_active")
        and not row.get("emergency_stop_active")
        and finite(row.get("v_icp_ms"))
    ]
    longitudinal = [
        row
        for row in good
        if row["segment_label"] == "longitudinal"
        and finite(row.get("throttle_norm"))
        and bool(row.get("has_observed_command"))
    ]
    xs_abs = [float(row["throttle_norm"]) for row in longitudinal]
    ys_abs = [abs(float(row["v_icp_ms"])) for row in longitudinal]
    throttle_to_speed_gain, throttle_speed_bias = fit_slope(xs_abs, ys_abs, through_zero=False)
    throttle_to_speed_gain_zero, _ = fit_slope(xs_abs, ys_abs, through_zero=True)

    cmd_rows = [
        row
        for row in good
        if finite(row.get("cmd_linear_x"))
        and bool(row.get("has_observed_command"))
        and abs(float(row["v_icp_ms"])) > 0.05
    ]
    cmd_slope, _ = fit_slope(
        [float(row["cmd_linear_x"]) for row in cmd_rows],
        [float(row["v_icp_ms"]) for row in cmd_rows],
        through_zero=True,
    )
    command_sign = None
    if cmd_slope is not None:
        command_sign = 1.0 if cmd_slope >= 0.0 else -1.0
    dynamic_speed = fit_dynamic_speed_model(longitudinal, command_sign)

    yaw_rows = [
        row
        for row in good
        if row["segment_label"] in {"yaw", "observed_motion_only_yaw"}
        and finite(row.get("articulation_rad"))
        and abs(float(row["v_icp_ms"])) > 0.05
    ]
    yaw_x = [
        float(row["v_icp_ms"]) * math.tan(float(row["articulation_rad"])) / wheelbase_m
        for row in yaw_rows
    ]
    yaw_y = [float(row["yaw_rate_icp_rad_s"]) for row in yaw_rows]
    yaw_gain, _ = fit_slope(yaw_x, yaw_y, through_zero=True)
    speed_gain_for_trust = throttle_to_speed_gain_zero or throttle_to_speed_gain
    speed_rmse = (
        rmse(
            [
                (throttle_to_speed_gain or 0.0) * x + (throttle_speed_bias or 0.0)
                for x in xs_abs
            ],
            ys_abs,
        )
        if throttle_to_speed_gain is not None
        else None
    )
    yaw_rmse = rmse([(yaw_gain or 0.0) * x for x in yaw_x], yaw_y) if yaw_gain is not None else None
    longitudinal_gain_plausible = (
        speed_gain_for_trust is not None
        and 0.5 <= abs(float(speed_gain_for_trust)) <= 5.0
    )
    yaw_gain_plausible = yaw_gain is not None and 0.1 <= float(yaw_gain) <= 5.0
    dynamic_rmse = dynamic_speed.get("dynamic_speed_rmse_ms")
    use_dynamic_speed_model = (
        dynamic_speed.get("dynamic_speed_gain_ms") is not None
        and dynamic_rmse is not None
        and speed_rmse is not None
        and float(dynamic_rmse) < 0.95 * float(speed_rmse)
    )

    return {
        "row_count": len(rows),
        "good_icp_rows": len(good),
        "longitudinal_rows": len(longitudinal),
        "yaw_rows": len(yaw_rows),
        "observed_motion_only_rows": sum(
            1 for row in good if str(row.get("segment_label", "")).startswith("observed_motion_only")
        ),
        "throttle_to_abs_speed_gain_ms": throttle_to_speed_gain,
        "throttle_to_abs_speed_bias_ms": throttle_speed_bias,
        "throttle_to_abs_speed_gain_zero_ms": throttle_to_speed_gain_zero,
        "cmd_linear_to_v_icp_gain": cmd_slope,
        "cmd_linear_physical_sign": command_sign,
        **dynamic_speed,
        "yaw_gain": yaw_gain,
        "speed_model_type": "dynamic_first_order" if use_dynamic_speed_model else "static_gain",
        "use_dynamic_speed_model": use_dynamic_speed_model,
        "longitudinal_gain_plausible": longitudinal_gain_plausible,
        "yaw_gain_plausible": yaw_gain_plausible,
        "wheelbase_m": wheelbase_m,
        "speed_rmse_ms": speed_rmse,
        "yaw_rate_rmse_rad_s": yaw_rmse,
        "trusted_for_longitudinal": (
            len(longitudinal) >= 20
            and throttle_to_speed_gain is not None
            and longitudinal_gain_plausible
            and (speed_rmse is None or speed_rmse <= 1.0)
        ),
        "trusted_for_yaw": len(yaw_rows) >= 20 and yaw_gain is not None and yaw_gain_plausible,
    }


def apply_model(rows: list[dict[str, Any]], fit: dict[str, Any], *, wheelbase_m: float) -> None:
    use_dynamic = bool(fit.get("use_dynamic_speed_model"))
    if use_dynamic:
        gain = fit.get("dynamic_speed_gain_ms") or DEFAULT_MAX_LINEAR_SPEED_MS
        deadband = fit.get("dynamic_speed_deadband")
        deadband = float(deadband) if deadband is not None else 0.0
        tau_s = fit.get("dynamic_speed_tau_s")
        tau_s = float(tau_s) if tau_s is not None else 0.0
    else:
        gain = (
            fit.get("throttle_to_abs_speed_gain_zero_ms")
            or fit.get("throttle_to_abs_speed_gain_ms")
            or DEFAULT_MAX_LINEAR_SPEED_MS
        )
        deadband = 0.0
        tau_s = 0.0
    yaw_gain = fit.get("yaw_gain") if fit.get("yaw_gain") is not None else 1.0
    command_sign = fit.get("cmd_linear_physical_sign")
    x = y = yaw = 0.0
    v = 0.0
    prev_t = None
    for row in rows:
        t = float(row["t"])
        dt = 0.0 if prev_t is None else max(0.0, min(t - prev_t, 1.0))
        direction = command_direction(row, command_sign)
        throttle = float(row["throttle_norm"]) if finite(row.get("throttle_norm")) else 0.0
        throttle_eff = max(0.0, throttle - deadband)
        v_target = float(gain) * throttle_eff * direction
        if dt > 0.0:
            alpha = 1.0 if tau_s <= 1e-6 else 1.0 - math.exp(-dt / tau_s)
            v += (v_target - v) * max(0.0, min(1.0, alpha))
        articulation = float(row["articulation_rad"]) if finite(row.get("articulation_rad")) else 0.0
        yaw_rate = float(yaw_gain) * v * math.tan(articulation) / wheelbase_m
        if dt > 0.0:
            dtheta = yaw_rate * dt
            heading_mid = yaw + 0.5 * dtheta
            x += v * dt * math.cos(heading_mid)
            y += v * dt * math.sin(heading_mid)
            yaw = wrap_angle(yaw + dtheta)
        row["v_model_ms"] = v
        row["v_target_model_ms"] = v_target
        row["yaw_rate_model_rad_s"] = yaw_rate
        row["model_x"] = x
        row["model_y"] = y
        row["model_yaw"] = yaw
        prev_t = t


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def values(rows: list[dict[str, Any]], key: str, label: str | None = None) -> list[float]:
    out = []
    for row in rows:
        if label is not None and row.get("segment_label") != label:
            continue
        if finite(row.get(key)):
            out.append(float(row[key]))
    return out


def write_plots(rows: list[dict[str, Any]], output_dir: Path, session_name: str) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        print(f"warning: matplotlib unavailable, skipping plots: {exc}", file=sys.stderr)
        return

    output_dir.mkdir(parents=True, exist_ok=True)

    long_rows = [row for row in rows if row.get("segment_label") == "longitudinal"]
    if long_rows:
        plt.figure(figsize=(8, 5))
        plt.scatter(values(long_rows, "throttle_norm"), [abs(v) for v in values(long_rows, "v_icp_ms")], s=8)
        plt.xlabel("throttle norm")
        plt.ylabel("|v_icp| [m/s]")
        plt.title(f"{session_name} throttle vs ICP speed")
        plt.grid(True, alpha=0.4)
        plt.tight_layout()
        plt.savefig(output_dir / "throttle_vs_v_icp.png", dpi=140)
        plt.close()

    yaw_rows = [row for row in rows if row.get("segment_label") == "yaw"]
    if yaw_rows:
        plt.figure(figsize=(8, 5))
        plt.scatter(values(yaw_rows, "yaw_rate_model_rad_s"), values(yaw_rows, "yaw_rate_icp_rad_s"), s=8)
        plt.xlabel("yaw_rate_model [rad/s]")
        plt.ylabel("yaw_rate_icp [rad/s]")
        plt.title(f"{session_name} yaw-rate model vs ICP")
        plt.grid(True, alpha=0.4)
        plt.tight_layout()
        plt.savefig(output_dir / "yaw_rate_model_vs_icp.png", dpi=140)
        plt.close()

    if rows:
        plt.figure(figsize=(9, 7))
        plt.plot(values(rows, "icp_x"), values(rows, "icp_y"), label="icp", linewidth=1.4)
        plt.plot(values(rows, "model_x"), values(rows, "model_y"), label="command model", linewidth=1.4)
        plt.axis("equal")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title(f"{session_name} trajectory model vs ICP")
        plt.grid(True, alpha=0.4)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output_dir / "trajectory_model_vs_icp.png", dpi=140)
        plt.close()

        plt.figure(figsize=(10, 5))
        t0 = float(rows[0]["t"])
        ts = [float(row["t"]) - t0 for row in rows]
        plt.plot(ts, values(rows, "v_icp_ms"), label="v_icp", linewidth=1.0)
        plt.plot(ts, values(rows, "v_model_ms"), label="v_model", linewidth=1.0)
        plt.xlabel("time [s]")
        plt.ylabel("speed [m/s]")
        plt.title(f"{session_name} speed residuals")
        plt.grid(True, alpha=0.4)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output_dir / "residuals_time.png", dpi=140)
        plt.close()


def compact_stats(rows: list[dict[str, Any]]) -> dict[str, Any]:
    labels: dict[str, int] = {}
    for row in rows:
        label = str(row.get("segment_label", "unknown"))
        labels[label] = labels.get(label, 0) + 1
    icp_jumps = values(rows, "icp_pose_step_m")
    return {
        "labels": labels,
        "icp_max_step_m": max(icp_jumps) if icp_jumps else None,
        "icp_mean_step_m": statistics.mean(icp_jumps) if icp_jumps else None,
        "duration_s": (float(rows[-1]["t"]) - float(rows[0]["t"])) if rows else 0.0,
    }


def process_session(session_dir: Path, args: argparse.Namespace) -> dict[str, Any]:
    bag_dir = session_dir / "bag"
    samples, skipped = read_bag_samples(bag_dir, TOPICS)
    icp_rows, icp_source = read_offline_icp(session_dir)
    if not icp_rows:
        icp_rows = samples.get("/mapping/icp_odom", [])
        icp_source = "bag/mapping/icp_odom" if icp_rows else "missing"
    rows = build_rows(
        session_dir,
        samples,
        icp_rows,
        max_linear_speed_ms=args.max_linear_speed_ms,
        max_articulation_rad=args.max_articulation_rad,
        icp_step_threshold_m=args.icp_step_threshold_m,
    )
    fit = fit_models(rows, args.wheelbase_m)
    apply_model(rows, fit, wheelbase_m=args.wheelbase_m)

    out_dir = session_dir / "motion_model_validation"
    write_csv(rows, out_dir / "model_dataset.csv")
    write_plots(rows, out_dir, session_dir.name)
    summary = {
        "session": session_dir.name,
        "icp_source": icp_source,
        "skipped_topics": skipped,
        "stats": compact_stats(rows),
        "fit": fit,
        "outputs": {
            "dataset": str(out_dir / "model_dataset.csv"),
            "summary": str(out_dir / "model_fit_summary.yaml"),
            "plots": [
                str(out_dir / "throttle_vs_v_icp.png"),
                str(out_dir / "yaw_rate_model_vs_icp.png"),
                str(out_dir / "trajectory_model_vs_icp.png"),
                str(out_dir / "residuals_time.png"),
            ],
        },
    }
    (out_dir / "model_fit_summary.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False),
        encoding="utf-8",
    )
    summary["_rows_for_global_fit"] = rows
    return summary


def parse_args() -> argparse.Namespace:
    root = infer_workspace_root(Path(__file__).resolve())
    parser = argparse.ArgumentParser(
        description="Fit simple throttle/speed and articulation/yaw models from bags using ICP."
    )
    parser.add_argument("input_path", nargs="?", default=str(root / "data"))
    parser.add_argument("--max-linear-speed-ms", type=float, default=DEFAULT_MAX_LINEAR_SPEED_MS)
    parser.add_argument("--wheelbase-m", type=float, default=DEFAULT_WHEELBASE_M)
    parser.add_argument("--max-articulation-rad", type=float, default=DEFAULT_MAX_ARTICULATION_RAD)
    parser.add_argument("--icp-step-threshold-m", type=float, default=1.0)
    parser.add_argument(
        "--report-path",
        default=str(root / "data" / "motion_model_fit_report.yaml"),
        help="Global YAML report path.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sessions = resolve_sessions(args.input_path)
    report: list[dict[str, Any]] = []
    global_rows: list[dict[str, Any]] = []
    failures = 0
    for i, session_dir in enumerate(sessions, start=1):
        print(f"[{i}/{len(sessions)}] {session_dir.name}", flush=True)
        try:
            summary = process_session(session_dir, args)
            fit = summary["fit"]
            print(
                "  rows={rows} long={long} yaw={yaw} gain={gain} yaw_gain={yaw_gain}".format(
                    rows=fit["row_count"],
                    long=fit["longitudinal_rows"],
                    yaw=fit["yaw_rows"],
                    gain=fit["throttle_to_abs_speed_gain_zero_ms"],
                    yaw_gain=fit["yaw_gain"],
                ),
                flush=True,
            )
            global_rows.extend(summary.pop("_rows_for_global_fit", []))
            report.append(summary)
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print(f"  FAILED: {exc}", file=sys.stderr, flush=True)
            report.append({"session": session_dir.name, "status": "failed", "error": str(exc)})

    report_path = Path(args.report_path)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    global_fit = fit_models(global_rows, args.wheelbase_m) if global_rows else {}
    report_doc = {
        "global_fit": global_fit,
        "sessions": report,
    }
    report_path.write_text(yaml.safe_dump(report_doc, sort_keys=False), encoding="utf-8")
    print(f"Report: {report_path}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
