#!/usr/bin/env python3
"""Validate cmd_sim motion-model outputs against local ICP odometry."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import sys
from pathlib import Path

import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # pragma: no cover - runtime dependency
    rosbag2_py = None
    deserialize_message = None
    get_message = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


TOPICS = {
    "/cmd_vel",
    "/cmd_vel/teleop",
    "/cmd_vel/teleop_raw",
    "/cmd_vel/manual",
    "/cmd_vel/manual_raw",
    "/controller/cmd_vel",
    "/mtt_status",
    "/mtt_tachometer",
    "/mtt_odometry",
    "/mtt_articulation_angle",
    "/mapping/icp_odom",
}

NUMERIC_ROW_FIELDS = {
    "t",
    "odom_x",
    "odom_y",
    "odom_heading",
    "odom_linear_x",
    "odom_angular_z",
    "icp_x",
    "icp_y",
    "icp_heading",
    "icp_linear_x",
    "icp_angular_z",
    "cmd_linear_x",
    "cmd_angular_z",
    "teleop_linear_x",
    "teleop_angular_z",
    "controller_linear_x",
    "controller_angular_z",
    "articulation_rad",
    "status_speed_ms",
    "status_steer_normalized",
    "status_throttle_raw",
    "status_brake_raw",
    "status_command_linear_speed_ms",
    "status_effective_linear_speed_command_ms",
    "status_hold_assist_output_ms",
    "tach_speed_ms",
    "tach_model_speed_ms",
    "tach_steer_cmd",
    "tach_model_yaw_rate_rad_s",
    "tach_model_articulation_rad",
    "model_x",
    "model_y",
    "model_heading",
}

POSTPROCESS_NUMERIC_FIELDS = {
    "t",
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
    "cmd_linear_x",
    "cmd_angular_z",
    "tach_speed_ms",
    "mtt_articulation_angle",
}


def infer_workspace_root(script_path: Path) -> Path:
    for candidate in [script_path.parent, *script_path.parents]:
        if (candidate / "src").exists() and (candidate / "demos").exists():
            return candidate
    return script_path.parent


def resolve_bag_dir(path_value: str) -> tuple[Path, Path]:
    path = Path(path_value).expanduser().resolve()
    if path.is_file() and path.name.endswith(".mcap"):
        if path.parent.name == "bag":
            return path.parent.parent, path.parent
        return path.parent, path.parent
    if (path / "bag" / "metadata.yaml").exists():
        return path, path / "bag"
    if (path / "metadata.yaml").exists():
        return path.parent, path
    raise SystemExit(f"Could not resolve a bag directory from: {path}")


def resolve_inputs(path_value: str) -> list[tuple[Path, Path]]:
    path = Path(path_value).expanduser().resolve()
    if path.is_dir() and not (path / "metadata.yaml").exists() and not (path / "bag" / "metadata.yaml").exists():
        sessions = []
        for metadata_path in sorted(path.glob("*/bag/metadata.yaml")):
            sessions.append((metadata_path.parent.parent, metadata_path.parent))
        if sessions:
            return sessions
    return [resolve_bag_dir(path_value)]


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(msg) -> float:
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_values_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def extract_sample(topic: str, msg, bag_time_s: float) -> dict[str, float | str | bool]:
    if topic in {
        "/cmd_vel",
        "/cmd_vel/teleop",
        "/cmd_vel/teleop_raw",
        "/cmd_vel/manual",
        "/cmd_vel/manual_raw",
        "/controller/cmd_vel",
    }:
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        twist = msg.twist.twist if hasattr(msg.twist, "twist") else msg.twist
        return {
            "t": t,
            "linear_x": float(twist.linear.x),
            "angular_z": float(twist.angular.z),
        }

    if topic == "/mtt_tachometer":
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        return {
            "t": t,
            "speed_ms": float(msg.speed_ms),
            "model_speed_ms": float(msg.model_speed_ms),
            "direction": str(msg.direction),
            "steer_cmd": float(msg.steer_cmd),
            "model_yaw_rate_rad_s": float(msg.model_yaw_rate_effective_rad_s),
            "model_articulation_rad": float(msg.model_articulation_effective_rad),
            "model_state_valid": bool(msg.model_state_valid),
            "tachometer_is_synthetic": bool(msg.tachometer_is_synthetic),
            "tachometer_source": str(msg.tachometer_source),
        }

    if topic == "/mtt_status":
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        return {
            "t": t,
            "speed_ms": float(msg.speed_ms),
            "steer_normalized": float(msg.steer_normalized),
            "throttle_raw": int(msg.throttle_raw),
            "brake_raw": int(msg.brake_raw),
            "command_linear_speed_ms": float(msg.command_linear_speed_ms),
            "effective_linear_speed_command_ms": float(msg.effective_linear_speed_command_ms),
            "hold_assist_active": bool(msg.hold_assist_active),
            "hold_assist_mode": str(msg.hold_assist_mode),
            "hold_assist_output_ms": float(msg.hold_assist_output_ms),
            "command_timeout_active": bool(msg.command_timeout_active),
            "tachometer_is_synthetic": bool(msg.tachometer_is_synthetic),
            "tachometer_source": str(msg.tachometer_source),
        }

    if topic in {"/mtt_odometry", "/mapping/icp_odom"}:
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        return {
            "t": t,
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "heading": quaternion_to_yaw(msg),
            "linear_x": float(msg.twist.twist.linear.x),
            "angular_z": float(msg.twist.twist.angular.z),
        }

    if topic == "/mtt_articulation_angle":
        return {"t": bag_time_s, "articulation_rad": float(msg.data)}

    raise ValueError(f"Unsupported topic: {topic}")


def read_topic_samples(bag_dir: Path) -> dict[str, list[dict[str, float | str | bool]]]:
    if IMPORT_ERROR is not None:
        raise SystemExit(f"rosbag2_py is not available: {IMPORT_ERROR}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics_and_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topics_and_types}
    selected_topics = sorted(TOPICS.intersection(type_map))
    if not selected_topics:
        raise SystemExit(f"No validation topics found in {bag_dir}")

    reader.set_filter(rosbag2_py.StorageFilter(topics=selected_topics))
    msg_types = {topic: get_message(type_map[topic]) for topic in selected_topics}
    samples = {topic: [] for topic in selected_topics}
    skipped_topics: set[str] = set()

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic in skipped_topics:
            continue
        try:
            msg = deserialize_message(data, msg_types[topic])
            samples[topic].append(extract_sample(topic, msg, timestamp_ns / 1e9))
        except (Exception, SystemExit) as exc:
            print(
                f"warning: skipping {topic} due to message/schema mismatch: {exc}",
                file=sys.stderr,
            )
            skipped_topics.add(topic)
            samples.pop(topic, None)
            continue

    return samples


def derive_icp_kinematics(
    samples: list[dict[str, float | str | bool]],
    smooth_window_s: float = 1.0,
) -> list[dict[str, float]]:
    """Compute ICP velocities using a centred sliding window to reduce per-frame noise.

    Naive frame-to-frame finite differences amplify ICP registration noise (positions
    may jump ±0.5 m between consecutive frames on short sessions).  A 1-second window
    averages over several registrations and gives physically plausible velocity estimates.
    Falls back to the mapper's own twist velocity when fewer than 2 samples exist in the
    window.
    """
    if not samples:
        return []

    ts = [float(s["t"]) for s in samples]
    xs = [float(s["x"]) for s in samples]
    ys = [float(s["y"]) for s in samples]
    hs = [float(s["heading"]) for s in samples]
    half = smooth_window_s / 2.0

    derived: list[dict[str, float]] = []
    for i, sample in enumerate(samples):
        t = ts[i]
        j0 = bisect.bisect_left(ts, t - half)
        j1 = bisect.bisect_right(ts, t + half) - 1
        dt_win = ts[j1] - ts[j0]
        if j1 > j0 and dt_win > 1e-6:
            dx = xs[j1] - xs[j0]
            dy = ys[j1] - ys[j0]
            heading_mid = wrap_angle(0.5 * (hs[j1] + hs[j0]))
            ds = dx * math.cos(heading_mid) + dy * math.sin(heading_mid)
            linear_x = ds / dt_win
            angular_z = wrap_angle(hs[j1] - hs[j0]) / dt_win
        else:
            # window too narrow — fall back to mapper's own velocity
            linear_x = float(sample["linear_x"])
            angular_z = float(sample["angular_z"])
        derived.append({
            "t": t,
            "x": xs[i],
            "y": ys[i],
            "heading": hs[i],
            "linear_x": linear_x,
            "angular_z": angular_z,
        })
    return derived


def derive_pose_kinematics(samples: list[dict[str, float | str | bool]]) -> list[dict[str, float]]:
    if not samples:
        return []

    derived: list[dict[str, float]] = []
    previous = None
    for sample in samples:
        row = {
            "t": float(sample["t"]),
            "x": float(sample["x"]),
            "y": float(sample["y"]),
            "heading": float(sample["heading"]),
            "linear_x": 0.0,
            "angular_z": 0.0,
        }
        if previous is not None:
            dt = row["t"] - previous["t"]
            if dt > 1e-6:
                dx = row["x"] - previous["x"]
                dy = row["y"] - previous["y"]
                heading_mid = wrap_angle(0.5 * (row["heading"] + previous["heading"]))
                row["linear_x"] = (dx * math.cos(heading_mid) + dy * math.sin(heading_mid)) / dt
                row["angular_z"] = wrap_angle(row["heading"] - previous["heading"]) / dt
        derived.append(row)
        previous = row
    return derived


class TimeSeries:
    def __init__(self, samples: list[dict[str, float | str | bool]]):
        self.samples = sorted(samples, key=lambda sample: float(sample["t"]))
        self.times = [float(sample["t"]) for sample in self.samples]

    def nearest(self, timestamp_s: float, tolerance_s: float) -> dict[str, float | str | bool] | None:
        if not self.samples:
            return None
        index = bisect.bisect_left(self.times, timestamp_s)
        candidates: list[dict[str, float | str | bool]] = []
        if index < len(self.samples):
            candidates.append(self.samples[index])
        if index > 0:
            candidates.append(self.samples[index - 1])
        if not candidates:
            return None
        best = min(candidates, key=lambda sample: abs(float(sample["t"]) - timestamp_s))
        if abs(float(best["t"]) - timestamp_s) > tolerance_s:
            return None
        return best


def detect_speed_sign(rows: list[dict[str, float | str | bool | None]]) -> float:
    """Return the speed_sign (±1.0) that makes the model trajectory align with ICP.

    Strategy: compare the overall displacement vectors of the ICP trajectory and the
    model trajectory (first→last point with valid data).  If they point in opposite
    directions (dot product < −0.5) and both moved at least 1 m, the model speed sign
    is inverted relative to physical motion → return −1.0.  Otherwise return +1.0.

    This auto-corrects the invert_linear_axis=true convention in mtt_operator_input_node
    which causes cmd_vel.linear.x (and model_speed_ms in cmd-sim mode) to be negative
    for physical forward motion.
    """
    # Collect first/last ICP and model positions
    icp_first = icp_last = None
    model_first = model_last = None
    for row in rows:
        ix, iy = row.get("icp_x"), row.get("icp_y")
        mx, my = row.get("model_x"), row.get("model_y")
        if ix is not None and iy is not None:
            if icp_first is None:
                icp_first = (float(ix), float(iy))
            icp_last = (float(ix), float(iy))
        if mx is not None and my is not None:
            if model_first is None:
                model_first = (float(mx), float(my))
            model_last = (float(mx), float(my))

    if icp_first is None or icp_last is None or model_first is None or model_last is None:
        return 1.0

    icp_dx = icp_last[0] - icp_first[0]
    icp_dy = icp_last[1] - icp_first[1]
    icp_dist = math.hypot(icp_dx, icp_dy)

    model_dx = model_last[0] - model_first[0]
    model_dy = model_last[1] - model_first[1]
    model_dist = math.hypot(model_dx, model_dy)

    # Need at least 1 m of movement in both to be confident
    if icp_dist < 1.0 or model_dist < 1.0:
        return 1.0

    # Normalised dot product of displacement directions
    dot = (icp_dx / icp_dist) * (model_dx / model_dist) + (icp_dy / icp_dist) * (model_dy / model_dist)
    return -1.0 if dot < -0.5 else 1.0


def rmse(values_a: list[float], values_b: list[float]) -> float | None:
    if not values_a or len(values_a) != len(values_b):
        return None
    squared_error = [(a - b) ** 2 for a, b in zip(values_a, values_b)]
    return math.sqrt(sum(squared_error) / len(squared_error))


def integrate_model_trajectory(
    rows: list[dict[str, float | str | bool | None]],
    speed_sign: float = 1.0,
) -> None:
    """Integrate kinematic model trajectory from tachometer speed and yaw-rate.

    speed_sign controls the convention correction:
      +1.0 (default): tach_model_speed_ms positive = physical forward.
      -1.0: tach_model_speed_ms negative = physical forward.
           Use when invert_linear_axis=true is in effect on the operator-input node,
           which causes cmd_vel.linear.x (and thus model_speed_ms in cmd-sim mode) to
           be negative for forward motion.  Both speed and yaw-rate are negated so the
           full 2-D trajectory is correctly mirrored.
    """
    model_x = 0.0
    model_y = 0.0
    model_heading = 0.0
    previous_t = None

    for row in rows:
        t = float(row["t"])
        if previous_t is None:
            dt = 0.0
        else:
            dt = max(0.0, min(t - previous_t, 1.0))

        speed = row.get("tach_model_speed_ms")
        yaw_rate = row.get("tach_model_yaw_rate_rad_s")
        if speed is not None and yaw_rate is not None and dt > 0.0:
            dtheta = float(yaw_rate) * speed_sign * dt
            ds    = float(speed)    * speed_sign * dt
            heading_mid = model_heading + 0.5 * dtheta
            model_x += ds * math.cos(heading_mid)
            model_y += ds * math.sin(heading_mid)
            model_heading = wrap_angle(model_heading + dtheta)

        row["model_x"] = model_x if speed is not None and yaw_rate is not None else None
        row["model_y"] = model_y if speed is not None and yaw_rate is not None else None
        row["model_heading"] = model_heading if speed is not None and yaw_rate is not None else None
        previous_t = t


def build_rows(samples: dict[str, list[dict[str, float | str | bool]]]) -> list[dict[str, float | str | bool | None]]:
    odom_samples = samples.get("/mtt_odometry", [])
    icp_series = TimeSeries(derive_icp_kinematics(samples.get("/mapping/icp_odom", [])))
    tacho_series = TimeSeries(samples.get("/mtt_tachometer", []))
    cmd_series = TimeSeries(samples.get("/cmd_vel", []))
    teleop_series = TimeSeries(samples.get("/cmd_vel/teleop", []))
    controller_series = TimeSeries(samples.get("/controller/cmd_vel", []))
    articulation_series = TimeSeries(samples.get("/mtt_articulation_angle", []))
    status_series = TimeSeries(samples.get("/mtt_status", []))
    reference_samples = odom_samples or samples.get("/mtt_tachometer", [])
    if not reference_samples:
        raise SystemExit("Neither /mtt_odometry nor /mtt_tachometer is available for validation.")

    rows: list[dict[str, float | str | bool | None]] = []
    for reference_sample in reference_samples:
        t = float(reference_sample["t"])
        tacho_sample = tacho_series.nearest(t, 0.1)
        cmd_sample = cmd_series.nearest(t, 0.1)
        teleop_sample = teleop_series.nearest(t, 0.1)
        controller_sample = controller_series.nearest(t, 0.1)
        icp_sample = icp_series.nearest(t, 0.2)
        articulation_sample = articulation_series.nearest(t, 0.1)
        status_sample = status_series.nearest(t, 0.1)
        odom_sample = reference_sample if odom_samples else None

        row: dict[str, float | str | bool | None] = {
            "t": t,
            "odom_x": float(odom_sample["x"]) if odom_sample else None,
            "odom_y": float(odom_sample["y"]) if odom_sample else None,
            "odom_heading": float(odom_sample["heading"]) if odom_sample else None,
            "odom_linear_x": float(odom_sample["linear_x"]) if odom_sample else None,
            "odom_angular_z": float(odom_sample["angular_z"]) if odom_sample else None,
            "icp_x": float(icp_sample["x"]) if icp_sample else None,
            "icp_y": float(icp_sample["y"]) if icp_sample else None,
            "icp_heading": float(icp_sample["heading"]) if icp_sample else None,
            "icp_linear_x": float(icp_sample["linear_x"]) if icp_sample else None,
            "icp_angular_z": float(icp_sample["angular_z"]) if icp_sample else None,
            "cmd_linear_x": float(cmd_sample["linear_x"]) if cmd_sample else None,
            "cmd_angular_z": float(cmd_sample["angular_z"]) if cmd_sample else None,
            "teleop_linear_x": float(teleop_sample["linear_x"]) if teleop_sample else None,
            "teleop_angular_z": float(teleop_sample["angular_z"]) if teleop_sample else None,
            "controller_linear_x": float(controller_sample["linear_x"]) if controller_sample else None,
            "controller_angular_z": float(controller_sample["angular_z"]) if controller_sample else None,
            "articulation_rad": float(articulation_sample["articulation_rad"]) if articulation_sample else None,
            "status_speed_ms": float(status_sample["speed_ms"]) if status_sample else None,
            "status_steer_normalized": float(status_sample["steer_normalized"]) if status_sample else None,
            "status_throttle_raw": int(status_sample["throttle_raw"]) if status_sample else None,
            "status_brake_raw": int(status_sample["brake_raw"]) if status_sample else None,
            "status_command_linear_speed_ms": (
                float(status_sample["command_linear_speed_ms"]) if status_sample else None
            ),
            "status_effective_linear_speed_command_ms": (
                float(status_sample["effective_linear_speed_command_ms"]) if status_sample else None
            ),
            "status_hold_assist_active": bool(status_sample["hold_assist_active"]) if status_sample else None,
            "status_hold_assist_mode": str(status_sample["hold_assist_mode"]) if status_sample else None,
            "status_hold_assist_output_ms": (
                float(status_sample["hold_assist_output_ms"]) if status_sample else None
            ),
            "status_command_timeout_active": (
                bool(status_sample["command_timeout_active"]) if status_sample else None
            ),
            "tach_speed_ms": float(tacho_sample["speed_ms"]) if tacho_sample else None,
            "tach_model_speed_ms": float(tacho_sample["model_speed_ms"]) if tacho_sample else None,
            "tach_direction": str(tacho_sample["direction"]) if tacho_sample else None,
            "tach_steer_cmd": float(tacho_sample["steer_cmd"]) if tacho_sample else None,
            "tach_model_yaw_rate_rad_s": float(tacho_sample["model_yaw_rate_rad_s"]) if tacho_sample else None,
            "tach_model_articulation_rad": float(tacho_sample["model_articulation_rad"]) if tacho_sample else None,
            "tach_model_state_valid": bool(tacho_sample["model_state_valid"]) if tacho_sample else None,
            "tachometer_is_synthetic": bool(tacho_sample["tachometer_is_synthetic"]) if tacho_sample else None,
            "tachometer_source": str(tacho_sample["tachometer_source"]) if tacho_sample else None,
        }
        rows.append(row)

    integrate_model_trajectory(rows)
    return rows


def _csv_float(value: str | None) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _csv_bool(value: str | None) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def build_rows_from_postprocess_csv(csv_path: Path) -> list[dict[str, float | str | bool | None]]:
    raw_rows: list[dict[str, str]] = []
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        raw_rows = [dict(row) for row in reader]
    if not raw_rows:
        return []

    icp_samples: list[dict[str, float | str | bool]] = []
    odom_samples: list[dict[str, float | str | bool]] = []
    for row in raw_rows:
        t = _csv_float(row.get("t"))
        if t is None:
            continue
        if _csv_bool(row.get("has_icp")):
            icp_x = _csv_float(row.get("icp_x"))
            icp_y = _csv_float(row.get("icp_y"))
            qx = _csv_float(row.get("icp_qx"))
            qy = _csv_float(row.get("icp_qy"))
            qz = _csv_float(row.get("icp_qz"))
            qw = _csv_float(row.get("icp_qw"))
            if None not in (icp_x, icp_y, qx, qy, qz, qw):
                assert icp_x is not None and icp_y is not None and qx is not None and qy is not None and qz is not None and qw is not None
                icp_samples.append(
                    {
                        "t": t,
                        "x": icp_x,
                        "y": icp_y,
                        "heading": quaternion_values_to_yaw(qx, qy, qz, qw),
                        "linear_x": 0.0,
                        "angular_z": 0.0,
                    }
                )
        if _csv_bool(row.get("has_odom")):
            odom_x = _csv_float(row.get("odom_x"))
            odom_y = _csv_float(row.get("odom_y"))
            odom_yaw = _csv_float(row.get("odom_yaw"))
            if None not in (odom_x, odom_y, odom_yaw):
                assert odom_x is not None and odom_y is not None and odom_yaw is not None
                odom_samples.append({"t": t, "x": odom_x, "y": odom_y, "heading": odom_yaw})

    icp_series = TimeSeries(derive_pose_kinematics(icp_samples))
    odom_series = TimeSeries(derive_pose_kinematics(odom_samples))

    rows: list[dict[str, float | str | bool | None]] = []
    for source in raw_rows:
        t = _csv_float(source.get("t"))
        if t is None:
            continue
        icp_sample = icp_series.nearest(t, 0.2)
        odom_sample = odom_series.nearest(t, 0.1)
        tach_source = str(source.get("tach_source") or "")
        tach_synthetic = _csv_bool(source.get("tach_is_synthetic"))
        row: dict[str, float | str | bool | None] = {
            "t": t,
            "odom_x": float(odom_sample["x"]) if odom_sample else None,
            "odom_y": float(odom_sample["y"]) if odom_sample else None,
            "odom_heading": float(odom_sample["heading"]) if odom_sample else None,
            "odom_linear_x": float(odom_sample["linear_x"]) if odom_sample else None,
            "odom_angular_z": float(odom_sample["angular_z"]) if odom_sample else None,
            "icp_x": float(icp_sample["x"]) if icp_sample else None,
            "icp_y": float(icp_sample["y"]) if icp_sample else None,
            "icp_heading": float(icp_sample["heading"]) if icp_sample else None,
            "icp_linear_x": float(icp_sample["linear_x"]) if icp_sample else None,
            "icp_angular_z": float(icp_sample["angular_z"]) if icp_sample else None,
            "cmd_linear_x": _csv_float(source.get("cmd_linear_x")),
            "cmd_angular_z": _csv_float(source.get("cmd_angular_z")),
            "teleop_linear_x": None,
            "teleop_angular_z": None,
            "controller_linear_x": None,
            "controller_angular_z": None,
            "articulation_rad": _csv_float(source.get("mtt_articulation_angle")),
            "status_speed_ms": None,
            "status_steer_normalized": None,
            "status_throttle_raw": None,
            "status_brake_raw": None,
            "status_command_linear_speed_ms": None,
            "status_effective_linear_speed_command_ms": None,
            "status_hold_assist_active": None,
            "status_hold_assist_mode": None,
            "status_hold_assist_output_ms": None,
            "status_command_timeout_active": None,
            "tach_speed_ms": _csv_float(source.get("tach_speed_ms")),
            "tach_model_speed_ms": None,
            "tach_direction": str(source.get("tach_direction") or "") or None,
            "tach_steer_cmd": None,
            "tach_model_yaw_rate_rad_s": None,
            "tach_model_articulation_rad": None,
            "tach_model_state_valid": None,
            "tachometer_is_synthetic": tach_synthetic,
            "tachometer_source": tach_source or None,
        }
        rows.append(row)

    integrate_model_trajectory(rows)
    return rows


def finite_xy(rows: list[dict[str, float | str | bool | None]], x_key: str, y_key: str) -> tuple[list[float], list[float]]:
    xs = []
    ys = []
    for row in rows:
        x = row.get(x_key)
        y = row.get(y_key)
        if x is None or y is None:
            continue
        xf = float(x)
        yf = float(y)
        if math.isfinite(xf) and math.isfinite(yf):
            xs.append(xf)
            ys.append(yf)
    return xs, ys


def write_xy_plot(rows: list[dict[str, float | str | bool | None]], plot_path: Path, title: str) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        print(f"warning: matplotlib is not available, skipping plot: {exc}", file=sys.stderr)
        return False

    series = [
        ("icp", "icp_x", "icp_y"),
        ("motion_model", "model_x", "model_y"),
    ]

    plotted = False
    _, ax = plt.subplots(figsize=(10, 8))

    # Collect unique ICP positions (avoid plotting the 50 Hz resampled repeats)
    icp_unique_x: list[float] = []
    icp_unique_y: list[float] = []
    _prev_icp: tuple[float, float] | None = None
    for row in rows:
        if row.get("icp_x") is not None and row.get("icp_y") is not None:
            cur = (float(row["icp_x"]), float(row["icp_y"]))  # type: ignore[arg-type]
            if cur != _prev_icp:
                icp_unique_x.append(cur[0])
                icp_unique_y.append(cur[1])
                _prev_icp = cur

    # Compute ICP jumps for diagnostics only. The plot itself stays neutral:
    # motion-model validation should not label a trajectory as poor from a
    # single local jump threshold when the offline ICP coverage/scale is good.
    icp_jumps: list[float] = []
    for k in range(1, len(icp_unique_x)):
        icp_jumps.append(math.hypot(icp_unique_x[k] - icp_unique_x[k - 1], icp_unique_y[k] - icp_unique_y[k - 1]))
    icp_max_jump = max(icp_jumps) if icp_jumps else 0.0

    series_colors = {"icp": "tab:orange", "motion_model": "tab:green"}

    for label, x_key, y_key in series:
        color = series_colors[label]
        if label == "icp":
            # Use unique ICP positions for a cleaner trajectory line
            if not icp_unique_x:
                continue
            ax.plot(icp_unique_x, icp_unique_y, label="icp", linewidth=1.5, color=color)
            # Direction arrow near the midpoint of the ICP trajectory
            mid = len(icp_unique_x) // 2
            if mid + 1 < len(icp_unique_x):
                ax.annotate("", xy=(icp_unique_x[mid + 1], icp_unique_y[mid + 1]),
                            xytext=(icp_unique_x[mid], icp_unique_y[mid]),
                            arrowprops=dict(arrowstyle="->", color=color, lw=1.5))
        else:
            xs, ys = finite_xy(rows, x_key, y_key)
            if not xs:
                continue
            ax.plot(xs, ys, label=label, linewidth=1.5, color=color)
            # Direction arrow near midpoint
            mid = len(xs) // 2
            if mid + 1 < len(xs):
                ax.annotate("", xy=(xs[mid + 1], ys[mid + 1]),
                            xytext=(xs[mid], ys[mid]),
                            arrowprops=dict(arrowstyle="->", color=color, lw=1.5))
        plotted = True

    if not plotted:
        plt.close()
        return False

    # Mark start point shared by all trajectories
    ax.plot(0, 0, "ko", markersize=6, zorder=5, label="start (0,0)")

    diagnostics_str = f"ICP max step: {icp_max_jump:.3f} m"
    ax.set_title(f"{title}\n{diagnostics_str}", fontsize=10)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, linewidth=0.4, alpha=0.5)
    ax.legend()
    plt.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(plot_path, dpi=140)
    plt.close()
    return True


def compute_summary(rows: list[dict[str, float | str | bool | None]], session_dir: Path) -> dict[str, object]:
    speed_pred: list[float] = []
    speed_ref: list[float] = []
    yaw_pred: list[float] = []
    yaw_ref: list[float] = []
    sign_total = 0
    sign_matches = 0
    synthetic_rows = 0
    hold_assist_rows = 0
    max_hold_assist_output_ms = 0.0

    for row in rows:
        if row["tachometer_is_synthetic"]:
            synthetic_rows += 1
        if row["status_hold_assist_active"]:
            hold_assist_rows += 1
        if row["status_hold_assist_output_ms"] is not None:
            max_hold_assist_output_ms = max(
                max_hold_assist_output_ms,
                abs(float(row["status_hold_assist_output_ms"])),
            )
        if row["icp_linear_x"] is not None and row["odom_linear_x"] is not None:
            speed_pred.append(float(row["odom_linear_x"]))
            speed_ref.append(float(row["icp_linear_x"]))
        elif row["icp_linear_x"] is not None and row["tach_model_speed_ms"] is not None:
            speed_pred.append(float(row["tach_model_speed_ms"]))
            speed_ref.append(float(row["icp_linear_x"]))
        if row["icp_angular_z"] is not None and row["odom_angular_z"] is not None:
            yaw_pred.append(float(row["odom_angular_z"]))
            yaw_ref.append(float(row["icp_angular_z"]))
        elif row["icp_angular_z"] is not None and row["tach_model_yaw_rate_rad_s"] is not None:
            yaw_pred.append(float(row["tach_model_yaw_rate_rad_s"]))
            yaw_ref.append(float(row["icp_angular_z"]))
        tach_or_model_speed = row["tach_model_speed_ms"]
        if tach_or_model_speed is None:
            tach_or_model_speed = row["tach_speed_ms"]
        if tach_or_model_speed is not None and row["icp_linear_x"] is not None:
            model_speed = float(tach_or_model_speed)
            icp_speed = float(row["icp_linear_x"])
            if abs(model_speed) > 0.05 and abs(icp_speed) > 0.05:
                sign_total += 1
                if math.copysign(1.0, model_speed) == math.copysign(1.0, icp_speed):
                    sign_matches += 1

    final_pose_error_m = None
    if rows and rows[-1]["odom_x"] is not None and rows[-1]["odom_y"] is not None and rows[-1]["icp_x"] is not None and rows[-1]["icp_y"] is not None:
        dx = float(rows[-1]["odom_x"]) - float(rows[-1]["icp_x"])
        dy = float(rows[-1]["odom_y"]) - float(rows[-1]["icp_y"])
        final_pose_error_m = math.hypot(dx, dy)

    # ICP diagnostic: measure pose jumps between consecutive unique ICP positions.
    # This is a warning-level metric only; a single step above 0.5 m can happen on
    # fast replayed trajectories without making the full ICP trajectory unusable.
    prev_icp_xy: tuple[float, float] | None = None
    icp_jumps: list[float] = []
    for row in rows:
        ix = row.get("icp_x")
        iy = row.get("icp_y")
        if ix is None or iy is None:
            continue
        cur = (float(ix), float(iy))
        if prev_icp_xy is not None and cur != prev_icp_xy:
            icp_jumps.append(math.hypot(cur[0] - prev_icp_xy[0], cur[1] - prev_icp_xy[1]))
        if cur != prev_icp_xy:
            prev_icp_xy = cur
    icp_max_jump = max(icp_jumps) if icp_jumps else None
    icp_mean_jump = (sum(icp_jumps) / len(icp_jumps)) if icp_jumps else None
    # Keep the historical field but use a practical offline threshold. The detailed
    # offline_icp summary remains the source of truth for coverage and mapper health.
    icp_quality_ok = (icp_max_jump is not None and icp_max_jump < 1.0)

    start_t = float(rows[0]["t"]) if rows else 0.0
    end_t = float(rows[-1]["t"]) if rows else 0.0
    return {
        "session": session_dir.name,
        "duration_s": max(0.0, end_t - start_t),
        "row_count": len(rows),
        "synthetic_ratio": (synthetic_rows / len(rows)) if rows else 0.0,
        "hold_assist_ratio": (hold_assist_rows / len(rows)) if rows else 0.0,
        "max_hold_assist_output_ms": max_hold_assist_output_ms if rows else None,
        "speed_rmse_ms": rmse(speed_pred, speed_ref),
        "yaw_rate_rmse_rad_s": rmse(yaw_pred, yaw_ref),
        "sign_agreement": (sign_matches / sign_total) if sign_total else None,
        "sign_checks": sign_total,
        "final_pose_error_m": final_pose_error_m,
        "icp_max_pose_jump_m": round(icp_max_jump, 4) if icp_max_jump is not None else None,
        "icp_mean_pose_jump_m": round(icp_mean_jump, 4) if icp_mean_jump is not None else None,
        "icp_quality_ok": icp_quality_ok,
    }


def write_csv(rows: list[dict[str, float | str | bool | None]], csv_path: Path) -> None:
    fieldnames = list(rows[0].keys()) if rows else []
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def read_csv(csv_path: Path) -> list[dict[str, float | str | bool | None]]:
    rows: list[dict[str, float | str | bool | None]] = []
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        for csv_row in reader:
            row: dict[str, float | str | bool | None] = {}
            for key, value in csv_row.items():
                if value == "":
                    row[key] = None
                elif key in NUMERIC_ROW_FIELDS:
                    row[key] = float(value)
                elif value == "True":
                    row[key] = True
                elif value == "False":
                    row[key] = False
                else:
                    row[key] = value
            rows.append(row)
    if rows and ("model_x" not in rows[0] or "model_y" not in rows[0]):
        integrate_model_trajectory(rows)
    return rows


def parse_vtk_trajectory(
    vtk_path: Path, duration_s: float, start_time: float
) -> list[dict[str, float | str | bool]]:
    """Parse offline ICP trajectory.vtk into icp_odom-compatible samples.

    Timestamps are linearly interpolated across *duration_s* starting at
    *start_time*.  Heading is derived from forward-difference of positions so
    that derive_icp_kinematics can compute velocities correctly.
    """
    if not vtk_path.exists() or vtk_path.stat().st_size < 200:
        return []
    try:
        lines = vtk_path.read_text(encoding="utf-8", errors="ignore").splitlines()
    except OSError:
        return []
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
    if data_start is None or point_count <= 1:
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
    pts = [(values[i * 3], values[i * 3 + 1]) for i in range(point_count)]
    # Heading from forward differences (last point reuses previous heading)
    headings: list[float] = []
    for i in range(len(pts) - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        headings.append(math.atan2(dy, dx) if (dx * dx + dy * dy) > 1e-10 else 0.0)
    headings.append(headings[-1] if headings else 0.0)
    denom = max(1, point_count - 1)
    samples: list[dict[str, float | str | bool]] = []
    for i, (x, y) in enumerate(pts):
        samples.append({
            "t": start_time + (i / denom) * duration_s,
            "x": x,
            "y": y,
            "heading": headings[i],
            "linear_x": 0.0,
            "angular_z": 0.0,
        })
    return samples


def parse_args(workspace_root: Path) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare cmd_sim motion-model outputs against local ICP odometry."
    )
    parser.add_argument(
        "input_path",
        nargs="?",
        default=str(workspace_root / "data"),
        help="Session directory, bag directory, or bag_0.mcap path.",
    )
    parser.add_argument(
        "--from-postprocess-csv",
        action="store_true",
        help="Read postprocess_dataset/dataset.csv instead of decoding the original bag.",
    )
    parser.add_argument(
        "--use-offline-icp",
        action="store_true",
        help=(
            "Replace /mapping/icp_odom from the bag with the offline ICP trajectory "
            "(offline_icp/trajectory.vtk). Gives higher-quality ground truth at the "
            "cost of linearly-interpolated timestamps."
        ),
    )
    return parser.parse_args()


def main() -> int:
    workspace_root = infer_workspace_root(Path(__file__).resolve())
    args = parse_args(workspace_root)
    inputs = resolve_inputs(args.input_path)
    failures = 0

    for index, (session_dir, bag_dir) in enumerate(inputs, start=1):
        if len(inputs) > 1:
            print(f"\n[{index}/{len(inputs)}] {session_dir.name}")

        output_dir = session_dir / "motion_model_validation"
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / "aligned_samples.csv"
        summary_path = output_dir / "summary.yaml"
        plot_path = output_dir / "trajectory_xy.png"

        try:
            if args.from_postprocess_csv:
                source_csv = session_dir / "postprocess_dataset" / "dataset.csv"
                if not source_csv.exists():
                    raise FileNotFoundError(source_csv)
                rows = build_rows_from_postprocess_csv(source_csv)
                if not rows:
                    raise RuntimeError(f"no rows in {source_csv}")
            else:
                samples = read_topic_samples(bag_dir)
                if args.use_offline_icp:
                    # Priority 1: icp_odom_replay bag — real ROS timestamps + SE(3) quaternion,
                    #   produced by ros2 bag record during the offline ICP run.
                    # Priority 2: trajectory.vtk fallback — linearly interpolated timestamps,
                    #   position-only (no rotation). Use only when replay bag is absent.
                    icp_source = None
                    replay_bag = session_dir / "offline_icp" / "icp_odom_replay"
                    if (replay_bag / "metadata.yaml").exists():
                        try:
                            replay_samples = read_topic_samples(replay_bag)
                            msgs = replay_samples.get("/mapping/icp_odom", [])
                            if msgs:
                                samples["/mapping/icp_odom"] = msgs
                                icp_source = "offline_icp/icp_odom_replay"
                                print(f"  offline ICP: icp_odom_replay bag → {len(msgs)} poses (real timestamps)")
                        except Exception as exc:
                            print(f"  warning: could not read icp_odom_replay: {exc}", file=sys.stderr)
                    if icp_source is None:
                        # Fallback: VTK with linear timestamp interpolation
                        all_times = [
                            float(s["t"])
                            for topic_samples in samples.values()
                            for s in topic_samples
                        ]
                        if all_times:
                            t_start = min(all_times)
                            duration_s = max(all_times) - t_start
                            vtk_path = session_dir / "offline_icp" / "trajectory.vtk"
                            vtk_samples = parse_vtk_trajectory(vtk_path, duration_s, t_start)
                            if vtk_samples:
                                samples["/mapping/icp_odom"] = vtk_samples
                                icp_source = "offline_icp/trajectory.vtk (interpolated)"
                                print(f"  offline ICP: trajectory.vtk fallback → {len(vtk_samples)} poses (interpolated timestamps)")
                            else:
                                print("  warning: no offline ICP source found, using live bag icp_odom", file=sys.stderr)
                rows = build_rows(samples)
            # ── Auto-correct speed sign convention ────────────────────────────
            # mtt_operator_input_node uses invert_linear_axis=true by default,
            # which makes cmd_vel.linear.x (and model_speed_ms in cmd-sim mode)
            # negative for physical forward motion.  Detect by comparing the
            # overall ICP and model displacement directions.
            speed_sign = detect_speed_sign(rows)
            if speed_sign != 1.0:
                integrate_model_trajectory(rows, speed_sign=speed_sign)
            summary = compute_summary(rows, session_dir)
            summary["speed_sign_corrected"] = (speed_sign == -1.0)
            if args.from_postprocess_csv:
                summary["input_source"] = "postprocess_dataset/dataset.csv"
                summary["motion_model_fields_available"] = False
            if getattr(args, "use_offline_icp", False) and not args.from_postprocess_csv:
                summary["icp_source"] = icp_source or "bag_live"
            write_csv(rows, csv_path)
            summary_path.write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
        except Exception as exc:
            if csv_path.exists():
                print(f"warning: bag read failed, plotting existing CSV for {session_dir.name}: {exc}", file=sys.stderr)
                rows = read_csv(csv_path)
                summary = compute_summary(rows, session_dir)
            else:
                failures += 1
                print(f"ERROR: failed {session_dir}: {exc}", file=sys.stderr)
                continue

        plot_written = write_xy_plot(rows, plot_path, session_dir.name)

        print(f"Session:      {session_dir}")
        print(f"Bag:          {bag_dir}")
        print(f"Aligned CSV:  {csv_path}")
        print(f"Summary YAML: {summary_path}")
        print(f"XY Plot:      {plot_path if plot_written else 'not written'}")
        print(yaml.safe_dump(summary, sort_keys=False).strip())

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
