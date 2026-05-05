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
    "/controller/cmd_vel",
    "/mtt_status",
    "/mtt_tachometer",
    "/mtt_odometry",
    "/mtt_articulation_angle",
    "/mapping/icp_odom",
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


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(msg) -> float:
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def extract_sample(topic: str, msg, bag_time_s: float) -> dict[str, float | str | bool]:
    if topic in {"/cmd_vel", "/cmd_vel/teleop", "/controller/cmd_vel"}:
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        return {
            "t": t,
            "linear_x": float(msg.twist.linear.x),
            "angular_z": float(msg.twist.angular.z),
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
        except Exception as exc:
            if topic == "/mtt_status":
                print(
                    f"warning: skipping {topic} due to message/schema mismatch: {exc}",
                    file=sys.stderr,
                )
                skipped_topics.add(topic)
                samples.pop(topic, None)
                continue
            raise

    return samples


def derive_icp_kinematics(samples: list[dict[str, float | str | bool]]) -> list[dict[str, float]]:
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
            "linear_x": float(sample["linear_x"]),
            "angular_z": float(sample["angular_z"]),
        }
        if previous is None:
            row["linear_x"] = 0.0
            row["angular_z"] = 0.0
        else:
            dt = row["t"] - previous["t"]
            if dt > 1e-6:
                dx = row["x"] - previous["x"]
                dy = row["y"] - previous["y"]
                heading_mid = wrap_angle(0.5 * (row["heading"] + previous["heading"]))
                ds_signed = dx * math.cos(heading_mid) + dy * math.sin(heading_mid)
                row["linear_x"] = ds_signed / dt
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


def rmse(values_a: list[float], values_b: list[float]) -> float | None:
    if not values_a or len(values_a) != len(values_b):
        return None
    squared_error = [(a - b) ** 2 for a, b in zip(values_a, values_b)]
    return math.sqrt(sum(squared_error) / len(squared_error))


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

    return rows


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
        if row["tach_model_speed_ms"] is not None and row["icp_linear_x"] is not None:
            model_speed = float(row["tach_model_speed_ms"])
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
    }


def write_csv(rows: list[dict[str, float | str | bool | None]], csv_path: Path) -> None:
    fieldnames = list(rows[0].keys()) if rows else []
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


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
    return parser.parse_args()


def main() -> int:
    workspace_root = infer_workspace_root(Path(__file__).resolve())
    args = parse_args(workspace_root)
    session_dir, bag_dir = resolve_bag_dir(args.input_path)
    samples = read_topic_samples(bag_dir)
    rows = build_rows(samples)
    summary = compute_summary(rows, session_dir)

    output_dir = session_dir / "motion_model_validation"
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "aligned_samples.csv"
    summary_path = output_dir / "summary.yaml"
    write_csv(rows, csv_path)
    summary_path.write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")

    print(f"Session:      {session_dir}")
    print(f"Bag:          {bag_dir}")
    print(f"Aligned CSV:  {csv_path}")
    print(f"Summary YAML: {summary_path}")
    print(yaml.safe_dump(summary, sort_keys=False).strip())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
