#!/usr/bin/env python3
"""Build one clean 100 Hz MTT research dataset from a qualified reference.

The builder reads selected topics directly from an MCAP through rosbag2_py. It
does not start a ROS graph, play a bag, or emit intermediate per-topic files.
All estimator poses are transported to ``track_contact_center`` and expressed
in the operational reference world frame.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation, Slerp

from build_msa_canonical_dataset import TRACK_IN_BASE_M
from extract_v2_measurements import (
    build_frame_graph,
    lookup_transform,
    read_tf_static,
)


TOPIC_KIND = {
    "/cmd_vel": "twist",
    "/cmd_vel/manual": "twist",
    "/cmd_vel/manual_raw": "twist",
    "/controller/cmd_vel": "twist",
    "/mtt/articulation_cmd": "float",
    "/mtt_articulation_setpoint": "float",
    "/mtt_articulation_velocity_cmd": "float",
    "/articulation_servo/setpoint_rad": "float",
    "/articulation_servo/measured_rad": "float",
    "/articulation_servo/error_rad": "float",
    "/hardware/articulation_angle": "float",
    "/mtt_tachometer": "tacho",
    "/mti100/data": "imu",
    "/zed/zed_node/imu/data": "imu",
    "/mtt_odometry": "odom",
    "/zed/zed_node/odom": "odom",
}

POSE_SPECS = {
    "/mtt_odometry": ("mtt_odom_track", "base_footprint", 0.10),
    "/zed/zed_node/odom": ("zed_odom_track", "zed_camera_link", 0.20),
    # /mapping/icp_odom is deliberately not read from the bag: the recorded
    # live ICP is known-broken in these sessions. ``icp_track`` is populated
    # only when an explicitly qualified offline ICP CSV is supplied.
    "/mapping/icp_odom": ("icp_track", "base_footprint", 0.15),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--terrain", required=True)
    parser.add_argument("--surface", required=True)
    parser.add_argument("--calibration-id", required=True)
    parser.add_argument("--reference-grade", required=True)
    parser.add_argument("--reference-source", required=True)
    parser.add_argument("--reference-world-frame", required=True)
    parser.add_argument("--offline-icp", type=Path)
    parser.add_argument("--tacho-scale", type=float)
    parser.add_argument("--tacho-calibration-id", default="none")
    parser.add_argument(
        "--lidar-articulation-smoothed",
        type=Path,
        help="Offline RS-Airy PCA hitch-angle CSV (timestamp_s,time_rel_s,"
        "lidar_rad,cmd_rad,pca_ratio,points,lidar_rad_smoothed) from Mohamed's "
        "separate offline LiDAR post-processing. Optional -- only sessions "
        "without a reliable hardware encoder have this.",
    )
    return parser.parse_args()


def stamp_to_s(stamp: Any, fallback_ns: int) -> float:
    if stamp is not None and (stamp.sec or stamp.nanosec):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return fallback_ns / 1e9


def message_time_s(message: Any, fallback_ns: int) -> float:
    header = getattr(message, "header", None)
    return stamp_to_s(getattr(header, "stamp", None), fallback_ns)


def extract_bag(
    bag: Path,
) -> tuple[dict[str, pd.DataFrame], dict[str, str], list[Any]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    static_edges = read_tf_static(topic_types, bag)
    wanted = [topic for topic in TOPIC_KIND if topic in topic_types]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    message_types = {
        topic: get_message(topic_types[topic]) for topic in wanted
    }
    records: dict[str, list[dict[str, Any]]] = defaultdict(list)
    corrupt_count: dict[str, int] = defaultdict(int)

    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        # A single truncated/corrupted CDR message anywhere in a multi-GB bag
        # ("rmw_serialize: invalid data size") must not abort extraction of
        # every other topic -- skip just this message and keep going (same
        # protection as extract_v2_measurements.py's bulk pass).
        try:
            message = deserialize_message(serialized, message_types[topic])
        except Exception:
            corrupt_count[topic] += 1
            continue
        t = message_time_s(message, record_ns)
        kind = TOPIC_KIND[topic]
        row: dict[str, Any] = {"t": t, "record_time_s": record_ns / 1e9}
        if kind == "float":
            row["value"] = float(message.data)
        elif kind == "twist":
            row.update(
                {
                    "linear_x": float(message.twist.linear.x),
                    "linear_y": float(message.twist.linear.y),
                    "linear_z": float(message.twist.linear.z),
                    "angular_x": float(message.twist.angular.x),
                    "angular_y": float(message.twist.angular.y),
                    "angular_z": float(message.twist.angular.z),
                }
            )
        elif kind == "imu":
            row.update(
                {
                    "frame_id": str(message.header.frame_id),
                    "ax": float(message.linear_acceleration.x),
                    "ay": float(message.linear_acceleration.y),
                    "az": float(message.linear_acceleration.z),
                    "wx": float(message.angular_velocity.x),
                    "wy": float(message.angular_velocity.y),
                    "wz": float(message.angular_velocity.z),
                }
            )
        elif kind == "tacho":
            direction = str(message.direction).casefold()
            direction_sign = (
                -1.0
                if direction == "reverse"
                else (1.0 if direction == "forward" else np.nan)
            )
            row.update(
                {
                    "speed_magnitude": float(message.speed_ms),
                    "direction_sign": direction_sign,
                    "is_synthetic": bool(message.tachometer_is_synthetic),
                    "source": str(message.tachometer_source),
                }
            )
        elif kind == "odom":
            position = message.pose.pose.position
            quaternion = message.pose.pose.orientation
            linear = message.twist.twist.linear
            angular = message.twist.twist.angular
            row.update(
                {
                    "world_frame": str(message.header.frame_id),
                    "child_frame": str(message.child_frame_id),
                    "x": float(position.x),
                    "y": float(position.y),
                    "z": float(position.z),
                    "qx": float(quaternion.x),
                    "qy": float(quaternion.y),
                    "qz": float(quaternion.z),
                    "qw": float(quaternion.w),
                    "vx": float(linear.x),
                    "vy": float(linear.y),
                    "vz": float(linear.z),
                    "wx": float(angular.x),
                    "wy": float(angular.y),
                    "wz": float(angular.z),
                }
            )
        records[topic].append(row)

    if corrupt_count:
        print(f"WARNING: corrupt messages skipped in extract_bag(): {dict(corrupt_count)}",
              file=sys.stderr)

    frames = {
        topic: (
            pd.DataFrame(rows)
            .sort_values("t")
            .drop_duplicates("t", keep="last")
            .reset_index(drop=True)
        )
        for topic, rows in records.items()
    }
    return frames, topic_types, static_edges


def previous_sample(
    target_t: np.ndarray,
    frame: pd.DataFrame | None,
    column: str,
    max_age_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    values = np.full(len(target_t), np.nan)
    source_time = np.full(len(target_t), np.nan)
    age = np.full(len(target_t), np.nan)
    if frame is None or frame.empty or column not in frame:
        return values, source_time, age
    source_t = frame["t"].to_numpy(dtype=float)
    source_v = pd.to_numeric(frame[column], errors="coerce").to_numpy(dtype=float)
    index = np.searchsorted(source_t, target_t, side="right") - 1
    valid_index = index >= 0
    clipped = np.maximum(index, 0)
    candidate_age = target_t - source_t[clipped]
    valid = (
        valid_index
        & np.isfinite(source_v[clipped])
        & (candidate_age >= -1e-9)
        & (candidate_age <= max_age_s)
    )
    values[valid] = source_v[clipped[valid]]
    source_time[valid] = source_t[clipped[valid]]
    age[valid] = candidate_age[valid]
    return values, source_time, age


def linear_sample(
    target_t: np.ndarray,
    frame: pd.DataFrame | None,
    column: str,
    max_gap_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    values = np.full(len(target_t), np.nan)
    source_time = np.full(len(target_t), np.nan)
    age = np.full(len(target_t), np.nan)
    if frame is None or frame.empty or column not in frame:
        return values, source_time, age
    source_t = frame["t"].to_numpy(dtype=float)
    source_v = pd.to_numeric(frame[column], errors="coerce").to_numpy(dtype=float)
    finite = np.isfinite(source_t) & np.isfinite(source_v)
    source_t, source_v = source_t[finite], source_v[finite]
    if len(source_t) < 2:
        return values, source_time, age
    right = np.searchsorted(source_t, target_t, side="left")
    left = right - 1
    exact = (right < len(source_t)) & (
        np.abs(source_t[np.minimum(right, len(source_t) - 1)] - target_t)
        <= 1e-9
    )
    left[exact] = right[exact]
    valid_index = (left >= 0) & (right < len(source_t))
    left_c = np.clip(left, 0, len(source_t) - 1)
    right_c = np.clip(right, 0, len(source_t) - 1)
    gap = source_t[right_c] - source_t[left_c]
    valid = valid_index & (gap <= max_gap_s)
    if np.any(valid):
        values[valid] = np.interp(
            target_t[valid], source_t, source_v
        )
        source_time[valid] = source_t[left_c[valid]]
        age[valid] = target_t[valid] - source_time[valid]
    return values, source_time, age


def homogeneous(rotation: Rotation, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rotation.as_matrix()
    result[:3, 3] = translation
    return result


def decompose(transform: np.ndarray) -> tuple[Rotation, np.ndarray]:
    return Rotation.from_matrix(transform[:3, :3]), transform[:3, 3].copy()


def child_to_track_transform(
    static_graph: dict[str, tuple[str, list[list[float]], list[float]]],
    child_frame: str,
) -> np.ndarray:
    base_frame = (
        "base_footprint"
        if "base_footprint" in static_graph
        or any(parent == "base_footprint" for parent, _, _ in static_graph.values())
        else "base_link"
    )
    t_base_track = homogeneous(Rotation.identity(), TRACK_IN_BASE_M)
    if child_frame in {"base_footprint", "base_link", base_frame}:
        return t_base_track
    r_base_child, p_base_child = lookup_transform(
        static_graph, base_frame, child_frame
    )
    t_base_child = homogeneous(
        Rotation.from_matrix(np.asarray(r_base_child, dtype=float)),
        np.asarray(p_base_child, dtype=float),
    )
    return np.linalg.inv(t_base_child) @ t_base_track


def transform_odom_to_track(
    frame: pd.DataFrame,
    t_child_track: np.ndarray,
) -> pd.DataFrame:
    result = frame.copy()
    source_rotation = Rotation.from_quat(
        result[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
    )
    source_position = result[["x", "y", "z"]].to_numpy(dtype=float)
    r_child_track, p_child_track = decompose(t_child_track)
    track_rotation = source_rotation * r_child_track
    track_position = source_position + source_rotation.apply(
        np.broadcast_to(p_child_track, source_position.shape)
    )

    linear_child = result[["vx", "vy", "vz"]].to_numpy(dtype=float)
    angular_child = result[["wx", "wy", "wz"]].to_numpy(dtype=float)
    linear_at_track_child = linear_child + np.cross(
        angular_child, np.broadcast_to(p_child_track, linear_child.shape)
    )
    r_track_child = r_child_track.inv()
    linear_track = r_track_child.apply(linear_at_track_child)
    angular_track = r_track_child.apply(angular_child)

    quaternion = track_rotation.as_quat()
    result[["x", "y", "z"]] = track_position
    result[["qx", "qy", "qz", "qw"]] = quaternion
    result[["vx", "vy", "vz"]] = linear_track
    result[["wx", "wy", "wz"]] = angular_track
    result["child_frame"] = "track_contact_center"
    return result


def interpolate_reference_pose(
    reference: pd.DataFrame, query_t: float
) -> np.ndarray:
    t = reference["time_s"].to_numpy(dtype=float)
    if query_t < t[0] or query_t > t[-1]:
        raise ValueError("Alignment time is outside reference coverage")
    position = np.array(
        [
            np.interp(query_t, t, reference[f"gt_track_{axis}_m"])
            for axis in "xyz"
        ]
    )
    rotation = Slerp(
        t,
        Rotation.from_quat(
            reference[
                ["gt_track_qx", "gt_track_qy", "gt_track_qz", "gt_track_qw"]
            ].to_numpy(dtype=float)
        ),
    )(query_t)
    return homogeneous(rotation, position)


def align_pose_world_once(
    frame: pd.DataFrame, reference: pd.DataFrame
) -> tuple[pd.DataFrame, np.ndarray]:
    valid = frame[
        (frame["t"] >= reference["time_s"].iloc[0])
        & (frame["t"] <= reference["time_s"].iloc[-1])
    ]
    if valid.empty:
        raise ValueError("Pose source has no overlap with the reference")
    first = valid.iloc[0]
    source_first = homogeneous(
        Rotation.from_quat(
            [first.qx, first.qy, first.qz, first.qw]
        ),
        np.array([first.x, first.y, first.z], dtype=float),
    )
    reference_first = interpolate_reference_pose(reference, float(first.t))
    t_reference_source = reference_first @ np.linalg.inv(source_first)

    source_rotation = Rotation.from_quat(
        frame[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
    )
    source_position = frame[["x", "y", "z"]].to_numpy(dtype=float)
    align_rotation, align_position = decompose(t_reference_source)
    aligned_rotation = align_rotation * source_rotation
    aligned_position = align_rotation.apply(source_position) + align_position
    result = frame.copy()
    result[["x", "y", "z"]] = aligned_position
    result[["qx", "qy", "qz", "qw"]] = aligned_rotation.as_quat()
    result["world_frame"] = "reference_world"
    return result, t_reference_source


def resample_pose(
    target_t: np.ndarray,
    frame: pd.DataFrame,
    prefix: str,
    max_gap_s: float,
    nearest_tolerance_s: float | None = None,
) -> pd.DataFrame:
    source = frame.sort_values("t").drop_duplicates("t", keep="last")
    source_t = source["t"].to_numpy(dtype=float)
    right = np.searchsorted(source_t, target_t, side="left")
    left = right - 1
    exact = (right < len(source_t)) & (
        np.abs(source_t[np.minimum(right, len(source_t) - 1)] - target_t)
        <= 1e-9
    )
    left[exact] = right[exact]
    valid_index = (left >= 0) & (right < len(source_t))
    left_c = np.clip(left, 0, len(source_t) - 1)
    right_c = np.clip(right, 0, len(source_t) - 1)
    gap = source_t[right_c] - source_t[left_c]
    interpolation_valid = valid_index & (gap <= max_gap_s)
    nearest_index = np.zeros(len(target_t), dtype=int)
    nearest_valid = np.zeros(len(target_t), dtype=bool)
    if nearest_tolerance_s is not None:
        right_near = np.clip(right, 0, len(source_t) - 1)
        left_near = np.clip(right - 1, 0, len(source_t) - 1)
        choose_right = (
            np.abs(source_t[right_near] - target_t)
            < np.abs(source_t[left_near] - target_t)
        )
        nearest_index = np.where(choose_right, right_near, left_near)
        nearest_distance = np.abs(source_t[nearest_index] - target_t)
        nearest_valid = (
            ~interpolation_valid
            & (nearest_distance <= nearest_tolerance_s)
        )
    valid = interpolation_valid | nearest_valid

    out = pd.DataFrame(index=np.arange(len(target_t)))
    for column, suffix in [
        ("x", "x_m"),
        ("y", "y_m"),
        ("z", "z_m"),
        ("vx", "vx_ms"),
        ("vy", "vy_ms"),
        ("vz", "vz_ms"),
        ("wx", "wx_rad_s"),
        ("wy", "wy_rad_s"),
        ("wz", "wz_rad_s"),
    ]:
        values = np.full(len(target_t), np.nan)
        source_values = source[column].to_numpy(dtype=float)
        if np.any(interpolation_valid):
            values[interpolation_valid] = np.interp(
                target_t[interpolation_valid], source_t, source_values
            )
        values[nearest_valid] = source_values[nearest_index[nearest_valid]]
        out[f"{prefix}_{suffix}"] = values

    quaternion = np.full((len(target_t), 4), np.nan)
    if np.any(interpolation_valid):
        quaternion[interpolation_valid] = Slerp(
            source_t,
            Rotation.from_quat(
                source[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
            ),
        )(target_t[interpolation_valid]).as_quat()
    quaternion[nearest_valid] = source[
        ["qx", "qy", "qz", "qw"]
    ].to_numpy(dtype=float)[nearest_index[nearest_valid]]
    out[
        [f"{prefix}_qx", f"{prefix}_qy", f"{prefix}_qz", f"{prefix}_qw"]
    ] = quaternion
    rpy = np.full((len(target_t), 3), np.nan)
    if np.any(valid):
        rpy[valid] = Rotation.from_quat(quaternion[valid]).as_euler("xyz")
    out[
        [
            f"{prefix}_roll_rad",
            f"{prefix}_pitch_rad",
            f"{prefix}_yaw_rad",
        ]
    ] = rpy
    source_time = np.full(len(target_t), np.nan)
    age = np.full(len(target_t), np.nan)
    source_time[interpolation_valid] = source_t[left_c[interpolation_valid]]
    source_time[nearest_valid] = source_t[nearest_index[nearest_valid]]
    age[valid] = target_t[valid] - source_time[valid]
    out[f"{prefix}_source_time_s"] = source_time
    out[f"{prefix}_age_s"] = age
    out[f"valid_{prefix}_pose"] = valid
    return out


def offline_icp_frame(path: Path) -> pd.DataFrame:
    raw = pd.read_csv(path)
    # KISS-ICP's icp_odom.csv names its angular-velocity columns
    # roll_rate/pitch_rate/yaw_rate instead of norlab's wx/wy/wz -- same
    # quantities, different header. Normalize to wx/wy/wz here so the rest of
    # this function (and every downstream consumer) is source-agnostic.
    if not {"wx", "wy", "wz"} <= set(raw.columns) and {"roll_rate", "pitch_rate", "yaw_rate"} <= set(raw.columns):
        raw = raw.rename(columns={"roll_rate": "wx", "pitch_rate": "wy", "yaw_rate": "wz"})
    time = (
        raw["timestamp_sec"].to_numpy(dtype=float)
        + raw["timestamp_nanosec"].to_numpy(dtype=float) * 1e-9
    )
    result = pd.DataFrame(
        {
            "t": time,
            "record_time_s": time,
            "world_frame": "icp_map",
            # Despite the map/trajectory filenames containing "hesai", the
            # mapper CSV publishes the robot/base pose. This is verified by
            # the identity first ICP pose and its millimetric agreement with
            # pose_robot_gt_100hz at the same timestamp.
            "child_frame": "base_footprint",
        }
    )
    for column in [
        "x",
        "y",
        "z",
        "qx",
        "qy",
        "qz",
        "qw",
        "vx",
        "vy",
        "vz",
        "wx",
        "wy",
        "wz",
    ]:
        result[column] = pd.to_numeric(raw[column], errors="coerce")
    return result.sort_values("t").drop_duplicates("t", keep="last")


def add_held_signal(
    output: pd.DataFrame,
    target_t: np.ndarray,
    frame: pd.DataFrame | None,
    source_column: str,
    output_column: str,
    time_prefix: str,
    max_age_s: float = 0.08,
) -> None:
    values, source_time, age = previous_sample(
        target_t, frame, source_column, max_age_s
    )
    output[output_column] = values
    output[f"{time_prefix}_source_time_s"] = source_time
    output[f"{time_prefix}_age_s"] = age


def add_linear_signal(
    output: pd.DataFrame,
    target_t: np.ndarray,
    frame: pd.DataFrame | None,
    source_column: str,
    output_column: str,
    time_prefix: str,
    max_gap_s: float = 0.08,
) -> None:
    values, source_time, age = linear_sample(
        target_t, frame, source_column, max_gap_s
    )
    output[output_column] = values
    output[f"{time_prefix}_source_time_s"] = source_time
    output[f"{time_prefix}_age_s"] = age


def rotate_imu(
    frame: pd.DataFrame,
    static_graph: dict[str, tuple[str, list[list[float]], list[float]]],
    fallback_frame: str,
) -> tuple[pd.DataFrame, str]:
    if frame.empty:
        return frame, "missing"
    source_frame = str(frame["frame_id"].dropna().iloc[0]) or fallback_frame
    base_frame = (
        "base_footprint"
        if "base_footprint" in static_graph
        or any(parent == "base_footprint" for parent, _, _ in static_graph.values())
        else "base_link"
    )
    candidates = [source_frame]
    if source_frame == "zed_imu_link":
        candidates.extend(
            [
                "zed_camera_link",
                "zed_camera_center",
                "zed_left_camera_frame",
            ]
        )
    r_base_source = None
    used_frame = None
    for candidate in candidates:
        try:
            r_base_source, _ = lookup_transform(
                static_graph, base_frame, candidate
            )
            used_frame = candidate
            break
        except RuntimeError:
            continue
    if r_base_source is None or used_frame is None:
        return frame.iloc[0:0].copy(), f"unresolved:{source_frame}"
    rotation = Rotation.from_matrix(np.asarray(r_base_source, dtype=float))
    result = frame.copy()
    result[["ax", "ay", "az"]] = rotation.apply(
        frame[["ax", "ay", "az"]].to_numpy(dtype=float)
    )
    result[["wx", "wy", "wz"]] = rotation.apply(
        frame[["wx", "wy", "wz"]].to_numpy(dtype=float)
    )
    status = (
        f"recorded_tf:{source_frame}"
        if used_frame == source_frame
        else f"fallback_tf:{used_frame}:for:{source_frame}:unvalidated"
    )
    return result, status


def write_trajectory_ply(path: Path, reference: pd.DataFrame) -> None:
    valid = reference["valid_gt_pose"].astype(bool).to_numpy()
    xyz = reference.loc[
        valid, ["gt_track_x_m", "gt_track_y_m", "gt_track_z_m"]
    ].to_numpy(dtype="<f4")
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(xyz)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    ).encode("ascii")
    with path.open("wb") as stream:
        stream.write(header)
        for point in xyz:
            stream.write(
                struct.pack(
                    "<fffBBB",
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                    0,
                    0,
                    0,
                )
            )


def validate(output: pd.DataFrame) -> dict[str, float]:
    required = [
        "time_s",
        "sample_index",
        "gt_track_x_m",
        "gt_track_y_m",
        "gt_track_qx",
        "gt_track_qy",
        "gt_track_qz",
        "gt_track_qw",
        "valid_gt_pose",
        "cmd_vel_linear_x_ms",
        "articulation_hardware_rad",
    ]
    missing = [column for column in required if column not in output]
    if missing:
        raise ValueError(f"Missing mandatory columns: {missing}")
    t = output["time_s"].to_numpy(dtype=float)
    if not np.isfinite(t).all() or not np.all(np.diff(t) > 0):
        raise ValueError("Canonical timestamps are not finite and increasing")
    if output["sample_index"].tolist() != list(range(len(output))):
        raise ValueError("sample_index is not contiguous")
    median_dt = float(np.median(np.diff(t)))
    if not math.isclose(median_dt, 0.01, abs_tol=2e-6):
        raise ValueError(f"Unexpected median dt: {median_dt}")
    valid = output["valid_gt_pose"].astype(bool).to_numpy()
    quaternion = output.loc[
        valid, ["gt_track_qx", "gt_track_qy", "gt_track_qz", "gt_track_qw"]
    ].to_numpy(dtype=float)
    norm_error = np.abs(np.linalg.norm(quaternion, axis=1) - 1.0)
    if not np.isfinite(norm_error).all() or float(np.max(norm_error)) > 1e-5:
        raise ValueError("Reference quaternion norm validation failed")
    return {
        "rows": float(len(output)),
        "duration_s": float(t[-1] - t[0]),
        "median_dt_s": median_dt,
        "gt_valid_fraction": float(np.mean(valid)),
        "max_gt_quaternion_norm_error": float(np.max(norm_error)),
    }


def main() -> int:
    args = parse_args()
    session = args.session.resolve()
    bag = session / "bag"
    reference = pd.read_csv(args.reference).sort_values("time_s").reset_index(
        drop=True
    )
    if not np.all(np.diff(reference["time_s"].to_numpy(dtype=float)) > 0):
        raise ValueError("Reference timestamps are not strictly increasing")

    frames, _, static_edges = extract_bag(bag)
    static_graph = build_frame_graph(static_edges)
    target_t = reference["time_s"].to_numpy(dtype=float)
    output = reference.copy()

    # Remove old packaging metadata and replace it with the final contract.
    output = output.drop(
        columns=[
            "reference_frame",
            "reference_source",
            "calibration_profile",
        ],
        errors="ignore",
    )
    output.insert(0, "session_id", args.session_id)
    output.insert(1, "terrain", args.terrain)
    output.insert(2, "surface", args.surface)
    output.insert(3, "calibration_id", args.calibration_id)
    output.insert(4, "reference_grade", args.reference_grade)
    output.insert(5, "reference_source", args.reference_source)
    output.insert(6, "reference_world_frame", args.reference_world_frame)
    output.insert(7, "body_reference_frame", "track_contact_center")
    output.insert(8, "sample_index", np.arange(len(output), dtype=np.int64))
    output["time_from_start_s"] = target_t - target_t[0]
    output["dt_s"] = np.r_[np.nan, np.diff(target_t)]
    if "split_contiguous_v2" in output:
        output["split_id"] = output["split_contiguous_v2"]
    elif "split_contiguous_v1" in output:
        output["split_id"] = output["split_contiguous_v1"]
    else:
        output["split_id"] = "unassigned"
    output = output.drop(
        columns=["split_contiguous_v1", "split_contiguous_v2"],
        errors="ignore",
    )
    output["valid_row"] = True
    output["valid_gt_twist"] = (
        output["valid_gt_pose"].astype(bool)
        & np.isfinite(output["gt_track_vx_raw_ms"])
        & np.isfinite(output["gt_track_wz_raw_rad_s"])
    )
    if "gt_quality_flags" not in output:
        output["gt_quality_flags"] = np.where(
            output["valid_gt_pose"].astype(bool), "ok", "invalid_pose"
        )
    for column in [
        "gt_sigma_x_m",
        "gt_sigma_y_m",
        "gt_sigma_z_m",
        "gt_sigma_yaw_rad",
    ]:
        if column not in output:
            output[column] = np.nan

    cmd = frames.get("/cmd_vel")
    add_held_signal(
        output,
        target_t,
        cmd,
        "linear_x",
        "cmd_vel_linear_x_ms",
        "cmd_vel",
    )
    cmd_angular, _, _ = previous_sample(target_t, cmd, "angular_z", 0.08)
    output["cmd_vel_angular_z_normalized"] = cmd_angular
    output["valid_cmd_vel"] = (
        np.isfinite(output["cmd_vel_linear_x_ms"])
        & np.isfinite(output["cmd_vel_angular_z_normalized"])
    )

    manual = frames.get("/cmd_vel/manual")
    output["cmd_manual_linear_x_ms"], _, _ = previous_sample(
        target_t, manual, "linear_x", 0.08
    )
    output["cmd_manual_angular_z_normalized"], _, _ = previous_sample(
        target_t, manual, "angular_z", 0.08
    )
    controller = frames.get("/controller/cmd_vel")
    output["cmd_controller_linear_x_ms"], _, _ = previous_sample(
        target_t, controller, "linear_x", 0.08
    )
    output["cmd_controller_angular_z_normalized"], _, _ = previous_sample(
        target_t, controller, "angular_z", 0.08
    )

    raw_articulation_cmd = frames.get("/mtt/articulation_cmd")
    add_held_signal(
        output,
        target_t,
        raw_articulation_cmd,
        "value",
        "articulation_command_raw",
        "articulation_command",
    )
    setpoint = frames.get("/mtt_articulation_setpoint")
    articulation_rad, setpoint_time, setpoint_age = previous_sample(
        target_t, setpoint, "value", 0.08
    )
    output["articulation_command_rad"] = articulation_rad
    has_radian_setpoint = np.isfinite(articulation_rad).any()
    output["articulation_command_normalized"] = (
        np.nan if has_radian_setpoint else cmd_angular
    )
    if has_radian_setpoint:
        output["articulation_command_source_time_s"] = setpoint_time
        output["articulation_command_age_s"] = setpoint_age
    else:
        output["articulation_command_source_time_s"] = output[
            "cmd_vel_source_time_s"
        ]
        output["articulation_command_age_s"] = output["cmd_vel_age_s"]
    output["valid_articulation_command"] = (
        np.isfinite(output["articulation_command_rad"])
        | np.isfinite(output["articulation_command_normalized"])
    )

    hardware = frames.get("/hardware/articulation_angle")
    add_linear_signal(
        output,
        target_t,
        hardware,
        "value",
        "articulation_hardware_rad",
        "articulation_hardware",
    )
    output["articulation_hardware_available"] = np.isfinite(
        output["articulation_hardware_rad"]
    )
    output["articulation_hardware_within_physical_limit"] = (
        output["articulation_hardware_available"]
        & (np.abs(output["articulation_hardware_rad"]) <= np.deg2rad(45.0) + 1e-9)
    )

    # Offline RS-Airy PCA hitch angle (lidar_rad_smoothed): a separate,
    # research-grade ground truth for sessions without a reliable hardware
    # encoder (see articulation_hardware_available above). Not a bag topic --
    # comes from Mohamed's standalone offline post-processing CSV, so it is
    # loaded and resampled onto target_t the same way bag-derived signals are,
    # rather than going through extract_bag()/frames.
    if args.lidar_articulation_smoothed is not None:
        lidar_smoothed = pd.read_csv(args.lidar_articulation_smoothed).rename(
            columns={"timestamp_s": "t"}
        )
        # RS-Airy PCA runs at ~10 Hz (~0.1s spacing) -- add_linear_signal's
        # default max_gap_s=0.08 would reject every sample; widen to cover
        # the actual source rate with margin.
        add_linear_signal(
            output, target_t, lidar_smoothed, "lidar_rad",
            "articulation_lidar_pca_rad", "articulation_lidar_pca",
            max_gap_s=0.15,
        )
        add_linear_signal(
            output, target_t, lidar_smoothed, "lidar_rad_smoothed",
            "articulation_lidar_pca_smoothed_rad", "articulation_lidar_pca_smoothed",
            max_gap_s=0.15,
        )
        output["articulation_lidar_pca_cmd_rad"], _, _ = previous_sample(
            target_t, lidar_smoothed, "cmd_rad", 0.15
        )
        output["articulation_lidar_pca_ratio"], _, _ = previous_sample(
            target_t, lidar_smoothed, "pca_ratio", 0.15
        )
        output["articulation_lidar_pca_points"], _, _ = previous_sample(
            target_t, lidar_smoothed, "points", 0.15
        )
    else:
        for column in [
            "articulation_lidar_pca_rad",
            "articulation_lidar_pca_smoothed_rad",
            "articulation_lidar_pca_cmd_rad",
            "articulation_lidar_pca_ratio",
            "articulation_lidar_pca_points",
        ]:
            output[column] = np.nan
        output["articulation_lidar_pca_source_time_s"] = np.nan
        output["articulation_lidar_pca_age_s"] = np.nan
        output["articulation_lidar_pca_smoothed_source_time_s"] = np.nan
        output["articulation_lidar_pca_smoothed_age_s"] = np.nan
    output["articulation_lidar_pca_smoothed_available"] = np.isfinite(
        output["articulation_lidar_pca_smoothed_rad"]
    )

    for topic, column in [
        ("/articulation_servo/setpoint_rad", "articulation_servo_setpoint_rad"),
        ("/articulation_servo/measured_rad", "articulation_servo_measured_rad"),
        ("/articulation_servo/error_rad", "articulation_servo_error_rad"),
        (
            "/mtt_articulation_velocity_cmd",
            "articulation_velocity_command_rad_s",
        ),
    ]:
        output[column], _, _ = previous_sample(
            target_t, frames.get(topic), "value", 0.08
        )

    tacho = frames.get("/mtt_tachometer")
    add_linear_signal(
        output,
        target_t,
        tacho,
        "speed_magnitude",
        "tacho_speed_magnitude_raw_ms",
        "tacho",
    )
    direction, _, _ = previous_sample(
        target_t, tacho, "direction_sign", 0.08
    )
    output["tacho_direction_sign"] = direction
    output["tacho_speed_signed_raw_ms"] = (
        output["tacho_speed_magnitude_raw_ms"] * direction
    )
    if args.tacho_scale is None:
        output["tacho_speed_signed_calibrated_ms"] = np.nan
    else:
        output["tacho_speed_signed_calibrated_ms"] = (
            args.tacho_scale * output["tacho_speed_signed_raw_ms"]
        )
    output["tacho_calibration_id"] = args.tacho_calibration_id
    synthetic, _, _ = previous_sample(
        target_t, tacho, "is_synthetic", 0.08
    )
    output["tacho_is_synthetic"] = synthetic
    output["valid_tacho"] = (
        np.isfinite(output["tacho_speed_signed_raw_ms"])
        & (output["tacho_is_synthetic"] == 0)
    )

    for topic, prefix, fallback_frame in [
        ("/mti100/data", "imu_mti100", "imu_link"),
        ("/zed/zed_node/imu/data", "imu_zed", "zed_imu_link"),
    ]:
        imu = frames.get(topic)
        extrinsic_status = "missing"
        if imu is not None and not imu.empty:
            imu, extrinsic_status = rotate_imu(
                imu, static_graph, fallback_frame
            )
        first_time = None
        first_age = None
        for source_column, suffix in [
            ("ax", "ax_ms2"),
            ("ay", "ay_ms2"),
            ("az", "az_ms2"),
            ("wx", "wx_rad_s"),
            ("wy", "wy_rad_s"),
            ("wz", "wz_rad_s"),
        ]:
            values, source_time, age = linear_sample(
                target_t, imu, source_column, 0.03
            )
            output[f"{prefix}_{suffix}"] = values
            if first_time is None:
                first_time, first_age = source_time, age
        output[f"{prefix}_source_time_s"] = first_time
        output[f"{prefix}_age_s"] = first_age
        output[f"{prefix}_extrinsic_status"] = extrinsic_status
        output[f"valid_{prefix}"] = np.isfinite(
            output[f"{prefix}_wz_rad_s"]
        )

    for topic, (prefix, expected_child, max_gap) in POSE_SPECS.items():
        pose = frames.get(topic)
        if topic == "/mapping/icp_odom" and args.offline_icp:
            pose = offline_icp_frame(args.offline_icp.resolve())
            expected_child = "base_footprint"
            max_gap = 0.15
        if pose is None or pose.empty:
            continue
        observed_child = str(pose["child_frame"].dropna().iloc[0])
        child = observed_child or expected_child
        t_child_track = child_to_track_transform(static_graph, child)
        pose_track = transform_odom_to_track(pose, t_child_track)
        # The qualified offline Ice-rink ICP already uses the same map frame as
        # the operational reference. Other odometries receive one fixed SE(3)
        # alignment at their first common timestamp.
        if not (topic == "/mapping/icp_odom" and args.offline_icp):
            pose_track, _ = align_pose_world_once(pose_track, reference)
        nearest_tolerance = (
            0.03
            if topic == "/mapping/icp_odom" and not args.offline_icp
            else None
        )
        sampled = resample_pose(
            target_t,
            pose_track,
            prefix,
            max_gap,
            nearest_tolerance_s=nearest_tolerance,
        )
        output = pd.concat(
            [output.reset_index(drop=True), sampled.reset_index(drop=True)],
            axis=1,
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    summary = validate(output)
    output.to_csv(args.output, index=False, compression="gzip")
    reloaded = pd.read_csv(args.output)
    if reloaded.shape != output.shape or list(reloaded.columns) != list(
        output.columns
    ):
        raise RuntimeError("CSV read-back validation failed")
    write_trajectory_ply(args.trajectory, output)

    print(f"wrote {args.output}: {output.shape[0]} rows x {output.shape[1]} columns")
    print(f"wrote {args.trajectory}")
    for key, value in summary.items():
        print(f"{key}: {value}")
    for prefix in ["mtt_odom_track", "icp_track", "zed_odom_track"]:
        column = f"valid_{prefix}_pose"
        if column in output:
            print(f"{prefix}_coverage: {float(output[column].mean()):.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
