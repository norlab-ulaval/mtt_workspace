#!/usr/bin/env python3
"""Build a colored 3D map from recorded ZED point clouds and ZED pose.

This is an offline diagnostic/reconstruction tool. It intentionally does not use
wheel odometry or ICP. The expected input is a bag containing:
  /zed/zed_node/pose
  /zed/zed_node/point_cloud/cloud_registered
  /tf_static
"""

from __future__ import annotations

import argparse
import csv
import math
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@dataclass(frozen=True)
class PoseSample:
    t: float
    T: np.ndarray


def stamp_to_sec(msg, fallback_ns: int) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and (stamp.sec or stamp.nanosec):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(fallback_ns) * 1e-9


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def transform_from_xyz_quat(xyz: Iterable[float], quat_xyzw: Iterable[float]) -> np.ndarray:
    qx, qy, qz, qw = [float(v) for v in quat_xyzw]
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
    T[:3, 3] = np.array([float(v) for v in xyz], dtype=np.float64)
    return T


def resolve_bag_dir(path: Path) -> Path:
    path = path.expanduser().resolve()
    if (path / "metadata.yaml").is_file():
        return path
    if (path / "bag" / "metadata.yaml").is_file():
        return path / "bag"
    raise SystemExit(f"Cannot resolve bag directory from {path}")


def open_reader(bag_dir: Path, topics: list[str]):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = [topic for topic in topics if topic in topic_types]
    if not selected:
        raise SystemExit(f"None of requested topics found: {topics}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    msg_types = {topic: get_message(topic_types[topic]) for topic in selected}
    return reader, msg_types


def load_static_transform(bag_dir: Path, parent: str, child: str) -> np.ndarray:
    reader, msg_types = open_reader(bag_dir, ["/tf_static"])
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        del timestamp_ns
        msg = deserialize_message(raw, msg_types[topic])
        for tr in msg.transforms:
            if tr.header.frame_id == parent and tr.child_frame_id == child:
                q = tr.transform.rotation
                t = tr.transform.translation
                return transform_from_xyz_quat((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
    raise SystemExit(f"Missing static TF {parent} -> {child}")


def load_poses(bag_dir: Path, pose_topic: str, max_speed_mps: float, max_gap_s: float) -> tuple[list[PoseSample], list[bool]]:
    reader, msg_types = open_reader(bag_dir, [pose_topic])
    poses: list[PoseSample] = []
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        msg = deserialize_message(raw, msg_types[topic])
        t = stamp_to_sec(msg, timestamp_ns)
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        p = pose.position
        q = pose.orientation
        poses.append(PoseSample(t, transform_from_xyz_quat((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))))

    if not poses:
        raise SystemExit(f"No poses on {pose_topic}")

    valid = [True] * len(poses)
    for i in range(1, len(poses)):
        dt = poses[i].t - poses[i - 1].t
        step = float(np.linalg.norm(poses[i].T[:3, 3] - poses[i - 1].T[:3, 3]))
        if dt <= 0.0 or dt > max_gap_s or (dt > 1e-6 and step / dt > max_speed_mps):
            valid[i] = False
    return poses, valid


def nearest_pose_index(times: list[float], t: float) -> int:
    idx = bisect_left(times, t)
    if idx <= 0:
        return 0
    if idx >= len(times):
        return len(times) - 1
    return idx if abs(times[idx] - t) < abs(times[idx - 1] - t) else idx - 1


def parse_box(values: list[float] | None) -> tuple[float, float, float, float, float, float] | None:
    if values is None:
        return None
    if len(values) != 6:
        raise SystemExit("--exclude-box expects 6 values: xmin xmax ymin ymax zmin zmax")
    xmin, xmax, ymin, ymax, zmin, zmax = values
    return (xmin, xmax, ymin, ymax, zmin, zmax)


def decode_zed_cloud(
    msg,
    pixel_step: int,
    min_range_m: float,
    max_range_m: float,
    min_forward_m: float,
    exclude_box: tuple[float, float, float, float, float, float] | None,
) -> tuple[np.ndarray, np.ndarray]:
    if msg.point_step != 16:
        raise RuntimeError(f"Unsupported point_step={msg.point_step}; expected 16 bytes")
    dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
    arr = np.frombuffer(msg.data, dtype=dtype, count=int(msg.width) * int(msg.height))
    if pixel_step > 1:
        arr = arr[::pixel_step]
    xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float32, copy=False)
    ranges = np.linalg.norm(xyz, axis=1)
    mask = (
        np.isfinite(xyz).all(axis=1)
        & (ranges >= min_range_m)
        & (ranges <= max_range_m)
        & (xyz[:, 0] >= min_forward_m)
    )
    if exclude_box is not None:
        xmin, xmax, ymin, ymax, zmin, zmax = exclude_box
        in_box = (
            (xyz[:, 0] >= xmin)
            & (xyz[:, 0] <= xmax)
            & (xyz[:, 1] >= ymin)
            & (xyz[:, 1] <= ymax)
            & (xyz[:, 2] >= zmin)
            & (xyz[:, 2] <= zmax)
        )
        mask &= ~in_box
    xyz = xyz[mask]
    rgb_float = arr["rgb"][mask]
    rgb_u32 = rgb_float.view(np.uint32)
    colors = np.column_stack(
        (
            ((rgb_u32 >> 16) & 255),
            ((rgb_u32 >> 8) & 255),
            (rgb_u32 & 255),
        )
    ).astype(np.float32) / 255.0
    return xyz.astype(np.float64), colors.astype(np.float64)


def write_trajectory_csv(path: Path, poses: list[PoseSample], valid: list[bool]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t", "x", "y", "z", "valid"])
        for pose, ok in zip(poses, valid):
            p = pose.T[:3, 3]
            writer.writerow([f"{pose.t:.9f}", f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}", int(ok)])


def voxel_downsample(xyz: np.ndarray, rgb: np.ndarray, voxel_size: float) -> tuple[np.ndarray, np.ndarray]:
    if voxel_size <= 0.0 or len(xyz) == 0:
        return xyz, rgb
    keys = np.floor(xyz / voxel_size).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse).astype(np.float64)
    out_xyz = np.zeros((len(counts), 3), dtype=np.float64)
    out_rgb = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(out_xyz, inverse, xyz)
    np.add.at(out_rgb, inverse, rgb)
    out_xyz /= counts[:, None]
    out_rgb /= counts[:, None]
    return out_xyz, out_rgb


def write_binary_ply(path: Path, xyz: np.ndarray, rgb: np.ndarray) -> None:
    rgb_u8 = np.clip(np.rint(rgb * 255.0), 0, 255).astype(np.uint8)
    data = np.empty(
        len(xyz),
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
        ],
    )
    data["x"] = xyz[:, 0].astype(np.float32)
    data["y"] = xyz[:, 1].astype(np.float32)
    data["z"] = xyz[:, 2].astype(np.float32)
    data["red"] = rgb_u8[:, 0]
    data["green"] = rgb_u8[:, 1]
    data["blue"] = rgb_u8[:, 2]
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(data)}\n"
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
        data.tofile(stream)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="Session directory or bag directory")
    parser.add_argument("--pose-topic", default="/zed/zed_node/pose")
    parser.add_argument("--cloud-topic", default="/zed/zed_node/point_cloud/cloud_registered")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--cloud-period-s", type=float, default=0.50)
    parser.add_argument("--pixel-step", type=int, default=8)
    parser.add_argument("--voxel-size", type=float, default=0.08)
    parser.add_argument("--min-range-m", type=float, default=0.4)
    parser.add_argument("--max-range-m", type=float, default=18.0)
    parser.add_argument(
        "--min-forward-m",
        type=float,
        default=0.4,
        help="Reject ZED points with x lower than this value in zed_left_camera_frame; useful to remove robot bumper.",
    )
    parser.add_argument(
        "--exclude-box",
        type=float,
        nargs=6,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        help="Reject a local ZED-frame box before transforming points.",
    )
    parser.add_argument("--max-pose-dt-s", type=float, default=0.20)
    parser.add_argument("--max-speed-mps", type=float, default=8.0)
    parser.add_argument("--max-pose-gap-s", type=float, default=1.0)
    parser.add_argument("--max-clouds", type=int, default=0, help="0 = no explicit limit")
    parser.add_argument("--write-raw", action="store_true", help="Also write the raw sampled PLY before voxelization")
    args = parser.parse_args()

    session = args.session.expanduser().resolve()
    bag_dir = resolve_bag_dir(session)
    session_dir = bag_dir.parent if bag_dir.name == "bag" else session
    out_dir = args.output_dir or (session_dir / "zed_visual_map")
    out_dir.mkdir(parents=True, exist_ok=True)

    T_camera_left = (
        load_static_transform(bag_dir, "zed_camera_link", "zed_camera_center")
        @ load_static_transform(bag_dir, "zed_camera_center", "zed_left_camera_frame")
    )
    poses, pose_valid = load_poses(bag_dir, args.pose_topic, args.max_speed_mps, args.max_pose_gap_s)
    pose_times = [pose.t for pose in poses]
    write_trajectory_csv(out_dir / "zed_pose_trajectory.csv", poses, pose_valid)
    exclude_box = parse_box(args.exclude_box)

    reader, msg_types = open_reader(bag_dir, [args.cloud_topic])
    all_xyz: list[np.ndarray] = []
    all_rgb: list[np.ndarray] = []
    last_cloud_t = -math.inf
    used_clouds = 0
    skipped_time = 0
    skipped_pose = 0
    skipped_empty = 0

    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        msg = deserialize_message(raw, msg_types[topic])
        t = stamp_to_sec(msg, timestamp_ns)
        if t - last_cloud_t < args.cloud_period_s:
            skipped_time += 1
            continue
        pose_idx = nearest_pose_index(pose_times, t)
        if abs(pose_times[pose_idx] - t) > args.max_pose_dt_s or not pose_valid[pose_idx]:
            skipped_pose += 1
            continue
        xyz, colors = decode_zed_cloud(
            msg,
            args.pixel_step,
            args.min_range_m,
            args.max_range_m,
            args.min_forward_m,
            exclude_box,
        )
        if xyz.size == 0:
            skipped_empty += 1
            continue
        T_map_left = poses[pose_idx].T @ T_camera_left
        xyz_map = (T_map_left[:3, :3] @ xyz.T).T + T_map_left[:3, 3]
        all_xyz.append(xyz_map)
        all_rgb.append(colors)
        last_cloud_t = t
        used_clouds += 1
        if used_clouds % 100 == 0:
            print(f"used_clouds={used_clouds} points_raw={sum(len(x) for x in all_xyz)} t={t:.2f}", flush=True)
        if args.max_clouds > 0 and used_clouds >= args.max_clouds:
            break

    if not all_xyz:
        raise SystemExit("No clouds accepted; relax timing/range filters")

    xyz = np.concatenate(all_xyz, axis=0)
    rgb = np.concatenate(all_rgb, axis=0)
    raw_path = out_dir / "zed_visual_map_raw_sampled.ply"
    if args.write_raw:
        write_binary_ply(raw_path, xyz, rgb)
    elif raw_path.exists():
        raw_path.unlink()

    voxel_xyz, voxel_rgb = xyz, rgb
    if args.voxel_size > 0.0:
        voxel_xyz, voxel_rgb = voxel_downsample(xyz, rgb, args.voxel_size)
    clean_path = out_dir / "zed_visual_map_voxel.ply"
    write_binary_ply(clean_path, voxel_xyz, voxel_rgb)

    with (out_dir / "summary.txt").open("w", encoding="utf-8") as stream:
        stream.write(f"session: {session_dir}\n")
        stream.write(f"bag: {bag_dir}\n")
        stream.write(f"pose_topic: {args.pose_topic}\n")
        stream.write(f"cloud_topic: {args.cloud_topic}\n")
        stream.write(f"used_clouds: {used_clouds}\n")
        stream.write(f"skipped_time: {skipped_time}\n")
        stream.write(f"skipped_pose: {skipped_pose}\n")
        stream.write(f"skipped_empty: {skipped_empty}\n")
        stream.write(f"raw_points: {len(xyz)}\n")
        stream.write(f"voxel_points: {len(voxel_xyz)}\n")
        stream.write(f"voxel_size_m: {args.voxel_size}\n")
        stream.write(f"min_forward_m: {args.min_forward_m}\n")
        stream.write(f"exclude_box: {exclude_box}\n")
        stream.write(f"raw_map: {raw_path if args.write_raw else '<not written>'}\n")
        stream.write(f"voxel_map: {clean_path}\n")
        stream.write(f"trajectory_csv: {out_dir / 'zed_pose_trajectory.csv'}\n")

    if args.write_raw:
        print(f"saved raw map: {raw_path}")
    print(f"saved voxel map: {clean_path}")
    print(f"saved trajectory: {out_dir / 'zed_pose_trajectory.csv'}")
    print(f"used_clouds={used_clouds} raw_points={len(xyz)} voxel_points={len(voxel_xyz)}")
    print(f"skipped_time={skipped_time} skipped_pose={skipped_pose} skipped_empty={skipped_empty}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
