#!/usr/bin/env python3
"""Build a best-effort offline reference state for MTT bags.

Pipeline:
  1. Audit metadata and decide which sources are available.
  2. Optionally rebuild ICP through demos/bag_replay/scripts/offline_icp.py.
  3. Extract compact synchronized measurements from the bag.
  4. Run the C++ GTSAM batch smoother.
  5. Write plots and quality summaries.

The output is a reference estimate, not survey-grade ground truth.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import math
import os
import re
import subprocess
import sys
from bisect import bisect_left
from pathlib import Path
from typing import Any
from zipfile import ZipFile

import numpy as np
import pandas as pd
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


TOPICS = {
    "/mtt_odometry",
    "/mtt_tachometer",
    "/mtt_articulation_angle",
    "/trailer/angle",
    "/trailer/articulation_angle",
    "/trailer/pose",
    "/gps/fix",
    "/gps_left/fix",
    "/gps_right/fix",
    "/gps/heading",
    # Required by the offline_reference_solver rewrite (GTSAM ISAM2 batch
    # smoother, see mtt_localization/src/offline_reference_solver.cpp): the
    # solver's keyframes are at IMU rate, so IMU is no longer optional the
    # way it was for the old planar (icp_x,icp_y,icp_yaw) solver CLI.
    "/mti100/data",
    "/zed/zed_node/odom",
    "/isaac/vslam/odometry",
    "/mtt/articulation_state",
    "/hardware/articulation_angle",
    "/hardware/articulation_pitch_rad",
    "/tf_static",
}

# Read once, before the main filtered pass, to compute the fixed rotation
# from the IMU's own sensor frame into base_frame (see resolve_imu_rotation()
# below) — same convention as factor_graph_node.cpp's R_base_imu_.
TF_STATIC_TOPIC = "/tf_static"

TOPIC_LABELS = {
    "/mtt_odometry": "mtt_odometry",
    "/mtt_tachometer": "mtt_tachometer",
    "/mtt_articulation_angle": "mtt_articulation_angle",
    "/trailer/angle": "trailer_angle_alias",
    "/trailer/articulation_angle": "trailer_articulation_angle",
    "/trailer/pose": "trailer_pose",
    "/gps/fix": "gps_fix",
    "/gps_left/fix": "gps_left_fix",
    "/gps_right/fix": "gps_right_fix",
    "/gps/heading": "gps_heading",
    "/external/gps_llh": "external_gps_llh",
    "/mti100/data": "imu",
    "/zed/zed_node/odom": "zed_odom",
    "/isaac/vslam/odometry": "isaac_vslam_odom",
    "/mtt/articulation_state": "articulation_state",
}


GPS_CANDIDATE_DIRS = (
    Path("/data/GPS"),
    Path("/data/mtt_bags/GPS"),
    Path("data/GPS"),
)


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
    sessions = sorted(p for p in path.glob("*/bag/metadata.yaml"))
    if sessions:
        return [p.parent.parent for p in sessions]
    raise SystemExit(f"Could not resolve sessions from {path}")


def load_metadata(session_dir: Path) -> tuple[dict[str, int], float, int]:
    metadata_path = session_dir / "bag" / "metadata.yaml"
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


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def q_to_yaw(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def q_values_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def csv_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def csv_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


# ─── TF composition (same convention as scripts/extract_v2_measurements.py
# and factor_graph_node.cpp's R_base_imu_) — kept self-contained here since
# this script must remain independently runnable for arbitrary sessions. ──
def _quat_to_rotmat(x: float, y: float, z: float, w: float) -> list[list[float]]:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_transpose(a: list[list[float]]) -> list[list[float]]:
    return [[a[j][i] for j in range(3)] for i in range(3)]


def build_frame_graph(edges: list[tuple[str, str, float, float, float, float, float, float, float]]):
    """edges: (parent, child, x, y, z, qx, qy, qz, qw) -> {child: (parent, R, t)}"""
    graph = {}
    for parent, child, x, y, z, qx, qy, qz, qw in edges:
        graph[child] = (parent, _quat_to_rotmat(qx, qy, qz, qw), [x, y, z])
    return graph


def lookup_rotation(graph: dict, target: str, source: str) -> list[list[float]] | None:
    """Rotation-only R such that v_target = R @ v_source, composing static edges."""
    def path_to_root(frame: str) -> list[str]:
        chain = [frame]
        while chain[-1] in graph:
            chain.append(graph[chain[-1]][0])
        return chain

    if source not in graph and source != target:
        return None
    chain_s = path_to_root(source)
    chain_t = path_to_root(target)
    set_t = set(chain_t)
    common = next((f for f in chain_s if f in set_t), None)
    if common is None:
        return None

    r_sc = [[1.0, 0, 0], [0, 1, 0], [0, 0, 1]]
    frame = source
    while frame != common:
        _, r_pc, _ = graph[frame]
        r_sc = _mat_mul(r_pc, r_sc)
        frame = graph[frame][0]

    r_tc = [[1.0, 0, 0], [0, 1, 0], [0, 0, 1]]
    frame = target
    while frame != common:
        _, r_pc, _ = graph[frame]
        r_tc = _mat_mul(r_pc, r_tc)
        frame = graph[frame][0]

    return _mat_mul(_mat_transpose(r_tc), r_sc)


def rotate_vec(r: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = v
    return (
        r[0][0] * x + r[0][1] * y + r[0][2] * z,
        r[1][0] * x + r[1][1] * y + r[1][2] * z,
        r[2][0] * x + r[2][1] * y + r[2][2] * z,
    )


def extract_sample(topic: str, msg: Any, bag_time_s: float) -> dict[str, Any]:
    if topic == "/tf_static":
        return {
            "t": bag_time_s,
            "edges": [
                (
                    tr.header.frame_id, tr.child_frame_id,
                    float(tr.transform.translation.x), float(tr.transform.translation.y),
                    float(tr.transform.translation.z),
                    float(tr.transform.rotation.x), float(tr.transform.rotation.y),
                    float(tr.transform.rotation.z), float(tr.transform.rotation.w),
                )
                for tr in msg.transforms
            ],
        }

    if topic in {"/mapping/icp_odom", "/mtt_odometry", "/zed/zed_node/odom", "/isaac/vslam/odometry"}:
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        return {
            "t": t,
            "x": float(p.x), "y": float(p.y), "z": float(p.z),
            "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w),
            "yaw": q_to_yaw(q),
            "vx": float(v.x), "vy": float(v.y), "vz": float(v.z),
            "wx": float(w.x), "wy": float(w.y), "wz": float(w.z),
        }

    if topic == "/mti100/data":
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        a = msg.linear_acceleration
        g = msg.angular_velocity
        return {
            "t": t, "frame_id": msg.header.frame_id,
            "ax": float(a.x), "ay": float(a.y), "az": float(a.z),
            "gx": float(g.x), "gy": float(g.y), "gz": float(g.z),
        }

    if topic == "/mtt/articulation_state":
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        return {
            "t": t,
            "hardware_rad": float(msg.hardware_rad), "hardware_fresh": bool(msg.hardware_fresh),
            "lidar_rad": float(msg.lidar_rad), "lidar_detected": bool(msg.lidar_detected),
            "pitch_rad": float(msg.pitch_rad), "pitch_fresh": bool(msg.pitch_fresh),
        }

    if topic in {"/hardware/articulation_angle", "/hardware/articulation_pitch_rad"}:
        return {"t": bag_time_s, "value": float(msg.data)}

    if topic == "/mtt_tachometer":
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        return {
            "t": t,
            "source": str(getattr(msg, "tachometer_source", "")),
            "synthetic": bool(getattr(msg, "tachometer_is_synthetic", False)),
            "model_valid": bool(getattr(msg, "model_state_valid", False)),
            "speed_ms": float(getattr(msg, "speed_ms", 0.0)),
            "model_speed_ms": float(getattr(msg, "model_speed_ms", 0.0)),
            "direction": str(getattr(msg, "direction", "")),
        }

    if topic in {"/mtt_articulation_angle", "/trailer/angle", "/trailer/articulation_angle"}:
        return {"t": bag_time_s, "angle": float(msg.data)}

    if topic == "/trailer/pose":
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        return {
            "t": t,
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "z": float(msg.pose.position.z),
            "yaw": q_to_yaw(msg.pose.orientation),
        }

    if topic in {"/gps/fix", "/gps_left/fix", "/gps_right/fix"}:
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        return {
            "t": t,
            "lat": float(msg.latitude),
            "lon": float(msg.longitude),
            "alt": float(msg.altitude),
            "status": int(msg.status.status),
        }

    if topic == "/gps/heading":
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        return {"t": t, "yaw": q_to_yaw(msg.quaternion)}

    raise ValueError(topic)


def read_samples(bag_dir: Path) -> tuple[dict[str, list[dict[str, Any]]], dict[str, str]]:
    if IMPORT_ERROR is not None:
        raise SystemExit(f"rosbag2_py is not available: {IMPORT_ERROR}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = sorted(TOPICS.intersection(topic_types))
    if not selected:
        raise RuntimeError(f"no offline reference topics found in {bag_dir}")

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
        except Exception as exc:
            skipped[topic] = str(exc)
            samples.pop(topic, None)
            print(f"warning: skipping {topic}: {exc}", file=sys.stderr)

    for topic_rows in samples.values():
        topic_rows.sort(key=lambda row: float(row["t"]))
    return samples, skipped


def sample_time_bounds(samples: dict[str, list[dict[str, Any]]]) -> tuple[float | None, float | None]:
    times: list[float] = []
    for rows in samples.values():
        times.extend(float(row["t"]) for row in rows if "t" in row)
    if not times:
        return None, None
    return min(times), max(times)


def parse_llh_time(date_text: str, time_text: str) -> float:
    dt = datetime.strptime(f"{date_text} {time_text}", "%Y/%m/%d %H:%M:%S.%f")
    return dt.replace(tzinfo=timezone.utc).timestamp()


def parse_llh_line(line: str) -> dict[str, Any] | None:
    parts = line.split()
    if len(parts) < 6 or not re.match(r"^\d{4}/\d{2}/\d{2}$", parts[0]):
        return None
    try:
        return {
            "t": parse_llh_time(parts[0], parts[1]),
            "lat": float(parts[2]),
            "lon": float(parts[3]),
            "alt": float(parts[4]),
            "status": int(float(parts[5])),
            "satellites": int(float(parts[6])) if len(parts) > 6 else 0,
        }
    except ValueError:
        return None


def iter_llh_files(gps_dir: Path) -> list[tuple[Path, str | None]]:
    if not gps_dir.exists():
        return []
    direct = [(path, None) for path in sorted(gps_dir.glob("*.LLH"))]
    zipped: list[tuple[Path, str | None]] = []
    for zip_path in sorted(gps_dir.glob("*.zip")):
        try:
            with ZipFile(zip_path) as archive:
                for name in archive.namelist():
                    if name.upper().endswith(".LLH"):
                        zipped.append((zip_path, name))
        except Exception as exc:
            print(f"warning: cannot inspect GPS zip {zip_path}: {exc}", file=sys.stderr)
    return direct + zipped


def load_llh_rows(path: Path, member: str | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    try:
        if member:
            with ZipFile(path) as archive:
                with archive.open(member) as stream:
                    for raw in stream:
                        row = parse_llh_line(raw.decode("utf-8", errors="ignore"))
                        if row:
                            rows.append(row)
        else:
            with path.open("r", encoding="utf-8", errors="ignore") as stream:
                for line in stream:
                    row = parse_llh_line(line)
                    if row:
                        rows.append(row)
    except Exception as exc:
        print(f"warning: cannot read GPS LLH {path}: {exc}", file=sys.stderr)
    return rows


def load_external_gps_rows(
    gps_dir: Path | None,
    bag_start: float | None,
    bag_end: float | None,
    output_dir: Path,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if gps_dir is None or bag_start is None or bag_end is None:
        return [], {"gps_dir": str(gps_dir) if gps_dir else None, "reason": "missing_dir_or_bag_time"}

    margin_s = 30.0
    selected: list[dict[str, Any]] = []
    candidates: list[dict[str, Any]] = []

    for path, member in iter_llh_files(gps_dir):
        rows = load_llh_rows(path, member)
        if not rows:
            continue
        start = float(rows[0]["t"])
        end = float(rows[-1]["t"])
        overlap = max(0.0, min(end, bag_end + margin_s) - max(start, bag_start - margin_s))
        candidate = {
            "path": str(path),
            "member": member,
            "start": start,
            "end": end,
            "samples": len(rows),
            "overlap_s": overlap,
        }
        candidates.append(candidate)
        if overlap > 0.0:
            for row in rows:
                t = float(row["t"])
                if bag_start - margin_s <= t <= bag_end + margin_s:
                    row = dict(row)
                    row["source_file"] = path.name if member is None else f"{path.name}:{member}"
                    selected.append(row)

    selected.sort(key=lambda row: float(row["t"]))
    if selected:
        write_csv(selected, output_dir / "external_gps_llh.csv")
    (output_dir / "external_gps_candidates.yaml").write_text(
        yaml.safe_dump(candidates, sort_keys=False),
        encoding="utf-8",
    )
    return selected, {"gps_dir": str(gps_dir), "candidate_files": len(candidates), "selected_samples": len(selected)}


class Series:
    def __init__(self, rows: list[dict[str, Any]]):
        self.rows = sorted(rows, key=lambda row: float(row["t"]))
        self.times = [float(row["t"]) for row in self.rows]

    def nearest(self, t: float, tol: float) -> dict[str, Any] | None:
        if not self.rows:
            return None
        idx = bisect_left(self.times, t)
        candidates = []
        if idx < len(self.rows):
            candidates.append(self.rows[idx])
        if idx:
            candidates.append(self.rows[idx - 1])
        best = min(candidates, key=lambda row: abs(float(row["t"]) - t))
        return best if abs(float(best["t"]) - t) <= tol else None


def gps_to_local_converter(gps_rows: list[dict[str, Any]]):
    valid = [row for row in gps_rows if int(row.get("status", -1)) >= 0]
    if not valid:
        return None
    origin = valid[0]
    earth_radius = 6378137.0
    deg2rad = math.pi / 180.0
    lat0 = float(origin["lat"])
    lon0 = float(origin["lon"])
    alt0 = float(origin["alt"])
    m_per_deg_lat = earth_radius * deg2rad
    m_per_deg_lon = earth_radius * deg2rad * math.cos(lat0 * deg2rad)

    def convert(row: dict[str, Any]) -> tuple[float, float, float]:
        # x north, y east. This is consistent enough for local factor constraints.
        return (
            (float(row["lat"]) - lat0) * m_per_deg_lat,
            (float(row["lon"]) - lon0) * m_per_deg_lon,
            float(row["alt"]) - alt0,
        )

    return convert


def resolve_imu_rotation(samples: dict[str, list[dict[str, Any]]], imu_frame: str, base_frame: str = "base_footprint"):
    """Compose /tf_static into a fixed rotation base_frame <- imu_frame, same
    convention as factor_graph_node.cpp's R_base_imu_ (TF lookup at startup)
    and scripts/extract_v2_measurements.py. Falls back to 'base_link' and, if
    tf_static has no path between the two frames, to identity (logged)."""
    tf_rows = samples.get("/tf_static", [])
    edges: list[tuple] = []
    for row in tf_rows:
        edges.extend(row.get("edges", []))
    if not edges:
        print("warning: no /tf_static in this bag — IMU used unrotated (identity)", file=sys.stderr)
        return None
    graph = build_frame_graph(edges)
    frames = set(graph.keys()) | {e[0] for e in edges}
    frame = base_frame if base_frame in frames else ("base_link" if "base_link" in frames else base_frame)
    r = lookup_rotation(graph, frame, imu_frame)
    if r is None:
        print(f"warning: no TF path {frame} <- {imu_frame} — IMU used unrotated (identity)", file=sys.stderr)
        return None
    return r


def load_gt_icp_csv(path: Path) -> list[dict[str, Any]]:
    """Read a GT_icp/icp_odom_*.csv (written by
    norlab_ws/src/icp_odom_logger/icp_odom_logger_node.py during an offline
    mapper rebuild), converting its split sec/nanosec timestamp to the same
    row shape as extract_sample()'s /mapping/icp_odom rows."""
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        for r in csv.DictReader(f):
            rows.append({
                "t": float(r["timestamp_sec"]) + float(r["timestamp_nanosec"]) * 1e-9,
                "x": float(r["x"]), "y": float(r["y"]), "z": float(r["z"]),
                "qx": float(r["qx"]), "qy": float(r["qy"]), "qz": float(r["qz"]), "qw": float(r["qw"]),
                "vx": float(r["vx"]), "vy": float(r["vy"]), "vz": float(r["vz"]),
                "wx": float(r["wx"]), "wy": float(r["wy"]), "wz": float(r["wz"]),
            })
    rows.sort(key=lambda row: row["t"])
    return rows


def validate_offline_icp_csv(path: Path) -> None:
    """Refuse a structurally broken file. Never judges whether the ICP result
    is scientifically good -- that judgment already happened (--icp-approved-by
    records who made it), this only catches a wrong/corrupt/empty file."""
    required_columns = {
        "timestamp_sec", "timestamp_nanosec", "x", "y", "z",
        "qx", "qy", "qz", "qw",
    }
    df = pd.read_csv(path, nrows=0)
    missing = required_columns - set(df.columns)
    if missing:
        raise ValueError(
            f"{path}: missing required columns {sorted(missing)} -- this does not "
            "look like a GT_icp/icp_odom_*.csv, refusing to use it as an offline "
            "ICP reference.")
    full = pd.read_csv(path)
    if full.empty:
        raise ValueError(f"{path}: has a header but zero data rows.")
    t = full["timestamp_sec"].to_numpy(dtype=float) + full["timestamp_nanosec"].to_numpy(dtype=float) * 1e-9
    if not np.all(np.isfinite(t)):
        raise ValueError(f"{path}: non-finite timestamps present.")
    if not np.all(np.diff(t) > 0):
        raise ValueError(f"{path}: timestamps are not strictly increasing.")
    pose_cols = ["x", "y", "z", "qx", "qy", "qz", "qw"]
    if not np.all(np.isfinite(full[pose_cols].to_numpy(dtype=float))):
        raise ValueError(f"{path}: non-finite pose values present.")


def build_source_csvs(samples: dict[str, list[dict[str, Any]]], output_dir: Path,
                       offline_icp_path: Path, icp_approved_by: str) -> dict[str, Any]:
    """Write the per-source CSVs the rewritten offline_reference_solver expects
    (imu/icp/artic/track_odom/zed_odom), replacing the old single fused
    (icp_x,icp_y,icp_yaw,...) CSV the previous planar solver consumed."""
    output_dir.mkdir(parents=True, exist_ok=True)
    stats: dict[str, Any] = {}

    imu_rows = samples.get("/mti100/data", [])
    stats["imu_samples"] = len(imu_rows)
    if imu_rows:
        imu_frame = next((r["frame_id"] for r in imu_rows if r.get("frame_id")), "imu_link")
        rot = resolve_imu_rotation(samples, imu_frame)
        with (output_dir / "imu.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "ax", "ay", "az", "gx", "gy", "gz"])
            for r in imu_rows:
                a = (r["ax"], r["ay"], r["az"])
                g = (r["gx"], r["gy"], r["gz"])
                if rot is not None:
                    a = rotate_vec(rot, a)
                    g = rotate_vec(rot, g)
                w.writerow([r["t"], *a, *g])

    if offline_icp_path is None:
        raise ValueError(
            "--offline-icp is required: this script only uses explicitly-approved offline "
            "ICP CSVs, never live /mapping/icp_odom. Run the offline_icp_mapper to produce "
            "a qualified GT_icp/icp_odom_*.csv, then pass it via --offline-icp <path> "
            "--icp-approved-by <name>.")
    icp_rows = load_gt_icp_csv(offline_icp_path)
    stats["icp_source"] = str(offline_icp_path)
    stats["icp_approved_by"] = icp_approved_by
    stats["icp_samples"] = len(icp_rows)
    with (output_dir / "icp.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz", "wx", "wy", "wz"])
        for r in icp_rows:
            w.writerow([r["t"], r["x"], r["y"], r["z"], r["qx"], r["qy"], r["qz"], r["qw"],
                        r["vx"], r["vy"], r["vz"], r["wx"], r["wy"], r["wz"]])

    artic_rows = samples.get("/mtt/articulation_state", [])
    hw_rows = Series(samples.get("/hardware/articulation_angle", []))
    pitch_rows = Series(samples.get("/hardware/articulation_pitch_rad", []))
    stats["articulation_state_samples"] = len(artic_rows)
    stats["hardware_articulation_angle_samples"] = len(hw_rows.rows)
    with (output_dir / "artic.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "hardware_rad", "hardware_fresh", "lidar_rad", "lidar_detected", "pitch_rad", "pitch_fresh"])
        if artic_rows:
            for r in artic_rows:
                w.writerow([r["t"], r["hardware_rad"], int(r["hardware_fresh"]),
                            r["lidar_rad"], int(r["lidar_detected"]), r["pitch_rad"], int(r["pitch_fresh"])])
        elif hw_rows.rows:
            # Fallback for bags without /mtt/articulation_state (older recordings):
            # hardware encoder only, no LiDAR-fused hitch angle, no pitch.
            for r in hw_rows.rows:
                w.writerow([r["t"], r["value"], 1, 0.0, 0, 0.0, 0])

    track_rows = samples.get("/mtt_odometry", [])
    stats["track_odom_samples"] = len(track_rows)
    with (output_dir / "track_odom.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for r in track_rows:
            w.writerow([r["t"], r["x"], r["y"], r["z"], r["qx"], r["qy"], r["qz"], r["qw"]])

    zed_rows = samples.get("/zed/zed_node/odom", [])
    stats["zed_odom_samples"] = len(zed_rows)
    with (output_dir / "zed_odom.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for r in zed_rows:
            w.writerow([r["t"], r["x"], r["y"], r["z"], r["qx"], r["qy"], r["qz"], r["qw"]])

    isaac_vslam_rows = samples.get("/isaac/vslam/odometry", [])
    stats["isaac_vslam_samples"] = len(isaac_vslam_rows)
    with (output_dir / "isaac_vslam.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for r in isaac_vslam_rows:
            w.writerow([r["t"], r["x"], r["y"], r["z"], r["qx"], r["qy"], r["qz"], r["qw"]])

    return stats


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def run_command(command: list[str], log_path: Path, cwd: Path) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.run(command, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, text=True, check=False)
    return process.returncode


def run_solver(args: argparse.Namespace, workspace_root: Path, source_dir: Path, output_dir: Path, log_path: Path) -> int:
    if args.solver:
        command = [args.solver]
    else:
        command = ["ros2", "run", "mtt_localization", "offline_reference_solver"]
    output_dir.mkdir(parents=True, exist_ok=True)
    command += ["--imu", str(source_dir / "imu.csv"), "--icp", str(source_dir / "icp.csv"),
                "--output-dir", str(output_dir)]
    if (source_dir / "artic.csv").exists():
        command += ["--artic", str(source_dir / "artic.csv")]
    if (source_dir / "track_odom.csv").exists():
        command += ["--track-odom", str(source_dir / "track_odom.csv")]
    if (source_dir / "zed_odom.csv").exists():
        command += ["--zed-odom", str(source_dir / "zed_odom.csv")]
    if (source_dir / "isaac_vslam.csv").exists():
        command += ["--isaac-vslam", str(source_dir / "isaac_vslam.csv")]
    return run_command(command, log_path, workspace_root)


def plot_reference(reference_csv: Path, plot_path: Path) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    xs, ys, sigmas = [], [], []
    with reference_csv.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            sx = csv_float(row.get("sigma_x"))
            sy = csv_float(row.get("sigma_y"))
            sigmas.append(math.hypot(sx, sy) if sx is not None and sy is not None else float("nan"))
    if not xs:
        return False

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    _, ax = plt.subplots(figsize=(8, 6))
    sc = ax.scatter(xs, ys, c=sigmas, s=4, cmap="viridis")
    ax.plot(xs, ys, linewidth=0.8, alpha=0.5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, linewidth=0.4, alpha=0.5)
    plt.colorbar(sc, ax=ax, label="position sigma [m] (NaN where marginals not computed at this stride)")
    plt.tight_layout()
    plt.savefig(plot_path, dpi=140)
    plt.close()
    return True


def process_session(session_dir: Path, args: argparse.Namespace, workspace_root: Path) -> dict[str, Any]:
    bag_dir = session_dir / "bag"
    output_dir = args.output_dir if args.output_dir is not None else session_dir / "offline_reference"
    log_dir = output_dir / "logs"
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    counts, duration_s, total_messages = load_metadata(session_dir)
    result: dict[str, Any] = {
        "session": session_dir.name,
        "session_dir": str(session_dir),
        "bag_dir": str(bag_dir),
        "duration_s": duration_s,
        "total_messages": total_messages,
        "status": "failed",
        "topic_counts": {label: counts.get(topic, 0) for topic, label in TOPIC_LABELS.items()},
        "notes": [],
    }

    if not counts:
        result["status"] = "skipped_missing_metadata"
        return result

    has_two_lidars = counts.get("/hesai_lidar/points", 0) > 0 and counts.get("/rsairy_ns/points", 0) > 0
    has_merged = counts.get("/merged_points_filtered", 0) > 0
    if has_two_lidars:
        result["notes"].append("two_lidars_available")
    if has_two_lidars and not has_merged:
        result["notes"].append("merged_cloud_missing_but_can_be_rebuilt")

    if args.run_icp:
        offline_icp = workspace_root / "demos" / "bag_replay" / "scripts" / "offline_icp.py"
        command = [sys.executable, str(offline_icp), str(session_dir)]
        if args.force_icp:
            command.append("--force")
        code = run_command(command, log_dir / "offline_icp.log", workspace_root)
        result["offline_icp_returncode"] = code
        if code != 0:
            result["notes"].append("offline_icp_failed")

    if args.from_postprocess_csv:
        # The rewritten offline_reference_solver keyframes at IMU rate and
        # requires raw accel/gyro (see mtt_localization/src/
        # offline_reference_solver.cpp) — postprocess_dataset/dataset.csv is a
        # planar (x,y,yaw) fused CSV with no raw IMU samples, so it cannot
        # feed the new solver. Rather than silently degrade or fabricate IMU
        # data, this mode is explicitly unsupported until a dataset with raw
        # IMU is available; use the direct bag-reading path instead.
        result["status"] = "skipped_postprocess_csv_incompatible_with_new_solver"
        result["notes"].append(
            "offline_reference_solver now requires raw IMU (accel/gyro) at its own rate; "
            "postprocess_dataset/dataset.csv has no IMU columns. Re-run without --from-postprocess-csv."
        )
        return result

    samples, skipped = read_samples(bag_dir)
    bag_start, bag_end = sample_time_bounds(samples)
    gps_dir = Path(args.gps_log_dir).expanduser() if args.gps_log_dir else next(
        (path if path.is_absolute() else workspace_root / path for path in GPS_CANDIDATE_DIRS if (path if path.is_absolute() else workspace_root / path).exists()),
        None,
    )
    if not args.no_external_gps:
        external_rows, external_stats = load_external_gps_rows(gps_dir, bag_start, bag_end, output_dir)
        if external_rows:
            samples["/external/gps_llh"] = external_rows
            result["notes"].append("external_gps_llh_matched")
        result["external_gps"] = external_stats
    result["skipped_topics"] = skipped

    if not samples.get("/mti100/data"):
        result["status"] = "skipped_no_imu"
        result["notes"].append("no /mti100/data in this bag — offline_reference_solver requires IMU keyframes")
        return result
    if args.offline_icp is None:
        result["status"] = "skipped_no_offline_icp"
        result["notes"].append("--offline-icp is required; see --help for usage")
        return result

    source_dir = output_dir / "measurements"
    stats = build_source_csvs(samples, source_dir, args.offline_icp, args.icp_approved_by)
    result["measurement_stats"] = stats

    graph_dir = output_dir / "graph"
    code = run_solver(
        args=args,
        workspace_root=workspace_root,
        source_dir=source_dir,
        output_dir=graph_dir,
        log_path=log_dir / "offline_reference_solver.log",
    )
    result["solver_returncode"] = code
    if code != 0:
        result["status"] = "solver_failed"
        return result

    reference_csv = graph_dir / "optimized_trajectory.csv"
    result["status"] = "ok"
    result["source_csv_dir"] = str(source_dir)
    result["reference_state_csv"] = str(reference_csv)
    result["solver_summary_yaml"] = str(graph_dir / "solver_summary.yaml")
    result["trajectory_plot"] = str(output_dir / "trajectory_xy.png")
    result["plot_written"] = plot_reference(reference_csv, output_dir / "trajectory_xy.png")
    return result


def parse_args(workspace_root: Path) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build offline best-reference state for MTT bags.")
    parser.add_argument("input_path", nargs="?", default=str(workspace_root / "data"))
    parser.add_argument("--run-icp", action="store_true", help="Run the existing offline ICP rebuild before smoothing.")
    parser.add_argument("--force-icp", action="store_true", help="Force offline ICP rebuild when --run-icp is set.")
    parser.add_argument("--solver", default="", help="Path to offline_reference_solver; default uses ros2 run.")
    parser.add_argument("--gps-log-dir", default="", help="Directory containing RSplus .LLH files or ZIP exports.")
    parser.add_argument("--no-external-gps", action="store_true", help="Ignore external RSplus .LLH GPS logs.")
    parser.add_argument("--from-postprocess-csv", action="store_true",
                         help="Deprecated/unsupported: postprocess_dataset/dataset.csv has no raw IMU, "
                              "which the rewritten solver requires. Kept only to produce a clear error.")
    parser.add_argument("--output-dir", type=Path, default=None,
                         help="Override the per-session output directory (default: "
                             "<session_dir>/offline_reference). Only valid together with "
                             "--offline-icp / a single resolved session -- see Step 6's "
                             "single-session enforcement below; using this in a multi-session "
                             "sweep would make every session collide on the same directory.")
    parser.add_argument("--offline-icp", type=Path, default=None,
                         help="Path to the qualified offline ICP CSV (GT_icp/icp_odom_*.csv "
                             "format). REQUIRED for any session actually processed -- there "
                             "is no live-ICP fallback and no automatic discovery. See the "
                             "absolute ICP rule in documentations/paper_results.md.")
    parser.add_argument("--icp-approved-by", default=None,
                         help="Name of the person who visually qualified --offline-icp "
                             "(e.g. 'mohamed'). Required together with --offline-icp; this "
                             "is the qualification record, not something this script "
                             "computes or infers.")
    args = parser.parse_args()
    if bool(args.offline_icp) != bool(args.icp_approved_by):
        parser.error("--offline-icp and --icp-approved-by must be supplied together.")
    if args.offline_icp is not None and not args.offline_icp.is_file():
        parser.error(f"--offline-icp path does not exist or is not a file: {args.offline_icp}")
    if args.offline_icp is not None:
        try:
            validate_offline_icp_csv(args.offline_icp)
        except ValueError as exc:
            parser.error(str(exc))
    return args


def main() -> int:
    workspace_root = infer_workspace_root(Path(__file__).resolve())
    args = parse_args(workspace_root)
    sessions = resolve_sessions(args.input_path)
    if args.offline_icp is not None and len(sessions) > 1:
        print(f"error: --offline-icp supplied but {len(sessions)} sessions would be processed. "
              "When using an offline ICP reference, process one session at a time (pass the "
              "exact session directory or a single bag). This enforces provenance tracking.")
        return 1
    if args.output_dir is not None and len(sessions) > 1:
        print(f"error: --output-dir supplied but {len(sessions)} sessions would be processed. "
              "Every session would collide on the same output directory. Process one session "
              "at a time when overriding --output-dir.")
        return 1
    failures = 0
    report = []

    for index, session_dir in enumerate(sessions, start=1):
        print(f"[{index}/{len(sessions)}] {session_dir.name}")
        try:
            result = process_session(session_dir, args, workspace_root)
        except Exception as exc:
            result = {
                "session": session_dir.name,
                "session_dir": str(session_dir),
                "status": "failed_exception",
                "error": str(exc),
            }
        report.append(result)

        output_dir = args.output_dir if args.output_dir is not None else session_dir / "offline_reference"
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "summary.yaml").write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
        print(f"  {result['status']}")
        if result["status"] != "ok" and not str(result["status"]).startswith("skipped"):
            failures += 1

    report_path = workspace_root / "data" / "offline_reference_report.yaml"
    report_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    print(f"Report: {report_path}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
