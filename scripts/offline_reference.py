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
    "/mapping/icp_odom",
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
}

TOPIC_LABELS = {
    "/mapping/icp_odom": "recorded_icp",
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


def extract_sample(topic: str, msg: Any, bag_time_s: float) -> dict[str, Any]:
    if topic in {"/mapping/icp_odom", "/mtt_odometry"}:
        t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time_s
        return {
            "t": t,
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw": q_to_yaw(msg.pose.pose.orientation),
        }

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


def build_measurements(samples: dict[str, list[dict[str, Any]]]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    icp = Series(samples.get("/mapping/icp_odom", []))
    odom = Series(samples.get("/mtt_odometry", []))
    tacho = Series(samples.get("/mtt_tachometer", []))
    trailer_angle = Series(samples.get("/trailer/angle") or samples.get("/trailer/articulation_angle", []))
    mtt_articulation = Series(samples.get("/mtt_articulation_angle", []))
    trailer_pose = Series(samples.get("/trailer/pose", []))

    gps_topic = "/gps/fix"
    if not samples.get(gps_topic):
        if samples.get("/gps_left/fix"):
            gps_topic = "/gps_left/fix"
        elif samples.get("/gps_right/fix"):
            gps_topic = "/gps_right/fix"
        else:
            gps_topic = "/external/gps_llh"
    gps_rows = samples.get(gps_topic, [])
    gps = Series(gps_rows)
    gps_converter = gps_to_local_converter(gps_rows)

    reference_times = []
    for topic in ("/mapping/icp_odom", "/mtt_odometry", "/gps/fix", "/gps_left/fix", "/gps_right/fix", "/external/gps_llh"):
        reference_times.extend(float(row["t"]) for row in samples.get(topic, []))
    reference_times = sorted(set(round(t, 3) for t in reference_times))

    if len(reference_times) > 8000:
        step = max(1, len(reference_times) // 8000)
        reference_times = reference_times[::step]

    rows: list[dict[str, Any]] = []
    synthetic_count = 0
    cmd_sim_count = 0
    real_tacho_count = 0

    for t in reference_times:
        icp_row = icp.nearest(t, 0.20)
        odom_row = odom.nearest(t, 0.10)
        tacho_row = tacho.nearest(t, 0.10)
        angle_row = trailer_angle.nearest(t, 0.15) or mtt_articulation.nearest(t, 0.15)
        trailer_pose_row = trailer_pose.nearest(t, 0.15)
        gps_row = gps.nearest(t, 0.50)

        odom_is_synthetic = False
        if tacho_row:
            odom_is_synthetic = bool(tacho_row.get("synthetic")) or str(tacho_row.get("source")) == "cmd_sim"
            synthetic_count += int(bool(tacho_row.get("synthetic")))
            cmd_sim_count += int(str(tacho_row.get("source")) == "cmd_sim")
            real_tacho_count += int(str(tacho_row.get("source")) == "real")

        local_gps = None
        if gps_row and gps_converter and int(gps_row.get("status", -1)) >= 0:
            local_gps = gps_converter(gps_row)

        rows.append({
            "t": t,
            "has_icp": icp_row is not None,
            "icp_x": icp_row["x"] if icp_row else "",
            "icp_y": icp_row["y"] if icp_row else "",
            "icp_yaw": icp_row["yaw"] if icp_row else "",
            "has_odom": odom_row is not None,
            "odom_is_synthetic": odom_is_synthetic,
            "odom_x": odom_row["x"] if odom_row else "",
            "odom_y": odom_row["y"] if odom_row else "",
            "odom_yaw": odom_row["yaw"] if odom_row else "",
            "has_gps": local_gps is not None,
            "gps_x": local_gps[0] if local_gps else "",
            "gps_y": local_gps[1] if local_gps else "",
            "gps_z": local_gps[2] if local_gps else "",
            "has_trailer_angle": angle_row is not None,
            "trailer_angle": angle_row["angle"] if angle_row else "",
            "has_trailer_pose": trailer_pose_row is not None,
            "trailer_x": trailer_pose_row["x"] if trailer_pose_row else "",
            "trailer_y": trailer_pose_row["y"] if trailer_pose_row else "",
            "trailer_z": trailer_pose_row["z"] if trailer_pose_row else "",
        })

    stats = {
        "reference_times": len(reference_times),
        "synthetic_tachometer_samples": synthetic_count,
        "cmd_sim_tachometer_samples": cmd_sim_count,
        "real_tachometer_samples": real_tacho_count,
        "gps_topic_used": gps_topic if gps_rows else None,
    }
    return rows, stats


def build_measurements_from_postprocess_csv(csv_path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        source_rows = [dict(row) for row in csv.DictReader(stream)]

    rows: list[dict[str, Any]] = []
    synthetic_count = 0
    cmd_sim_count = 0
    real_tacho_count = 0
    icp_count = 0
    odom_count = 0
    trailer_angle_count = 0
    trailer_pose_count = 0

    for source in source_rows:
        t = csv_float(source.get("t"))
        if t is None:
            continue

        has_icp = csv_bool(source.get("has_icp"))
        icp_x = csv_float(source.get("icp_x"))
        icp_y = csv_float(source.get("icp_y"))
        icp_qx = csv_float(source.get("icp_qx"))
        icp_qy = csv_float(source.get("icp_qy"))
        icp_qz = csv_float(source.get("icp_qz"))
        icp_qw = csv_float(source.get("icp_qw"))
        if has_icp and None not in (icp_x, icp_y, icp_qx, icp_qy, icp_qz, icp_qw):
            assert icp_x is not None and icp_y is not None
            assert icp_qx is not None and icp_qy is not None and icp_qz is not None and icp_qw is not None
            icp_yaw: float | str = q_values_to_yaw(icp_qx, icp_qy, icp_qz, icp_qw)
            icp_count += 1
        else:
            has_icp = False
            icp_x = icp_y = icp_yaw = ""

        has_odom = csv_bool(source.get("has_odom"))
        odom_x = csv_float(source.get("odom_x"))
        odom_y = csv_float(source.get("odom_y"))
        odom_yaw = csv_float(source.get("odom_yaw"))
        if has_odom and None not in (odom_x, odom_y, odom_yaw):
            odom_count += 1
        else:
            has_odom = False
            odom_x = odom_y = odom_yaw = ""

        tach_source = str(source.get("tach_source") or "")
        tach_synthetic = csv_bool(source.get("tach_is_synthetic"))
        synthetic_count += int(tach_synthetic)
        cmd_sim_count += int(tach_source == "cmd_sim")
        real_tacho_count += int(tach_source == "real")
        odom_is_synthetic = tach_synthetic or tach_source == "cmd_sim"

        has_trailer_angle = csv_bool(source.get("has_trailer_angle"))
        trailer_angle = csv_float(source.get("trailer_articulation_angle"))
        if has_trailer_angle and trailer_angle is not None:
            trailer_angle_count += 1
        else:
            has_trailer_angle = False
            trailer_angle = ""

        has_trailer_pose = csv_bool(source.get("has_trailer_pose"))
        trailer_x = csv_float(source.get("trailer_pose_x"))
        trailer_y = csv_float(source.get("trailer_pose_y"))
        trailer_z = csv_float(source.get("trailer_pose_z"))
        if has_trailer_pose and None not in (trailer_x, trailer_y, trailer_z):
            trailer_pose_count += 1
        else:
            has_trailer_pose = False
            trailer_x = trailer_y = trailer_z = ""

        rows.append({
            "t": t,
            "has_icp": has_icp,
            "icp_x": icp_x,
            "icp_y": icp_y,
            "icp_yaw": icp_yaw,
            "has_odom": has_odom,
            "odom_is_synthetic": odom_is_synthetic,
            "odom_x": odom_x,
            "odom_y": odom_y,
            "odom_yaw": odom_yaw,
            "has_gps": False,
            "gps_x": "",
            "gps_y": "",
            "gps_z": "",
            "has_trailer_angle": has_trailer_angle,
            "trailer_angle": trailer_angle,
            "has_trailer_pose": has_trailer_pose,
            "trailer_x": trailer_x,
            "trailer_y": trailer_y,
            "trailer_z": trailer_z,
        })

    stats = {
        "reference_times": len(rows),
        "synthetic_tachometer_samples": synthetic_count,
        "cmd_sim_tachometer_samples": cmd_sim_count,
        "real_tachometer_samples": real_tacho_count,
        "gps_topic_used": None,
        "source": "postprocess_dataset/dataset.csv",
        "icp_samples": icp_count,
        "odom_samples": odom_count,
        "trailer_angle_samples": trailer_angle_count,
        "trailer_pose_samples": trailer_pose_count,
    }
    return rows, stats


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


def run_solver(args: argparse.Namespace, workspace_root: Path, measurements_csv: Path, output_csv: Path, summary_yaml: Path, log_path: Path) -> int:
    if args.solver:
        command = [args.solver]
    else:
        command = ["ros2", "run", "mtt_localization", "offline_reference_solver"]
    command += [
        "--input", str(measurements_csv),
        "--output", str(output_csv),
        "--summary", str(summary_yaml),
        "--icp-sigma-xy", str(args.icp_sigma_xy),
        "--icp-sigma-yaw", str(args.icp_sigma_yaw),
        "--odom-sigma-xy", str(args.odom_sigma_xy),
        "--odom-sigma-yaw", str(args.odom_sigma_yaw),
        "--gps-sigma-xy", str(args.gps_sigma_xy),
    ]
    return run_command(command, log_path, workspace_root)


def plot_reference(reference_csv: Path, plot_path: Path) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    xs, ys, qs = [], [], []
    with reference_csv.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            qs.append(float(row["source_quality"]))
    if not xs:
        return False

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    _, ax = plt.subplots(figsize=(8, 6))
    sc = ax.scatter(xs, ys, c=qs, s=4, cmap="viridis", vmin=0.0, vmax=1.0)
    ax.plot(xs, ys, linewidth=0.8, alpha=0.5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, linewidth=0.4, alpha=0.5)
    plt.colorbar(sc, ax=ax, label="quality")
    plt.tight_layout()
    plt.savefig(plot_path, dpi=140)
    plt.close()
    return True


def process_session(session_dir: Path, args: argparse.Namespace, workspace_root: Path) -> dict[str, Any]:
    bag_dir = session_dir / "bag"
    output_dir = session_dir / "offline_reference"
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
    has_recorded_icp = counts.get("/mapping/icp_odom", 0) > 0
    has_merged = counts.get("/merged_points_filtered", 0) > 0
    if has_two_lidars:
        result["notes"].append("two_lidars_available")
    if not has_recorded_icp:
        result["notes"].append("recorded_icp_missing_or_dead")
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
        postprocess_csv = session_dir / "postprocess_dataset" / "dataset.csv"
        if not postprocess_csv.exists():
            result["status"] = "skipped_missing_postprocess_dataset"
            result["notes"].append("postprocess_dataset_csv_missing")
            return result
        measurements, stats = build_measurements_from_postprocess_csv(postprocess_csv)
        result["skipped_topics"] = {}
        result["notes"].append("measurements_from_postprocess_dataset")
    else:
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
        measurements, stats = build_measurements(samples)
    result["measurement_stats"] = stats

    if not measurements:
        result["status"] = "skipped_no_measurements"
        return result

    measurements_csv = output_dir / "measurements.csv"
    reference_csv = output_dir / "reference_state.csv"
    solver_summary = output_dir / "solver_summary.yaml"
    write_csv(measurements, measurements_csv)

    code = run_solver(
        args=args,
        workspace_root=workspace_root,
        measurements_csv=measurements_csv,
        output_csv=reference_csv,
        summary_yaml=solver_summary,
        log_path=log_dir / "offline_reference_solver.log",
    )
    result["solver_returncode"] = code
    if code != 0:
        result["status"] = "solver_failed"
        return result

    result["status"] = "ok"
    result["measurements_csv"] = str(measurements_csv)
    result["reference_state_csv"] = str(reference_csv)
    result["solver_summary_yaml"] = str(solver_summary)
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
    parser.add_argument("--from-postprocess-csv", action="store_true", help="Use postprocess_dataset/dataset.csv as the solver input source.")
    parser.add_argument("--icp-sigma-xy", type=float, default=0.03)
    parser.add_argument("--icp-sigma-yaw", type=float, default=0.03)
    parser.add_argument("--odom-sigma-xy", type=float, default=0.30)
    parser.add_argument("--odom-sigma-yaw", type=float, default=0.25)
    parser.add_argument("--gps-sigma-xy", type=float, default=1.50)
    return parser.parse_args()


def main() -> int:
    workspace_root = infer_workspace_root(Path(__file__).resolve())
    args = parse_args(workspace_root)
    sessions = resolve_sessions(args.input_path)
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

        output_dir = session_dir / "offline_reference"
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
