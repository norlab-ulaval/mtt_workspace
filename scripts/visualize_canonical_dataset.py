#!/usr/bin/env python3
"""Generate visual research reports from canonical MTT mapping datasets."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path
from typing import Any

import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:  # pragma: no cover
    rosbag2_py = None
    deserialize_message = None
    get_message = None


def resolve_sessions(path_value: str) -> list[Path]:
    path = Path(path_value).expanduser().resolve()
    if (path / "bag" / "metadata.yaml").exists():
        return [path]
    if (path / "metadata.yaml").exists():
        return [path.parent]
    sessions = sorted(p.parent.parent for p in path.glob("*/bag/metadata.yaml"))
    if sessions:
        return sessions
    raise SystemExit(f"Could not resolve sessions from {path}")


def parse_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def parse_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as stream:
        return [dict(row) for row in csv.DictReader(stream)]


def read_vtk_points(path: Path, max_points: int = 200_000) -> list[tuple[float, float, float]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", errors="ignore") as stream:
        lines = stream.readlines()

    start = None
    count = 0
    for i, line in enumerate(lines):
        parts = line.split()
        if len(parts) >= 3 and parts[0] == "POINTS":
            start = i + 1
            count = int(parts[1])
            break
    if start is None or count <= 0:
        return []

    stride = max(1, math.ceil(count / max_points))
    points: list[tuple[float, float, float]] = []
    seen = 0
    for line in lines[start:]:
        parts = line.split()
        if len(parts) < 3:
            continue
        if seen % stride == 0:
            try:
                points.append((float(parts[0]), float(parts[1]), float(parts[2])))
            except ValueError:
                pass
        seen += 1
        if seen >= count:
            break
    return points


def trajectory_from_dataset(rows: list[dict[str, str]], prefix: str) -> list[tuple[float, float, float, float]]:
    out = []
    for row in rows:
        t = parse_float(row.get("t"))
        x = parse_float(row.get(f"{prefix}_x"))
        y = parse_float(row.get(f"{prefix}_y"))
        if prefix == "icp":
            yaw = parse_float(row.get("icp_yaw"))
            if yaw is None:
                qx = parse_float(row.get("icp_qx"))
                qy = parse_float(row.get("icp_qy"))
                qz = parse_float(row.get("icp_qz"))
                qw = parse_float(row.get("icp_qw"))
                if None not in (qx, qy, qz, qw):
                    assert qx is not None and qy is not None and qz is not None and qw is not None
                    yaw = yaw_from_quat_xyzw(qx, qy, qz, qw)
        else:
            yaw = parse_float(row.get(f"{prefix}_yaw"))
        has_key = f"has_{prefix}"
        if has_key in row and not parse_bool(row.get(has_key)):
            continue
        if None not in (t, x, y, yaw):
            assert t is not None and x is not None and y is not None and yaw is not None
            out.append((t, x, y, yaw))
    return out


def derive_speed_yaw(traj: list[tuple[float, float, float, float]]) -> list[tuple[float, float, float]]:
    out = []
    prev = None
    for sample in traj:
        t, x, y, yaw = sample
        v = 0.0
        wz = 0.0
        if prev is not None:
            pt, px, py, pyaw = prev
            dt = t - pt
            if dt > 1e-6:
                heading_mid = wrap_angle(0.5 * (yaw + pyaw))
                v = ((x - px) * math.cos(heading_mid) + (y - py) * math.sin(heading_mid)) / dt
                wz = wrap_angle(yaw - pyaw) / dt
        out.append((t, v, wz))
        prev = sample
    return out


def se2_align_to_reference(
    ref: list[tuple[float, float, float, float]],
    other: list[tuple[float, float, float, float]],
) -> list[tuple[float, float, float, float]]:
    if not ref or not other:
        return []
    _, rx0, ry0, ryaw0 = ref[0]
    _, ox0, oy0, oyaw0 = other[0]
    d_yaw = wrap_angle(ryaw0 - oyaw0)
    cd = math.cos(d_yaw)
    sd = math.sin(d_yaw)
    out = []
    for t, x, y, yaw in other:
        dx = x - ox0
        dy = y - oy0
        ax = rx0 + cd * dx - sd * dy
        ay = ry0 + sd * dx + cd * dy
        out.append((t, ax, ay, wrap_angle(yaw + d_yaw)))
    return out


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def read_optional_bag_trajectories(session_dir: Path) -> dict[str, list[tuple[float, float, float, float]]]:
    if rosbag2_py is None or deserialize_message is None or get_message is None:
        return {}
    bag_dir = session_dir / "bag"
    if not (bag_dir / "metadata.yaml").exists():
        return {}
    wanted = {"/zed/zed_node/odom"}
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        type_map = {item.name: item.type for item in reader.get_all_topics_and_types()}
        selected = sorted(wanted.intersection(type_map))
        if not selected:
            return {}
        msg_types = {topic: get_message(type_map[topic]) for topic in selected}
        reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
        out = {topic: [] for topic in selected}
        while reader.has_next():
            topic, raw, timestamp_ns = reader.read_next()
            msg = deserialize_message(raw, msg_types[topic])
            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            t = stamp_to_sec(stamp) if stamp and (stamp.sec or stamp.nanosec) else timestamp_ns / 1e9
            pose = msg.pose.pose
            yaw = yaw_from_quat_xyzw(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            )
            out[topic].append((t, float(pose.position.x), float(pose.position.y), yaw))
        return {topic: rows for topic, rows in out.items() if rows}
    except Exception:
        return {}


def stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "median": None, "max": None}
    return {
        "count": len(values),
        "mean": sum(values) / len(values),
        "median": statistics.median(values),
        "max": max(values),
    }


def process_session(session_dir: Path, max_map_points: int) -> dict[str, Any]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(f"matplotlib unavailable: {exc}") from exc

    dataset = read_csv(session_dir / "postprocess_dataset" / "dataset.csv")
    motion_dataset = read_csv(session_dir / "postprocess_dataset" / "motion_model_dataset.csv")
    canonical = session_dir / "offline_icp_canonical"
    map_points = read_vtk_points(canonical / "map.vtk", max_points=max_map_points)
    vtk_traj = read_vtk_points(canonical / "trajectory.vtk", max_points=1_000_000)

    icp_traj = trajectory_from_dataset(dataset, "icp")
    odom_traj = trajectory_from_dataset(dataset, "odom")
    odom_aligned = se2_align_to_reference(icp_traj, odom_traj)
    bag_extra = read_optional_bag_trajectories(session_dir)
    zed_aligned = se2_align_to_reference(icp_traj, bag_extra.get("/zed/zed_node/odom", []))

    report_dir = session_dir / "research_report"
    report_dir.mkdir(parents=True, exist_ok=True)
    plots: dict[str, str] = {}

    fig, ax = plt.subplots(figsize=(9, 8))
    if map_points:
        ax.scatter(
            [p[0] for p in map_points],
            [p[1] for p in map_points],
            s=0.08,
            c="0.70",
            alpha=0.45,
            linewidths=0,
            label="map",
        )
    if vtk_traj:
        ax.plot([p[0] for p in vtk_traj], [p[1] for p in vtk_traj], label="trajectory.vtk", linewidth=1.8)
    if icp_traj:
        ax.plot([p[1] for p in icp_traj], [p[2] for p in icp_traj], label="icp dataset", linewidth=1.2)
    if odom_aligned:
        ax.plot([p[1] for p in odom_aligned], [p[2] for p in odom_aligned], label="mtt_odometry aligned", linewidth=1.0)
    if zed_aligned:
        ax.plot([p[1] for p in zed_aligned], [p[2] for p in zed_aligned], label="zed odom aligned", linewidth=1.0)
    ax.set_title(session_dir.name)
    ax.set_xlabel("map x [m]")
    ax.set_ylabel("map y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    path = report_dir / "map_trajectory_overlay.png"
    fig.savefig(path, dpi=180)
    plots["map_trajectory_overlay"] = str(path)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    t0 = parse_float(dataset[0].get("t")) if dataset else None
    if dataset and t0 is not None:
        ts = [(parse_float(row.get("t")) or t0) - t0 for row in dataset]
        for key, label in (("cmd_linear_x", "cmd"), ("tach_speed_ms", "tach")):
            values = [parse_float(row.get(key)) for row in dataset]
            axes[0].plot(ts, [v if v is not None else math.nan for v in values], label=label, linewidth=0.8)
        for label, kin in (("icp", derive_speed_yaw(icp_traj)), ("odom", derive_speed_yaw(odom_traj))):
            if kin:
                kt0 = kin[0][0]
                axes[0].plot([k[0] - kt0 for k in kin], [k[1] for k in kin], label=f"{label} speed", linewidth=1.0)
                axes[1].plot([k[0] - kt0 for k in kin], [k[2] for k in kin], label=f"{label} yaw_rate", linewidth=1.0)
        axes[1].plot(ts, [parse_float(row.get("imu_angular_velocity_z")) or math.nan for row in dataset], label="imu wz", linewidth=0.8)
        axes[2].plot(ts, [parse_float(row.get("mtt_articulation_angle")) or math.nan for row in dataset], label="mtt articulation", linewidth=0.9)
        axes[2].plot(ts, [parse_float(row.get("trailer_articulation_angle")) or math.nan for row in dataset], label="trailer articulation", linewidth=0.9)
    axes[0].set_ylabel("speed [m/s]")
    axes[1].set_ylabel("yaw rate [rad/s]")
    axes[2].set_ylabel("angle [rad]")
    axes[2].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.tight_layout()
    path = report_dir / "signals_motion_inputs.png"
    fig.savefig(path, dpi=160)
    plots["signals_motion_inputs"] = str(path)
    plt.close(fig)

    if motion_dataset:
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        mt0 = parse_float(motion_dataset[0].get("t")) or 0.0
        mts = [(parse_float(row.get("t")) or mt0) - mt0 for row in motion_dataset]
        for key, label in (("v_icp_ms", "v icp"), ("v_model_ms", "v model"), ("v_target_model_ms", "v target")):
            axes[0].plot(mts, [parse_float(row.get(key)) or math.nan for row in motion_dataset], label=label, linewidth=0.9)
        for key, label in (("yaw_rate_icp_rad_s", "wz icp"), ("yaw_rate_model_rad_s", "wz model")):
            axes[1].plot(mts, [parse_float(row.get(key)) or math.nan for row in motion_dataset], label=label, linewidth=0.9)
        axes[0].set_ylabel("speed [m/s]")
        axes[1].set_ylabel("yaw rate [rad/s]")
        axes[1].set_xlabel("time [s]")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend()
        fig.tight_layout()
        path = report_dir / "motion_model_current_baseline.png"
        fig.savefig(path, dpi=160)
        plots["motion_model_current_baseline"] = str(path)
        plt.close(fig)

    icp_steps = [parse_float(row.get("icp_step_m")) for row in dataset]
    icp_steps_f = [v for v in icp_steps if v is not None]
    summary = {
        "session": session_dir.name,
        "dataset_rows": len(dataset),
        "motion_model_rows": len(motion_dataset),
        "map_points_sampled": len(map_points),
        "vtk_trajectory_points": len(vtk_traj),
        "icp_trajectory_points": len(icp_traj),
        "odom_trajectory_points": len(odom_traj),
        "odom_overlay_alignment": "first_pose_se2_to_icp",
        "zed_odom_points": len(bag_extra.get("/zed/zed_node/odom", [])),
        "icp_step_m": stats(icp_steps_f),
        "plots": plots,
        "notes": [
            "MTT odom and ZED/GPS trajectories are not in map by default; overlays use first-pose SE2 alignment to ICP for visual drift comparison.",
            "Use offline_icp_canonical as ground truth only when canonical_quality.status is PASS.",
        ],
    }
    (report_dir / "research_report.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False),
        encoding="utf-8",
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_path")
    parser.add_argument("--max-map-points", type=int, default=200_000)
    args = parser.parse_args()

    sessions = resolve_sessions(args.input_path)
    report = []
    for i, session in enumerate(sessions, start=1):
        print(f"[{i}/{len(sessions)}] {session.name}", flush=True)
        try:
            summary = process_session(session, args.max_map_points)
            print(f"  ok plots={len(summary.get('plots', {}))}", flush=True)
            report.append(summary)
        except Exception as exc:  # noqa: BLE001
            print(f"  failed: {exc}", flush=True)
            report.append({"session": session.name, "status": "failed", "error": str(exc)})

    root_report = Path(args.input_path).expanduser().resolve()
    if not (root_report / "bag" / "metadata.yaml").exists():
        out = root_report / "research_report_summary.yaml"
        out.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
        print(f"Report: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
