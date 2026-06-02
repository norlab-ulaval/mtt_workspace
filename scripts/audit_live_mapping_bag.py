#!/usr/bin/env python3
"""Audit live MTT mapping bags for odom/ICP scale and map inputs.

The report is intentionally narrow: it answers whether a too-short WILN route
comes from command/tachometer odometry or from ICP/map localization.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # pragma: no cover - this must run in the ROS image
    raise SystemExit(
        "This script must run in the ROS 2 workspace image: "
        "docker compose run --rm compile python3 scripts/audit_live_mapping_bag.py <session>"
    ) from exc


ODOM_TOPICS = {
    "mtt_odom": "/mtt_odometry",
    "icp_odom": "/mapping/icp_odom",
    "fallback_odom": "/mtt_monitor/cmd_fallback_odom",
    "zed_odom": "/zed/zed_node/odom",
}
TACH_TOPIC = "/mtt_tachometer"
CMD_TOPICS = ["/cmd_vel", "/cmd_vel/manual", "/cmd_vel/manual_raw", "/controller/cmd_vel"]
ARTICULATION_TOPIC = "/mtt/articulation_state"
ARTICULATION_ANGLE_TOPIC = "/mtt_articulation_angle"
HARDWARE_ARTICULATION_ANGLE_TOPIC = "/hardware/articulation_angle"
PATH_TOPIC = "/mapping/trajectory_path"
ALIGNED_SCAN_TOPIC = "/mapping/aligned_scan"
FILTERED_SCAN_TOPIC = "/mapping/scan_after_input_filters"
LIDAR_TOPIC = "/hesai_lidar/points"


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class ScalarSample:
    t: float
    value: float


def stamp_to_sec(msg: Any, bag_time_ns: int) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and (stamp.sec != 0 or stamp.nanosec != 0):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(bag_time_ns) * 1e-9


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
    )


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    idx = int(round((pct / 100.0) * (len(ordered) - 1)))
    idx = max(0, min(len(ordered) - 1, idx))
    return ordered[idx]


def path_stats(samples: list[PoseSample]) -> dict[str, Any]:
    if len(samples) < 2:
        return {
            "samples": len(samples),
            "duration_s": 0.0,
            "hz_mean": 0.0,
            "path_length_m": 0.0,
            "net_displacement_m": 0.0,
        }

    xy_steps = [
        math.hypot(b.x - a.x, b.y - a.y)
        for a, b in zip(samples, samples[1:])
    ]
    z_steps = [abs(b.z - a.z) for a, b in zip(samples, samples[1:])]
    yaw_steps = [
        abs(wrap_to_pi(b.yaw - a.yaw))
        for a, b in zip(samples, samples[1:])
    ]
    gaps = [
        max(0.0, b.t - a.t)
        for a, b in zip(samples, samples[1:])
    ]
    duration = max(samples[-1].t - samples[0].t, 1e-9)
    speeds = [step / max(gap, 1e-6) for step, gap in zip(xy_steps, gaps)]
    yaw_rates = [step / max(gap, 1e-6) for step, gap in zip(yaw_steps, gaps)]

    return {
        "samples": len(samples),
        "duration_s": duration,
        "hz_mean": (len(samples) - 1) / duration,
        "hz_median": 1.0 / statistics.median(gaps) if gaps and statistics.median(gaps) > 1e-9 else 0.0,
        "path_length_m": sum(xy_steps),
        "net_displacement_m": math.hypot(samples[-1].x - samples[0].x, samples[-1].y - samples[0].y),
        "start_xy": [samples[0].x, samples[0].y],
        "end_xy": [samples[-1].x, samples[-1].y],
        "bbox_xy": {
            "x_min": min(s.x for s in samples),
            "x_max": max(s.x for s in samples),
            "y_min": min(s.y for s in samples),
            "y_max": max(s.y for s in samples),
        },
        "max_gap_s": max(gaps) if gaps else 0.0,
        "p95_gap_s": percentile(gaps, 95.0),
        "max_xy_step_m": max(xy_steps) if xy_steps else 0.0,
        "p99_xy_step_m": percentile(xy_steps, 99.0),
        "max_z_step_m": max(z_steps) if z_steps else 0.0,
        "max_yaw_step_deg": math.degrees(max(yaw_steps)) if yaw_steps else 0.0,
        "max_speed_ms": max(speeds) if speeds else 0.0,
        "max_yaw_rate_deg_s": math.degrees(max(yaw_rates)) if yaw_rates else 0.0,
    }


def scalar_stats(samples: list[ScalarSample]) -> dict[str, Any]:
    if not samples:
        return {"samples": 0}
    values = [s.value for s in samples]
    return {
        "samples": len(samples),
        "min": min(values),
        "mean": sum(values) / len(values),
        "median": statistics.median(values),
        "p95": percentile(values, 95.0),
        "max": max(values),
    }


def vtk_path_stats(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"exists": False}
    points: list[PoseSample] = []
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    n_points = None
    start_index = 0
    for i, line in enumerate(lines):
        parts = line.split()
        if len(parts) >= 3 and parts[0].upper() == "POINTS":
            n_points = int(parts[1])
            start_index = i + 1
            break
    if n_points is None:
        return {"exists": True, "points": 0}
    for line in lines[start_index:]:
        parts = line.split()
        for idx in range(0, len(parts), 3):
            vals = parts[idx:idx + 3]
            if len(vals) != 3:
                continue
            try:
                x, y, z = (float(v) for v in vals)
            except ValueError:
                continue
            points.append(PoseSample(float(len(points)), x, y, z, 0.0))
            if len(points) >= n_points:
                break
        if len(points) >= n_points:
            break
    stats = path_stats(points)
    stats["exists"] = True
    stats["points"] = len(points)
    return stats


def resolve_bag_dir(path: Path) -> tuple[Path, Path]:
    path = path.expanduser().resolve()
    if (path / "bag" / "metadata.yaml").exists():
        return path, path / "bag"
    if (path / "metadata.yaml").exists():
        return path.parent, path
    raise SystemExit(f"Could not find bag/metadata.yaml under {path}")


def open_reader(bag_dir: Path) -> tuple[Any, dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap")
    converter = rosbag2_py.ConverterOptions("", "")
    reader.open(storage, converter)
    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }
    return reader, topic_types


def diagnose(report: dict[str, Any]) -> list[str]:
    findings: list[str] = []
    mtt_path = (report.get("odometry") or {}).get("mtt_odom", {}).get("path_length_m")
    icp_path = (report.get("odometry") or {}).get("icp_odom", {}).get("path_length_m")
    cmd_path = (report.get("commands") or {}).get("/cmd_vel", {}).get("integrated_abs_linear_m")
    tach_delta = (report.get("tachometer") or {}).get("distance_delta_m")

    if icp_path is not None and mtt_path is not None and mtt_path > 1.0:
        ratio = icp_path / max(mtt_path, 1e-9)
        if ratio < 0.6:
            findings.append(
                f"ICP path is much shorter than wheel odom ({icp_path:.2f} m vs {mtt_path:.2f} m, ratio={ratio:.2f}); likely ICP/map attraction or map-frame localization issue."
            )
        elif ratio > 1.6:
            findings.append(
                f"ICP path is much longer than wheel odom ({icp_path:.2f} m vs {mtt_path:.2f} m, ratio={ratio:.2f}); likely ICP jump/drift."
            )
    if cmd_path is not None and tach_delta is not None and cmd_path > 2.0:
        ratio = tach_delta / max(cmd_path, 1e-9)
        if ratio < 0.5:
            findings.append(
                f"Tachometer distance is much shorter than integrated /cmd_vel ({tach_delta:.2f} m vs {cmd_path:.2f} m, ratio={ratio:.2f}); check CAN tachometer scaling/units."
            )
    tach = report.get("tachometer") or {}
    fallback_abs = tach.get("fallback_speed_delta_abs_m")
    accepted_abs = tach.get("accepted_distance_delta_abs_m")
    if fallback_abs is not None and accepted_abs is not None and fallback_abs > 2.0 and accepted_abs < 0.5:
        findings.append(
            f"Tachometer cumulative distance is mostly stuck or implausible while speed is non-zero; fixed odom should integrate speed fallback ({fallback_abs:.2f} m abs) instead of accepted cumulative deltas ({accepted_abs:.2f} m abs)."
        )
    art = report.get("articulation_state") or {}
    if art.get("samples", 0) and art.get("hardware_fresh_ratio", 0.0) < 0.2:
        findings.append(
            "Articulation state mostly lacks fresh hardware angle; odometry may fall back to lidar/model articulation."
        )
    if art.get("state_lidar_used_ratio", 0.0) > 0.2:
        findings.append(
            "Odometry used lidar-derived articulation for a significant part of the run; this was unstable in previous bags."
        )
    icp = (report.get("odometry") or {}).get("icp_odom", {})
    if icp.get("hz_mean", 0.0) < 8.0:
        findings.append(
            f"ICP odom publish rate is low ({icp.get('hz_mean', 0.0):.2f} Hz); mapper is skipping/rejecting many lidar scans or running too slowly."
        )
    if not findings:
        findings.append("No dominant failure found from aggregate stats; inspect per-time plots/logs next.")
    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_or_bag", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    session_dir, bag_dir = resolve_bag_dir(args.session_or_bag)
    reader, topic_types = open_reader(bag_dir)

    selected_topics = set(ODOM_TOPICS.values())
    selected_topics.add(TACH_TOPIC)
    selected_topics.update(CMD_TOPICS)
    selected_topics.update({
        ARTICULATION_TOPIC,
        ARTICULATION_ANGLE_TOPIC,
        HARDWARE_ARTICULATION_ANGLE_TOPIC,
        PATH_TOPIC,
        ALIGNED_SCAN_TOPIC,
        FILTERED_SCAN_TOPIC,
        LIDAR_TOPIC,
    })

    msg_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in selected_topics
    }

    odom_samples: dict[str, list[PoseSample]] = {key: [] for key in ODOM_TOPICS}
    cmd_samples: dict[str, list[ScalarSample]] = {topic: [] for topic in CMD_TOPICS}
    articulation_angle: list[ScalarSample] = []
    hardware_articulation_angle: list[ScalarSample] = []
    counts: Counter[str] = Counter()
    tach: dict[str, Any] = {
        "samples": 0,
        "fresh": 0,
        "synthetic": 0,
        "times": [],
        "signed_speed": [],
        "speed_abs": [],
        "distance_m": [],
        "instant_ticks_per_s": [],
        "cumulative_ticks": [],
        "direction": Counter(),
    }
    art: dict[str, Any] = {
        "samples": 0,
        "hardware_fresh": 0,
        "lidar_detected": 0,
        "effective_source": Counter(),
        "hardware_lidar_residual_abs": [],
        "command_residual_abs": [],
        "state_lidar_used": 0,
    }

    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        counts[topic] += 1
        msg_type = msg_types.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(data, msg_type)
        t = stamp_to_sec(msg, bag_time_ns)

        for key, odom_topic in ODOM_TOPICS.items():
            if topic == odom_topic:
                pose = msg.pose.pose
                odom_samples[key].append(PoseSample(
                    t,
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                    yaw_from_quat(pose.orientation),
                ))
                break
        if topic == TACH_TOPIC:
            tach["samples"] += 1
            tach["fresh"] += int(bool(getattr(msg, "telemetry_fresh", False)))
            tach["synthetic"] += int(bool(getattr(msg, "tachometer_is_synthetic", False)))
            direction = str(getattr(msg, "direction", ""))
            speed_ms = float(getattr(msg, "speed_ms", 0.0))
            if speed_ms >= -1e-4:
                speed_ms *= -1.0 if direction == "Reverse" else 1.0
            tach["times"].append(t)
            tach["signed_speed"].append(speed_ms)
            tach["speed_abs"].append(abs(speed_ms))
            tach["distance_m"].append(float(getattr(msg, "distance_km", 0.0)) * 1000.0)
            tach["instant_ticks_per_s"].append(float(getattr(msg, "tachometer_instant", 0.0)))
            tach["cumulative_ticks"].append(float(getattr(msg, "tachometer_cumulative", 0.0)))
            tach["direction"][direction] += 1
        elif topic in CMD_TOPICS:
            cmd_samples[topic].append(ScalarSample(t, float(msg.twist.linear.x)))
        elif topic == ARTICULATION_TOPIC:
            art["samples"] += 1
            art["hardware_fresh"] += int(bool(getattr(msg, "hardware_fresh", False)))
            art["lidar_detected"] += int(bool(getattr(msg, "lidar_detected", False)))
            source = str(getattr(msg, "effective_source", ""))
            art["effective_source"][source] += 1
            if source == "state_lidar":
                art["state_lidar_used"] += 1
            if bool(getattr(msg, "hardware_fresh", False)) and bool(getattr(msg, "lidar_detected", False)):
                art["hardware_lidar_residual_abs"].append(abs(float(getattr(msg, "hardware_lidar_residual_rad", 0.0))))
            art["command_residual_abs"].append(abs(float(getattr(msg, "command_residual_rad", 0.0))))
        elif topic == ARTICULATION_ANGLE_TOPIC:
            articulation_angle.append(ScalarSample(t, float(msg.data)))
        elif topic == HARDWARE_ARTICULATION_ANGLE_TOPIC:
            hardware_articulation_angle.append(ScalarSample(t, float(msg.data)))

    command_report: dict[str, Any] = {}
    for topic, samples in cmd_samples.items():
        values = [s.value for s in samples]
        integrated = 0.0
        integrated_abs = 0.0
        for a, b in zip(samples, samples[1:]):
            dt = max(0.0, b.t - a.t)
            v = a.value
            integrated += v * dt
            integrated_abs += abs(v) * dt
        stats = scalar_stats(samples)
        stats.update({
            "integrated_linear_m": integrated,
            "integrated_abs_linear_m": integrated_abs,
            "nonzero_ratio": (
                sum(1 for v in values if abs(v) > 0.02) / len(values)
                if values else 0.0
            ),
        })
        command_report[topic] = stats

    distance_values = tach["distance_m"]
    tach_integrals = tachometer_integrals(
        tach["times"],
        tach["signed_speed"],
        tach["distance_m"],
    )
    tach_report = {
        "samples": tach["samples"],
        "fresh_ratio": tach["fresh"] / max(tach["samples"], 1),
        "synthetic_ratio": tach["synthetic"] / max(tach["samples"], 1),
        "speed_abs_ms": finite_stats_from_values(tach["speed_abs"]),
        "integrated_signed_speed_m": tach_integrals["integrated_signed_speed_m"],
        "integrated_abs_speed_m": tach_integrals["integrated_abs_speed_m"],
        "accepted_distance_delta_signed_m": tach_integrals["accepted_distance_delta_signed_m"],
        "accepted_distance_delta_abs_m": tach_integrals["accepted_distance_delta_abs_m"],
        "accepted_distance_delta_ratio": tach_integrals["accepted_distance_delta_ratio"],
        "fallback_speed_delta_signed_m": tach_integrals["fallback_speed_delta_signed_m"],
        "fallback_speed_delta_abs_m": tach_integrals["fallback_speed_delta_abs_m"],
        "distance_start_m": distance_values[0] if distance_values else None,
        "distance_end_m": distance_values[-1] if distance_values else None,
        "distance_delta_m": (distance_values[-1] - distance_values[0]) if len(distance_values) >= 2 else None,
        "instant_ticks_per_s": finite_stats_from_values(tach["instant_ticks_per_s"]),
        "cumulative_ticks_delta": (
            tach["cumulative_ticks"][-1] - tach["cumulative_ticks"][0]
            if len(tach["cumulative_ticks"]) >= 2 else None
        ),
        "direction_counts": dict(tach["direction"]),
    }

    art_report = {
        "samples": art["samples"],
        "hardware_fresh_ratio": art["hardware_fresh"] / max(art["samples"], 1),
        "lidar_detected_ratio": art["lidar_detected"] / max(art["samples"], 1),
        "state_lidar_used_ratio": art["state_lidar_used"] / max(art["samples"], 1),
        "effective_source_counts": dict(art["effective_source"]),
        "hardware_lidar_residual_abs_rad": finite_stats_from_values(art["hardware_lidar_residual_abs"]),
        "command_residual_abs_rad": finite_stats_from_values(art["command_residual_abs"]),
        "published_articulation_angle_rad": scalar_stats(articulation_angle),
        "hardware_articulation_angle_rad": scalar_stats(hardware_articulation_angle),
    }

    report: dict[str, Any] = {
        "session": session_dir.name,
        "bag_dir": str(bag_dir),
        "topic_counts": {topic: counts.get(topic, 0) for topic in sorted(selected_topics)},
        "odometry": {
            key: path_stats(samples)
            for key, samples in odom_samples.items()
        },
        "trajectory_vtk": vtk_path_stats(session_dir / "trajectory.vtk"),
        "commands": command_report,
        "tachometer": tach_report,
        "articulation_state": art_report,
    }
    report["diagnosis"] = diagnose(report)

    output = args.output or (session_dir / "live_mapping_audit.yaml")
    output.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(report, sort_keys=False))
    print(f"wrote: {output}", file=sys.stderr)
    return 0


def finite_stats_from_values(values: list[float]) -> dict[str, Any]:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return {"samples": 0}
    return {
        "samples": len(finite),
        "min": min(finite),
        "mean": sum(finite) / len(finite),
        "median": statistics.median(finite),
        "p95": percentile(finite, 95.0),
        "max": max(finite),
    }


def tachometer_delta_usable(raw_delta_m: float, signed_speed_ms: float, dt: float) -> bool:
    if not math.isfinite(raw_delta_m) or not math.isfinite(dt) or dt <= 1e-6:
        return False
    if raw_delta_m < 0.0:
        return False

    min_allowed_speed_ms = 8.0
    speed_margin_ms = 1.0
    abs_delta_m = abs(raw_delta_m)
    abs_speed_ms = abs(signed_speed_ms)
    moving_speed_threshold_ms = 0.05
    stuck_distance_epsilon_m = 1.0e-6
    if abs_speed_ms > moving_speed_threshold_ms and abs_delta_m <= stuck_distance_epsilon_m:
        return False

    implied_speed_ms = abs_delta_m / dt
    max_allowed_speed_ms = max(min_allowed_speed_ms, max(5.56 * 1.5, abs_speed_ms + speed_margin_ms))
    if implied_speed_ms > max_allowed_speed_ms:
        return False

    abs_delta_margin_m = 0.05
    relative_delta_margin = 1.5
    expected_delta_m = abs_speed_ms * dt
    delta_margin_m = max(abs_delta_margin_m, relative_delta_margin * expected_delta_m)
    return abs(abs_delta_m - expected_delta_m) <= delta_margin_m


def tachometer_integrals(
    times: list[float],
    signed_speed: list[float],
    distance_m: list[float],
) -> dict[str, Any]:
    integrated_signed_speed_m = 0.0
    integrated_abs_speed_m = 0.0
    accepted_signed_m = 0.0
    accepted_abs_m = 0.0
    fallback_signed_m = 0.0
    fallback_abs_m = 0.0
    accepted = 0
    candidates = 0

    for i in range(1, min(len(times), len(signed_speed), len(distance_m))):
        dt = times[i] - times[i - 1]
        if dt <= 0.0 or dt > 1.0:
            continue
        speed = signed_speed[i - 1]
        speed_step = speed * dt
        integrated_signed_speed_m += speed_step
        integrated_abs_speed_m += abs(speed_step)

        raw_delta = distance_m[i] - distance_m[i - 1]
        candidates += 1
        if tachometer_delta_usable(raw_delta, speed, dt):
            signed_delta = math.copysign(abs(raw_delta), speed) if abs(speed) > 1e-6 else raw_delta
            accepted_signed_m += signed_delta
            accepted_abs_m += abs(signed_delta)
            accepted += 1
        else:
            fallback_signed_m += speed_step
            fallback_abs_m += abs(speed_step)

    return {
        "integrated_signed_speed_m": integrated_signed_speed_m,
        "integrated_abs_speed_m": integrated_abs_speed_m,
        "accepted_distance_delta_signed_m": accepted_signed_m,
        "accepted_distance_delta_abs_m": accepted_abs_m,
        "accepted_distance_delta_ratio": accepted / max(candidates, 1),
        "fallback_speed_delta_signed_m": fallback_signed_m,
        "fallback_speed_delta_abs_m": fallback_abs_m,
    }


if __name__ == "__main__":
    raise SystemExit(main())
