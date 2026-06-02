#!/usr/bin/env python3
"""Audit one offline ICP run and write a machine-readable quality verdict."""

from __future__ import annotations

import argparse
import math
import re
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


def yaw_from_quaternion(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((pct / 100.0) * (len(ordered) - 1)))))
    return ordered[index]


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception as exc:  # noqa: BLE001
        return {"_load_error": str(exc)}


def load_bag_duration(session_dir: Path) -> float:
    metadata = load_yaml(session_dir / "bag" / "metadata.yaml")
    info = metadata.get("rosbag2_bagfile_information", metadata)
    return float(info.get("duration", {}).get("nanoseconds", 0)) / 1e9


def read_icp_odom(run_dir: Path) -> list[dict[str, float]]:
    if IMPORT_ERROR is not None:
        raise RuntimeError(f"rosbag2_py is not available: {IMPORT_ERROR}")
    bag_dir = run_dir / "icp_odom_replay"
    if not (bag_dir / "metadata.yaml").exists():
        return []

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if "/mapping/icp_odom" not in type_map:
        return []
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/mapping/icp_odom"]))
    msg_type = get_message(type_map["/mapping/icp_odom"])

    rows: list[dict[str, float]] = []
    while reader.has_next():
        _, data, timestamp_ns = reader.read_next()
        msg = deserialize_message(data, msg_type)
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if t <= 0.0:
            t = timestamp_ns / 1e9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rows.append(
            {
                "t": t,
                "x": float(p.x),
                "y": float(p.y),
                "z": float(p.z),
                "yaw": yaw_from_quaternion(q),
            }
        )
    return sorted(rows, key=lambda row: row["t"])


def scan_logs(run_dir: Path) -> dict[str, Any]:
    patterns = {
        "accepted": re.compile(r"\|\s*accepted=(\d+)\s+rejected=(\d+)"),
        "quality_reject": re.compile(r"Scan rejected by quality gate|rejected after ICP", re.IGNORECASE),
        "overlap_reject": re.compile(r"overlap", re.IGNORECASE),
        "convergence": re.compile(r"convergence failure|limit out of bounds", re.IGNORECASE),
        "z_jump": re.compile(r"z jump", re.IGNORECASE),
        "map_skip": re.compile(r"Skipping map insertion", re.IGNORECASE),
        "map_update": re.compile(r"Deterministic map update done|Initial map deterministically seeded", re.IGNORECASE),
        "map_trim": re.compile(r"Map trimmed", re.IGNORECASE),
    }
    counts = {key: 0 for key in patterns if key != "accepted"}
    accepted = rejected = 0
    for log_path in sorted((run_dir / "logs").rglob("*.log")):
        text = log_path.read_text(encoding="utf-8", errors="ignore")
        for line in text.splitlines():
            match = patterns["accepted"].search(line)
            if match:
                accepted = max(accepted, int(match.group(1)))
                rejected = max(rejected, int(match.group(2)))
            for key, pattern in patterns.items():
                if key != "accepted" and pattern.search(line):
                    counts[key] += 1
    counts["accepted"] = accepted
    counts["rejected"] = rejected
    counts["acceptance_ratio"] = accepted / max(accepted + rejected, 1)
    return counts


def evaluate(run_dir: Path, session_dir: Path, args: argparse.Namespace) -> dict[str, Any]:
    rows = read_icp_odom(run_dir)
    duration_s = load_bag_duration(session_dir)
    log_stats = scan_logs(run_dir)
    summary = load_yaml(run_dir / "summary.yaml")

    quality: dict[str, Any] = {
        "run_dir": str(run_dir),
        "session_dir": str(session_dir),
        "status": "FAIL",
        "reasons": [],
        "bag_duration_s": duration_s,
        "icp_msg_count": len(rows),
        "log_stats": log_stats,
        "offline_summary_status": summary.get("status"),
        "offline_summary_mode": summary.get("mode"),
        "map_size_bytes": summary.get("map_size_bytes"),
        "trajectory_size_bytes": summary.get("trajectory_size_bytes"),
    }

    if len(rows) < 2:
        quality["reasons"].append("too_few_icp_messages")
        return quality

    gaps = [b["t"] - a["t"] for a, b in zip(rows, rows[1:]) if b["t"] >= a["t"]]
    steps = [math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(rows, rows[1:])]
    z_steps = [abs(b["z"] - a["z"]) for a, b in zip(rows, rows[1:])]
    yaw_steps = [abs(wrap_angle(b["yaw"] - a["yaw"])) for a, b in zip(rows, rows[1:])]
    coverage = rows[-1]["t"] - rows[0]["t"]
    coverage_ratio = coverage / max(duration_s, 1.0)
    distance = sum(steps)

    quality.update(
        {
            "coverage_s": coverage,
            "coverage_ratio": coverage_ratio,
            "max_icp_gap_s": max(gaps) if gaps else 0.0,
            "p95_icp_gap_s": percentile(gaps, 95),
            "max_pose_step_m": max(steps) if steps else 0.0,
            "p99_pose_step_m": percentile(steps, 99),
            "max_z_step_m": max(z_steps) if z_steps else 0.0,
            "max_yaw_step_deg": math.degrees(max(yaw_steps) if yaw_steps else 0.0),
            "trajectory_length_m": distance,
            "map_file_exists": (run_dir / "map.vtk").exists(),
            "trajectory_file_exists": (run_dir / "trajectory.vtk").exists(),
            "map_freeze_detected": log_stats["map_update"] <= 1 and distance > args.min_motion_for_map_growth_m,
        }
    )

    if summary.get("status") != "ok":
        quality["reasons"].append(f"offline_icp_status:{summary.get('status')}")
    if coverage_ratio < args.min_coverage_ratio:
        quality["reasons"].append(f"low_coverage:{coverage_ratio:.3f}")
    if quality["max_icp_gap_s"] > args.max_gap_s:
        quality["reasons"].append(f"gap:{quality['max_icp_gap_s']:.3f}s")
    if (quality["p99_pose_step_m"] or 0.0) > args.max_p99_step_m:
        quality["reasons"].append(f"p99_step:{quality['p99_pose_step_m']:.3f}m")
    if quality["max_pose_step_m"] > args.max_step_m:
        quality["reasons"].append(f"max_step:{quality['max_pose_step_m']:.3f}m")
    if quality["max_z_step_m"] > args.max_z_step_m:
        quality["reasons"].append(f"z_step:{quality['max_z_step_m']:.3f}m")
    if log_stats["acceptance_ratio"] < args.min_acceptance_ratio:
        quality["reasons"].append(f"low_acceptance:{log_stats['acceptance_ratio']:.3f}")
    if quality["map_freeze_detected"]:
        quality["reasons"].append("map_freeze")
    if not quality["map_file_exists"] or not quality["trajectory_file_exists"]:
        quality["reasons"].append("missing_map_or_trajectory")

    quality["status"] = "PASS" if not quality["reasons"] else "FAIL"
    if quality["status"] == "FAIL" and args.manual_candidate_threshold == "relaxed":
        hard_reasons = {
            "missing_map_or_trajectory",
            "map_freeze",
            "too_few_icp_messages",
        }
        reason_prefixes = tuple(str(reason).split(":", 1)[0] for reason in quality["reasons"])
        has_hard_reason = any(reason in hard_reasons for reason in reason_prefixes)
        relaxed_ok = (
            not has_hard_reason
            and summary.get("status") == "ok"
            and quality["map_file_exists"]
            and quality["trajectory_file_exists"]
            and coverage_ratio >= args.manual_min_coverage_ratio
            and quality["max_icp_gap_s"] <= args.manual_max_gap_s
            and (quality["p99_pose_step_m"] or 0.0) <= args.manual_max_p99_step_m
            and quality["max_pose_step_m"] <= args.manual_max_step_m
            and quality["max_z_step_m"] <= args.max_z_step_m
            and log_stats["acceptance_ratio"] >= args.manual_min_acceptance_ratio
        )
        if relaxed_ok:
            quality["status"] = "MANUAL_CANDIDATE"
            quality["manual_candidate_reasons"] = list(quality["reasons"])
    quality["score"] = score_quality(quality)
    return quality


def score_quality(q: dict[str, Any]) -> float:
    score = 0.0
    score += 100.0 * min(float(q.get("coverage_ratio") or 0.0), 1.0)
    score += 20.0 * float(q.get("log_stats", {}).get("acceptance_ratio") or 0.0)
    score -= 10.0 * float(q.get("max_icp_gap_s") or 0.0)
    score -= 25.0 * float(q.get("p99_pose_step_m") or 0.0)
    score -= 5.0 * len(q.get("reasons") or [])
    return round(score, 3)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--session-dir", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--min-coverage-ratio", type=float, default=0.95)
    parser.add_argument("--min-acceptance-ratio", type=float, default=0.90)
    parser.add_argument("--max-gap-s", type=float, default=2.0)
    parser.add_argument("--max-p99-step-m", type=float, default=0.35)
    parser.add_argument("--max-step-m", type=float, default=1.0)
    parser.add_argument("--max-z-step-m", type=float, default=0.75)
    parser.add_argument("--min-motion-for-map-growth-m", type=float, default=5.0)
    parser.add_argument("--manual-candidate-threshold", choices=["off", "relaxed"], default="off")
    parser.add_argument("--manual-min-coverage-ratio", type=float, default=0.95)
    parser.add_argument("--manual-min-acceptance-ratio", type=float, default=0.65)
    parser.add_argument("--manual-max-gap-s", type=float, default=3.0)
    parser.add_argument("--manual-max-p99-step-m", type=float, default=0.55)
    parser.add_argument("--manual-max-step-m", type=float, default=1.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    session_dir = args.session_dir.resolve() if args.session_dir else run_dir.parents[1]
    quality = evaluate(run_dir, session_dir, args)
    output = args.output or run_dir / "quality.yaml"
    output.write_text(yaml.safe_dump(quality, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(quality, sort_keys=False))
    return 0 if quality["status"] in {"PASS", "MANUAL_CANDIDATE"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
