#!/usr/bin/env python3
"""Validate a WILN .ltr route before replaying it on the MTT."""

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


@dataclass
class Pose2D:
    x: float
    y: float
    z: float
    yaw: float


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        qw * qw + qx * qx - qy * qy - qz * qz,
    )


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def read_ltr(path: Path) -> Tuple[str, List[List[Pose2D]]]:
    frame_id = ""
    segments: List[List[Pose2D]] = []
    in_trajectory = False

    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("# vtk"):
                raise ValueError(f"{path} looks like a VTK map, not a WILN .ltr trajectory")
            if line.startswith("#############################"):
                in_trajectory = True
                continue
            if not in_trajectory:
                continue
            if line.startswith("frame_id"):
                frame_id = line.split(":", 1)[1].strip()
                continue
            if line == "changing direction":
                segments.append([])
                continue

            fields = line.split(",")
            if len(fields) != 7:
                raise ValueError(f"invalid pose line in {path}: {line}")
            x, y, z, qx, qy, qz, qw = [float(value) for value in fields]
            if not segments:
                segments.append([])
            segments[-1].append(Pose2D(x=x, y=y, z=z, yaw=yaw_from_quaternion(qx, qy, qz, qw)))

    return frame_id, [segment for segment in segments if segment]


def route_stats(segments: List[List[Pose2D]]) -> dict:
    pose_count = sum(len(segment) for segment in segments)
    steps = []
    yaw_steps = []
    z_values = []

    for segment in segments:
        z_values.extend(pose.z for pose in segment)
        for previous, current in zip(segment, segment[1:]):
            steps.append(math.hypot(current.x - previous.x, current.y - previous.y))
            yaw_steps.append(abs(wrap_to_pi(current.yaw - previous.yaw)))

    path_length = sum(steps)
    max_step = max(steps) if steps else 0.0
    mean_step = path_length / len(steps) if steps else 0.0
    max_yaw_step = max(yaw_steps) if yaw_steps else 0.0
    z_span = (max(z_values) - min(z_values)) if z_values else 0.0

    return {
        "segments": len(segments),
        "poses": pose_count,
        "path_length_m": path_length,
        "mean_step_m": mean_step,
        "max_step_m": max_step,
        "max_yaw_step_rad": max_yaw_step,
        "z_span_m": z_span,
    }


def grade(stats: dict, max_step_warn_m: float, max_yaw_step_warn_rad: float) -> Tuple[str, List[str]]:
    warnings = []
    if stats["segments"] < 1 or stats["poses"] < 10:
        warnings.append("too_few_poses")
    if stats["path_length_m"] < 1.0:
        warnings.append("route_too_short")
    if stats["max_step_m"] > max_step_warn_m:
        warnings.append("large_xy_jump")
    if stats["max_yaw_step_rad"] > max_yaw_step_warn_rad:
        warnings.append("large_yaw_jump")
    if stats["z_span_m"] > 2.0:
        warnings.append("large_z_span")

    if not warnings:
        return "good", warnings
    return "reject_for_replay", warnings


def print_yaml(data: dict) -> None:
    for key, value in data.items():
        if isinstance(value, list):
            print(f"{key}:")
            for item in value:
                print(f"  - {item}")
        elif isinstance(value, float):
            print(f"{key}: {value:.6f}")
        else:
            print(f"{key}: {value}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("route", type=Path, help="WILN .ltr route file")
    parser.add_argument("--max-step-warn-m", type=float, default=1.0)
    parser.add_argument("--max-yaw-step-warn-rad", type=float, default=0.80)  # matches wiln_route_node C++ threshold
    args = parser.parse_args()

    try:
        frame_id, segments = read_ltr(args.route)
    except (OSError, ValueError) as exc:
        print_yaml(
            {
                "route": str(args.route),
                "grade": "reject_for_replay",
                "warnings": ["invalid_ltr"],
                "error": str(exc),
            }
        )
        return 2

    stats = route_stats(segments)
    route_grade, warnings = grade(stats, args.max_step_warn_m, args.max_yaw_step_warn_rad)

    print_yaml(
        {
            "route": str(args.route),
            "frame_id": frame_id,
            **stats,
            "grade": route_grade,
            "warnings": warnings,
        }
    )
    return 0 if route_grade != "reject_for_replay" else 2


if __name__ == "__main__":
    raise SystemExit(main())
