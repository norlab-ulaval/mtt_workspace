#!/usr/bin/env python3
"""Export an ICP trajectory CSV to a lightweight WILN .ltr route."""

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, Iterable, List


def truthy(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def yaw_to_quaternion_z(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def read_rows(path: Path) -> Iterable[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        yield from csv.DictReader(stream)


def export_route(input_csv: Path, output_ltr: Path, min_step_m: float, max_step_m: float) -> dict:
    poses: List[tuple[float, float, float]] = []
    rejected_jumps = 0
    last_x = None
    last_y = None

    for row in read_rows(input_csv):
        if not truthy(row.get("icp_quality_ok", "")):
            continue
        try:
            x = float(row["icp_x"])
            y = float(row["icp_y"])
            yaw = float(row["icp_yaw"])
        except (KeyError, TypeError, ValueError):
            continue

        if last_x is not None and last_y is not None:
            step = math.hypot(x - last_x, y - last_y)
            if step > max_step_m:
                rejected_jumps += 1
                continue
            if step < min_step_m:
                continue

        poses.append((x, y, yaw))
        last_x = x
        last_y = y

    output_ltr.parent.mkdir(parents=True, exist_ok=True)
    with output_ltr.open("w", encoding="utf-8") as stream:
        stream.write("#############################\n")
        stream.write("frame_id : map\n")
        for x, y, yaw in poses:
            qx, qy, qz, qw = yaw_to_quaternion_z(yaw)
            stream.write(f"{x:.9f},{y:.9f},0.000000000,{qx:.9f},{qy:.9f},{qz:.9f},{qw:.9f}\n")

    path_length = 0.0
    for previous, current in zip(poses, poses[1:]):
        path_length += math.hypot(current[0] - previous[0], current[1] - previous[1])

    return {
        "input_csv": str(input_csv),
        "output_ltr": str(output_ltr),
        "poses": len(poses),
        "path_length_m": path_length,
        "rejected_jumps": rejected_jumps,
        "min_step_m": min_step_m,
        "max_step_m": max_step_m,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_ltr", type=Path)
    parser.add_argument("--min-step-m", type=float, default=0.10)
    parser.add_argument("--max-step-m", type=float, default=0.75)
    args = parser.parse_args()

    stats = export_route(args.input_csv, args.output_ltr, args.min_step_m, args.max_step_m)
    for key, value in stats.items():
        if isinstance(value, float):
            print(f"{key}: {value:.6f}")
        else:
            print(f"{key}: {value}")
    if stats["poses"] < 10:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
