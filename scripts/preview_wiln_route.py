#!/usr/bin/env python3
"""Plot and score a WILN route before replaying it on the real MTT."""

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        qw * qw + qx * qx - qy * qy - qz * qz,
    )


def truthy(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


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
            x, y, _z, qx, qy, qz, qw = [float(value) for value in fields]
            if not segments:
                segments.append([])
            segments[-1].append(Pose2D(x=x, y=y, yaw=yaw_from_quaternion(qx, qy, qz, qw)))

    return frame_id, [segment for segment in segments if segment]


def flatten_segments(segments: List[List[Pose2D]]) -> List[Pose2D]:
    poses: List[Pose2D] = []
    for segment in segments:
        poses.extend(segment)
    return poses


def path_distance(poses: List[Pose2D]) -> List[float]:
    distances = [0.0]
    for previous, current in zip(poses, poses[1:]):
        distances.append(distances[-1] + math.hypot(current.x - previous.x, current.y - previous.y))
    return distances


def estimate_route_commands(
    poses: List[Pose2D],
    wheelbase_m: float,
    psi_max_rad: float,
    default_speed_ms: float,
    max_speed_ms: float,
    min_speed_ms: float,
    slowdown_alpha: float,
) -> Dict[str, List[float]]:
    distance = path_distance(poses)
    curvature = [0.0 for _ in poses]
    articulation = [0.0 for _ in poses]
    steer_norm = [0.0 for _ in poses]
    speed = [0.0 for _ in poses]
    yaw_step = [0.0 for _ in poses]
    xy_step = [0.0 for _ in poses]

    for i in range(1, len(poses)):
        ds = max(distance[i] - distance[i - 1], 1e-6)
        dyaw = wrap_to_pi(poses[i].yaw - poses[i - 1].yaw)
        kappa = dyaw / ds
        psi = math.atan(wheelbase_m * kappa)
        psi = max(-psi_max_rad, min(psi_max_rad, psi))
        v = min(default_speed_ms, max_speed_ms)
        v = v / (1.0 + slowdown_alpha * abs(kappa))
        v = max(min_speed_ms, min(max_speed_ms, v))

        curvature[i] = kappa
        articulation[i] = psi
        steer_norm[i] = psi / max(psi_max_rad, 1e-6)
        speed[i] = v
        yaw_step[i] = abs(dyaw)
        xy_step[i] = ds

    if len(speed) > 1:
        speed[0] = speed[1]
    return {
        "s_m": distance,
        "curvature_m_inv": curvature,
        "articulation_rad": articulation,
        "steer_norm": steer_norm,
        "speed_ms": speed,
        "yaw_step_rad": yaw_step,
        "xy_step_m": xy_step,
    }


def read_dataset_csv(path: Optional[Path]) -> Dict[str, List[float]]:
    out: Dict[str, List[float]] = {
        "icp_x": [],
        "icp_y": [],
        "model_x": [],
        "model_y": [],
        "cmd_linear_x": [],
        "cmd_angular_z": [],
    }
    if path is None or not path.exists():
        return out

    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if not truthy(row.get("icp_quality_ok", "true")):
                continue
            for key in out:
                try:
                    value = row.get(key, "")
                    if value != "":
                        out[key].append(float(value))
                except ValueError:
                    pass
    return out


def finite_max_abs(values: Iterable[float]) -> float:
    finite = [abs(value) for value in values if math.isfinite(value)]
    return max(finite) if finite else 0.0


def build_summary(route: Path, frame_id: str, poses: List[Pose2D], commands: Dict[str, List[float]], args) -> Dict:
    distance = commands["s_m"][-1] if commands["s_m"] else 0.0
    max_step = max(commands["xy_step_m"]) if commands["xy_step_m"] else 0.0
    max_yaw_step = max(commands["yaw_step_rad"]) if commands["yaw_step_rad"] else 0.0
    max_steer = finite_max_abs(commands["steer_norm"])
    max_curvature = finite_max_abs(commands["curvature_m_inv"])
    warnings = []
    if len(poses) < 10:
        warnings.append("too_few_route_poses")
    if distance < 1.0:
        warnings.append("route_too_short")
    if max_step > args.max_step_warn_m:
        warnings.append("large_xy_step")
    if max_yaw_step > args.max_yaw_step_warn_rad:
        warnings.append("large_yaw_step")
    if max_steer > 0.95:
        warnings.append("steering_near_saturation")
    if max_curvature > args.kappa_max_warn:
        warnings.append("high_curvature")

    if not warnings:
        grade = "good_for_dry_run"
    elif all(w in {"steering_near_saturation", "high_curvature"} for w in warnings):
        grade = "usable_slowly"
    else:
        grade = "inspect_before_replay"

    return {
        "route": str(route),
        "frame_id": frame_id,
        "poses": len(poses),
        "path_length_m": distance,
        "max_step_m": max_step,
        "max_yaw_step_rad": max_yaw_step,
        "max_curvature_m_inv": max_curvature,
        "max_abs_steer_norm": max_steer,
        "speed_min_ms": min(commands["speed_ms"]) if commands["speed_ms"] else 0.0,
        "speed_max_ms": max(commands["speed_ms"]) if commands["speed_ms"] else 0.0,
        "grade": grade,
        "warnings": warnings,
    }


def plot_preview(
    route: Path,
    output_png: Path,
    poses: List[Pose2D],
    commands: Dict[str, List[float]],
    dataset: Dict[str, List[float]],
    summary: Dict,
) -> None:
    output_png.parent.mkdir(parents=True, exist_ok=True)
    xs = [pose.x for pose in poses]
    ys = [pose.y for pose in poses]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(f"WILN route preview: {route.name} ({summary['grade']})")

    ax = axes[0][0]
    if dataset["icp_x"] and dataset["icp_y"]:
        ax.plot(dataset["icp_x"], dataset["icp_y"], color="0.75", linewidth=1.2, label="bag ICP pose")
    if dataset["model_x"] and dataset["model_y"]:
        ax.plot(dataset["model_x"], dataset["model_y"], color="tab:green", linewidth=1.0, alpha=0.8, label="motion model")
    ax.plot(xs, ys, color="tab:blue", linewidth=2.0, label="WILN taught route")
    if xs and ys:
        ax.scatter([xs[0]], [ys[0]], color="green", s=50, label="start", zorder=5)
        ax.scatter([xs[-1]], [ys[-1]], color="red", s=50, label="end", zorder=5)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.35)
    ax.legend()

    ax = axes[0][1]
    ax.plot(commands["s_m"], commands["steer_norm"], label="planned steer normalized")
    ax.axhline(1.0, color="r", linestyle="--", linewidth=0.8)
    ax.axhline(-1.0, color="r", linestyle="--", linewidth=0.8)
    ax.set_xlabel("route distance [m]")
    ax.set_ylabel("steer [-1, 1]")
    ax.grid(True, alpha=0.35)
    ax.legend()

    ax = axes[1][0]
    ax.plot(commands["s_m"], commands["speed_ms"], label="planned speed")
    ax.set_xlabel("route distance [m]")
    ax.set_ylabel("speed [m/s]")
    ax.grid(True, alpha=0.35)
    ax.legend()

    ax = axes[1][1]
    ax.plot(commands["s_m"], commands["curvature_m_inv"], label="route curvature")
    ax.set_xlabel("route distance [m]")
    ax.set_ylabel("curvature [1/m]")
    ax.grid(True, alpha=0.35)
    ax.legend()

    warnings_text = ", ".join(summary["warnings"]) if summary["warnings"] else "none"
    fig.text(
        0.01,
        0.01,
        f"path={summary['path_length_m']:.2f} m | poses={summary['poses']} | "
        f"max_step={summary['max_step_m']:.2f} m | max_steer={summary['max_abs_steer_norm']:.2f} | "
        f"warnings={warnings_text}",
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0.03, 1, 0.96))
    fig.savefig(output_png, dpi=150)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("route", type=Path, help="WILN .ltr route")
    parser.add_argument("--dataset-csv", type=Path, default=None, help="Optional model_dataset.csv for ICP/model overlay")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--wheelbase-m", type=float, default=2.40)
    parser.add_argument("--psi-max-rad", type=float, default=1.0471975512)
    parser.add_argument("--default-speed-ms", type=float, default=0.40)
    parser.add_argument("--max-speed-ms", type=float, default=0.50)
    parser.add_argument("--min-speed-ms", type=float, default=0.25)
    parser.add_argument("--slowdown-alpha", type=float, default=2.0)
    parser.add_argument("--max-step-warn-m", type=float, default=1.0)
    parser.add_argument("--max-yaw-step-warn-rad", type=float, default=0.50)
    parser.add_argument("--kappa-max-warn", type=float, default=0.70)
    args = parser.parse_args()

    frame_id, segments = read_ltr(args.route)
    poses = flatten_segments(segments)
    commands = estimate_route_commands(
        poses,
        args.wheelbase_m,
        args.psi_max_rad,
        args.default_speed_ms,
        args.max_speed_ms,
        args.min_speed_ms,
        args.slowdown_alpha,
    )
    summary = build_summary(args.route, frame_id, poses, commands, args)
    dataset = read_dataset_csv(args.dataset_csv)

    output_dir = args.output_dir or args.route.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    output_png = output_dir / "preview.png"
    output_yaml = output_dir / "preview.yaml"
    output_metadata = output_dir / "metadata.yaml"
    plot_preview(args.route, output_png, poses, commands, dataset, summary)
    summary_yaml = yaml.safe_dump(summary, sort_keys=False)
    output_yaml.write_text(summary_yaml, encoding="utf-8")
    output_metadata.write_text(summary_yaml, encoding="utf-8")

    print(f"preview_plot: {output_png}")
    print(f"preview_summary: {output_yaml}")
    print(f"metadata: {output_metadata}")
    print(f"grade: {summary['grade']}")
    if summary["warnings"]:
        print("warnings:")
        for warning in summary["warnings"]:
            print(f"  - {warning}")
    return 0 if summary["grade"] != "inspect_before_replay" else 2


if __name__ == "__main__":
    raise SystemExit(main())
