#!/usr/bin/env python3
"""Plot full-bag rollout of a selected motion model against ICP."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any

import yaml

import evaluate_motion_model_horizon as horizon


def rollout(rows: list[dict[str, Any]], args: argparse.Namespace) -> list[dict[str, float]]:
    if not rows:
        return []

    x = float(rows[0]["icp_x"])
    y = float(rows[0]["icp_y"])
    yaw = float(rows[0]["icp_heading"])
    prev_t = float(rows[0]["t"])
    start_speed, start_phi = horizon.model_inputs(
        rows[0],
        speed_source=args.speed_source,
        articulation_source=args.articulation_source,
    )
    v_state = start_speed if start_speed is not None else 0.0
    phi_state = start_phi if start_phi is not None else 0.0
    r_state = (
        v_state * horizon.nominal_curvature(phi_state, args.curvature_model)
        if abs(v_state) >= horizon.MIN_TURN_SPEED_MS
        else 0.0
    )

    out: list[dict[str, float]] = []
    t0 = float(rows[0]["t"])
    for row in rows:
        t = float(row["t"])
        dt = max(0.0, min(t - prev_t, 1.0))
        speed_cmd, phi_cmd = horizon.model_inputs(
            row,
            speed_source=args.speed_source,
            articulation_source=args.articulation_source,
        )
        speed = 0.0
        yaw_rate = 0.0
        if speed_cmd is not None and phi_cmd is not None:
            if args.model_kind == "direct":
                speed = speed_cmd
                yaw_rate = (
                    speed * horizon.nominal_curvature(phi_cmd, args.curvature_model)
                    if abs(speed) >= horizon.MIN_TURN_SPEED_MS
                    else 0.0
                )
            elif args.model_kind == "slip_heuristic":
                speed = speed_cmd
                curvature = horizon.nominal_curvature(phi_cmd, args.curvature_model)
                yaw_rate = (
                    speed * curvature * horizon.slip_scale(speed, phi_cmd)
                    if abs(speed) >= horizon.MIN_TURN_SPEED_MS
                    else 0.0
                )
            elif args.model_kind == "m1_phi_lag":
                alpha_v = horizon.clamp(dt / max(args.tau_v_s, 1e-6), 0.0, 1.0)
                alpha_phi = horizon.clamp(dt / max(args.tau_phi_s, 1e-6), 0.0, 1.0)
                v_state += (speed_cmd - v_state) * alpha_v
                phi_state += (phi_cmd - phi_state) * alpha_phi
                phi_state = horizon.clamp(phi_state, -horizon.MAX_ARTICULATION_RAD, horizon.MAX_ARTICULATION_RAD)
                speed = v_state
                yaw_rate = (
                    speed * horizon.nominal_curvature(phi_state, args.curvature_model)
                    if abs(speed) >= horizon.MIN_TURN_SPEED_MS
                    else 0.0
                )
            elif args.model_kind == "m1_yaw_lag":
                alpha_v = horizon.clamp(dt / max(args.tau_v_s, 1e-6), 0.0, 1.0)
                alpha_r = horizon.clamp(dt / max(args.tau_r_s, 1e-6), 0.0, 1.0)
                v_state += (speed_cmd - v_state) * alpha_v
                target_r = (
                    v_state * horizon.nominal_curvature(phi_cmd, args.curvature_model)
                    if abs(v_state) >= horizon.MIN_TURN_SPEED_MS
                    else 0.0
                )
                r_state += (target_r - r_state) * alpha_r
                speed = v_state
                yaw_rate = r_state

        if dt > 0.0:
            dtheta = yaw_rate * dt
            heading_mid = yaw + 0.5 * dtheta
            x += speed * dt * math.cos(heading_mid)
            y += speed * dt * math.sin(heading_mid)
            yaw = horizon.wrap_angle(yaw + dtheta)
        icp_x = float(row["icp_x"])
        icp_y = float(row["icp_y"])
        icp_yaw = float(row["icp_heading"])
        out.append(
            {
                "t": t,
                "offset_s": t - t0,
                "icp_x": icp_x,
                "icp_y": icp_y,
                "icp_yaw": icp_yaw,
                "model_x": x,
                "model_y": y,
                "model_yaw": yaw,
                "position_error_m": math.hypot(x - icp_x, y - icp_y),
                "yaw_error_rad": horizon.wrap_angle(yaw - icp_yaw),
                "model_speed_ms": speed,
                "model_yaw_rate_rad_s": yaw_rate,
                "icp_speed_ms": horizon.parse_float(row.get("icp_linear_x")) or 0.0,
                "icp_yaw_rate_rad_s": horizon.parse_float(row.get("icp_angular_z")) or 0.0,
            }
        )
        prev_t = t
    return out


def write_csv(path: Path, rows: list[dict[str, float]]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_plots(
    out_dir: Path,
    prefix: str,
    rows: list[dict[str, float]],
    source_rows: list[dict[str, Any]],
) -> dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}

    outputs: dict[str, str] = {}
    if rows:
        plt.figure(figsize=(10, 8))
        plt.plot([r["icp_x"] for r in rows], [r["icp_y"] for r in rows], label="ICP", linewidth=1.4)
        plt.plot([r["model_x"] for r in rows], [r["model_y"] for r in rows], label="model", linewidth=1.2)
        plt.axis("equal")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Full-bag trajectory rollout")
        plt.grid(True, alpha=0.4)
        plt.legend()
        plt.tight_layout()
        path = out_dir / f"{prefix}_full_trajectory_xy.png"
        plt.savefig(path, dpi=150)
        plt.close()
        outputs["trajectory_xy"] = str(path)

    icp_xs, icp_ys, odom_xs, odom_ys = horizon.aligned_xy_to_icp(
        source_rows,
        x_key="odom_x",
        y_key="odom_y",
        yaw_key="odom_heading",
    )
    if icp_xs and odom_xs:
        plt.figure(figsize=(10, 8))
        plt.plot(icp_xs, icp_ys, label="ICP", linewidth=1.35)
        plt.plot(odom_xs, odom_ys, label="MTT odom aligned to ICP start", linewidth=1.15)
        plt.plot(icp_xs[0], icp_ys[0], "ko", markersize=5, label="start")
        plt.axis("equal")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Global MTT odom vs ICP")
        plt.grid(True, alpha=0.4)
        plt.legend()
        plt.tight_layout()
        path = out_dir / f"{prefix}_global_mtt_odom_vs_icp.png"
        plt.savefig(path, dpi=150)
        plt.close()
        outputs["global_mtt_odom_vs_icp"] = str(path)

    plt.figure(figsize=(12, 5))
    plt.plot([r["offset_s"] for r in rows], [r["position_error_m"] for r in rows], label="position")
    plt.xlabel("time [s]")
    plt.ylabel("position error [m]")
    plt.title("Full rollout accumulated position error")
    plt.grid(True, alpha=0.4)
    plt.legend()
    plt.tight_layout()
    path = out_dir / f"{prefix}_full_position_error_time.png"
    plt.savefig(path, dpi=150)
    plt.close()
    outputs["position_error_time"] = str(path)

    plt.figure(figsize=(12, 6))
    plt.subplot(2, 1, 1)
    plt.plot([r["offset_s"] for r in rows], [r["model_speed_ms"] for r in rows], label="model")
    plt.plot([r["offset_s"] for r in rows], [r["icp_speed_ms"] for r in rows], label="ICP", alpha=0.75)
    plt.ylabel("speed [m/s]")
    plt.grid(True, alpha=0.4)
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.plot([r["offset_s"] for r in rows], [r["model_yaw_rate_rad_s"] for r in rows], label="model")
    plt.plot([r["offset_s"] for r in rows], [r["icp_yaw_rate_rad_s"] for r in rows], label="ICP", alpha=0.75)
    plt.xlabel("time [s]")
    plt.ylabel("yaw rate [rad/s]")
    plt.grid(True, alpha=0.4)
    plt.legend()
    plt.tight_layout()
    path = out_dir / f"{prefix}_full_speed_yaw_rate_time.png"
    plt.savefig(path, dpi=150)
    plt.close()
    outputs["speed_yaw_rate_time"] = str(path)
    return outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--speed-source", choices=["tach", "status", "cmd"], default="status")
    parser.add_argument("--articulation-source", choices=["measured", "status", "cmd"], default="status")
    parser.add_argument("--curvature-model", choices=["exact", "tangent"], default="exact")
    parser.add_argument("--model-kind", choices=["direct", "m1_yaw_lag", "m1_phi_lag", "slip_heuristic"], default="direct")
    parser.add_argument("--tau-v-s", type=float, default=0.2)
    parser.add_argument("--tau-r-s", type=float, default=0.2)
    parser.add_argument("--tau-phi-s", type=float, default=0.2)
    parser.add_argument("--output-prefix", default="rollout")
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session = args.session_dir.expanduser()
    out_dir = args.output_dir.expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = horizon.read_rows(session / "motion_model_validation" / "aligned_samples.csv")
    traces = rollout(rows, args)
    csv_path = out_dir / f"{args.output_prefix}_full_rollout.csv"
    write_csv(csv_path, traces)
    outputs = write_plots(out_dir, args.output_prefix, traces, rows)
    summary = {
        "session": session.name,
        "settings": vars(args) | {"session_dir": str(session), "output_dir": str(out_dir)},
        "rows": len(traces),
        "csv": str(csv_path),
        "outputs": outputs,
    }
    path = out_dir / f"{args.output_prefix}_full_rollout_summary.yaml"
    path.write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
