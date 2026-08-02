#!/usr/bin/env python3
"""Build the 100 Hz GT deliverable CSVs from the solver's native-rate output.

No interpolation happens here: offline_reference_solver keyframes are already
at IMU rate (~100 Hz), so optimized_trajectory.csv IS the 100 Hz trajectory —
this script only reformats it to the deliverable schema and RIGIDLY composes
the hesai_lidar extrinsic (constant lever-arm transport of position AND
velocity — see transport_velocity()) and the already-graph-optimized hitch
kinematics (trailer_pose_kinematic.csv) into the required per-sensor files.

Schema (all four *_gt_100hz.csv files):
  timestamp, x, y, z, qx, qy, qz, qw, roll, pitch, yaw,
  vx_body, vy_body, vz_body, wx_body, wy_body, wz_body,
  sigma_x, sigma_y, sigma_z, sigma_roll, sigma_pitch, sigma_yaw,
  valid_pose, quality_flags
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import yaml

SCHEMA = ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw", "roll", "pitch", "yaw",
          "vx_body", "vy_body", "vz_body", "wx_body", "wy_body", "wz_body",
          "sigma_x", "sigma_y", "sigma_z", "sigma_roll", "sigma_pitch", "sigma_yaw",
          "valid_pose", "quality_flags"]


def quat_to_rotmat(x: float, y: float, z: float, w: float) -> list[list[float]]:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def rotmat_to_quat(r: list[list[float]]) -> tuple[float, float, float, float]:
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        w, x, y, z = (r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        w, x, y, z = (r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
        w, x, y, z = (r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s
    return x, y, z, w


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def mat_vec(r, v):
    return tuple(sum(r[i][k] * v[k] for k in range(3)) for i in range(3))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def load_extrinsic(audit_dir: Path, target_frame: str) -> tuple[list[list[float]], tuple[float, float, float]]:
    """Compose /tf_static edges into R,t such that p_base = R @ p_target + t
    (base_footprint <- target_frame), by chaining child->parent edges."""
    data = yaml.safe_load((audit_dir / "tf_static.yaml").read_text())
    graph: dict[str, tuple[str, list[list[float]], tuple[float, float, float]]] = {}
    for e in data["edges"]:
        graph[e["child"]] = (e["parent"], quat_to_rotmat(*e["qxyzw"]), tuple(e["xyz"]))

    def path_to_root(frame: str) -> list[str]:
        chain = [frame]
        while chain[-1] in graph:
            chain.append(graph[chain[-1]][0])
        return chain

    chain_s = path_to_root(target_frame)
    chain_t = path_to_root("base_footprint")
    common = next(f for f in chain_s if f in set(chain_t))

    r_sc, t_sc = [[1.0, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0]
    frame = target_frame
    while frame != common:
        parent, r_pc, t_pc = graph[frame]
        t_sc = list(mat_vec(r_pc, t_sc))
        t_sc = [t_sc[i] + t_pc[i] for i in range(3)]
        r_sc = mat_mul(r_pc, r_sc)
        frame = parent

    r_tc, t_tc = [[1.0, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0]
    frame = "base_footprint"
    while frame != common:
        parent, r_pc, t_pc = graph[frame]
        t_tc = list(mat_vec(r_pc, t_tc))
        t_tc = [t_tc[i] + t_pc[i] for i in range(3)]
        r_tc = mat_mul(r_pc, r_tc)
        frame = parent

    r_tc_inv = mat_transpose(r_tc)
    r_result = mat_mul(r_tc_inv, r_sc)
    diff = [t_sc[i] - t_tc[i] for i in range(3)]
    t_result = mat_vec(r_tc_inv, diff)
    return r_result, t_result


def transport_velocity(v_base_body, w_base_body, r_base: tuple[float, float, float]):
    """Rigid-body velocity transport: velocity of a point rigidly offset by
    r_base (in the base frame) from the base origin, still in the BASE frame.
    v_point = v_base + omega x r. (Angular velocity is identical everywhere on
    a rigid body.)"""
    return tuple(v_base_body[i] + cross(w_base_body, r_base)[i] for i in range(3))


def write_pose_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=SCHEMA)
        w.writeheader()
        w.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--graph-dir", type=Path, required=True,
                         help="Directory containing optimized_trajectory.csv "
                              "(offline_reference.py's --output-dir/graph).")
    parser.add_argument("--audit-dir", type=Path, required=True,
                         help="Directory containing tf_static.yaml "
                              "(extract_v2_measurements.py's --output-dir/audit).")
    parser.add_argument("--out-dir", type=Path, required=True,
                         help="Where to write pose_{robot,hesai,trailer}_gt_100hz.csv.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    traj_rows = list(csv.DictReader((args.graph_dir / "optimized_trajectory.csv").open()))
    with (args.graph_dir / "trailer_pose_kinematic.csv").open() as f:
        trailer_rows = list(csv.DictReader(f))
    trailer_by_t = {row["t"]: row for row in trailer_rows}

    n = len(traj_rows)
    first_icp_margin_s = 2.0  # matches the ~1.7s pre-first-ICP startup window
    t0 = float(traj_rows[0]["t"])

    robot_rows, hesai_rows, trailer_out_rows = [], [], []

    r_hesai, t_hesai = load_extrinsic(args.audit_dir, "hesai_lidar")

    prev_trailer_pos = None
    prev_trailer_quat = None
    prev_t = None

    for i, row in enumerate(traj_rows):
        t = float(row["t"])
        x, y, z = float(row["x"]), float(row["y"]), float(row["z"])
        qx, qy, qz, qw = float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])
        roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
        vb = (float(row["vx_body"]), float(row["vy_body"]), float(row["vz_body"]))
        wb = (float(row["wx_body"]), float(row["wy_body"]), float(row["wz_body"]))
        sigmas = [row["sigma_x"], row["sigma_y"], row["sigma_z"],
                  row["sigma_roll"], row["sigma_pitch"], row["sigma_yaw"]]

        finite = all(math.isfinite(v) for v in (x, y, z, qx, qy, qz, qw))
        flags = []
        if t - t0 < first_icp_margin_s:
            flags.append("pre_first_icp_low_confidence")
        if row.get("has_marginals") == "0":
            flags.append("no_marginals_this_row")
        valid = 1 if finite else 0
        if not finite:
            flags.append("non_finite_pose")

        robot_rows.append({
            "timestamp": t, "x": x, "y": y, "z": z, "qx": qx, "qy": qy, "qz": qz, "qw": qw,
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "vx_body": vb[0], "vy_body": vb[1], "vz_body": vb[2],
            "wx_body": wb[0], "wy_body": wb[1], "wz_body": wb[2],
            "sigma_x": sigmas[0], "sigma_y": sigmas[1], "sigma_z": sigmas[2],
            "sigma_roll": sigmas[3], "sigma_pitch": sigmas[4], "sigma_yaw": sigmas[5],
            "valid_pose": valid, "quality_flags": ";".join(flags) if flags else "ok",
        })

        # ── Hesai: rigid composition, base_footprint -> hesai_lidar ──
        r_base = quat_to_rotmat(qx, qy, qz, qw)
        r_world_hesai = mat_mul(r_base, r_hesai)
        p_world_hesai = mat_vec(r_base, t_hesai)
        p_world_hesai = (x + p_world_hesai[0], y + p_world_hesai[1], z + p_world_hesai[2])
        hqx, hqy, hqz, hqw = rotmat_to_quat(r_world_hesai)
        h_roll, h_pitch, h_yaw = quat_to_rpy(hqx, hqy, hqz, hqw)
        # Body-frame (hesai's own frame) velocity: rotate the transported
        # base-frame velocity into hesai's frame via r_hesai^T (hesai<-base).
        v_transported_base = transport_velocity(vb, wb, t_hesai)
        v_hesai_body = mat_vec(mat_transpose(r_hesai), v_transported_base)
        w_hesai_body = mat_vec(mat_transpose(r_hesai), wb)
        hesai_rows.append({
            "timestamp": t, "x": p_world_hesai[0], "y": p_world_hesai[1], "z": p_world_hesai[2],
            "qx": hqx, "qy": hqy, "qz": hqz, "qw": hqw,
            "roll": h_roll, "pitch": h_pitch, "yaw": h_yaw,
            "vx_body": v_hesai_body[0], "vy_body": v_hesai_body[1], "vz_body": v_hesai_body[2],
            "wx_body": w_hesai_body[0], "wy_body": w_hesai_body[1], "wz_body": w_hesai_body[2],
            "sigma_x": sigmas[0], "sigma_y": sigmas[1], "sigma_z": sigmas[2],
            "sigma_roll": sigmas[3], "sigma_pitch": sigmas[4], "sigma_yaw": sigmas[5],
            "valid_pose": valid,
            "quality_flags": ";".join(flags + ["rigid_transport_from_robot_pose"]) if flags
                              else "rigid_transport_from_robot_pose",
        })

        # ── Trailer: kinematic pose already composed by the solver
        # (hitch_kinematics::computeDelta(phi_opt, alpha_opt)); velocities are
        # NOT graph states (no trailer V/W in the factor graph) — derived here
        # by 100 Hz central finite-difference of the composed pose, explicitly
        # flagged as such, not presented as a graph-optimized velocity. ──
        tr = trailer_by_t.get(row["t"])
        if tr is not None:
            tx, ty, tz = float(tr["x"]), float(tr["y"]), float(tr["z"])
            tqx, tqy, tqz, tqw = float(tr["qx"]), float(tr["qy"]), float(tr["qz"]), float(tr["qw"])
            t_roll, t_pitch, t_yaw = quat_to_rpy(tqx, tqy, tqz, tqw)
            trailer_out_rows.append({
                "timestamp": t, "x": tx, "y": ty, "z": tz, "qx": tqx, "qy": tqy, "qz": tqz, "qw": tqw,
                "roll": t_roll, "pitch": t_pitch, "yaw": t_yaw,
                "vx_body": float("nan"), "vy_body": float("nan"), "vz_body": float("nan"),
                "wx_body": float("nan"), "wy_body": float("nan"), "wz_body": float("nan"),
                "sigma_x": "nan", "sigma_y": "nan", "sigma_z": "nan",
                "sigma_roll": "nan", "sigma_pitch": "nan", "sigma_yaw": "nan",
                "valid_pose": valid,
                "quality_flags": ";".join(flags + ["kinematic_only_no_lidar_trailer_pose",
                                                     "velocity_not_yet_finite_differenced"]),
            })

    # Finite-difference trailer body-frame velocities (central difference, raw —
    # no smoothing filter; see report.md for why: matches the "no smoothing
    # that deforms turns" requirement, at the cost of more sample-to-sample
    # jitter than the graph-derived robot/hesai velocities).
    for i in range(len(trailer_out_rows)):
        lo = max(0, i - 1)
        hi = min(len(trailer_out_rows) - 1, i + 1)
        if hi == lo:
            continue
        dt = trailer_out_rows[hi]["timestamp"] - trailer_out_rows[lo]["timestamp"]
        if dt <= 0:
            continue
        r_body = quat_to_rotmat(trailer_out_rows[i]["qx"], trailer_out_rows[i]["qy"],
                                 trailer_out_rows[i]["qz"], trailer_out_rows[i]["qw"])
        r_body_inv = mat_transpose(r_body)
        dp_world = (
            (trailer_out_rows[hi]["x"] - trailer_out_rows[lo]["x"]) / dt,
            (trailer_out_rows[hi]["y"] - trailer_out_rows[lo]["y"]) / dt,
            (trailer_out_rows[hi]["z"] - trailer_out_rows[lo]["z"]) / dt,
        )
        v_body = mat_vec(r_body_inv, dp_world)
        trailer_out_rows[i]["vx_body"], trailer_out_rows[i]["vy_body"], trailer_out_rows[i]["vz_body"] = v_body
        dyaw = trailer_out_rows[hi]["yaw"] - trailer_out_rows[lo]["yaw"]
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        trailer_out_rows[i]["wz_body"] = dyaw / dt
        trailer_out_rows[i]["wx_body"] = 0.0
        trailer_out_rows[i]["wy_body"] = 0.0
        flags = trailer_out_rows[i]["quality_flags"].split(";")
        flags = [f for f in flags if f != "velocity_not_yet_finite_differenced"]
        trailer_out_rows[i]["quality_flags"] = ";".join(flags) if flags else "ok"

    write_pose_csv(args.out_dir / "pose_robot_gt_100hz.csv", robot_rows)
    write_pose_csv(args.out_dir / "pose_hesai_gt_100hz.csv", hesai_rows)
    write_pose_csv(args.out_dir / "pose_trailer_gt_100hz.csv", trailer_out_rows)

    print(f"pose_robot_gt_100hz.csv:   {len(robot_rows)} rows")
    print(f"pose_hesai_gt_100hz.csv:   {len(hesai_rows)} rows")
    print(f"pose_trailer_gt_100hz.csv: {len(trailer_out_rows)} rows")
    valid_frac = sum(r["valid_pose"] for r in robot_rows) / max(1, len(robot_rows))
    print(f"valid_pose fraction (robot): {valid_frac:.4%}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
