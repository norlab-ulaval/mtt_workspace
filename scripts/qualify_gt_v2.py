#!/usr/bin/env python3
"""Qualify the V2 ICE_RINK GT: continuity, sensor residuals, revisit return-error
(heading-aware), rigidity, ablation comparison, and figures.

Reads only already-produced artifacts (no bag access, no ROS). Writes to
artifacts/gt_v2_icerink/qualification/.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import math
import statistics
from pathlib import Path

import yaml

def quat_to_yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def read_csv_rows(path: Path) -> list[dict]:
    with path.open() as f:
        return list(csv.DictReader(f))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-dir", type=Path, required=True,
                         help="Directory containing graph/, measurements/, poses/, audit/ "
                              "(the same --output-dir root used by offline_reference.py, "
                              "extract_v2_measurements.py, and build_gt_v2_100hz.py).")
    parser.add_argument("--ablation-dir", type=Path, default=None,
                         help="Optional graph_ablation_icp_imu_only/ directory; ablation "
                              "section is skipped if not supplied.")
    parser.add_argument("--out-dir", type=Path, required=True,
                         help="Where to write qualification_report.md, quality_summary.yaml, figures/.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    graph_dir = args.base_dir / "graph"
    meas_dir = args.base_dir / "measurements"
    poses_dir = args.base_dir / "poses"
    audit_dir = args.base_dir / "audit"
    out_dir = args.out_dir
    fig_dir = out_dir / "figures"
    ablation_dir = args.ablation_dir

    out_dir.mkdir(parents=True, exist_ok=True)
    fig_dir.mkdir(parents=True, exist_ok=True)
    report_lines: list[str] = []

    def log(line: str = "") -> None:
        print(line)
        report_lines.append(line)

    log("# GT V2 ICE_RINK — Qualification\n")

    traj = read_csv_rows(graph_dir / "optimized_trajectory.csv")
    t = [float(r["t"]) for r in traj]
    x = [float(r["x"]) for r in traj]
    y = [float(r["y"]) for r in traj]
    z = [float(r["z"]) for r in traj]

    # ── 1. Temporal continuity ──
    dts = [t[i + 1] - t[i] for i in range(len(t) - 1)]
    dt_mean, dt_std, dt_max = statistics.mean(dts), statistics.pstdev(dts), max(dts)
    dt_min = min(dts)
    n_gaps = sum(1 for d in dts if d > 0.05)  # > 5x nominal 100Hz period
    n_nonmonotonic = sum(1 for d in dts if d <= 0)
    log("## 1. Temporal continuity")
    log(f"- n_keyframes: {len(t)}")
    log(f"- dt: mean={dt_mean*1000:.3f}ms std={dt_std*1000:.4f}ms min={dt_min*1000:.3f}ms max={dt_max*1000:.3f}ms")
    log(f"- gaps (dt > 50ms): {n_gaps}")
    log(f"- non-monotonic timestamps: {n_nonmonotonic}")

    # ── 2. Spatial continuity — no position jumps between consecutive keyframes ──
    step_dists = [math.sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2 + (z[i+1]-z[i])**2) for i in range(len(x)-1)]
    max_step = max(step_dists)
    max_step_speed = max(step_dists[i] / dts[i] for i in range(len(dts)) if dts[i] > 0)
    log("\n## 2. Spatial continuity")
    log(f"- max consecutive-keyframe step: {max_step:.4f} m")
    log(f"- max implied instantaneous speed: {max_step_speed:.3f} m/s")
    log(f"- {'PASS' if max_step_speed < 15.0 else 'FLAG'}: no unphysical jump (threshold 15 m/s for a tracked utility vehicle)")

    # ── 3. Pose-derivative vs twist consistency ──
    # Finite-difference world position, rotate into body frame using the
    # graph's own orientation, compare against the graph's V(k) (already
    # rotated to body in vx_body/vy_body/vz_body).
    resid = []
    for i in range(1, len(traj) - 1):
        dt_c = t[i + 1] - t[i - 1]
        if dt_c <= 0:
            continue
        qx, qy, qz, qw = float(traj[i]["qx"]), float(traj[i]["qy"]), float(traj[i]["qz"]), float(traj[i]["qw"])
        # world->body rotation via quaternion conjugate application (inline, no numpy)
        vwx = (x[i + 1] - x[i - 1]) / dt_c
        vwy = (y[i + 1] - y[i - 1]) / dt_c
        vwz = (z[i + 1] - z[i - 1]) / dt_c
        # rotate world vector into body frame: v_body = R^T v_world, R from quaternion
        n = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw) or 1.0
        qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
        r = [
            [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
            [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
            [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
        ]
        vb_fd = (r[0][0]*vwx+r[1][0]*vwy+r[2][0]*vwz,
                 r[0][1]*vwx+r[1][1]*vwy+r[2][1]*vwz,
                 r[0][2]*vwx+r[1][2]*vwy+r[2][2]*vwz)
        vb_graph = (float(traj[i]["vx_body"]), float(traj[i]["vy_body"]), float(traj[i]["vz_body"]))
        d = math.sqrt(sum((vb_fd[k]-vb_graph[k])**2 for k in range(3)))
        resid.append(d)
    log("\n## 3. Pose-derivative vs graph-twist consistency")
    log(f"- mean |v_finite_diff - v_graph|: {statistics.mean(resid):.4f} m/s")
    log(f"- median: {statistics.median(resid):.4f} m/s, p95: {sorted(resid)[int(0.95*len(resid))]:.4f} m/s")

    # ── 3b. Angular-rate consistency: wx_body/wy_body/wz_body in the deliverable
    # are bias-corrected gyro measurements passed through, NOT a graph state (the
    # graph never estimates angular velocity — only the orientation states X(k)
    # and the additive bias B(k)). This check verifies that "measurement minus
    # converged bias" agrees with what a consumer would independently reconstruct
    # by finite-differencing the graph's own optimized quaternions — i.e. that
    # ORIENTATION (which IS a graph state) and the reported angular rate agree,
    # the same role section 3 plays for linear velocity vs position.
    def quat_mul(a, b):
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return (
            aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
        )

    def quat_conj(q):
        w, x, y, z = q
        return (w, -x, -y, -z)

    w_resid = []
    for i in range(1, len(traj) - 1):
        dt_c = t[i + 1] - t[i - 1]
        if dt_c <= 0:
            continue
        def q_of(row):
            qx, qy, qz, qw = float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])
            n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) or 1.0
            return (qw/n, qx/n, qy/n, qz/n)  # (w,x,y,z)
        q_prev, q_next = q_of(traj[i - 1]), q_of(traj[i + 1])
        dq = quat_mul(quat_conj(q_prev), q_next)  # body-frame rotation prev->next
        dw, dx, dy, dz = dq
        if dw < 0:  # shortest-path convention
            dw, dx, dy, dz = -dw, -dx, -dy, -dz
        axis_norm = math.sqrt(dx*dx + dy*dy + dz*dz)
        angle = 2 * math.atan2(axis_norm, dw)
        if axis_norm > 1e-12:
            wb_fd = (dx/axis_norm*angle/dt_c, dy/axis_norm*angle/dt_c, dz/axis_norm*angle/dt_c)
        else:
            wb_fd = (0.0, 0.0, 0.0)
        wb_graph = (float(traj[i]["wx_body"]), float(traj[i]["wy_body"]), float(traj[i]["wz_body"]))
        w_resid.append(math.sqrt(sum((wb_fd[k] - wb_graph[k])**2 for k in range(3))))
    log("\n## 3b. Angular-rate consistency (finite-diff of graph orientation X(k) vs reported wx/wy/wz_body)")
    log(f"- mean |w_finite_diff - w_reported|: {statistics.mean(w_resid):.5f} rad/s")
    log(f"- median: {statistics.median(w_resid):.5f} rad/s, p95: {sorted(w_resid)[int(0.95*len(w_resid))]:.5f} rad/s")
    log("  NOTE: wx/wy/wz_body are bias-corrected gyro measurements (not an optimized graph state); "
        "this checks they agree with the graph's own optimized orientation trajectory.")

    # ── 4. IMU residuals: mti100 (in-graph, via bias convergence) + ZED (control) ──
    log("\n## 4. IMU residuals")
    bax = [float(r["bax"]) for r in traj]
    bay = [float(r["bay"]) for r in traj]
    baz = [float(r["baz"]) for r in traj]
    bgx = [float(r["bgx"]) for r in traj]
    bgy = [float(r["bgy"]) for r in traj]
    bgz = [float(r["bgz"]) for r in traj]
    log(f"- mti100 (in-graph) estimated accel bias: final=({bax[-1]:.5f},{bay[-1]:.5f},{baz[-1]:.5f}) m/s^2, "
        f"range accel_bias_norm=[{min(math.sqrt(bax[i]**2+bay[i]**2+baz[i]**2) for i in range(len(bax))):.5f}, "
        f"{max(math.sqrt(bax[i]**2+bay[i]**2+baz[i]**2) for i in range(len(bax))):.5f}]")
    log(f"- mti100 (in-graph) estimated gyro bias: final=({bgx[-1]:.6f},{bgy[-1]:.6f},{bgz[-1]:.6f}) rad/s")
    log(f"  {'PASS' if max(abs(b) for b in (bax[-1],bay[-1],baz[-1])) < 0.5 else 'FLAG'}: accel bias magnitude physically plausible for XSens MTi-100 (<0.5 m/s^2)")

    zed_imu_path = meas_dir / "zed_imu.csv"
    if zed_imu_path.exists():
        zed_imu = read_csv_rows(zed_imu_path)
        az = [float(r["az"]) for r in zed_imu[:300]]  # first 3s, stationary
        log(f"- ZED IMU (control, never fused) stationary-window mean az: {statistics.mean(az):.4f} m/s^2 "
            f"(expect ~+9.81; cross-check against mti100 audit value)")
    else:
        log("- ZED IMU control file not found — skipped")

    # ── 5. ICP innovation (residual between graph estimate and ICP prior mean) ──
    icp = read_csv_rows(meas_dir / "icp.csv")
    icp_t = [float(r["t"]) for r in icp]
    traj_idx = 0
    innov_trans = []
    for row in icp:
        ti = float(row["t"])
        while traj_idx + 1 < len(t) and abs(t[traj_idx + 1] - ti) < abs(t[traj_idx] - ti):
            traj_idx += 1
        dx = float(row["x"]) - x[traj_idx]
        dy = float(row["y"]) - y[traj_idx]
        dz = float(row["z"]) - z[traj_idx]
        innov_trans.append(math.sqrt(dx*dx + dy*dy + dz*dz))
    log("\n## 5. ICP prior innovation (|ICP measurement - graph estimate| at nearest keyframe)")
    log(f"- mean: {statistics.mean(innov_trans):.4f} m, median: {statistics.median(innov_trans):.4f} m, "
        f"p95: {sorted(innov_trans)[int(0.95*len(innov_trans))]:.4f} m, max: {max(innov_trans):.4f} m")
    icp_sigma_xy = 0.04
    rejected = sum(1 for d in innov_trans if d > 3 * icp_sigma_xy)
    log(f"- fraction beyond 3-sigma ({3*icp_sigma_xy:.3f} m): {rejected}/{len(innov_trans)} ({rejected/len(innov_trans):.2%}) "
        f"— note: Huber-robust, not hard-rejected; large values are DOWN-WEIGHTED not dropped")

    # ── 6. ZED odom vs optimized trajectory (ZED IS a loose factor here, per
    # the "use everything" decision — this check still reports agreement) ──
    zed_odom_path = meas_dir / "zed_odom.csv"
    if zed_odom_path.exists():
        zed = read_csv_rows(zed_odom_path)
        if len(zed) > 1:
            zed_disp = []
            traj_idx = 0
            prev = None
            for row in zed:
                tz = float(row["t"])
                while traj_idx + 1 < len(t) and t[traj_idx + 1] <= tz:
                    traj_idx += 1
                cur = (float(row["x"]), float(row["y"]), (x[traj_idx], y[traj_idx]))
                if prev is not None and prev[2] != cur[2]:
                    dz_ = math.hypot(cur[0]-prev[0], cur[1]-prev[1])
                    dg = math.hypot(cur[2][0]-prev[2][0], cur[2][1]-prev[2][1])
                    zed_disp.append(abs(dz_ - dg))
                prev = cur
            if zed_disp:
                log("\n## 6. ZED odom vs optimized trajectory (displacement agreement, loosely-weighted factor)")
                log(f"- mean |zed_disp - graph_disp| per step: {statistics.mean(zed_disp):.4f} m")
    else:
        log("\n## 6. ZED odom control file not found — skipped")

    # ── 7. Revisits — heading-aware classification ──
    # loop_closures.csv's icp_row_i/icp_row_j are exact 0-indexed row indices
    # into icp.csv (same file, same order, written and read by the same two
    # tools) — use those directly rather than matching on the "t" column as a
    # string, which silently matches nothing (the C++ solver and this script
    # format floats to different precision/representation).
    loops = read_csv_rows(graph_dir / "loop_closures.csv")
    icp_yaw_by_idx = [quat_to_yaw(float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"])) for r in icp]
    same_heading, opposite_heading, crossing = [], [], []
    for row in loops:
        i_idx, j_idx = int(row["icp_row_i"]), int(row["icp_row_j"])
        if i_idx >= len(icp_yaw_by_idx) or j_idx >= len(icp_yaw_by_idx):
            continue
        dyaw = abs(wrap(icp_yaw_by_idx[j_idx] - icp_yaw_by_idx[i_idx])) * 180 / math.pi
        trans_err = float(row["return_trans_err_m"])
        if dyaw < 30:
            same_heading.append(trans_err)
        elif dyaw > 150:
            opposite_heading.append(trans_err)
        else:
            crossing.append(trans_err)
    log("\n## 7. Revisit return-error, classified by heading difference at revisit")
    log(f"- same-heading (<30 deg) revisits, n={len(same_heading)}: "
        f"{'mean='+format(statistics.mean(same_heading),'.4f')+'m median='+format(statistics.median(same_heading),'.4f')+'m' if same_heading else 'none'}")
    log(f"- opposite-heading (>150 deg) crossings, n={len(opposite_heading)}: "
        f"{'mean='+format(statistics.mean(opposite_heading),'.4f')+'m' if opposite_heading else 'none'}")
    log(f"- oblique crossings (30-150 deg), n={len(crossing)}: "
        f"{'mean='+format(statistics.mean(crossing),'.4f')+'m' if crossing else 'none'}")
    log("- Interpretation: only same-heading revisits are genuine loop-closure-style return-error "
        "evidence; opposite/oblique entries are path crossings at different headings, where a large "
        "translation/rotation 'error' is expected and NOT evidence of drift.")

    # ── 8. Rigidity robot<->hesai (re-verify from the 100Hz deliverable) ──
    robot = read_csv_rows(poses_dir / "pose_robot_gt_100hz.csv")
    hesai = read_csv_rows(poses_dir / "pose_hesai_gt_100hz.csv")
    dists = []
    for i in range(0, len(robot), 500):
        dx = float(hesai[i]["x"]) - float(robot[i]["x"])
        dy = float(hesai[i]["y"]) - float(robot[i]["y"])
        dz = float(hesai[i]["z"]) - float(robot[i]["z"])
        dists.append(math.sqrt(dx*dx + dy*dy + dz*dz))
    log("\n## 8. Rigidity robot<->hesai_lidar")
    log(f"- distance: min={min(dists):.6f}m max={max(dists):.6f}m std={statistics.pstdev(dists):.2e}m")
    log(f"  {'PASS' if statistics.pstdev(dists) < 1e-9 else 'FLAG'}: constant to numerical precision (exact rigid composition)")

    # ── 9. Articulation coherence: graph phi vs raw hardware encoder ──
    artic = read_csv_rows(meas_dir / "articulation_state.csv")
    phi_resid = []
    traj_idx = 0
    for row in artic:
        if row["hardware_fresh"] != "1":
            continue
        ta = float(row["t"])
        while traj_idx + 1 < len(t) and abs(t[traj_idx + 1] - ta) < abs(t[traj_idx] - ta):
            traj_idx += 1
        phi_graph = float(traj[traj_idx]["phi_rad"])
        phi_meas = float(row["hardware_rad"])
        phi_resid.append(abs(wrap(phi_graph - phi_meas)))
    log("\n## 9. Articulation (hitch yaw) coherence: graph H(k) vs raw hardware encoder")
    log(f"- mean residual: {statistics.mean(phi_resid)*180/math.pi:.3f} deg, "
        f"p95: {sorted(phi_resid)[int(0.95*len(phi_resid))]*180/math.pi:.3f} deg")
    log("- LiDAR-fused hitch angle (/mtt/articulation_state.lidar_rad) never fired in this bag "
        "(0% lidar_detected, see audit_report.md) — H(k) is hardware-encoder-only, stated not hidden.")
    log("- RS-Airy raw point cloud was NOT reprocessed offline (would require re-implementing "
        "trailer_pose_node's PCA pipeline — out of scope for this CSV pipeline); trailer pose is "
        "kinematic-only (hitch_kinematics::computeDelta on graph-optimized phi/alpha), not LiDAR-corrected.")

    # ── 10. Frame/origin jump check ──
    jumps = [i for i, d in enumerate(step_dists) if d > 2.0]
    log("\n## 10. Frame/origin jump check")
    log(f"- consecutive-keyframe steps > 2.0 m: {len(jumps)}" + (f" at indices {jumps[:10]}..." if jumps else " (none)"))

    # ── 11. Ablation: full config vs ICP+IMU+articulation only (no track/zed odom) ──
    log("\n## 11. Ablation — full vs ICP+IMU+articulation only (no track/zed odom)")
    if ablation_dir is not None and ablation_dir.exists() and (ablation_dir / "optimized_trajectory.csv").exists():
        abl = read_csv_rows(ablation_dir / "optimized_trajectory.csv")
        abl_t = [float(r["t"]) for r in abl]
        abl_x = [float(r["x"]) for r in abl]
        abl_y = [float(r["y"]) for r in abl]
        diffs = [math.hypot(x[i] - abl_x[i], y[i] - abl_y[i]) for i in range(min(len(x), len(abl_x)))]
        log(f"- full config factor/variable counts: see solver_summary.yaml")
        log(f"- position difference (full vs ICP+IMU+artic only): mean={statistics.mean(diffs):.4f}m "
            f"median={statistics.median(diffs):.4f}m max={max(diffs):.4f}m")
        log("- Interpretation: small difference means track/zed odom (loosely weighted, sigma >> ICP sigma) "
            "contribute negligibly to the trajectory ITSELF given dense ICP priors already anchor it every "
            "~50ms — their real contribution is redundancy/cross-validation, not correction. A large "
            "difference would indicate ICP has gaps track/zed odom are filling.")
        ablation_csv_rows = [
            {"config": "ICP+IMU+articulation only", "icp": "yes", "imu": "yes", "track_odom": "no", "zed_odom": "no",
             "mean_pos_diff_vs_full_m": f"{statistics.mean(diffs):.4f}", "max_pos_diff_vs_full_m": f"{max(diffs):.4f}"},
            {"config": "full (this deliverable)", "icp": "yes", "imu": "yes", "track_odom": "yes", "zed_odom": "yes",
             "mean_pos_diff_vs_full_m": "0.0000", "max_pos_diff_vs_full_m": "0.0000"},
        ]
    else:
        log("- ablation run not yet available")
        ablation_csv_rows = []

    if ablation_csv_rows:
        with (out_dir / "ablation.csv").open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(ablation_csv_rows[0].keys()))
            w.writeheader()
            w.writerows(ablation_csv_rows)

    # ── 12. Invalid segment detection (quality_flags already computed in poses/) ──
    invalid = [r for r in robot if r["valid_pose"] != "1"]
    log("\n## 12. Invalid segments")
    log(f"- {len(invalid)}/{len(robot)} rows flagged valid_pose=0 ({len(invalid)/len(robot):.4%})")
    low_conf = [r for r in robot if "pre_first_icp_low_confidence" in r["quality_flags"]]
    log(f"- {len(low_conf)} rows flagged pre_first_icp_low_confidence (startup window, first ~2s)")
    with (out_dir / "invalid_segments.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "quality_flags"])
        for r in robot:
            if r["valid_pose"] != "1" or r["quality_flags"] != "ok":
                w.writerow([r["timestamp"], r["quality_flags"]])

    # ── Figures ──
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot(x, y, linewidth=0.6)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.axis("equal")
        ax.set_title("GT V2 ICE_RINK — optimized XY trajectory")
        ax.grid(True, alpha=0.4)
        plt.tight_layout(); plt.savefig(fig_dir / "xy_trajectory.png", dpi=140); plt.close()

        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axs[0].plot(t, x, linewidth=0.5); axs[0].set_ylabel("x [m]")
        axs[1].plot(t, y, linewidth=0.5); axs[1].set_ylabel("y [m]")
        axs[2].plot(t, z, linewidth=0.5); axs[2].set_ylabel("z [m]"); axs[2].set_xlabel("t [s]")
        for a in axs: a.grid(True, alpha=0.4)
        plt.tight_layout(); plt.savefig(fig_dir / "xyz_vs_time.png", dpi=140); plt.close()

        yaw = [quat_to_yaw(float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"])) for r in traj]
        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(t, [v * 180 / math.pi for v in yaw], linewidth=0.5)
        ax.set_xlabel("t [s]"); ax.set_ylabel("yaw [deg]"); ax.grid(True, alpha=0.4)
        ax.set_title("Yaw vs time")
        plt.tight_layout(); plt.savefig(fig_dir / "yaw_vs_time.png", dpi=140); plt.close()

        speed = [math.hypot(float(r["vx_world"]), float(r["vy_world"])) for r in traj]
        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(t, speed, linewidth=0.5)
        ax.set_xlabel("t [s]"); ax.set_ylabel("speed [m/s]"); ax.grid(True, alpha=0.4)
        ax.set_title("Horizontal speed vs time (graph V(k))")
        plt.tight_layout(); plt.savefig(fig_dir / "speed_vs_time.png", dpi=140); plt.close()

        wz = [float(r["wz_body"]) for r in traj]
        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(t, wz, linewidth=0.5)
        ax.set_xlabel("t [s]"); ax.set_ylabel("yaw rate [rad/s]"); ax.grid(True, alpha=0.4)
        ax.set_title("Yaw rate vs time (graph W(k), bias-corrected gyro)")
        plt.tight_layout(); plt.savefig(fig_dir / "yaw_rate_vs_time.png", dpi=140); plt.close()

        accel_proxy = [(speed[i + 1] - speed[i]) / (t[i + 1] - t[i]) if t[i+1] > t[i] else 0.0 for i in range(len(speed) - 1)]
        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(t[:-1], accel_proxy, linewidth=0.4)
        ax.set_xlabel("t [s]"); ax.set_ylabel("d(speed)/dt [m/s^2]"); ax.grid(True, alpha=0.4)
        ax.set_title("Horizontal acceleration proxy (finite-diff of graph speed)")
        plt.tight_layout(); plt.savefig(fig_dir / "accel_proxy_vs_time.png", dpi=140); plt.close()

        fig, ax = plt.subplots(figsize=(8, 4))
        ax.hist(innov_trans, bins=80)
        ax.set_xlabel("ICP innovation |meas - estimate| [m]"); ax.set_ylabel("count")
        ax.set_title("ICP prior innovation histogram")
        plt.tight_layout(); plt.savefig(fig_dir / "icp_innovation_hist.png", dpi=140); plt.close()

        fig, ax = plt.subplots(figsize=(8, 4))
        all_rot = [float(r["return_rot_err_deg"]) for r in loops]
        ax.hist(all_rot, bins=40)
        ax.set_xlabel("revisit rotation difference [deg]"); ax.set_ylabel("count")
        ax.set_title("Revisit heading-difference distribution (same-heading vs crossing)")
        plt.tight_layout(); plt.savefig(fig_dir / "revisit_heading_hist.png", dpi=140); plt.close()

        print(f"Figures written to {FIG_DIR}")
    except ImportError:
        log("\n(matplotlib not available — figures skipped)")

    # ── Quality summary YAML ──
    summary = {
        "n_keyframes": len(t),
        "dt_mean_ms": dt_mean * 1000,
        "dt_std_ms": dt_std * 1000,
        "temporal_gaps": n_gaps,
        "non_monotonic": n_nonmonotonic,
        "max_position_step_m": max_step,
        "pose_twist_residual_mean_mps": statistics.mean(resid),
        "pose_angular_rate_residual_mean_radps": statistics.mean(w_resid),
        "icp_innovation_mean_m": statistics.mean(innov_trans),
        "icp_innovation_p95_m": sorted(innov_trans)[int(0.95 * len(innov_trans))],
        "rigidity_hesai_std_m": statistics.pstdev(dists),
        "articulation_residual_mean_deg": statistics.mean(phi_resid) * 180 / math.pi,
        "revisit_same_heading_count": len(same_heading),
        "revisit_same_heading_mean_trans_err_m": statistics.mean(same_heading) if same_heading else None,
        "invalid_pose_fraction": len(invalid) / len(robot),
    }
    (out_dir / "quality_summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False))
    (out_dir / "qualification_report.md").write_text("\n".join(report_lines) + "\n")

    print(f"\nWrote {out_dir / 'qualification_report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
