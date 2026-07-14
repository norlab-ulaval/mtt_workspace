#!/usr/bin/env python3
"""Generate visual diagnostics for motion research datasets and model suites."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from lib.mtt_motion_research import finite_stats, parse_bool, read_csv_rows, resolve_sessions


def dataset_csv_for_target(target: Path) -> Path:
    if (target / "dataset.csv").exists():
        return target / "dataset.csv"
    return target / "motion_research" / "dataset.csv"


def target_name(target: Path) -> str:
    return target.parent.name if target.name == "motion_research" else target.name


def resolve_targets(path: Path) -> list[Path]:
    path = path.expanduser().resolve()
    if (path / "dataset.csv").exists():
        return [path]
    direct = sorted(p for p in path.glob("*/dataset.csv"))
    if direct:
        return [p.parent for p in direct]
    return resolve_sessions(path)


def plot_session(session: Path, out_dir: Path) -> dict[str, str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}
    dataset_csv = dataset_csv_for_target(session)
    rows = read_csv_rows(dataset_csv)
    outputs: dict[str, str] = {}
    t0 = float(rows[0]["t"]) if rows else 0.0
    valid = [row for row in rows if parse_bool(row.get("research_quality_ok"))]

    colors = {
        "straight": "tab:green",
        "constant_turn": "tab:orange",
        "s_curve": "tab:purple",
        "aggressive_turn": "tab:red",
        "high_accel": "tab:brown",
        "stop": "tab:gray",
        "mild_motion": "tab:blue",
    }
    plt.figure(figsize=(9, 8))
    for klass, color in colors.items():
        subset = [row for row in rows if row.get("motion_class") == klass]
        if subset:
            passed = [row for row in subset if parse_bool(row.get("research_quality_ok"))]
            weak = [row for row in subset if not parse_bool(row.get("research_quality_ok"))]
            if passed:
                plt.scatter([float(r["icp_x"]) for r in passed], [float(r["icp_y"]) for r in passed], s=4, c=color, label=klass, alpha=0.75)
            if weak:
                plt.scatter([float(r["icp_x"]) for r in weak], [float(r["icp_y"]) for r in weak], s=3, c=color, alpha=0.18)
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid(True, alpha=0.3)
    plt.legend(markerscale=3, fontsize=8)
    plt.tight_layout()
    path = out_dir / "segments_xy.png"
    plt.savefig(path, dpi=170)
    plt.close()
    outputs[path.name] = str(path)

    delta_vals = [float(r.get("delta_kappa_exact", 0.0)) for r in valid if abs(float(r.get("delta_kappa_exact", 0.0))) < 5.0]
    if delta_vals:
        plt.figure(figsize=(11, 5))
        plt.hist(delta_vals, bins=80, color="tab:blue", alpha=0.75)
        plt.axvline(0.0, color="black", linewidth=1.0)
        plt.xlabel("delta kappa exact [1/m]")
        plt.ylabel("sample count")
        plt.title("Curvature residual distribution")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        path = out_dir / "delta_kappa_histogram.png"
        plt.savefig(path, dpi=170)
        plt.close()
        outputs[path.name] = str(path)

    plt.figure(figsize=(12, 6))
    class_names = [klass for klass in colors if any(row.get("motion_class") == klass for row in valid)]
    class_data = [
        [float(row.get("delta_kappa_exact", 0.0)) for row in valid if row.get("motion_class") == klass and abs(float(row.get("delta_kappa_exact", 0.0))) < 5.0]
        for klass in class_names
    ]
    class_data = [data if data else [0.0] for data in class_data]
    if class_names:
        plt.boxplot(class_data, labels=class_names, showfliers=True)
        plt.ylabel("delta kappa exact [1/m]")
        plt.title("Curvature residual by motion class")
        plt.grid(True, axis="y", alpha=0.3)
        plt.xticks(rotation=30, ha="right")
        plt.tight_layout()
        path = out_dir / "delta_kappa_by_motion_class_boxplot.png"
        plt.savefig(path, dpi=170)
        plt.close()
        outputs[path.name] = str(path)

    plt.figure(figsize=(12, 6))
    for name, key in (("roll", "imu_roll_rad"), ("pitch", "imu_pitch_rad")):
        vals = [float(r.get(key, 0.0)) for r in valid]
        if vals:
            vals_centered = [v - sorted(vals)[len(vals) // 2] for v in vals]
            plt.hist(vals_centered, bins=80, histtype="step", linewidth=1.8, label=f"{name} centered")
    plt.xlabel("centered angle [rad]")
    plt.ylabel("sample count")
    plt.title("IMU roll/pitch distribution around session median")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    path = out_dir / "imu_roll_pitch_centered_histogram.png"
    plt.savefig(path, dpi=170)
    plt.close()
    outputs[path.name] = str(path)

    rel_t = [float(row["t"]) - t0 for row in rows]
    plt.figure(figsize=(13, 8))
    ax1 = plt.subplot(4, 1, 1)
    ax1.plot(rel_t, [float(row.get("tacho_signed_ms", 0.0)) for row in rows], label="tacho/derived speed")
    ax1.plot(rel_t, [float(row.get("cmd_speed_ms", 0.0)) for row in rows], label="cmd speed", alpha=0.8)
    ax1.set_ylabel("v [m/s]")
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax2 = plt.subplot(4, 1, 2, sharex=ax1)
    ax2.plot(rel_t, [float(row.get("phi_rad", 0.0)) for row in rows], label="phi measured")
    ax2.plot(rel_t, [float(row.get("cmd_phi_rad", 0.0)) for row in rows], label="phi cmd", alpha=0.8)
    ax2.set_ylabel("phi [rad]")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax3 = plt.subplot(4, 1, 3, sharex=ax1)
    ax3.plot(rel_t, [float(row.get("delta_kappa_exact", 0.0)) for row in rows], label="delta kappa exact")
    ax3.set_ylabel("d kappa [1/m]")
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)
    ax4 = plt.subplot(4, 1, 4, sharex=ax1)
    ax4.plot(rel_t, [float(row.get("imu_roll_rad", 0.0)) for row in rows], label="roll")
    ax4.plot(rel_t, [float(row.get("imu_pitch_rad", 0.0)) for row in rows], label="pitch")
    ax4.plot(rel_t, [float(row.get("imu_yaw_rate_rad_s", 0.0)) for row in rows], label="imu yaw rate", alpha=0.7)
    ax4.set_xlabel("time [s]")
    ax4.set_ylabel("IMU")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)
    plt.tight_layout()
    path = out_dir / "signals_residual_imu.png"
    plt.savefig(path, dpi=170)
    plt.close()
    outputs[path.name] = str(path)

    plt.figure(figsize=(13, 4))
    plt.scatter([abs(float(r.get("phi_rad", 0.0))) for r in valid], [float(r.get("delta_kappa_exact", 0.0)) for r in valid], s=8, alpha=0.45)
    plt.xlabel("|articulation| [rad]")
    plt.ylabel("delta kappa exact [1/m]")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path = out_dir / "residual_vs_articulation.png"
    plt.savefig(path, dpi=170)
    plt.close()
    outputs[path.name] = str(path)

    plt.figure(figsize=(13, 4))
    plt.scatter([float(r.get("imu_roll_rad", 0.0)) for r in valid], [float(r.get("delta_kappa_exact", 0.0)) for r in valid], s=8, alpha=0.45, label="roll")
    plt.scatter([float(r.get("imu_pitch_rad", 0.0)) for r in valid], [float(r.get("delta_kappa_exact", 0.0)) for r in valid], s=8, alpha=0.45, label="pitch")
    plt.xlabel("roll/pitch [rad]")
    plt.ylabel("delta kappa exact [1/m]")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    path = out_dir / "residual_vs_roll_pitch.png"
    plt.savefig(path, dpi=170)
    plt.close()
    outputs[path.name] = str(path)
    return outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sessions = resolve_targets(args.path)
    summaries = []
    for session in sessions:
        dataset_csv = dataset_csv_for_target(session)
        if not dataset_csv.exists():
            print(f"[skip] missing {dataset_csv}")
            continue
        name = target_name(session)
        out_dir = (args.output_dir / name if args.output_dir else dataset_csv.parent / "figures")
        out_dir.mkdir(parents=True, exist_ok=True)
        outputs = plot_session(session, out_dir)
        rows = read_csv_rows(dataset_csv)
        valid = [row for row in rows if parse_bool(row.get("research_quality_ok"))]
        summary = {
            "session": name,
            "rows": len(rows),
            "valid_rows": len(valid),
            "delta_kappa_exact": finite_stats(float(row.get("delta_kappa_exact", 0.0)) for row in valid),
            "outputs": outputs,
        }
        (out_dir / "report_summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
        summaries.append(summary)
    print(yaml.safe_dump({"sessions": len(summaries), "summaries": summaries}, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
