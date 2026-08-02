#!/usr/bin/env python3
"""Reshape a 100Hz robot-pose CSV (scripts/build_gt_v2_100hz.py's
pose_robot_gt_100hz.csv schema) into the operational-reference CSV that
scripts/build_session_dataset.py's --reference expects: time_s-indexed,
full gt_track_* pose/twist/derived/quality columns, already transported to
track_contact_center. Reuses add_derived_reference()/TRACK_IN_BASE_M from
build_msa_canonical_dataset.py rather than re-deriving that math -- do not
duplicate it here if that function's signature ever changes, update the
call site instead.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from build_msa_canonical_dataset import TRACK_IN_BASE_M, add_derived_reference


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose", type=Path, required=True,
                         help="pose_robot_gt_100hz.csv from build_gt_v2_100hz.py.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--filter-window-s", type=float, default=0.21)
    parser.add_argument("--filter-polyorder", type=int, default=2)
    parser.add_argument("--curvature-speed-gate-ms", type=float, default=0.2)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    pose = pd.read_csv(args.pose)

    p_base = pose[["x", "y", "z"]].to_numpy(dtype=float)
    q_base = pose[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
    rot = Rotation.from_quat(q_base)
    # Rigid transport base_footprint -> track_contact_center, position only
    # (TRACK_IN_BASE_M is a fixed lever arm in the body frame; orientation
    # of the two points is identical for a rigid body).
    p_track = p_base + rot.apply(TRACK_IN_BASE_M)
    euler = rot.as_euler("xyz")

    reference = pd.DataFrame({
        "time_s": pose["timestamp"].to_numpy(dtype=float),
        "gt_track_x_m": p_track[:, 0],
        "gt_track_y_m": p_track[:, 1],
        "gt_track_z_m": p_track[:, 2],
        "gt_track_qx": q_base[:, 0],
        "gt_track_qy": q_base[:, 1],
        "gt_track_qz": q_base[:, 2],
        "gt_track_qw": q_base[:, 3],
        "gt_track_roll_rad": euler[:, 0],
        "gt_track_pitch_rad": euler[:, 1],
        "gt_track_yaw_rad": euler[:, 2],
        "gt_track_yaw_unwrapped_rad": np.unwrap(euler[:, 2]),
        "gt_track_vx_raw_ms": pose["vx_body"].to_numpy(dtype=float),
        "gt_track_vy_raw_ms": pose["vy_body"].to_numpy(dtype=float),
        "gt_track_vz_raw_ms": pose["vz_body"].to_numpy(dtype=float),
        "gt_track_wx_raw_rad_s": pose["wx_body"].to_numpy(dtype=float),
        "gt_track_wy_raw_rad_s": pose["wy_body"].to_numpy(dtype=float),
        "gt_track_wz_raw_rad_s": pose["wz_body"].to_numpy(dtype=float),
        "gt_sigma_x_m": pose["sigma_x"].to_numpy(dtype=float),
        "gt_sigma_y_m": pose["sigma_y"].to_numpy(dtype=float),
        "gt_sigma_z_m": pose["sigma_z"].to_numpy(dtype=float),
        "gt_sigma_yaw_rad": pose["sigma_yaw"].to_numpy(dtype=float),
        "valid_gt_pose": pose["valid_pose"].astype(bool),
        "gt_quality_flags": pose["quality_flags"].astype(str),
    })

    reference, filter_info = add_derived_reference(
        reference, args.filter_window_s, args.filter_polyorder,
        args.curvature_speed_gate_ms)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    reference.to_csv(args.output, index=False)
    print(f"Wrote {args.output} ({len(reference)} rows). Filter info: {filter_info}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
