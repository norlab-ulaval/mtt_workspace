#!/usr/bin/env python3
"""
Calibrate hesai_lidar_joint (reference_point → hesai_lidar) from a static bag.

Determines the 3D position of the Hesai LiDAR relative to reference_point by
detecting known robot geometry in the Hesai point cloud:

  Z-axis  — ground plane (flat floor, robot parked)
  Y-axis  — left/right track symmetry (equidistance from robot centreline)
  X-axis  — wall scan OR robot front face detection (most important: cage shift)

The yaw of hesai_lidar_joint is preserved from --template and is
NOT recalibrated here. Roll/pitch, however, ARE recalibrated from the ground
plane normal (see calibrate_roll_pitch): earlier versions hardcoded roll=pitch=0
assuming a perfectly level mount, which forced the ICP mapper to run with
force4DOF=0 (free roll/pitch) to compensate for the unmodeled residual tilt —
see docs discussion of the "bowl" map artifact (z drift over long traverses).

Usage
-----
# z + y only (any static bag, no wall needed):
  python3 scripts/calib_hesai_scan.py /path/to/bag.mcap

# z + y + x via wall in FRONT of robot, bumper ~1.20 m from wall:
  python3 scripts/calib_hesai_scan.py /path/to/bag.mcap --wall-dist 1.20

# z + y + x via wall, front bumper TOUCHING the wall (dist = 0):
  python3 scripts/calib_hesai_scan.py /path/to/bag.mcap --wall-touch

# Write a candidate for review (never overwrite the measured calibration):
  python3 scripts/calib_hesai_scan.py /path/to/bag.mcap --wall-dist 1.20 \
          --output artifacts/calib_v2_candidate.xacro

Coordinate conventions
-----------------------
hesai_lidar frame (rpy="0 0 π/2" from reference_point):
  X_hesai = +Y_base  (LEFT  of robot)
  Y_hesai = -X_base  (BACKWARD of robot)
  Z_hesai = +Z_base  (UP)

Ground at z_base=0 appears at z_hesai = -hesai_z_in_base ≈ -0.88 m.
Wall in FRONT appears at y_hesai ≈ -(hesai_x_in_base + dist_to_wall).
Left/right tracks appear at x_hesai = ±(track_y_halfwidth - hesai_y_in_base).

reference_point position in base_link (from URDF, stable):
  xyz = (-0.7175, +0.2160, +0.5233)

Robot body geometry constants (approximate, from URDF mesh extents):
  FRONT_BUMPER_X_IN_BASE  ≈ +0.85 m   (forward from base_link)
  TRACK_Y_HALFWIDTH       ≈  0.31 m   (half track separation, left = +, right = -)
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path
from typing import Optional

import numpy as np

from lib.calibration_xacro import calibration_properties, write_candidate

DEFAULT_TEMPLATE = Path(__file__).resolve().parents[1] / 'src/mtt_core/mtt_description/urdf/calibrations/calib_v2.xacro'

# ── URDF-derived constants ───────────────────────────────────────────────────
REFERENCE_POINT_IN_BASE = np.array([-0.71748, 0.21598, 0.52330])
FRONT_BUMPER_X_IN_BASE  = 0.85   # m from base_link (adjust if known more precisely)
TRACK_Y_HALFWIDTH       = 0.31   # m half-separation of track outer walls

# Known v1 values (used as sanity-check reference)
V1_XYZ = np.array([0.62, -0.266, 0.355])

HESAI_TOPIC = "/hesai_lidar/points"


# ── ROS2 / MCAP helpers ──────────────────────────────────────────────────────

def _resolve_bag_path(bag_path: Path) -> Path:
    """Accept a session dir, bag dir, or .mcap file — return the bag directory."""
    if bag_path.is_file():
        return bag_path.parent
    if (bag_path / "metadata.yaml").exists():
        return bag_path
    if (bag_path / "bag" / "metadata.yaml").exists():
        return bag_path / "bag"
    raise FileNotFoundError(
        f"Cannot find metadata.yaml under {bag_path}. "
        "Pass a session dir, bag dir, or .mcap file.")


def load_pointcloud_frames(bag_path: Path, topic: str, max_frames: int = 10):
    """Return list of Nx3 float32 arrays from a PointCloud2 topic in an MCAP bag."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        print("ERROR: rosbag2_py not found — run inside the ROS2 container.")
        sys.exit(1)

    bag_path = _resolve_bag_path(bag_path)
    print(f"  Bag dir resolved: {bag_path}")

    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    reader.open(storage, rosbag2_py.ConverterOptions("", ""))

    type_map = {i.name: i.type for i in reader.get_all_topics_and_types()}
    if topic not in type_map:
        print(f"ERROR: topic {topic!r} not found in bag.")
        print(f"Available topics: {sorted(type_map)}")
        sys.exit(1)

    msg_type = get_message(type_map[topic])
    frames = []
    while reader.has_next() and len(frames) < max_frames:
        t, raw, _ = reader.read_next()
        if t != topic:
            continue
        msg = deserialize_message(raw, msg_type)
        pts = _decode_pointcloud2(msg)
        if pts is not None and len(pts) > 100:
            frames.append(pts)
    return frames


def _decode_pointcloud2(msg) -> Optional[np.ndarray]:
    """Decode sensor_msgs/PointCloud2 into Nx3 float32 array (x,y,z)."""
    x_off = y_off = z_off = None
    fmt_chars = {1: 'b', 2: 'B', 3: 'h', 4: 'H', 5: 'i', 6: 'I', 7: 'f', 8: 'd'}
    for field in msg.fields:
        if field.name == 'x':
            x_off = field.offset
        elif field.name == 'y':
            y_off = field.offset
        elif field.name == 'z':
            z_off = field.offset
    if None in (x_off, y_off, z_off):
        return None

    step = msg.point_step
    data = bytes(msg.data)
    n = len(data) // step
    pts = np.zeros((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        pts[i, 0] = struct.unpack_from('f', data, base + x_off)[0]
        pts[i, 1] = struct.unpack_from('f', data, base + y_off)[0]
        pts[i, 2] = struct.unpack_from('f', data, base + z_off)[0]

    # Remove NaN / Inf
    valid = np.all(np.isfinite(pts), axis=1)
    return pts[valid]


# ── Geometry helpers ─────────────────────────────────────────────────────────

def ransac_plane(pts: np.ndarray, n_iter: int = 500, thr: float = 0.02,
                 min_inliers: int = 100) -> Optional[tuple[np.ndarray, float]]:
    """RANSAC plane fit.  Returns (normal, offset) where normal·p = offset."""
    rng = np.random.default_rng(42)
    best_normal, best_offset, best_count = None, 0.0, 0
    for _ in range(n_iter):
        idx = rng.choice(len(pts), 3, replace=False)
        p = pts[idx]
        v1, v2 = p[1] - p[0], p[2] - p[0]
        n = np.cross(v1, v2)
        norm = np.linalg.norm(n)
        if norm < 1e-6:
            continue
        n /= norm
        d = float(n @ p[0])
        dist = np.abs(pts @ n - d)
        count = int((dist < thr).sum())
        if count > best_count:
            best_normal, best_offset, best_count = n.copy(), d, count
    if best_count < min_inliers:
        return None
    inliers = pts[np.abs(pts @ best_normal - best_offset) < thr]
    best_normal, _, _, _ = np.linalg.lstsq(inliers, np.ones(len(inliers)), rcond=None)
    norm = np.linalg.norm(best_normal)
    best_normal /= norm
    best_offset = float(best_normal @ inliers.mean(axis=0))
    return best_normal, best_offset


# ── Calibration routines ─────────────────────────────────────────────────────

def calibrate_z(cloud: np.ndarray, v1_xyz: np.ndarray) -> Optional[float]:
    """Ground plane → Hesai z above ground → hesai_z_in_reference.

    Raw Hesai frame: X=FORWARD, Y=LEFT, Z=UP.
    Ground is at z < 0 (below sensor at ~0.88 m height).
    We avoid the front wall (robot touching wall → wall at x≈+1 m) by
    looking only at x < 0 (rear of robot), and use a tight z band so
    side walls (vertical planes at ±y) don't fool RANSAC.
    """
    mask = (cloud[:, 2] < -0.60) & (cloud[:, 2] > -1.10) & \
           (cloud[:, 0] > -4.0) & (cloud[:, 0] < 0.5) & \
           (np.abs(cloud[:, 1]) < 2.5)
    gnd = cloud[mask]
    if len(gnd) < 200:
        print(f"  [Z] Not enough ground points ({len(gnd)}). Skipping z calibration.")
        return None

    result = ransac_plane(gnd)
    if result is None:
        print("  [Z] RANSAC failed on ground plane.")
        return None
    normal, offset = result

    # Ground normal should be mostly Z (up) in hesai frame
    if abs(normal[2]) < 0.85:
        print(f"  [Z] Ground plane normal {normal} looks wrong (not upward). Skipping.")
        return None

    # offset = n · p; since n ≈ [0,0,1], offset ≈ z_ground_in_hesai
    z_ground_in_hesai = offset / normal[2]
    z_hesai_in_base = -z_ground_in_hesai
    z_hesai_in_ref  = z_hesai_in_base - REFERENCE_POINT_IN_BASE[2]

    print(f"  [Z] ground z in hesai frame : {z_ground_in_hesai:+.4f} m")
    print(f"  [Z] → hesai z above ground  : {z_hesai_in_base:.4f} m")
    print(f"  [Z] → hesai_calib_xyz[2]    : {z_hesai_in_ref:+.4f}  (v1={v1_xyz[2]:+.4f})")
    return z_hesai_in_ref


def calibrate_roll_pitch(cloud: np.ndarray) -> Optional[tuple[float, float, np.ndarray]]:
    """Ground plane normal → residual mounting roll/pitch (rad).

    Raw Hesai frame: X=FORWARD, Y=LEFT, Z=UP. If the sensor were perfectly
    level, the ground plane normal in this frame would be exactly (0,0,1).
    A small mounting tilt (roll about X, pitch about Y) shows up directly as
    a small (nx, ny) deviation. First-order derivation (R_sensor_to_world =
    Rz(yaw)*Ry(pitch)*Rx(roll), yaw does not affect this since it acts last):

        normal_measured ≈ (-sin(pitch), sin(roll), cos(pitch)*cos(roll))
                        ≈ (-pitch, roll, 1)          for small angles

    → roll  = atan2(ny, nz)
      pitch = atan2(-nx, sqrt(ny^2 + nz^2))

    Uses the same ground-point mask as calibrate_z() (rear of robot, tight z
    band to avoid side walls polluting the fit).
    """
    mask = (cloud[:, 2] < -0.60) & (cloud[:, 2] > -1.10) & \
           (cloud[:, 0] > -4.0) & (cloud[:, 0] < 0.5) & \
           (np.abs(cloud[:, 1]) < 2.5)
    gnd = cloud[mask]
    if len(gnd) < 200:
        print(f"  [RollPitch] Not enough ground points ({len(gnd)}). Skipping.")
        return None

    result = ransac_plane(gnd)
    if result is None:
        print("  [RollPitch] RANSAC failed on ground plane.")
        return None
    normal, _offset = result

    if normal[2] < 0:
        normal = -normal  # RANSAC sign is arbitrary; force "up-ish"

    if abs(normal[2]) < 0.85:
        print(f"  [RollPitch] Ground plane normal {normal} looks wrong (not upward). Skipping.")
        return None

    roll  = math.atan2(normal[1], normal[2])
    pitch = math.atan2(-normal[0], math.hypot(normal[1], normal[2]))

    print(f"  [RollPitch] ground plane normal (raw hesai frame): "
          f"({normal[0]:+.5f}, {normal[1]:+.5f}, {normal[2]:+.5f})")
    print(f"  [RollPitch] → roll  = {roll:+.5f} rad ({math.degrees(roll):+.3f} deg)")
    print(f"  [RollPitch] → pitch = {pitch:+.5f} rad ({math.degrees(pitch):+.3f} deg)")

    max_sane_deg = 5.0
    if abs(math.degrees(roll)) > max_sane_deg or abs(math.degrees(pitch)) > max_sane_deg:
        print(f"  [RollPitch] WARNING: |angle| > {max_sane_deg} deg — this is larger than a "
              "plausible mounting tilt. The ground mask likely picked up a wall or the "
              "trailer instead of the floor. NOT applying — keeping roll=pitch=0. "
              "Re-run with the robot on a visibly flat, obstruction-free floor.")
        return None

    return roll, pitch, normal


def calibrate_y(cloud: np.ndarray, v1_xyz: np.ndarray) -> Optional[float]:
    """Track symmetry → hesai y offset from robot centreline.

    Raw Hesai frame: X=FORWARD, Y=LEFT, Z=UP.
    Tracks are lateral: left track at y ≈ +TRACK_Y_HALFWIDTH (from robot centre),
    right track at y ≈ -TRACK_Y_HALFWIDTH.
    If sensor is offset by y_hesai_in_base to the LEFT, tracks shift by -y_hesai_in_base.
    centroid = (y_left + y_right) / 2 = -y_hesai_in_base
    """
    mask = (cloud[:, 2] > -0.90) & (cloud[:, 2] < -0.30) & \
           (np.abs(cloud[:, 1]) > 0.15) & (np.abs(cloud[:, 1]) < 0.60) & \
           (cloud[:, 0] > -2.00) & (cloud[:, 0] < 2.00)
    side_pts = cloud[mask]
    if len(side_pts) < 50:
        print(f"  [Y] Not enough track-side points ({len(side_pts)}). Skipping y calibration.")
        return None

    left  = side_pts[side_pts[:, 1] > 0]   # left track  (+Y = LEFT in raw frame)
    right = side_pts[side_pts[:, 1] < 0]   # right track (-Y = RIGHT in raw frame)
    if len(left) < 30 or len(right) < 30:
        print(f"  [Y] Insufficient left ({len(left)}) or right ({len(right)}) track points.")
        return None

    y_left_median  = float(np.median(left[:, 1]))
    y_right_median = float(np.median(right[:, 1]))

    centroid = (y_left_median + y_right_median) / 2.0
    y_hesai_in_base = -centroid
    y_hesai_in_ref  = y_hesai_in_base - REFERENCE_POINT_IN_BASE[1]

    print(f"  [Y] left track y in raw data : {y_left_median:+.4f} m")
    print(f"  [Y] right track y in raw data: {y_right_median:+.4f} m")
    print(f"  [Y] centreline offset         : {centroid:+.4f} m")
    print(f"  [Y] → hesai_calib_xyz[1]     : {y_hesai_in_ref:+.4f}  (v1={v1_xyz[1]:+.4f})")
    return y_hesai_in_ref


def calibrate_x_wall(cloud: np.ndarray, wall_dist_m: float,
                     v1_xyz: np.ndarray) -> Optional[float]:
    """Wall scan → hesai x offset.

    Raw Hesai frame: X=FORWARD, Y=LEFT, Z=UP.
    Wall in front of robot is at POSITIVE X in raw data.
    Sensor origin = physical hesai position.

    x_wall_in_raw = x_wall_in_base - x_hesai_in_base   (both in base_link)
    → x_hesai_in_base = x_wall_in_base - x_wall_in_raw

    RANSAC plane: n · p = offset  →  x_wall_in_raw = offset / normal[0]
    (normal[0] < 0 when RANSAC picks outward normal toward sensor,
     and offset < 0 accordingly, giving positive x_wall_in_raw).
    """
    x_wall_in_base = FRONT_BUMPER_X_IN_BASE + wall_dist_m
    print(f"  [X] Wall expected at x_base = {x_wall_in_base:.3f} m "
          f"(bumper {FRONT_BUMPER_X_IN_BASE:.2f} + dist {wall_dist_m:.2f})")

    # Wall in front: x > 0.3, z in plausible range, |y| wide
    mask = (cloud[:, 0] > 0.30) & (cloud[:, 0] < 3.00) & \
           (cloud[:, 2] > -0.70) & (cloud[:, 2] < 0.50) & \
           (np.abs(cloud[:, 1]) < 2.50)
    wall_pts = cloud[mask]
    if len(wall_pts) < 100:
        print(f"  [X] Not enough wall points ({len(wall_pts)}). Skipping x calibration.")
        return None

    result = ransac_plane(wall_pts)
    if result is None:
        print("  [X] RANSAC failed on wall plane.")
        return None
    normal, offset = result

    # Wall must be X-facing (front wall has large X component in normal)
    if abs(normal[0]) < 0.85:
        print(f"  [X] Wall normal {normal} does not look like a front wall (need |nx|>0.85). Skipping.")
        return None

    x_wall_in_raw = offset / normal[0]   # signed distance from sensor to wall along X

    x_hesai_in_base = x_wall_in_base - x_wall_in_raw
    x_hesai_in_ref  = x_hesai_in_base - REFERENCE_POINT_IN_BASE[0]

    print(f"  [X] wall x in raw data      : {x_wall_in_raw:+.4f} m")
    print(f"  [X] → hesai x in base_link  : {x_hesai_in_base:+.4f} m")
    print(f"  [X] → hesai_calib_xyz[0]    : {x_hesai_in_ref:+.4f}  (v1={v1_xyz[0]:+.4f})")
    delta = x_hesai_in_ref - v1_xyz[0]
    print(f"  [X] Δ from v1 (cage shift)  : {delta:+.4f} m  ({delta*100:.1f} cm)")
    return x_hesai_in_ref


def calibrate_x_front_face(cloud: np.ndarray, v1_xyz: np.ndarray) -> Optional[float]:
    """Detect the robot's front face in the Hesai scan (no wall needed).

    The robot front body panel is visible in the Hesai scan as a vertical
    plane behind the sensor (in the +Y_hesai = -X_base direction).
    We look for a vertical planar surface in the region y_hesai > 0
    (which is -X_base < 0 = behind the sensor = in front of the robot).

    This method is less reliable than the wall scan but requires no extra setup.
    """
    # Raw frame: X=FORWARD, Y=LEFT, Z=UP.
    # Robot front face is at POSITIVE x in raw data (in front of sensor).
    # Looking for front panel: x > 0.1, z range, |y| narrow (body width ~0.7m)

    mask = (cloud[:, 0] > 0.10) & (cloud[:, 0] < 2.00) & \
           (cloud[:, 2] > -0.70) & (cloud[:, 2] < 0.30) & \
           (np.abs(cloud[:, 1]) < 0.70)
    front_pts = cloud[mask]
    if len(front_pts) < 80:
        print(f"  [X-face] Not enough front-face points ({len(front_pts)}).")
        return None

    result = ransac_plane(front_pts)
    if result is None:
        print("  [X-face] RANSAC failed.")
        return None
    normal, offset = result

    if abs(normal[0]) < 0.80:
        print(f"  [X-face] Dominant plane normal {normal} is not a front face (need |nx|>0.80). Skipping.")
        return None

    x_front_in_raw = offset / normal[0]
    # x_front_in_raw = x_front_in_base - x_hesai_in_base
    # → x_hesai_in_base = x_front_in_base - x_front_in_raw = BUMPER - x_front_in_raw
    x_hesai_in_base = FRONT_BUMPER_X_IN_BASE - x_front_in_raw
    x_hesai_in_ref  = x_hesai_in_base - REFERENCE_POINT_IN_BASE[0]

    print(f"  [X-face] front panel x in raw  : {x_front_in_raw:+.4f} m")
    print(f"  [X-face] → hesai x in base_link: {x_hesai_in_base:+.4f} m")
    print(f"  [X-face] → hesai_calib_xyz[0]  : {x_hesai_in_ref:+.4f}  (v1={V1_XYZ[0]:+.4f})")
    print(f"  [X-face]   NOTE: uses FRONT_BUMPER_X_IN_BASE={FRONT_BUMPER_X_IN_BASE:.2f} m "
          "(edit constant at top of script if wrong)")
    delta = x_hesai_in_ref - v1_xyz[0]
    print(f"  [X-face] Δ from v1 (cage shift): {delta:+.4f} m  ({delta*100:.1f} cm)")
    return x_hesai_in_ref


# ── xacro output ────────────────────────────────────────────────────────────

def write_xacro(path: Path, xyz: np.ndarray, roll: float, pitch: float,
                template: Path = DEFAULT_TEMPLATE) -> None:
    write_candidate(template, path, xyz, roll, pitch)
    print(f"\nWrote unvalidated candidate {path}; review before deployment")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    global FRONT_BUMPER_X_IN_BASE  # must be first — used in default= below
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path, help="Path to MCAP bag or bag directory")
    parser.add_argument("--topic", default=HESAI_TOPIC,
                        help=f"Hesai PointCloud2 topic (default: {HESAI_TOPIC})")
    parser.add_argument("--frames", type=int, default=5,
                        help="Number of scan frames to average (default: 5)")
    parser.add_argument("--wall-dist", type=float, default=None,
                        help="Distance from front bumper to wall face [m]. Enables x calibration.")
    parser.add_argument("--wall-touch", action="store_true",
                        help="Front bumper is touching the wall (wall-dist=0). Enables x calibration.")
    parser.add_argument("--front-face", action="store_true",
                        help="Try to detect robot front face in scan (no wall, less reliable).")
    parser.add_argument("--bumper-x", type=float, default=FRONT_BUMPER_X_IN_BASE,
                        help=f"Front bumper x in base_link [m] (default: {FRONT_BUMPER_X_IN_BASE})")
    parser.add_argument("--output", type=Path, default=None,
                        help="Write a complete calibration candidate to a NEW path; never overwrite")
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE,
                        help="Complete calibration whose yaw and other sensor mounts are preserved")
    args = parser.parse_args()

    if args.output and args.output.exists():
        parser.error('--output already exists; choose a new candidate path')
    _, template_props = calibration_properties(args.template)
    template_xyz = np.array([float(v) for v in template_props['hesai_calib_xyz'].attrib['value'].split()])
    template_rpy = [float(v) for v in template_props['hesai_calib_rpy'].attrib['value'].split()]
    yaw = template_rpy[2]

    FRONT_BUMPER_X_IN_BASE = args.bumper_x

    if args.wall_touch:
        args.wall_dist = 0.0

    print(f"\n=== Hesai scan calibration ===")
    print(f"Bag   : {args.bag}")
    print(f"Topic : {args.topic}")
    print(f"v1 ref: xyz={V1_XYZ}")

    # Load scans
    print(f"\nLoading up to {args.frames} frames from {args.topic} …")
    frames = load_pointcloud_frames(args.bag, args.topic, args.frames)
    if not frames:
        print("ERROR: no valid frames found.")
        sys.exit(1)
    print(f"Loaded {len(frames)} frames, averaging …")
    # Downsample each frame to 20 000 pts max then stack
    combined = np.vstack([
        f[np.random.choice(len(f), min(len(f), 20_000), replace=False)]
        for f in frames
    ])
    print(f"Combined cloud: {len(combined)} points\n")

    xyz = template_xyz.copy()
    result_z = calibrate_z(combined, V1_XYZ)
    if result_z is not None:
        xyz[2] = result_z

    print()
    result_y = calibrate_y(combined, V1_XYZ)
    if result_y is not None:
        xyz[1] = result_y

    print()
    result_x = None
    if args.wall_dist is not None:
        result_x = calibrate_x_wall(combined, args.wall_dist, V1_XYZ)
    elif args.front_face:
        result_x = calibrate_x_front_face(combined, V1_XYZ)
    else:
        print("  [X] No x calibration requested (use --wall-dist or --front-face).")
        print(f"  [X] Keeping template value: {template_xyz[0]:.6f}")

    if result_x is not None:
        xyz[0] = result_x

    print()
    roll, pitch = template_rpy[:2]
    result_rp = calibrate_roll_pitch(combined)
    if result_rp is not None:
        roll, pitch, _normal = result_rp

    print("\n=== Result ===")
    delta = xyz - V1_XYZ
    print(f"  hesai_calib_xyz  = \"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}\"")
    print(f"  Δ from v1        = ({delta[0]:+.4f}, {delta[1]:+.4f}, {delta[2]:+.4f})")
    print(f"  Sensor shift ΔX = {delta[0]*100:+.1f} cm along reference_point +X")
    print(f"  hesai_calib_rpy  = \"{roll:.6f} {pitch:.6f} {yaw:.6f}\"  "
          f"(roll={math.degrees(roll):+.3f} deg, pitch={math.degrees(pitch):+.3f} deg)")
    if result_rp is None:
        print("  NOTE: roll/pitch estimation was skipped or rejected; template tilt preserved.")

    if args.output:
        write_xacro(args.output, xyz, roll, pitch, args.template)
    else:
        print("\nCandidate values only: verify the cloud frame and physical measurements before use.")
        print(f'  <xacro:property name="hesai_calib_xyz" '
              f'value="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}"/>')
        print(f'  <xacro:property name="hesai_calib_rpy" '
              f'value="{roll:.6f} {pitch:.6f} {yaw:.6f}"/>')


if __name__ == "__main__":
    main()
