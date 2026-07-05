#!/usr/bin/env python3
"""
Validate ZED camera calibration using a visible robot body feature.

The ZED camera is mounted on the sensor cage rigidly relative to the Hesai.
If only the cage x-position changed (cage slid backward), the ZED→hesai
relative transform (zed2i_mount_joint) does NOT change.  This script
cross-validates that assumption and can also refine the ZED mount if needed.

Method
------
1. Extract a ZED depth frame from a bag.
2. The user specifies a 3D point on the robot body that is:
     a. Visible in the ZED image (user sees it in Foxglove RGB panel)
     b. At a known position in base_link (from URDF / tape measure)
3. The script reprojects that point through the current TF chain:
       known_base → hesai_lidar (via hesai_calib_xyz) → zed_camera_link
   and compares the expected depth with the measured depth at that pixel.
4. Reports the 3D residual.  If residual > 3 cm, suggests running
   calibrate_static_bag.py --mode camera_lidar for a full re-calibration.

Additionally: if a ZED depth cloud is available, the script also fits a ground
plane in the ZED frame and compares the measured ZED height above ground with
the value predicted from the URDF chain — a quick sanity check.

Usage
-----
# Ground plane sanity check only:
  python3 scripts/calib_zed_body_validate.py /path/to/bag.mcap

# Cross-validate with a known body feature (e.g. battery-box top corner):
  python3 scripts/calib_zed_body_validate.py /path/to/bag.mcap \
          --body-point 0.45 0.10 0.65 \
          --pixel-u 640 --pixel-v 360 \
          --hesai-xyz "0.58 -0.266 0.355"

Arguments for --body-point: x y z in base_link (metres).
Arguments for --pixel-u/v:  pixel column/row in ZED LEFT image where the
                             feature appears (use Foxglove image panel to find).
Arguments for --hesai-xyz:  updated hesai_calib_xyz from calib_hesai_scan.py
                             (defaults to v1 values).

ZED topics expected in bag
--------------------------
  /zed2i/zed_node/point_cloud/cloud_registered   (PointCloud2 in camera frame)
  /zed2i/zed_node/left/image_rect_color          (sensor_msgs/Image, optional)

ZED mount transform (zed2i_mount_joint, from URDF v1):
  hesai_lidar → zed_camera_link:  xyz="0.0 -0.47 -0.1627"  rpy="0 0 -1.5708"
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

# ── URDF constants ───────────────────────────────────────────────────────────
REFERENCE_POINT_IN_BASE = np.array([-0.71748, 0.21598, 0.52330])
V1_HESAI_XYZ            = np.array([0.62, -0.266, 0.355])

# zed2i_mount_joint: hesai_lidar → zed_camera_link
# xyz="0.0 -0.47 -0.1627" rpy="0 0 -1.5708"
# rpy net: Hesai has +π/2 from reference_point → ZED has −π/2 from Hesai → net=0 (aligned with base_link)
ZED_XYZ_IN_HESAI = np.array([0.0, -0.47, -0.1627])
ZED_RPY_IN_HESAI = np.array([0.0, 0.0, -1.5707963])

ZED_TOPIC_CLOUD = "/zed2i/zed_node/point_cloud/cloud_registered"
HESAI_TOPIC     = "/hesai_lidar/points"


def rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rx(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rpy_to_rot(rpy):
    r, p, y = rpy
    return rz(y) @ ry(p) @ rx(r)


def hesai_xyz_to_base(hesai_calib_xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (R_hesai_in_base, t_hesai_in_base)."""
    R = rz(np.pi / 2)
    t = REFERENCE_POINT_IN_BASE + hesai_calib_xyz  # rpy=0 for reference_point_joint
    return R, t


def zed_in_base(hesai_calib_xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (R_zed_in_base, t_zed_in_base)."""
    R_h, t_h = hesai_xyz_to_base(hesai_calib_xyz)
    R_z_in_h = rpy_to_rot(ZED_RPY_IN_HESAI)
    R_z = R_h @ R_z_in_h
    t_z = t_h + R_h @ ZED_XYZ_IN_HESAI
    return R_z, t_z


def load_one_cloud(bag_path: Path, topic: str) -> np.ndarray | None:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        print("ERROR: rosbag2_py not available — run inside ROS2 container.")
        sys.exit(1)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {i.name: i.type for i in reader.get_all_topics_and_types()}
    if topic not in type_map:
        return None
    msg_type = get_message(type_map[topic])
    while reader.has_next():
        t, raw, _ = reader.read_next()
        if t != topic:
            continue
        msg = deserialize_message(raw, msg_type)
        pts = _decode_xyz(msg)
        if pts is not None and len(pts) > 200:
            return pts
    return None


def _decode_xyz(msg) -> np.ndarray | None:
    x_off = y_off = z_off = None
    for f in msg.fields:
        if f.name == 'x': x_off = f.offset
        if f.name == 'y': y_off = f.offset
        if f.name == 'z': z_off = f.offset
    if None in (x_off, y_off, z_off):
        return None
    step = msg.point_step
    data = bytes(msg.data)
    n = len(data) // step
    pts = np.zeros((n, 3), dtype=np.float32)
    for i in range(n):
        b = i * step
        pts[i] = (
            struct.unpack_from('f', data, b + x_off)[0],
            struct.unpack_from('f', data, b + y_off)[0],
            struct.unpack_from('f', data, b + z_off)[0],
        )
    return pts[np.all(np.isfinite(pts), axis=1)]


def ransac_plane(pts, n_iter=300, thr=0.025):
    rng = np.random.default_rng(0)
    best = (None, 0.0, 0)
    for _ in range(n_iter):
        idx = rng.choice(len(pts), 3, replace=False)
        p = pts[idx]
        n = np.cross(p[1]-p[0], p[2]-p[0])
        nrm = np.linalg.norm(n)
        if nrm < 1e-6: continue
        n /= nrm
        d = float(n @ p[0])
        cnt = int((np.abs(pts @ n - d) < thr).sum())
        if cnt > best[2]:
            best = (n, d, cnt)
    if best[0] is None or best[2] < 50:
        return None
    inl = pts[np.abs(pts @ best[0] - best[1]) < thr]
    n2, _, _, _ = np.linalg.lstsq(inl, np.ones(len(inl)), rcond=None)
    n2 /= np.linalg.norm(n2)
    return n2, float(n2 @ inl.mean(axis=0))


def ground_plane_check(zed_cloud: np.ndarray, R_zed: np.ndarray, t_zed: np.ndarray) -> None:
    """Ground-plane sanity check in ZED frame."""
    print("\n── Ground plane (ZED) ──")
    # ZED looks forward; ground below: in ZED frame (aligned with base) z = -t_zed[2]
    # Filter: x forward (-2 to 5), z below sensor
    mask = (zed_cloud[:, 2] < -0.30) & (zed_cloud[:, 2] > -1.20) & \
           (zed_cloud[:, 0] > 0.20) & (zed_cloud[:, 0] < 4.0) & \
           (np.abs(zed_cloud[:, 1]) < 2.0)
    gnd = zed_cloud[mask]
    if len(gnd) < 100:
        print(f"  Only {len(gnd)} ground candidate points. Skipping.")
        return
    result = ransac_plane(gnd)
    if result is None:
        print("  RANSAC failed.")
        return
    n, d = result
    if abs(n[2]) < 0.85:
        print(f"  Ground normal {n} does not look horizontal. Skipping.")
        return
    z_ground_in_zed = d / n[2]
    z_zed_in_base_measured = -z_ground_in_zed
    z_zed_in_base_urdf = t_zed[2]
    print(f"  Ground z in ZED frame    : {z_ground_in_zed:+.4f} m")
    print(f"  ZED height above ground (measured): {z_zed_in_base_measured:.4f} m")
    print(f"  ZED height above ground (URDF)    : {z_zed_in_base_urdf:.4f} m")
    print(f"  Δz                                : {z_zed_in_base_measured - z_zed_in_base_urdf:+.4f} m")
    if abs(z_zed_in_base_measured - z_zed_in_base_urdf) < 0.03:
        print("  ✓ ZED z calibration looks good (< 3 cm error)")
    else:
        print("  ⚠  ZED z error > 3 cm — verify hesai z calibration first")


def body_point_check(body_pt_base: np.ndarray,
                     pixel_uv: tuple[int, int],
                     zed_cloud: np.ndarray,
                     R_zed: np.ndarray, t_zed: np.ndarray,
                     img_width: int, img_height: int) -> None:
    """Check body feature reprojection against ZED depth cloud."""
    print("\n── Body feature cross-check ──")
    # Transform known point from base to ZED frame
    pt_in_zed = R_zed.T @ (body_pt_base - t_zed)
    print(f"  Known body point in base_link : {body_pt_base}")
    print(f"  Expected position in ZED frame : {pt_in_zed}")

    # Find the ZED cloud point closest to the pixel (u,v)
    # ZED cloud is organised: index = v * width + u
    u, v = pixel_uv
    if img_width <= 0 or img_height <= 0:
        print("  Cannot look up pixel — pass --img-width and --img-height for organised cloud.")
        return
    idx = v * img_width + u
    if idx >= len(zed_cloud):
        print(f"  Pixel ({u},{v}) → index {idx} out of range ({len(zed_cloud)} pts).")
        return

    pt_measured = zed_cloud[idx]
    if not np.all(np.isfinite(pt_measured)):
        print(f"  Pixel ({u},{v}) has no depth (NaN). Try a nearby pixel.")
        return

    residual = pt_in_zed - pt_measured
    dist = float(np.linalg.norm(residual))
    print(f"  Measured ZED depth at ({u},{v}): {pt_measured}")
    print(f"  3D residual (expected - measured): {residual}  |r|={dist:.4f} m")
    if dist < 0.03:
        print("  ✓ ZED calibration consistent with hesai calibration (< 3 cm)")
    elif dist < 0.07:
        print("  ⚠  ZED residual 3–7 cm — acceptable for field use, run full re-calib if needed")
    else:
        print("  ✗  ZED residual > 7 cm — run calibrate_static_bag.py --mode camera_lidar")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("--hesai-xyz", type=str, default=None,
                    help='Updated hesai_calib_xyz, e.g. "0.58 -0.266 0.355"')
    ap.add_argument("--body-point", nargs=3, type=float, metavar=("X","Y","Z"),
                    default=None,
                    help="Known robot body feature in base_link [m]")
    ap.add_argument("--pixel-u", type=int, default=None, help="Feature pixel column in ZED left")
    ap.add_argument("--pixel-v", type=int, default=None, help="Feature pixel row in ZED left")
    ap.add_argument("--img-width", type=int, default=1280)
    ap.add_argument("--img-height", type=int, default=720)
    args = ap.parse_args()

    hesai_xyz = V1_HESAI_XYZ.copy()
    if args.hesai_xyz:
        hesai_xyz = np.array([float(v) for v in args.hesai_xyz.split()])
    print(f"Using hesai_calib_xyz = {hesai_xyz}")

    R_zed, t_zed = zed_in_base(hesai_xyz)
    print(f"ZED position in base_link: {t_zed}")

    print(f"\nLoading ZED cloud from {ZED_TOPIC_CLOUD} …")
    zed_cloud = load_one_cloud(args.bag, ZED_TOPIC_CLOUD)
    if zed_cloud is None:
        print(f"Topic {ZED_TOPIC_CLOUD!r} not in bag — check available topics.")
        sys.exit(1)
    print(f"ZED cloud: {len(zed_cloud)} points")

    ground_plane_check(zed_cloud, R_zed, t_zed)

    if args.body_point is not None and args.pixel_u is not None and args.pixel_v is not None:
        body_point_check(
            np.array(args.body_point),
            (args.pixel_u, args.pixel_v),
            zed_cloud, R_zed, t_zed,
            args.img_width, args.img_height,
        )
    else:
        print("\nTip: pass --body-point X Y Z --pixel-u U --pixel-v V to cross-validate "
              "a known robot feature with its pixel position in the ZED image.")


if __name__ == "__main__":
    main()
