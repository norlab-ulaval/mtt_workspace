#!/usr/bin/env python3
"""
export_pinslam_colored_clouds.py
Extract Hesai LiDAR clouds from a .mcap bag, color them with the ZED RGB
camera, strip robot self-points, and save them as sequential .pcd files for
PIN-SLAM to read directly.

Calibration is not hardcoded: the extrinsic (hesai_lidar -> ZED optical frame)
is composed from the bag's own /tf_static, and the intrinsics come from the
ZED /camera_info message. This mirrors the live lidar_camera_colorizer_node
(src/mtt_core/mtt_lidar_camera_colorizer/src/lidar_camera_colorizer_node.cpp):
same projection math, same "cache latest image, tolerate a timeout" strategy.

Every Hesai frame is kept (this is what drives PIN-SLAM's scan-to-scan
continuity). A point gets a real RGB color when a camera image was seen
within --image-max-dt seconds and the frame is outside the glitch window;
otherwise it falls back to a grayscale color derived from LiDAR intensity
so the geometry stays visually distinguishable instead of going flat gray.

Usage
-----
  # venv created by this task: scripts/.venv_pinslam
  scripts/.venv_pinslam/bin/python scripts/export_pinslam_colored_clouds.py \\
      data/BAG_ICE_RINK_.../bag/bag_0.mcap \\
      --end-time-offset 482
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from rosbags.highlevel import AnyReader

# ── Robot self-filter bounding boxes (hesai_lidar frame, metres) ──
TRACTOR_BOX = ((-0.6, 1.0), (-0.45, 0.45), (-0.35, 0.15))
TRAILER_BOX = ((-6.0, -1.5), (-2.5, 2.5), (-1.0, 1.0))

# sensor_msgs/PointField datatype codes -> numpy dtype
_PF_TO_NP = {
    1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8",
}


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def transform_matrix(translation, quat_xyzw) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rot(*quat_xyzw)
    T[:3, 3] = translation
    return T


def build_static_tf_graph(tf_msg) -> dict[tuple[str, str], np.ndarray]:
    """parent,child -> T_parent_child (maps a point in child frame into parent frame)."""
    edges: dict[tuple[str, str], np.ndarray] = {}
    for tr in tf_msg.transforms:
        t = tr.transform.translation
        q = tr.transform.rotation
        T = transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        edges[(tr.header.frame_id, tr.child_frame_id)] = T
    return edges


def lookup_transform(edges: dict[tuple[str, str], np.ndarray], target: str, source: str) -> np.ndarray:
    """Generic BFS over the (undirected) static TF tree: returns T_target_source
    such that p_target = T_target_source @ p_source."""
    adjacency: dict[str, list[tuple[str, np.ndarray]]] = {}
    for (parent, child), T in edges.items():
        adjacency.setdefault(parent, []).append((child, T))
        adjacency.setdefault(child, []).append((parent, np.linalg.inv(T)))

    if source not in adjacency:
        raise RuntimeError(f"frame '{source}' not found in /tf_static")

    # BFS from source, accumulating T_source_X for every visited frame X.
    visited = {source: np.eye(4)}
    queue = [source]
    while queue:
        cur = queue.pop(0)
        if cur == target:
            break
        for neighbor, T_cur_neighbor in adjacency.get(cur, []):
            if neighbor in visited:
                continue
            # p_cur = T_cur_neighbor @ p_neighbor  =>  T_source_neighbor = T_source_cur @ T_cur_neighbor
            visited[neighbor] = visited[cur] @ T_cur_neighbor
            queue.append(neighbor)

    if target not in visited:
        raise RuntimeError(f"no static TF path from '{source}' to '{target}'")

    T_source_target = visited[target]          # p_source = T_source_target @ p_target
    return np.linalg.inv(T_source_target)       # p_target = T_target_source @ p_source


def decode_pointcloud(msg) -> tuple[np.ndarray, np.ndarray]:
    """Returns (xyz [N,3] float64, intensity [N] float32), NaN/Inf points dropped."""
    endian = ">" if msg.is_bigendian else "<"
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.name not in ("x", "y", "z", "intensity") or f.count != 1:
            continue
        np_code = _PF_TO_NP.get(f.datatype)
        if np_code is None:
            continue
        names.append(f.name)
        formats.append(endian + np_code)
        offsets.append(f.offset)

    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})
    raw = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)

    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)
    intensity = raw["intensity"].astype(np.float32) if "intensity" in names else np.zeros(len(raw), np.float32)

    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite], intensity[finite]


def robot_filter_mask(xyz: np.ndarray) -> np.ndarray:
    def in_box(box):
        (x0, x1), (y0, y1), (z0, z1) = box
        return (
            (xyz[:, 0] > x0) & (xyz[:, 0] < x1)
            & (xyz[:, 1] > y0) & (xyz[:, 1] < y1)
            & (xyz[:, 2] > z0) & (xyz[:, 2] < z1)
        )

    in_robot = in_box(TRACTOR_BOX) | in_box(TRAILER_BOX)
    return ~in_robot


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


def color_points(
    xyz: np.ndarray,
    intensity: np.ndarray,
    T_cam_lidar: np.ndarray,
    intr: Intrinsics,
    image_bgr: np.ndarray | None,
) -> np.ndarray:
    """Returns colors [N,3] float64 in [0,1] (RGB)."""
    gray = np.clip(intensity, 0.0, 255.0) / 255.0
    colors = np.stack([gray, gray, gray], axis=1).astype(np.float64)

    if image_bgr is None:
        return colors

    ones = np.ones((xyz.shape[0], 1), dtype=np.float64)
    p_lidar_h = np.concatenate([xyz, ones], axis=1)
    p_cam = (T_cam_lidar @ p_lidar_h.T).T[:, :3]

    in_front = p_cam[:, 2] > 0.0
    inv_z = np.zeros(xyz.shape[0])
    inv_z[in_front] = 1.0 / p_cam[in_front, 2]

    u = np.round(intr.fx * p_cam[:, 0] * inv_z + intr.cx).astype(np.int64)
    v = np.round(intr.fy * p_cam[:, 1] * inv_z + intr.cy).astype(np.int64)

    in_fov = in_front & (u >= 0) & (u < intr.width) & (v >= 0) & (v < intr.height)
    idx = np.where(in_fov)[0]
    if idx.size:
        bgr = image_bgr[v[idx], u[idx]].astype(np.float64) / 255.0
        colors[idx, 0] = bgr[:, 2]
        colors[idx, 1] = bgr[:, 1]
        colors[idx, 2] = bgr[:, 0]

    return colors


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path, help="Path to bag_N.mcap (or its containing 'bag' directory)")
    ap.add_argument("--output-dir", type=Path, default=None, help="Default: <bag_dir>/colored_pcds")
    ap.add_argument("--cloud-topic", default="/hesai_lidar/points")
    ap.add_argument("--image-topic", default="/zed/zed_node/rgb/color/rect/image/compressed")
    ap.add_argument("--camera-info-topic", default="/zed/zed_node/rgb/color/rect/camera_info")
    ap.add_argument("--tf-static-topic", default="/tf_static")
    ap.add_argument("--lidar-frame", default="hesai_lidar")
    ap.add_argument("--image-max-dt", type=float, default=0.5, help="Max |t_cloud - t_image| (s) to accept a color match")
    ap.add_argument("--glitch-start-offset", type=float, default=482.0, help="Seconds from bag start")
    ap.add_argument("--glitch-end-offset", type=float, default=527.0, help="Seconds from bag start")
    ap.add_argument("--start-time-offset", type=float, default=0.0, help="Skip cloud frames before this many seconds from bag start")
    ap.add_argument("--end-time-offset", type=float, default=None, help="Stop after this many seconds from bag start (e.g. 482 to stop right before the ZED glitch)")
    ap.add_argument("--max-frames", type=int, default=None, help="Stop after this many written frames (smoke test)")
    ap.add_argument("--ascii", action="store_true", help="Write ASCII PCD instead of binary (default binary)")
    ap.add_argument("--progress-every", type=int, default=200)
    args = ap.parse_args()

    bag_path = args.bag
    bag_dir = bag_path if bag_path.is_dir() else bag_path.parent
    out_dir = args.output_dir or (bag_dir / "colored_pcds")
    out_dir.mkdir(parents=True, exist_ok=True)

    want_topics = {args.cloud_topic, args.image_topic, args.camera_info_topic, args.tf_static_topic}

    tf_edges: dict[tuple[str, str], np.ndarray] = {}
    camera_optical_frame: str | None = None
    T_cam_lidar: np.ndarray | None = None
    intr: Intrinsics | None = None
    latest_image_bgr: np.ndarray | None = None
    latest_image_stamp: float | None = None

    t0: float | None = None
    written = 0
    n_colored = 0
    n_gray_no_image = 0
    n_gray_glitch = 0
    n_robot_pts_removed = 0
    n_nan_pts_removed = 0

    with AnyReader([bag_dir]) as reader:
        conns = [c for c in reader.connections if c.topic in want_topics]
        found_topics = {c.topic for c in conns}
        missing = want_topics - found_topics
        if missing:
            print(f"ERROR: topics not found in bag: {sorted(missing)}", file=sys.stderr)
            return 1

        for conn, _log_time, rawdata in reader.messages(connections=conns):
            if conn.topic == args.tf_static_topic:
                msg = reader.deserialize(rawdata, conn.msgtype)
                tf_edges.update(build_static_tf_graph(msg))
                continue

            if conn.topic == args.camera_info_topic:
                msg = reader.deserialize(rawdata, conn.msgtype)
                intr = Intrinsics(
                    fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5],
                    width=msg.width, height=msg.height,
                )
                camera_optical_frame = msg.header.frame_id
                continue

            if conn.topic == args.image_topic:
                msg = reader.deserialize(rawdata, conn.msgtype)
                buf = np.frombuffer(msg.data, dtype=np.uint8)
                decoded = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if decoded is not None:
                    latest_image_bgr = decoded
                    latest_image_stamp = stamp_to_sec(msg.header.stamp)
                continue

            if conn.topic == args.cloud_topic:
                msg = reader.deserialize(rawdata, conn.msgtype)
                cloud_stamp = stamp_to_sec(msg.header.stamp)
                if t0 is None:
                    t0 = cloud_stamp
                    print(f"[t0] first Hesai cloud stamp = {t0:.3f} (epoch s)")

                elapsed = cloud_stamp - t0
                if elapsed < args.start_time_offset:
                    continue
                if args.end_time_offset is not None and elapsed > args.end_time_offset:
                    break

                # lazily resolve the extrinsic once we have both tf_static and camera_info
                if T_cam_lidar is None and tf_edges and intr is not None:
                    T_cam_lidar = lookup_transform(tf_edges, camera_optical_frame, args.lidar_frame)
                    print(f"[tf] {args.lidar_frame} -> {camera_optical_frame} resolved")

                xyz, intensity = decode_pointcloud(msg)
                n_nan_pts_removed += msg.width * msg.height - xyz.shape[0]

                keep = robot_filter_mask(xyz)
                n_robot_pts_removed += int((~keep).sum())
                xyz = xyz[keep]
                intensity = intensity[keep]

                if xyz.shape[0] == 0:
                    print(f"[warn] frame {written} empty after robot filter, skipping")
                    continue

                in_glitch = args.glitch_start_offset <= elapsed <= args.glitch_end_offset
                use_image = (
                    not in_glitch
                    and T_cam_lidar is not None
                    and intr is not None
                    and latest_image_bgr is not None
                    and latest_image_stamp is not None
                    and abs(cloud_stamp - latest_image_stamp) <= args.image_max_dt
                )

                if use_image:
                    colors = color_points(xyz, intensity, T_cam_lidar, intr, latest_image_bgr)
                    n_colored += 1
                elif in_glitch:
                    colors = color_points(xyz, intensity, T_cam_lidar, intr, None)
                    n_gray_glitch += 1
                else:
                    colors = color_points(xyz, intensity, T_cam_lidar, intr, None)
                    n_gray_no_image += 1

                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(xyz)
                pcd.colors = o3d.utility.Vector3dVector(colors)
                out_path = out_dir / f"{written:06d}.pcd"
                o3d.io.write_point_cloud(str(out_path), pcd, write_ascii=args.ascii)

                if written % args.progress_every == 0:
                    print(f"[{written:6d}] t+{elapsed:7.2f}s  pts={xyz.shape[0]:6d}  "
                          f"{'COLOR' if use_image else ('GLITCH' if in_glitch else 'NO-IMG')}  -> {out_path.name}")

                written += 1
                if args.max_frames is not None and written >= args.max_frames:
                    break

    print()
    print("=== summary ===")
    print(f"frames written        : {written}")
    print(f"  colored (camera)     : {n_colored}")
    print(f"  gray fallback (glitch): {n_gray_glitch}")
    print(f"  gray fallback (no img): {n_gray_no_image}")
    print(f"points removed (NaN/Inf)  : {n_nan_pts_removed}")
    print(f"points removed (robot box): {n_robot_pts_removed}")
    print(f"output dir: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
