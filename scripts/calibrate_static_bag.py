#!/usr/bin/env python3
"""
calibrate_static_bag.py
Offline static calibration refinement for MTT LiDAR-LiDAR and Camera-LiDAR pairs.

Robot must be STATIC during recording.  The script accumulates multiple sweeps
from each sensor to build a dense map, then refines the sensor-to-sensor transform
using multi-scale Generalized ICP (GICP) with SE(3) Lie-group perturbations.

Modes
-----
  inspect      — List all topics and TF frames in the bag.
  lidar_lidar  — Refine the transform between two LiDARs (Hesai ↔ RSAiry).
  camera_lidar — Refine the transform between a camera and a LiDAR (OAK ↔ RSAiry).

Usage (MTT defaults)
--------------------
  # Show bag content
  python3 calibrate_static_bag.py --bag /path/to/bag --mode inspect

  # Calibrate Hesai XT-32 ↔ RS-Airy
  python3 calibrate_static_bag.py --bag /path/to/bag --mode lidar_lidar \\
      --target-topic /hesai_lidar/points --source-topic /rsairy_ns/points \\
      --target-frame hesai_lidar --source-frame rsairy

  # Calibrate OAK-D ↔ RS-Airy
  python3 calibrate_static_bag.py --bag /path/to/bag --mode camera_lidar \\
      --camera-topic /oak/rgb/image_rect \\
      --camera-info-topic /oak/rgb/camera_info \\
      --lidar-topic /rsairy_ns/points

Dependencies
------------
  pip install open3d>=0.13 scipy opencv-python
  ROS 2: rosbag2_py, sensor_msgs_py, rclpy
"""

import argparse
import math
import struct
import sys
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import yaml

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as ScipyR


# ============================================================
#  SE(3) Lie group — exp, log, Jacobians
# ============================================================

def skew(w: np.ndarray) -> np.ndarray:
    """3-vector → 3×3 skew-symmetric matrix."""
    w = np.asarray(w, dtype=np.float64)
    return np.array([
        [0.0, -w[2],  w[1]],
        [w[2],  0.0, -w[0]],
        [-w[1],  w[0],  0.0],
    ])


def _V_matrix(phi: np.ndarray) -> np.ndarray:
    """
    Left Jacobian of SO(3): J_l(phi)  (3×3).

    Maps translational part of se(3) to the physical translation:
        t = V @ rho

    Closed-form:
        V = I + (1 - cos θ)/θ² K + (θ - sin θ)/θ³ K²
    """
    theta = np.linalg.norm(phi)
    K = skew(phi)
    if theta < 1e-10:
        return np.eye(3) + 0.5 * K
    return (
        np.eye(3)
        + (1.0 - math.cos(theta)) / theta ** 2 * K
        + (theta - math.sin(theta)) / theta ** 3 * (K @ K)
    )


def _V_inv_matrix(phi: np.ndarray) -> np.ndarray:
    """
    Inverse left Jacobian of SO(3): J_l^{-1}(phi) (3×3).

    Closed-form:
        V⁻¹ = I - ½K + (1/θ² - (1+cos θ)/(2θ sin θ)) K²
    """
    theta = np.linalg.norm(phi)
    K = skew(phi)
    if theta < 1e-10:
        return np.eye(3) - 0.5 * K
    c = (1.0 / theta ** 2) - (1.0 + math.cos(theta)) / (2.0 * theta * math.sin(theta))
    return np.eye(3) - 0.5 * K + c * (K @ K)


def se3_exp(xi: np.ndarray) -> np.ndarray:
    """
    Exponential map:  se(3) → SE(3).

    xi = [rho (3), phi (3)]  — Lie algebra element (left perturbation convention)
    Returns 4×4 homogeneous matrix T.
    """
    rho = np.asarray(xi[:3], dtype=np.float64)
    phi = np.asarray(xi[3:], dtype=np.float64)
    T = np.eye(4)
    T[:3, :3] = ScipyR.from_rotvec(phi).as_matrix()
    T[:3, 3] = _V_matrix(phi) @ rho
    return T


def se3_log(T: np.ndarray) -> np.ndarray:
    """
    Logarithm map:  SE(3) → se(3).

    Returns xi = [rho (3), phi (3)] s.t. Exp(xi) ≈ T.
    """
    phi = ScipyR.from_matrix(T[:3, :3]).as_rotvec()   # rotation vector
    rho = _V_inv_matrix(phi) @ T[:3, 3]
    return np.concatenate([rho, phi])


def inv_T(T: np.ndarray) -> np.ndarray:
    """Exact SE(3) inverse: [R | t] ↦ [Rᵀ | -Rᵀt]."""
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def compose(T1: np.ndarray, T2: np.ndarray) -> np.ndarray:
    """SE(3) group product: T1 ⊗ T2."""
    return T1 @ T2


def delta_se3(T_ref: np.ndarray, T: np.ndarray) -> tuple[float, float, np.ndarray]:
    """
    Compute left-invariant SE(3) delta:  D = Log(T ⊗ T_ref⁻¹).

    Returns (translation_norm_m, rotation_deg, xi_6d).
    """
    D = compose(T, inv_T(T_ref))
    xi = se3_log(D)
    dt_m = float(np.linalg.norm(xi[:3]))
    dr_deg = float(np.rad2deg(np.linalg.norm(xi[3:])))
    return dt_m, dr_deg, xi


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=np.float64)
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


# ============================================================
#  SE(3) ↔ human-readable formats
# ============================================================

def tf_msg_to_matrix(t) -> np.ndarray:
    q = [t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w]
    T = np.eye(4)
    T[:3, :3] = ScipyR.from_quat(q).as_matrix()
    T[:3, 3] = [t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z]
    return T


def matrix_to_components(T: np.ndarray) -> dict:
    """Return xyz, rpy (URDF intrinsic XYZ Euler) and quaternion xyzw."""
    q = ScipyR.from_matrix(T[:3, :3]).as_quat()           # x y z w
    rpy = ScipyR.from_matrix(T[:3, :3]).as_euler('xyz')   # intrinsic XYZ = URDF rpy
    return {
        "xyz": {"x": float(T[0, 3]), "y": float(T[1, 3]), "z": float(T[2, 3])},
        "rpy": {"r": float(rpy[0]), "p": float(rpy[1]), "y": float(rpy[2])},
        "quat_xyzw": {
            "x": float(q[0]), "y": float(q[1]),
            "z": float(q[2]), "w": float(q[3]),
        },
    }


def urdf_joint_snippet(parent: str, child: str, T: np.ndarray, joint_name: str = "") -> str:
    """Return a URDF <joint> block ready to paste into the xacro."""
    c = matrix_to_components(T)
    xyz = c["xyz"]
    rpy = c["rpy"]
    name = joint_name or f"{parent}_to_{child}_joint"
    return (
        f'<joint name="{name}" type="fixed">\n'
        f'  <parent link="{parent}"/>\n'
        f'  <child link="{child}"/>\n'
        f'  <origin xyz="{xyz["x"]:.6f} {xyz["y"]:.6f} {xyz["z"]:.6f}"'
        f' rpy="{rpy["r"]:.6f} {rpy["p"]:.6f} {rpy["y"]:.6f}"/>\n'
        f'</joint>'
    )


# ============================================================
#  ROS bag reader
# ============================================================

class BagReader:
    def __init__(self, bag_path: str, storage_id: str = "mcap"):
        self.bag_path = str(bag_path)
        self.storage_id = storage_id
        self._open()

    def _open(self, topics=None):
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(
            rosbag2_py.StorageOptions(uri=self.bag_path, storage_id=self.storage_id),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        self.topic_types = {t.name: t.type for t in self.reader.get_all_topics_and_types()}
        if topics:
            self.reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    def iter_messages(self, topics=None, max_msgs: int = None):
        self._open(topics)
        count = 0
        while self.reader.has_next():
            topic, data, ts = self.reader.read_next()
            msg_type = self.topic_types.get(topic)
            if msg_type is None:
                continue
            try:
                cls = get_message(msg_type)
                msg = deserialize_message(data, cls)
            except Exception:
                continue
            yield topic, msg, ts
            count += 1
            if max_msgs is not None and count >= max_msgs:
                break


# ============================================================
#  TF graph  (BFS lookup, handles tf + tf_static)
# ============================================================

class TFGraph:
    def __init__(self):
        # (parent, child) → T_parent_child
        self._T: dict[tuple[str, str], np.ndarray] = {}

    def add_transform(self, tf_stamped):
        parent = tf_stamped.header.frame_id.lstrip("/")
        child = tf_stamped.child_frame_id.lstrip("/")
        if parent and child:
            self._T[(parent, child)] = tf_msg_to_matrix(tf_stamped)

    def frames(self) -> list[str]:
        fs: set[str] = set()
        for p, c in self._T:
            fs.add(p)
            fs.add(c)
        return sorted(fs)

    def lookup(self, target: str, source: str) -> np.ndarray:
        """
        Return T_target_source: maps points from *source* frame into *target* frame.
        BFS on the undirected TF graph.
        """
        target = target.lstrip("/")
        source = source.lstrip("/")
        if target == source:
            return np.eye(4)

        adj: dict[str, list[tuple[str, np.ndarray]]] = defaultdict(list)
        for (p, c), T_pc in self._T.items():
            adj[c].append((p, T_pc))          # child→parent via T_pc
            adj[p].append((c, inv_T(T_pc)))   # parent→child via T_pc⁻¹

        q: deque[tuple[str, np.ndarray]] = deque([(source, np.eye(4))])
        visited = {source}
        while q:
            cur, T_cur_src = q.popleft()
            if cur == target:
                return T_cur_src
            for nb, T_nb_cur in adj[cur]:
                if nb not in visited:
                    visited.add(nb)
                    q.append((nb, T_nb_cur @ T_cur_src))

        raise RuntimeError(f"No TF chain: {source} → {target}. Known frames: {self.frames()}")

    def print_tree(self):
        print("\n[TF] Known static transforms:")
        for (p, c), T in sorted(self._T.items()):
            xyz = T[:3, 3]
            rpy = ScipyR.from_matrix(T[:3, :3]).as_euler('xyz')
            print(
                f"  {p} → {c}"
                f"   xyz=[{xyz[0]:+.4f} {xyz[1]:+.4f} {xyz[2]:+.4f}]"
                f"   rpy=[{rpy[0]:+.4f} {rpy[1]:+.4f} {rpy[2]:+.4f}]"
            )


def load_tf_graph(bag: BagReader) -> TFGraph:
    graph = TFGraph()
    for _, msg, _ in bag.iter_messages(topics=["/tf_static", "/tf"]):
        for t in msg.transforms:
            graph.add_transform(t)
    return graph


def inject_optical_frame_if_missing(graph: TFGraph, optical_frame: str) -> bool:
    """
    If *optical_frame* is absent from the TF graph (typical when the camera
    driver was not running during bag recording), inject a synthetic edge using
    the standard ROS body→optical rotation:

        x_optical =  z_body  (forward)
        y_optical = -y_body  (no... right = -left)
        z_optical = -x_body  ...

    Standard ROS convention (camera body = x-forward / y-left / z-up):
        p_optical = R_optical_body @ p_body
        R = [[ 0, -1,  0],
             [ 0,  0, -1],
             [ 1,  0,  0]]

    Stored as T_base_optical (parent=base, child=optical) with translation=0.

    Searches for a base frame by stripping common optical suffixes:
        _optical_frame  →  try both the immediate parent token and the
                           full name without suffix.

    Returns True if the frame was injected, False if optical_frame already
    reachable or no matching base was found.
    """
    known = set(graph.frames())
    if optical_frame in known:
        return False  # already reachable

    # Standard body→optical rotation: R_optical_body
    R_opt_body = np.array([
        [ 0., -1.,  0.],
        [ 0.,  0., -1.],
        [ 1.,  0.,  0.],
    ], dtype=np.float64)
    # T_base_optical: maps optical coords into body (base) frame
    T_base_opt = np.eye(4)
    T_base_opt[:3, :3] = R_opt_body.T   # R_body_optical = R_opt_body^T

    # Try progressively shorter prefixes (e.g. oak_rgb_camera_optical_frame → oak_rgb_camera_optical → ... → oak)
    tokens = optical_frame.split("_")
    candidates = ["_".join(tokens[:i]) for i in range(len(tokens) - 1, 0, -1)]

    for base in candidates:
        if base in known:
            # Inject the edge: base → optical_frame
            # TFGraph._T stores (parent, child) → T_parent_child
            graph._T[(base, optical_frame)] = T_base_opt
            print(
                f"[TF] Injected synthetic optical-frame edge: "
                f"'{base}' → '{optical_frame}'  "
                f"(standard ROS optical rotation, t=[0,0,0])"
            )
            return True

    return False


# ============================================================
#  Fast point cloud parsing  (numpy binary, not Python iterator)
# ============================================================

def cloud_msg_to_numpy(msg, max_range: float = 80.0, min_range: float = 0.2) -> np.ndarray:
    """
    Parse a PointCloud2 message directly from its binary buffer.

    Returns an (N, 3) float64 array of valid xyz points.
    Avoids the slow Python-iterator API and handles NaN/Inf properly.
    """
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float64)

    step = msg.point_step
    n_pts = msg.width * msg.height

    if n_pts == 0:
        return np.empty((0, 3), dtype=np.float64)

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n_pts, step)

    def extract_float32(offset: int) -> np.ndarray:
        chunk = np.ascontiguousarray(raw[:, offset:offset + 4]).tobytes()
        return np.frombuffer(chunk, dtype=np.float32).astype(np.float64)

    fx = fields["x"].offset
    fy = fields["y"].offset
    fz = fields["z"].offset

    xyz = np.column_stack([extract_float32(fx), extract_float32(fy), extract_float32(fz)])
    r = np.linalg.norm(xyz, axis=1)
    mask = np.isfinite(r) & (r >= min_range) & (r <= max_range)
    return xyz[mask]


def read_merged_cloud(
    bag: BagReader,
    topic: str,
    max_clouds: int = 80,
    stride: int = 3,
    min_range: float = 0.2,
    max_range: float = 80.0,
    max_points_total: int = 800_000,
) -> tuple[np.ndarray, str]:
    """
    Accumulate up to *max_clouds* sweeps (every *stride*-th message) into a
    single dense static map.  Returns (xyz_Nx3_float64, frame_id).
    """
    clouds = []
    frame_id = None
    kept = seen = 0

    for _, msg, _ in bag.iter_messages(topics=[topic]):
        seen += 1
        if seen % stride != 0:
            continue
        if frame_id is None:
            frame_id = msg.header.frame_id.lstrip("/")
        pts = cloud_msg_to_numpy(msg, min_range=min_range, max_range=max_range)
        if len(pts) > 0:
            clouds.append(pts)
            kept += 1
        if kept >= max_clouds:
            break

    if not clouds:
        raise RuntimeError(f"No usable point cloud on topic '{topic}'")

    pts = np.vstack(clouds)
    if len(pts) > max_points_total:
        idx = np.random.default_rng(0).choice(len(pts), max_points_total, replace=False)
        pts = pts[idx]

    print(f"[CLOUD] {topic}: frame='{frame_id}'  sweeps={kept}  points={len(pts):,}")
    return pts, frame_id


# ============================================================
#  Open3D helpers
# ============================================================

def _make_pcd(
    points: np.ndarray,
    voxel: float,
    aabb: np.ndarray | None = None,
) -> o3d.geometry.PointCloud:
    """
    Build a voxel-downsampled, outlier-filtered Open3D point cloud.

    *aabb* is an optional (2, 3) array [[xmin, ymin, zmin], [xmax, ymax, zmax]]
    used to crop the cloud before downsampling.
    """
    pts = np.asarray(points, dtype=np.float64)

    if aabb is not None:
        lo, hi = aabb
        mask = np.all((pts >= lo) & (pts <= hi), axis=1)
        pts = pts[mask]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd = pcd.voxel_down_sample(voxel)
    pcd.remove_non_finite_points()

    if len(pcd.points) > 1000:
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    return pcd


def _overlap_aabb(
    source_pts: np.ndarray,
    target_pts: np.ndarray,
    T_target_source: np.ndarray,
    margin: float = 0.5,
) -> np.ndarray | None:
    """
    Find the axis-aligned bounding-box of the overlap between the two clouds.

    source_pts are transformed into the target frame using T_target_source.
    Returns (2, 3) array or None if no overlap.
    """
    src_t = transform_points(T_target_source, source_pts)
    lo = np.maximum(src_t.min(axis=0), target_pts.min(axis=0)) - margin
    hi = np.minimum(src_t.max(axis=0), target_pts.max(axis=0)) + margin
    if np.any(lo >= hi):
        return None
    return np.array([lo, hi])


# ============================================================
#  ICP registration primitives — P2P, P2Pl, GICP
# ============================================================

def _criteria(iters: int) -> o3d.pipelines.registration.ICPConvergenceCriteria:
    return o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=iters, relative_fitness=1e-7, relative_rmse=1e-7
    )


def _estimate_normals(
    pcd: o3d.geometry.PointCloud,
    voxel: float,
    sensor_origin: np.ndarray | None = None,
) -> o3d.geometry.PointCloud:
    """
    Estimate surface normals and orient them toward the sensor origin.

    radius = 5× voxel (enough neighbours for reliable normal, min 0.20 m).
    Orientation toward sensor ensures normals point consistently outward on
    planar surfaces (walls, floor) — critical for point-to-plane convergence.
    """
    radius = max(voxel * 5.0, 0.20)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=50)
    )
    origin = sensor_origin if sensor_origin is not None else np.zeros(3)
    pcd.orient_normals_towards_camera_location(origin.tolist())
    return pcd


def _p2p(
    src: o3d.geometry.PointCloud,
    tgt: o3d.geometry.PointCloud,
    max_corr: float,
    T: np.ndarray,
    iters: int,
) -> o3d.pipelines.registration.RegistrationResult:
    """
    Point-to-point ICP.

    Minimises:  Σ ‖T·pᵢ − qᵢ‖²

    No normals required.  Robust choice for coarse alignment — it does not
    depend on normal quality and tolerates outliers well at large voxel sizes.
    Converges to a slightly less accurate minimum than P2Pl but is a stable
    starting point for the subsequent surface-fitting step.
    """
    return o3d.pipelines.registration.registration_icp(
        src, tgt, max_corr, T,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        _criteria(iters),
    )


def _p2pl(
    src: o3d.geometry.PointCloud,
    tgt: o3d.geometry.PointCloud,
    max_corr: float,
    T: np.ndarray,
    iters: int,
    tgt_origin: np.ndarray | None = None,
) -> o3d.pipelines.registration.RegistrationResult:
    """
    Point-to-plane ICP with Huber robust loss.

    Minimises:  Σ ρ_Huber( (T·pᵢ − qᵢ)·n̂_{qᵢ} )

    Target must have normals (estimated here if absent).
    The Huber kernel (k = max_corr/4) downweights — but does not discard —
    correspondences with large residuals.  This is key for the Hesai/RSAiry
    pair: most false cross-FoV matches have large point-to-plane residuals
    and are automatically downweighted, while the true overlapping surface
    correspondences (floor, near walls) have small residuals and dominate.

    Point-to-plane converges faster and to a more accurate minimum than P2P
    because it exploits the local surface tangent — one degree of freedom
    per correspondence instead of three.
    """
    if not tgt.has_normals():
        _estimate_normals(tgt, max_corr / 2.0, tgt_origin)

    # Huber kernel: k = max_corr/4 — quadratic up to that threshold, linear beyond.
    k_huber = max_corr / 4.0
    try:
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane(
            kernel=o3d.pipelines.registration.HuberLoss(k=k_huber)
        )
    except TypeError:
        # Open3D < 0.14: kernel parameter not supported yet.
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()

    return o3d.pipelines.registration.registration_icp(
        src, tgt, max_corr, T, estimation, _criteria(iters)
    )


def _gicp(
    src: o3d.geometry.PointCloud,
    tgt: o3d.geometry.PointCloud,
    max_corr: float,
    T: np.ndarray,
    iters: int,
    tgt_origin: np.ndarray | None = None,
) -> o3d.pipelines.registration.RegistrationResult:
    """
    Generalized ICP (GICP) — covariance-weighted finishing pass.

    Minimises:  Σ (T·pᵢ − qᵢ)ᵀ (Σᵢᵀ + T·Σᵢˢ·Tᵀ)⁻¹ (T·pᵢ − qᵢ)

    Fits a local Gaussian (covariance matrix from neighbourhood) to each point
    rather than using the raw position.  This is the statistically optimal ICP
    variant under Gaussian sensor noise and provides the best sub-centimetre
    precision for dense, clean point clouds.

    Falls back to P2Pl with Huber if Open3D < 0.13.
    """
    try:
        return o3d.pipelines.registration.registration_generalized_icp(
            src, tgt, max_corr, T,
            o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(epsilon=1e-3),
            _criteria(iters),
        )
    except AttributeError:
        print("  [INFO] GICP unavailable (open3d < 0.13) — using P2Pl with Huber.")
        return _p2pl(src, tgt, max_corr, T, iters, tgt_origin)


# ============================================================
#  Multi-scale P2P → P2Pl → GICP pyramid
# ============================================================

# (voxel_m, max_corr_m, max_iters, method)
#
# Rationale for the three-method cascade:
#
#   P2P  (coarse): no normals needed → numerically robust, insensitive to
#     normal estimation quality.  Brings the transform within ~2 cm of the
#     true minimum so the subsequent P2Pl starts from a safe basin.
#
#   P2Pl (medium/fine): minimises signed distance to the tangent plane →
#     converges in fewer iterations and to a tighter minimum than P2P.
#     The Huber loss discards false cross-FoV correspondences automatically.
#     This is the main accuracy-driving step.
#
#   GICP (ultra-fine): treats each point as a Gaussian patch → optimal under
#     sensor noise.  Best sub-centimetre resolution when P2Pl has already
#     converged near the true minimum.
#
# max_corr = voxel × 2  (tight): prevents long-range spurious matches between
# the horizontal Hesai FoV and the downward/rearward RSAiry FoV.
_ICP_PYRAMID = [
    # voxel   max_corr  iters  method
    (0.10,    0.20,     60,    "p2p"),    # P2P  coarse  — no normals, stable
    (0.06,    0.12,     80,    "p2p"),    # P2P  medium  — tighten correspondence
    (0.04,    0.08,     100,   "p2pl"),   # P2Pl medium  — surface alignment + Huber
    (0.02,    0.05,     150,   "p2pl"),   # P2Pl fine    — accurate surface fit
    (0.02,    0.04,     200,   "gicp"),   # GICP ultra   — covariance-weighted finish
]

# Drift guard: discard a scale if it moves more than this from T_init.
# With a reliable URDF initial TF, real corrections are < 10 cm / 4°.
_MAX_SCALE_DRIFT_M   = 0.12
_MAX_SCALE_DRIFT_DEG = 5.0

_QUALITY_GATES = {
    # fitness: fraction of source points with a correspondence within max_corr.
    # With two sensors of very different FoV (Hesai horizontal ±15° vs RSAiry
    # hemispherical), the overlap is inherently partial — expect 0.10-0.35 even
    # with perfect calibration.  The meaningful metric is RMSE, not fitness.
    "min_fitness":       0.10,
    # RMSE of matched pairs: < 3 cm is excellent, < 5 cm is acceptable.
    "max_inlier_rmse":   0.05,
    # Lie-algebra delta vs T_init: large values indicate a local minimum.
    "max_delta_t_m":     0.12,
    "max_delta_r_deg":   5.0,
}


def refine_lidar_lidar_icp(
    source_pts: np.ndarray,
    target_pts: np.ndarray,
    T_init: np.ndarray,
    save_dir: Path | None = None,
) -> tuple[np.ndarray, o3d.pipelines.registration.RegistrationResult]:
    """
    Multi-scale GICP refinement of T_target_source.

    source_pts : (N, 3) points in the source LiDAR frame
    target_pts : (M, 3) points in the target LiDAR frame
    T_init     : 4×4 SE(3) initial estimate (T_target_source)

    Returns (T_refined 4×4, last RegistrationResult).
    Quality gates raise RuntimeError if the result is implausible.
    """
    # ── Automatic overlap AABB ──
    aabb = _overlap_aabb(source_pts, target_pts, T_init)
    if aabb is None:
        print("[WARN] No geometric overlap detected — skipping AABB crop.")
        aabb_msg = "FULL (no overlap detected)"
    else:
        lo, hi = aabb
        aabb_msg = (
            f"x=[{lo[0]:.2f}, {hi[0]:.2f}]  "
            f"y=[{lo[1]:.2f}, {hi[1]:.2f}]  "
            f"z=[{lo[2]:.2f}, {hi[2]:.2f}]"
        )
    print(f"[ICP] Overlap AABB: {aabb_msg}")

    T = T_init.copy()
    last_reg = None

    # Target (Hesai) sensor origin in its own frame is always [0, 0, 0].
    # This is used to orient target normals outward for P2Pl steps.
    tgt_sensor_origin = np.zeros(3)

    _methods = {"p2p": _p2p, "p2pl": _p2pl, "gicp": _gicp}

    for step_i, (voxel, max_corr, iters, method) in enumerate(_ICP_PYRAMID):
        src = _make_pcd(source_pts, voxel, aabb)
        tgt = _make_pcd(target_pts, voxel, aabb)

        n_src, n_tgt = len(src.points), len(tgt.points)
        if n_src < 200 or n_tgt < 200:
            print(
                f"[ICP] {method.upper()} scale {step_i}: SKIP  "
                f"(src={n_src}, tgt={n_tgt} — too sparse for voxel={voxel:.3f})"
            )
            continue

        fn = _methods[method]
        # P2Pl and GICP receive the target sensor origin for normal orientation.
        if method in ("p2pl", "gicp"):
            reg = fn(src, tgt, max_corr, T, iters, tgt_sensor_origin)
        else:
            reg = fn(src, tgt, max_corr, T, iters)

        T_candidate = np.array(reg.transformation, dtype=np.float64)
        dt_m, dr_deg, _ = delta_se3(T_init, T_candidate)

        print(
            f"[ICP] {method.upper()} scale {step_i}: "
            f"voxel={voxel:.3f}  max_corr={max_corr:.3f}  "
            f"src={n_src}  tgt={n_tgt}  "
            f"fitness={reg.fitness:.4f}  rmse={reg.inlier_rmse*100:.2f}cm  "
            f"Δt={dt_m*100:.1f}cm  Δr={dr_deg:.2f}°"
        )

        # ── Per-scale drift guard ──
        # Discard any scale that moves the solution far from T_init —
        # signals a spurious local minimum from false cross-FoV correspondences.
        if dt_m > _MAX_SCALE_DRIFT_M or dr_deg > _MAX_SCALE_DRIFT_DEG:
            print(
                f"      ↳ DRIFT GUARD: exceeds "
                f"[{_MAX_SCALE_DRIFT_M*100:.0f}cm / {_MAX_SCALE_DRIFT_DEG}°] "
                f"— discarded, keeping previous T."
            )
        else:
            T = T_candidate
            last_reg = reg

    if last_reg is None:
        print(
            "[WARN] All ICP scales were discarded by the drift guard or had "
            "insufficient points — returning T_init unchanged."
        )
        return T_init.copy(), None

    # ── Quality gates ──
    dt_m, dr_deg, xi_delta = delta_se3(T_init, T)
    failures = []
    if last_reg is not None:
        if last_reg.fitness < _QUALITY_GATES["min_fitness"]:
            failures.append(
                f"fitness {last_reg.fitness:.4f} < {_QUALITY_GATES['min_fitness']}"
            )
        if last_reg.inlier_rmse > _QUALITY_GATES["max_inlier_rmse"]:
            failures.append(
                f"RMSE {last_reg.inlier_rmse*100:.2f}cm > "
                f"{_QUALITY_GATES['max_inlier_rmse']*100:.0f}cm"
            )
    if dt_m > _QUALITY_GATES["max_delta_t_m"]:
        failures.append(
            f"|Δt| {dt_m*100:.1f}cm > "
            f"{_QUALITY_GATES['max_delta_t_m']*100:.0f}cm vs initial"
        )
    if dr_deg > _QUALITY_GATES["max_delta_r_deg"]:
        failures.append(
            f"|Δr| {dr_deg:.2f}° > {_QUALITY_GATES['max_delta_r_deg']}° vs initial"
        )

    if failures:
        print("\n[WARN] ═══════════════════════════════════════════════════════════")
        print("[WARN] Quality gate FAILED — calibration may be unreliable:")
        for f in failures:
            print(f"[WARN]   • {f}")
        print("[WARN] Check PLY overlays and verify sensor overlap region.")
        print("[WARN] ═══════════════════════════════════════════════════════════\n")
    else:
        print("\n[ICP] ✓ All quality gates passed.\n")

    # ── PLY overlays for visual inspection ──
    if save_dir:
        save_dir = Path(save_dir)
        save_dir.mkdir(parents=True, exist_ok=True)
        fine_voxel = _ICP_PYRAMID[-1][0]

        src_initial = _make_pcd(transform_points(T_init, source_pts), fine_voxel, aabb)
        src_refined = _make_pcd(transform_points(T, source_pts),     fine_voxel, aabb)
        tgt_final   = _make_pcd(target_pts,                          fine_voxel, aabb)

        # Colour: source=green, target=red (before), source=blue (after)
        src_initial.paint_uniform_color([0.2, 0.8, 0.2])
        src_refined.paint_uniform_color([0.2, 0.4, 0.9])
        tgt_final.paint_uniform_color([0.9, 0.2, 0.2])

        o3d.io.write_point_cloud(str(save_dir / "source_initial_in_target.ply"), src_initial)
        o3d.io.write_point_cloud(str(save_dir / "source_refined_in_target.ply"), src_refined)
        o3d.io.write_point_cloud(str(save_dir / "target.ply"),                   tgt_final)
        print(f"[PLY] Saved overlays to {save_dir}")

    return T, last_reg


# ============================================================
#  Camera-LiDAR edge-based refinement  (unchanged from original)
# ============================================================

def read_camera_info(bag: BagReader, topic: str):
    for _, msg, _ in bag.iter_messages(topics=[topic], max_msgs=20):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        frame = msg.header.frame_id.lstrip("/")
        print(f"[CAM INFO] {topic}: frame='{frame}' size={msg.width}×{msg.height}")
        return K, D, frame, int(msg.width), int(msg.height)
    raise RuntimeError(f"No camera_info on '{topic}'")


def read_image(bag: BagReader, topic: str):
    for _, msg, _ in bag.iter_messages(topics=[topic], max_msgs=50):
        frame = msg.header.frame_id.lstrip("/")
        if hasattr(msg, "format"):          # CompressedImage
            data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            img = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if img is not None:
                print(f"[IMAGE] {topic}: compressed  frame='{frame}'  shape={img.shape}")
                return img, frame
        else:                               # raw Image
            enc = msg.encoding.lower()
            data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            h, w = msg.height, msg.width
            if "rgb8" in enc:
                img = cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
            elif "bgr8" in enc:
                img = data.reshape(h, w, 3)
            elif "mono8" in enc:
                img = cv2.cvtColor(data.reshape(h, w), cv2.COLOR_GRAY2BGR)
            else:
                raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")
            print(f"[IMAGE] {topic}: raw  frame='{frame}'  shape={img.shape}")
            return img, frame
    raise RuntimeError(f"No usable image on '{topic}'")


def _project_points(K, T_cam_lidar, pts, image_shape):
    pts_cam = transform_points(T_cam_lidar, pts)
    z = pts_cam[:, 2]
    valid = z > 0.3
    pts_cam, z = pts_cam[valid], z[valid]
    if len(pts_cam) == 0:
        return np.empty((0, 2)), np.empty(0), np.where(valid)[0][:0]
    uvw = (K @ pts_cam.T).T
    uv = uvw[:, :2] / uvw[:, 2:3]
    h, w = image_shape[:2]
    inside = (uv[:, 0] >= 2) & (uv[:, 0] < w - 3) & (uv[:, 1] >= 2) & (uv[:, 1] < h - 3)
    return uv[inside], z[inside], np.where(valid)[0][inside]


def _bilinear_sample(img, uv):
    x, y = uv[:, 0], uv[:, 1]
    x0 = np.clip(np.floor(x).astype(np.int32), 0, img.shape[1] - 2)
    y0 = np.clip(np.floor(y).astype(np.int32), 0, img.shape[0] - 2)
    x1, y1 = x0 + 1, y0 + 1
    wa = (x1 - x) * (y1 - y)
    wb = (x1 - x) * (y - y0)
    wc = (x - x0) * (y1 - y)
    wd = (x - x0) * (y - y0)
    return wa * img[y0, x0] + wb * img[y1, x0] + wc * img[y0, x1] + wd * img[y1, x1]


def refine_camera_lidar_edges(
    image, K, lidar_points, T_init,
    max_points=25_000, prior_t=0.04, prior_r_deg=1.0,
    max_t=0.20, max_r_deg=6.0, save_dir=None,
):
    pts = np.asarray(lidar_points, dtype=np.float64)
    uv, _, valid_idx = _project_points(K, T_init, pts, image.shape)
    if len(valid_idx) < 500:
        raise RuntimeError(
            f"Too few projected LiDAR points ({len(valid_idx)}) with initial TF. "
            "Check frame convention or camera FoV."
        )
    pts = pts[valid_idx]
    if len(pts) > max_points:
        rng = np.random.default_rng(0)
        pts = pts[rng.choice(len(pts), max_points, replace=False)]

    gray = cv2.equalizeHist(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 60, 160)
    dist = np.minimum(cv2.distanceTransform(255 - edges, cv2.DIST_L2, 5).astype(np.float32), 30.0)

    if save_dir:
        Path(save_dir).mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(Path(save_dir) / "edges.png"), edges)

    prior_r = np.deg2rad(prior_r_deg)

    # Fixed-size residual: len(pts) edge-distance terms + 6 regularization terms.
    # Points that project outside the image get max-penalty (30 px / 5.0 scale).
    # Using a fixed size is required by scipy least_squares — the residual length
    # MUST NOT change between calls (projected point count varies with transform).
    _n_pts = len(pts)
    _max_penalty = 30.0 / 5.0

    def residual(xi):
        T = se3_exp(xi) @ T_init
        uv_, _, valid_idx_ = _project_points(K, T, pts, image.shape)
        r = np.full(_n_pts, _max_penalty, dtype=np.float64)
        if len(valid_idx_) >= 100:
            r[valid_idx_] = _bilinear_sample(dist, uv_) / 5.0
        return np.concatenate([r, xi[:3] / prior_t, xi[3:] / prior_r])

    lb = np.array([-max_t, -max_t, -max_t,
                   -np.deg2rad(max_r_deg), -np.deg2rad(max_r_deg), -np.deg2rad(max_r_deg)])
    print(f"[CAM-LIDAR] optimizing {len(pts)} projected points")
    res = least_squares(residual, x0=np.zeros(6), bounds=(lb, -lb),
                        loss="huber", max_nfev=80, verbose=1)

    T_refined = se3_exp(res.x) @ T_init

    if save_dir:
        _overlay_projection(image, K, T_init,    pts, Path(save_dir) / "overlay_before.png")
        _overlay_projection(image, K, T_refined, pts, Path(save_dir) / "overlay_after.png")

    print(f"[CAM-LIDAR] xi={res.x}  cost={res.cost:.4f}")
    return T_refined, res


def _overlay_projection(image, K, T_cam_lidar, pts_lidar, out_path, max_pts=8000):
    img = image.copy()
    if len(pts_lidar) > max_pts:
        pts_lidar = pts_lidar[np.random.default_rng(1).choice(len(pts_lidar), max_pts, replace=False)]
    uv, depth, _ = _project_points(K, T_cam_lidar, pts_lidar, img.shape)
    if len(uv) > 0:
        dmin, dmax = np.percentile(depth, [5, 95])
        denom = max(dmax - dmin, 1e-6)
        for (u, v), d in zip(uv, depth):
            a = float(np.clip((d - dmin) / denom, 0, 1))
            cv2.circle(img, (int(u), int(v)), 1, (int(255 * (1 - a)), int(255 * a), 30), -1)
    cv2.imwrite(str(out_path), img)


# ============================================================
#  Output — YAML + URDF snippet + static_transform_publisher
# ============================================================

def save_results(
    out_path: Path,
    parent_frame: str,
    child_frame: str,
    T_init: np.ndarray,
    T_refined: np.ndarray,
    quality: dict,
    joint_name: str = "",
):
    dt_m, dr_deg, xi_delta = delta_se3(T_init, T_refined)
    c_init    = matrix_to_components(T_init)
    c_refined = matrix_to_components(T_refined)

    data = {
        "parent_frame": parent_frame,
        "child_frame":  child_frame,
        "convention":   "T_parent_child: maps points from child_frame into parent_frame",
        "initial": {**c_init,    "matrix": T_init.tolist()},
        "refined": {**c_refined, "matrix": T_refined.tolist()},
        "delta_lie_algebra": {
            "translation_norm_m": float(dt_m),
            "rotation_deg":       float(dr_deg),
            "xi_rho_phi":         xi_delta.tolist(),
            "note": "Computed in Lie algebra: D = Log(T_refined ⊗ T_init⁻¹)",
        },
        "quality": quality,
    }

    out_path.parent.mkdir(parents=True, exist_ok=True)

    with open(out_path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"[SAVE] YAML         → {out_path}")

    # ── static_transform_publisher command ──
    c = c_refined
    xyz, q = c["xyz"], c["quat_xyzw"]
    tf_cmd = (
        "ros2 run tf2_ros static_transform_publisher "
        f"{xyz['x']:.6f} {xyz['y']:.6f} {xyz['z']:.6f} "
        f"{q['x']:.6f} {q['y']:.6f} {q['z']:.6f} {q['w']:.6f} "
        f"{parent_frame} {child_frame}"
    )
    sh_path = out_path.with_suffix(".static_tf.sh")
    with open(sh_path, "w") as f:
        f.write("#!/bin/bash\n# Generated by calibrate_static_bag.py\n")
        f.write(tf_cmd + "\n")
    print(f"[SAVE] static_tf.sh → {sh_path}")

    # ── URDF joint snippet ──
    snippet = urdf_joint_snippet(parent_frame, child_frame, T_refined, joint_name)
    urdf_path = out_path.with_suffix(".urdf_joint.xml")
    with open(urdf_path, "w") as f:
        f.write("<!-- Paste this <joint> block into your URDF/xacro -->\n")
        f.write(snippet + "\n")
    print(f"[SAVE] URDF snippet → {urdf_path}")

    # ── Human-readable summary ──
    rpy = c_refined["rpy"]
    print("\n" + "═" * 62)
    print("  CALIBRATION RESULT")
    print("═" * 62)
    print(f"  parent : {parent_frame}")
    print(f"  child  : {child_frame}")
    print(f"  xyz    : [{xyz['x']:+.6f}  {xyz['y']:+.6f}  {xyz['z']:+.6f}]  m")
    print(f"  rpy    : [{rpy['r']:+.6f}  {rpy['p']:+.6f}  {rpy['y']:+.6f}]  rad")
    print(f"  Δ vs initial : |t|={dt_m*100:.2f} cm  |r|={dr_deg:.3f}°")
    print("─" * 62)
    print(f"  URDF snippet:")
    print(f"    {snippet}")
    print("═" * 62 + "\n")


# ============================================================
#  Modes
# ============================================================

def mode_inspect(args):
    bag = BagReader(args.bag, args.storage_id)
    print("\n[TOPICS]")
    for topic, typ in sorted(bag.topic_types.items()):
        print(f"  {topic:60s}  {typ}")
    graph = load_tf_graph(bag)
    graph.print_tree()
    print("\n[FRAMES]")
    for f in graph.frames():
        print(f"  {f}")


def mode_lidar_lidar(args):
    bag = BagReader(args.bag, args.storage_id)
    graph = load_tf_graph(bag)

    target_pts, target_frame = read_merged_cloud(
        bag, args.target_topic,
        max_clouds=args.max_clouds, stride=args.stride,
        min_range=args.min_range, max_range=args.max_range,
        max_points_total=args.max_points_total,
    )
    source_pts, source_frame = read_merged_cloud(
        bag, args.source_topic,
        max_clouds=args.max_clouds, stride=args.stride,
        min_range=args.min_range, max_range=args.max_range,
        max_points_total=args.max_points_total,
    )

    target_frame = args.target_frame or target_frame
    source_frame = args.source_frame or source_frame

    print(f"\n[LIDAR-LIDAR] target frame : {target_frame}")
    print(f"[LIDAR-LIDAR] source frame : {source_frame}")

    T_init = graph.lookup(target_frame, source_frame)
    c = matrix_to_components(T_init)
    print(
        f"[LIDAR-LIDAR] initial TF   : "
        f"xyz=[{c['xyz']['x']:+.4f} {c['xyz']['y']:+.4f} {c['xyz']['z']:+.4f}]  "
        f"rpy=[{c['rpy']['r']:+.4f} {c['rpy']['p']:+.4f} {c['rpy']['y']:+.4f}]"
    )

    out_path = Path(args.out)
    T_refined, reg = refine_lidar_lidar_icp(
        source_pts, target_pts, T_init,
        save_dir=out_path.parent,
    )

    quality = {
        "icp_fitness":     float(reg.fitness)      if reg is not None else None,
        "icp_inlier_rmse": float(reg.inlier_rmse)  if reg is not None else None,
        "target_topic":    args.target_topic,
        "source_topic":    args.source_topic,
        "target_n_points": int(len(target_pts)),
        "source_n_points": int(len(source_pts)),
    }

    save_results(
        out_path,
        parent_frame=target_frame,
        child_frame=source_frame,
        T_init=T_init,
        T_refined=T_refined,
        quality=quality,
        joint_name=args.joint_name,
    )


def mode_camera_lidar(args):
    bag = BagReader(args.bag, args.storage_id)
    graph = load_tf_graph(bag)

    image, image_frame = read_image(bag, args.camera_topic)
    K, D, camera_info_frame, width, height = read_camera_info(bag, args.camera_info_topic)
    lidar_pts, lidar_frame = read_merged_cloud(
        bag, args.lidar_topic,
        max_clouds=args.max_clouds, stride=args.stride,
        min_range=args.min_range, max_range=args.max_range,
        max_points_total=args.max_points_total,
    )

    camera_frame = args.camera_frame or camera_info_frame or image_frame
    lidar_frame  = args.lidar_frame  or lidar_frame

    print(f"[CAM-LIDAR] camera frame : {camera_frame}")
    print(f"[CAM-LIDAR] lidar frame  : {lidar_frame}")

    # If the camera's optical frame is not in the bag TF tree (driver was not
    # running during recording), inject the standard ROS body→optical rotation
    # so that lookup() can traverse the chain.
    inject_optical_frame_if_missing(graph, camera_frame)

    T_init = graph.lookup(camera_frame, lidar_frame)
    out_path = Path(args.out)

    T_refined, opt = refine_camera_lidar_edges(
        image, K, lidar_pts, T_init,
        max_points=args.camera_opt_points,
        prior_t=args.prior_t,
        prior_r_deg=args.prior_r_deg,
        max_t=args.max_t,
        max_r_deg=args.max_r_deg,
        save_dir=out_path.parent,
    )

    quality = {
        "cost":                float(opt.cost),
        "success":             bool(opt.success),
        "message":             str(opt.message),
        "camera_topic":        args.camera_topic,
        "camera_info_topic":   args.camera_info_topic,
        "lidar_topic":         args.lidar_topic,
        "xi_rho_phi":          [float(x) for x in opt.x],
    }

    save_results(
        out_path,
        parent_frame=camera_frame,
        child_frame=lidar_frame,
        T_init=T_init,
        T_refined=T_refined,
        quality=quality,
        joint_name=args.joint_name,
    )

    # ── Back-project to URDF joint (optional) ──
    # Calibration gives T_refined = T_camera_lidar (maps lidar_frame → camera_frame).
    # The URDF joint `oak_mount_joint` expresses T_urdf_parent_urdf_child,
    # e.g. T_rsairy_oak.  The internal camera transform (oak → camera_frame)
    # is known from the static TF graph and is independent of the joint being
    # calibrated — so we can recover the corrected joint transform as:
    #
    #   T_camera_lidar = T_camera_oak  ⊗  T_oak_lidar
    #   T_urdf_parent_urdf_child  =  T_camera_lidar⁻¹  ⊗  T_camera_oak
    #
    # where T_camera_oak = graph.lookup(camera_frame, urdf_child).
    if args.urdf_joint_parent and args.urdf_joint_child:
        try:
            T_camera_oak = graph.lookup(camera_frame, args.urdf_joint_child)
            T_urdf_joint_new = inv_T(T_refined) @ T_camera_oak
            joint_name = args.joint_name or f"{args.urdf_joint_parent}_to_{args.urdf_joint_child}_joint"
            snippet = urdf_joint_snippet(
                args.urdf_joint_parent, args.urdf_joint_child,
                T_urdf_joint_new, joint_name,
            )
            bp_path = out_path.with_name(out_path.stem + ".backproject_urdf.xml")
            with open(bp_path, "w") as f:
                f.write("<!-- Back-projected URDF joint from camera-LiDAR calibration -->\n")
                f.write("<!-- paste into robot.urdf.xacro, replacing existing joint block -->\n")
                f.write(snippet + "\n")
            c_new = matrix_to_components(T_urdf_joint_new)
            print("\n" + "═" * 62)
            print("  BACK-PROJECTED URDF JOINT")
            print("═" * 62)
            print(f"  joint  : {joint_name}")
            print(f"  parent : {args.urdf_joint_parent}")
            print(f"  child  : {args.urdf_joint_child}")
            xyz_n = c_new["xyz"]
            rpy_n = c_new["rpy"]
            print(f"  xyz    : [{xyz_n['x']:+.6f}  {xyz_n['y']:+.6f}  {xyz_n['z']:+.6f}]  m")
            print(f"  rpy    : [{rpy_n['r']:+.6f}  {rpy_n['p']:+.6f}  {rpy_n['y']:+.6f}]  rad")
            print(f"  snippet:")
            print(f"    {snippet}")
            print("═" * 62 + "\n")
            print(f"[SAVE] Back-projected joint → {bp_path}")
        except RuntimeError as e:
            print(f"[WARN] Back-projection failed: {e}")
            print(f"[WARN] Known frames: {graph.frames()}")


# ============================================================
#  CLI
# ============================================================

def main():
    p = argparse.ArgumentParser(
        description="Offline static calibration refinement — MTT LiDAR-LiDAR / Camera-LiDAR.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    p.add_argument("--bag",        required=True, help="Path to ROS 2 bag directory")
    p.add_argument("--storage-id", default="mcap", help="Storage plugin: mcap or sqlite3")
    p.add_argument("--mode",       required=True, choices=["inspect", "lidar_lidar", "camera_lidar"])
    p.add_argument("--out",        default="results/calibration.yaml", help="Output YAML path")
    p.add_argument("--joint-name", default="", help="URDF joint name to use in the output snippet")

    # Cloud accumulation
    g = p.add_argument_group("Point cloud accumulation")
    g.add_argument("--max-clouds",       type=int,   default=80)
    g.add_argument("--stride",           type=int,   default=3,
                   help="Keep every N-th message (reduce redundant sweeps)")
    g.add_argument("--min-range",        type=float, default=0.3, help="metres")
    g.add_argument("--max-range",        type=float, default=70.0, help="metres")
    g.add_argument("--max-points-total", type=int,   default=800_000)

    # LiDAR-LiDAR
    g = p.add_argument_group("lidar_lidar mode (MTT defaults: Hesai ↔ RSAiry)")
    g.add_argument("--target-topic", default="/hesai_lidar/points")
    g.add_argument("--source-topic", default="/rsairy_ns/points")
    g.add_argument("--target-frame", default=None,
                   help="Override frame_id from the PointCloud2 message")
    g.add_argument("--source-frame", default=None)

    # Camera-LiDAR
    g = p.add_argument_group("camera_lidar mode (MTT defaults: OAK ↔ RSAiry)")
    g.add_argument("--camera-topic",      default="/oak/rgb/image_rect")
    g.add_argument("--camera-info-topic", default="/oak/rgb/camera_info")
    g.add_argument("--lidar-topic",       default="/rsairy_ns/points")
    g.add_argument("--camera-frame",      default=None)
    g.add_argument("--lidar-frame",       default=None)
    g.add_argument("--camera-opt-points", type=int,   default=25_000)
    g.add_argument("--prior-t",           type=float, default=0.04,
                   help="Translation regularisation prior (m)")
    g.add_argument("--prior-r-deg",       type=float, default=1.0,
                   help="Rotation regularisation prior (deg)")
    g.add_argument("--max-t",             type=float, default=0.20,
                   help="Max allowed translation correction (m)")
    g.add_argument("--max-r-deg",         type=float, default=6.0,
                   help="Max allowed rotation correction (deg)")
    g.add_argument(
        "--urdf-joint-parent", default=None,
        help=(
            "Back-project the calibration result to recover a specific URDF joint. "
            "Set to the joint's parent frame (e.g. 'rsairy' for oak_mount_joint). "
            "Must be used together with --urdf-joint-child."
        ),
    )
    g.add_argument(
        "--urdf-joint-child", default=None,
        help="Child frame of the URDF joint to recover (e.g. 'oak' for oak_mount_joint).",
    )

    args = p.parse_args()

    if args.mode == "inspect":
        mode_inspect(args)
    elif args.mode == "lidar_lidar":
        mode_lidar_lidar(args)
    elif args.mode == "camera_lidar":
        mode_camera_lidar(args)


if __name__ == "__main__":
    main()
