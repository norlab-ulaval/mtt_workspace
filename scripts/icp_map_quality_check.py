#!/usr/bin/env python3
"""ICP map quality monitor: growth rate, double-wall detection, odom jump count.

Subscribes to /mapping/map and /mapping/icp_odom.  Reports every 10 seconds:
  - Total point count and growth rate
  - Voxel density histogram at 0.5m resolution (double walls show as high-density voxels)
  - ICP odometry jump count (sudden large displacements)
  - Optional: /mapping/aligned_scan overlap with local map region

Usage:
    python3 scripts/icp_map_quality_check.py [--duration 120]
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2


GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"

VOXEL_SIZE = 0.50           # metres -- coarse enough for double-wall detection
DOUBLE_WALL_THRESHOLD = 8   # points per voxel above this -> suspected double wall
JUMP_THRESHOLD_M = 0.50     # ICP pose jump threshold (same as diagnose_mapping.py)
REPORT_INTERVAL_S = 10.0


def _pc2_to_xyz(msg: PointCloud2) -> Optional[np.ndarray]:
    """Extract XYZ from a PointCloud2 message.  Returns (N, 3) float32 array."""
    # Find field offsets
    fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return None

    x_off = fields["x"][0]
    y_off = fields["y"][0]
    z_off = fields["z"][0]
    point_step = msg.point_step
    data = msg.data
    n_points = msg.width * msg.height

    if n_points == 0:
        return np.empty((0, 3), dtype=np.float32)

    # Fast path: if x, y, z are contiguous floats at known offsets
    try:
        arr = np.frombuffer(data, dtype=np.uint8).reshape(n_points, point_step)
        xs = arr[:, x_off:x_off + 4].view(np.float32).reshape(-1)
        ys = arr[:, y_off:y_off + 4].view(np.float32).reshape(-1)
        zs = arr[:, z_off:z_off + 4].view(np.float32).reshape(-1)
        xyz = np.stack([xs, ys, zs], axis=1)
        # Remove NaN / Inf
        valid = np.isfinite(xyz).all(axis=1)
        return xyz[valid]
    except Exception:
        return None


def _voxel_counts(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    """Return count per occupied voxel as 1-D array."""
    if len(xyz) == 0:
        return np.array([], dtype=np.int32)
    keys = np.floor(xyz / voxel_size).astype(np.int64)
    # Pack 3 int64 into a single int64 key (lossy but fast for diagnostics)
    packed = keys[:, 0] + keys[:, 1] * 100003 + keys[:, 2] * 200003001
    _, counts = np.unique(packed, return_counts=True)
    return counts


# ---------------------------------------------------------------------------

@dataclass
class IcpSample:
    t: float
    x: float
    y: float
    yaw: float


@dataclass
class MapSnapshot:
    t: float
    n_points: int
    double_wall_voxels: int
    total_voxels: int


class MapQualityNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("mtt_icp_map_quality")
        self.args = args
        qos_be = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=4,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(PointCloud2, "/mapping/map", self._on_map, qos_be)
        self.create_subscription(Odometry, "/mapping/icp_odom", self._on_icp, qos_be)

        self.map_snapshots: list[MapSnapshot] = []
        self.icp_samples: list[IcpSample] = []
        self.icp_jumps: int = 0
        self._last_report_t = time.monotonic()
        self._report_count = 0

        print(f"{BOLD}== ICP Map Quality Monitor =={RESET}")
        print(f"voxel_size: {VOXEL_SIZE}m  double_wall_threshold: {DOUBLE_WALL_THRESHOLD} pts/voxel")
        print("Waiting for /mapping/map and /mapping/icp_odom...\n")

    def _on_map(self, msg: PointCloud2) -> None:
        xyz = _pc2_to_xyz(msg)
        if xyz is None:
            return
        n = len(xyz)
        counts = _voxel_counts(xyz, VOXEL_SIZE)
        total_voxels = len(counts)
        dw_voxels = int(np.sum(counts >= DOUBLE_WALL_THRESHOLD)) if len(counts) > 0 else 0
        snap = MapSnapshot(
            t=time.monotonic(),
            n_points=n,
            double_wall_voxels=dw_voxels,
            total_voxels=total_voxels,
        )
        self.map_snapshots.append(snap)

    def _on_icp(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        sample = IcpSample(t=time.monotonic(), x=float(p.x), y=float(p.y), yaw=yaw)
        if self.icp_samples:
            last = self.icp_samples[-1]
            step = math.hypot(sample.x - last.x, sample.y - last.y)
            if step > JUMP_THRESHOLD_M:
                self.icp_jumps += 1
        self.icp_samples.append(sample)

        now = time.monotonic()
        if now - self._last_report_t >= REPORT_INTERVAL_S:
            self._print_report()
            self._last_report_t = now

    def _print_report(self) -> None:
        self._report_count += 1
        print(f"\n{BOLD}-- Report #{self._report_count} (t={time.monotonic():.0f}s) --{RESET}")

        # ICP stats
        n_icp = len(self.icp_samples)
        if n_icp > 1:
            elapsed = self.icp_samples[-1].t - self.icp_samples[0].t
            icp_hz = (n_icp - 1) / max(elapsed, 1e-6)
            total_dist = sum(
                math.hypot(b.x - a.x, b.y - a.y)
                for a, b in zip(self.icp_samples, self.icp_samples[1:])
            )
            jump_color = GREEN if self.icp_jumps == 0 else RED
            print(
                f"icp_odom: n={n_icp} hz={icp_hz:.2f} "
                f"total_dist={total_dist:.2f}m "
                f"jumps={jump_color}{self.icp_jumps}{RESET}"
            )
        else:
            print(f"icp_odom: {YELLOW}waiting...{RESET}")

        # Map stats
        if not self.map_snapshots:
            print(f"map: {YELLOW}no map received yet{RESET}")
            return

        latest = self.map_snapshots[-1]
        growth_rate = 0.0
        if len(self.map_snapshots) >= 2:
            dt = latest.t - self.map_snapshots[0].t
            dn = latest.n_points - self.map_snapshots[0].n_points
            growth_rate = dn / max(dt, 1.0)  # pts/s

        dw_ratio = latest.double_wall_voxels / max(latest.total_voxels, 1)
        dw_color = GREEN if dw_ratio < 0.02 else (YELLOW if dw_ratio < 0.10 else RED)

        print(
            f"map: n_points={latest.n_points}  "
            f"growth={growth_rate:.0f}pts/s  "
            f"voxels={latest.total_voxels}  "
            f"double_wall_voxels={dw_color}{latest.double_wall_voxels}{RESET}"
            f" ({dw_ratio*100:.1f}%)"
        )

        # Growth rate anomaly
        if len(self.map_snapshots) >= 3:
            recent = self.map_snapshots[-3:]
            dts = [b.t - a.t for a, b in zip(recent, recent[1:])]
            dns = [b.n_points - a.n_points for a, b in zip(recent, recent[1:])]
            local_rate = sum(dns) / max(sum(dts), 1.0)
            if local_rate > growth_rate * 3.0 and local_rate > 5000:
                print(
                    f"  {RED}WARNING{RESET}: map growing anomalously fast recently: "
                    f"{local_rate:.0f} pts/s (baseline {growth_rate:.0f} pts/s)"
                )
                print("    Possible map contamination. Check ICP residual and aligned scan overlap.")

        # Double wall warning
        if dw_ratio > 0.02:
            print(
                f"  {YELLOW if dw_ratio < 0.10 else RED}WARNING{RESET}: "
                f"{latest.double_wall_voxels} high-density voxels detected "
                f"({dw_ratio*100:.1f}% of map voxels). "
                "This may indicate double walls from ICP drift."
            )

    def print_final_summary(self) -> dict:
        print(f"\n{BOLD}== Map Quality Final Summary =={RESET}")
        issues = []

        if not self.map_snapshots:
            print(f"{RED}No map messages received.{RESET}")
            return {"status": "no_map_data"}

        latest = self.map_snapshots[-1]
        dw_ratio = latest.double_wall_voxels / max(latest.total_voxels, 1)

        print(f"map_publications: {len(self.map_snapshots)}")
        print(f"final_map_points: {latest.n_points}")
        print(f"icp_jumps: {self.icp_jumps}")
        print(f"double_wall_voxels: {latest.double_wall_voxels} ({dw_ratio*100:.1f}%)")

        if self.icp_jumps > 0:
            issues.append(f"icp_jumps_{self.icp_jumps}")
        if dw_ratio > 0.10:
            issues.append(f"high_double_wall_ratio_{dw_ratio*100:.1f}pct")
        elif dw_ratio > 0.02:
            issues.append(f"moderate_double_wall_ratio_{dw_ratio*100:.1f}pct")

        status = "ok" if not issues else "warn"
        if self.icp_jumps > 5 or dw_ratio > 0.10:
            status = "fail"

        print(f"status: {GREEN if status=='ok' else (YELLOW if status=='warn' else RED)}{status}{RESET}")
        for issue in issues:
            print(f"  - {issue}")

        return {
            "map_publications": len(self.map_snapshots),
            "final_map_points": latest.n_points,
            "icp_jumps": self.icp_jumps,
            "double_wall_voxels": latest.double_wall_voxels,
            "double_wall_ratio": round(dw_ratio, 4),
            "status": status,
            "issues": issues,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=120.0,
                        help="Monitoring duration in seconds (default 120)")
    parser.add_argument("--output", default="",
                        help="Write YAML summary to this file")
    args = parser.parse_args()

    rclpy.init()
    node = MapQualityNode(args)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        summary = node.print_final_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if args.output:
        with open(args.output, "w") as fh:
            for k, v in summary.items():
                fh.write(f"{k}: {v}\n")
        print(f"\nYAML written to {args.output}")

    return 0 if summary.get("status") == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
