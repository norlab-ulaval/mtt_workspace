#!/usr/bin/env python3
"""
diagnose_mapping.py — Diagnostic tool for ICP mapper trajectory jumps.

Subscribes to:
  /mapping/icp_odom      — pose per scan (nav_msgs/Odometry)
  /mapping/map           — map publication events (sensor_msgs/PointCloud2)
  /tf                    — map→odom transform

Detects:
  - Trajectory jumps (sudden large displacement between consecutive poses)
  - Backwards motion (ICP correction moves robot behind previous pose)
  - Correlation between map publications and pose anomalies
  - ICP processing latency (scan header stamp vs odom publish time)
  - odomToMap correction magnitude per scan

Usage (inside Docker or with ROS 2 sourced):
  python3 diagnose_mapping.py [--jump-thresh 0.5] [--out /tmp/mapping_diag.csv]
"""

import sys
import math
import time
import csv
import argparse
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import numpy as np


# ─── helpers ──────────────────────────────────────────────────────────────────

def quat_to_yaw(q) -> float:
    """Extract yaw (Z rotation) from a quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def pose_from_odom(msg: Odometry):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return np.array([p.x, p.y, p.z]), quat_to_yaw(q)


def stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def now_sec() -> float:
    return time.monotonic()


# ─── dataclasses ──────────────────────────────────────────────────────────────

@dataclass
class ScanRecord:
    wall_time: float        # monotonic time when odom was received
    scan_stamp: float       # header stamp of the scan (sim time)
    pos: np.ndarray         # [x, y, z]
    yaw: float              # radians
    latency_ms: float       # wall_time - scan_stamp (detects bag speedup issues)
    step_m: float = 0.0     # distance from previous accepted pose
    step_yaw: float = 0.0   # yaw change from previous
    backwards: bool = False # True if motion direction reversed
    jump: bool = False      # True if step_m > jump_thresh


@dataclass
class MapPubRecord:
    wall_time: float
    stamp: float            # header stamp on the PointCloud2
    n_points: int
    size_mb: float


@dataclass
class TfRecord:
    wall_time: float
    stamp: float            # transform stamp
    tx: float
    ty: float
    tz: float
    yaw: float              # correction yaw (map←odom)


# ─── node ─────────────────────────────────────────────────────────────────────

class MappingDiagNode(Node):
    def __init__(self, jump_thresh: float, out_path: Optional[str]):
        super().__init__('mapping_diag')
        self.jump_thresh = jump_thresh
        self.out_path = out_path

        self._lock = threading.Lock()
        self._scans: List[ScanRecord] = []
        self._maps: List[MapPubRecord] = []
        self._tfs: List[TfRecord] = []

        # Rolling window: last 5 poses to compute travel direction
        self._recent_pos: deque = deque(maxlen=5)
        self._prev_pos: Optional[np.ndarray] = None
        self._prev_yaw: Optional[float] = None
        self._prev_scan_stamp: Optional[float] = None

        # Last map publication wall time (to correlate with jumps)
        self._last_map_pub_wall: Optional[float] = None
        self._last_map_pub_stamp: Optional[float] = None

        # Stats
        self._n_jumps = 0
        self._n_backwards = 0
        self._n_scans = 0

        # Subscriptions
        self.create_subscription(Odometry, '/mapping/icp_odom',
                                 self._odom_cb, 200)
        self.create_subscription(PointCloud2, '/mapping/map',
                                 self._map_cb, 10)
        self.create_subscription(TFMessage, '/tf',
                                 self._tf_cb, 500)

        # Print header
        self._print_header()

        # Summary timer every 10s
        self.create_timer(10.0, self._print_summary)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        wall = now_sec()
        scan_stamp = stamp_sec(msg.header.stamp)
        pos, yaw = pose_from_odom(msg)

        # Latency: how long since scan header stamp (in wall time, bag = sim time)
        # We use ROS clock for scan_stamp. Under sim time, wall latency is not meaningful,
        # so we report scan-to-scan interval instead.
        latency_ms = (wall - self._last_odom_wall) * 1000.0 if hasattr(self, '_last_odom_wall') else 0.0
        self._last_odom_wall = wall

        # Scan interval
        dt_ms = 0.0
        if self._prev_scan_stamp is not None:
            dt_ms = (scan_stamp - self._prev_scan_stamp) * 1000.0

        step_m = 0.0
        step_yaw = 0.0
        backwards = False
        jump = False

        if self._prev_pos is not None:
            delta = pos - self._prev_pos
            step_m = float(np.linalg.norm(delta[:2]))
            step_yaw = abs(_wrap_pi(yaw - self._prev_yaw))

            # Detect jump
            jump = step_m > self.jump_thresh

            # Detect backwards: project delta onto estimated travel direction.
            # Travel direction = mean of last few deltas.
            if len(self._recent_pos) >= 2:
                travel_vecs = [
                    self._recent_pos[i+1] - self._recent_pos[i]
                    for i in range(len(self._recent_pos)-1)
                ]
                travel_dir = np.mean(travel_vecs, axis=0)[:2]
                norm_dir = np.linalg.norm(travel_dir)
                if norm_dir > 0.01 and step_m > 0.02:
                    dot = np.dot(delta[:2], travel_dir) / (norm_dir * step_m)
                    backwards = dot < -0.5  # >120° from travel direction

        # Time since last map publication
        map_age_ms = None
        if self._last_map_pub_wall is not None:
            map_age_ms = (wall - self._last_map_pub_wall) * 1000.0

        rec = ScanRecord(
            wall_time=wall,
            scan_stamp=scan_stamp,
            pos=pos,
            yaw=yaw,
            latency_ms=latency_ms,
            step_m=step_m,
            step_yaw=math.degrees(step_yaw),
            backwards=backwards,
            jump=jump,
        )

        with self._lock:
            self._scans.append(rec)
            self._n_scans += 1
            if jump:
                self._n_jumps += 1
            if backwards:
                self._n_backwards += 1

        # Update rolling window
        self._recent_pos.append(pos.copy())
        self._prev_pos = pos
        self._prev_yaw = yaw
        self._prev_scan_stamp = scan_stamp

        # Print if anomaly or every 20 scans
        anomaly = jump or backwards
        if anomaly or (self._n_scans % 20 == 0):
            flag = ""
            if jump:
                flag += " *** JUMP ***"
            if backwards:
                flag += " *** BACKWARDS ***"
            map_info = f" [map_pub {map_age_ms:.0f}ms ago]" if map_age_ms is not None else ""
            print(
                f"[ODOM #{self._n_scans:5d}] t={scan_stamp:.3f}  "
                f"pos=({pos[0]:7.3f},{pos[1]:7.3f})  yaw={math.degrees(yaw):6.1f}°  "
                f"step={step_m*100:.1f}cm  Δyaw={step_yaw:.1f}°  "
                f"dt={dt_ms:.1f}ms{map_info}{flag}"
            )

        if anomaly:
            # Also dump context: last 5 scans + current
            print("  ↳ Context (last 5 scans):")
            with self._lock:
                recent = self._scans[-min(6, len(self._scans)):]
            for r in recent:
                print(f"    t={r.scan_stamp:.3f}  pos=({r.pos[0]:.3f},{r.pos[1]:.3f})"
                      f"  step={r.step_m*100:.1f}cm"
                      + (" <JUMP>" if r.jump else "")
                      + (" <BACK>" if r.backwards else ""))

    def _map_cb(self, msg: PointCloud2):
        wall = now_sec()
        stamp = stamp_sec(msg.header.stamp)
        n_pts = msg.width * msg.height
        size_mb = (msg.row_step * msg.height) / 1e6

        self._last_map_pub_wall = wall
        self._last_map_pub_stamp = stamp

        rec = MapPubRecord(wall_time=wall, stamp=stamp, n_points=n_pts, size_mb=size_mb)
        with self._lock:
            self._maps.append(rec)

        print(
            f"\n[MAP   #{len(self._maps):4d}] t={stamp:.3f}  "
            f"pts={n_pts:,}  size={size_mb:.1f}MB\n"
        )

    def _tf_cb(self, msg: TFMessage):
        wall = now_sec()
        for tf in msg.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id in ('odom', 'odom_hesai'):
                stamp = stamp_sec(tf.header.stamp)
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = quat_to_yaw(q)
                rec = TfRecord(
                    wall_time=wall, stamp=stamp,
                    tx=t.x, ty=t.y, tz=t.z, yaw=yaw
                )
                with self._lock:
                    self._tfs.append(rec)
                # Only keep last 500 TF records (high rate)
                with self._lock:
                    if len(self._tfs) > 500:
                        self._tfs = self._tfs[-500:]
                break

    # ── helpers ───────────────────────────────────────────────────────────────

    def _print_header(self):
        print("=" * 80)
        print("  ICP MAPPER DIAGNOSTIC")
        print(f"  jump threshold: {self.jump_thresh*100:.0f}cm")
        print("  Subscribing to: /mapping/icp_odom  /mapping/map  /tf")
        print("=" * 80)

    def _print_summary(self):
        with self._lock:
            n = self._n_scans
            nj = self._n_jumps
            nb = self._n_backwards
            nm = len(self._maps)

        print(f"\n── SUMMARY ── scans={n}  jumps={nj}  backwards={nb}  map_pubs={nm}")

        # Compute step statistics
        with self._lock:
            if len(self._scans) > 2:
                steps = [s.step_m for s in self._scans[1:] if s.step_m > 0]
                if steps:
                    print(f"   step: mean={np.mean(steps)*100:.1f}cm  "
                          f"max={np.max(steps)*100:.1f}cm  "
                          f"p99={np.percentile(steps, 99)*100:.1f}cm")

        # Map-jump correlation
        with self._lock:
            jumps = [s for s in self._scans if s.jump or s.backwards]
            maps = list(self._maps)

        if jumps and maps:
            print("   Jump-to-last-map-pub delays:")
            for j in jumps[-5:]:
                # Find nearest preceding map pub
                preceding = [m for m in maps if m.wall_time <= j.wall_time]
                if preceding:
                    lag = (j.wall_time - preceding[-1].wall_time) * 1000.0
                    print(f"     jump@t={j.scan_stamp:.3f}  map_pub_lag={lag:.0f}ms  "
                          f"map_pts={preceding[-1].n_points:,}")

        if self.out_path:
            self._write_csv()

    def _write_csv(self):
        with self._lock:
            scans = list(self._scans)
            maps = list(self._maps)

        try:
            with open(self.out_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['type', 'wall_time', 'stamp', 'x', 'y', 'yaw_deg',
                             'step_cm', 'step_yaw_deg', 'jump', 'backwards',
                             'n_pts', 'size_mb'])
                for s in scans:
                    w.writerow(['scan', f'{s.wall_time:.6f}', f'{s.scan_stamp:.6f}',
                                f'{s.pos[0]:.4f}', f'{s.pos[1]:.4f}', f'{s.yaw*180/math.pi:.2f}',
                                f'{s.step_m*100:.2f}', f'{s.step_yaw:.2f}',
                                int(s.jump), int(s.backwards), '', ''])
                for m in maps:
                    w.writerow(['map', f'{m.wall_time:.6f}', f'{m.stamp:.6f}',
                                '', '', '', '', '', '', '',
                                m.n_points, f'{m.size_mb:.2f}'])
            print(f"[DIAG] CSV written to {self.out_path}")
        except Exception as e:
            print(f"[DIAG] CSV write failed: {e}")

    # ── shutdown ──────────────────────────────────────────────────────────────

    def shutdown_report(self):
        """Call on Ctrl+C to print final report."""
        print("\n" + "=" * 80)
        print("  FINAL REPORT")
        print("=" * 80)
        self._print_summary()

        with self._lock:
            jumps = [s for s in self._scans if s.jump or s.backwards]
            maps = list(self._maps)

        if not jumps:
            print("\n  No jumps detected. Map publication doesn't seem to cause issues.")
        else:
            print(f"\n  {len(jumps)} anomalies detected.")
            # Show all jumps with context
            for i, j in enumerate(jumps):
                print(f"\n  Anomaly #{i+1}:")
                print(f"    scan_stamp = {j.scan_stamp:.6f}  pos=({j.pos[0]:.3f},{j.pos[1]:.3f})")
                print(f"    step = {j.step_m*100:.1f}cm  backwards={j.backwards}")
                # Find map pubs near this jump
                nearby_maps = [m for m in maps
                               if abs(m.wall_time - j.wall_time) < 2.0]
                if nearby_maps:
                    for m in nearby_maps:
                        lag = (j.wall_time - m.wall_time) * 1000.0
                        print(f"    ← map_pub {lag:+.0f}ms relative to jump  pts={m.n_points:,}")
                else:
                    print("    No map publication within ±2s of this jump")

        if self.out_path:
            self._write_csv()


def _wrap_pi(a: float) -> float:
    while a > math.pi:  a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--jump-thresh', type=float, default=0.5,
                        help='Step distance (m) that counts as a jump (default: 0.5)')
    parser.add_argument('--out', type=str, default='/tmp/mapping_diag.csv',
                        help='CSV output path (default: /tmp/mapping_diag.csv)')
    args = parser.parse_args()

    rclpy.init()
    node = MappingDiagNode(jump_thresh=args.jump_thresh, out_path=args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
