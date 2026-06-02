#!/usr/bin/env python3
"""Per-scan TF chain health logger for live MTT ICP mapping.

For each /mapping/icp_odom message, looks up the complete TF chain:
  map -> odom -> base_footprint -> base_link -> hesai_lidar
Logs the result of each lookup, the age of each transform, and whether
the composed TF pose matches the published ICP pose.

Primary diagnostic targets:
  - Startup race: does the first ICP scan arrive before odom->base_footprint
    is being published by mtt_odometry_node?
  - Stale TF: does the mapper use transforms older than 0.2s?
  - ICP vs TF divergence: does the published ICP odom differ from the TF chain?
  - Duplicate static TF: multiple URDF publishers cause base_link->hesai_lidar flicker.

Usage:
    python3 scripts/audit_tf_chain.py [--duration 60] [--output tf_audit.yaml]
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Pose2:
    x: float
    y: float
    yaw: float


def pose_from_odom(msg: Odometry) -> Pose2:
    p = msg.pose.pose.position
    return Pose2(float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation))


def pose_from_tf(tf: TransformStamped) -> Pose2:
    t = tf.transform.translation
    return Pose2(float(t.x), float(t.y), yaw_from_quat(tf.transform.rotation))


def compose_se2(a: Pose2, b: Pose2) -> Pose2:
    ca, sa = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2(
        a.x + ca * b.x - sa * b.y,
        a.y + sa * b.x + ca * b.y,
        wrap_pi(a.yaw + b.yaw),
    )


def dist2(a: Pose2, b: Pose2) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


# ---------------------------------------------------------------------------
# Per-scan record
# ---------------------------------------------------------------------------

@dataclass
class ScanRecord:
    scan_idx: int
    wall_time: float
    scan_stamp: float           # ICP odom header.stamp as float

    # TF lookup results (None = lookup failed)
    odom_base_ok: bool = False
    odom_base_age: float = float("nan")
    map_odom_ok: bool = False
    map_odom_age: float = float("nan")
    static_base_link_ok: bool = False
    static_hesai_ok: bool = False

    # ICP pose vs TF chain
    icp_x: float = float("nan")
    icp_y: float = float("nan")
    icp_yaw: float = float("nan")
    tf_x: float = float("nan")
    tf_y: float = float("nan")
    tf_yaw: float = float("nan")
    pose_delta_m: float = float("nan")
    pose_delta_deg: float = float("nan")

    # flags
    all_tf_ok: bool = False
    pose_diverged: bool = False   # ICP pose vs TF composed > 0.10m


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class TfChainAuditNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("mtt_audit_tf_chain")
        self.args = args
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_be = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Odometry, args.icp_topic, self._on_icp, qos_be)

        self.records: list[ScanRecord] = []
        self._first_all_tf_idx: Optional[int] = None
        self._static_checked = False
        self._startup_wall = time.monotonic()

        print(f"{BOLD}== MTT TF Chain Audit =={RESET}")
        print(f"icp_topic: {args.icp_topic}")
        print(
            f"frames: map={args.map_frame} odom={args.odom_frame} "
            f"base={args.base_frame} sensor={args.sensor_frame}"
        )
        print("Waiting for ICP odom...\n")

    def _lookup_tf_age(self, target: str, source: str, stamp_msg) -> tuple[bool, Optional[Pose2], float]:
        """Return (success, pose, age_s). age_s = nan on failure."""
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source,
                Time.from_msg(stamp_msg),
                timeout=Duration(seconds=self.args.tf_timeout),
            )
            age = (
                Time.from_msg(stamp_msg).nanoseconds -
                Time.from_msg(tf.header.stamp).nanoseconds
            ) / 1e9
            return True, pose_from_tf(tf), abs(age)
        except TransformException:
            return False, None, float("nan")

    def _lookup_static(self, target: str, source: str) -> bool:
        try:
            self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.1))
            return True
        except TransformException:
            return False

    def _check_static_tf(self) -> None:
        if self._static_checked:
            return
        bl_ok = self._lookup_static("base_footprint", "base_link")
        hs_ok = self._lookup_static("base_link", self.args.sensor_frame)
        if bl_ok and hs_ok:
            try:
                tf_hs = self.tf_buffer.lookup_transform(
                    "base_link", self.args.sensor_frame, Time(), timeout=Duration(seconds=0.1)
                )
                yaw_deg = math.degrees(yaw_from_quat(tf_hs.transform.rotation))
                print(
                    f"[STATIC TF] base_link->{self.args.sensor_frame}: "
                    f"x={tf_hs.transform.translation.x:.3f} "
                    f"y={tf_hs.transform.translation.y:.3f} "
                    f"z={tf_hs.transform.translation.z:.3f} "
                    f"yaw={yaw_deg:.1f}deg  (expected ~+90deg)"
                )
            except TransformException:
                pass
            self._static_checked = True

    def _on_icp(self, msg: Odometry) -> None:
        self._check_static_tf()

        now_wall = time.monotonic()
        scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        icp_pose = pose_from_odom(msg)
        idx = len(self.records)

        rec = ScanRecord(
            scan_idx=idx,
            wall_time=now_wall,
            scan_stamp=scan_stamp,
            icp_x=icp_pose.x,
            icp_y=icp_pose.y,
            icp_yaw=icp_pose.yaw,
        )

        # --- odom -> base_footprint ---
        ok_ob, pose_ob, age_ob = self._lookup_tf_age(
            self.args.odom_frame, self.args.base_frame, msg.header.stamp
        )
        rec.odom_base_ok = ok_ob
        rec.odom_base_age = age_ob

        # --- map -> odom ---
        ok_mo, pose_mo, age_mo = self._lookup_tf_age(
            self.args.map_frame, self.args.odom_frame, msg.header.stamp
        )
        rec.map_odom_ok = ok_mo
        rec.map_odom_age = age_mo

        # --- static: base_footprint -> base_link, base_link -> hesai ---
        rec.static_base_link_ok = self._lookup_static("base_footprint", "base_link")
        rec.static_hesai_ok = self._lookup_static("base_link", self.args.sensor_frame)

        rec.all_tf_ok = ok_ob and ok_mo and rec.static_base_link_ok and rec.static_hesai_ok

        if self._first_all_tf_idx is None and rec.all_tf_ok:
            self._first_all_tf_idx = idx
            startup_delay = now_wall - self._startup_wall
            print(
                f"[STARTUP] First scan with complete TF chain: scan #{idx} "
                f"({startup_delay:.1f}s after audit start)"
            )

        # --- Composed TF chain vs published ICP pose ---
        if ok_ob and ok_mo and pose_ob and pose_mo:
            composed = compose_se2(pose_mo, pose_ob)
            rec.tf_x = composed.x
            rec.tf_y = composed.y
            rec.tf_yaw = composed.yaw
            rec.pose_delta_m = dist2(icp_pose, composed)
            rec.pose_delta_deg = math.degrees(abs(wrap_pi(icp_pose.yaw - composed.yaw)))
            rec.pose_diverged = rec.pose_delta_m > self.args.diverge_thresh

        self.records.append(rec)

        # Print anomalies and periodic summaries
        if idx % 50 == 0 and idx > 0:
            self._print_progress(idx)
        if not rec.all_tf_ok or rec.pose_diverged:
            self._print_anomaly(rec)

    def _print_progress(self, idx: int) -> None:
        n = len(self.records)
        tf_fail = sum(1 for r in self.records if not r.all_tf_ok)
        diverged = sum(1 for r in self.records if r.pose_diverged)
        ages_ob = [r.odom_base_age for r in self.records if not math.isnan(r.odom_base_age)]
        ages_mo = [r.map_odom_age for r in self.records if not math.isnan(r.map_odom_age)]
        print(
            f"[{idx:5d}] scans={n}  tf_missing={tf_fail}  pose_diverged={diverged}"
            + (f"  odom->base age max={max(ages_ob)*1000:.0f}ms" if ages_ob else "")
            + (f"  map->odom age max={max(ages_mo)*1000:.0f}ms" if ages_mo else "")
        )

    def _print_anomaly(self, rec: ScanRecord) -> None:
        prefix = f"[ANOMALY #{rec.scan_idx}]"
        if not rec.all_tf_ok:
            missing = []
            if not rec.odom_base_ok:
                missing.append(f"odom->base_footprint")
            if not rec.map_odom_ok:
                missing.append(f"map->odom")
            if not rec.static_base_link_ok:
                missing.append("base_footprint->base_link(static)")
            if not rec.static_hesai_ok:
                missing.append(f"base_link->hesai_lidar(static)")
            print(f"{prefix} TF MISSING: {', '.join(missing)}"
                  f"  odom_base_age={rec.odom_base_age*1000:.0f}ms"
                  f"  map_odom_age={rec.map_odom_age*1000:.0f}ms")
        if rec.pose_diverged:
            print(
                f"{prefix} POSE DIVERGED {rec.pose_delta_m:.3f}m / {rec.pose_delta_deg:.1f}deg"
                f"  icp=({rec.icp_x:.3f},{rec.icp_y:.3f},{math.degrees(rec.icp_yaw):.1f}deg)"
                f"  tf=({rec.tf_x:.3f},{rec.tf_y:.3f},{math.degrees(rec.tf_yaw) if not math.isnan(rec.tf_yaw) else float('nan'):.1f}deg)"
            )

    def print_summary(self) -> dict:
        n = len(self.records)
        if n == 0:
            print(f"\n{RED}No ICP odom messages received.{RESET}")
            return {"n": 0, "status": "no_data"}

        tf_fail = sum(1 for r in self.records if not r.all_tf_ok)
        diverged = sum(1 for r in self.records if r.pose_diverged)

        ages_ob = [r.odom_base_age for r in self.records if not math.isnan(r.odom_base_age)]
        ages_mo = [r.map_odom_age for r in self.records if not math.isnan(r.map_odom_age)]

        print(f"\n{BOLD}== TF Chain Audit Summary =={RESET}")
        print(f"total_scans: {n}")
        print(f"scans_with_missing_tf: {tf_fail}  ({100*tf_fail/n:.1f}%)")
        print(f"scans_with_diverged_pose: {diverged}  ({100*diverged/n:.1f}%)")
        if self._first_all_tf_idx is not None:
            print(f"first_complete_tf_at_scan: {self._first_all_tf_idx}")
            if self._first_all_tf_idx > 0:
                print(f"  {YELLOW}WARNING{RESET}: {self._first_all_tf_idx} scans arrived before TF chain was ready (startup race)")
        else:
            print(f"first_complete_tf_at_scan: {RED}NEVER{RESET}")

        if ages_ob:
            print(f"odom_base_age_ms: max={max(ages_ob)*1000:.1f} mean={sum(ages_ob)/len(ages_ob)*1000:.1f}")
        if ages_mo:
            print(f"map_odom_age_ms: max={max(ages_mo)*1000:.1f} mean={sum(ages_mo)/len(ages_mo)*1000:.1f}")

        issues = []
        if tf_fail > 0:
            issues.append(f"tf_missing_in_{tf_fail}_scans")
        if diverged > 0:
            issues.append(f"pose_diverged_in_{diverged}_scans")
        if self._first_all_tf_idx is not None and self._first_all_tf_idx > 0:
            issues.append(f"startup_race_{self._first_all_tf_idx}_scans_before_tf_ready")

        status = "ok" if not issues else "warn"
        if tf_fail > n * 0.05 or self._first_all_tf_idx is None:
            status = "fail"

        print(f"status: {GREEN if status=='ok' else (YELLOW if status=='warn' else RED)}{status}{RESET}")
        if issues:
            for issue in issues:
                print(f"  - {issue}")

        return {
            "n": n,
            "tf_fail": tf_fail,
            "pose_diverged": diverged,
            "first_complete_tf_scan": self._first_all_tf_idx,
            "status": status,
            "issues": issues,
        }


# ---------------------------------------------------------------------------
# YAML writer
# ---------------------------------------------------------------------------

def _write_yaml(summary: dict, records: list[ScanRecord], path: str, duration: float) -> None:
    with open(path, "w") as fh:
        fh.write(f"duration_s: {duration:.1f}\n")
        fh.write("summary:\n")
        for k, v in summary.items():
            if isinstance(v, list):
                fh.write(f"  {k}: {v}\n")
            else:
                fh.write(f"  {k}: {v}\n")
        # Write per-scan anomalies only
        anomalies = [r for r in records if not r.all_tf_ok or r.pose_diverged]
        if anomalies:
            fh.write("anomalies:\n")
            for r in anomalies[:200]:
                fh.write(
                    f"  - scan: {r.scan_idx}\n"
                    f"    stamp: {r.scan_stamp:.3f}\n"
                    f"    all_tf_ok: {r.all_tf_ok}\n"
                    f"    odom_base_ok: {r.odom_base_ok}\n"
                    f"    odom_base_age_ms: {r.odom_base_age*1000:.1f}\n"
                    f"    map_odom_ok: {r.map_odom_ok}\n"
                    f"    map_odom_age_ms: {r.map_odom_age*1000:.1f}\n"
                    f"    pose_diverged: {r.pose_diverged}\n"
                    f"    pose_delta_m: {r.pose_delta_m:.4f}\n"
                )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--icp-topic", default="/mapping/icp_odom")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--sensor-frame", default="hesai_lidar")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="Monitoring duration in seconds (default 60)")
    parser.add_argument("--tf-timeout", type=float, default=0.05,
                        help="TF lookup timeout in seconds (default 0.05)")
    parser.add_argument("--diverge-thresh", type=float, default=0.10,
                        help="Pose divergence threshold in metres (default 0.10)")
    parser.add_argument("--output", default="",
                        help="Write YAML report to this file (default: stdout only)")
    args = parser.parse_args()

    rclpy.init()
    node = TfChainAuditNode(args)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        summary = node.print_summary()
        records = list(node.records)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if args.output:
        _write_yaml(summary, records, args.output, args.duration)
        print(f"\nYAML written to {args.output}")

    return 0 if summary.get("status") == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
