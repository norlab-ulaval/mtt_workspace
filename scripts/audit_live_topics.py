#!/usr/bin/env python3
"""Live Hz / jitter / age / QoS / frame_id monitor for critical MTT topics.

Run for a fixed duration (default 30s), then print a YAML summary and a
colour-coded human report.  Designed to catch QoS mismatches, stale data,
and rate drops that differ between Zenoh (live) and CycloneDDS (bag replay).

Usage:
    python3 scripts/audit_live_topics.py [--duration 30] [--output audit.yaml]
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2

try:
    from geometry_msgs.msg import TwistWithCovarianceStamped
    _HAS_TWIST_COV = True
except ImportError:
    _HAS_TWIST_COV = False

# Optional ZED odom uses Odometry too; mtt_tachometer / mtt_odometry use
# Odometry; hardware articulation uses Float64Stamped.
try:
    from std_msgs.msg import Float64
    _HAS_FLOAT64 = True
except ImportError:
    _HAS_FLOAT64 = False


GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"

# ---------------------------------------------------------------------------
# Topic spec table
# ---------------------------------------------------------------------------

@dataclass
class TopicSpec:
    topic: str
    msg_type: str               # human label
    expected_hz: float          # expected publish rate
    qos_reliable: bool          # True = RELIABLE, False = BEST_EFFORT
    expected_frame_id: str = ""  # "" means don't check
    required: bool = True


TOPIC_SPECS: list[TopicSpec] = [
    TopicSpec("/hesai_lidar/points",         "PointCloud2", 10.0,  False, "hesai_lidar"),
    TopicSpec("/mapping/icp_odom",           "Odometry",    10.0,  False, "odom"),
    TopicSpec("/mapping/map",                "PointCloud2",  1.0,  False, ""),
    TopicSpec("/mtt_odometry",               "Odometry",    50.0,  False, "odom"),
    TopicSpec("/mtt_tachometer",             "Odometry",    50.0,  False, ""),
    TopicSpec("/hardware/articulation_angle","Float64",     50.0,  False, ""),
    TopicSpec("/zed/zed_node/odom",          "Odometry",    15.0,  False, "odom", required=False),
    TopicSpec("/wiln/obstacles",             "PointCloud2", 10.0,  False, "", required=False),
    TopicSpec("/mtt/articulation_state",     "Odometry",    10.0,  False, "", required=False),
]


# ---------------------------------------------------------------------------
# Per-topic accumulator
# ---------------------------------------------------------------------------

@dataclass
class TopicStats:
    spec: TopicSpec
    recv_wall: list[float] = field(default_factory=list)    # monotonic() at receive
    header_stamps: list[float] = field(default_factory=list)  # header.stamp as float
    frame_ids: list[str] = field(default_factory=list)

    def record(self, wall: float, stamp_sec: float, frame_id: str) -> None:
        self.recv_wall.append(wall)
        self.header_stamps.append(stamp_sec)
        self.frame_ids.append(frame_id)

    def summarize(self) -> dict:
        n = len(self.recv_wall)
        if n < 2:
            return {"n": n, "status": "no_data"}

        elapsed = self.recv_wall[-1] - self.recv_wall[0]
        hz = (n - 1) / max(elapsed, 1e-6)
        gaps = [b - a for a, b in zip(self.recv_wall, self.recv_wall[1:])]
        jitter = statistics.stdev(gaps) if len(gaps) >= 2 else 0.0
        max_gap = max(gaps)

        now = time.monotonic()
        last_age = now - self.recv_wall[-1]

        # Header stamp age: wall time minus header stamp
        age_values: list[float] = []
        if self.header_stamps and self.recv_wall:
            wall_epoch_offset = time.time() - time.monotonic()
            for w, s in zip(self.recv_wall, self.header_stamps):
                if s > 1e9:   # looks like a Unix timestamp
                    age_values.append((w + wall_epoch_offset) - s)
        max_stamp_age = max(age_values) if age_values else float("nan")

        last_frame = self.frame_ids[-1] if self.frame_ids else ""
        wrong_frame = (
            bool(self.spec.expected_frame_id)
            and last_frame != self.spec.expected_frame_id
        )

        hz_ok = hz >= self.spec.expected_hz * 0.5
        jitter_ok = jitter < (1.0 / max(self.spec.expected_hz, 0.1)) * 2.0
        gap_ok = max_gap < (1.0 / max(self.spec.expected_hz, 0.1)) * 3.0
        age_ok = last_age < 0.5
        frame_ok = not wrong_frame

        issues = []
        if not hz_ok:
            issues.append(f"low_hz ({hz:.2f} < {self.spec.expected_hz * 0.5:.2f})")
        if not jitter_ok:
            issues.append(f"high_jitter ({jitter*1000:.1f}ms)")
        if not gap_ok:
            issues.append(f"large_gap ({max_gap*1000:.0f}ms)")
        if not age_ok:
            issues.append(f"stale ({last_age:.2f}s since last msg)")
        if wrong_frame:
            issues.append(f"wrong_frame_id ({last_frame!r} != {self.spec.expected_frame_id!r})")

        status = "ok" if not issues else ("warn" if self.spec.required else "warn_optional")

        return {
            "n": n,
            "hz_mean": round(hz, 2),
            "jitter_ms": round(jitter * 1000, 1),
            "max_gap_ms": round(max_gap * 1000, 1),
            "last_recv_age_s": round(last_age, 3),
            "max_stamp_age_s": round(max_stamp_age, 3) if not math.isnan(max_stamp_age) else "n/a",
            "last_frame_id": last_frame,
            "qos_profile": "RELIABLE" if self.spec.qos_reliable else "BEST_EFFORT",
            "status": status,
            "issues": issues,
        }


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class AuditNode(Node):
    def __init__(self) -> None:
        super().__init__("mtt_audit_live_topics")
        self._stats: dict[str, TopicStats] = {}

        qos_be = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        qos_rel = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        for spec in TOPIC_SPECS:
            st = TopicStats(spec=spec)
            self._stats[spec.topic] = st
            qos = qos_rel if spec.qos_reliable else qos_be

            if spec.msg_type == "Odometry":
                self.create_subscription(
                    Odometry, spec.topic,
                    lambda msg, s=st: self._on_odom(msg, s),
                    qos,
                )
            elif spec.msg_type == "PointCloud2":
                self.create_subscription(
                    PointCloud2, spec.topic,
                    lambda msg, s=st: self._on_cloud(msg, s),
                    qos,
                )
            elif spec.msg_type == "Float64":
                # /hardware/articulation_angle is std_msgs/Float64 (no header)
                # We record wall time only, no stamp age check.
                from std_msgs.msg import Float64 as F64
                self.create_subscription(
                    F64, spec.topic,
                    lambda msg, s=st: self._on_float64(msg, s),
                    qos,
                )

    def _on_odom(self, msg: Odometry, st: TopicStats) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        st.record(time.monotonic(), stamp, str(msg.header.frame_id))

    def _on_cloud(self, msg: PointCloud2, st: TopicStats) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        st.record(time.monotonic(), stamp, str(msg.header.frame_id))

    def _on_float64(self, msg, st: TopicStats) -> None:
        # std_msgs/Float64 has no header -- record wall time only
        st.record(time.monotonic(), float("nan"), "")

    def get_all_stats(self) -> dict[str, dict]:
        return {topic: st.summarize() for topic, st in self._stats.items()}


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

def _color_status(status: str) -> str:
    if status == "ok":
        return f"{GREEN}ok{RESET}"
    if status.startswith("warn"):
        return f"{YELLOW}warn{RESET}"
    return f"{RED}fail{RESET}"


def _print_report(all_stats: dict[str, dict], duration: float) -> None:
    print(f"\n{BOLD}== MTT Live Topic Audit =={RESET}")
    print(f"duration_s: {duration:.1f}\n")
    fmt = "{:<38s} {:>7s} {:>8s} {:>9s} {:>6s}  {:s}"
    print(fmt.format("topic", "hz_mean", "jit(ms)", "gap(ms)", "n", "status / issues"))
    print("-" * 90)
    any_fail = False
    for topic, s in all_stats.items():
        if s.get("status") == "no_data":
            spec = next(t for t in TOPIC_SPECS if t.topic == topic)
            tag = f"{RED}no_data{RESET}" if spec.required else f"{YELLOW}no_data(opt){RESET}"
            print(fmt.format(topic, "-", "-", "-", "0", tag))
            if spec.required:
                any_fail = True
            continue
        status_str = _color_status(s["status"])
        issues_str = "  ".join(s["issues"]) if s["issues"] else ""
        print(fmt.format(
            topic,
            str(s["hz_mean"]),
            str(s["jitter_ms"]),
            str(s["max_gap_ms"]),
            str(s["n"]),
            f"{status_str}  {issues_str}",
        ))
        if s["status"] not in ("ok", "warn_optional"):
            any_fail = True

    print()
    if any_fail:
        print(f"verdict: {RED}WARN{RESET} - one or more required topics have issues")
    else:
        print(f"verdict: {GREEN}OK{RESET} - all required topics healthy")


def _write_yaml(all_stats: dict[str, dict], path: str, duration: float) -> None:
    import json
    # Simple YAML-like output using json indentation as a proxy
    with open(path, "w") as fh:
        fh.write(f"duration_s: {duration:.1f}\n")
        fh.write("topics:\n")
        for topic, s in all_stats.items():
            fh.write(f"  {topic}:\n")
            for k, v in s.items():
                if isinstance(v, list):
                    if v:
                        fh.write(f"    {k}:\n")
                        for item in v:
                            fh.write(f"      - {json.dumps(item)}\n")
                    else:
                        fh.write(f"    {k}: []\n")
                else:
                    fh.write(f"    {k}: {json.dumps(v)}\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Monitoring duration in seconds (default 30)")
    parser.add_argument("--output", default="",
                        help="Write YAML summary to this file (default: stdout only)")
    args = parser.parse_args()

    rclpy.init()
    node = AuditNode()
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        all_stats = node.get_all_stats()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    _print_report(all_stats, args.duration)

    if args.output:
        _write_yaml(all_stats, args.output, args.duration)
        print(f"\nYAML written to {args.output}")

    any_fail = any(
        s.get("status") not in ("ok", "warn_optional", "no_data")
        or (s.get("status") == "no_data" and
            next(t for t in TOPIC_SPECS if t.topic == topic).required)
        for topic, s in all_stats.items()
    )
    return 1 if any_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
