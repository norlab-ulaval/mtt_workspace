#!/usr/bin/env python3
"""Live ICP mapping health monitor for MTT-154.

Detects and diagnoses the ICP dropout failure cascade:
  - Tracks /mapping/icp_odom Hz and gap duration
  - Tracks /mapping/map point-count growth
  - Parses /rosout for mapper WARN/ERROR rejection reasons
  - Reports consecutive rejection count and time since last accepted scan
  - Alerts in real time when gaps exceed 0.5s / 1.0s thresholds

Usage:
    python3 scripts/audit_icp_health.py [--duration 120] [--output icp_health.yaml]

Compose:
    docker compose run --rm icp_health
"""

from __future__ import annotations

import argparse
import collections
import re
import sys
import time
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2

GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
CYAN = "\033[96m"
RESET = "\033[0m"

# ---------------------------------------------------------------------------
# Rejection reason categories
# ---------------------------------------------------------------------------

REJECTION_PATTERNS: List[tuple[str, re.Pattern]] = [
    ("few_points",   re.compile(r"Too few input points")),
    ("translation",  re.compile(r"Translation correction too large")),
    ("rotation",     re.compile(r"Rotation correction too large")),
    ("velocity",     re.compile(r"Implied velocity too high")),
    ("yaw_rate",     re.compile(r"Implied yaw rate too high")),
    ("timeout",      re.compile(r"ICP took too long")),
    ("convergence",  re.compile(r"convergence|ConvergenceError|BoundTransformation", re.IGNORECASE)),
    ("pose_step",    re.compile(r"pose xy step too high|pose speed too high")),
    ("origin_snap",  re.compile(r"origin snap rejected")),
    ("pose_z",       re.compile(r"z jump too high")),
    ("nan_inf",      re.compile(r"NaN or Inf")),
    ("quality_gate", re.compile(r"Scan rejected by quality gate")),
]

# Mapper node name fragment to filter /rosout (avoid noise from other nodes)
MAPPER_NAME_FILTER = re.compile(r"mapper|norlab_icp", re.IGNORECASE)


@dataclass
class RejectionEvent:
    wall_time: float
    category: str
    raw_msg: str


@dataclass
class IcpOdomSample:
    wall_time: float
    stamp: float
    x: float
    y: float


class IcpHealthMonitor(Node):
    def __init__(self, duration: float):
        super().__init__("icp_health_monitor")
        self._duration = duration
        self._start_wall = time.monotonic()

        # ICP odom tracking
        self._odom_stamps: Deque[float] = collections.deque(maxlen=200)
        self._last_odom_wall: Optional[float] = None
        self._max_gap_s: float = 0.0
        self._gap_alert_threshold_warn = 0.5
        self._gap_alert_threshold_crit = 1.0

        # Map point tracking
        self._map_point_counts: List[tuple[float, int]] = []  # (wall_time, count)
        self._last_map_wall: Optional[float] = None

        # Rejection events from /rosout
        self._rejections: List[RejectionEvent] = []
        self._rejection_counts: Dict[str, int] = collections.defaultdict(int)
        self._consecutive_rejections: int = 0
        self._max_consecutive_rejections: int = 0
        self._last_accepted_wall: Optional[float] = None
        self._total_accepted: int = 0
        self._total_rejected: int = 0

        # Report timing
        self._last_report_wall: float = self._start_wall
        self._report_interval_s: float = 5.0

        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self.create_subscription(Odometry, "/mapping/icp_odom", self._on_odom, be_qos)
        self.create_subscription(PointCloud2, "/mapping/map", self._on_map, be_qos)
        self.create_subscription(Log, "/rosout", self._on_rosout, reliable_qos)

        self.create_timer(self._report_interval_s, self._periodic_report)
        self.create_timer(0.1, self._check_gap)  # 10 Hz gap watchdog

        self.get_logger().info(
            f"ICP health monitor started (duration={duration}s). "
            "Watching /mapping/icp_odom, /mapping/map, /rosout"
        )

    # -----------------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._last_odom_wall is not None:
            gap = now - self._last_odom_wall
            if gap > self._max_gap_s:
                self._max_gap_s = gap
            if gap >= self._gap_alert_threshold_crit:
                elapsed = now - self._start_wall
                print(
                    f"{RED}{BOLD}[{elapsed:6.1f}s] CRITICAL: ICP odom gap = {gap:.2f}s "
                    f"(>{self._gap_alert_threshold_crit}s threshold){RESET}"
                )
            elif gap >= self._gap_alert_threshold_warn:
                elapsed = now - self._start_wall
                print(
                    f"{YELLOW}[{elapsed:6.1f}s] WARN:     ICP odom gap = {gap:.2f}s "
                    f"(>{self._gap_alert_threshold_warn}s threshold){RESET}"
                )

        self._last_odom_wall = now
        self._last_accepted_wall = now
        self._odom_stamps.append(now)
        self._total_accepted += 1
        if self._consecutive_rejections > 0:
            elapsed = now - self._start_wall
            print(
                f"{GREEN}[{elapsed:6.1f}s] RECOVERED after {self._consecutive_rejections} "
                f"consecutive rejections{RESET}"
            )
        self._consecutive_rejections = 0

    def _on_map(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        count = msg.width * msg.height
        self._map_point_counts.append((now, count))
        self._last_map_wall = now

    def _on_rosout(self, msg: Log) -> None:
        # Only care about WARN (4) and ERROR (8) from mapper
        if msg.level < 4:
            return
        node_name = getattr(msg, "name", "") or ""
        text = msg.msg or ""
        if not MAPPER_NAME_FILTER.search(node_name) and not MAPPER_NAME_FILTER.search(text):
            return

        now = time.monotonic()
        category = "other"
        for cat, pattern in REJECTION_PATTERNS:
            if pattern.search(text):
                category = cat
                break

        # Only count scan-level rejections, not map-update skips
        is_rejection = any(
            p.search(text) for _, p in REJECTION_PATTERNS
        ) or "rejected" in text.lower()

        if is_rejection:
            self._rejections.append(RejectionEvent(now, category, text[:120]))
            self._rejection_counts[category] += 1
            self._consecutive_rejections += 1
            self._total_rejected += 1
            if self._consecutive_rejections > self._max_consecutive_rejections:
                self._max_consecutive_rejections = self._consecutive_rejections

            elapsed = now - self._start_wall
            level_str = f"{RED}ERROR{RESET}" if msg.level >= 8 else f"{YELLOW}WARN {RESET}"
            print(
                f"[{elapsed:6.1f}s] {level_str}  [{category}] "
                f"{text[:100]}"
            )

    def _check_gap(self) -> None:
        """10 Hz watchdog: alert if odom has been silent too long during a run."""
        if self._last_odom_wall is None:
            return
        now = time.monotonic()
        gap = now - self._last_odom_wall
        if gap >= self._gap_alert_threshold_crit and self._consecutive_rejections == 0:
            # The odom stream stopped but we haven't seen a rejection log yet
            # (mapper may have lost the LiDAR topic entirely — no /rosout error)
            elapsed = now - self._start_wall
            print(
                f"{RED}{BOLD}[{elapsed:6.1f}s] CRITICAL: ICP odom silent for {gap:.1f}s "
                f"(no rejection log seen — check LiDAR QoS / topic){RESET}"
            )

    def _periodic_report(self) -> None:
        now = time.monotonic()
        elapsed = now - self._start_wall

        # Hz over last 10s window
        recent = [t for t in self._odom_stamps if now - t <= 10.0]
        hz = len(recent) / 10.0 if len(recent) >= 2 else 0.0

        # Map point growth rate
        map_info = "no map received"
        if len(self._map_point_counts) >= 2:
            t0, c0 = self._map_point_counts[0]
            t1, c1 = self._map_point_counts[-1]
            dt = max(t1 - t0, 1e-3)
            rate = (c1 - c0) / dt
            map_info = f"{c1:,} pts, growth {rate:+.0f} pts/s"
        elif len(self._map_point_counts) == 1:
            map_info = f"{self._map_point_counts[-1][1]:,} pts"

        # Time since last accepted
        if self._last_accepted_wall is not None:
            since_accepted = now - self._last_accepted_wall
            since_str = f"{since_accepted:.1f}s ago"
        else:
            since_str = "never"

        # Build rejection summary
        rej_summary = " ".join(
            f"{cat}:{cnt}" for cat, cnt in sorted(self._rejection_counts.items())
        ) or "none"

        # Gap bar
        gap_color = GREEN if self._max_gap_s < 0.5 else (YELLOW if self._max_gap_s < 1.0 else RED)

        print(f"\n{BOLD}{CYAN}━━━ ICP Health [{elapsed:.0f}s] ━━━{RESET}")
        print(f"  odom Hz (last 10s) : {_hz_color(hz)}{hz:.1f} Hz{RESET}  (expected ~10 Hz)")
        print(f"  max gap seen       : {gap_color}{self._max_gap_s:.2f}s{RESET}")
        print(f"  last accepted      : {since_str}")
        print(f"  accepted / rejected: {self._total_accepted} / {self._total_rejected}")
        print(f"  max consec rejects : {self._max_consecutive_rejections}")
        print(f"  rejection types    : {rej_summary}")
        print(f"  map                : {map_info}")

    def done(self) -> bool:
        return time.monotonic() - self._start_wall >= self._duration

    def summary(self) -> dict:
        now = time.monotonic()
        recent = [t for t in self._odom_stamps if now - t <= 10.0]
        hz = len(recent) / 10.0 if len(recent) >= 2 else 0.0

        map_count = self._map_point_counts[-1][1] if self._map_point_counts else 0

        return {
            "duration_s": self._duration,
            "odom_hz_last_10s": round(hz, 2),
            "max_gap_s": round(self._max_gap_s, 3),
            "total_accepted": self._total_accepted,
            "total_rejected": self._total_rejected,
            "max_consecutive_rejections": self._max_consecutive_rejections,
            "rejection_counts": dict(self._rejection_counts),
            "map_point_count_final": map_count,
            "verdict": _verdict(self._max_gap_s, self._total_rejected, hz),
        }


def _hz_color(hz: float) -> str:
    if hz >= 8.0:
        return GREEN
    if hz >= 5.0:
        return YELLOW
    return RED


def _verdict(max_gap: float, total_rejected: int, hz: float) -> str:
    if max_gap >= 1.0:
        return "FAIL: critical ICP gaps detected"
    if max_gap >= 0.5:
        return "WARN: intermittent ICP gaps"
    if total_rejected > 0:
        return "WARN: scan rejections detected (review rejection_counts)"
    if hz < 8.0:
        return "WARN: ICP Hz below expected"
    return "OK"


def main() -> None:
    parser = argparse.ArgumentParser(description="Live ICP mapping health monitor")
    parser.add_argument("--duration", type=float, default=120.0,
                        help="Monitoring duration in seconds (default: 120)")
    parser.add_argument("--output", type=str, default="",
                        help="Optional YAML output path for summary")
    args = parser.parse_args()

    rclpy.init()
    node = IcpHealthMonitor(duration=args.duration)

    print(f"{BOLD}ICP Health Monitor — {args.duration:.0f}s window{RESET}")
    print(f"  watching: /mapping/icp_odom  /mapping/map  /rosout")
    print(f"  thresholds: gap>0.5s=WARN  gap>1.0s=CRIT  consecutive rejections tracked")
    print()

    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass

    summary = node.summary()
    verdict = summary["verdict"]
    color = GREEN if verdict.startswith("OK") else (YELLOW if verdict.startswith("WARN") else RED)

    print(f"\n{BOLD}{'━'*50}{RESET}")
    print(f"{BOLD}Final verdict: {color}{verdict}{RESET}")
    print(f"  ICP odom Hz    : {summary['odom_hz_last_10s']:.1f} Hz")
    print(f"  Max gap        : {summary['max_gap_s']:.3f}s")
    print(f"  Accepted/Rej   : {summary['total_accepted']} / {summary['total_rejected']}")
    print(f"  Max consec rej : {summary['max_consecutive_rejections']}")
    print(f"  Rejection types: {summary['rejection_counts']}")
    print(f"  Map pts (final): {summary['map_point_count_final']:,}")

    if args.output:
        import yaml
        with open(args.output, "w") as f:
            yaml.dump(summary, f, default_flow_style=False)
        print(f"  Summary written: {args.output}")

    node.destroy_node()
    rclpy.shutdown()

    rc = 0 if verdict.startswith("OK") else (1 if verdict.startswith("WARN") else 2)
    sys.exit(rc)


if __name__ == "__main__":
    main()
