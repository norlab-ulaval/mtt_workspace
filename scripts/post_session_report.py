#!/usr/bin/env python3
"""
post_session_report.py — Generate a Markdown session report from a recorded bag.

Reads the session_info.yaml sidecar and runs `ros2 bag info` to collect
statistics, then writes report.md next to the bag.

Usage:
  python3 scripts/post_session_report.py <bag_directory>
  python3 scripts/post_session_report.py data/mtt_motion_model_straight_snow_2024-...

Output:
  <bag_directory>/report.md
  (also printed to stdout)
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import yaml


# ── ros2 bag info parser ──────────────────────────────────────────────────────

def run_ros2_bag_info(bag_dir: Path) -> dict:
    """
    Call `ros2 bag info` on the bag directory and parse the output into a dict.
    Returns empty dict if ros2 is not available or bag not found.
    """
    # Try JSON output first (ros2 jazzy supports --output json for some commands)
    # Fallback to text parsing
    try:
        result = subprocess.run(
            ["ros2", "bag", "info", str(bag_dir)],
            capture_output=True, text=True, timeout=30
        )
        return _parse_bag_info_text(result.stdout)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return {}


def _parse_bag_info_text(text: str) -> dict:
    """Parse `ros2 bag info` text output into a structured dict."""
    info: dict = {"raw": text, "topics": []}
    if not text:
        return info

    for line in text.splitlines():
        line = line.strip()

        m = re.match(r"Files:\s+(.+)", line)
        if m:
            info["files"] = m.group(1).strip()

        m = re.match(r"Bag size:\s+(.+)", line)
        if m:
            info["bag_size"] = m.group(1).strip()

        m = re.match(r"Storage id:\s+(.+)", line)
        if m:
            info["storage_id"] = m.group(1).strip()

        m = re.match(r"Duration:\s+(.+)", line)
        if m:
            info["duration"] = m.group(1).strip()

        m = re.match(r"Start:\s+(.+)", line)
        if m:
            info["start"] = m.group(1).strip()

        m = re.match(r"End:\s+(.+)", line)
        if m:
            info["end"] = m.group(1).strip()

        m = re.match(r"Messages:\s+(\d+)", line)
        if m:
            info["total_messages"] = int(m.group(1))

        m = re.match(r"Topic count:\s+(\d+)", line)
        if m:
            info["topic_count"] = int(m.group(1))

        # Topic lines look like: "Topic: /mtt_odometry | Type: nav_msgs/msg/Odometry | Count: 3000 | Serialization Format: cdr"
        m = re.match(r"Topic:\s+(\S+)\s+\|\s+Type:\s+(\S+)\s+\|\s+Count:\s+(\d+)", line)
        if m:
            info["topics"].append({
                "topic": m.group(1),
                "type": m.group(2),
                "count": int(m.group(3)),
            })

    return info


# ── Report generator ──────────────────────────────────────────────────────────

def generate_report(bag_dir: Path) -> str:
    session_info_path = bag_dir / "session_info.yaml"
    session: dict = {}
    if session_info_path.exists():
        with session_info_path.open() as f:
            session = yaml.safe_load(f) or {}

    bag_info = run_ros2_bag_info(bag_dir)

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    duration_str = bag_info.get("duration", "unknown")
    bag_size_str = bag_info.get("bag_size", "unknown")
    total_msgs = bag_info.get("total_messages", "unknown")
    topic_count = bag_info.get("topic_count", len(bag_info.get("topics", [])))

    # Infer duration in seconds for Hz computation
    duration_s: Optional[float] = None
    if isinstance(duration_str, str):
        m = re.search(r"(\d+\.?\d*)s", duration_str)
        if m:
            duration_s = float(m.group(1))

    lines = []
    lines.append(f"# MTT Session Report")
    lines.append(f"")
    lines.append(f"Generated: {now}  ")
    lines.append(f"Bag: `{bag_dir.name}`")
    lines.append(f"")

    # ── Session metadata ──────────────────────────────────────────────────────
    lines.append(f"## Session Metadata")
    lines.append(f"")
    lines.append(f"| Field | Value |")
    lines.append(f"|---|---|")
    fields = [
        ("Experiment",      session.get("experiment_name", "—")),
        ("Type",            session.get("session_type", "—")),
        ("Terrain",         session.get("terrain", "—")),
        ("Trailer",         session.get("trailer_attached", "—")),
        ("Operator",        session.get("operator", "—")),
        ("Weather",         session.get("weather", "—")),
        ("Temperature",     f"{session.get('temperature_c', '—')} °C"),
        ("GPS mode",        session.get("gps_mode", "—")),
        ("Started at",      session.get("started_at", "—")),
        ("Notes",           session.get("notes", "—") or "—"),
    ]
    for k, v in fields:
        lines.append(f"| {k} | {v} |")
    lines.append(f"")

    # ── Bag statistics ────────────────────────────────────────────────────────
    lines.append(f"## Bag Statistics")
    lines.append(f"")
    lines.append(f"| Metric | Value |")
    lines.append(f"|---|---|")
    lines.append(f"| Duration | {duration_str} |")
    lines.append(f"| Bag size | {bag_size_str} |")
    lines.append(f"| Total messages | {total_msgs} |")
    lines.append(f"| Topic count | {topic_count} |")
    lines.append(f"| Storage | {bag_info.get('storage_id', 'mcap')} |")
    lines.append(f"")

    # ── Topic breakdown ───────────────────────────────────────────────────────
    topics = bag_info.get("topics", [])
    if topics:
        lines.append(f"## Recorded Topics ({len(topics)})")
        lines.append(f"")

        # Group by sensor
        groups = {
            "System / TF": ["/clock", "/tf", "/tf_static", "/robot_description",
                             "/joint_states", "/session/events"],
            "MTT CAN state": ["/mtt_odometry", "/mtt_tachometer", "/mtt_articulation_angle",
                               "/mtt_status", "/mtt_steer_cmd", "/mtt_driving_mode",
                               "/mtt_aux_cmd", "/initialpose"],
            "Commands": ["/cmd_vel", "/cmd_vel/nav", "/cmd_vel/teleop", "/controller/cmd_vel",
                         "/joy", "/joy/set_feedback"],
            "IMU": [t for t in [tp["topic"] for tp in topics] if t.startswith("/mti")],
            "LiDAR": [t for t in [tp["topic"] for tp in topics]
                      if "hesai" in t or "rsairy" in t],
            "ZED Camera": [t for t in [tp["topic"] for tp in topics] if "/zed/" in t],
            "OAK Camera": [t for t in [tp["topic"] for tp in topics] if "/oak/" in t],
            "GPS": [t for t in [tp["topic"] for tp in topics] if "/gps" in t],
            "Mapping / Localization": [t for t in [tp["topic"] for tp in topics]
                                       if t.startswith("/mapping") or t.startswith("/localization")
                                       or t.startswith("/trailer")],
        }

        topic_map = {tp["topic"]: tp for tp in topics}
        printed = set()

        lines.append(f"| Topic | Type | Messages | Avg Hz |")
        lines.append(f"|---|---|---|---|")

        for group_name, group_topics in groups.items():
            group_rows = []
            for t in group_topics:
                if t in topic_map and t not in printed:
                    tp = topic_map[t]
                    hz_str = "—"
                    if duration_s and duration_s > 0:
                        hz = tp["count"] / duration_s
                        hz_str = f"{hz:.1f}"
                    group_rows.append((tp["topic"], tp["type"].split("/")[-1],
                                       tp["count"], hz_str))
                    printed.add(t)
            if group_rows:
                lines.append(f"| **{group_name}** | | | |")
                for topic, msg_type, count, hz in group_rows:
                    lines.append(f"| `{topic}` | {msg_type} | {count:,} | {hz} |")

        # Any remaining topics not in groups
        remaining = [tp for tp in topics if tp["topic"] not in printed]
        if remaining:
            lines.append(f"| **Other** | | | |")
            for tp in remaining:
                hz_str = "—"
                if duration_s and duration_s > 0:
                    hz_str = f"{tp['count'] / duration_s:.1f}"
                lines.append(f"| `{tp['topic']}` | {tp['type'].split('/')[-1]} | "
                              f"{tp['count']:,} | {hz_str} |")
        lines.append(f"")

    # ── Sensor coverage summary ───────────────────────────────────────────────
    lines.append(f"## Sensor Coverage")
    lines.append(f"")
    recorded_topics = {tp["topic"] for tp in topics}

    def coverage(name: str, topics_to_check: list[str]) -> str:
        present = [t for t in topics_to_check if t in recorded_topics]
        if len(present) == len(topics_to_check):
            return f"✅ {name} — all {len(present)} topics"
        elif present:
            return f"⚠️  {name} — {len(present)}/{len(topics_to_check)} topics"
        else:
            return f"❌ {name} — not recorded"

    lines.append(coverage("CAN / Odometry",
        ["/mtt_odometry", "/mtt_tachometer", "/mtt_articulation_angle"]))
    lines.append(coverage("IMU MTi-100",
        ["/mti100/data", "/mti100/data_raw", "/mti100/time_reference"]))
    lines.append(coverage("IMU MTi-10", ["/mti10/data"]))
    lines.append(coverage("Hesai LiDAR",
        ["/hesai_lidar/points", "/hesai_lidar/lidar_packets"]))
    lines.append(coverage("RS Bpearl LiDAR", ["/rsairy_ns/points"]))
    lines.append(coverage("ZED Camera",
        ["/zed/zed_node/rgb/image_rect_color/compressed",
         "/zed/zed_node/depth/depth_registered/compressedDepth"]))
    lines.append(coverage("OAK-D Camera", ["/oak/rgb/image_raw/compressed"]))
    lines.append(coverage("GPS",
        ["/gps_left/fix", "/gps_right/fix", "/gps/heading"]))
    lines.append(coverage("ICP Mapping", ["/mapping/icp_odom"]))
    lines.append("")

    # ── Events / annotations ──────────────────────────────────────────────────
    lines.append(f"## Operator Annotations")
    lines.append(f"")
    lines.append(f"To review in-bag annotations after playback:")
    lines.append(f"```bash")
    lines.append(f"ros2 bag play {bag_dir} --topics /session/events &")
    lines.append(f"ros2 topic echo /session/events")
    lines.append(f"```")
    lines.append(f"")

    # ── Post-processing hints ─────────────────────────────────────────────────
    lines.append(f"## Post-Processing Hints")
    lines.append(f"")
    lines.append(f"```bash")
    lines.append(f"# Replay all sensors (real-time)")
    lines.append(f"ros2 bag play {bag_dir}")
    lines.append(f"")
    lines.append(f"# Replay at 0.5× speed for debugging")
    lines.append(f"ros2 bag play {bag_dir} --rate 0.5")
    lines.append(f"")
    lines.append(f"# Extract only CAN + IMU for motion model ID")
    lines.append(f"ros2 bag filter {bag_dir} -o /tmp/mtt_motion_model \\")
    lines.append(f"  --include-topic /mtt_odometry /mtt_tachometer "
                 f"/mtt_articulation_angle /mtt_steer_cmd /mti100/data")
    lines.append(f"```")
    lines.append(f"")

    return "\n".join(lines)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> int:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <bag_directory>")
        return 1

    bag_dir = Path(sys.argv[1]).resolve()
    if not bag_dir.exists():
        print(f"Error: bag directory not found: {bag_dir}")
        return 1

    report = generate_report(bag_dir)

    report_path = bag_dir / "report.md"
    report_path.write_text(report, encoding="utf-8")

    print(report)
    print(f"\n{'─' * 60}")
    print(f"Report written to: {report_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
