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


# ── Session / bag path resolver ──

def resolve_session_and_bag_dir(path: Path) -> tuple[Path, Path]:
    path = path.resolve()
    if path.is_file():
        bag_dir = path.parent
        session_dir = bag_dir.parent if bag_dir.name == "bag" else bag_dir
        return session_dir, bag_dir
    if (path / "bag" / "metadata.yaml").exists():
        return path, path / "bag"
    if (path / "metadata.yaml").exists():
        session_dir = path.parent if path.parent.name == "bag" else path.parent
        return session_dir, path
    raise FileNotFoundError(f"Could not resolve session/bag directory from: {path}")


# ── ros2 bag info parser ──

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
        parsed = _parse_bag_info_text(result.stdout)
        if parsed.get("topics"):
            return parsed
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return _load_bag_info_from_metadata(bag_dir / "metadata.yaml")


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


def _load_bag_info_from_metadata(metadata_path: Path) -> dict:
    """Fallback parser when `ros2 bag info` is unavailable or returns no text."""
    if not metadata_path.exists():
        return {"raw": "", "topics": []}

    try:
        with metadata_path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except yaml.YAMLError:
        data = {}
    info = data.get("rosbag2_bagfile_information", data)

    duration_ns = int(info.get("duration", {}).get("nanoseconds", 0))
    start_ns = int(info.get("starting_time", {}).get("nanoseconds_since_epoch", 0))
    topics = []
    for entry in info.get("topics_with_message_count", []):
        meta = entry.get("topic_metadata", {})
        topics.append({
            "topic": meta.get("name", ""),
            "type": meta.get("type", ""),
            "count": int(entry.get("message_count", 0)),
        })

    relative_paths = []
    for path_info in info.get("relative_file_paths", []):
        if isinstance(path_info, str):
            relative_paths.append(path_info)
        elif path_info:
            relative_paths.append(path_info.get("path", ""))

    bag_dir = metadata_path.parent
    total_size_bytes = 0
    for rel_path in relative_paths:
        file_path = bag_dir / rel_path
        if file_path.exists():
            total_size_bytes += file_path.stat().st_size

    if total_size_bytes >= 1024 ** 3:
        bag_size = f"{total_size_bytes / (1024 ** 3):.2f} GiB"
    elif total_size_bytes >= 1024 ** 2:
        bag_size = f"{total_size_bytes / (1024 ** 2):.2f} MiB"
    elif total_size_bytes > 0:
        bag_size = f"{total_size_bytes} B"
    else:
        bag_size = "unknown"

    return {
        "raw": "",
        "storage_id": info.get("storage_identifier", "mcap"),
        "duration": f"{duration_ns / 1e9:.9f}s" if duration_ns else "unknown",
        "bag_size": bag_size,
        "total_messages": int(info.get("message_count", 0)),
        "topic_count": len(topics),
        "topics": topics,
        "start": str(start_ns) if start_ns else "unknown",
        "files": ", ".join(path for path in relative_paths if path),
    }


# ── Report generator ──

def generate_report(session_dir: Path, bag_dir: Path) -> str:
    session_info_path = session_dir / "session_info.yaml"
    session: dict = {}
    if session_info_path.exists():
        try:
            with session_info_path.open("r", encoding="utf-8") as f:
                session = yaml.safe_load(f) or {}
        except yaml.YAMLError:
            pass

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
    lines.append(f"Bag session: `{session_dir.name}`  ")
    lines.append(f"Bag directory: `{bag_dir}`")
    lines.append(f"")

    # ── Session metadata ──
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
        ("GPS antennas",    session.get("gps_antennas", "—")),
        ("Tachometer mode", session.get("tachometer_mode", "—")),
        ("Started at",      session.get("started_at", "—")),
        ("Notes",           session.get("notes", "—") or "—"),
    ]
    for k, v in fields:
        lines.append(f"| {k} | {v} |")
    lines.append(f"")

    # ── Bag statistics ──
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

    # ── Topic breakdown ──
    topics = bag_info.get("topics", [])
    topic_map = {tp["topic"]: tp for tp in topics}
    if topics:
        lines.append(f"## Recorded Topics ({len(topics)})")
        lines.append(f"")

        # Group by sensor
        groups = {
            "System / TF": ["/tf", "/tf_static", "/robot_description",
                             "/joint_states", "/runtime_joint_states", "/session/events"],
            "MTT CAN state": ["/mtt_odometry", "/mtt_tachometer", "/mtt_articulation_angle",
                               "/mtt_monitor/cmd_fallback_odom",
                               "/mtt_status", "/mtt_steer_cmd", "/mtt_driving_mode",
                               "/mtt_aux_cmd", "/initialpose"],
            "Commands": ["/cmd_vel", "/cmd_vel/manual_raw", "/cmd_vel/manual", "/controller/cmd_vel",
                         "/selected_mode", "/auto_mode_enabled", "/mtt_control/manual_activity",
                         "/mtt_control/selected_source", "/joy", "/joy/set_feedback",
                         "/teleop_deadman", "/teleop_estop"],
            "IMU": [t for t in [tp["topic"] for tp in topics] if t.startswith("/mti")],
            "LiDAR": [t for t in [tp["topic"] for tp in topics]
                      if "hesai" in t or "rsairy" in t],
            "ZED Camera": [t for t in [tp["topic"] for tp in topics] if "/zed/" in t],
            "OAK Camera": [t for t in [tp["topic"] for tp in topics] if "/oak/" in t],
            "GPS": [t for t in [tp["topic"] for tp in topics] if "/gps" in t],
            "Perception": [t for t in [tp["topic"] for tp in topics]
                           if t.startswith("/merged_points") or t.startswith("/trailer/")],
            "Mapping / Localization": [t for t in [tp["topic"] for tp in topics]
                                       if t.startswith("/mapping") or t.startswith("/localization")],
            "Teach / Repeat": [t for t in [tp["topic"] for tp in topics]
                               if t.startswith("/wiln/")
                               or t.startswith("/planned_trajectory")
                               or t.startswith("/real_trajectory")
                               or t.startswith("/mtt_path_follower/")
                               or t.startswith("/mtt_repeat/")],
        }

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

    # ── Sensor coverage summary ──
    lines.append(f"## Sensor Coverage")
    lines.append(f"")
    recorded_topics = {tp["topic"] for tp in topics}

    def coverage(name: str, topics_to_check: list[str]) -> str:
        present = [t for t in topics_to_check if topic_map.get(t, {}).get("count", 0) > 0]
        if len(present) == len(topics_to_check):
            return f"✅ {name} — all {len(present)} topics"
        elif present:
            return f"⚠️  {name} — {len(present)}/{len(topics_to_check)} topics"
        else:
            return f"❌ {name} — not recorded"

    lines.append(coverage("CAN / Odometry",
        ["/mtt_odometry", "/mtt_tachometer", "/mtt_articulation_angle"]))
    lines.append(coverage("Fallback Odom",
        ["/mtt_monitor/cmd_fallback_odom"]))
    lines.append(coverage("IMU MTi-100",
        ["/mti100/data", "/mti100/data_raw", "/mti100/time_reference"]))
    lines.append(coverage("IMU MTi-10", ["/mti10/data"]))
    lines.append(coverage("Hesai LiDAR",
        ["/hesai_lidar/points", "/lidar_packets"]))
    lines.append(coverage("RS Bpearl LiDAR", ["/rsairy_ns/points"]))
    lines.append(coverage("ZED Camera",
        ["/zed/zed_node/rgb/color/rect/image/compressed",
         "/zed/zed_node/depth/depth_registered/compressedDepth",
         "/zed/zed_node/point_cloud/cloud_registered"]))
    lines.append(coverage("OAK-D Camera",
        ["/oak/rgb/image_rect",
         "/oak/stereo/image_raw",
         "/oak/points"]))
    lines.append(coverage("GPS single",
        ["/gps/fix", "/gps/nmea_sentence", "/gps/time_reference"]))
    lines.append(coverage("GPS dual (legacy)",
        ["/gps_left/fix", "/gps_right/fix", "/gps/heading"]))
    lines.append(coverage("Perception / Merged Cloud",
        ["/merged_points_filtered", "/trailer/angle"]))
    lines.append(coverage("ICP Mapping", ["/mapping/icp_odom"]))
    lines.append(coverage("Teach / Repeat",
        ["/wiln/pose", "/planned_trajectory", "/real_trajectory", "/mtt_repeat/state"]))
    lines.append("")

    artifact_dir = session_dir / "artifacts"
    if artifact_dir.exists():
        lines.append("## Saved Artifacts")
        lines.append("")
        for artifact in [
            "final_map.vtk",
            "final_trajectory.vtk",
            "final_route.ltr",
            "save_map_call.txt",
            "save_trajectory_call.txt",
            "save_wiln_route_call.txt",
        ]:
            path = artifact_dir / artifact
            lines.append(f"- `{artifact}`: {'present' if path.exists() else 'missing'}")
        lines.append("")

    lines.append(f"## Key Findings")
    lines.append(f"")
    gps_nmea_count = topic_map.get("/gps/nmea_sentence", {}).get("count", 0)
    gps_fix_count = topic_map.get("/gps/fix", {}).get("count", 0)
    gps_timeref_count = topic_map.get("/gps/time_reference", {}).get("count", 0)
    oak_rgb_count = topic_map.get("/oak/rgb/image_rect", {}).get("count", 0)
    oak_depth_count = topic_map.get("/oak/stereo/image_raw", {}).get("count", 0)
    oak_points_count = topic_map.get("/oak/points", {}).get("count", 0)

    if gps_nmea_count > 0 and gps_fix_count == 0:
        lines.append("- GPS NMEA is present but `/gps/fix` is empty. The Reach link is alive, but valid GGA is missing or rejected by the parser.")
    if gps_fix_count == 0 and gps_timeref_count == 0:
        lines.append("- `/gps/time_reference` is also empty, so this session does not contain a usable GPS UTC time anchor.")
    if oak_rgb_count > 0 and oak_depth_count == 0:
        lines.append("- OAK RGB is present without OAK depth. This points to an RGB-only or degraded USB/runtime state, not a rosbag writer failure.")
    if oak_rgb_count > 0 and oak_points_count == 0:
        lines.append("- OAK point cloud is absent. Check the RGBD pipeline and `pointcloud.enable` setting, then verify the OAK USB connection after vibration.")
    if "/mapping/icp_odom" in recorded_topics and session.get("tachometer_mode") == "cmd_sim":
        lines.append("- ICP odometry is available, but the session uses synthetic tachometer/odometry. Treat ICP as a local motion reference, not as ground truth.")
    if len(lines) > 0 and lines[-1] == "":
        pass
    lines.append("")

    lines.append(f"## State / Reference Semantics")
    lines.append(f"")
    if session.get("tachometer_mode") == "cmd_sim":
        lines.append("- `/mtt_tachometer` and `/mtt_odometry` are synthetic, derived from the commanded motion rather than a real wheel/tachometer sensor.")
        lines.append("- `/mtt_monitor/cmd_fallback_odom` is the explicit command-only fallback and should be treated as a degraded state estimate.")
    else:
        lines.append("- `/mtt_tachometer` and `/mtt_odometry` come from the live MTT CAN/tachometer chain.")
    if "/mapping/icp_odom" in recorded_topics and session.get("tachometer_mode") == "cmd_sim":
        lines.append("- `/mapping/icp_odom` is a useful local ICP reference, but not ground truth, because scan deskew and motion prior may depend on synthetic odometry.")
    elif "/mapping/icp_odom" in recorded_topics:
        lines.append("- `/mapping/icp_odom` is a local ICP odometry reference. It remains an estimate, not a survey-grade truth source.")
    if session.get("gps_antennas") == "single":
        lines.append("- GPS is running in single-rover mode. `/gps/fix` and `/gps/time_reference` must be present to use it for time or global consistency checks.")
    lines.append("")

    # ── Events / annotations ──
    lines.append(f"## Operator Annotations")
    lines.append(f"")
    lines.append(f"To review in-bag annotations after playback:")
    lines.append(f"```bash")
    lines.append(f"ros2 bag play {bag_dir} --topics /session/events &")
    lines.append(f"ros2 topic echo /session/events")
    lines.append(f"```")
    lines.append(f"")

    # ── Post-processing hints ──
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


# ── Terminal UI & Multi-bag ──

class Colors:
    OK = "\033[92m"
    WARN = "\033[93m"
    FAIL = "\033[91m"
    CYAN = "\033[96m"
    BOLD = "\033[1m"
    RESET = "\033[0m"


def extract_session_stats(session_dir: Path, bag_dir: Path) -> dict:
    """Extracts high-level metrics for the comparative terminal table."""
    session_info_path = session_dir / "session_info.yaml"
    session: dict = {}
    if session_info_path.exists():
        try:
            with session_info_path.open("r", encoding="utf-8") as f:
                session = yaml.safe_load(f) or {}
        except yaml.YAMLError:
            pass

    bag_info = run_ros2_bag_info(bag_dir)
    duration_str = str(bag_info.get("duration", "0s"))
    bag_size = bag_info.get("bag_size", "unknown")
    
    duration_s = 0.0
    m = re.search(r"(\d+\.?\d*)s", duration_str)
    if m:
        duration_s = float(m.group(1))

    topics = bag_info.get("topics", [])
    topic_map = {tp["topic"]: tp for tp in topics}

    def get_hz(topic_name: str) -> float:
        if duration_s <= 0: return 0.0
        return topic_map.get(topic_name, {}).get("count", 0) / duration_s

    lidar_hz = get_hz("/hesai_lidar/points") or get_hz("/rsairy_ns/points")
    zed_hz = get_hz("/zed/zed_node/rgb/color/rect/image/compressed")
    oak_hz = get_hz("/oak/rgb/image_rect")
    can_hz = get_hz("/mtt_odometry")
    gps_fix = topic_map.get("/gps/fix", {}).get("count", 0)

    # Status logic
    status = "OK"
    warnings = []
    
    if duration_s < 5:
        status = "WARN"
        warnings.append("Very short (<5s)")
    if lidar_hz < 5.0 and zed_hz < 5.0 and oak_hz < 5.0:
        status = "FAIL"
        warnings.append("No vision/LiDAR")
    elif lidar_hz > 0 and lidar_hz < 8.0:
        status = "WARN"
        warnings.append("Low LiDAR Hz")
    
    if can_hz > 0 and can_hz < 10.0:
        if status == "OK": status = "WARN"
        warnings.append("Low CAN Hz")
        
    if not topics:
        status = "FAIL"
        warnings.append("Bag is empty or corrupt")

    return {
        "name": session_dir.name,
        "duration": duration_str,
        "size": bag_size,
        "lidar_hz": lidar_hz,
        "zed_hz": zed_hz,
        "oak_hz": oak_hz,
        "can_hz": can_hz,
        "gps": "✓" if gps_fix > 0 else "✗",
        "status": status,
        "warnings": ", ".join(warnings)
    }


def print_comparative_table(stats_list: list[dict]):
    if not stats_list:
        print(f"{Colors.WARN}No sessions found.{Colors.RESET}")
        return

    print(f"\n{Colors.BOLD}{Colors.CYAN}══════════════════════════════════════════════════════════════════════════════════════════════════════════{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}   MTT Multi-Session Analysis{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}══════════════════════════════════════════════════════════════════════════════════════════════════════════{Colors.RESET}\n")

    header = f" {Colors.BOLD}Status | Session Name                                       | Duration | Size    | LiDAR | ZED  | OAK  | CAN  | GPS{Colors.RESET} "
    print(header)
    print("-" * 105)

    for s in stats_list:
        name = s['name']
        if len(name) > 48: name = name[:45] + "..."
        
        dur = s['duration'].replace("s", "")
        if len(dur) > 8: dur = dur[:8]
        
        def color_hz(hz, min_ok, min_warn):
            val = f"{hz:4.1f}"
            if hz == 0.0: return f"{Colors.FAIL}  — {Colors.RESET}"
            if hz >= min_ok: return f"{Colors.OK}{val}{Colors.RESET}"
            if hz >= min_warn: return f"{Colors.WARN}{val}{Colors.RESET}"
            return f"{Colors.FAIL}{val}{Colors.RESET}"

        l_hz = color_hz(s['lidar_hz'], 9.0, 5.0)
        z_hz = color_hz(s['zed_hz'], 10.0, 5.0)
        o_hz = color_hz(s['oak_hz'], 10.0, 5.0)
        c_hz = color_hz(s['can_hz'], 40.0, 20.0)
        
        gps_color = Colors.OK if s['gps'] == '✓' else Colors.FAIL
        gps_str = f"{gps_color}{s['gps']}{Colors.RESET}  "
        
        if s['status'] == 'OK':
            status_icon = f"{Colors.OK}  ✓   {Colors.RESET}"
        elif s['status'] == 'WARN':
            status_icon = f"{Colors.WARN}  ⚠   {Colors.RESET}"
        else:
            status_icon = f"{Colors.FAIL}  ✗   {Colors.RESET}"

        row = f" {status_icon}| {name:<48} | {dur:>8} | {s['size']:>7} | {l_hz} | {z_hz} | {o_hz} | {c_hz} | {gps_str}"
        print(row)
        
        if s['warnings']:
            print(f"        {Colors.WARN}↳ Warnings: {s['warnings']}{Colors.RESET}")

    print("\n" + "-" * 105 + "\n")


# ── Entry point ──

def main() -> int:
    import argparse
    parser = argparse.ArgumentParser(description="Generate a Markdown session report from a bag or summarize multiple bags.")
    parser.add_argument("path", type=Path, help="Path to a single bag session or a directory containing multiple sessions.")
    args = parser.parse_args()

    input_path = args.path.resolve()
    if not input_path.exists():
        print(f"{Colors.FAIL}Error: bag directory not found: {input_path}{Colors.RESET}")
        return 1

    sessions_to_process = []
    
    # Try resolving as a single session
    try:
        session_dir, bag_dir = resolve_session_and_bag_dir(input_path)
        sessions_to_process.append((session_dir, bag_dir))
    except FileNotFoundError:
        # If it fails, assume it's a parent directory containing multiple sessions
        if input_path.is_dir():
            for child in input_path.iterdir():
                if child.is_dir() and not child.name.startswith("."):
                    try:
                        s_dir, b_dir = resolve_session_and_bag_dir(child)
                        if (s_dir, b_dir) not in sessions_to_process:
                            sessions_to_process.append((s_dir, b_dir))
                    except FileNotFoundError:
                        continue

    if not sessions_to_process:
        print(f"{Colors.FAIL}Error: No valid ROS 2 bag sessions found in {input_path}{Colors.RESET}")
        return 1

    sessions_to_process.sort(key=lambda x: x[0].name, reverse=True) # Show newest first if timestamped

    print(f"{Colors.BOLD}Analyzing {len(sessions_to_process)} session(s)...{Colors.RESET}")
    
    stats_list = []
    for i, (s_dir, b_dir) in enumerate(sessions_to_process, 1):
        print(f"\rProcessing {i}/{len(sessions_to_process)}: {s_dir.name[:40]:<40}...", end="", flush=True)
        # Generate markdown report
        report = generate_report(s_dir, b_dir)
        report_path = s_dir / "report.md"
        report_path.write_text(report, encoding="utf-8")
        
        # Extract metrics for terminal
        stats_list.append(extract_session_stats(s_dir, b_dir))

    print("\r" + " " * 80 + "\r", end="") # Clear progress line
    print_comparative_table(stats_list)
    print(f"{Colors.OK}✓ Markdown reports (report.md) have been successfully written to each session directory.{Colors.RESET}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
