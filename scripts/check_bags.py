#!/usr/bin/env python3
"""
check_bags.py — Comprehensive Health Check for MTT ROS2 Bags.

Reads metadata.yaml directly (zero ROS runtime required).
Calculates exact message frequencies (Hz), checks hardware vs software signals,
trailer articulation topics, and parses session_info.yaml metadata.

Usage:
  python3 scripts/check_bags.py
  python3 scripts/check_bags.py --data-dir /media/mohamed/SSD-Mathis/bag_mohamed/
  python3 scripts/check_bags.py --verbose
"""

import sys, argparse
from pathlib import Path

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False


def infer_workspace_root(script_path: Path) -> Path:
    for candidate in [script_path.parent, *script_path.parents]:
        if (candidate / "src").exists() and (candidate / "demos").exists():
            return candidate
    return script_path.parent


DATA_DIR = infer_workspace_root(Path(__file__).resolve()) / "data"

# ANSI Colors
BOLD   = "\033[1m"
RED    = "\033[91m"
YELLOW = "\033[93m"
GREEN  = "\033[92m"
CYAN   = "\033[96m"
DIM    = "\033[2m"
ORANGE = "\033[38;5;208m"
RESET  = "\033[0m"

# ── Topics & Expected Frequencies (Hz) ──
# Format: { group_label: [(topic, is_primary, display_name, min_expected_hz)] }
WATCH = {
    "LiDAR (HW)": [
        ("/hesai_lidar/points",          True,  "hesai/points",          5.0),
        ("/rsairy_ns/points",            True,  "rsairy/points",         5.0),
        ("/merged_points_filtered",      False, "merged_points",         5.0),
    ],
    "IMU (HW)": [
        ("/mti100/data",                 True,  "mti100/data",          50.0),
        ("/mti10/data",                  True,  "mti10/data",           50.0),
        ("/zed/zed_node/imu/data",       False, "zed/imu/data",        100.0),
    ],
    "CAN/Odom (HW)": [
        ("/mtt_tachometer",              True,  "mtt_tachometer",       20.0),
        ("/mtt_status",                  True,  "mtt_status (HW angle)", 20.0),
        ("/mtt_odometry",                False, "mtt_odometry",         20.0),
        ("/from_can_bus",                False, "from_can_bus",         10.0),
    ],
    "BMS (HW)": [
        ("/mtt_battery/status",          True,  "mtt_battery/status",   20.0),
    ],
    "Trailer (HW/SW)": [
        ("/trailer/angle",               False, "trailer/angle (SW)",    5.0),
        ("/trailer/articulation_angle",  False, "trailer/art_angle",     5.0),
        ("/trailer/odom",                False, "trailer/odom",          5.0),
        ("/trailer/pose",                False, "trailer/pose",          5.0),
    ],
    "ZED Cam (HW/VSLAM)": [
        ("/zed/zed_node/rgb/color/rect/image/compressed",         True,  "rgb/compressed",        1.0),
        ("/zed/zed_node/depth/depth_registered/compressedDepth",  True,  "depth/compressedDepth", 1.0),
        ("/zed/zed_node/odom",                                    False, "zed/odom (VSLAM)",       5.0),
        ("/zed/zed_node/pose",                                    False, "zed/pose (VSLAM)",       5.0),
    ],
    "OAK Cam (HW)": [
        ("/oak/rgb/image_rect",          True,  "oak/rgb/image_rect",    1.0),
        ("/oak/stereo/image_raw",        True,  "oak/stereo/image_raw",  1.0),
        ("/oak/points",                  False, "oak/points",            1.0),
    ],
    "GPS Single (HW)": [
        ("/gps/fix",                     True,  "gps/fix",               1.0),
        ("/gps/time_reference",          False, "gps/time_reference",    1.0),
        ("/gps/nmea_sentence",           False, "gps/nmea_sentence",     1.0),
    ],
    "GPS Dual RTK (HW)": [
        ("/gps_left/fix",                True,  "gps_left/fix",          1.0),
        ("/gps_right/fix",               True,  "gps_right/fix",         1.0),
        ("/gps/heading",                 True,  "gps/heading",           1.0),
    ],
    "Control (SW)": [
        ("/cmd_vel",                     True,  "cmd_vel",               5.0),
        ("/teleop_estop",                False, "teleop_estop",          1.0),
        ("/teleop_deadman",              False, "teleop_deadman",        1.0),
    ],
    "ICP (SW Ref)": [
        ("/mapping/icp_odom",            True,  "mapping/icp_odom",      1.0),
    ],
}

# ── Root Cause Catalog ──
KNOWN_CAUSES = {
    "/zed/zed_node/rgb/color/rect/image/compressed":
        "QoS mismatch: ZED SDK forces BEST_EFFORT, recorder expects RELIABLE.\n"
        "     FIX: qos_override.yaml + --qos-profile-overrides-path in compose.yaml OK DONE",
    "/zed/zed_node/depth/depth_registered/compressedDepth":
        "QoS mismatch: ZED depth stream.\n"
        "     FIX: included in qos_override.yaml OK DONE",
    "/zed/zed_node/imu/data":
        "QoS mismatch: ZED IMU stream.\n"
        "     FIX: included in qos_override.yaml OK DONE",
    "/gps_left/fix":
        "GPS Driver attempts TCP port 5001 -> Reach RS listens on 9001/9696.\n"
        "     FIX: check host/port in gps_tcp.yaml",
    "/gps_right/fix":
        "GPS Driver attempts TCP port 5001 -> Reach RS listens on 9001/9696.\n"
        "     FIX: check host/port in gps_tcp.yaml",
    "/gps/fix":
        "Reach RS emits NMEA but no valid GGA is converted to NavSatFix.\n"
        "     FIX: verify GGA 5 Hz + RMC 1 Hz in ReachView3",
    "/mtt_tachometer":
        "Inductive encoder failed or disk gap incorrect.\n"
        "     FIX: hardware replacement / verify 1-2mm sensor gap",
    "/mapping/icp_odom":
        "Live ICP absent or broken. Rerunning offline ICP + factor graph is REQUIRED for Ground Truth.",
    "/oak/stereo/image_raw":
        "OAK USB disconnected or power failure.\n"
        "     FIX: reconnect OAK USB and check /dev/bus/usb",
    "/mtt_battery/status":
        "BMS driver uncompiled or CAN interface down.\n"
        "     FIX: colcon build --packages-select mtt_driver && ip link show can0",
    "/cmd_vel":
        "No autonomous or teleop commands published during recording.",
}


def parse_metadata(meta_path: Path):
    """Parse metadata.yaml -> (counts dict, duration_s, total_msgs)."""
    text = meta_path.read_text(encoding="utf-8", errors="ignore")

    if HAS_YAML:
        try:
            data = yaml.safe_load(text) or {}
            info = data.get("rosbag2_bagfile_information", data)
            counts = {
                e["topic_metadata"]["name"]: e["message_count"]
                for e in info.get("topics_with_message_count", [])
            }
            dur_s = info.get("duration", {}).get("nanoseconds", 0) / 1e9
            total = info.get("message_count", 0)
            return counts, dur_s, total
        except Exception:
            pass

    # Fallback parser
    counts = {}
    current_name = None
    dur_s, total = 0, 0
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("name:") and "/" in s:
            current_name = s.split("name:", 1)[1].strip().strip('"')
        elif s.startswith("message_count:") and current_name:
            try:
                counts[current_name] = int(s.split(":", 1)[1].strip())
            except ValueError:
                pass
            current_name = None
        elif s.startswith("nanoseconds:") and dur_s == 0:
            try:
                dur_s = int(s.split(":", 1)[1].strip()) / 1e9
            except ValueError:
                pass
    return counts, dur_s, sum(counts.values())


def parse_session_info(session_dir: Path) -> dict:
    """Parse session_info.yaml if present."""
    info_path = session_dir / "session_info.yaml"
    if not info_path.exists():
        return {}
    try:
        raw = info_path.read_bytes().decode("utf-8", errors="ignore")
        if HAS_YAML:
            return yaml.safe_load(raw) or {}
        # Fallback simple key-value
        res = {}
        for line in raw.splitlines():
            if ":" in line and not line.strip().startswith("#"):
                k, v = line.split(":", 1)
                res[k.strip()] = v.strip().strip('"').strip("'")
        return res
    except Exception:
        return {}


def group_status(entries, counts, dur_s):
    """
    Returns 'ok', 'low_hz', 'partial', 'dead', 'missing'.
    """
    primary_entries = [e for e in entries if e[1]]
    if not primary_entries:
        primary_entries = entries

    primary_counts = [(t, counts.get(t, -1), min_hz) for t, p, s, min_hz in primary_entries]

    if all(c == -1 for t, c, _ in primary_counts):
        return "missing"

    primary_alive = [(c, min_hz) for t, c, min_hz in primary_counts if c > 0]

    if not primary_alive:
        return "dead"

    if len(primary_alive) < len(primary_counts):
        return "partial"

    # Check frequencies
    if dur_s > 0:
        low_hz = any((c / dur_s) < (min_hz * 0.5) for c, min_hz in primary_alive)
        if low_hz:
            return "low_hz"

    return "ok"


STATUS_ICON = {
    "ok":      f"{GREEN}OK        {RESET}",
    "low_hz":  f"{ORANGE}~ LOW HZ {RESET}",
    "partial": f"{YELLOW}~ PARTIAL{RESET}",
    "dead":    f"{RED}FAIL DEAD   {RESET}",
    "missing": f"{DIM}— NOT REC{RESET}",
}


def analyze_session(session_dir: Path, verbose: bool = False) -> tuple:
    """Analyze a single session directory."""
    bag_dir   = session_dir / "bag"
    meta_path = bag_dir / "metadata.yaml"
    if not meta_path.exists():
        meta_path = session_dir / "metadata.yaml"

    name = session_dir.name
    sinfo = parse_session_info(session_dir)

    exp_name = sinfo.get("experiment_name", "N/A")
    s_type   = sinfo.get("session_type", "N/A")
    terrain  = sinfo.get("terrain", "N/A")
    trailer  = sinfo.get("trailer_attached", None)
    operator = sinfo.get("operator", "N/A")

    if not meta_path.exists():
        print(f"\n  {BOLD}{name}{RESET}")
        print(f"    {YELLOW}WARN  No bag/ or metadata.yaml found!{RESET}")
        return set(), name, "missing", {}, 0

    counts, dur_s, total = parse_metadata(meta_path)
    total_k = total / 1000.0

    print(f"\n  {BOLD}{CYAN}{name}{RESET}")

    # Metadata banner
    meta_str = f"Exp: {exp_name} | Type: {s_type} | Terrain: {terrain} | Trailer: {trailer}"
    print(f"    {DIM}{meta_str}{RESET}")
    print(f"    {DIM}Duration: {dur_s:.1f}s ({dur_s/60.0:.1f} min) | Total Msgs: {total_k:.1f}k | Op: {operator}{RESET}")

    broken_primaries = set()
    group_statuses = {}

    for group, entries in WATCH.items():
        status = group_status(entries, counts, dur_s)
        group_statuses[group] = status

        if verbose or status in ("dead", "partial", "low_hz"):
            icon = STATUS_ICON[status]
            print(f"      {BOLD}{group:18s}{RESET} {icon}")

            for topic, is_primary, short_name, min_hz in entries:
                c = counts.get(topic, -1)
                marker = "[P]" if is_primary else "   "

                if c == -1:
                    if verbose:
                        print(f"        {DIM}{marker} {short_name:<30s} — (not recorded){RESET}")
                elif c == 0:
                    col = RED if is_primary else YELLOW
                    print(f"        {col}{marker} {short_name:<30s} 0 msgs (DEAD){RESET}")
                    if is_primary:
                        broken_primaries.add(topic)
                else:
                    hz = (c / dur_s) if dur_s > 0 else 0.0
                    if hz < (min_hz * 0.5):
                        col = ORANGE
                        hz_str = f"{hz:6.1f} Hz (LOW, expected >={min_hz:.1f}Hz)"
                    else:
                        col = GREEN
                        hz_str = f"{hz:6.1f} Hz"
                    print(f"        {col}{marker} {short_name:<30s} {c:8,d} msgs ({hz_str}){RESET}")

    return broken_primaries, name, group_statuses, counts, dur_s


def print_summary_table(session_results: list):
    """Print overall summary table."""
    groups = ["LiDAR", "IMU", "CAN/Odom", "BMS", "Trailer", "ZED", "GPS", "Control", "ICP"]
    group_keys = ["LiDAR (HW)", "IMU (HW)", "CAN/Odom (HW)", "BMS (HW)", "Trailer (HW/SW)", "ZED Cam (HW/VSLAM)", "GPS Single (HW)", "Control (SW)", "ICP (SW Ref)"]

    print(f"\n{BOLD}{CYAN}══ Overall Health Check Summary Table ══{RESET}\n")

    header = f"  {'Session Directory':48s} {'Dur(m)':>6s} {'Trail':>5s}" + "".join(f" {g[:6]:>6s}" for g in groups)
    print(f"{BOLD}{header}{RESET}")
    print(f"  {'-'*48} {'-'*6} {'-'*5}" + " ".join(["------"] * len(groups)))

    icons = {
        "ok":      f"{GREEN}  OK   {RESET}",
        "low_hz":  f"{ORANGE}  ~   {RESET}",
        "partial": f"{YELLOW}  ~   {RESET}",
        "dead":    f"{RED}  FAIL   {RESET}",
        "missing": f"{DIM}  —   {RESET}"
    }

    for name, dur_s, sinfo, statuses in session_results:
        dur_min = f"{dur_s/60.0:.1f}m" if dur_s > 0 else "—"
        trailer = str(sinfo.get("trailer_attached", "?"))[:5]

        short_name = name if len(name) <= 48 else ("..." + name[-45:])
        row = f"  {short_name:48s} {dur_min:>6s} {trailer:>5s}"

        for key in group_keys:
            st = statuses.get(key, "missing")
            row += icons.get(st, "  ?   ")

        print(row)


def main():
    parser = argparse.ArgumentParser(description="Check ROS2 Bag Health & Topic Frequencies")
    parser.add_argument("--data-dir", type=Path, default=DATA_DIR, help="Path to bags directory")
    parser.add_argument("-v", "--verbose", action="store_true", help="Print all topics regardless of status")
    args = parser.parse_args()

    data_dir = args.data_dir
    if not data_dir.exists():
        print(f"{RED}Data directory not found: {data_dir}{RESET}")
        sys.exit(1)

    print(f"\n{BOLD}{CYAN}══════════════════════════════════════════════════════════════════════{RESET}")
    print(f"{BOLD}{CYAN}   MTT ROS2 Bag Detailed Health & Topic Frequency Checker{RESET}")
    print(f"{BOLD}{CYAN}   Target Directory: {data_dir}{RESET}")
    print(f"{BOLD}{CYAN}══════════════════════════════════════════════════════════════════════{RESET}")
    print(f"  {DIM}[P] = Primary required topic | Hz = Calculated message frequency{RESET}")

    # Discover all bag directories (folders containing bag/ or metadata.yaml or .mcap)
    candidates = [d for d in data_dir.iterdir() if d.is_dir() and not d.name.startswith(".")]
    sessions = []
    for d in candidates:
        if (d / "bag").exists() or (d / "metadata.yaml").exists() or any(d.glob("*.mcap")) or (d / "bag" / "metadata.yaml").exists():
            sessions.append(d)

    sessions = sorted(sessions, key=lambda x: x.name)

    if not sessions:
        print(f"{YELLOW}No bag directories found in {data_dir}{RESET}")
        sys.exit(0)

    all_broken = set()
    summary_results = []

    for s in sessions:
        broken, name, group_statuses, counts, dur_s = analyze_session(s, verbose=args.verbose)
        all_broken.update(broken)
        sinfo = parse_session_info(s)
        summary_results.append((name, dur_s, sinfo, group_statuses))

    print_summary_table(summary_results)

    if all_broken:
        print(f"\n{BOLD}{RED}══ Root Cause Analysis for Missing Primary Topics ══{RESET}\n")
        for topic in sorted(all_broken):
            cause = KNOWN_CAUSES.get(topic, "Unknown cause — verify node execution and QoS configuration.")
            print(f"  {RED}● {topic}{RESET}")
            print(f"    {cause}\n")


if __name__ == "__main__":
    main()
