#!/usr/bin/env python3
"""
health_check.py — MTT sensor health monitoring.

Phase 0 — Filesystem pre-check (device nodes, ZED calibration, no ROS needed).
Phase 1 — Network pre-check (ping + TCP port tests, no ROS needed).
Phase 2 — Smart sensor wait: polls the topic graph every second until all
           required primary sensors appear, or a 40 s startup timeout expires.
Phase 3 — Measures actual publication frequencies over DURATION seconds.
Phase 4 — Reports results with per-sensor diagnosis and actionable fixes.

Exit codes:
  0 — all required sensors OK
  1 — one or more required sensors failed / missing
  2 — warnings only (optional sensors missing or slow)

Environment variables:
  HEALTH_CHECK_DURATION     Measurement window in seconds (default 15)
  HEALTH_CHECK_WAIT_TIMEOUT Max seconds to wait for sensors to appear (default 40)
  GPS_MODE                  serial | tcp (default serial)
  GPS_ANTENNAS              single | dual (default single)
  REACH_ROVER_IP            Reach RS rover IP (default 192.168.2.59)
  REACH_ROVER_TCP_PORT      Reach RS rover TCP port (default 9001)
  HESAI_IP                  Hesai LiDAR IP (default 192.168.2.201)
"""

import glob
import importlib
import os
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import rclpy.executors
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import yaml

# ── ANSI colors ───────────────────────────────────────────────────────────────
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
RESET  = "\033[0m"

# ── Config from env ───────────────────────────────────────────────────────────
DURATION        = float(os.environ.get("HEALTH_CHECK_DURATION",    "15"))
WAIT_TIMEOUT    = float(os.environ.get("HEALTH_CHECK_WAIT_TIMEOUT","40"))
GPS_MODE        = os.environ.get("GPS_MODE", "serial")
GPS_ANTENNAS    = os.environ.get("GPS_ANTENNAS", "single")
OAK_MODE        = os.environ.get("OAK_MODE", "stable")
ROVER_IP        = os.environ.get("REACH_ROVER_IP",       "192.168.2.59")
ROVER_PORT      = int(os.environ.get("REACH_ROVER_TCP_PORT", "9001"))
HESAI_IP        = os.environ.get("HESAI_IP",          "192.168.2.201")
OAK_RATE_HZ     = 10.0 if OAK_MODE.strip().lower() == "max" else 5.0

GPS_FIX_LABELS = {-1: "NO FIX", 0: "GPS SPP", 1: "SBAS", 2: "RTK", 3: "RTK Float", 4: "RTK Fixed"}


def load_driver_param(name: str, default):
    params_path = os.environ.get("DRIVER_PARAMS_FILE")
    if not params_path:
        return default
    try:
        with Path(params_path).open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except OSError:
        return default
    node = data.get("mtt_can_node") or {}
    params = node.get("ros__parameters", {})
    return params.get(name, default)


TACHOMETER_MODE = str(load_driver_param("tachometer_mode", "real"))


@dataclass
class TopicSpec:
    topic: str
    label: str
    expected_hz: float
    tol_pct: float = 25.0
    required: bool = True
    group: str = ""


# ── Topic list ────────────────────────────────────────────────────────────────
def build_topics() -> list[TopicSpec]:
    topics: list[TopicSpec] = [
    # ── Infrastructure ────────────────────────────────────────────────────────
    TopicSpec("/tf",           "TF",           50.0, 50.0,  group="infra"),
    TopicSpec("/tf_static",    "TF Static",     1.0, 300.0, group="infra"),
    TopicSpec("/joint_states", "Joint States", 50.0, 30.0,  group="infra"),

    # ── MTT CAN driver ────────────────────────────────────────────────────────
    TopicSpec("/mtt_odometry",           "MTT Odometry",   50.0, 30.0, group="can"),
    TopicSpec("/mtt_tachometer",         "MTT Tachometer", 50.0, 30.0, group="can"),
    TopicSpec("/mtt_articulation_angle", "Articul. Angle", 50.0, 30.0, group="can"),
    TopicSpec("/mtt_status",             "MTT Status",     10.0, 40.0, group="can"),
    TopicSpec("/mtt_health",             "MTT Health",     10.0, 40.0, required=False, group="can"),
    # Raw CAN bus — socketcan_bridge must be running (publishes every frame verbatim)
    TopicSpec("/from_can_bus", "Raw CAN bus", 20.0, 80.0, required=False, group="can"),
    TopicSpec("/joy",                    "Joystick",       20.0, 80.0, required=False, group="operator"),
    TopicSpec("/teleop_deadman",         "Teleop Deadman", 20.0, 80.0, required=False, group="operator"),
    TopicSpec("/teleop_estop",           "Teleop E-Stop",  20.0, 80.0, required=False, group="operator"),

    # ── BMS / Battery ─────────────────────────────────────────────────────────
    TopicSpec("/mtt_battery/status", "BMS Status", 10.0, 50.0, required=False, group="bms"),

    # ── IMU — XSens MTi-100 (primary, required) ───────────────────────────────
    # data_raw: 100 Hz raw measurements; data: ~100 Hz Kalman-filtered orientation.
    # Both recorded. data_raw used for odometry fusion; data for heading reference.
    TopicSpec("/mti100/data",           "MTi-100 data",     100.0, 30.0, group="imu"),
    TopicSpec("/mti100/data_raw",       "MTi-100 data_raw", 100.0, 30.0, group="imu"),
    TopicSpec("/mti100/time_reference", "MTi-100 TimeRef",   10.0, 50.0, group="imu"),

    # ── IMU — XSens MTi-10 (secondary, optional) ──────────────────────────────
    TopicSpec("/mti10/data",     "MTi-10 data",     100.0, 30.0, required=False, group="imu"),
    TopicSpec("/mti10/data_raw", "MTi-10 data_raw", 100.0, 30.0, required=False, group="imu"),

    # ── LiDAR — Hesai XT-32 (required) ───────────────────────────────────────
    TopicSpec("/hesai_lidar/points",        "Hesai PointCloud",  10.0, 20.0, group="lidar"),
    TopicSpec("/hesai_lidar/lidar_packets_loss", "Hesai Pkt Loss", 10.0, 80.0, required=False, group="lidar"),

    # ── LiDAR — RoboSense Bpearl (optional — rear/trailer) ───────────────────
    TopicSpec("/rsairy_ns/points", "RS Bpearl pts", 10.0, 20.0, required=False, group="lidar"),

    # ── GPS — Emlid Reach RS single rover (rear-right mast, default) ─────────
    # RTK corrections flow internally between rover and base via LoRa/TCP.
    # GGA @ 5 Hz gives position; RMC @ 1 Hz gives date for GPS-UTC timestamps.
    TopicSpec("/gps/fix",            "GPS Rover fix",    1.0, 80.0, group="gps"),
    TopicSpec("/gps/nmea_sentence",  "GPS Rover NMEA",   5.0, 80.0, required=False, group="gps"),
    TopicSpec("/gps/time_reference", "GPS Rover TimeRef",1.0, 80.0, required=False, group="gps"),

    # ── GPS — Dual-antenna legacy (only active with gps_antennas:=dual) ──────
    TopicSpec("/gps_left/fix",   "GPS Left fix",  1.0, 80.0, required=False, group="gps"),
    TopicSpec("/gps_right/fix",  "GPS Right fix", 1.0, 80.0, required=False, group="gps"),
    TopicSpec("/gps/heading",    "GPS Heading",   1.0, 80.0, required=False, group="gps"),

    # ── Camera — ZED 2i stereo (optional but important) ───────────────────────
    # Rates: RGB/Depth ≈ 5 Hz compressed, IMU ≈ 100 Hz
    TopicSpec("/zed/zed_node/rgb/color/rect/image/compressed",
              "ZED RGB",   10.0, 40.0, required=False, group="camera"),
    TopicSpec("/zed/zed_node/depth/depth_registered/compressedDepth",
              "ZED Depth", 10.0, 40.0, required=False, group="camera"),
    TopicSpec("/zed/zed_node/point_cloud/cloud_registered",
              "ZED PointCloud", 10.0, 60.0, required=False, group="camera"),
    TopicSpec("/zed/zed_node/imu/data",
              "ZED IMU",  100.0, 30.0, required=False, group="camera"),

    # ── Camera — OAK-D (optional — rear/trailer) ──────────────────────────────
    TopicSpec("/oak/rgb/image_rect",
              "OAK RGB", OAK_RATE_HZ, 40.0, required=False, group="camera"),
    TopicSpec("/oak/stereo/image_raw",
              "OAK Depth", OAK_RATE_HZ, 60.0, required=False, group="camera"),
    TopicSpec("/oak/points",
              "OAK PointCloud", OAK_RATE_HZ, 70.0, required=False, group="camera"),

    # ── ICP Mapping (optional — starts after mapping_delay) ───────────────────
    TopicSpec("/merged_points_filtered",        "Merged Cloud",      10.0, 50.0, required=False, group="mapping"),
    TopicSpec("/mapping/icp_odom",               "ICP Odom",          10.0, 30.0, required=False, group="mapping"),
    TopicSpec("/mapping/scan_after_deskew",      "ICP scan deskew",   10.0, 50.0, required=False, group="mapping"),
    TopicSpec("/mapping/scan_after_input_filters","ICP scan filtered", 10.0, 50.0, required=False, group="mapping"),
    TopicSpec("/trailer/angle",                  "Trailer Angle",      5.0, 80.0, required=False, group="mapping"),

    # ── Teach / Repeat supervision ───────────────────────────────────────────
    TopicSpec("/mtt_repeat/state",               "Repeat State",       0.5, 500.0, required=False, group="repeat"),
    TopicSpec("/mtt_repeat/ready",               "Repeat Ready",       0.5, 500.0, required=False, group="repeat"),

    # ── Operator annotations ──────────────────────────────────────────────────
    TopicSpec("/session/events", "Session Events", 0.1, 1000.0, required=False, group="infra"),
    ]

    if TACHOMETER_MODE == "cmd_sim":
        topics.append(
            TopicSpec(
                "/mtt_monitor/cmd_fallback_odom",
                "Cmd Fallback Odom",
                10.0,
                50.0,
                required=False,
                group="fusion",
            )
        )
    else:
        topics.append(
            TopicSpec(
                "/imu_and_wheel_odom",
                "IMU+Wheel Odom",
                50.0,
                30.0,
                required=False,
                group="fusion",
            )
        )
    return topics


TOPICS: list[TopicSpec] = build_topics()

# Required groups — if ALL topics in a group are missing, add group-level hint
GROUP_HINTS = {
    "gps": (
        f"GPS mode='{GPS_MODE}' antennas='{GPS_ANTENNAS}'. "
        "Serial: check /dev/reach_rover exists inside container "
        "(run: sudo bash scripts/setup_udev_reach_rs.sh on the robot). "
        f"TCP: rover at {ROVER_IP}:{ROVER_PORT} — check ReachView3 TCP output enabled. "
        "If /gps/nmea_sentence is alive but /gps/fix stays at 0, the Reach is connected "
        "but GGA is missing or rejected. Verify GGA 5 Hz + RMC 1 Hz on the rover USB output."
    ),
    "camera": (
        "ZED 0 images: check /usr/local/zed/settings/SN*.conf exists (ZED calibration). "
        "If missing: connect robot to internet and run ZED_Diagnostic once, "
        "or copy SN<serial>.conf from another machine. "
        "OAK: check USB cable and strain relief after vibrations, then verify RGBD + /oak/points "
        "are enabled in oak.launch.py."
    ),
    "lidar": (
        f"Hesai: check {HESAI_IP} on the network (ping). "
        "RS Bpearl: check 192.168.1.102 / USB config."
    ),
    "imu": "XSens MTi-100: check /dev/serial/by-id/usb-Xsens_MTi-100* USB device.",
    "can": (
        "CAN bus: check 'ip link show can0' — must be UP at 250000 baud. "
        "/from_can_bus missing → socketcan_bridge not running or can0 not yet UP "
        "(bridge waits up to 60s for state UP)."
    ),
    "operator": (
        "Joystick override: check /dev/input/js0 inside the container, joy_linux running, "
        "and that the controller actually sends /joy. teleop_deadman should toggle with the deadman button."
    ),
    "bms": "BMS: mtt_battery/status requires mtt_driver with BMS decoder compiled in.",
    "mapping": (
        "ICP odom=0: check 'robot_frame: base_footprint' in _icp_mapper.yaml "
        "(base_link causes TF loop → odom never published). "
        "Mapper starts after mapping_delay_seconds (default 5s)."
    ),
}


# ── Phase 1: Network pre-checks (no ROS) ─────────────────────────────────────

def _ping(ip: str, timeout_s: float = 1.5) -> bool:
    """Single ICMP ping."""
    try:
        ret = subprocess.run(
            ["ping", "-c", "1", "-W", str(int(timeout_s)), ip],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=timeout_s + 1
        )
        return ret.returncode == 0
    except Exception:
        return False


def _tcp_connect(ip: str, port: int, timeout_s: float = 2.0) -> Tuple[bool, str]:
    """Try TCP connect and optionally read first bytes to verify NMEA stream."""
    try:
        with socket.create_connection((ip, port), timeout=timeout_s) as s:
            s.settimeout(2.0)
            try:
                data = s.recv(128)
                if data:
                    snippet = data.decode("ascii", errors="replace").strip()[:40]
                    if "$" in snippet:
                        return True, f"NMEA: {snippet}"
                    return True, f"data: {snippet}"
            except socket.timeout:
                return True, "connected (no data in 2s)"
    except ConnectionRefusedError:
        return False, "connection refused — wrong port?"
    except OSError as e:
        return False, str(e)
    except Exception as e:
        return False, str(e)


def run_filesystem_prechecks() -> bool:
    """
    Phase 0: Filesystem / device checks before starting ROS.
    Detects common misconfigurations that cause silent 0-message topics.
    Returns True if no blocking issues found.
    """
    print(f"\n{BOLD}{CYAN}── Phase 0: Filesystem pre-check ───────────────────{RESET}")
    any_error = False

    # /dev/reach_rover — required for single-antenna GPS
    rover_dev = "/dev/reach_rover"
    if os.path.exists(rover_dev):
        print(f"  {GREEN}✓{RESET}  {rover_dev}  exists (GPS serial device OK)")
    else:
        print(f"  {RED}✗{RESET}  {rover_dev}  MISSING")
        print(f"      {YELLOW}→ GPS driver will retry silently, 0 messages in bag.{RESET}")
        print(f"      {YELLOW}  Fix: sudo bash scripts/setup_udev_reach_rs.sh  (on robot host){RESET}")
        any_error = True

    # ZED calibration file — /usr/local/zed/settings/SN*.conf
    zed_settings_dir = "/usr/local/zed/settings"
    zed_cal_files = glob.glob(os.path.join(zed_settings_dir, "SN*.conf"))
    if zed_cal_files:
        for f in zed_cal_files:
            print(f"  {GREEN}✓{RESET}  ZED calibration  {DIM}{os.path.basename(f)}{RESET}")
    else:
        print(f"  {YELLOW}⚠{RESET}  ZED calibration  MISSING  ({zed_settings_dir}/SN*.conf)")
        print(f"      {YELLOW}→ ZED SDK will attempt download (needs internet).{RESET}")
        print(f"      {YELLOW}  Without calibration: IMU starts briefly then SDK crashes → 0 images.{RESET}")
        print(f"      {YELLOW}  Fix: connect robot to internet once, or copy SN<serial>.conf manually.{RESET}")
        # Not a hard error — SDK might download successfully

    # ZED resources — pos_tracking models (optional but good to know)
    zed_resources = "/usr/local/zed/resources"
    if os.path.isdir(zed_resources) and os.listdir(zed_resources):
        print(f"  {GREEN}✓{RESET}  ZED resources    {DIM}{zed_resources}{RESET}  present")
    else:
        print(f"  {DIM}  ZED resources    {zed_resources}  empty/missing (pos_tracking disabled — OK){RESET}")

    print()
    return not any_error


def run_network_prechecks() -> bool:
    """
    Phase 1: Network checks before starting ROS.
    Returns True if at least critical infrastructure is reachable.
    """
    print(f"\n{BOLD}{CYAN}── Phase 1: Network pre-check ──────────────────────{RESET}")
    any_net_warn = False

    # Hesai LiDAR
    ok = _ping(HESAI_IP)
    s = f"{GREEN}✓{RESET}" if ok else f"{YELLOW}⚠{RESET}"
    detail = "reachable" if ok else f"no response — Hesai at {HESAI_IP} may be off or wrong IP"
    print(f"  {s}  Hesai XT-32   {DIM}{HESAI_IP}{RESET}  {detail}")
    if not ok:
        any_net_warn = True

    # GPS connectivity
    if GPS_MODE == "tcp":
        print(f"  {DIM}GPS mode=tcp — testing TCP connection to rover{RESET}")
        ping_ok = _ping(ROVER_IP)
        if ping_ok:
            tcp_ok, msg = _tcp_connect(ROVER_IP, ROVER_PORT)
            if tcp_ok:
                print(f"  {GREEN}✓{RESET}  GPS Rover     {DIM}{ROVER_IP}:{ROVER_PORT}{RESET}  {msg}")
            else:
                print(f"  {RED}✗{RESET}  GPS Rover     {DIM}{ROVER_IP}:{ROVER_PORT}{RESET}  {msg}")
                print(f"      {YELLOW}→ Reach RS reachable but port {ROVER_PORT} refused. "
                      f"Enable TCP output in ReachView3.{RESET}")
                any_net_warn = True
        else:
            print(f"  {RED}✗{RESET}  GPS Rover     {DIM}{ROVER_IP}:{ROVER_PORT}{RESET}  "
                  f"host unreachable — Reach RS not on network?")
            any_net_warn = True
    else:
        print(f"  {DIM}GPS mode=serial (/dev/reach_rover) — TCP not checked{RESET}")
        # Optionally ping rover WiFi IP to confirm it's powered on
        ping_ok = _ping(ROVER_IP)
        s = f"{GREEN}✓{RESET}" if ping_ok else f"{DIM}–{RESET}"
        detail = "reachable (WiFi)" if ping_ok else "not on WiFi (normal if LoRa only)"
        print(f"  {s}  GPS Rover WiFi  {DIM}{ROVER_IP}{RESET}  {detail}")

    print()
    return not any_net_warn


def _import_msg_class(type_string: str):
    try:
        parts = type_string.split("/")
        if len(parts) != 3 or parts[1] != "msg":
            return None
        package, _, classname = parts
        module = importlib.import_module(f"{package}.msg")
        return getattr(module, classname)
    except Exception:
        return None


# ── Phase 2+3: ROS topic monitoring ──────────────────────────────────────────

def run_ros_healthcheck(duration: float, wait_timeout: float) -> int:
    rclpy.init()
    node = rclpy.create_node("health_check_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="spin", daemon=True)
    spin_thread.start()

    counts: Dict[str, int] = {s.topic: 0 for s in TOPICS}
    # Track worst GPS fix quality seen during the window.
    # Single-antenna mode: /gps/fix; dual legacy: /gps_left/fix + /gps_right/fix.
    gps_fix_worst: Dict[str, int] = {
        "/gps/fix":       +99,   # single rover (primary)
        "/gps_left/fix":  +99,   # dual legacy
        "/gps_right/fix": +99,   # dual legacy
    }
    lock = threading.Lock()
    subs = []

    best_effort_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST, depth=5)
    reliable_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST, depth=5)

    LATCHED = {"/tf_static", "/robot_description", "/session/events", "/mtt_repeat/state", "/mtt_repeat/ready"}

    def make_counter(topic: str):
        def cb(msg):
            with lock:
                counts[topic] += 1
                if topic in gps_fix_worst and hasattr(msg, "status"):
                    gps_fix_worst[topic] = min(gps_fix_worst[topic], int(msg.status.status))
        return cb

    required_primary = {s.topic for s in TOPICS if s.required and s.group in ("can", "imu", "lidar", "gps")}

    # ── Phase 2: Smart wait for primary sensors ───────────────────────────────
    print(f"{BOLD}{CYAN}── Phase 2: Waiting for sensors (max {wait_timeout:.0f}s) ──{RESET}")
    wait_start = time.monotonic()
    subscribed_topics: set = set()

    while True:
        elapsed = time.monotonic() - wait_start
        topic_type_map: Dict[str, str] = {
            name: types[0]
            for name, types in node.get_topic_names_and_types()
            if types
        }

        # Subscribe to newly discovered topics
        for spec in TOPICS:
            if spec.topic in subscribed_topics:
                continue
            if spec.topic not in topic_type_map:
                continue
            msg_class = _import_msg_class(topic_type_map[spec.topic])
            if msg_class is None:
                continue
            try:
                qos = reliable_qos if spec.topic in LATCHED else best_effort_qos
                sub = node.create_subscription(msg_class, spec.topic, make_counter(spec.topic), qos)
                subs.append(sub)
                subscribed_topics.add(spec.topic)
            except Exception:
                pass

        found_required = required_primary & set(topic_type_map.keys())
        missing_required = required_primary - found_required
        pct = int(100 * len(found_required) / max(len(required_primary), 1))

        bar_done = "█" * (pct // 5)
        bar_left = "░" * (20 - pct // 5)
        print(f"\r  [{bar_done}{bar_left}] {pct:3d}%  "
              f"{len(found_required)}/{len(required_primary)} primary sensors  "
              f"t={elapsed:.0f}s/{wait_timeout:.0f}s   ",
              end="", flush=True)

        if not missing_required:
            print(f"\r  {GREEN}✓{RESET} All primary sensors found ({elapsed:.0f}s)."
                  f"{' ' * 30}")
            break

        if elapsed >= wait_timeout:
            print(f"\r  {YELLOW}⚠{RESET} Timeout after {wait_timeout:.0f}s. Missing: "
                  f"{', '.join(t.split('/')[-1] for t in sorted(missing_required))}"
                  f"{' ' * 20}")
            break

        time.sleep(1.0)

    print()

    # Final subscription pass for any remaining topics
    topic_type_map = {
        name: types[0]
        for name, types in node.get_topic_names_and_types()
        if types
    }
    for spec in TOPICS:
        if spec.topic in subscribed_topics:
            continue
        if spec.topic not in topic_type_map:
            continue
        msg_class = _import_msg_class(topic_type_map[spec.topic])
        if msg_class is None:
            continue
        try:
            qos = reliable_qos if spec.topic in LATCHED else best_effort_qos
            node.create_subscription(msg_class, spec.topic, make_counter(spec.topic), qos)
        except Exception:
            pass

    # Reset counts — start fresh for measurement window
    with lock:
        for k in counts:
            counts[k] = 0
        for k in gps_fix_worst:
            gps_fix_worst[k] = +99

    # ── Phase 3: Measure ─────────────────────────────────────────────────────
    print(f"{BOLD}{CYAN}── Phase 3: Measuring for {duration:.0f}s ─────────────────{RESET}")
    t_measure_start = time.monotonic()
    for i in range(int(duration)):
        bar_done = "█" * (i + 1)
        bar_left = "░" * (int(duration) - i - 1)
        print(f"\r  [{bar_done}{bar_left}] {i+1}/{int(duration)}s", end="", flush=True)
        time.sleep(1.0)
    actual_duration = time.monotonic() - t_measure_start
    print(f"\r  Measurement complete.{' ' * 30}\n")

    discovered = {t for t, _ in node.get_topic_names_and_types()}
    service_names = {name for name, _ in node.get_service_names_and_types()}
    follow_path_action_ready = (
        "/follow_path/_action/send_goal" in service_names
        and "/follow_path/_action/get_result" in service_names
        and "/follow_path/_action/cancel_goal" in service_names
    )
    # Shut down executor first so the spin thread exits cleanly, then call
    # rclpy.shutdown() and join the thread before returning. Without this,
    # the daemon thread is still alive at interpreter shutdown and races on
    # stderr, producing "Fatal Python error: _enter_buffered_busy" + exit 134.
    executor.shutdown(timeout_sec=2.0)
    rclpy.shutdown()
    spin_thread.join(timeout=3.0)

    # ── Phase 4: Report ───────────────────────────────────────────────────────
    COL = [44, 12, 10, 20, 5]
    sep = "─" * (sum(COL) + 4)
    header = (f"{'Topic':<{COL[0]}} {'Expected':>{COL[1]}} {'Measured':>{COL[2]}} "
              f"{'Status':>{COL[3]}} {'Req?':>{COL[4]}}")
    print(f"{BOLD}{header}{RESET}")
    print(sep)

    has_error = False
    has_warning = False
    missing_by_group: Dict[str, List[str]] = {}

    current_group = ""
    for spec in TOPICS:
        if spec.group != current_group:
            current_group = spec.group
            print(f"{DIM}  ── {current_group.upper()} ──{RESET}")

        n_msgs = counts[spec.topic]
        actual_hz = n_msgs / actual_duration
        exists = spec.topic in discovered
        exp_low = spec.expected_hz * (1.0 - spec.tol_pct / 100.0)

        if not exists:
            status_str = "NO TOPIC"
            color = RED if spec.required else YELLOW
            if spec.required:
                has_error = True
            else:
                has_warning = True
            missing_by_group.setdefault(spec.group, []).append(spec.topic)
        elif actual_hz < exp_low and spec.expected_hz > 0.3:
            status_str = f"LOW {actual_hz:.1f} Hz"
            color = RED if spec.required else YELLOW
            if spec.required:
                has_error = True
            else:
                has_warning = True
        else:
            status_str = f"OK  {actual_hz:.1f} Hz"
            color = GREEN

        req_str = "✓" if spec.required else "opt"
        exp_str = f"{spec.expected_hz:.0f} Hz"
        tdisplay = spec.topic if len(spec.topic) <= COL[0] else spec.topic[:COL[0]-2] + ".."

        print(
            f"{color}{tdisplay:<{COL[0]}} {exp_str:>{COL[1]}} "
            f"{actual_hz:>{COL[2]-3}.1f} Hz {status_str:>{COL[3]}} {req_str:>{COL[4]}}{RESET}"
        )

    print(sep)

    # ── GPS fix quality ───────────────────────────────────────────────────────
    print(f"\n{BOLD}GPS fix quality (worst seen during window):{RESET}")
    gps_quality_ok = True
    for gps_topic, worst_status in gps_fix_worst.items():
        # Skip legacy dual-antenna topics if they have no data and we're in single mode
        if worst_status == +99 and gps_topic != "/gps/fix":
            continue   # dual topics absent in single-antenna mode — expected
        label = {
            "/gps/fix":       "Rover (single)",
            "/gps_left/fix":  "Left  (dual)  ",
            "/gps_right/fix": "Right (dual)  ",
        }.get(gps_topic, gps_topic)
        if worst_status == +99:
            fix_label = "no data"
            color = RED
            note = "no messages — /dev/reach_rover missing or driver not connected"
            gps_quality_ok = False
        elif worst_status >= 4:
            fix_label = "RTK Fixed"
            color = GREEN
            note = "±1 cm — best"
        elif worst_status >= 3:
            fix_label = "RTK Float"
            color = YELLOW
            note = "~10–50 cm — wait for Fixed before recording"
            gps_quality_ok = False
        elif worst_status >= 2:
            fix_label = GPS_FIX_LABELS.get(worst_status, f"fix={worst_status}")
            color = GREEN
            note = "RTK — good for research"
        elif worst_status == 1:
            fix_label = "SBAS/DGPS"
            color = YELLOW
            note = "sub-meter — marginal, wait for RTK"
            gps_quality_ok = False
        elif worst_status == 0:
            fix_label = "GPS SPP"
            color = YELLOW
            note = "~10 m — not suitable for motion model ID"
            gps_quality_ok = False
        else:
            fix_label = "NO FIX"
            color = RED
            note = "no satellites — check sky view, LoRa corrections"
            gps_quality_ok = False
        print(f"  {color}{label} ({gps_topic}): {fix_label}  — {note}{RESET}")

    # ── Group-level hints for missing sensors ─────────────────────────────────
    if missing_by_group:
        print(f"\n{BOLD}Diagnosis:{RESET}")
        for grp, topics in missing_by_group.items():
            hint = GROUP_HINTS.get(grp, "")
            short_names = [t.rsplit("/", 1)[-1] for t in topics]
            print(f"  {YELLOW}● {grp.upper()}: {', '.join(short_names)}{RESET}")
            if hint:
                print(f"    {DIM}{hint}{RESET}")

    print(f"\n{BOLD}Repeat readiness:{RESET}")
    repeat_services = {
        "/mtt_repeat/teach_start",
        "/mtt_repeat/teach_stop",
        "/mtt_repeat/play_line",
        "/mtt_repeat/play_loop",
        "/mtt_repeat/cancel",
    }
    repeat_service_count = sum(1 for svc in repeat_services if svc in service_names)
    repeat_topics_present = {"/mtt_repeat/state", "/mtt_repeat/ready"} & discovered
    repeat_stack_running = repeat_service_count > 0 or bool(repeat_topics_present)
    if not repeat_stack_running:
        print(f"  {DIM}– WILN / repeat supervisor not running in this session.{RESET}")
    else:
        icp_ok = counts.get("/mapping/icp_odom", 0) > 0
        health_ok = counts.get("/mtt_health", 0) > 0
        repeat_ok = repeat_service_count == len(repeat_services) and follow_path_action_ready and icp_ok and health_ok
        color = GREEN if repeat_ok else YELLOW
        print(f"  {color}{'✓' if repeat_ok else '⚠'} services: {repeat_service_count}/{len(repeat_services)}  action(/follow_path): {'ok' if follow_path_action_ready else 'missing'}{RESET}")
        print(f"  {color}{'✓' if repeat_ok else '⚠'} topics: state={'yes' if '/mtt_repeat/state' in discovered else 'no'} ready={'yes' if '/mtt_repeat/ready' in discovered else 'no'}  icp={'yes' if icp_ok else 'no'}  mtt_health={'yes' if health_ok else 'no'}{RESET}")
        if not repeat_ok:
            has_warning = True

    # ── Final verdict ─────────────────────────────────────────────────────────
    print()
    if has_error:
        print(f"{RED}{BOLD}❌  HEALTH CHECK FAILED — required sensors missing or too slow.{RESET}")
        print("   Fix sensor issues before starting recording.\n")
        return 1
    elif has_warning or not gps_quality_ok:
        print(f"{YELLOW}{BOLD}⚠   HEALTH CHECK PASSED WITH WARNINGS.{RESET}")
        if not gps_quality_ok:
            print("   GPS fix quality below RTK — recording will proceed but GPS accuracy is poor.")
        print("   Recording can start, but check warnings above.\n")
        return 2
    else:
        print(f"{GREEN}{BOLD}✅  ALL SENSORS HEALTHY — ready to record.{RESET}\n")
        return 0


def main() -> int:
    print(f"\n{BOLD}{CYAN}══════════════════════════════════════════════════{RESET}")
    print(f"{BOLD}{CYAN}   MTT Sensor Health Check{RESET}")
    print(f"{BOLD}{CYAN}   duration={DURATION:.0f}s  wait={WAIT_TIMEOUT:.0f}s  "
          f"gps_mode={GPS_MODE}  gps_antennas={GPS_ANTENNAS}  tachometer_mode={TACHOMETER_MODE}{RESET}")
    print(f"{BOLD}{CYAN}══════════════════════════════════════════════════{RESET}")

    run_filesystem_prechecks()
    run_network_prechecks()

    return run_ros_healthcheck(DURATION, WAIT_TIMEOUT)


if __name__ == "__main__":
    sys.exit(main())
