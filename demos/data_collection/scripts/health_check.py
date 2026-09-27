#!/usr/bin/env python3
"""
health_check.py — MTT sensor health monitoring.

Phase 0 — Filesystem pre-check (device nodes, ZED calibration, no ROS needed).
Phase 1 — Network pre-check (ping + TCP port tests, no ROS needed).
Phase 2 — Smart sensor wait: polls the topic graph every second until all
           required primary sensors appear, or a 40 s startup timeout expires.
Phase 3 — Verifies topic activity over DURATION seconds.
Phase 4 — Reports results with per-sensor diagnosis and actionable fixes.

Exit codes:
  0 — all required sensors OK
  1 — one or more required sensors failed / missing
  2 — warnings only (optional sensors missing or slow)

Environment variables:
  HEALTH_CHECK_DURATION     Measurement window in seconds (default 15)
  HEALTH_CHECK_WAIT_TIMEOUT Max seconds to wait for sensors to appear (default 40)
  GPS_MODE                  serial | tcp (default serial)
  GPS_ANTENNAS              front | single | dual (default front)
  REACH_FRONT_DEV           Front Reach serial device (default /dev/reach_front)
  REACH_ROVER_DEV           Rover Reach serial device (default /dev/reach_rover)
  REACH_FRONT_IP            Front Reach IP for tcp mode (default 192.168.2.59)
  REACH_FRONT_TCP_PORT      Front Reach TCP port (default 9001)
  REACH_ROVER_IP            Reach RS rover IP (default 192.168.2.59)
  REACH_ROVER_TCP_PORT      Reach RS rover TCP port (default 9001)
  HESAI_IP                  Hesai LiDAR IP (default 192.168.2.201)
  RS_IP                     RoboSense/RS-Airy LiDAR IP (default 192.168.1.200)
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
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import Log as RosoutLog

# ── ANSI colors ──
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
RESET  = "\033[0m"

# ── Config from env ──
DURATION        = float(os.environ.get("HEALTH_CHECK_DURATION",    "15"))
WAIT_TIMEOUT    = float(os.environ.get("HEALTH_CHECK_WAIT_TIMEOUT","40"))
GPS_MODE        = os.environ.get("GPS_MODE", "serial")
GPS_ANTENNAS    = os.environ.get("GPS_ANTENNAS", "front")
OAK_MODE        = os.environ.get("OAK_MODE", "stable")
FRONT_DEV       = os.environ.get("REACH_FRONT_DEV", "/dev/reach_front")
ROVER_DEV       = os.environ.get("REACH_ROVER_DEV", "/dev/reach_rover")
FRONT_IP        = os.environ.get("REACH_FRONT_IP", "192.168.2.59")
FRONT_PORT      = int(os.environ.get("REACH_FRONT_TCP_PORT", "9001"))
ROVER_IP        = os.environ.get("REACH_ROVER_IP",       "192.168.2.59")
ROVER_PORT      = int(os.environ.get("REACH_ROVER_TCP_PORT", "9001"))
HESAI_IP        = os.environ.get("HESAI_IP",          "192.168.2.201")
RS_IP           = os.environ.get("RS_IP",             "192.168.1.200")
OAK_RATE_HZ     = 10.0 if OAK_MODE.strip().lower() == "max" else 5.0


def env_bool(name: str, default: bool = True) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


ENABLE_GPS = env_bool("ENABLE_GPS", True)
ENABLE_OAK = env_bool("ENABLE_OAK", False)
ENABLE_MTI10 = env_bool("ENABLE_MTI10", True)

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


# ── Topic list ──
def build_topics() -> list[TopicSpec]:
    topics: list[TopicSpec] = [
    # ── Infrastructure ──
    TopicSpec("/tf",           "TF",           50.0, 90.0,  group="infra"),
    TopicSpec("/tf_static",    "TF Static",     1.0, 300.0, group="infra"),
    TopicSpec("/runtime_joint_states", "Runtime Joint States", 50.0, 90.0,  group="infra"),

    # ── MTT CAN driver ──
    TopicSpec("/mtt_odometry",           "MTT Odometry",   50.0, 90.0, group="can"),
    TopicSpec("/mtt_tachometer",         "MTT Tachometer", 50.0, 90.0, group="can"),
    TopicSpec("/mtt_articulation_angle", "Articul. Angle", 50.0, 90.0, group="can"),
    TopicSpec("/mtt_status",             "MTT Status",     10.0, 40.0, group="can"),
    TopicSpec("/mtt_health",             "MTT Health",     10.0, 40.0, required=False, group="can"),
    # Raw CAN bus — socketcan_bridge must be running (publishes every frame verbatim)
    TopicSpec("/from_can_bus", "Raw CAN bus", 20.0, 80.0, required=False, group="can"),
    TopicSpec("/joy",                    "Joystick",       20.0, 80.0, required=False, group="operator"),
    TopicSpec("/teleop_deadman",         "Teleop Deadman", 20.0, 80.0, required=False, group="operator"),
    TopicSpec("/teleop_estop",           "Teleop E-Stop",  20.0, 80.0, required=False, group="operator"),

    # ── BMS / Battery ──
    TopicSpec("/mtt_battery/status", "BMS Status", 10.0, 50.0, required=False, group="bms"),

    # ── IMU — XSens MTi-100 (primary, required) ──
    # data_raw: 100 Hz raw measurements; data: ~100 Hz Kalman-filtered orientation.
    # Both recorded. data_raw used for odometry fusion; data for heading reference.
    TopicSpec("/mti100/data",           "MTi-100 data",     100.0, 95.0, group="imu"),
    TopicSpec("/mti100/data_raw",       "MTi-100 data_raw", 100.0, 95.0, group="imu"),
    TopicSpec("/mti100/time_reference", "MTi-100 TimeRef",   10.0, 50.0, group="imu"),

    # ── LiDAR — Hesai XT-32 (required) ──
    TopicSpec("/hesai_lidar/points",        "Hesai PointCloud",  10.0, 60.0, group="lidar"),
    TopicSpec("/hesai_lidar/lidar_packets_loss", "Hesai Pkt Loss", 10.0, 80.0, required=False, group="lidar"),

    # ── LiDAR — RoboSense Bpearl (optional — rear/trailer) ──
    TopicSpec("/rsairy_ns/points", "RS Bpearl pts", 10.0, 20.0, required=False, group="lidar"),

    # ── Camera — ZED 2i stereo (optional but important) ──
    # Rates: RGB/Depth ≈ 5 Hz compressed, IMU ≈ 100 Hz
    TopicSpec("/zed/zed_node/rgb/color/rect/image/compressed",
              "ZED RGB",   10.0, 40.0, required=False, group="camera"),
    TopicSpec("/zed/zed_node/depth/depth_registered/compressedDepth",
              "ZED Depth", 10.0, 40.0, required=False, group="camera"),
    TopicSpec("/zed/zed_node/point_cloud/cloud_registered",
              "ZED PointCloud", 10.0, 60.0, required=False, group="camera"),
    TopicSpec("/zed/zed_node/imu/data",
              "ZED IMU",  100.0, 30.0, required=False, group="camera"),

    # ── ICP mapping diagnostics (norlab_icp_mapper_ros) ──
    # These start after mapping_delay_seconds (5s in data_collection, 10s default).
    # Not required here because health_check runs before full startup; use
    # check_icp_odom / audit_tf_chain for per-session ICP validation.
    TopicSpec("/mapping/icp_odom",         "ICP Odom",          10.0, 30.0, required=False, group="mapping"),
    TopicSpec("/mapping/map",              "ICP Map",            1.0, 50.0, required=False, group="mapping"),
    TopicSpec("/mapping/trajectory_path",  "ICP Trajectory",     1.0, 50.0, required=False, group="mapping"),

    # ── Individual odometry sources feeding factor_graph_node ──
    # None required here: each is optional depending on which stack/profile is
    # running (isaac profile, ZED pos_tracking, bag-replay-only imu_odom).
    # Coherence between them (do they agree with each other, not just "are they
    # publishing") is checked separately below via the factor graph's own
    # innovation diagnostics -- see check_localization_health().
    TopicSpec("/mtt_odometry",             "MTT Odom (tacho)",  50.0, 30.0, required=False, group="localization"),
    TopicSpec("/zed/zed_node/odom",        "ZED Odom",          13.0, 40.0, required=False, group="localization"),
    TopicSpec("/isaac/vslam/odometry",     "Isaac VSLAM Odom",  50.0, 40.0, required=False, group="localization"),
    TopicSpec("/mti100/imu_odom",          "IMU Odom (replay)",  0.5, 500.0, required=False, group="localization"),
    TopicSpec("/localization/odom",        "Factor Graph Odom",  8.0, 60.0, required=False, group="localization"),
    TopicSpec("/localization/odom_fast",   "Factor Graph 100Hz", 80.0, 30.0, required=False, group="localization"),

    # ── Optional perception diagnostics ──
    TopicSpec("/trailer/articulation_angle",     "Trailer Angle",      5.0, 80.0, required=False, group="mapping"),
    TopicSpec("/trailer/pose",                   "Trailer Pose",       5.0, 80.0, required=False, group="mapping"),

    # ── Teach / Repeat supervision ──
    TopicSpec("/mtt_repeat/state",               "Repeat State",       0.5, 500.0, required=False, group="repeat"),
    TopicSpec("/mtt_repeat/ready",               "Repeat Ready",       0.5, 500.0, required=False, group="repeat"),

    # ── Operator annotations ──
    TopicSpec("/session/events", "Session Events", 0.1, 1000.0, required=False, group="infra"),
    ]

    if ENABLE_MTI10:
        topics.extend([
            # ── IMU — XSens MTi-10 (secondary, optional) ──
            TopicSpec("/mti10/data", "MTi-10 data", 100.0, 30.0, required=False, group="imu"),
            TopicSpec("/mti10/data_raw", "MTi-10 data_raw", 100.0, 30.0, required=False, group="imu"),
        ])

    if ENABLE_GPS:
        if GPS_ANTENNAS == "front":
            topics.extend([
                TopicSpec("/gps_front/fix", "GPS Front fix", 1.0, 80.0, group="gps"),
                TopicSpec("/gps_front/nmea_sentence", "GPS Front NMEA", 5.0, 80.0, required=False, group="gps"),
                TopicSpec("/gps_front/time_reference", "GPS Front TimeRef", 1.0, 80.0, required=False, group="gps"),
            ])
        elif GPS_ANTENNAS == "dual":
            topics.extend([
                TopicSpec("/gps_left/fix", "GPS Left fix", 1.0, 80.0, group="gps"),
                TopicSpec("/gps_right/fix", "GPS Right fix", 1.0, 80.0, group="gps"),
                TopicSpec("/gps/heading", "GPS Heading", 1.0, 80.0, required=False, group="gps"),
            ])
        else: # single
            topics.extend([
                TopicSpec("/gps/fix", "GPS Rover fix", 1.0, 80.0, group="gps"),
                TopicSpec("/gps/nmea_sentence", "GPS Rover NMEA", 5.0, 80.0, required=False, group="gps"),
                TopicSpec("/gps/time_reference", "GPS Rover TimeRef", 1.0, 80.0, required=False, group="gps"),
            ])

    if ENABLE_OAK:
        topics.extend([
            # ── Camera — OAK-D (optional — rear/trailer) ──
            TopicSpec("/oak/rgb/image_rect", "OAK RGB", OAK_RATE_HZ, 40.0, required=False, group="camera"),
            TopicSpec("/oak/stereo/image_raw", "OAK Depth", OAK_RATE_HZ, 60.0, required=False, group="camera"),
            TopicSpec("/oak/points", "OAK PointCloud", OAK_RATE_HZ, 70.0, required=False, group="camera"),
        ])

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
        f"Serial: check the expected Reach symlink exists inside container "
        f"(front={FRONT_DEV}, rover={ROVER_DEV}; run: sudo bash scripts/setup_udev_reach_rs.sh --front on the robot). "
        f"TCP: front at {FRONT_IP}:{FRONT_PORT} or rover at {ROVER_IP}:{ROVER_PORT} — check ReachView3 TCP output enabled. "
        "If /gps/nmea_sentence is alive but /gps/fix stays at 0, the Reach is connected "
        "but GGA is missing or rejected. Verify GGA 5 Hz + RMC 1 Hz on the rover USB output."
    ),
    "camera": (
        "ZED 0 images right after startup is often a timing issue: the SDK can take several seconds "
        "to open the camera and initialize AI/depth. Check /usr/local/zed/settings/SN*.conf, "
        "ZED_Diagnostic, and rerun the health check after sensors has fully started. "
        "OAK is only checked when ENABLE_OAK=true."
    ),
    "lidar": (
        f"Hesai: check {HESAI_IP} on the network (ping). "
        f"RS Bpearl: check {RS_IP} on the RoboSense/RS-Airy link "
        "(robot host address is 192.168.1.102)."
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
        "ICP odom=0: (1) mapper may not have started yet (mapping_delay_seconds=5s in "
        "data_collection, 10s in live_robot — run this check after that window). "
        "(2) check 'robot_frame: base_footprint' in _icp_mapper.yaml "
        "(base_link causes TF loop → odom never published). "
        "(3) verify TF chain with: docker compose run --rm audit_tf. "
        "If ENABLE_CLOUD_MERGER=false (default), /merged_points_filtered absent is expected."
    ),
    "localization": (
        "Odometry sources are all optional individually (which ones exist depends on "
        "the profile: isaac_vslam profile for Isaac odom, ZED pos_tracking for ZED odom, "
        "imu_odom only runs in bag replay). localization/odom absent means "
        "factor_graph_node isn't running (start the 'localization' service). "
        "See the Localization / Factor Graph health section below for whether the "
        "sources that ARE present actually agree with each other."
    ),
}


# ── Phase 1: Network pre-checks (no ROS) ──

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

    gps_devices = {
        "front": [FRONT_DEV],
        "single": [ROVER_DEV],
        "dual": ["/dev/reach_left", "/dev/reach_right"],
    }.get(GPS_ANTENNAS, [FRONT_DEV])
    if not ENABLE_GPS:
        print(f"  {DIM}–  GPS disabled by ENABLE_GPS=false; skipping Reach serial device check{RESET}")
    else:
        for gps_dev in gps_devices:
            if os.path.exists(gps_dev):
                print(f"  {GREEN}OK{RESET}  {gps_dev}  exists (GPS serial device OK)")
            else:
                print(f"  {RED}FAIL{RESET}  {gps_dev}  MISSING")
                print(f"      {YELLOW}→ GPS driver will retry silently, 0 messages in bag.{RESET}")
                print(f"      {YELLOW}  Fix: sudo bash scripts/setup_udev_reach_rs.sh --{GPS_ANTENNAS}  (on robot host){RESET}")
                any_error = True

    # ZED calibration file — /usr/local/zed/settings/SN*.conf
    zed_settings_dir = "/usr/local/zed/settings"
    zed_cal_files = glob.glob(os.path.join(zed_settings_dir, "SN*.conf"))
    if zed_cal_files:
        for f in zed_cal_files:
            print(f"  {GREEN}OK{RESET}  ZED calibration  {DIM}{os.path.basename(f)}{RESET}")
    else:
        print(f"  {YELLOW}WARN{RESET}  ZED calibration  MISSING  ({zed_settings_dir}/SN*.conf)")
        print(f"      {YELLOW}→ ZED SDK will attempt download (needs internet).{RESET}")
        print(f"      {YELLOW}  Without calibration: IMU starts briefly then SDK crashes → 0 images.{RESET}")
        print(f"      {YELLOW}  Fix: connect robot to internet once, or copy SN<serial>.conf manually.{RESET}")
        # Not a hard error — SDK might download successfully

    # ZED resources — pos_tracking models (optional but good to know)
    zed_resources = "/usr/local/zed/resources"
    if os.path.isdir(zed_resources) and os.listdir(zed_resources):
        print(f"  {GREEN}OK{RESET}  ZED resources    {DIM}{zed_resources}{RESET}  present")
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
    s = f"{GREEN}OK{RESET}" if ok else f"{YELLOW}WARN{RESET}"
    detail = "reachable" if ok else f"no response — Hesai at {HESAI_IP} may be off or wrong IP"
    print(f"  {s}  Hesai XT-32   {DIM}{HESAI_IP}{RESET}  {detail}")
    if not ok:
        any_net_warn = True

    # RoboSense / RS-Airy LiDAR
    ok = _ping(RS_IP)
    s = f"{GREEN}OK{RESET}" if ok else f"{YELLOW}WARN{RESET}"
    detail = "reachable" if ok else (
        f"no response — RS-Airy at {RS_IP} may be off, wrong IP, or on the wrong interface"
    )
    print(f"  {s}  RS-Airy       {DIM}{RS_IP}{RESET}  {detail}")
    if not ok:
        any_net_warn = True

    # GPS connectivity
    if not ENABLE_GPS:
        print(f"  {DIM}GPS disabled by ENABLE_GPS=false — network/serial checks skipped{RESET}")
    elif GPS_MODE == "tcp":
        gps_ip = FRONT_IP if GPS_ANTENNAS == "front" else ROVER_IP
        gps_port = FRONT_PORT if GPS_ANTENNAS == "front" else ROVER_PORT
        gps_label = "GPS Front" if GPS_ANTENNAS == "front" else "GPS Rover"
        print(f"  {DIM}GPS mode=tcp — testing TCP connection to Reach receiver{RESET}")
        ping_ok = _ping(gps_ip)
        if ping_ok:
            tcp_ok, msg = _tcp_connect(gps_ip, gps_port)
            if tcp_ok:
                print(f"  {GREEN}OK{RESET}  {gps_label:<13} {DIM}{gps_ip}:{gps_port}{RESET}  {msg}")
            else:
                print(f"  {RED}FAIL{RESET}  {gps_label:<13} {DIM}{gps_ip}:{gps_port}{RESET}  {msg}")
                print(f"      {YELLOW}→ Reach receiver reachable but port {gps_port} refused. "
                      f"Enable TCP output in ReachView3.{RESET}")
                any_net_warn = True
        else:
            print(f"  {RED}FAIL{RESET}  {gps_label:<13} {DIM}{gps_ip}:{gps_port}{RESET}  "
                  f"host unreachable — Reach receiver not on network?")
            any_net_warn = True
    else:
        gps_dev = FRONT_DEV if GPS_ANTENNAS == "front" else ROVER_DEV
        print(f"  {DIM}GPS mode=serial ({gps_dev}) — TCP not checked{RESET}")

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


# ── Phase 2+3: ROS topic monitoring ──

def run_ros_healthcheck(duration: float, wait_timeout: float) -> int:
    rclpy.init()
    node = rclpy.create_node("health_check_node")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    def spin_for(seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            executor.spin_once(timeout_sec=0.0)

    counts: Dict[str, int] = {s.topic: 0 for s in TOPICS}
    # Track worst GPS fix quality seen during the window.
    # Single-antenna mode: /gps/fix; dual legacy: /gps_left/fix + /gps_right/fix.
    gps_fix_worst: Dict[str, int] = {
        "/gps/fix":       +99,   # single rover (primary)
        "/gps_left/fix":  +99,   # dual legacy
        "/gps_right/fix": +99,   # dual legacy
        "/gps_front/fix": +99,   # front (Emlid on ZED)
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

    # ── Factor graph innovation diagnostics: per-source agreement with the
    # graph's own IMU-predicted state. This is what actually answers "does
    # ICP odom jump/diverge from VSLAM (or vice versa)": both are z-scored
    # against the SAME predicted reference each tick, so a spike in one
    # channel while the others stay calm IS the divergence signal — no need
    # to re-derive frame/extrinsic handling here, factor_graph_node.cpp
    # already does it correctly. Layout: see setup_publishers() in
    # factor_graph_node.cpp (22 elements as of 2026-07-27).
    DIAG_CHANNELS = {
        "gps":          (0, 2),   # (valid_idx, zscore_idx)
        "articulation": (3, 5),
        "lidar_odom":   (6, 9),
        "trailer":      (10, 13),
        "visual_odom":  (14, 17),
        "map_anchor":   (18, 21),
    }
    diag_stats = {
        name: {"valid_count": 0, "max_abs_zscore": 0.0} for name in DIAG_CHANNELS
    }
    diag_msg_count = 0

    def on_innovation_diag(msg: Float64MultiArray):
        nonlocal diag_msg_count
        with lock:
            diag_msg_count += 1
            data = msg.data
            for name, (valid_idx, zscore_idx) in DIAG_CHANNELS.items():
                if len(data) <= zscore_idx:
                    continue
                if data[valid_idx] < 0.5:
                    continue
                diag_stats[name]["valid_count"] += 1
                z = abs(data[zscore_idx])
                if z > diag_stats[name]["max_abs_zscore"]:
                    diag_stats[name]["max_abs_zscore"] = z

    diag_sub = node.create_subscription(
        Float64MultiArray, "/localization/factor_graph/innovation_diagnostics",
        on_innovation_diag, best_effort_qos)
    subs.append(diag_sub)

    # ── ISAM2 stability: count "Smoother update failed" / "Resetting
    # smoother" ERRORs from factor_graph_node during the window. Any reset
    # means the smoother lost its history within the lag window (3s
    # default) — the published pose doesn't jump, but recent corrections
    # are gone. Zero during a healthy window is expected; recurring resets
    # mean don't trust the output right now.
    smoother_reset_count = 0

    def on_rosout(msg: RosoutLog):
        nonlocal smoother_reset_count
        if msg.name != "factor_graph_node" or msg.level < 40:  # ERROR=40
            return
        if "Smoother update failed" in msg.msg or "Resetting smoother" in msg.msg:
            with lock:
                smoother_reset_count += 1

    rosout_sub = node.create_subscription(RosoutLog, "/rosout", on_rosout, reliable_qos)
    subs.append(rosout_sub)

    def make_counter(topic: str):
        def cb(msg):
            with lock:
                counts[topic] += 1
                if topic in gps_fix_worst and hasattr(msg, "status"):
                    gps_fix_worst[topic] = min(gps_fix_worst[topic], int(msg.status.status))
        return cb

    required_primary = {s.topic for s in TOPICS if s.required and s.group in ("can", "imu", "lidar", "gps")}

    # ── Phase 2: Smart wait for primary sensors ──
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
            print(f"\r  {GREEN}OK{RESET} All primary sensors found ({elapsed:.0f}s)."
                  f"{' ' * 30}")
            break

        if elapsed >= wait_timeout:
            print(f"\r  {YELLOW}WARN{RESET} Timeout after {wait_timeout:.0f}s. Missing: "
                  f"{', '.join(t.split('/')[-1] for t in sorted(missing_required))}"
                  f"{' ' * 20}")
            break

        spin_for(1.0)

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
        for k in diag_stats:
            diag_stats[k] = {"valid_count": 0, "max_abs_zscore": 0.0}
        diag_msg_count = 0
        smoother_reset_count = 0

    # ── Phase 3: Measure ──
    print(f"{BOLD}{CYAN}── Phase 3: Measuring for {duration:.0f}s ─────────────────{RESET}")
    t_measure_start = time.monotonic()
    for i in range(int(duration)):
        bar_done = "█" * (i + 1)
        bar_left = "░" * (int(duration) - i - 1)
        print(f"\r  [{bar_done}{bar_left}] {i+1}/{int(duration)}s", end="", flush=True)
        spin_for(1.0)
    actual_duration = time.monotonic() - t_measure_start
    print(f"\r  Measurement complete.{' ' * 30}\n")

    discovered = {t for t, _ in node.get_topic_names_and_types()}
    service_names = {name for name, _ in node.get_service_names_and_types()}
    node.destroy_node()
    executor.shutdown(timeout_sec=2.0)
    rclpy.shutdown()

    # ── Phase 4: Report ──
    COL = [44, 12, 10, 20, 5]
    sep = "─" * (sum(COL) + 4)
    header = (f"{'Topic':<{COL[0]}} {'Expected':>{COL[1]}} {'Sampled':>{COL[2]}} "
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
        elif n_msgs == 0 and spec.topic not in LATCHED and spec.expected_hz > 0.3:
            status_str = "NO MSGS"
            color = RED if spec.required else YELLOW
            if spec.required:
                has_error = True
            else:
                has_warning = True
        else:
            status_str = "ACTIVE"
            color = GREEN

        req_str = "OK" if spec.required else "opt"
        exp_str = f"{spec.expected_hz:.0f} Hz"
        tdisplay = spec.topic if len(spec.topic) <= COL[0] else spec.topic[:COL[0]-2] + ".."

        print(
            f"{color}{tdisplay:<{COL[0]}} {exp_str:>{COL[1]}} "
            f"{actual_hz:>{COL[2]-3}.1f} Hz {status_str:>{COL[3]}} {req_str:>{COL[4]}}{RESET}"
        )

    print(sep)

    # ── GPS fix quality ──
    gps_quality_ok = True
    if ENABLE_GPS:
        print(f"\n{BOLD}GPS fix quality (worst seen during window):{RESET}")
        expected_gps_topics = []
        if GPS_ANTENNAS == "front":
            expected_gps_topics = ["/gps_front/fix"]
        elif GPS_ANTENNAS == "dual":
            expected_gps_topics = ["/gps_left/fix", "/gps_right/fix"]
        else:
            expected_gps_topics = ["/gps/fix"]

        for gps_topic in expected_gps_topics:
            worst_status = gps_fix_worst.get(gps_topic, +99)
            label = {
                "/gps/fix":       "Rover (single)",
                "/gps_left/fix":  "Left  (dual)  ",
                "/gps_right/fix": "Right (dual)  ",
                "/gps_front/fix": "Front (ZED)   ",
            }.get(gps_topic, gps_topic)
            if worst_status == +99:
                fix_label = "no data"
                color = RED
                expected_dev = FRONT_DEV if GPS_ANTENNAS == "front" else ROVER_DEV
                nmea_topic = gps_topic.rsplit("/", 1)[0] + "/nmea_sentence"
                if counts.get(nmea_topic, 0) > 0:
                    note = (
                        f"NMEA is flowing on {nmea_topic}, but no valid GGA fix is being published. "
                        "Check ReachView3 USB output: enable GGA and RMC, then verify sky view/fix."
                    )
                else:
                    note = f"no messages — {expected_dev} missing, no NMEA output, or driver not connected"
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
    else:
        print(f"\n{BOLD}GPS fix quality:{RESET} {DIM}skipped (ENABLE_GPS=false){RESET}")

    # ── Localization / Factor Graph health ──
    # Individual source Hz is already in the topic table above (localization
    # group: mtt_odometry, zed odom, isaac vslam odom, imu_odom,
    # localization/odom, localization/odom_fast). This section is the
    # cross-check: does each source actually AGREE with the graph's own
    # IMU-predicted state (which is how a divergence between two sources,
    # e.g. ICP vs VSLAM, shows up — as elevated z-score on one or both,
    # against the same reference), and is the ISAM2 solver itself stable
    # (converging, not resetting).
    print(f"\n{BOLD}Localization / Factor Graph health:{RESET}")
    if diag_msg_count == 0:
        print(f"  {DIM}–  factor_graph_node not running or not publishing "
              f"innovation_diagnostics (start the 'localization' service).{RESET}")
    else:
        print(f"  {DIM}{diag_msg_count} diagnostics samples over the window{RESET}")
        ZSCORE_WARN, ZSCORE_FAIL = 3.0, 6.0
        for name, stats in diag_stats.items():
            vc = stats["valid_count"]
            mz = stats["max_abs_zscore"]
            if vc == 0:
                print(f"  {DIM}–  {name:<13} no valid factors this window "
                      f"(source not active, or not enabled, or OOSM-lagged every tick){RESET}")
                continue
            if mz >= ZSCORE_FAIL:
                color, verdict = RED, "DIVERGING — jump vs the fused estimate"
                has_warning = True
            elif mz >= ZSCORE_WARN:
                color, verdict = YELLOW, "elevated disagreement"
                has_warning = True
            else:
                color, verdict = GREEN, "OK"
            mark = "OK" if color == GREEN else "WARN"
            print(f"  {color}{mark}  {name:<13} n={vc:<4} max|z|={mz:5.2f}  {verdict}{RESET}")

        if smoother_reset_count > 0:
            print(f"  {RED}WARN  ISAM2 smoother reset {smoother_reset_count}x during window "
                  f"(IndeterminantLinearSystemException) — recent corrections within the "
                  f"lag window were lost; the published pose itself did not jump, but "
                  f"don't trust it as fully converged right now.{RESET}")
            has_warning = True
        else:
            print(f"  {GREEN}OK  ISAM2 smoother: stable, 0 resets{RESET}")

        odom_active = counts.get("/localization/odom", 0) > 0
        odom_fast_active = counts.get("/localization/odom_fast", 0) > 0
        graph_trustworthy = (
            odom_active and odom_fast_active and smoother_reset_count == 0
            and all(s["max_abs_zscore"] < ZSCORE_FAIL for s in diag_stats.values())
        )
        color = GREEN if graph_trustworthy else YELLOW
        mark = "OK" if graph_trustworthy else "WARN"
        verdict = "safe to use for control/localization" if graph_trustworthy else "review warnings above before trusting for control"
        print(f"  {color}{mark}  Overall: {verdict}{RESET}")

    # ── Group-level hints for missing sensors ──
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
        "/mtt_repeat/mark_ready",
    }
    wiln_topics = {
        "/wiln/command",
        "/wiln/trajectory",
        "/wiln/global_plan",
        "/wiln/teach/state",
        "/wiln/route/state",
        "/wiln/replay/state",
        "/wiln/follower/state",
    }
    repeat_service_count = sum(1 for svc in repeat_services if svc in service_names)
    wiln_topic_count = sum(1 for topic in wiln_topics if topic in discovered)
    repeat_topics_present = {"/mtt_repeat/state", "/mtt_repeat/ready"} & discovered
    repeat_stack_running = repeat_service_count > 0 or bool(repeat_topics_present) or wiln_topic_count > 0
    if not repeat_stack_running:
        print(f"  {DIM}– WILN / repeat supervisor not running in this session.{RESET}")
        print(f"    {DIM}Start with: docker compose --profile wiln up -d wiln{RESET}")
    else:
        icp_ok = counts.get("/mapping/icp_odom", 0) > 0
        health_ok = counts.get("/mtt_health", 0) > 0
        repeat_ok = repeat_service_count == len(repeat_services) and "/wiln/command" in discovered and icp_ok and health_ok
        color = GREEN if repeat_ok else YELLOW
        print(f"  {color}{'OK' if repeat_ok else 'WARN'} services: {repeat_service_count}/{len(repeat_services)}{RESET}")
        print(f"  {color}{'OK' if '/wiln/command' in discovered else 'WARN'} WILN command topic: {'yes' if '/wiln/command' in discovered else 'no'}{RESET}")
        print(f"  {color}{'OK' if wiln_topic_count else 'WARN'} WILN topics: {wiln_topic_count}/{len(wiln_topics)}{RESET}")
        print(f"  {color}{'OK' if repeat_ok else 'WARN'} repeat topics: state={'yes' if '/mtt_repeat/state' in discovered else 'no'} ready={'yes' if '/mtt_repeat/ready' in discovered else 'no'}  icp={'yes' if icp_ok else 'no'}  mtt_health={'yes' if health_ok else 'no'}{RESET}")
        if "/wiln/command" not in discovered:
            print(f"    {YELLOW}No /wiln/command: the wiln container likely crashed or did not start. Check: docker compose logs --tail=200 wiln{RESET}")
        if repeat_service_count != len(repeat_services):
            missing = sorted(repeat_services - service_names)
            print(f"    {YELLOW}Missing repeat services: {', '.join(missing)}{RESET}")
        if not repeat_ok:
            has_warning = True

    # ── M5 motion-model experiment pose-source readiness ──
    # Targeted summary for mtt_experiment_conductor.py / mtt_experiment_monitor.py
    # sessions (ice/asphalt/grass identification runs). Added 2026-07-29 after a
    # full ~1h grass session recorded with the ZED camera never having come up:
    # /isaac/vslam/odometry was "present but zero messages" the whole time (the
    # node was running but starved of camera frames), and this was only noticed
    # in POST-session bag analysis. The individual topic rows above already show
    # this, but it's easy to skim past in a 40+ row table -- this section exists
    # so the ICP/VSLAM readiness question has one unambiguous answer, right above
    # the final verdict, that a tired operator in a hurry cannot miss.
    print(f"\n{BOLD}M5 experiment pose-source readiness (real-time monitor):{RESET}")
    icp_live = counts.get("/mapping/icp_odom", 0) > 0
    vslam_live = counts.get("/isaac/vslam/odometry", 0) > 0
    zed_image_live = counts.get("/zed/zed_node/rgb/color/rect/image/compressed", 0) > 0
    zed_imu_live = counts.get("/zed/zed_node/imu/data", 0) > 0

    print(f"  {GREEN if icp_live else RED}{'OK' if icp_live else 'FAIL'}  ICP odom (/mapping/icp_odom):   "
          f"{'live, ' + str(counts.get('/mapping/icp_odom', 0)) + ' msgs' if icp_live else 'NO MESSAGES'}{RESET}")
    print(f"  {GREEN if vslam_live else YELLOW}{'OK' if vslam_live else 'FAIL'}  VSLAM odom (/isaac/vslam/odometry): "
          f"{'live, ' + str(counts.get('/isaac/vslam/odometry', 0)) + ' msgs' if vslam_live else 'NO MESSAGES'}{RESET}")
    if not vslam_live:
        if not zed_image_live and not zed_imu_live:
            print(f"    {YELLOW}→ ZED camera itself has 0 images AND 0 IMU messages -- VSLAM has nothing to "
                  f"track. This is a camera hardware/driver problem (check cable/USB/power and "
                  f"/usr/local/zed/settings/SN*.conf), not a VSLAM config problem.{RESET}")
        elif zed_imu_live and not zed_image_live:
            print(f"    {YELLOW}→ ZED IMU is alive but 0 images -- camera opened but image stream failed. "
                  f"Check 'docker compose logs sensors' and 'docker compose logs isaac_vslam'.{RESET}")
        else:
            print(f"    {YELLOW}→ ZED images are flowing but VSLAM still shows 0 odom messages -- check "
                  f"'docker compose logs isaac_vslam' directly (tracking may be failing/lost).{RESET}")

    if icp_live and vslam_live:
        print(f"  {GREEN}{BOLD}OK  Both live: experiment_monitor will use VSLAM (fast) with ICP as fallback, as designed.{RESET}")
    elif icp_live and not vslam_live:
        print(f"  {YELLOW}{BOLD}WARN  ICP only: experiment_monitor will run correctly on the ICP fallback path "
              f"for the ENTIRE session (not just briefly) -- this is safe but slower/lower-rate than the "
              f"VSLAM-primary design intends. Fine to proceed if you've confirmed this is expected "
              f"(e.g. ZED genuinely not mounted this session).{RESET}")
        has_warning = True
    elif vslam_live and not icp_live:
        print(f"  {YELLOW}{BOLD}WARN  VSLAM only: analyze_ice_session.py (offline fit) is ICP-primary and will have "
              f"NOTHING to fit on until ICP comes up -- check the 'mapping' service.{RESET}")
        has_warning = True
    else:
        print(f"  {RED}{BOLD}FAIL  NEITHER ICP NOR VSLAM is publishing -- the experiment has NO usable pose "
              f"source. Do not start mtt_experiment_conductor.py yet; fix mapping and/or the ZED camera first.{RESET}")
        has_warning = True

    # ── Final verdict ──
    print()
    if has_error:
        print(f"{RED}{BOLD}ERROR  HEALTH CHECK FAILED — required sensors missing or too slow.{RESET}")
        print("   Fix sensor issues before starting recording.\n")
        return 1
    elif has_warning or not gps_quality_ok:
        print(f"{YELLOW}{BOLD}WARN   HEALTH CHECK PASSED WITH WARNINGS.{RESET}")
        if not gps_quality_ok:
            print("   GPS fix quality below RTK — recording will proceed but GPS accuracy is poor.")
        print("   Recording can start, but check warnings above.\n")
        return 2
    else:
        print(f"{GREEN}{BOLD}OK  ALL SENSORS HEALTHY — ready to record.{RESET}\n")
        return 0


def main() -> int:
    print(f"\n{BOLD}{CYAN}══════════════════════════════════════════════════{RESET}")
    print(f"{BOLD}{CYAN}   MTT Sensor Health Check{RESET}")
    print(f"{BOLD}{CYAN}   duration={DURATION:.0f}s  wait={WAIT_TIMEOUT:.0f}s  "
          f"gps_mode={GPS_MODE}  gps_antennas={GPS_ANTENNAS}  tachometer_mode={TACHOMETER_MODE}{RESET}")
    print(f"{BOLD}{CYAN}   enable_gps={ENABLE_GPS}  enable_oak={ENABLE_OAK}  enable_mti10={ENABLE_MTI10}{RESET}")
    print(f"{BOLD}{CYAN}══════════════════════════════════════════════════{RESET}")

    run_filesystem_prechecks()
    run_network_prechecks()

    return run_ros_healthcheck(DURATION, WAIT_TIMEOUT)


if __name__ == "__main__":
    sys.exit(main())
