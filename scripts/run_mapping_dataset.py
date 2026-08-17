#!/usr/bin/env python3
"""Run the replay mapper stack and save ICP artefacts for dataset building."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

WORKSPACE_ROOT = Path(os.environ.get("WORKSPACE", Path(__file__).resolve().parents[1])).resolve()
BAG_REPLAY_SCRIPTS = WORKSPACE_ROOT / "demos" / "bag_replay" / "scripts"
sys.path.insert(0, str(BAG_REPLAY_SCRIPTS))

from offline_icp import (  # noqa: E402
    SAVE_MAP_SERVICE,
    parse_bag_metadata,
    resolve_session_dir,
    save_mapping_outputs,
    start_process,
    terminate_process,
    validate_icp_odom_replay,
    wait_for_service,
    write_summary,
)


def env(name: str, default: str) -> str:
    value = os.environ.get(name)
    return default if value is None or value == "" else value


def bool_env(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay a bag through the same mapping stack used by demos/bag_replay "
            "and save ICP odom/status, map.vtk, and trajectory.vtk."
        )
    )
    parser.add_argument(
        "input_path",
        nargs="?",
        default=os.environ.get("BAG_PATH") or os.environ.get("DATA_DIR") or str(WORKSPACE_ROOT / "data"),
    )
    parser.add_argument("--force", action="store_true", default=bool_env("MAPPING_DATASET_FORCE", False))
    parser.add_argument("--replay-rate", type=float, default=float(env("REPLAY_RATE", "0.25")))
    parser.add_argument(
        "--odom-source",
        default=env("MAPPING_DATASET_ODOM_SOURCE", "auto"),
        choices=["auto", "runtime", "imu", "zed", "none"],
        help="auto uses runtime odom when /mtt_tachometer exists, otherwise IMU rotation-only odom.",
    )
    parser.add_argument(
        "--ready-timeout-s",
        type=float,
        default=float(env("MAPPING_DATASET_READY_TIMEOUT_S", "60.0")),
    )
    parser.add_argument(
        "--play-timeout-margin-s",
        type=float,
        default=float(env("MAPPING_DATASET_PLAY_TIMEOUT_MARGIN_S", "180.0")),
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=float(env("MAPPING_DATASET_SETTLE_S", "8.0")),
    )
    parser.add_argument(
        "--no-global-output-map",
        dest="enable_global_output_map",
        action="store_false",
        default=bool_env("MAPPING_DATASET_ENABLE_GLOBAL_OUTPUT_MAP", True),
    )
    return parser.parse_args()


def write_cyclone_loopback_config(output_dir: Path) -> Path:
    path = output_dir / "cyclonedds_loopback.xml"
    path.write_text(
        """<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="lo" multicast="false"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers/>
    </Discovery>
  </Domain>
</CycloneDDS>
""",
        encoding="utf-8",
    )
    return path


def isolated_env(output_dir: Path) -> dict[str, str]:
    child_env = os.environ.copy()
    domain = env("DATASET_ROS_DOMAIN_ID", env("MAPPING_DATASET_ROS_DOMAIN_ID", ""))
    if not domain:
        domain = str(120 + (os.getpid() % 100))
    child_env["ROS_DOMAIN_ID"] = domain
    child_env["ROS_LOCALHOST_ONLY"] = "1"
    child_env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    child_env["RMW_IMPLEMENTATION"] = env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    child_env["CYCLONEDDS_URI"] = f"file://{write_cyclone_loopback_config(output_dir)}"
    return child_env


def run_quiet(command: list[str], log_path: Path, child_env: dict[str, str]) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8") as log:
        subprocess.run(command, cwd=WORKSPACE_ROOT, env=child_env, stdout=log, stderr=subprocess.STDOUT, check=False)


def bag_dir_for_session(session_dir: Path) -> Path:
    bag_dir = session_dir / "bag"
    if not (bag_dir / "metadata.yaml").exists():
        raise RuntimeError(f"Missing bag metadata: {bag_dir / 'metadata.yaml'}")
    return bag_dir


def choose_odom_source(requested: str, counts: dict[str, int]) -> str:
    if requested != "auto":
        return requested
    if counts.get("/mtt_tachometer", 0) > 0:
        return "runtime"
    return "imu"


def default_mapping_config(odom_source: str) -> Path:
    config_dir = WORKSPACE_ROOT / "src" / "external" / "norlab_robot" / "config" / "mapping"
    if odom_source == "imu":
        return config_dir / "_config_hesai_imu_replay.yaml"
    return config_dir / "_config_hesai_wheel_replay.yaml"


def bag_exclude_topics(odom_source: str) -> list[str]:
    topics = [
        "/robot_description",
        "/joint_states",
        "/runtime_joint_states",
        "/tf",
        "/tf_static",
        "/merged_points",
        "/merged_points_raw",
        "/merged_points_filtered",
        "/hesai_in_base",
        "/rsairy_in_base",
        "/trailer/angle",
        "/trailer/pose",
        "/trailer/body_markers",
        "/trailer/trailer_roi_cloud",
        "/planned_trajectory",
        "/real_trajectory",
        "/wiln/pose",
        "/wiln/markers",
        "/wiln/obstacles",
        "/wiln/trajectory",
        "/wiln/global_plan",
        "/wiln/control/local_plan",
        "/wiln/debug/deformed_plan",
        "/wiln/debug/horizon",
        "/wiln/replay/diagnostics",
        "/mtt_route/active_path",
        "/mtt_route/closest_pose",
        "/mtt_route/error_marker",
        "/mtt_route/start_pose",
        "/mtt_route/status_text",
        "/mtt_route/safety_status",
        "/localization/articulation_angle",
        "/localization/articulation_pitch",
        "/trailer/odom",
        "/trailer/pose_in_map",
        "/zed/zed_node/pose",
        "/mapping/icp_odom",
        "/mapping/status",
        "/mapping/map",
        "/mapping/pose_in",
        "/mapping/scan_after_deskew",
        "/mapping/scan_after_input_filters",
        "/mapping/trajectory_path",
        "/mapping/aligned_scan",
    ]
    if odom_source != "zed":
        topics.append("/zed/zed_node/odom")
    return topics


def odom_command(odom_source: str) -> list[str] | None:
    if odom_source == "none":
        return None
    if odom_source == "imu":
        return [
            "ros2",
            "run",
            "imu_odom",
            "imu_odom_node",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            f"odom_frame:={env('MAPPING_ODOM_FRAME', 'odom')}",
            "-p",
            f"robot_frame:={env('MAPPING_ROBOT_FRAME', 'base_footprint')}",
            "-p",
            f"imu_frame:={env('MAPPING_DATASET_IMU_FRAME', 'imu_link')}",
            "-p",
            f"rotation_only:={env('MAPPING_DATASET_IMU_ROTATION_ONLY', 'true')}",
            "-p",
            "real_time:=false",
            "-r",
            f"imu_topic:={env('MAPPING_DESKEW_IMU_TOPIC', '/mti100/data')}",
        ]
    if odom_source == "runtime":
        return [
            "ros2",
            "run",
            "mtt_driver",
            "mtt_odometry_node_exe",
            "--ros-args",
            "--params-file",
            str(WORKSPACE_ROOT / "demos" / "common" / "config" / "mtt_driver.yaml"),
            "-p",
            "use_sim_time:=true",
            "-p",
            "broadcast_tf:=true",
            "-p",
            "cmd_vel_topic:=cmd_vel",
            "-p",
            f"hardware_articulation_topic:={env('REPLAY_ODOM_ARTICULATION_TOPIC', '/mtt_articulation_angle')}",
            "-p",
            f"articulation_state_topic:={env('REPLAY_ODOM_ARTICULATION_STATE_TOPIC', '/unused/articulation_state')}",
            "-p",
            "articulation_state_output_topic:=mtt/articulation_state/runtime",
            "-p",
            "use_articulation_state_lidar:=false",
            "-p",
            f"imu_yaw_rate_topic:={env('REPLAY_ODOM_IMU_TOPIC', '/mti100/data')}",
            "-p",
            f"imu_yaw_rate_sign:={env('REPLAY_ODOM_IMU_YAW_SIGN', '-1.0')}",
            "-p",
            f"imu_yaw_rate_bias_rad_s:={env('REPLAY_ODOM_IMU_BIAS', '0.0')}",
            "-p",
            "imu_yaw_rate_timeout_seconds:=0.2",
            "-p",
            "use_imu_heading:=false",
            "-r",
            "mtt_odometry:=mtt_odometry/runtime",
            "-r",
            "mtt_articulation_angle:=mtt_articulation_angle/runtime",
        ]
    if odom_source == "zed":
        return [
            "python3",
            "-u",
            str(WORKSPACE_ROOT / "scripts" / "zed_odom_to_tf.py"),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            f"odom_frame:={env('MAPPING_ODOM_FRAME', 'odom')}",
            "-p",
            f"robot_frame:={env('MAPPING_ROBOT_FRAME', 'base_footprint')}",
            "-p",
            f"camera_frame:={env('ZED_ODOM_CAMERA_FRAME', 'zed_camera_link')}",
        ]
    raise RuntimeError(f"Unknown odom source: {odom_source}")


def mapping_command(odom_source: str, enable_global_output_map: bool) -> list[str]:
    mapping_config = env("MAPPING_CONFIG", str(default_mapping_config(odom_source)))
    map_publish_rate = env("MAPPING_DATASET_MAP_PUBLISH_RATE", env("MAPPING_MAP_PUBLISH_RATE", "0"))
    map_publish_radius = env("MAPPING_DATASET_MAP_PUBLISH_RADIUS_M", env("MAPPING_MAP_PUBLISH_RADIUS_M", "40.0"))
    return [
        "ros2",
        "launch",
        "norlab_robot",
        "mapping.launch.py",
        "use_sim_time:=true",
        f"mapping_points_topic:={env('MAPPING_POINTS_TOPIC', '/hesai_lidar/points')}",
        f"mapping_config:={mapping_config}",
        f"mapping_deskew:={env('MAPPING_DESKEW', 'true')}",
        f"mapping_deskew_source:={env('MAPPING_DESKEW_SOURCE', 'imu')}",
        f"mapping_deskew_imu_topic:={env('MAPPING_DESKEW_IMU_TOPIC', '/mti100/data')}",
        f"mapping_compression_voxel_size:={env('MAPPING_COMPRESSION_VOXEL_SIZE', '0.50')}",
        f"mapping_map_publish_rate:={map_publish_rate}",
        f"mapping_map_tf_publish_rate:={env('MAPPING_MAP_TF_PUBLISH_RATE', '50.0')}",
        f"mapping_min_input_points:={env('MAPPING_MIN_INPUT_POINTS', '200')}",
        f"mapping_max_translation_correction:={env('MAPPING_MAX_TRANSLATION_CORRECTION', '2.50')}",
        f"mapping_max_rotation_correction_deg:={env('MAPPING_MAX_ROTATION_CORRECTION_DEG', '35.0')}",
        f"mapping_max_velocity_ms:={env('MAPPING_MAX_VELOCITY_MS', '8.0')}",
        f"mapping_max_yaw_rate_deg_s:={env('MAPPING_MAX_YAW_RATE_DEG_S', '80.0')}",
        f"mapping_max_pose_yaw_step_deg:={env('MAPPING_MAX_POSE_YAW_STEP_DEG', '20.0')}",
        f"mapping_max_pose_yaw_odom_residual_deg:={env('MAPPING_MAX_POSE_YAW_ODOM_RESIDUAL_DEG', '12.0')}",
        f"mapping_max_pose_step_m:={env('MAPPING_MAX_POSE_STEP_M', '8.0')}",
        f"mapping_max_z_jump_m:={env('MAPPING_MAX_Z_JUMP_M', '2.0')}",
        f"mapping_max_registration_time_ms:={env('MAPPING_MAX_REGISTRATION_TIME_MS', '3000.0')}",
        f"mapping_max_idle_time:={env('MAPPING_MAX_IDLE_TIME', '600.0')}",
        f"deterministic_map_update_distance_m:={env('MAPPING_DETERMINISTIC_MAP_UPDATE_DISTANCE_M', '0.10')}",
        f"deterministic_map_update_yaw_deg:={env('MAPPING_DETERMINISTIC_MAP_UPDATE_YAW_DEG', '1.0')}",
        f"deterministic_map_min_dist_new_point:={env('MAPPING_DETERMINISTIC_MAP_MIN_DIST_NEW_POINT', '0.15')}",
        f"mapping_enable_global_output_map:={str(enable_global_output_map).lower()}",
        f"mapping_global_output_map_min_dist_new_point:={env('MAPPING_GLOBAL_OUTPUT_MAP_MIN_DIST_NEW_POINT', '0.12')}",
        f"mapping_enable_map_trimming:={env('MAPPING_ENABLE_MAP_TRIMMING', 'true')}",
        f"mapping_map_trim_radius_m:={env('MAPPING_MAP_TRIM_RADIUS_M', '20.0')}",
        f"mapping_max_map_points_before_trim:={env('MAPPING_MAX_MAP_POINTS_BEFORE_TRIM', '45000')}",
        f"mapping_map_trim_interval_scans:={env('MAPPING_MAP_TRIM_INTERVAL_SCANS', '2')}",
        f"mapping_min_pose_overlap_near_ratio:={env('MAPPING_MIN_POSE_OVERLAP_NEAR_RATIO', '0.40')}",
        f"mapping_min_pose_overlap_loose_ratio:={env('MAPPING_MIN_POSE_OVERLAP_LOOSE_RATIO', '0.55')}",
        f"mapping_min_map_overlap_near_ratio:={env('MAPPING_MIN_MAP_OVERLAP_NEAR_RATIO', '0.35')}",
        f"mapping_min_map_overlap_loose_ratio:={env('MAPPING_MIN_MAP_OVERLAP_LOOSE_RATIO', '0.50')}",
        f"mapping_max_map_update_translation_correction_m:={env('MAPPING_MAX_MAP_UPDATE_TRANSLATION_CORRECTION_M', '4.0')}",
        f"mapping_max_map_update_rotation_correction_deg:={env('MAPPING_MAX_MAP_UPDATE_ROTATION_CORRECTION_DEG', '12.0')}",
        f"mapping_enable_map_recovery:={env('MAPPING_ENABLE_MAP_RECOVERY', 'true')}",
        f"mapping_recovery_reload_after_rejections:={env('MAPPING_RECOVERY_RELOAD_AFTER_REJECTIONS', '4')}",
        f"mapping_recovery_attempt_interval_scans:={env('MAPPING_RECOVERY_ATTEMPT_INTERVAL_SCANS', '5')}",
        f"mapping_recovery_local_map_radius_m:={env('MAPPING_RECOVERY_LOCAL_MAP_RADIUS_M', '24.0')}",
        f"mapping_recovery_local_map_min_points:={env('MAPPING_RECOVERY_LOCAL_MAP_MIN_POINTS', '5000')}",
        f"mapping_recovery_local_map_max_points:={env('MAPPING_RECOVERY_LOCAL_MAP_MAX_POINTS', '60000')}",
        f"mapping_snapshot_save_interval_scans:={env('MAPPING_SNAPSHOT_SAVE_INTERVAL_SCANS', '20')}",
        f"mapping_snapshot_max_translation_correction_m:={env('MAPPING_SNAPSHOT_MAX_TRANSLATION_CORRECTION_M', '1.0')}",
        f"mapping_snapshot_max_rotation_correction_deg:={env('MAPPING_SNAPSHOT_MAX_ROTATION_CORRECTION_DEG', '5.0')}",
        f"mapping_enable_motion_adaptive_gate:={env('MAPPING_ENABLE_MOTION_ADAPTIVE_GATE', 'true')}",
        f"mapping_adaptive_max_dt_s:={env('MAPPING_ADAPTIVE_MAX_DT_S', '2.0')}",
        f"mapping_adaptive_velocity_gain:={env('MAPPING_ADAPTIVE_VELOCITY_GAIN', '1.25')}",
        f"mapping_adaptive_acceleration_gain:={env('MAPPING_ADAPTIVE_ACCELERATION_GAIN', '0.50')}",
        f"mapping_adaptive_yaw_rate_gain:={env('MAPPING_ADAPTIVE_YAW_RATE_GAIN', '1.25')}",
        f"mapping_aggressive_speed_ms:={env('MAPPING_AGGRESSIVE_SPEED_MS', '2.0')}",
        f"mapping_aggressive_yaw_rate_deg_s:={env('MAPPING_AGGRESSIVE_YAW_RATE_DEG_S', '35.0')}",
        f"mapping_pivot_linear_speed_ms:={env('MAPPING_PIVOT_LINEAR_SPEED_MS', '0.75')}",
        f"mapping_pivot_yaw_rate_deg_s:={env('MAPPING_PIVOT_YAW_RATE_DEG_S', '35.0')}",
        f"mapping_pivot_max_translation_correction_m:={env('MAPPING_PIVOT_MAX_TRANSLATION_CORRECTION_M', '1.25')}",
        f"mapping_enable_odom_bridge:={env('MAPPING_ENABLE_ODOM_BRIDGE', 'true')}",
        f"mapping_odom_bridge_after_rejections:={env('MAPPING_ODOM_BRIDGE_AFTER_REJECTIONS', '0')}",
        f"mapping_odom_bridge_min_speed_ms:={env('MAPPING_ODOM_BRIDGE_MIN_SPEED_MS', '1.5')}",
        f"mapping_enable_planar_pose_constraint:={env('MAPPING_ENABLE_PLANAR_POSE_CONSTRAINT', 'true')}",
        f"mapping_planar_pose_max_z_drift_m:={env('MAPPING_PLANAR_POSE_MAX_Z_DRIFT_M', '2.0')}",
        f"mapping_input_qos_reliable:={env('MAPPING_INPUT_QOS_RELIABLE', 'true')}",
        f"mapping_anchor_map_at_initial_robot_pose:={env('MAPPING_ANCHOR_MAP_AT_INITIAL_ROBOT_POSE', 'true')}",
        f"mapping_map_publish_radius_m:={map_publish_radius}",
        f"mapping_odom_frame:={env('MAPPING_ODOM_FRAME', 'odom')}",
        f"mapping_robot_frame:={env('MAPPING_ROBOT_FRAME', 'base_footprint')}",
    ]


def process_session(session_dir: Path, args: argparse.Namespace) -> dict[str, Any]:
    bag_dir = bag_dir_for_session(session_dir)
    output_dir = session_dir / "mapping_dataset_icp"
    log_dir = output_dir / "logs"
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    map_file = output_dir / "map.vtk"
    trajectory_file = output_dir / "trajectory.vtk"
    icp_bag_dir = output_dir / "icp_odom_replay"
    counts, duration_s = parse_bag_metadata(bag_dir / "metadata.yaml")
    odom_source = choose_odom_source(args.odom_source, counts)

    result: dict[str, Any] = {
        "session": session_dir.name,
        "session_dir": str(session_dir),
        "bag_dir": str(bag_dir),
        "status": "failed",
        "mode": "mapping_dataset_icp",
        "odom_source": odom_source,
        "requested_odom_source": args.odom_source,
        "replay_rate": args.replay_rate,
        "ros_domain_id": None,
        "map_file": str(map_file),
        "trajectory_file": str(trajectory_file),
        "icp_odom_replay": str(icp_bag_dir),
        "output_dir": str(output_dir),
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "bag_duration_seconds": duration_s,
        "topic_counts": {
            "/hesai_lidar/points": counts.get("/hesai_lidar/points", 0),
            "/mti100/data": counts.get("/mti100/data", 0),
            "/mtt_tachometer": counts.get("/mtt_tachometer", 0),
        },
    }
    if counts.get("/hesai_lidar/points", 0) <= 0:
        result["status"] = "skipped_missing_hesai"
        write_summary(output_dir, result)
        return result
    if not args.force and map_file.exists() and trajectory_file.exists() and (icp_bag_dir / "metadata.yaml").exists():
        result["status"] = "skipped_existing"
        result["icp_odom_replay_validation"] = validate_icp_odom_replay(icp_bag_dir, duration_s)
        write_summary(output_dir, result)
        return result

    child_env = isolated_env(output_dir)
    result["ros_domain_id"] = child_env["ROS_DOMAIN_ID"]
    run_quiet(["ros2", "daemon", "stop"], log_dir / "ros_daemon.log", child_env)

    description_proc = odom_proc = mapping_proc = record_proc = bag_proc = None
    try:
        shutil.rmtree(icp_bag_dir, ignore_errors=True)
        if args.force:
            for path in (map_file, trajectory_file):
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass

        description_proc = start_process(
            ["ros2", "launch", str(WORKSPACE_ROOT / "demos" / "bag_replay" / "launch" / "description_calib.launch.py")],
            log_dir / "description.log",
            env=child_env,
        )
        odom_cmd = odom_command(odom_source)
        if odom_cmd is not None:
            odom_proc = start_process(odom_cmd, log_dir / f"odom_{odom_source}.log", env=child_env)
        mapping_proc = start_process(
            mapping_command(odom_source, args.enable_global_output_map),
            log_dir / "mapping.log",
            env=child_env,
        )
        if not wait_for_service(SAVE_MAP_SERVICE, args.ready_timeout_s, log_dir, env=child_env):
            raise RuntimeError("Mapper save_map service did not appear before timeout.")

        record_proc = start_process(
            [
                "ros2",
                "bag",
                "record",
                "--output",
                str(icp_bag_dir),
                "--storage",
                "mcap",
                "--topics",
                "/mapping/icp_odom",
                "/mapping/status",
            ],
            log_dir / "record_icp_odom_status.log",
            env=child_env,
        )
        play_timeout = max(60.0, duration_s / max(args.replay_rate, 1e-6) + args.play_timeout_margin_s)
        bag_proc = start_process(
            [
                "ros2",
                "bag",
                "play",
                "--storage",
                "mcap",
                str(bag_dir),
                "--clock",
                "--rate",
                str(args.replay_rate),
                "--read-ahead-queue-size",
                env("MAPPING_DATASET_READ_AHEAD_QUEUE_SIZE", "10000"),
                "--disable-keyboard-controls",
                "--qos-profile-overrides-path",
                str(WORKSPACE_ROOT / "src" / "external" / "norlab_robot" / "config" / "rosbag_record" / "qos_replay_override.yaml"),
                "--exclude-topics",
                *bag_exclude_topics(odom_source),
            ],
            log_dir / "bag_play.log",
            env=child_env,
        )
        try:
            bag_returncode = bag_proc.wait(timeout=play_timeout)
        except subprocess.TimeoutExpired:
            terminate_process(bag_proc, grace_s=5.0)
            bag_returncode = bag_proc.returncode if bag_proc.returncode is not None else 124
            result["playback_error"] = f"bag_play_timeout_after_{play_timeout:.1f}s"
        if bag_returncode != 0 and "playback_error" not in result:
            result["playback_error"] = f"bag_play_exit_{bag_returncode}"

        time.sleep(args.settle_s)
        terminate_process(record_proc)
        record_proc = None

        saved, save_errors = save_mapping_outputs(
            map_file,
            trajectory_file,
            log_dir,
            "mapping_dataset",
            duration_s,
            env=child_env,
        )
        result["map_size_bytes"] = map_file.stat().st_size if map_file.exists() else 0
        result["trajectory_size_bytes"] = trajectory_file.stat().st_size if trajectory_file.exists() else 0
        result["icp_odom_replay_validation"] = validate_icp_odom_replay(icp_bag_dir, duration_s)
        if not saved:
            result["status"] = "partial_saved" if result["icp_odom_replay_validation"]["message_count"] else "failed_save"
            result["save_errors"] = save_errors
        elif "playback_error" in result:
            result["status"] = "partial_saved"
        else:
            result["status"] = "ok"
        return result
    except Exception as exc:  # noqa: BLE001
        result["status"] = "failed_exception"
        result["error"] = str(exc)
        return result
    finally:
        terminate_process(bag_proc)
        terminate_process(record_proc)
        terminate_process(mapping_proc)
        terminate_process(odom_proc)
        terminate_process(description_proc)
        run_quiet(["ros2", "daemon", "stop"], log_dir / "ros_daemon.log", child_env)
        write_summary(output_dir, result)


def main() -> int:
    args = parse_args()
    try:
        sessions = resolve_session_dir(args.input_path)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(f"Mapping dataset input: {args.input_path}", flush=True)
    print(f"Sessions found: {len(sessions)}", flush=True)
    print(f"Replay rate: {args.replay_rate}x", flush=True)
    print(f"Odom source request: {args.odom_source}", flush=True)
    print("", flush=True)

    results = []
    failures = 0
    for index, session_dir in enumerate(sessions, start=1):
        print(f"[{index}/{len(sessions)}] {session_dir.name}", flush=True)
        result = process_session(session_dir, args)
        results.append(result)
        if result["status"] == "ok":
            print(f"  OK      {result['output_dir']}", flush=True)
        elif str(result["status"]).startswith("skipped"):
            print(f"  SKIPPED {result['status']}", flush=True)
        else:
            failures += 1
            print(f"  {result['status']} {result.get('error') or result.get('playback_error') or ''}", flush=True)
        print("", flush=True)

    report_dir = WORKSPACE_ROOT / "data" / "reports" / "mapping_icp"
    report_dir.mkdir(parents=True, exist_ok=True)
    report_path = report_dir / f"mapping_dataset_icp_report_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.yaml"
    report_path.write_text(yaml.safe_dump(results, sort_keys=False), encoding="utf-8")
    print(f"Report: {report_path}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
