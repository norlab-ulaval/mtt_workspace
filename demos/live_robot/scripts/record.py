#!/usr/bin/env python3
"""Record a live MTT session and save the useful local and robot context next to the bag."""

from __future__ import annotations

import argparse
import os
import signal
import shutil
import subprocess
import time
from datetime import datetime
from pathlib import Path

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record a live MTT session with local metadata.")
    parser.add_argument("--config", required=True, help="Path to the records.yaml file.")
    parser.add_argument("--label", default="", help="Optional suffix added to the record directory.")
    parser.add_argument("--dry-run", action="store_true", help="Print the planned output path and bag command, then exit.")
    return parser.parse_args()


def load_config(config_path: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


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


def resolve_path(path_value: str | None, workspace_root: Path, base_dir: Path) -> Path | None:
    if not path_value:
        return None
    path = Path(path_value)
    if path.is_absolute():
        return path
    candidate = (base_dir / path).resolve()
    if candidate.exists():
        return candidate
    return (workspace_root / path).resolve()


def infer_workspace_root(config_path: Path) -> Path:
    for candidate in [config_path.parent, *config_path.parents]:
        if (candidate / "src").exists() and (candidate / "demos").exists():
            return candidate
    return config_path.parent


def load_topics(config: dict, config_path: Path, workspace_root: Path) -> list[str]:
    topics = list(config.get("topics", []))
    if topics:
        return topics

    topics_file = resolve_path(config.get("topics_file"), workspace_root, config_path.parent)
    if not topics_file or not topics_file.exists():
        return []

    topics_key = config.get("topics_key", "topics_main")
    with topics_file.open("r", encoding="utf-8") as stream:
        topic_config = yaml.safe_load(stream) or {}
    return list(topic_config.get(topics_key, []))


def write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def run_capture(command: list[str], destination: Path, timeout_s: float | None = None) -> int:
    try:
        result = subprocess.run(command, check=False, capture_output=True, text=True, timeout=timeout_s)
        write_text(destination, result.stdout)
        if result.stderr:
            write_text(destination.with_suffix(destination.suffix + ".stderr"), result.stderr)
        return result.returncode
    except subprocess.TimeoutExpired as exc:
        write_text(destination, exc.stdout or "")
        write_text(destination.with_suffix(destination.suffix + ".stderr"), (exc.stderr or "") + "\nTIMEOUT\n")
        return 124


def call_rosbag_service(record_dir: Path, service: str, service_type: str, request: str = "{}",
                        timeout_s: float = 10.0) -> int:
    runtime_dir = record_dir / "runtime"
    safe_name = service.strip("/").replace("/", "_")
    return run_capture(
        ["ros2", "service", "call", service, service_type, request],
        runtime_dir / f"{safe_name}.txt",
        timeout_s=timeout_s,
    )


def terminate_process(process: subprocess.Popen, grace_s: float = 10.0) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=grace_s)
        return
    except subprocess.TimeoutExpired:
        pass
    if process.poll() is None:
        process.kill()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            pass


def save_vcs_snapshot(workspace_root: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    run_capture(["git", "-C", str(workspace_root), "rev-parse", "--abbrev-ref", "HEAD"], destination / "branch.txt")
    run_capture(["git", "-C", str(workspace_root), "rev-parse", "HEAD"], destination / "commit.txt")
    run_capture(["git", "-C", str(workspace_root), "status", "--short"], destination / "status.txt")
    run_capture(["git", "-C", str(workspace_root), "diff", "--no-ext-diff"], destination / "diff.patch")


def save_runtime_graph_snapshot(destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    run_capture(["ros2", "node", "list"], destination / "node_list.txt", timeout_s=10)
    run_capture(["ros2", "topic", "list", "-t"], destination / "topic_list.txt", timeout_s=10)
    run_capture(["ros2", "service", "list", "-t"], destination / "service_list.txt", timeout_s=10)
    run_capture(["ros2", "action", "list", "-t"], destination / "action_list.txt", timeout_s=10)


def save_runtime_artifacts(record_dir: Path) -> None:
    artifact_dir = record_dir / "artifacts"
    artifact_dir.mkdir(parents=True, exist_ok=True)

    map_path = artifact_dir / "final_map.vtk"
    trajectory_path = artifact_dir / "final_trajectory.vtk"
    route_path = artifact_dir / "final_route.ltr"

    run_capture(
        [
            "ros2", "service", "call",
            "/mapping/save_map",
            "norlab_icp_mapper_ros/srv/SaveMap",
            f'{{map_file_name: {{data: "{map_path}"}}}}',
        ],
        artifact_dir / "save_map_call.txt",
        timeout_s=20,
    )
    run_capture(
        [
            "ros2", "service", "call",
            "/mapping/save_trajectory",
            "norlab_icp_mapper_ros/srv/SaveTrajectory",
            f'{{trajectory_file_name: {{data: "{trajectory_path}"}}}}',
        ],
        artifact_dir / "save_trajectory_call.txt",
        timeout_s=20,
    )
    run_capture(
        [
            "ros2", "service", "call",
            "/save_map_traj",
            "wiln/srv/SaveMapTraj",
            f'{{file_name: {{data: "{route_path}"}}}}',
        ],
        artifact_dir / "save_wiln_route_call.txt",
        timeout_s=20,
    )


def ensure_bag_metadata(bag_dir: Path, diagnostics_dir: Path, wait_s: int = 180) -> bool:
    metadata_path = bag_dir / "metadata.yaml"
    deadline = time.time() + wait_s
    while time.time() < deadline:
        if metadata_path.exists():
            return True
        time.sleep(1.0)

    run_capture(
        ["ros2", "bag", "reindex", str(bag_dir)],
        diagnostics_dir / "ros2_bag_reindex.txt",
        timeout_s=120,
    )

    deadline = time.time() + 30
    while time.time() < deadline:
        if metadata_path.exists():
            return True
        time.sleep(1.0)

    return metadata_path.exists()


def save_metadata(config: dict, config_path: Path, record_dir: Path, bag_dir: Path, topics: list[str]) -> None:
    metadata = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "workspace": os.environ.get("WORKSPACE"),
        "config_path": str(config_path),
        "bag_directory": str(bag_dir),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "robot_host": os.environ.get("ROBOT_HOST"),
        "robot_ssh_target": os.environ.get("ROBOT_SSH_TARGET"),
        "robot_foxglove_url": os.environ.get("ROBOT_FOXGLOVE_URL"),
        "robot_zenoh_endpoint": os.environ.get("ROBOT_ZENOH_ENDPOINT"),
        "gps_mode": os.environ.get("GPS_MODE"),
        "gps_antennas": os.environ.get("GPS_ANTENNAS"),
        "tachometer_mode": load_driver_param("tachometer_mode", "real"),
        "oak_mode": os.environ.get("OAK_MODE"),
        "enable_teleop": os.environ.get("ENABLE_TELEOP"),
        "enable_joystick": os.environ.get("ENABLE_JOYSTICK"),
        "mapping_deskew": os.environ.get("MAPPING_DESKEW"),
        "mapping_compression_voxel_size": os.environ.get("MAPPING_COMPRESSION_VOXEL_SIZE"),
        "mapping_map_publish_rate": os.environ.get("MAPPING_MAP_PUBLISH_RATE"),
        "mapping_map_tf_publish_rate": os.environ.get("MAPPING_MAP_TF_PUBLISH_RATE"),
        "mapping_points_topic": os.environ.get("MAPPING_POINTS_TOPIC"),
        "max_linear_speed_ms": load_driver_param("max_linear_speed_ms", None),
        "max_articulation_deg": load_driver_param("model_max_articulation_deg", None),
        "throttle_deadband": load_driver_param("throttle_deadband", None),
        "steer_deadband": load_driver_param("steer_deadband", None),
        "topics": topics,
        "topics_file": config.get("topics_file"),
        "topics_key": config.get("topics_key"),
        "qos_override": config.get("qos_override"),
    }
    write_text(record_dir / "metadata.yaml", yaml.safe_dump(metadata, sort_keys=False))


def maybe_copy_config(config_path: Path, record_dir: Path, enabled: bool) -> None:
    if not enabled:
        return
    target_dir = record_dir / "config"
    if target_dir.exists():
        shutil.rmtree(target_dir)
    shutil.copytree(config_path.parent, target_dir)


def maybe_save_remote_snapshot(workspace_root: Path, record_dir: Path, enabled: bool) -> None:
    if not enabled:
        return
    snapshot_script = workspace_root / "demos" / "live_robot" / "scripts" / "robot_snapshot.sh"
    if not snapshot_script.exists():
        return
    subprocess.run(
        [str(snapshot_script), str(record_dir / "robot"), os.environ.get("ROBOT_SSH_TARGET", "robot@192.168.2.2")],
        check=False,
    )


def finalize_recording(workspace_root: Path, record_dir: Path, bag_dir: Path) -> None:
    ensure_bag_metadata(bag_dir, record_dir / "runtime")
    mcap_files = sorted(bag_dir.glob("*.mcap"))
    if mcap_files:
        symlink_path = record_dir / "session.mcap"
        if symlink_path.exists() or symlink_path.is_symlink():
            symlink_path.unlink()
        symlink_path.symlink_to(mcap_files[0])
    save_runtime_graph_snapshot(record_dir / "runtime")
    save_runtime_artifacts(record_dir)
    run_capture(["ros2", "bag", "info", str(bag_dir)], record_dir / "ros2_bag_info.txt")
    run_capture(
        ["python3", str(workspace_root / "scripts" / "post_session_report.py"), str(record_dir)],
        record_dir / "post_session_report.stdout",
    )
    run_capture(
        ["python3", str(workspace_root / "scripts" / "audit_bag_topics.py"), str(record_dir), "--show-ok"],
        record_dir / "audit_bag_topics.txt",
    )


def main() -> int:
    args = parse_args()
    config_path = Path(args.config).resolve()
    workspace_root = Path(os.environ.get("WORKSPACE", infer_workspace_root(config_path))).resolve()
    config = load_config(config_path)

    demo_name = config.get("name", "live_robot")
    base_directory = Path(config.get("directory", workspace_root / "data" / "records"))
    if not base_directory.is_absolute():
        base_directory = workspace_root / base_directory

    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    run_name = timestamp
    if args.label:
        run_name = f"{run_name}_{args.label}"

    record_dir = base_directory / demo_name / run_name
    bag_dir = record_dir / "bag"

    topics = load_topics(config, config_path, workspace_root)
    if not topics:
        raise SystemExit("records.yaml does not define any topics and no valid topics_file/topics_key could be resolved")

    command = [
        "ros2", "bag", "record",
        "--node-name", "rosbag2_recorder",
        "--disable-keyboard-controls",
        "-s", config.get("storage", "mcap"),
        "-o", str(bag_dir),
    ]
    max_cache_size = config.get("max_cache_size")
    if max_cache_size:
        command.extend(["--max-cache-size", str(max_cache_size)])
    storage_config = config.get("storage_config")
    if storage_config:
        storage_config_path = Path(storage_config)
        if not storage_config_path.is_absolute():
            storage_config_path = workspace_root / storage_config_path
        command.extend(["--storage-config-file", str(storage_config_path)])
    qos_override = config.get("qos_override")
    if qos_override:
        qos_override_path = Path(qos_override)
        if not qos_override_path.is_absolute():
            qos_override_path = workspace_root / qos_override_path
        command.extend(["--qos-profile-overrides-path", str(qos_override_path)])
    command.extend(topics)

    if args.dry_run:
        print(f"Record directory: {record_dir}")
        print(f"Bag directory:    {bag_dir}")
        print("Bag command:")
        print("  " + " ".join(command))
        return 0

    record_dir.mkdir(parents=True, exist_ok=True)
    maybe_copy_config(config_path, record_dir, config.get("config", False))
    if config.get("vcs", False):
        save_vcs_snapshot(workspace_root, record_dir / "vcs")

    write_text(record_dir / "environment.env", "\n".join(f"{k}={v}" for k, v in sorted(os.environ.items())) + "\n")
    save_metadata(config, config_path, record_dir, bag_dir, topics)
    maybe_save_remote_snapshot(workspace_root, record_dir, config.get("remote_snapshot", False))
    write_text(record_dir / "record_topics.txt", "\n".join(topics) + "\n")

    print(f"Recording into {bag_dir}")
    bag_process = subprocess.Popen(command, cwd=workspace_root, start_new_session=True)
    stop_requested = False

    def _request_stop(signum, _frame) -> None:
        nonlocal stop_requested
        if stop_requested:
            return
        stop_requested = True
        if bag_process.poll() is None:
            call_rosbag_service(record_dir, "/rosbag2_recorder/pause", "rosbag2_interfaces/srv/Pause")
            time.sleep(1.0)
            bag_process.send_signal(signum)

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        try:
            return_code = bag_process.wait()
        except KeyboardInterrupt:
            _request_stop(signal.SIGINT, None)
            return_code = bag_process.wait(timeout=300)
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        finalize_recording(workspace_root, record_dir, bag_dir)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
