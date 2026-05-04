#!/usr/bin/env python3
"""Replay recorded bags offline, rebuild ICP odometry, and save map outputs."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import yaml


SAVE_MAP_SERVICE = "/mapping/save_map"
SAVE_TRAJECTORY_SERVICE = "/mapping/save_trajectory"
SAVE_MAP_TYPE = "norlab_icp_mapper_ros/srv/SaveMap"
SAVE_TRAJECTORY_TYPE = "norlab_icp_mapper_ros/srv/SaveTrajectory"

HESAI_TOPIC = "/hesai_lidar/points"
RSAIRY_TOPIC = "/rsairy_ns/points"
MERGED_TOPIC = "/merged_points_filtered"
MERGED_DEBUG_TOPIC = "/merged_points"

REBUILT_MAPPING_TOPICS = [
    "/mapping/icp_odom",
    "/mapping/map",
    "/mapping/scan_after_deskew",
    "/mapping/scan_after_input_filters",
    "/mapping/pose_in",
]

REBUILT_PERCEPTION_TOPICS = [
    MERGED_TOPIC,
    MERGED_DEBUG_TOPIC,
    "/trailer/angle",
]


@dataclass(frozen=True)
class PipelineChoice:
    mode: str
    points_topic: str
    launch_perception: bool
    reason: str


def bool_env(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def infer_workspace_root() -> Path:
    env_workspace = os.environ.get("WORKSPACE")
    if env_workspace:
        return Path(env_workspace).resolve()

    script_path = Path(__file__).resolve()
    for candidate in [script_path.parent, *script_path.parents]:
        if (candidate / "src").exists() and (candidate / "demos").exists():
            return candidate
    return script_path.parents[2]


def parse_args(workspace_root: Path) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay MTT sessions offline and save map.vtk + trajectory.vtk."
    )
    parser.add_argument(
        "input_path",
        nargs="?",
        default=os.environ.get("BAG_PATH") or os.environ.get("DATA_DIR") or str(workspace_root / "data"),
        help="Session dir, bag dir, .mcap file, or a data directory containing many sessions.",
    )
    parser.add_argument(
        "--mode",
        default=os.environ.get("OFFLINE_ICP_MODE", "auto"),
        choices=["auto", "fused", "hesai"],
        help="Mapping input policy. auto = fused if possible, otherwise fallback.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        default=bool_env("OFFLINE_ICP_FORCE", False),
        help="Rebuild even if map.vtk and trajectory.vtk already exist.",
    )
    parser.add_argument(
        "--replay-rate",
        type=float,
        default=float(os.environ.get("REPLAY_RATE", "1.0")),
        help="Replay rate passed to ros2 bag play.",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=float(os.environ.get("OFFLINE_ICP_SETTLE_S", "4.0")),
        help="Extra wall-clock delay after bag playback before saving outputs.",
    )
    parser.add_argument(
        "--ready-timeout-seconds",
        type=float,
        default=float(os.environ.get("OFFLINE_ICP_READY_TIMEOUT_S", "30.0")),
        help="How long to wait for mapper services before starting bag playback.",
    )
    parser.add_argument(
        "--play-timeout-margin-seconds",
        type=float,
        default=float(os.environ.get("OFFLINE_ICP_PLAY_TIMEOUT_MARGIN_S", "90.0")),
        help="Additional timeout margin added on top of bag duration / replay rate.",
    )
    return parser.parse_args()


def parse_bag_metadata(metadata_path: Path) -> tuple[dict[str, int], float]:
    data = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    info = data.get("rosbag2_bagfile_information", data)
    duration_s = float(info.get("duration", {}).get("nanoseconds", 0)) / 1e9
    counts = {
        entry["topic_metadata"]["name"]: int(entry["message_count"])
        for entry in info.get("topics_with_message_count", [])
    }
    return counts, duration_s


def resolve_session_dir(path_value: str) -> list[Path]:
    path = Path(path_value).resolve()

    if path.is_file():
        if path.name.endswith(".mcap") and path.parent.name == "bag":
            return [path.parent.parent]
        if path.name.endswith(".mcap"):
            return [path.parent]
        raise ValueError(f"Unsupported file input: {path}")

    if (path / "bag" / "metadata.yaml").exists():
        return [path]

    if (path / "metadata.yaml").exists():
        return [path.parent]

    candidates = sorted(
        child for child in path.iterdir()
        if child.is_dir() and child.name.startswith("mtt_")
    )
    if candidates:
        return candidates

    raise ValueError(f"Could not resolve any session from {path}")


def choose_pipeline(counts: dict[str, int], requested_mode: str) -> PipelineChoice:
    has_hesai = counts.get(HESAI_TOPIC, 0) > 0
    has_rsairy = counts.get(RSAIRY_TOPIC, 0) > 0
    has_merged = counts.get(MERGED_TOPIC, 0) > 0

    if requested_mode == "hesai":
        if not has_hesai:
            raise RuntimeError("Hesai point cloud is not available in this session.")
        return PipelineChoice("hesai", HESAI_TOPIC, False, "Hesai raw cloud available")

    if requested_mode == "fused":
        if has_hesai and has_rsairy:
            return PipelineChoice("fused", MERGED_TOPIC, True, "Both raw LiDAR clouds available")
        if has_merged:
            return PipelineChoice("merged_recorded", MERGED_TOPIC, False, "Recorded merged cloud available")
        raise RuntimeError("Requested fused mapping but the session does not contain both raw LiDAR clouds.")

    if has_hesai and has_rsairy:
        return PipelineChoice("fused", MERGED_TOPIC, True, "Both raw LiDAR clouds available")
    if has_merged:
        return PipelineChoice("merged_recorded", MERGED_TOPIC, False, "Recorded merged cloud available")
    if has_hesai:
        return PipelineChoice("hesai", HESAI_TOPIC, False, "Falling back to Hesai only")
    raise RuntimeError("No usable LiDAR point cloud is available for ICP mapping.")


def candidate_pipelines(counts: dict[str, int], requested_mode: str) -> list[PipelineChoice]:
    primary = choose_pipeline(counts, requested_mode)
    candidates = [primary]

    has_hesai = counts.get(HESAI_TOPIC, 0) > 0
    has_merged = counts.get(MERGED_TOPIC, 0) > 0

    if requested_mode != "auto":
        return candidates

    if primary.mode == "fused" and has_merged:
        candidates.append(PipelineChoice("merged_recorded", MERGED_TOPIC, False, "Fallback to recorded merged cloud"))
    if has_hesai and primary.mode != "hesai":
        candidates.append(PipelineChoice("hesai", HESAI_TOPIC, False, "Fallback to Hesai only"))

    deduped: list[PipelineChoice] = []
    seen: set[tuple[str, str, bool]] = set()
    for pipeline in candidates:
        key = (pipeline.mode, pipeline.points_topic, pipeline.launch_perception)
        if key not in seen:
            seen.add(key)
            deduped.append(pipeline)
    return deduped


def start_process(command: list[str], log_path: Path) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8")
    return subprocess.Popen(
        command,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def terminate_process(process: subprocess.Popen | None, grace_s: float = 10.0) -> None:
    if process is None or process.poll() is not None:
        return

    try:
        process.send_signal(signal.SIGINT)
    except ProcessLookupError:
        return

    deadline = time.time() + grace_s
    while time.time() < deadline:
        if process.poll() is not None:
            return
        time.sleep(0.2)

    try:
        process.send_signal(signal.SIGTERM)
    except ProcessLookupError:
        return

    deadline = time.time() + 5.0
    while time.time() < deadline:
        if process.poll() is not None:
            return
        time.sleep(0.2)

    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return


def run_capture(command: list[str], output_path: Path, timeout_s: float | None = None) -> subprocess.CompletedProcess:
    result = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(result.stdout, encoding="utf-8")
    if result.stderr:
        output_path.with_suffix(output_path.suffix + ".stderr").write_text(result.stderr, encoding="utf-8")
    return result


def wait_for_service(service_name: str, timeout_s: float, log_dir: Path) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        result = subprocess.run(
            ["ros2", "service", "list"],
            check=False,
            capture_output=True,
            text=True,
        )
        (log_dir / "service_list.txt").write_text(result.stdout, encoding="utf-8")
        if result.returncode == 0 and service_name in result.stdout.splitlines():
            return True
        time.sleep(1.0)
    return False


def call_save_service(service_name: str, service_type: str, request: str, log_path: Path) -> bool:
    result = run_capture(
        ["ros2", "service", "call", service_name, service_type, request],
        log_path,
        timeout_s=30.0,
    )
    return result.returncode == 0


def excluded_topics_for_pipeline(pipeline: PipelineChoice) -> list[str]:
    topics = list(REBUILT_MAPPING_TOPICS)
    if pipeline.launch_perception:
        topics.extend(REBUILT_PERCEPTION_TOPICS)
    return topics


def write_summary(output_dir: Path, result: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.yaml").write_text(
        yaml.safe_dump(result, sort_keys=False),
        encoding="utf-8",
    )


def run_pipeline(
    session_dir: Path,
    output_dir: Path,
    bag_dir: Path,
    duration_s: float,
    pipeline: PipelineChoice,
    args: argparse.Namespace,
    workspace_root: Path,
    base_result: dict,
) -> dict:
    output_dir = session_dir / "offline_icp"
    log_dir = output_dir / "logs"
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    map_file = session_dir / "map.vtk"
    trajectory_file = session_dir / "trajectory.vtk"
    result = dict(base_result)
    result["mode"] = pipeline.mode
    result["points_topic"] = pipeline.points_topic
    result["launch_perception"] = pipeline.launch_perception
    result["reason"] = pipeline.reason
    result["bag_duration_seconds"] = duration_s

    description_proc = None
    perception_proc = None
    mapping_proc = None
    bag_proc = None

    qos_override = workspace_root / "src" / "external" / "norlab_robot" / "config" / "rosbag_record" / "qos_replay_override.yaml"
    play_timeout = max(60.0, duration_s / max(args.replay_rate, 1e-6) + args.play_timeout_margin_seconds)

    try:
        description_proc = start_process(
            [
                "ros2", "launch", "norlab_robot", "live_robot.launch.py",
                "enable_description:=true",
                "enable_sensors:=false",
                "enable_mapping:=false",
                "enable_perception:=false",
                "enable_localization:=false",
                "setup_real_can:=false",
                "use_sim_time:=true",
            ],
            log_dir / f"description_{pipeline.mode}.log",
        )

        if pipeline.launch_perception:
            perception_proc = start_process(
                ["ros2", "launch", "mtt_perception", "perception.launch.py", "use_sim_time:=true"],
                log_dir / f"perception_{pipeline.mode}.log",
            )

        mapping_proc = start_process(
            [
                "ros2", "launch", "norlab_robot", "mapping.launch.py",
                "use_sim_time:=true",
                f"mapping_points_topic:={pipeline.points_topic}",
            ],
            log_dir / f"mapping_{pipeline.mode}.log",
        )

        pipeline_log_dir = log_dir / pipeline.mode
        pipeline_log_dir.mkdir(parents=True, exist_ok=True)

        if not wait_for_service(SAVE_MAP_SERVICE, args.ready_timeout_seconds, pipeline_log_dir):
            raise RuntimeError("Mapper services did not appear before timeout.")

        bag_proc = start_process(
            [
                "ros2", "bag", "play",
                "--input", str(bag_dir), "mcap",
                "--clock",
                "--rate", str(args.replay_rate),
                "--disable-keyboard-controls",
                "--qos-profile-overrides-path", str(qos_override),
            ]
            + (["--exclude-topics"] + excluded_topics_for_pipeline(pipeline)),
            log_dir / f"bag_play_{pipeline.mode}.log",
        )

        try:
            bag_returncode = bag_proc.wait(timeout=play_timeout)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"Bag playback timed out after {play_timeout:.1f}s") from exc

        if bag_returncode != 0:
            raise RuntimeError(f"ros2 bag play exited with code {bag_returncode}")

        time.sleep(args.settle_seconds)

        map_request = f'{{map_file_name: {{data: "{map_file}"}}}}'
        trajectory_request = f'{{trajectory_file_name: {{data: "{trajectory_file}"}}}}'

        if not call_save_service(
            SAVE_MAP_SERVICE,
            SAVE_MAP_TYPE,
            map_request,
            log_dir / f"save_map_{pipeline.mode}.txt",
        ):
            raise RuntimeError("Failed to save map via /mapping/save_map")

        if not call_save_service(
            SAVE_TRAJECTORY_SERVICE,
            SAVE_TRAJECTORY_TYPE,
            trajectory_request,
            log_dir / f"save_trajectory_{pipeline.mode}.txt",
        ):
            raise RuntimeError("Failed to save trajectory via /mapping/save_trajectory")

        if not map_file.exists() or not trajectory_file.exists():
            raise RuntimeError("Expected map.vtk and trajectory.vtk were not created.")

        result["status"] = "ok"
        result["map_size_bytes"] = map_file.stat().st_size
        result["trajectory_size_bytes"] = trajectory_file.stat().st_size
        return result

    finally:
        terminate_process(bag_proc)
        terminate_process(mapping_proc)
        terminate_process(perception_proc)
        terminate_process(description_proc)
        write_summary(output_dir, result)


def process_session(session_dir: Path, args: argparse.Namespace, workspace_root: Path) -> dict:
    bag_dir = session_dir / "bag"
    metadata_path = bag_dir / "metadata.yaml"
    output_dir = session_dir / "offline_icp"
    output_dir.mkdir(parents=True, exist_ok=True)

    result: dict[str, object] = {
        "session": session_dir.name,
        "session_dir": str(session_dir),
        "status": "failed",
        "mode": None,
        "points_topic": None,
        "launch_perception": None,
        "reason": None,
        "map_file": str(session_dir / "map.vtk"),
        "trajectory_file": str(session_dir / "trajectory.vtk"),
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }

    if not metadata_path.exists():
        result["status"] = "skipped_missing_metadata"
        write_summary(output_dir, result)
        return result

    map_file = session_dir / "map.vtk"
    trajectory_file = session_dir / "trajectory.vtk"
    if not args.force and map_file.exists() and trajectory_file.exists():
        result["status"] = "skipped_existing"
        write_summary(output_dir, result)
        return result

    counts, duration_s = parse_bag_metadata(metadata_path)
    pipelines = candidate_pipelines(counts, args.mode)

    errors: list[str] = []
    for index, pipeline in enumerate(pipelines, start=1):
        attempt_result = dict(result)
        attempt_result["attempt"] = index
        attempt_result["attempt_count"] = len(pipelines)
        if index > 1:
            attempt_result["fallback_from"] = pipelines[0].mode
        try:
            return run_pipeline(
                session_dir=session_dir,
                output_dir=output_dir,
                bag_dir=bag_dir,
                duration_s=duration_s,
                pipeline=pipeline,
                args=args,
                workspace_root=workspace_root,
                base_result=attempt_result,
            )
        except Exception as exc:  # noqa: BLE001
            errors.append(f"{pipeline.mode}: {exc}")

    result["error"] = " | ".join(errors) if errors else "No pipeline candidate succeeded"
    write_summary(output_dir, result)
    return result


def main() -> int:
    workspace_root = infer_workspace_root()
    args = parse_args(workspace_root)

    try:
        sessions = resolve_session_dir(args.input_path)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(f"Offline ICP input: {args.input_path}")
    print(f"Sessions found: {len(sessions)}")
    print(f"Mode: {args.mode}")
    print(f"Replay rate: {args.replay_rate}x")
    print("")

    results = []
    failures = 0

    for index, session_dir in enumerate(sessions, start=1):
        print(f"[{index}/{len(sessions)}] {session_dir.name}")
        try:
            result = process_session(session_dir, args, workspace_root)
        except Exception as exc:  # noqa: BLE001
            result = {
                "session": session_dir.name,
                "session_dir": str(session_dir),
                "status": "failed",
                "error": str(exc),
                "timestamp": datetime.now().isoformat(timespec="seconds"),
            }
            (session_dir / "offline_icp").mkdir(parents=True, exist_ok=True)
            (session_dir / "offline_icp" / "summary.yaml").write_text(
                yaml.safe_dump(result, sort_keys=False),
                encoding="utf-8",
            )

        results.append(result)
        status = result["status"]
        if status == "ok":
            print(f"  OK      {result['mode']} -> {session_dir / 'map.vtk'}")
        elif status.startswith("skipped"):
            print(f"  SKIPPED {status}")
        else:
            failures += 1
            print(f"  FAILED  {result.get('error', status)}")
        print("")

    report_path = workspace_root / "data" / f"offline_icp_report_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.yaml"
    report_path.write_text(yaml.safe_dump(results, sort_keys=False), encoding="utf-8")

    ok_count = sum(1 for item in results if item["status"] == "ok")
    skipped_count = sum(1 for item in results if str(item["status"]).startswith("skipped"))
    print("Offline ICP summary")
    print(f"  OK:       {ok_count}")
    print(f"  Skipped:  {skipped_count}")
    print(f"  Failed:   {failures}")
    print(f"  Report:   {report_path}")

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
