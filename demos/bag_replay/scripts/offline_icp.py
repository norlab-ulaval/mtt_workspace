#!/usr/bin/env python3
"""Replay recorded bags offline, rebuild ICP odometry, and save map outputs."""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


SAVE_MAP_SERVICE = "/mapping/save_map"
SAVE_TRAJECTORY_SERVICE = "/mapping/save_trajectory"
SAVE_MAP_TYPE = "norlab_icp_mapper_ros/srv/SaveMap"
SAVE_TRAJECTORY_TYPE = "norlab_icp_mapper_ros/srv/SaveTrajectory"

HESAI_TOPIC = "/hesai_lidar/points"
RSAIRY_TOPIC = "/rsairy_ns/points"
MERGED_TOPIC = "/merged_points_filtered"
MERGED_DEBUG_TOPIC = "/merged_points"
MERGED_RELIABLE_TOPIC = "/merged_points_reliable"

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
    MERGED_RELIABLE_TOPIC,
    "/trailer/angle",
    "/trailer/articulation_angle",
    "/trailer/pose",
    "/trailer/pose_confidence",
    "/trailer/body_markers",
    "/trailer/articulation_axis_marker",
    "/trailer/trailer_roi_cloud",
    "/trailer/articulation_roi_cloud",
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
        choices=["auto", "hesai_imu", "fused", "hesai"],
        help=(
            "Mapping input policy. auto = Hesai + IMU prior first, then documented "
            "fallbacks. hesai = raw Hesai with replayed TF/odom for comparison only."
        ),
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
        "--offline-quality",
        "--quality",
        dest="offline_quality",
        default=os.environ.get("OFFLINE_ICP_QUALITY", "standard"),
        choices=["standard", "max"],
        help="Offline mapper quality profile. max uses denser map compression and slower replay by default.",
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
    parser.add_argument(
        "--mapping-config",
        default=os.environ.get("OFFLINE_ICP_MAPPING_CONFIG", ""),
        help="Override libpointmatcher mapper YAML. Empty = profile default.",
    )
    parser.add_argument(
        "--filter-trailer",
        action=argparse.BooleanOptionalAction,
        default=bool_env("OFFLINE_ICP_FILTER_TRAILER", True),
        help="Remove trailer body points from /merged_points_filtered when replay builds fused clouds.",
    )
    parser.add_argument(
        "--mapping-odom-frame",
        default=os.environ.get("OFFLINE_ICP_MAPPING_ODOM_FRAME", "odom"),
        help="TF frame used as mapper odom prior. Default matches norlab_robot mapping launch.",
    )
    parser.add_argument(
        "--imu-frame",
        default=os.environ.get("OFFLINE_ICP_IMU_FRAME", "imu_link"),
        help=(
            "IMU frame used by imu_odom when rebuilding the odom prior. "
            "MTT bags publish /mti100/data with frame_id=imu_link."
        ),
    )
    parser.add_argument(
        "--imu-topic",
        default=os.environ.get("OFFLINE_ICP_IMU_TOPIC", "/mti100/data"),
        help="IMU topic used by the hesai_imu offline ICP profile.",
    )
    parser.add_argument(
        "--fused-points-topic",
        default=os.environ.get("OFFLINE_ICP_FUSED_POINTS_TOPIC", MERGED_RELIABLE_TOPIC),
        choices=[MERGED_TOPIC, MERGED_DEBUG_TOPIC, MERGED_RELIABLE_TOPIC],
        help=(
            "Point cloud consumed by fused mode. /merged_points_reliable is raw "
            "fused cloud with reliable QoS for the mapper; /merged_points is "
            "best-effort debug; /merged_points_filtered can be cloud-merger filtered."
        ),
    )
    parser.add_argument(
        "--fused-filter-owner",
        default=os.environ.get("OFFLINE_ICP_FUSED_FILTER_OWNER", "mapper"),
        choices=["mapper", "cloud_merger"],
        help=(
            "Where fused mode applies chassis/cage/trailer bbox filtering. "
            "mapper = raw reliable fused cloud with cloud_merger bbox disabled; "
            "cloud_merger = cloud_merger publishes already-filtered clouds."
        ),
    )
    parser.add_argument(
        "--fused-hesai-stride",
        type=int,
        default=int(os.environ.get("OFFLINE_ICP_FUSED_HESAI_STRIDE", "1")),
        help="Fused mode only: keep one out of N Hesai points before merging.",
    )
    parser.add_argument(
        "--fused-rsairy-stride",
        type=int,
        default=int(os.environ.get("OFFLINE_ICP_FUSED_RSAIRY_STRIDE", "1")),
        help=(
            "Fused mode only: keep one out of N RS-Airy points before merging. "
            "Use together with --fused-rsairy-inject-every-n for rear-lidar downweighting."
        ),
    )
    parser.add_argument(
        "--fused-rsairy-inject-every-n",
        type=int,
        default=int(os.environ.get("OFFLINE_ICP_FUSED_RSAIRY_INJECT_EVERY_N", "10")),
        help=(
            "Fused mode only: publish Hesai at every frame and inject RS-Airy "
            "only once every N Hesai frames. Default keeps ICP near 20 Hz while "
            "adding rear details occasionally."
        ),
    )
    parser.add_argument(
        "--experiment-name",
        default=os.environ.get("OFFLINE_ICP_EXPERIMENT_NAME", ""),
        help="Write outputs under offline_icp_runs/<name> and do not update compatibility map files.",
    )
    parser.add_argument(
        "--playback-policy",
        default=os.environ.get("OFFLINE_ICP_PLAYBACK_POLICY", "auto"),
        choices=["auto", "step", "bag_play"],
        help=(
            "How to feed the mapper. step publishes one cloud and waits for ICP odom "
            "before continuing; bag_play uses ros2 bag play. auto uses step for direct "
            "single-cloud pipelines and bag_play for perception/fused pipelines."
        ),
    )
    parser.add_argument(
        "--step-icp-timeout-s",
        type=float,
        default=float(os.environ.get("OFFLINE_ICP_STEP_TIMEOUT_S", "120.0")),
        help="Per-cloud ICP timeout used by --playback-policy step.",
    )
    parser.add_argument(
        "--step-max-consecutive-timeouts",
        type=int,
        default=int(os.environ.get("OFFLINE_ICP_STEP_MAX_CONSECUTIVE_TIMEOUTS", "20")),
        help="Abort step replay after this many consecutive cloud/ICP timeouts.",
    )
    parser.add_argument(
        "--step-cloud-stride",
        type=int,
        default=int(os.environ.get("OFFLINE_ICP_CLOUD_STRIDE", "1")),
        help=(
            "Step replay only: publish one mapping cloud out of N while still replaying "
            "support topics. Use 4 for about 5 Hz from a 20 Hz Hesai bag."
        ),
    )
    parser.add_argument(
        "--checkpoint-interval-s",
        type=float,
        default=float(os.environ.get("OFFLINE_ICP_CHECKPOINT_INTERVAL_S", "600.0")),
        help=(
            "Save map_checkpoint_latest.vtk and trajectory_checkpoint_latest.vtk "
            "periodically while the mapper is alive. Set <=0 to disable."
        ),
    )
    parser.add_argument(
        "--enable-global-output-map",
        action=argparse.BooleanOptionalAction,
        default=bool_env("OFFLINE_ICP_ENABLE_GLOBAL_OUTPUT_MAP", True),
        help=(
            "Keep an untrimmed global output map for final map.vtk. Disable for fast "
            "motion-model runs where only /mapping/icp_odom is required."
        ),
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

    if requested_mode == "hesai_imu":
        if not has_hesai:
            raise RuntimeError("Hesai point cloud is not available in this session.")
        return PipelineChoice(
            "hesai_imu",
            HESAI_TOPIC,
            False,
            "Hesai raw cloud with MTi-100 IMU odom prior and replay TF excluded",
        )

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

    if has_hesai:
        return PipelineChoice(
            "hesai_imu",
            HESAI_TOPIC,
            False,
            "Default baseline: Hesai raw cloud with MTi-100 IMU odom prior",
        )
    if has_merged:
        return PipelineChoice("merged_recorded", MERGED_TOPIC, False, "Recorded merged cloud available")
    raise RuntimeError("No usable LiDAR point cloud is available for ICP mapping.")


def candidate_pipelines(counts: dict[str, int], requested_mode: str) -> list[PipelineChoice]:
    primary = choose_pipeline(counts, requested_mode)
    candidates = [primary]

    has_hesai = counts.get(HESAI_TOPIC, 0) > 0
    has_rsairy = counts.get(RSAIRY_TOPIC, 0) > 0
    has_merged = counts.get(MERGED_TOPIC, 0) > 0

    if requested_mode != "auto":
        return candidates

    if has_merged and primary.mode != "merged_recorded":
        candidates.append(PipelineChoice("merged_recorded", MERGED_TOPIC, False, "Fallback to recorded merged cloud"))
    if has_hesai and has_rsairy and primary.mode != "fused":
        candidates.append(PipelineChoice("fused", MERGED_TOPIC, True, "Fallback to rebuilt fused cloud"))
    if has_hesai and primary.mode not in {"hesai", "hesai_imu"}:
        candidates.append(PipelineChoice("hesai", HESAI_TOPIC, False, "Fallback to Hesai with replayed TF"))

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
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return

    deadline = time.time() + grace_s
    while time.time() < deadline:
        if process.poll() is not None:
            return
        try:
            time.sleep(0.2)
        except KeyboardInterrupt:
            # Finish cleanup before propagating interruption. Leaving ros2 bag
            # record alive creates stale process false positives on the next run.
            continue

    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return

    deadline = time.time() + 5.0
    while time.time() < deadline:
        if process.poll() is not None:
            return
        try:
            time.sleep(0.2)
        except KeyboardInterrupt:
            continue

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


def call_save_service(
    service_name: str,
    service_type: str,
    request: str,
    log_path: Path,
    timeout_s: float = 300.0,
) -> bool:
    result = run_capture(
        ["ros2", "service", "call", service_name, service_type, request],
        log_path,
        timeout_s=timeout_s,
    )
    return result.returncode == 0


def save_mapping_outputs(
    map_file: Path,
    trajectory_file: Path,
    log_dir: Path,
    pipeline_mode: str,
    duration_s: float,
    *,
    label: str = "",
) -> tuple[bool, list[str]]:
    """Save mapper outputs while the mapper is still alive.

    Step replay can legitimately finish with long rejection bursts: the mapper
    processed clouds and built a map, but no /mapping/icp_odom was published for
    rejected scans.  In that case we still want map.vtk/trajectory.vtk for audit
    instead of losing hours of CPU work.
    """
    suffix = f"_{label}" if label else ""
    map_request = f'{{map_file_name: {{data: "{map_file}"}}}}'
    trajectory_request = f'{{trajectory_file_name: {{data: "{trajectory_file}"}}}}'
    save_timeout_s = max(120.0, 60.0 + duration_s)
    errors: list[str] = []

    if not call_save_service(
        SAVE_MAP_SERVICE,
        SAVE_MAP_TYPE,
        map_request,
        log_dir / f"save_map_{pipeline_mode}{suffix}.txt",
        timeout_s=save_timeout_s,
    ):
        errors.append("save_map_service_failed")

    if not call_save_service(
        SAVE_TRAJECTORY_SERVICE,
        SAVE_TRAJECTORY_TYPE,
        trajectory_request,
        log_dir / f"save_trajectory_{pipeline_mode}{suffix}.txt",
        timeout_s=60.0,
    ):
        errors.append("save_trajectory_service_failed")

    if not map_file.exists():
        errors.append("map_file_missing_after_save")
    if not trajectory_file.exists():
        errors.append("trajectory_file_missing_after_save")

    return not errors, errors


def excluded_topics_for_pipeline(pipeline: PipelineChoice) -> list[str]:
    topics = list(REBUILT_MAPPING_TOPICS)
    if pipeline.launch_perception:
        topics.extend(REBUILT_PERCEPTION_TOPICS)
    if pipeline.mode in {"hesai_imu", "fused", "hesai"}:
        topics.append("/tf")
    return topics


def playback_policy_for_pipeline(args: argparse.Namespace, pipeline: PipelineChoice) -> str:
    if args.playback_policy != "auto":
        return args.playback_policy
    if pipeline.launch_perception:
        return "bag_play"
    if pipeline.points_topic in {HESAI_TOPIC, RSAIRY_TOPIC, MERGED_TOPIC}:
        return "step"
    return "bag_play"


def validate_icp_odom_replay(bag_dir: Path, expected_duration_s: float) -> dict[str, Any]:
    """Read the recorded icp_odom_replay bag and check quality.

    Checks:
    - message count vs expected ICP rate (~2-10 Hz for a moving robot)
    - temporal coverage vs bag duration
    - max timestamp gap (ICP stall / mapper crash)
    - max 2-D pose jump (divergence detection — BoundTransformationChecker should cap at 2 m)
    """
    result: dict[str, Any] = {
        "status": "not_checked",
        "message_count": 0,
        "coverage_s": 0.0,
        "coverage_ratio": 0.0,
        "max_timestamp_gap_s": 0.0,
        "max_pose_jump_m": 0.0,
        "warnings": [],
    }

    mcap = next(bag_dir.glob("*.mcap"), None) if bag_dir.exists() else None
    if not mcap or mcap.stat().st_size < 512:
        result["status"] = "empty_or_missing"
        return result

    try:
        import rosbag2_py  # type: ignore[import]
        from rclpy.serialization import deserialize_message  # type: ignore[import]
        from rosidl_runtime_py.utilities import get_message  # type: ignore[import]
    except ImportError:
        result["status"] = "rosbag2_py_unavailable"
        result["message_count"] = -1
        return result

    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        reader.set_filter(rosbag2_py.StorageFilter(topics=["/mapping/icp_odom"]))
        msg_type = get_message("nav_msgs/msg/Odometry")

        poses: list[tuple[float, float, float]] = []  # (t_s, x, y)
        prev_t: float | None = None
        max_gap = 0.0

        while reader.has_next():
            _, data, ts_ns = reader.read_next()
            msg = deserialize_message(data, msg_type)
            t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            if prev_t is not None and t > prev_t:
                max_gap = max(max_gap, t - prev_t)
            prev_t = t
            poses.append((t, x, y))

    except Exception as exc:
        result["status"] = f"read_error: {exc}"
        return result

    if not poses:
        result["status"] = "empty"
        return result

    count = len(poses)
    coverage = poses[-1][0] - poses[0][0]
    coverage_ratio = coverage / max(expected_duration_s, 1.0)

    max_jump = 0.0
    for i in range(1, count):
        dx = poses[i][1] - poses[i - 1][1]
        dy = poses[i][2] - poses[i - 1][2]
        max_jump = max(max_jump, math.sqrt(dx * dx + dy * dy))

    warnings: list[str] = []
    # Coverage below 60 %: ICP stalled or crashed for most of the session
    if coverage_ratio < 0.60:
        warnings.append(f"low_coverage:{coverage_ratio:.0%}_of_{expected_duration_s:.0f}s")
    # Gap > 10 s: ICP was blocked or mapper restarted
    if max_gap > 10.0:
        warnings.append(f"timestamp_gap:{max_gap:.1f}s")
    # Pose jump > 1.5 m: BoundTransformationChecker set to 2 m, so >1.5 m is suspicious
    if max_jump > 1.5:
        warnings.append(f"pose_jump:{max_jump:.2f}m")
    # Fewer than 10 messages: essentially empty
    if count < 10:
        warnings.append(f"too_few_messages:{count}")

    result.update({
        "status": "ok" if not warnings else "ok_with_warnings",
        "message_count": count,
        "coverage_s": round(coverage, 2),
        "coverage_ratio": round(coverage_ratio, 3),
        "max_timestamp_gap_s": round(max_gap, 3),
        "max_pose_jump_m": round(max_jump, 4),
        "warnings": warnings,
    })
    return result


class RecordWatchdog:
    """Background thread: monitors record_proc during bag play, logs if it dies early."""

    def __init__(self, proc: subprocess.Popen, log_path: Path, bag_duration_s: float):
        self._proc = proc
        self._log_path = log_path
        self._bag_duration_s = bag_duration_s
        self._died_at: float | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        start = time.time()
        while not self._stop.is_set():
            if self._proc.poll() is not None:
                elapsed = time.time() - start
                self._died_at = elapsed
                msg = (
                    f"[watchdog] icp_odom recorder exited after {elapsed:.1f}s "
                    f"(expected ~{self._bag_duration_s:.0f}s). "
                    f"returncode={self._proc.returncode}\n"
                )
                try:
                    with self._log_path.open("a") as f:
                        f.write(msg)
                except OSError:
                    pass
                return
            time.sleep(2.0)

    def stop(self) -> float | None:
        """Stop the watchdog and return elapsed time when recorder died (None = still alive)."""
        self._stop.set()
        self._thread.join(timeout=5.0)
        return self._died_at


class MappingCheckpointSaver:
    """Periodically snapshots mapper outputs without stopping the offline run."""

    def __init__(
        self,
        map_file: Path,
        trajectory_file: Path,
        log_dir: Path,
        pipeline_mode: str,
        interval_s: float,
    ):
        self._map_file = map_file
        self._trajectory_file = trajectory_file
        self._log_dir = log_dir
        self._pipeline_mode = pipeline_mode
        self._interval_s = interval_s
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        checkpoint_log = self._log_dir / f"checkpoint_{self._pipeline_mode}.log"
        checkpoint_log.parent.mkdir(parents=True, exist_ok=True)
        while not self._stop.wait(self._interval_s):
            started = datetime.now().isoformat(timespec="seconds")
            try:
                saved, errors = save_mapping_outputs(
                    self._map_file,
                    self._trajectory_file,
                    self._log_dir,
                    self._pipeline_mode,
                    duration_s=60.0,
                    label="checkpoint_latest",
                )
                status = "saved" if saved else "failed:" + ",".join(errors)
            except Exception as exc:  # noqa: BLE001
                status = f"exception:{exc}"
            try:
                with checkpoint_log.open("a", encoding="utf-8") as f:
                    f.write(f"{started} {status}\n")
            except OSError:
                pass

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=30.0)


def write_summary(output_dir: Path, result: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.yaml").write_text(
        yaml.safe_dump(result, sort_keys=False),
        encoding="utf-8",
    )


def parse_cloud_merger_stats(log_path: Path) -> dict[str, Any]:
    stats: dict[str, Any] = {
        "status": "missing",
        "published_count": 0,
        "no_pair_drops": 0,
        "tf_drops": 0,
        "reuse_count": 0,
        "avg_pair_dt_s": None,
        "max_pair_dt_s": None,
        "last_hesai_points": None,
        "last_rsairy_points": None,
        "last_merged_points": None,
    }
    if not log_path.exists():
        return stats

    pattern = re.compile(
        r"H=(?P<hesai>\d+)\s+R=(?P<rsairy>\d+)\s+merged=(?P<merged>\d+).*"
        r"pub=(?P<pub>\d+)\s+no_pair=(?P<no_pair>\d+)\s+tf_drop=(?P<tf_drop>\d+)"
        r"\s+reuse=(?P<reuse>\d+)\s+avg_dt=(?P<avg>[0-9.]+)s\s+max_dt=(?P<max>[0-9.]+)s"
    )
    for line in log_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        match = pattern.search(line)
        if not match:
            continue
        stats.update({
            "status": "ok",
            "published_count": int(match.group("pub")),
            "no_pair_drops": int(match.group("no_pair")),
            "tf_drops": int(match.group("tf_drop")),
            "reuse_count": int(match.group("reuse")),
            "avg_pair_dt_s": float(match.group("avg")),
            "max_pair_dt_s": float(match.group("max")),
            "last_hesai_points": int(match.group("hesai")),
            "last_rsairy_points": int(match.group("rsairy")),
            "last_merged_points": int(match.group("merged")),
        })
    if stats["status"] == "missing":
        stats["status"] = "no_stats_found"
    return stats


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
    attempt = int(base_result.get("attempt", 1))
    log_dir = output_dir / "logs" / f"attempt_{attempt}_{pipeline.mode}"
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    map_file = output_dir / "map.vtk"
    trajectory_file = output_dir / "trajectory.vtk"
    compat_map_file = session_dir / "map.vtk"
    compat_trajectory_file = session_dir / "trajectory.vtk"
    result = dict(base_result)
    result["mode"] = pipeline.mode
    points_topic = args.fused_points_topic if pipeline.mode == "fused" else pipeline.points_topic
    result["points_topic"] = points_topic
    result["launch_perception"] = pipeline.launch_perception
    result["reason"] = pipeline.reason
    result["fused_filter_owner"] = args.fused_filter_owner if pipeline.mode == "fused" else None
    result["bag_duration_seconds"] = duration_s
    result["offline_quality"] = args.offline_quality
    result["map_file"] = str(map_file)
    result["trajectory_file"] = str(trajectory_file)
    result["output_dir"] = str(output_dir)
    result["experiment_name"] = args.experiment_name or None
    if not args.experiment_name:
        result["compat_map_file"] = str(compat_map_file)
        result["compat_trajectory_file"] = str(compat_trajectory_file)

    print(
        f"  RUN     mode={pipeline.mode} points={pipeline.points_topic} "
        f"duration={duration_s:.1f}s output={output_dir}",
        flush=True,
    )

    description_proc = None
    runtime_odom_proc = None
    perception_proc = None
    imu_odom_proc = None
    mapping_proc = None
    bag_proc = None
    record_proc = None
    record_watchdog = None
    checkpoint_saver = None

    qos_override = workspace_root / "src" / "external" / "norlab_robot" / "config" / "rosbag_record" / "qos_replay_override.yaml"
    standard_mapping_config = workspace_root / "src" / "external" / "norlab_robot" / "config" / "mapping" / "_config.yaml"
    dense_mapping_config = workspace_root / "src" / "external" / "norlab_robot" / "config" / "mapping" / "_config_replay_dense.yaml"
    hesai_wheel_mapping_config = workspace_root / "src" / "external" / "norlab_robot" / "config" / "mapping" / "_config_hesai_wheel_replay.yaml"
    hesai_imu_mapping_config = workspace_root / "src" / "external" / "norlab_robot" / "config" / "mapping" / "_config_hesai_imu_replay.yaml"
    fused_imu_mapping_config = workspace_root / "src" / "external" / "norlab_robot" / "config" / "mapping" / "_config_fused_imu_replay.yaml"
    fused_simple_mapping_config = workspace_root / "src" / "external" / "norlab_robot" / "config" / "mapping" / "_config_fused_simple_replay.yaml"
    replay_rate = args.replay_rate
    mapping_compression_voxel_size = "0.20"
    mapping_config = Path(args.mapping_config).expanduser() if args.mapping_config else standard_mapping_config
    mapping_robot_frame = "base_footprint"
    mapping_initial_robot_pose = ""
    mapping_is_online = "true"
    use_imu_odom_prior = pipeline.mode in {"hesai_imu", "fused"}
    use_rebuilt_wheel_odom_prior = pipeline.mode == "hesai"
    if use_rebuilt_wheel_odom_prior:
        mapping_is_online = "false"
    if args.offline_quality == "max":
        mapping_compression_voxel_size = "0.10"
        mapping_config = Path(args.mapping_config).expanduser() if args.mapping_config else dense_mapping_config
        max_replay_rate = float(os.environ.get("OFFLINE_ICP_MAX_REPLAY_RATE", "0.05"))
        if replay_rate > max_replay_rate:
            replay_rate = max_replay_rate
        result["quality_profile_notes"] = [
            "mapping_compression_voxel_size=0.10",
            f"mapping_config={mapping_config}",
            f"filter_trailer={args.filter_trailer}",
            f"effective_replay_rate={replay_rate}",
        ]
    else:
        result["quality_profile_notes"] = [
            f"mapping_compression_voxel_size={mapping_compression_voxel_size}",
            f"mapping_config={mapping_config}",
            f"filter_trailer={args.filter_trailer}",
            f"effective_replay_rate={replay_rate}",
        ]

    if use_rebuilt_wheel_odom_prior:
        mapping_compression_voxel_size = "0.15"
        mapping_config = Path(args.mapping_config).expanduser() if args.mapping_config else hesai_wheel_mapping_config
        result["quality_profile_notes"] = [
            "pipeline=hesai_wheel",
            "odom_prior=rebuilt_tachometer_articulation",
            "exclude_replayed_tf=true",
            "mapping_robot_frame=base_footprint",
            "mapping_is_online=false",
            "mapping_compression_voxel_size=0.15",
            f"mapping_config={mapping_config}",
            f"effective_replay_rate={replay_rate}",
            "gt_matcher_maxDist=1.10",
            "bounded_icp=0.55rad_1.50m",
            "map_update_distance=0.10m",
            "deterministic_minDistNewPoint=0.05m",
            "sensorMaxRange=60m",
        ]

    if use_imu_odom_prior:
        mapping_robot_frame = "base_footprint"  # consistent avec live mapper (base_link = +10cm Z)
        mapping_initial_robot_pose = "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]"
        mapping_is_online = "false"
        if pipeline.mode == "hesai_imu":
            mapping_compression_voxel_size = "0.50"
            mapping_config = Path(args.mapping_config).expanduser() if args.mapping_config else hesai_imu_mapping_config
        elif pipeline.mode == "fused":
            if args.fused_filter_owner == "mapper":
                mapping_compression_voxel_size = "0.10"
                mapping_config = Path(args.mapping_config).expanduser() if args.mapping_config else dense_mapping_config
            elif points_topic == MERGED_DEBUG_TOPIC:
                mapping_compression_voxel_size = "0.10"
                mapping_config = Path(args.mapping_config).expanduser() if args.mapping_config else dense_mapping_config
            else:
                mapping_compression_voxel_size = "0.20"
                mapping_config = Path(args.mapping_config).expanduser() if args.mapping_config else fused_imu_mapping_config
        result["quality_profile_notes"] = [
            f"pipeline={pipeline.mode}",
            f"source={points_topic}",
            "imu_odom_topic=/mti100/data",
            "imu_odom_rotation_only=true",
            "exclude_replayed_tf=true",
            "use_recorded_tf_static=true",
            "mapping_robot_frame=base_footprint",
            "mapping_initial_robot_pose=identity",
            "mapping_is_online=false",
            f"mapping_compression_voxel_size={mapping_compression_voxel_size}",
            f"mapping_config={mapping_config}",
            f"effective_replay_rate={replay_rate}",
            f"fused_hesai_stride={args.fused_hesai_stride if pipeline.mode == 'fused' else None}",
            f"fused_rsairy_stride={args.fused_rsairy_stride if pipeline.mode == 'fused' else None}",
            f"fused_rsairy_inject_every_n={args.fused_rsairy_inject_every_n if pipeline.mode == 'fused' else None}",
        ]
    result["mapping_config"] = str(mapping_config)
    result["filter_trailer"] = bool(args.filter_trailer)
    result["mapping_odom_frame"] = args.mapping_odom_frame
    result["mapping_robot_frame"] = mapping_robot_frame
    result["mapping_initial_robot_pose"] = "identity" if mapping_initial_robot_pose else None
    result["mapping_is_online"] = mapping_is_online == "true"
    result["use_imu_odom_prior"] = use_imu_odom_prior
    result["fused_hesai_stride"] = args.fused_hesai_stride if pipeline.mode == "fused" else None
    result["fused_rsairy_stride"] = args.fused_rsairy_stride if pipeline.mode == "fused" else None
    result["fused_rsairy_inject_every_n"] = args.fused_rsairy_inject_every_n if pipeline.mode == "fused" else None
    result["imu_topic"] = args.imu_topic if use_imu_odom_prior else None
    result["imu_frame"] = args.imu_frame if use_imu_odom_prior else None
    result["exclude_replayed_tf"] = use_imu_odom_prior or use_rebuilt_wheel_odom_prior
    result["use_recorded_tf_static"] = use_imu_odom_prior
    playback_policy = playback_policy_for_pipeline(args, pipeline)
    result["playback_policy"] = playback_policy
    play_timeout = max(60.0, duration_s / max(replay_rate, 1e-6) + args.play_timeout_margin_seconds)
    # Step replay is CPU-bound by ICP/map insertion, not by bag duration. Dense
    # ground-truth runs often process around 0.5 clouds/s from 20 Hz bags, so a
    # duration*30 guard can kill a healthy run before it reaches the save phase.
    # The outer canonical builder still has a stale-log watchdog to catch real
    # hangs; this timeout is only a hard upper bound.
    step_timeout = max(3600.0, duration_s * 80.0 + args.play_timeout_margin_seconds)
    icp_odom_bag_dir = output_dir / "icp_odom_replay"

    try:
        if use_rebuilt_wheel_odom_prior:
            description_proc = start_process(
                [
                    "ros2", "launch",
                    str(workspace_root / "src/external/norlab_robot/launch/include/description.launch.py"),
                    "use_joint_state_publisher:=false",
                    "use_sim_time:=true",
                ],
                log_dir / f"description_{pipeline.mode}.log",
            )
            runtime_odom_proc = start_process(
                [
                    "ros2", "run", "mtt_driver", "mtt_odometry_node_exe",
                    "--ros-args",
                    "--params-file", str(workspace_root / "demos/common/config/mtt_driver.yaml"),
                    "-p", "use_sim_time:=true",
                    "-p", "broadcast_tf:=true",
                    "-p", "cmd_vel_topic:=cmd_vel",
                    "-p", "hardware_articulation_topic:=/mtt_articulation_angle",
                    "-p", "articulation_state_topic:=/unused/articulation_state",
                    "-p", "articulation_state_output_topic:=mtt/articulation_state/runtime",
                    "-p", "use_articulation_state_lidar:=false",
                    "-p", f"imu_yaw_rate_topic:={args.imu_topic}",
                    "-r", "mtt_odometry:=mtt_odometry/runtime",
                    "-r", "mtt_articulation_angle:=mtt_articulation_angle/runtime",
                ],
                log_dir / f"runtime_odometry_{pipeline.mode}.log",
            )
        elif not use_imu_odom_prior:
            description_proc = start_process(
                [
                    "ros2", "launch",
                    str(workspace_root / "src/external/norlab_robot/launch/include/description.launch.py"),
                    "use_joint_state_publisher:=false",
                    "use_sim_time:=true",
                ],
                log_dir / f"description_{pipeline.mode}.log",
            )

        if pipeline.launch_perception:
            perception_proc = start_process(
                [
                    "ros2", "launch", "mtt_perception", "perception.launch.py",
                    "use_sim_time:=true",
                    "publish_filtered:=true",
                    f"cloud_merger_publish_reliable_raw:={'true' if pipeline.mode == 'fused' and points_topic == MERGED_RELIABLE_TOPIC else 'false'}",
                    "cloud_merger_publish_debug_inputs:=true",
                    f"cloud_merger_hesai_stride:={args.fused_hesai_stride if pipeline.mode == 'fused' else 1}",
                    f"cloud_merger_rsairy_stride:={args.fused_rsairy_stride if pipeline.mode == 'fused' else 1}",
                    f"cloud_merger_rsairy_inject_every_n:={args.fused_rsairy_inject_every_n if pipeline.mode == 'fused' else 1}",
                    f"cloud_merger_enable_self_bbox_filter:={'false' if pipeline.mode == 'fused' and args.fused_filter_owner == 'mapper' else 'true'}",
                    "cloud_merger_max_pair_dt:=0.120",
                    "cloud_merger_tf_timeout:=0.100",
                    f"cloud_merger_enable_trailer_bbox_filter:={'true' if args.filter_trailer else 'false'}",
                ],
                log_dir / f"perception_{pipeline.mode}.log",
            )

        if use_imu_odom_prior:
            imu_odom_proc = start_process(
                [
                    "ros2", "run", "imu_odom", "imu_odom_node",
                    "--ros-args",
                    "-p", "use_sim_time:=true",
                    "-p", f"odom_frame:={args.mapping_odom_frame}",
                    "-p", "robot_frame:=base_footprint",
                    "-p", f"imu_frame:={args.imu_frame}",
                    "-p", "rotation_only:=true",
                    "-p", "real_time:=false",
                    "-r", f"imu_topic:={args.imu_topic}",
                ],
                log_dir / f"imu_odom_{pipeline.mode}.log",
            )

        mapping_command = [
            "ros2", "launch", "norlab_robot", "mapping.launch.py",
            "use_sim_time:=true",
            f"mapping_points_topic:={points_topic}",
            f"mapping_config:={mapping_config}",
            f"mapping_compression_voxel_size:={mapping_compression_voxel_size}",
            f"mapping_odom_frame:={args.mapping_odom_frame}",
            f"mapping_robot_frame:={mapping_robot_frame}",
            f"mapping_is_online:={mapping_is_online}",
            "mapping_input_qos_reliable:=true",
            f"mapping_enable_global_output_map:={str(args.enable_global_output_map).lower()}",
            "mapping_global_output_map_min_dist_new_point:=0.05",
            "mapping_enable_map_trimming:=true",
            "mapping_map_trim_interval_scans:=10",
            f"mapping_map_trim_radius_m:={os.environ.get('OFFLINE_ICP_LOCAL_MAP_RADIUS_M', '60.0')}",
            "mapping_max_map_points_before_trim:=250000",
            f"mapping_max_idle_time:={max(300.0, duration_s / max(replay_rate, 1e-6) + args.play_timeout_margin_seconds)}",
        ]
        if use_rebuilt_wheel_odom_prior:
            mapping_command.extend([
                "deterministic_map_update_distance_m:=0.10",
                "deterministic_map_update_yaw_deg:=1.0",
                "deterministic_map_min_dist_new_point:=0.05",
                "mapping_max_registration_time_ms:=8000.0",
                "mapping_tf_lookup_timeout_ms:=1",
            ])
        if mapping_initial_robot_pose:
            mapping_command.append(f"mapping_initial_robot_pose:={mapping_initial_robot_pose}")

        mapping_proc = start_process(
            mapping_command,
            log_dir / f"mapping_{pipeline.mode}.log",
        )

        pipeline_log_dir = log_dir / pipeline.mode
        pipeline_log_dir.mkdir(parents=True, exist_ok=True)

        if not wait_for_service(SAVE_MAP_SERVICE, args.ready_timeout_seconds, pipeline_log_dir):
            raise RuntimeError("Mapper services did not appear before timeout.")

        run_capture(
            ["ros2", "param", "dump", "/mapping/icp_mapper"],
            log_dir / f"mapping_params_{pipeline.mode}.yaml",
            timeout_s=10.0,
        )
        if use_imu_odom_prior:
            run_capture(
                ["ros2", "param", "dump", "/imu_odom_node"],
                log_dir / f"imu_odom_params_{pipeline.mode}.yaml",
                timeout_s=10.0,
            )

        if args.checkpoint_interval_s > 0.0:
            checkpoint_saver = MappingCheckpointSaver(
                output_dir / "map_checkpoint_latest.vtk",
                output_dir / "trajectory_checkpoint_latest.vtk",
                log_dir,
                pipeline.mode,
                args.checkpoint_interval_s,
            )

        # Record high-quality icp_odom during replay → real sim-time timestamps + SE(3).
        # /mapping/icp_odom is excluded from bag play (REBUILT_MAPPING_TOPICS) so only
        # the fresh offline mapper output lands here.
        # ros2 bag record requires the output dir to NOT exist — it creates it itself.
        shutil.rmtree(icp_odom_bag_dir, ignore_errors=True)  # clean stale data before re-run
        record_proc = start_process(
            [
                "ros2", "bag", "record",
                "--output", str(icp_odom_bag_dir),
                "--storage", "mcap",
                "--topics", "/mapping/icp_odom",
            ],
            log_dir / f"record_icp_odom_{pipeline.mode}.log",
        )

        # Watchdog: alerts if the recorder dies unexpectedly during bag play
        record_watchdog = RecordWatchdog(
            record_proc,
            log_dir / f"record_icp_odom_{pipeline.mode}.log",
            duration_s / replay_rate,
        )

        playback_error: str | None = None
        bag_returncode = 0
        if playback_policy == "step":
            step_script = workspace_root / "demos/bag_replay/scripts/step_replay.py"
            step_command = [
                sys.executable,
                str(step_script),
                "--bag",
                str(bag_dir),
                "--points-topic",
                points_topic,
                "--icp-topic",
                "/mapping/icp_odom",
                "--icp-timeout-s",
                str(args.step_icp_timeout_s),
                "--max-consecutive-timeouts",
                str(args.step_max_consecutive_timeouts),
                "--cloud-stride",
                str(max(1, args.step_cloud_stride)),
                "--mapper-log",
                str(log_dir / f"mapping_{pipeline.mode}.log"),
            ]
            if use_rebuilt_wheel_odom_prior:
                step_command.extend(["--required-prior-topic", "/mtt_tachometer"])
            elif use_imu_odom_prior:
                step_command.extend(["--required-prior-topic", args.imu_topic])
            for topic in excluded_topics_for_pipeline(pipeline):
                step_command.extend(["--exclude-topic", topic])
            bag_proc = start_process(
                step_command,
                log_dir / f"step_replay_{pipeline.mode}.log",
            )
            try:
                bag_returncode = bag_proc.wait(timeout=step_timeout)
            except subprocess.TimeoutExpired as exc:
                playback_error = f"Step replay timed out after {step_timeout:.1f}s"
                terminate_process(bag_proc, grace_s=5.0)
                bag_returncode = bag_proc.returncode if bag_proc.returncode is not None else 124
        else:
            bag_proc = start_process(
                [
                    "ros2", "bag", "play",
                    "--storage", "mcap", str(bag_dir),
                    "--clock",
                    "--rate", str(replay_rate),
                    "--disable-keyboard-controls",
                    "--qos-profile-overrides-path", str(qos_override),
                ]
                + (["--exclude-topics"] + excluded_topics_for_pipeline(pipeline)),
                log_dir / f"bag_play_{pipeline.mode}.log",
            )
            try:
                bag_returncode = bag_proc.wait(timeout=play_timeout)
            except subprocess.TimeoutExpired as exc:
                playback_error = f"Bag playback timed out after {play_timeout:.1f}s"
                terminate_process(bag_proc, grace_s=5.0)
                bag_returncode = bag_proc.returncode if bag_proc.returncode is not None else 124

        died_at = record_watchdog.stop()
        if died_at is not None:
            recorder_error = (
                f"icp_odom recorder died after {died_at:.1f}s "
                f"(playback_policy={playback_policy}) — "
                "icp_odom_replay bag is incomplete."
            )
            if playback_error is None:
                raise RuntimeError(recorder_error)
            result["recorder_error"] = recorder_error

        if bag_returncode != 0 and playback_error is None:
            playback_error = f"{playback_policy} playback exited with code {bag_returncode}"

        # Adaptive settle: give the mapper time to flush last ICP updates.
        # 1 % of bag duration, bounded between 4 s and 15 s.
        effective_settle = max(4.0, min(15.0, duration_s * 0.01))
        time.sleep(effective_settle)

        if checkpoint_saver is not None:
            checkpoint_saver.stop()
            checkpoint_saver = None

        saved, save_errors = save_mapping_outputs(
            map_file,
            trajectory_file,
            log_dir,
            pipeline.mode,
            duration_s,
        )
        if not saved:
            playback_note = f"; playback_error={playback_error}" if playback_error else ""
            raise RuntimeError(
                "Failed to save mapper outputs: "
                + ", ".join(save_errors)
                + playback_note
            )

        if playback_error is not None:
            result["status"] = "partial_saved"
            result["playback_error"] = playback_error
            result["map_size_bytes"] = map_file.stat().st_size
            result["trajectory_size_bytes"] = trajectory_file.stat().st_size
            return result

        if not args.experiment_name:
            shutil.copy2(map_file, compat_map_file)
            shutil.copy2(trajectory_file, compat_trajectory_file)

        result["status"] = "ok"
        result["map_size_bytes"] = map_file.stat().st_size
        result["trajectory_size_bytes"] = trajectory_file.stat().st_size
        return result

    finally:
        if checkpoint_saver is not None:
            checkpoint_saver.stop()
        terminate_process(bag_proc)
        terminate_process(record_proc)  # SIGINT → closes mcap cleanly before killing mapper
        icp_odom_mcap = next(icp_odom_bag_dir.glob("*.mcap"), None)
        result["icp_odom_replay_size_bytes"] = icp_odom_mcap.stat().st_size if icp_odom_mcap else 0
        # Validate the recorded icp_odom bag now that it is finalized
        try:
            result["icp_odom_replay_validation"] = validate_icp_odom_replay(
                icp_odom_bag_dir, duration_s
            )
        except Exception as exc:  # noqa: BLE001
            result["icp_odom_replay_validation"] = {"status": f"validation_error:{exc}"}
        if pipeline.launch_perception:
            result["cloud_merger_stats"] = parse_cloud_merger_stats(
                log_dir / f"perception_{pipeline.mode}.log"
            )
        terminate_process(mapping_proc)
        terminate_process(imu_odom_proc)
        terminate_process(runtime_odom_proc)
        terminate_process(perception_proc)
        terminate_process(description_proc)
        write_summary(output_dir, result)


def process_session(session_dir: Path, args: argparse.Namespace, workspace_root: Path) -> dict:
    bag_dir = session_dir / "bag"
    metadata_path = bag_dir / "metadata.yaml"
    output_dir = session_dir / "offline_icp"
    if args.experiment_name:
        safe_name = "".join(c if c.isalnum() or c in ("-", "_", ".") else "_" for c in args.experiment_name)
        output_dir = session_dir / "offline_icp_runs" / safe_name
    output_dir.mkdir(parents=True, exist_ok=True)

    result: dict[str, object] = {
        "session": session_dir.name,
        "session_dir": str(session_dir),
        "status": "failed",
        "mode": None,
        "points_topic": None,
        "launch_perception": None,
        "reason": None,
        "map_file": str(output_dir / "map.vtk"),
        "trajectory_file": str(output_dir / "trajectory.vtk"),
        "output_dir": str(output_dir),
        "experiment_name": args.experiment_name or None,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }

    if not metadata_path.exists():
        result["status"] = "skipped_missing_metadata"
        write_summary(output_dir, result)
        return result

    map_file = output_dir / "map.vtk"
    trajectory_file = output_dir / "trajectory.vtk"
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

    print(f"Offline ICP input: {args.input_path}", flush=True)
    print(f"Sessions found: {len(sessions)}", flush=True)
    print(f"Mode: {args.mode}", flush=True)
    print(f"Replay rate: {args.replay_rate}x", flush=True)
    print("", flush=True)

    results = []
    failures = 0

    for index, session_dir in enumerate(sessions, start=1):
        print(f"[{index}/{len(sessions)}] {session_dir.name}", flush=True)
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
            print(f"  OK      {result['mode']} -> {session_dir / 'map.vtk'}", flush=True)
        elif status.startswith("skipped"):
            print(f"  SKIPPED {status}", flush=True)
        else:
            failures += 1
            print(f"  FAILED  {result.get('error', status)}", flush=True)
        print("", flush=True)

    report_path = workspace_root / "data" / f"offline_icp_report_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.yaml"
    report_path.write_text(yaml.safe_dump(results, sort_keys=False), encoding="utf-8")

    ok_count = sum(1 for item in results if item["status"] == "ok")
    skipped_count = sum(1 for item in results if str(item["status"]).startswith("skipped"))
    print("Offline ICP summary", flush=True)
    print(f"  OK:       {ok_count}", flush=True)
    print(f"  Skipped:  {skipped_count}", flush=True)
    print(f"  Failed:   {failures}", flush=True)
    print(f"  Report:   {report_path}", flush=True)

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
