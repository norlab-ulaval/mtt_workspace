#!/usr/bin/env python3
"""Build canonical offline ICP/map outputs and research datasets for MTT bags."""

from __future__ import annotations

import argparse
import contextlib
import os
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


ACTIVE_PROCESS: subprocess.Popen | None = None


@dataclass(frozen=True)
class Candidate:
    name: str
    mode: str
    replay_rate: float
    quality: str
    env: dict[str, str]
    reason: str


@dataclass(frozen=True)
class SensorChoice:
    tachometer_messages: int
    tachometer_sampled: int
    tachometer_synthetic_ratio: float
    tachometer_model_valid_ratio: float
    wheel_prior_usable: bool
    reason: str


@dataclass(frozen=True)
class TopicProbe:
    topic: str
    messages: int
    sampled: int
    readable: bool
    first_stamp_s: float | None
    reason: str


@dataclass(frozen=True)
class SessionPreflight:
    counts: dict[str, int]
    wheel: SensorChoice
    imu: TopicProbe
    cloud: TopicProbe
    status: str
    reasons: list[str]


def infer_workspace_root() -> Path:
    env_workspace = os.environ.get("WORKSPACE")
    if env_workspace:
        return Path(env_workspace).resolve()
    script_path = Path(__file__).resolve()
    for candidate in [script_path.parent, *script_path.parents]:
        if (candidate / "src").exists() and (candidate / "demos").exists():
            return candidate
    return script_path.parent


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception as exc:  # noqa: BLE001
        return {"_load_error": str(exc)}


def resolve_sessions(path_value: str) -> list[Path]:
    path = Path(path_value).expanduser().resolve()
    if path.is_file() and path.suffix == ".mcap":
        return [path.parent.parent if path.parent.name == "bag" else path.parent]
    if (path / "bag" / "metadata.yaml").exists():
        return [path]
    if (path / "metadata.yaml").exists():
        return [path.parent]
    sessions = sorted(p.parent.parent for p in path.glob("*/bag/metadata.yaml"))
    if sessions:
        return sessions
    raise SystemExit(f"Could not resolve sessions from {path}")


def resolve_session_list(list_path: Path) -> list[Path]:
    sessions: list[Path] = []
    for raw in list_path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        sessions.extend(resolve_sessions(line))
    seen: set[Path] = set()
    unique: list[Path] = []
    for session in sessions:
        resolved = session.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        unique.append(resolved)
    return unique


def bag_topic_counts(session_dir: Path) -> dict[str, int]:
    metadata = load_yaml(session_dir / "bag" / "metadata.yaml")
    info = metadata.get("rosbag2_bagfile_information", metadata)
    return {
        item["topic_metadata"]["name"]: int(item["message_count"])
        for item in info.get("topics_with_message_count", [])
    }


def safe_name(value: str) -> str:
    return "".join(c if c.isalnum() or c in ("-", "_", ".") else "_" for c in value)


def extract_stamp_s(msg: Any, fallback_ns: int) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is not None and nanosec is not None and (int(sec) != 0 or int(nanosec) != 0):
        return float(sec) + float(nanosec) * 1e-9
    return fallback_ns / 1e9


def probe_topic(session_dir: Path, counts: dict[str, int], topic: str, max_messages: int = 10) -> TopicProbe:
    messages = counts.get(topic, 0)
    if messages <= 0:
        return TopicProbe(topic, 0, 0, False, None, f"{topic} missing")
    try:
        import rosbag2_py  # type: ignore[import]
        from rclpy.serialization import deserialize_message  # type: ignore[import]
        from rosidl_runtime_py.utilities import get_message  # type: ignore[import]
    except ImportError as exc:
        return TopicProbe(topic, messages, 0, True, None, f"ROS bag Python unavailable; assuming readable: {exc}")

    sampled = 0
    first_stamp_s: float | None = None
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(session_dir / "bag"), storage_id="mcap"),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        if topic not in topic_types:
            return TopicProbe(topic, messages, 0, False, None, f"{topic} type missing")
        msg_type = get_message(topic_types[topic])
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        while reader.has_next() and sampled < max_messages:
            _, raw, timestamp_ns = reader.read_next()
            msg = deserialize_message(raw, msg_type)
            sampled += 1
            if first_stamp_s is None:
                first_stamp_s = extract_stamp_s(msg, timestamp_ns)
    except Exception as exc:  # noqa: BLE001
        return TopicProbe(topic, messages, sampled, False, first_stamp_s, f"{topic} unreadable: {exc}")

    return TopicProbe(topic, messages, sampled, sampled > 0, first_stamp_s, f"{topic} sampled={sampled}")


def inspect_sensor_choice(session_dir: Path, counts: dict[str, int], max_messages: int = 500) -> SensorChoice:
    if counts.get("/mtt_tachometer", 0) <= 0:
        return SensorChoice(0, 0, 1.0, 0.0, False, "no /mtt_tachometer")

    try:
        import rosbag2_py  # type: ignore[import]
        from rclpy.serialization import deserialize_message  # type: ignore[import]
        from rosidl_runtime_py.utilities import get_message  # type: ignore[import]
    except ImportError:
        return SensorChoice(
            counts.get("/mtt_tachometer", 0),
            0,
            0.0,
            0.0,
            True,
            "rosbag2_py unavailable; assuming recorded tachometer is usable",
        )

    sampled = 0
    synthetic = 0
    model_valid = 0
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(session_dir / "bag"), storage_id="mcap"),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        if "/mtt_tachometer" not in topic_types:
            return SensorChoice(0, 0, 1.0, 0.0, False, "no /mtt_tachometer type")
        msg_type = get_message(topic_types["/mtt_tachometer"])
        reader.set_filter(rosbag2_py.StorageFilter(topics=["/mtt_tachometer"]))
        while reader.has_next() and sampled < max_messages:
            _, raw, _ = reader.read_next()
            msg = deserialize_message(raw, msg_type)
            sampled += 1
            synthetic += int(bool(getattr(msg, "tachometer_is_synthetic", False)))
            model_valid += int(bool(getattr(msg, "model_state_valid", False)))
    except Exception as exc:  # noqa: BLE001
        return SensorChoice(
            counts.get("/mtt_tachometer", 0),
            sampled,
            0.0,
            0.0,
            False,
            f"tachometer inspection failed; wheel prior demoted: {exc}",
        )

    synthetic_ratio = synthetic / max(sampled, 1)
    model_valid_ratio = model_valid / max(sampled, 1)
    usable = sampled > 0 and synthetic_ratio < 0.20
    reason = (
        f"tachometer sampled={sampled} synthetic={synthetic_ratio:.1%} "
        f"model_valid={model_valid_ratio:.1%}"
    )
    if not usable:
        reason += " -> wheel prior demoted"
    return SensorChoice(
        counts.get("/mtt_tachometer", 0),
        sampled,
        synthetic_ratio,
        model_valid_ratio,
        usable,
        reason,
    )


def preflight_session(session_dir: Path) -> SessionPreflight:
    counts = bag_topic_counts(session_dir)
    wheel = inspect_sensor_choice(session_dir, counts)
    cloud = probe_topic(session_dir, counts, "/hesai_lidar/points", max_messages=3)
    imu = probe_topic(session_dir, counts, "/mti100/data", max_messages=10)
    reasons: list[str] = []
    status = "OK"
    if counts.get("/hesai_lidar/points", 0) <= 0:
        status = "SKIP"
        reasons.append("no_hesai_lidar_points")
    elif not cloud.readable:
        status = "SKIP"
        reasons.append(cloud.reason)
    if not wheel.wheel_prior_usable and not imu.readable:
        status = "SKIP"
        reasons.append("no_valid_wheel_or_imu_prior")
    return SessionPreflight(counts, wheel, imu, cloud, status, reasons)


def candidates_for_session(session_dir: Path, args: argparse.Namespace) -> list[Candidate]:
    preflight = preflight_session(session_dir)
    counts = preflight.counts
    has_hesai = counts.get("/hesai_lidar/points", 0) > 0
    has_rsairy = counts.get("/rsairy_ns/points", 0) > 0
    has_imu = preflight.imu.readable
    sensor_choice = preflight.wheel
    out: list[Candidate] = []

    if args.smart_preflight and preflight.status == "SKIP":
        return out

    if has_hesai:
        wheel_candidate = Candidate(
            "hesai_wheel_max",
            "hesai",
            args.replay_rate,
            args.quality,
            {
                "OFFLINE_ICP_MAPPING_CONFIG": str(
                    infer_workspace_root()
                    / "src/external/norlab_robot/config/mapping/_config_hesai_wheel_replay.yaml"
                )
            },
            "Hesai raw cloud with replay/rebuilt wheel odom prior; " + sensor_choice.reason,
        )
        imu_candidate = Candidate(
            "hesai_imu_max",
            "hesai_imu",
            args.replay_rate,
            args.quality,
            {},
            "Hesai raw cloud with MTi-100 rotation-only odom prior",
        )
        if args.profile_policy == "all":
            out.append(wheel_candidate)
            if has_imu:
                out.append(imu_candidate)
        elif sensor_choice.wheel_prior_usable:
            out.append(wheel_candidate)
            if has_imu and args.profile_policy == "smart":
                out.append(imu_candidate)
            elif has_imu and args.prefer_imu:
                out.append(imu_candidate)
        else:
            if has_imu:
                out.append(imu_candidate)
            if not args.smart_preflight:
                out.append(wheel_candidate)
        ultra_slow_mode = "hesai_imu" if has_imu and (args.prefer_imu or not sensor_choice.wheel_prior_usable) else "hesai"
        if ultra_slow_mode == "hesai_imu" or sensor_choice.wheel_prior_usable or not args.smart_preflight:
            out.append(
                Candidate(
                    "hesai_ultra_slow",
                    ultra_slow_mode,
                    min(args.replay_rate, 0.10),
                    args.quality,
                    {},
                    "Slow recovery candidate for difficult geometry or aggressive maneuvers",
                )
            )

    if has_hesai and has_rsairy and args.enable_fused:
        out.append(
            Candidate(
                "fused_lidar_max",
                "fused",
                min(args.replay_rate, 0.25),
                args.quality,
                {
                    "OFFLINE_ICP_FUSED_POINTS_TOPIC": "/merged_points_reliable",
                    "OFFLINE_ICP_FUSED_FILTER_OWNER": "mapper",
                },
                "Dual LiDAR candidate accepted only if objectively better",
            )
        )

    return out


def run_command(
    command: list[str],
    *,
    cwd: Path,
    env: dict[str, str] | None,
    log_path: Path,
) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    merged_env = os.environ.copy()
    if env:
        merged_env.update(env)
    with log_path.open("w", encoding="utf-8") as log:
        proc = subprocess.run(
            command,
            cwd=cwd,
            env=merged_env,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    return proc.returncode


def run_command_with_progress(
    command: list[str],
    *,
    cwd: Path,
    env: dict[str, str] | None,
    log_path: Path,
    label: str,
    stale_timeout_s: float,
    report_interval_s: float = 60.0,
    progress_root: Path | None = None,
) -> int:
    global ACTIVE_PROCESS
    log_path.parent.mkdir(parents=True, exist_ok=True)
    merged_env = os.environ.copy()
    if env:
        merged_env.update(env)
    with log_path.open("w", encoding="utf-8") as log:
        proc = subprocess.Popen(
            command,
            cwd=cwd,
            env=merged_env,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        ACTIVE_PROCESS = proc
        try:
            start = time.monotonic()
            last_report = start
            last_size = -1
            last_growth = start
            while proc.poll() is None:
                now = time.monotonic()
                size = progress_size(log_path, progress_root)
                if size != last_size:
                    last_size = size
                    last_growth = now
                if now - last_report >= report_interval_s:
                    print(
                        f"    {label}: running {now - start:.0f}s, log={size} bytes, "
                        f"silent={now - last_growth:.0f}s",
                        flush=True,
                    )
                    last_report = now
                if now - last_growth > stale_timeout_s:
                    print(
                        f"    {label}: stale for {now - last_growth:.0f}s, terminating candidate",
                        flush=True,
                    )
                    terminate_process_group(proc)
                    return 124
                time.sleep(5.0)
            return int(proc.returncode or 0)
        finally:
            ACTIVE_PROCESS = None


def progress_size(log_path: Path, progress_root: Path | None) -> int:
    total = log_path.stat().st_size if log_path.exists() else 0
    if progress_root and progress_root.exists():
        for path in progress_root.rglob("*"):
            if path.is_file():
                try:
                    total += path.stat().st_size
                except OSError:
                    pass
    return total


def terminate_process_group(proc: subprocess.Popen, grace_s: float = 10.0) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.2)
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.2)
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return


def handle_shutdown(signum: int, _frame: Any) -> None:
    proc = ACTIVE_PROCESS
    if proc is not None and proc.poll() is None:
        print(f"\nReceived signal {signum}; stopping active offline ICP process group...", flush=True)
        terminate_process_group(proc)
    raise KeyboardInterrupt


def copytree_clean(src: Path, dst: Path) -> None:
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)


def run_candidate(
    session_dir: Path,
    candidate: Candidate,
    args: argparse.Namespace,
    workspace_root: Path,
    run_id: str,
) -> dict[str, Any]:
    offline_script = workspace_root / "demos/bag_replay/scripts/offline_icp.py"
    audit_script = workspace_root / "scripts/audit_offline_icp_run.py"
    run_name = safe_name(run_id)
    run_dir = session_dir / "offline_icp_runs" / run_name
    log_dir = session_dir / "postprocess_dataset" / "logs" / "canonical_builder"
    env = {
        **candidate.env,
        "OFFLINE_ICP_MODE": candidate.mode,
        "OFFLINE_ICP_QUALITY": candidate.quality,
        "REPLAY_RATE": str(candidate.replay_rate),
        "OFFLINE_ICP_FORCE": "true",
        "ROS_DOMAIN_ID": str(args.ros_domain_id),
    }
    if candidate.mode in {"hesai_imu", "fused"}:
        env.setdefault("OFFLINE_ICP_IMU_TOPIC", args.imu_topic)
        env.setdefault("OFFLINE_ICP_IMU_FRAME", args.imu_frame)

    command = [
        sys.executable,
        str(offline_script),
        str(session_dir),
        "--mode",
        candidate.mode,
        "--offline-quality",
        candidate.quality,
        "--replay-rate",
        str(candidate.replay_rate),
        "--experiment-name",
        run_name,
        "--force",
    ]
    code = run_command_with_progress(
        command,
        cwd=workspace_root,
        env=env,
        log_path=log_dir / f"{run_name}.offline_icp.log",
        label=run_name,
        stale_timeout_s=args.stale_timeout_s,
        report_interval_s=args.progress_interval_s,
        progress_root=run_dir,
    )

    quality: dict[str, Any] = {
        "status": "FAIL",
        "reasons": [f"offline_icp_returncode:{code}"],
        "score": -9999.0,
        "run_dir": str(run_dir),
    }
    if run_dir.exists():
        audit_command = [
            sys.executable,
            str(audit_script),
            str(run_dir),
            "--session-dir",
            str(session_dir),
            "--output",
            str(run_dir / "quality.yaml"),
        ]
        if args.manual_candidate_threshold == "relaxed":
            audit_command.extend(["--manual-candidate-threshold", "relaxed"])
        audit_code = run_command(
            audit_command,
            cwd=workspace_root,
            env=None,
            log_path=log_dir / f"{run_name}.audit.log",
        )
        quality = load_yaml(run_dir / "quality.yaml")
        quality["audit_returncode"] = audit_code

    result = {
        "run_id": run_name,
        "candidate": candidate.__dict__,
        "offline_icp_returncode": code,
        "run_dir": str(run_dir),
        "quality": quality,
    }
    (run_dir / "selected_run.yaml").write_text(
        yaml.safe_dump(result, sort_keys=False),
        encoding="utf-8",
    ) if run_dir.exists() else None
    return result


def select_best(results: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not results:
        return None
    passed = [item for item in results if item.get("quality", {}).get("status") == "PASS"]
    pool = passed or results
    return max(pool, key=lambda item: float(item.get("quality", {}).get("score") or -9999.0))


def canonical_pass_exists(session_dir: Path) -> dict[str, Any] | None:
    canonical_dir = session_dir / "offline_icp_canonical"
    quality = load_yaml(canonical_dir / "quality.yaml")
    if quality.get("status") != "PASS":
        return None
    if not (canonical_dir / "map.vtk").exists() or not (canonical_dir / "trajectory.vtk").exists():
        return None
    return quality


def existing_completed_runs(session_dir: Path) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    runs_dir = session_dir / "offline_icp_runs"
    if not runs_dir.exists():
        return out
    for run_dir in sorted(item for item in runs_dir.iterdir() if item.is_dir()):
        quality = load_yaml(run_dir / "quality.yaml")
        if quality.get("status") not in {"PASS", "MANUAL_CANDIDATE"}:
            continue
        if not (run_dir / "map.vtk").exists() or not (run_dir / "trajectory.vtk").exists():
            continue
        selected = load_yaml(run_dir / "selected_run.yaml")
        quality["run_dir"] = str(run_dir)
        out.append({
            "run_id": run_dir.name,
            "candidate": selected.get("candidate", {}),
            "offline_icp_returncode": selected.get("offline_icp_returncode", 0),
            "run_dir": str(run_dir),
            "quality": quality,
            "resumed_existing": True,
        })
    return out


def build_dataset(session_dir: Path, args: argparse.Namespace, workspace_root: Path) -> dict[str, Any]:
    script = workspace_root / "scripts/build_postprocess_dataset.py"
    log_dir = session_dir / "postprocess_dataset" / "logs" / "canonical_builder"
    command = [
        sys.executable,
        str(script),
        str(session_dir),
        "--prefer-offline-icp",
        "--canonical-icp",
        "--quality",
        args.quality,
    ]
    code = run_command(command, cwd=workspace_root, env=None, log_path=log_dir / "build_postprocess_dataset.log")
    return {"returncode": code, "summary": load_yaml(session_dir / "postprocess_dataset" / "summary.yaml")}


def audit_dataset(session_dir: Path, workspace_root: Path) -> dict[str, Any]:
    script = workspace_root / "scripts/audit_postprocess_dataset.py"
    log_dir = session_dir / "postprocess_dataset" / "logs" / "canonical_builder"
    code = run_command(
        [sys.executable, str(script), str(session_dir), "--no-plots"],
        cwd=workspace_root,
        env=None,
        log_path=log_dir / "audit_postprocess_dataset.log",
    )
    return {"returncode": code, "audit": load_yaml(session_dir / "postprocess_dataset" / "audit.yaml")}


def process_session(session_dir: Path, args: argparse.Namespace, workspace_root: Path) -> dict[str, Any]:
    preflight = preflight_session(session_dir)
    counts = preflight.counts
    sensor_choice = preflight.wheel
    canonical_dir = session_dir / "offline_icp_canonical"
    report: dict[str, Any] = {
        "session": session_dir.name,
        "session_dir": str(session_dir),
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "status": "FAILED",
        "sensor_choice": sensor_choice.__dict__,
        "preflight": {
            "status": preflight.status,
            "reasons": preflight.reasons,
            "cloud": preflight.cloud.__dict__,
            "imu": preflight.imu.__dict__,
        },
        "runs": [],
    }
    if args.resume:
        existing_canonical = canonical_pass_exists(session_dir)
        if existing_canonical is not None:
            report["status"] = "SKIPPED_RESUMED_CANONICAL"
            report["canonical_quality"] = existing_canonical
            report["selected_run"] = load_yaml(canonical_dir / "selected_run.yaml")
            return report

    if args.smart_preflight and preflight.status == "SKIP":
        report["status"] = "SKIPPED_PREFLIGHT"
        return report
    candidates = candidates_for_session(session_dir, args)
    if not candidates:
        report["error"] = "no_lidar_candidate"
        return report

    if args.resume:
        resumed_runs = existing_completed_runs(session_dir)
        if resumed_runs:
            print(f"  resume: found {len(resumed_runs)} completed run(s); reusing best", flush=True)
            report["runs"].extend(resumed_runs)
    if not report["runs"]:
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        for index, candidate in enumerate(candidates, start=1):
            if not args.force and (session_dir / "offline_icp_canonical" / "quality.yaml").exists():
                existing_quality = load_yaml(session_dir / "offline_icp_canonical" / "quality.yaml")
                if existing_quality.get("status") == "PASS":
                    report["status"] = "SKIPPED_EXISTING"
                    report["canonical_quality"] = existing_quality
                    return report
                shutil.rmtree(session_dir / "offline_icp_canonical", ignore_errors=True)
            run_id = f"{timestamp}_{index:02d}_{candidate.name}"
            print(f"  candidate {index}/{len(candidates)}: {candidate.name}", flush=True)
            result = run_candidate(session_dir, candidate, args, workspace_root, run_id)
            report["runs"].append(result)
            if result.get("quality", {}).get("status") == "PASS" and not args.run_all_candidates:
                break

    best = select_best(report["runs"])
    if not best:
        report["error"] = "no_candidate_completed"
        return report

    best_status = best.get("quality", {}).get("status")
    if best_status not in {"PASS", "MANUAL_CANDIDATE"}:
        shutil.rmtree(canonical_dir, ignore_errors=True)
        report["status"] = "FAILED_NO_PASSING_CANDIDATE"
        report["best_failed_run"] = {
            "run_id": best.get("run_id"),
            "run_dir": best.get("run_dir"),
            "quality": best.get("quality", {}),
            "candidate": best.get("candidate", {}),
        }
        report["canonical_quality"] = best.get("quality", {})
        return report

    if best_status == "MANUAL_CANDIDATE":
        manual_dir = session_dir / "offline_icp_manual_candidates" / safe_name(str(best["run_id"]))
        copytree_clean(Path(best["run_dir"]), manual_dir)
        selected_doc = {
            "selected_run_id": best["run_id"],
            "selected_run_dir": best["run_dir"],
            "manual_candidate_dir": str(manual_dir),
            "selected_at": datetime.now().isoformat(timespec="seconds"),
            "quality": best.get("quality", {}),
            "candidate": best.get("candidate", {}),
        }
        (manual_dir / "selected_run.yaml").write_text(
            yaml.safe_dump(selected_doc, sort_keys=False),
            encoding="utf-8",
        )
        shutil.rmtree(canonical_dir, ignore_errors=True)
        report["selected_run"] = selected_doc
        report["canonical_quality"] = best.get("quality", {})
        report["status"] = "MANUAL_CANDIDATE"
        return report

    copytree_clean(Path(best["run_dir"]), canonical_dir)
    selected_doc = {
        "selected_run_id": best["run_id"],
        "selected_run_dir": best["run_dir"],
        "selected_at": datetime.now().isoformat(timespec="seconds"),
        "quality": best.get("quality", {}),
        "candidate": best.get("candidate", {}),
    }
    (canonical_dir / "selected_run.yaml").write_text(
        yaml.safe_dump(selected_doc, sort_keys=False),
        encoding="utf-8",
    )

    report["selected_run"] = selected_doc
    report["canonical_quality"] = best.get("quality", {})
    report["status"] = "PASS"
    report["dataset"] = build_dataset(session_dir, args, workspace_root)
    report["dataset_audit"] = audit_dataset(session_dir, workspace_root)
    if report["dataset"]["returncode"] != 0:
        report["status"] = "FAILED_DATASET"
    return report


def detect_stale_offline_processes(session_dir: Path) -> list[str]:
    session_text = str(session_dir)
    session_name = session_dir.name
    patterns = (
        "offline_icp.py",
        "step_replay.py",
        "ros2 bag play",
        "ros2 bag record",
        "mapping.launch.py",
        "imu_odom_node",
    )
    def scan() -> dict[str, str]:
        try:
            result = subprocess.run(["ps", "-ef"], check=False, capture_output=True, text=True)
        except Exception:  # noqa: BLE001
            return {}
        found: dict[str, str] = {}
        for line in result.stdout.splitlines():
            parts = line.split(None, 7)
            if len(parts) < 8:
                continue
            pid = parts[1]
            if pid == str(os.getpid()) and "build_canonical_mapping_dataset.py" in line:
                continue
            if (session_text in line or session_name in line) and any(pattern in line for pattern in patterns):
                found[pid] = line
        return found

    # ros2 bag record can linger for a fraction of a second after Ctrl-C cleanup.
    # Require the same PID to be present in two scans before blocking a new run.
    first = scan()
    if not first:
        return []
    time.sleep(1.0)
    second = scan()
    return [second[pid] for pid in sorted(set(first).intersection(second))]


def terminate_stale_offline_processes(stale_lines: list[str]) -> None:
    pids: list[int] = []
    for line in stale_lines:
        parts = line.split(None, 7)
        if len(parts) < 2:
            continue
        try:
            pids.append(int(parts[1]))
        except ValueError:
            continue

    for sig, wait_s in ((signal.SIGINT, 3.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 0.0)):
        remaining: list[int] = []
        for pid in pids:
            try:
                os.killpg(pid, sig)
            except ProcessLookupError:
                continue
            except PermissionError:
                try:
                    os.kill(pid, sig)
                except (ProcessLookupError, PermissionError):
                    continue
            except OSError:
                try:
                    os.kill(pid, sig)
                except (ProcessLookupError, PermissionError, OSError):
                    continue
            remaining.append(pid)

        if wait_s > 0.0:
            deadline = time.time() + wait_s
            while time.time() < deadline:
                alive = []
                for pid in remaining:
                    try:
                        os.kill(pid, 0)
                        alive.append(pid)
                    except ProcessLookupError:
                        pass
                if not alive:
                    return
                remaining = alive
                time.sleep(0.2)
            pids = remaining


@contextlib.contextmanager
def session_lock(session_dir: Path) -> Any:
    lock_path = session_dir / "postprocess_dataset" / ".canonical_builder.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(str(lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError as exc:
        owner = lock_path.read_text(encoding="utf-8", errors="ignore") if lock_path.exists() else ""
        raise RuntimeError(
            f"canonical builder lock already exists: {lock_path}. "
            f"Another run is active or was interrupted. Owner: {owner.strip()!r}. "
            "Remove the lock only after stopping the old process."
        ) from exc
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(f"pid={os.getpid()} started_at={datetime.now().isoformat(timespec='seconds')}\n")
        yield
    finally:
        try:
            lock_path.unlink()
        except FileNotFoundError:
            pass


def report_legacy(sessions: list[Path]) -> list[dict[str, Any]]:
    out = []
    for session in sessions:
        legacy = [name for name in ("offline_icp", "map.vtk", "trajectory.vtk") if (session / name).exists()]
        if legacy:
            out.append({"session": session.name, "legacy_paths": legacy})
    return out


def archive_legacy(sessions: list[Path]) -> list[dict[str, Any]]:
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out = []
    for session in sessions:
        archive = session / "archive" / f"legacy_mapping_{timestamp}"
        moved = []
        for name in ("offline_icp", "map.vtk", "trajectory.vtk"):
            src = session / name
            if not src.exists():
                continue
            archive.mkdir(parents=True, exist_ok=True)
            dst = archive / name
            shutil.move(str(src), str(dst))
            moved.append(name)
        if moved:
            out.append({"session": session.name, "archive_dir": str(archive), "moved": moved})
    return out


def parse_args() -> argparse.Namespace:
    workspace_root = infer_workspace_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_path", nargs="?", default=str(workspace_root / "data"))
    parser.add_argument("--session-list", type=Path, default=None, help="Text file with one session dir per line; overrides input_path.")
    parser.add_argument("--quality", default="max", choices=["standard", "max"])
    parser.add_argument("--replay-rate", type=float, default=1.0)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--force-failed", action="store_true", help="Alias for --force for failed/incomplete canonical runs.")
    parser.add_argument(
        "--resume",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Reuse completed PASS/MANUAL runs and existing canonical PASS outputs. "
            "This keeps long overnight batches resumable even when --force is used."
        ),
    )
    parser.add_argument("--prefer-imu", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enable-fused", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--smart-preflight", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--profile-policy", choices=["smart", "all"], default="smart")
    parser.add_argument(
        "--manual-candidate-threshold",
        choices=["off", "relaxed"],
        default="off",
        help="Pass relaxed thresholds to the run audit and store good visual candidates separately.",
    )
    parser.add_argument("--run-all-candidates", action="store_true")
    parser.add_argument("--progress-interval-s", type=float, default=60.0)
    parser.add_argument("--stale-timeout-s", type=float, default=900.0)
    parser.add_argument("--imu-topic", default="/mti100/data")
    parser.add_argument("--imu-frame", default="imu_link")
    parser.add_argument("--ros-domain-id", type=int, default=77, help="Isolated ROS domain for offline canonical processing.")
    parser.add_argument("--cleanup-stale", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--report-legacy", action="store_true")
    parser.add_argument("--archive-legacy", action="store_true")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--jobs", type=int, default=1, help="Reserved for future parallelism; current implementation is sequential.")
    return parser.parse_args()


def main() -> int:
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)
    workspace_root = infer_workspace_root()
    args = parse_args()
    args.force = bool(args.force or args.force_failed)
    sessions = resolve_session_list(args.session_list) if args.session_list else resolve_sessions(args.input_path)
    if args.jobs != 1:
        print("WARN: --jobs is currently reserved; processing sequentially for ROS_DOMAIN isolation.", flush=True)

    if args.report_legacy:
        report = report_legacy(sessions)
        print(yaml.safe_dump(report, sort_keys=False))
        return 0
    if args.archive_legacy:
        report = archive_legacy(sessions)
        print(yaml.safe_dump(report, sort_keys=False))
        return 0

    results = []
    failures = 0
    for index, session in enumerate(sessions, start=1):
        print(f"[{index}/{len(sessions)}] {session.name}", flush=True)
        try:
            stale = detect_stale_offline_processes(session)
            if stale:
                if args.cleanup_stale:
                    print("  cleaning stale offline ICP processes from previous run", flush=True)
                    terminate_stale_offline_processes(stale)
                    stale = detect_stale_offline_processes(session)
                if stale:
                    raise RuntimeError(
                        "stale offline ICP processes are still running for this session. "
                        "Stop them before relaunching:\n" + "\n".join(stale[:20])
                    )
            with session_lock(session):
                result = process_session(session, args, workspace_root)
        except Exception as exc:  # noqa: BLE001
            result = {"session": session.name, "status": "FAILED_EXCEPTION", "error": str(exc)}
        print(f"  {result.get('status')}", flush=True)
        if str(result.get("status", "")).startswith("FAILED"):
            failures += 1
        results.append(result)
        (session / "postprocess_dataset").mkdir(parents=True, exist_ok=True)
        (session / "postprocess_dataset" / "canonical_summary.yaml").write_text(
            yaml.safe_dump(result, sort_keys=False),
            encoding="utf-8",
        )

    report_path = workspace_root / "data" / "canonical_mapping_report.yaml"
    report_path.write_text(yaml.safe_dump(results, sort_keys=False), encoding="utf-8")
    print(f"Report: {report_path}")
    return 1 if args.strict and failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
