#!/usr/bin/env python3
"""Record a live MTT session and save the useful local and robot context next to the bag."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
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


def write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def run_capture(command: list[str], destination: Path) -> None:
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    write_text(destination, result.stdout)
    if result.stderr:
        write_text(destination.with_suffix(destination.suffix + ".stderr"), result.stderr)


def save_vcs_snapshot(workspace_root: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    run_capture(["git", "-C", str(workspace_root), "rev-parse", "--abbrev-ref", "HEAD"], destination / "branch.txt")
    run_capture(["git", "-C", str(workspace_root), "rev-parse", "HEAD"], destination / "commit.txt")
    run_capture(["git", "-C", str(workspace_root), "status", "--short"], destination / "status.txt")
    run_capture(["git", "-C", str(workspace_root), "diff", "--no-ext-diff"], destination / "diff.patch")


def save_metadata(config: dict, config_path: Path, record_dir: Path, bag_dir: Path) -> None:
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
        "topics": config.get("topics", []),
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


def main() -> int:
    args = parse_args()
    workspace_root = Path(os.environ.get("WORKSPACE", Path.cwd())).resolve()
    config_path = Path(args.config).resolve()
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

    topics = config.get("topics", [])
    if not topics:
        raise SystemExit("records.yaml does not define any topics")

    command = ["ros2", "bag", "record", "-s", config.get("storage", "mcap"), "-o", str(bag_dir)]
    max_cache_size = config.get("max_cache_size")
    if max_cache_size:
        command.extend(["--max-cache-size", str(max_cache_size)])
    storage_config = config.get("storage_config")
    if storage_config:
        storage_config_path = Path(storage_config)
        if not storage_config_path.is_absolute():
            storage_config_path = workspace_root / storage_config_path
        command.extend(["--storage-config-file", str(storage_config_path)])
    command.extend(topics)

    if args.dry_run:
        print(f"Record directory: {record_dir}")
        print(f"Bag directory:    {bag_dir}")
        print("Bag command:")
        print("  " + " ".join(command))
        return 0

    bag_dir.mkdir(parents=True, exist_ok=True)
    maybe_copy_config(config_path, record_dir, config.get("config", False))
    if config.get("vcs", False):
        save_vcs_snapshot(workspace_root, record_dir / "vcs")

    write_text(record_dir / "environment.env", "\n".join(f"{k}={v}" for k, v in sorted(os.environ.items())) + "\n")
    save_metadata(config, config_path, record_dir, bag_dir)
    maybe_save_remote_snapshot(workspace_root, record_dir, config.get("remote_snapshot", False))

    print(f"Recording into {bag_dir}")
    return subprocess.run(command, cwd=workspace_root).returncode


if __name__ == "__main__":
    raise SystemExit(main())
