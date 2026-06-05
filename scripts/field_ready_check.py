#!/usr/bin/env python3
"""Short operator-facing readiness check for MTT WILN teach/repeat."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from dataclasses import dataclass

import rclpy
from rclpy.node import Node


GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"


@dataclass
class Check:
    name: str
    ok: bool
    required: bool = True
    detail: str = ""


class GraphNode(Node):
    def __init__(self) -> None:
        super().__init__("mtt_field_ready_check")

    def topic_names(self) -> set[str]:
        return {name for name, _types in self.get_topic_names_and_types()}

    def service_names(self) -> set[str]:
        return {name for name, _types in self.get_service_names_and_types()}


def echo_field(topic: str, field: str = "data", timeout_s: float = 1.5) -> str:
    try:
        result = subprocess.run(
            ["timeout", f"{timeout_s:.1f}s", "ros2", "topic", "echo", "--once", topic, "--field", field],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except OSError:
        return "unavailable"
    out = result.stdout.strip()
    return out if result.returncode == 0 and out else "missing"


def print_check(check: Check) -> None:
    if check.ok:
        mark = f"{GREEN}OK{RESET}"
    elif check.required:
        mark = f"{RED}FAIL{RESET}"
    else:
        mark = f"{YELLOW}WARN{RESET}"
    suffix = f" - {check.detail}" if check.detail else ""
    print(f"{mark:18} {check.name}{suffix}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-icp-quality", action="store_true")
    parser.add_argument("--icp-duration", type=float, default=6.0)
    args = parser.parse_args()

    rclpy.init()
    node = GraphNode()
    rclpy.spin_once(node, timeout_sec=0.5)
    topics = node.topic_names()
    services = node.service_names()
    node.destroy_node()
    rclpy.shutdown()

    required_topics = [
        "/mapping/icp_odom",
        "/mtt_health",
        "/wiln/command",
        "/wiln/teach/state",
        "/wiln/route/state",
        "/wiln/replay/state",
        "/wiln/follower/state",
    ]
    optional_topics = [
        "/mapping/map",
        "/mapping/trajectory_path",
        "/mapping/aligned_scan",
        "/wiln/trajectory",
        "/wiln/global_plan",
        "/wiln/control/local_plan",
        "/controller/cmd_vel",
        "/cmd_vel",
        "/mtt_control/selected_source",
        "/mtt_control/selected_mode",
        "/mtt_control/auto_mode_enabled",
        "/mtt_control/teleop_deadman",
    ]
    required_services = [
        "/mtt_repeat/teach_start",
        "/mtt_repeat/teach_stop",
        "/mtt_repeat/play_line",
        "/mtt_repeat/cancel",
        "/mtt_repeat/mark_ready",
        "/mtt_route/load",
        "/mtt_route/replay",
        "/mtt_route/status",
        "/mtt_route/stop",
    ]

    checks: list[Check] = []
    checks.extend(Check(topic, topic in topics, True) for topic in required_topics)
    checks.extend(Check(topic, topic in topics, False) for topic in optional_topics)
    checks.extend(Check(service, service in services, True) for service in required_services)

    print(f"{BOLD}== MTT Field Ready Check =={RESET}")
    print("")
    print(f"{BOLD}ROS graph{RESET}")
    for check in checks:
        print_check(check)

    print("")
    print(f"{BOLD}Control snapshot{RESET}")
    mode = echo_field("/mtt_control/selected_mode")
    auto_enabled = echo_field("/mtt_control/auto_mode_enabled")
    source = echo_field("/mtt_control/selected_source")
    deadman = echo_field("/mtt_control/teleop_deadman")
    print(f"selected_mode: {mode}")
    print(f"auto_enabled:  {auto_enabled}")
    print(f"selected_src:  {source}")
    print(f"deadman:       {deadman}")
    print("buttons: A=auto, B=stop, Y=manual, RB=deadman")

    required_ok = all(check.ok for check in checks if check.required)
    icp_ok = True
    if not args.skip_icp_quality:
        print("")
        workspace = os.environ.get("WORKSPACE", "/home/mohamed/Documents/Project_MTT/Workspace/mtt_workspace")
        command = [
            sys.executable,
            f"{workspace}/scripts/check_icp_odom.py",
            "--duration",
            f"{args.icp_duration:.1f}",
        ]
        result = subprocess.run(command, text=True, check=False)
        icp_ok = result.returncode == 0

    print("")
    if required_ok and icp_ok:
        print(f"verdict: {GREEN}OK{RESET} ready to teach; repeat still requires a valid loaded route")
        return 0

    print(f"verdict: {RED}FAIL{RESET} fix failed required checks before field repeat")
    if "/wiln/command" not in topics:
        print("hint: start WILN with: docker compose --profile wiln up -d --force-recreate wiln")
    if "/mapping/icp_odom" not in topics:
        print("hint: robot/mapping is not publishing ICP odom")
    if source == "AUTO_WAIT":
        print("hint: AUTO_WAIT means arbiter is waiting for fresh controller/cmd_vel")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
