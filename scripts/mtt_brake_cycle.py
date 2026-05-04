#!/usr/bin/env python3
"""Small operator tool to cycle the MTT brake command and watch ROS feedback.

This script only publishes `mtt_aux_cmd`:
- brake command in [0.0, 1.0]
- winch neutral
- light off

It is meant for diagnostics when the brake appears to stay engaged:
- verify that ROS really sends brake=0 back to the driver
- detect when safety locks override the brake command
- exercise the hydraulic circuit with pulse or triangle profiles

This does NOT command throttle or steering.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from mtt_msgs.msg import MttAuxCommand, MttHealthState, MttVehicleStatus


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Cycle the MTT brake command and print live ROS feedback."
    )
    parser.add_argument(
        "--mode",
        choices=("release", "hold", "pulse", "triangle"),
        default="release",
        help="Brake profile to send.",
    )
    parser.add_argument(
        "--max-brake",
        type=float,
        default=1.0,
        help="Maximum brake command in [0,1]. Default: 1.0",
    )
    parser.add_argument(
        "--min-brake",
        type=float,
        default=0.0,
        help="Minimum brake command in [0,1]. Default: 0.0",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=2.0,
        help="Duration for hold/release mode. Default: 2.0",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=1.5,
        help="Period in seconds for pulse/triangle modes. Default: 1.5",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=5,
        help="Number of pulse/triangle cycles. Default: 5",
    )
    parser.add_argument(
        "--duty-cycle",
        type=float,
        default=0.5,
        help="High phase duty cycle for pulse mode in [0,1]. Default: 0.5",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=10.0,
        help="Publish rate. Default: 10 Hz",
    )
    parser.add_argument(
        "--report-every",
        type=float,
        default=0.5,
        help="Console report period in seconds. Default: 0.5",
    )
    parser.add_argument(
        "--release-on-exit-seconds",
        type=float,
        default=2.0,
        help="How long to keep publishing brake=0 before exit. Default: 2.0",
    )
    parser.add_argument(
        "--aux-topic",
        default="mtt_aux_cmd",
        help="Auxiliary command topic. Default: mtt_aux_cmd",
    )
    parser.add_argument(
        "--status-topic",
        default="mtt_status",
        help="Vehicle status topic. Default: mtt_status",
    )
    parser.add_argument(
        "--health-topic",
        default="mtt_health",
        help="Health topic. Default: mtt_health",
    )
    return parser.parse_args()


@dataclass
class BrakeCommandSnapshot:
    brake: float
    mode_label: str
    cycle_index: int


class BrakeCycleNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("mtt_brake_cycle")
        self.args = args
        self.pub_ = self.create_publisher(MttAuxCommand, args.aux_topic, 10)
        self.create_subscription(MttVehicleStatus, args.status_topic, self.on_status, 10)
        self.create_subscription(MttHealthState, args.health_topic, self.on_health, 10)

        self.status_msg: Optional[MttVehicleStatus] = None
        self.health_msg: Optional[MttHealthState] = None
        self.start_time = self.get_clock().now()
        self.release_phase = False
        self.release_phase_started_s: Optional[float] = None
        self.done = False
        self.last_report_s = -1e9

        self.min_brake = clamp01(args.min_brake)
        self.max_brake = clamp01(args.max_brake)
        self.duty_cycle = clamp01(args.duty_cycle)
        self.publish_period = 1.0 / max(args.rate_hz, 1e-3)
        self.timer = self.create_timer(self.publish_period, self.on_timer)

    def elapsed_s(self) -> float:
        delta = self.get_clock().now() - self.start_time
        return delta.nanoseconds * 1e-9

    def on_status(self, msg: MttVehicleStatus) -> None:
        self.status_msg = msg

    def on_health(self, msg: MttHealthState) -> None:
        self.health_msg = msg

    def command_for_time(self, elapsed_s: float) -> BrakeCommandSnapshot:
        if self.release_phase:
            return BrakeCommandSnapshot(0.0, "release-on-exit", -1)

        if self.args.mode == "release":
            return BrakeCommandSnapshot(0.0, "forced-release", 0)

        if self.args.mode == "hold":
            return BrakeCommandSnapshot(self.max_brake, "hold", 0)

        cycle_index = int(elapsed_s / max(self.args.period, 1e-6))
        phase = (elapsed_s % max(self.args.period, 1e-6)) / max(self.args.period, 1e-6)

        if self.args.mode == "pulse":
            brake = self.max_brake if phase < self.duty_cycle else self.min_brake
            return BrakeCommandSnapshot(brake, "pulse", cycle_index)

        # triangle
        if phase < 0.5:
            alpha = phase / 0.5
        else:
            alpha = (1.0 - phase) / 0.5
        brake = self.min_brake + alpha * (self.max_brake - self.min_brake)
        return BrakeCommandSnapshot(brake, "triangle", cycle_index)

    def should_enter_release_phase(self, elapsed_s: float, snapshot: BrakeCommandSnapshot) -> bool:
        if self.release_phase:
            return False
        if self.args.mode in {"release", "hold"}:
            return elapsed_s >= self.args.hold_seconds
        return snapshot.cycle_index >= self.args.cycles

    def publish_aux(self, brake: float) -> None:
        msg = MttAuxCommand()
        msg.brake = float(clamp01(brake))
        msg.winch_command = 0
        msg.light_state = 0
        self.pub_.publish(msg)

    def report(self, elapsed_s: float, snapshot: BrakeCommandSnapshot) -> None:
        if elapsed_s - self.last_report_s < self.args.report_every:
            return
        self.last_report_s = elapsed_s

        cmd_raw = int(round(clamp01(snapshot.brake) * 255.0))
        status_brake = self.status_msg.brake_raw if self.status_msg is not None else None
        safety_state = self.status_msg.safety_state if self.status_msg is not None else "no-status"
        telemetry_age = (
            f"{self.status_msg.telemetry_age_ms:.0f} ms"
            if self.status_msg is not None
            else "n/a"
        )

        health_tail = ""
        if self.health_msg is not None:
            health_tail = (
                f"  estop={'yes' if self.health_msg.emergency_stop_active else 'no'}"
                f"  deadman={'yes' if self.health_msg.deadman_active else 'no'}"
                f"  timeout={'yes' if self.health_msg.command_timeout_active else 'no'}"
                f"  unlocked={'yes' if self.health_msg.security_unlocked else 'no'}"
            )

        print(
            "  ".join(
                [
                    f"t={elapsed_s:5.1f}s",
                    f"profile={snapshot.mode_label}",
                    f"cmd={snapshot.brake:0.2f}",
                    f"cmd_raw={cmd_raw:3d}",
                    f"status_brake={status_brake if status_brake is not None else 'n/a'}",
                    f"safety={safety_state}",
                    f"telemetry_age={telemetry_age}",
                ]
            )
            + health_tail,
            flush=True,
        )

        if self.health_msg is not None and self.health_msg.warnings:
            print(f"    warnings: {', '.join(self.health_msg.warnings)}", flush=True)

    def on_timer(self) -> None:
        elapsed_s = self.elapsed_s()
        snapshot = self.command_for_time(elapsed_s)

        if self.should_enter_release_phase(elapsed_s, snapshot):
            self.release_phase = True
            self.release_phase_started_s = elapsed_s
            snapshot = self.command_for_time(elapsed_s)
            self.get_logger().info("Switching to brake=0 release phase before exit.")

        self.publish_aux(snapshot.brake)
        self.report(elapsed_s, snapshot)

        if self.release_phase and self.release_phase_started_s is not None:
            if elapsed_s - self.release_phase_started_s >= self.args.release_on_exit_seconds:
                self.done = True
                self.timer.cancel()

    def force_release_burst(self, repeats: int = 10) -> None:
        for _ in range(max(repeats, 1)):
            self.publish_aux(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = BrakeCycleNode(args)

    print("MTT brake diagnostic", flush=True)
    print(
        "This script only publishes mtt_aux_cmd with throttle=0 and steering untouched.",
        flush=True,
    )
    print(
        f"mode={args.mode}  min={clamp01(args.min_brake):.2f}  max={clamp01(args.max_brake):.2f}"
        f"  period={args.period:.2f}s  cycles={args.cycles}",
        flush=True,
    )
    print(
        "Use mtt_health_monitor.py in another terminal if you want the full operator view.",
        flush=True,
    )

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nInterrupted, sending brake=0 release burst before exit.", flush=True)
        node.force_release_burst()
    finally:
        if rclpy.ok():
            node.force_release_burst(repeats=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
