#!/usr/bin/env python3
"""Terminal monitor for /mtt_health.

This monitor is meant for operator use:
- command actually sent,
- telemetry freshness,
- temperatures with a short physical label,
- battery values with explicit raw/estimated semantics,
- fallback command-only motion estimate,
- active warnings.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node

from mtt_msgs.msg import MttHealthState


def _fmt_float(value: float, unit: str = "", digits: int = 2) -> str:
    if math.isnan(value) or math.isinf(value):
        return "n/a"
    return f"{value:.{digits}f}{unit}"


def _fmt_bool(value: bool) -> str:
    return "yes" if value else "no"


def _fmt_temp(value: float, state: str) -> str:
    if state and state != "ok":
        return f"{value:.1f} C ({state})"
    return f"{value:.1f} C"


class HealthMonitor(Node):
    def __init__(self, topic: str, refresh_s: float, clear_screen: bool) -> None:
        super().__init__("mtt_health_terminal_monitor")
        self._topic = topic
        self._refresh_s = refresh_s
        self._clear_screen = clear_screen
        self._latest: MttHealthState | None = None
        self._received_at: float | None = None

        self.create_subscription(MttHealthState, topic, self._on_health, 10)
        self.create_timer(refresh_s, self._render)

    def _on_health(self, msg: MttHealthState) -> None:
        self._latest = msg
        self._received_at = time.monotonic()

    def _render(self) -> None:
        if self._clear_screen:
            print("\033[2J\033[H", end="")

        if self._latest is None:
            print(f"MTT health monitor  topic={self._topic}\n")
            print("waiting for /mtt_health ...", flush=True)
            return

        msg = self._latest
        age = 0.0 if self._received_at is None else max(0.0, time.monotonic() - self._received_at)
        lines: list[str] = []
        lines.append(f"MTT health monitor  topic={self._topic}  age={age:.1f}s")
        lines.append("")
        lines.append(
            "summary  "
            f"{msg.health_summary}  "
            f"telemetry_fresh={_fmt_bool(msg.telemetry_fresh)}  "
            f"telemetry_age={_fmt_float(msg.telemetry_age_ms, ' ms', 0)}"
        )
        lines.append(
            "tachy    "
            f"seen_once={_fmt_bool(msg.tachometer_present)}  "
            f"stale={_fmt_bool(msg.tachometer_stale)}  "
            f"source={msg.tachometer_source or 'unknown'}  "
            f"synthetic={_fmt_bool(msg.tachometer_is_synthetic)}"
        )
        lines.append("")
        lines.append(
            "command  "
            f"can=0x{msg.command_can_id:X}  "
            f"thr={msg.throttle_raw}  brk={msg.brake_raw}  steer={msg.steer_raw}  "
            f"steer_norm={msg.steer_normalized:.3f}  dir={msg.direction}"
        )
        lines.append(
            "         "
            f"v_cmd={_fmt_float(msg.commanded_linear_speed_ms, ' m/s')}  "
            f"ang_in={_fmt_float(msg.commanded_angular_input)}  "
            f"yaw_cmd={_fmt_float(msg.commanded_yaw_rate_rad_s, ' rad/s')}  "
            f"mode={msg.steer_control_mode}/{msg.cmd_angular_mode}"
        )
        lines.append(
            "safety   "
            f"unlocked={_fmt_bool(msg.security_unlocked)}  "
            f"deadman={_fmt_bool(msg.deadman_active)}  "
            f"estop={_fmt_bool(msg.emergency_stop_active)}  "
            f"timeout={_fmt_bool(msg.command_timeout_active)}  "
            f"external={_fmt_bool(msg.external_control_active)}"
        )
        lines.append("")
        lines.append(
            "main T   "
            f"A={_fmt_temp(msg.main_sensor_temp_a_c, msg.controller_temp_state)} "
            "[module principal]   "
            f"B={_fmt_temp(msg.main_sensor_temp_b_c, msg.encoder_temp_state)} "
            "[cote encodeur/tachy]"
        )
        lines.append(
            "battery T "
            f"cells=({msg.cell_temp_1_c:.1f}, {msg.cell_temp_2_c:.1f}, {msg.cell_temp_3_c:.1f}, {msg.cell_temp_4_c:.1f}) C   "
            f"ambient={msg.ambient_temp_c:.1f} C   mosfet={msg.mosfet_temp_c:.1f} C"
        )
        lines.append(
            "         "
            f"heatpadA={msg.heatpad_a_temp_c:.1f} C   heatpadB={msg.heatpad_b_temp_c:.1f} C   "
            f"battery_state={msg.battery_temp_state}"
        )
        lines.append("")
        lines.append(
            "battery  "
            f"soc={msg.soc_percent}%  "
            f"current_raw={msg.battery_current_raw}  "
            f"current_est={_fmt_float(msg.battery_current_estimated_a, ' A')} "
            f"({'ok' if msg.battery_current_estimated_valid else 'invalid'})"
        )
        voltage_text = (
            _fmt_float(msg.battery_voltage_v, " V")
            if msg.battery_voltage_valid
            else f"{msg.battery_voltage_raw} raw [echelle a valider]"
        )
        power_text = (
            _fmt_float(msg.power_watts, " W")
            if msg.power_valid
            else "non publie [tension non validee]"
        )
        lines.append(
            "         "
            f"voltage={voltage_text}  "
            f"power={power_text}  "
            f"charge_remaining={msg.charge_time_remaining_min} min  "
            f"heatpads=A{int(msg.heatpad_a_on)}/B{int(msg.heatpad_b_on)}"
        )
        lines.append("")
        lines.append(
            "fallback "
            f"active={_fmt_bool(msg.fallback_active)}  "
            f"low_conf={_fmt_bool(msg.fallback_low_confidence)}  "
            f"speed={_fmt_float(msg.fallback_speed_ms, ' m/s')}  "
            f"yaw={_fmt_float(msg.fallback_yaw_rate_rad_s, ' rad/s')}"
        )
        lines.append(
            "         "
            f"dist={_fmt_float(msg.fallback_distance_m, ' m')}  "
            f"heading={_fmt_float(msg.fallback_heading_rad, ' rad')}  "
            f"reason={msg.fallback_reason}"
        )
        lines.append("")
        if msg.can_debug_enabled:
            lines.append(
                "can rx   "
                f"debug={_fmt_bool(msg.can_debug_available)}  "
                f"0x2FF={_fmt_float(msg.telemetry_frame_hz, ' Hz')}  "
                f"0x600={_fmt_float(msg.bms_cell_frame_hz, ' Hz')}  "
                f"0x601={_fmt_float(msg.bms_sys_frame_hz, ' Hz')}  "
                f"0x602={_fmt_float(msg.bms_core_frame_hz, ' Hz')}"
            )
            lines.append(
                "         "
                f"0x603={_fmt_float(msg.bms_datetime_frame_hz, ' Hz')}  "
                f"charger={_fmt_float(msg.charger_status_frame_hz, ' Hz')}"
            )
        else:
            lines.append("can rx   debug disabled in mtt_can_node")
        lines.append("")
        lines.append("warnings")
        if msg.warnings:
            lines.extend([f"  - {text}" for text in msg.warnings])
        else:
            lines.append("  - none")

        print("\n".join(lines), flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live terminal monitor for /mtt_health")
    parser.add_argument("--topic", default="mtt_health", help="Health topic to monitor")
    parser.add_argument("--refresh", type=float, default=0.5, help="Screen refresh period in seconds")
    parser.add_argument("--no-clear", action="store_true", help="Do not clear the terminal between refreshes")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = HealthMonitor(args.topic, args.refresh, not args.no_clear)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
