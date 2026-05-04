#!/usr/bin/env python3
"""Live MTT CAN monitor with DBC-aware summaries.

This is a passive operator-side dashboard:
- battery status from 0x602 / 0x600 / 0x601,
- drive status from 0x2FF,
- last command frame from 0x001 / 0x100.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import can

from mtt_can_support import KNOWN_MTT_IDS
from mtt_can_support import decode_frame
from mtt_can_support import format_age
from mtt_can_support import load_dbc


def _interface_exists(interface: str) -> bool:
    return Path(f"/sys/class/net/{interface}").exists()


def _value(signals: dict[str, Any], *names: str) -> Any:
    for name in names:
        if name in signals:
            return signals[name]
    return None


@dataclass
class LatestFrame:
    frame_id: int
    data: bytes = b""
    timestamp: float = 0.0
    count: int = 0
    decoded: dict[str, Any] | None = None

    def update(self, data: bytes, timestamp: float, decoded: dict[str, Any]) -> None:
        self.data = data
        self.timestamp = timestamp
        self.count += 1
        self.decoded = decoded


def _render_summary(frames: dict[int, LatestFrame], start_time: float, interface: str, dbc_path: Path, dbc_loaded: bool) -> str:
    now = time.monotonic()
    uptime = now - start_time

    lines = [
        f"MTT CAN monitor  interface={interface}  uptime={uptime:.1f}s  dbc={'on' if dbc_loaded else 'fallback'} ({dbc_path})",
        "",
    ]

    battery = frames.get(0x602)
    if battery and battery.decoded:
        signals = battery.decoded["signals"]
        current_raw = _value(signals, "BatteryCurrent_raw")
        current_a = _value(signals, "BatteryCurrent")
        if not isinstance(current_a, (int, float)) and isinstance(current_raw, (int, float)):
            current_a = current_raw * 0.0103 - 0.72
        battery_line = (
            "battery  "
            f"soc={_value(signals, 'StateOfCharge')}%  "
            f"current_raw={current_raw}  "
        )
        if isinstance(current_a, (int, float)):
            battery_line += f"current_A={current_a:.2f}  "
        battery_line += (
            f"voltage_raw={_value(signals, 'BatteryVoltage_raw')}  "
            f"heatpads=A{_value(signals, 'HeatpadA_On', 'HeatpadAOn')}/B{_value(signals, 'HeatpadB_On', 'HeatpadBOn')}  "
            f"remaining={_value(signals, 'ChargeTimeRemaining')} min  "
            f"age={format_age(now - battery.timestamp)}"
        )
        lines.append(battery_line)
    else:
        lines.append("battery  no 0x602 frame seen yet")

    battery_temps = frames.get(0x600)
    if battery_temps and battery_temps.decoded:
        signals = battery_temps.decoded["signals"]
        lines.append(
            "cells    "
            f"t1={_value(signals, 'CellTemp1')}  "
            f"t2={_value(signals, 'CellTemp2')}  "
            f"t3={_value(signals, 'CellTemp3')}  "
            f"t4={_value(signals, 'CellTemp4')}  "
            f"age={format_age(now - battery_temps.timestamp)}"
        )

    system_temps = frames.get(0x601)
    if system_temps and system_temps.decoded:
        signals = system_temps.decoded["signals"]
        lines.append(
            "bms temp "
            f"ambient={_value(signals, 'AmbientTemp')}  "
            f"mos={_value(signals, 'MosfetTemp')}  "
            f"padA={_value(signals, 'HeatpadATemp')}  "
            f"padB={_value(signals, 'HeatpadBTemp')}  "
            f"age={format_age(now - system_temps.timestamp)}"
        )

    drive = frames.get(0x2FF)
    if drive and drive.decoded:
        signals = drive.decoded["signals"]
        derived = drive.decoded["derived"]
        lines.append(
            "drive    "
            f"tach={_value(signals, 'TachometerInstant_ticks_per_s')} ticks/s  "
            f"speed={derived.get('speed_kmh_estimate', 'n/a')} km/h  "
            f"dist={derived.get('absolute_distance_m_estimate', 'n/a')} m  "
            f"tempA={_value(signals, 'MainSensorTempA')}C  "
            f"tempB={_value(signals, 'MainSensorTempB')}C  "
            f"age={format_age(now - drive.timestamp)}"
        )
        lines.append(
            "         "
            f"speed_ms={derived.get('speed_ms_estimate', 'n/a')}  "
            f"decode={drive.decoded.get('source', 'unknown')}"
        )
    else:
        lines.append("drive    no 0x2FF frame seen yet")

    command_frame = None
    external = frames.get(0x100)
    joystick = frames.get(0x001)
    if external and joystick:
        command_frame = external if external.timestamp >= joystick.timestamp else joystick
    else:
        command_frame = external or joystick

    if command_frame and command_frame.decoded:
        signals = command_frame.decoded["signals"]
        lines.append(
            "command  "
            f"id=0x{command_frame.frame_id:X}  "
            f"unlock={_value(signals, 'SecurityUnlocked')}  "
            f"light_patch={_value(signals, 'LightOff_EStopPatch')}  "
            f"reverse={_value(signals, 'DirectionReverse')}  "
            f"thr={_value(signals, 'Throttle')}  "
            f"brk={_value(signals, 'Brake')}  "
            f"steer={_value(signals, 'Steering')}  "
            f"mode={_value(signals, 'SteeringModeClosedLoop')}  "
            f"age={format_age(now - command_frame.timestamp)}"
        )
        if command_frame.frame_id == 0x100:
            lines.append("         external control frame is currently the freshest command")
    else:
        lines.append("command  no 0x001 or 0x100 frame seen yet")

    seen_ids = sorted(frames.values(), key=lambda item: item.timestamp, reverse=True)[:8]
    if seen_ids:
        lines.append("")
        lines.append("recent")
        for item in seen_ids:
            lines.append(
                f"  0x{item.frame_id:<8X} {KNOWN_MTT_IDS.get(item.frame_id, 'unknown'):<24} "
                f"count={item.count:<5} age={format_age(now - item.timestamp)}"
            )

    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live MTT CAN monitor with DBC-aware summaries")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface to read")
    parser.add_argument("--dbc", default=None, help="DBC path, default is the repo MTT simple DBC")
    parser.add_argument("--refresh", type=float, default=0.5, help="Screen refresh period in seconds")
    parser.add_argument("--duration", type=float, default=0.0, help="Optional run duration in seconds, 0 means forever")
    parser.add_argument("--no-clear", action="store_true", help="Do not clear the terminal between refreshes")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not _interface_exists(args.interface):
        print(f"CAN interface '{args.interface}' was not found.")
        return 1

    database, dbc_path = load_dbc(args.dbc)
    if database is None:
        print(f"DBC not loaded, using manual fallback decode from {dbc_path}")
    bus = can.interface.Bus(interface="socketcan", channel=args.interface)
    frames: dict[int, LatestFrame] = {}
    start_time = time.monotonic()
    next_refresh = start_time

    try:
        while True:
            now = time.monotonic()
            if args.duration > 0.0 and (now - start_time) >= args.duration:
                break

            timeout = max(0.0, next_refresh - now)
            message = bus.recv(timeout=timeout)
            if message is not None:
                decoded = decode_frame(message.arbitration_id, bytes(message.data), database)
                frame = frames.setdefault(message.arbitration_id, LatestFrame(frame_id=message.arbitration_id))
                frame.update(bytes(message.data), time.monotonic(), decoded)

            now = time.monotonic()
            if now >= next_refresh:
                if not args.no_clear:
                    print("\033[2J\033[H", end="")
                print(_render_summary(frames, start_time, args.interface, dbc_path, database is not None), flush=True)
                next_refresh = now + max(0.1, args.refresh)
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
