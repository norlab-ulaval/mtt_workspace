#!/usr/bin/env python3
"""Passive CAN audit tool for the MTT platform.

This script is meant for Phase 0 field work:
- sniff the bus without sending anything,
- group traffic by arbitration ID,
- estimate per-ID rates,
- highlight known MTT frames,
- and emit a JSON artifact you can compare across scenarios.
"""

from __future__ import annotations

import argparse
import json
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import can

from mtt_can_support import KNOWN_MTT_IDS
from mtt_can_support import decode_frame
from mtt_can_support import load_dbc


def _hex_bytes(data: bytes) -> str:
    return "".join(f"{byte:02X}" for byte in data)


def _interface_exists(interface: str) -> bool:
    return Path(f"/sys/class/net/{interface}").exists()


@dataclass
class FrameStats:
    name: str
    count: int = 0
    first_ts: float = 0.0
    last_ts: float = 0.0
    last_data_hex: str = ""
    dlc: int = 0
    extended: bool = False
    change_mask: list[int] = field(default_factory=lambda: [0] * 8)
    payload_counts: Counter[str] = field(default_factory=Counter)

    def update(self, message: can.Message, timestamp: float) -> None:
        data = bytes(message.data)
        payload_hex = _hex_bytes(data)

        if self.count == 0:
            self.first_ts = timestamp
            self.dlc = message.dlc
            self.extended = bool(message.is_extended_id)
        else:
            old_data = bytes.fromhex(self.last_data_hex) if self.last_data_hex else b""
            for index, (old_byte, new_byte) in enumerate(zip(old_data, data)):
                if old_byte != new_byte:
                    self.change_mask[index] = 1

        self.count += 1
        self.last_ts = timestamp
        self.last_data_hex = payload_hex
        self.payload_counts[payload_hex] += 1

    def rate_hz(self) -> float:
        if self.count <= 1:
            return 0.0
        duration = self.last_ts - self.first_ts
        if duration <= 0.0:
            return 0.0
        return (self.count - 1) / duration

    def to_summary(self, arbitration_id: int, database: Any | None = None) -> dict[str, Any]:
        decoded_bundle = decode_frame(arbitration_id, bytes.fromhex(self.last_data_hex), database) if self.last_data_hex else {
            "source": "none",
            "signals": {},
            "derived": {},
        }
        top_payloads = [
            {"payload": payload, "count": count}
            for payload, count in self.payload_counts.most_common(5)
        ]
        return {
            "id_hex": f"0x{arbitration_id:X}",
            "name": self.name,
            "count": self.count,
            "rate_hz": round(self.rate_hz(), 3),
            "dlc": self.dlc,
            "extended": self.extended,
            "last_data_hex": self.last_data_hex,
            "byte_change_mask": self.change_mask,
            "top_payloads": top_payloads,
            "decode_source": decoded_bundle["source"],
            "decoded_last_frame": decoded_bundle["signals"],
            "derived_last_frame": decoded_bundle["derived"],
        }


def _build_report(interface: str, duration: float, frame_stats: dict[int, FrameStats], database: Any | None = None, dbc_path: Path | None = None) -> dict[str, Any]:
    summaries = [
        stats.to_summary(arbitration_id, database)
        for arbitration_id, stats in sorted(frame_stats.items(), key=lambda item: (-item[1].count, item[0]))
    ]
    observed_ids = set(frame_stats.keys())

    warnings = []
    if 0x001 not in observed_ids and 0x100 not in observed_ids:
        warnings.append("No control frame (0x001 or 0x100) was seen during capture")
    if 0x2FF not in observed_ids:
        warnings.append("No main telemetry frame (0x2FF) was seen during capture")
    if any(frame_id in observed_ids for frame_id in (0x600, 0x601, 0x602, 0x603)) and 0x2FF not in observed_ids:
        warnings.append("BMS frames are present but traction telemetry is absent; display/controller path may be asleep or disconnected")
    if 0x001 in observed_ids and 0x100 in observed_ids:
        warnings.append("Both 0x001 and 0x100 are present; confirm the real arbitration rule with the firmware owner")

    return {
        "interface": interface,
        "capture_duration_s": duration,
        "total_unique_ids": len(frame_stats),
        "dbc_path": str(dbc_path) if dbc_path is not None else None,
        "dbc_loaded": database is not None,
        "observed_known_ids": sorted(f"0x{frame_id:X}" for frame_id in observed_ids if frame_id in KNOWN_MTT_IDS),
        "warnings": warnings,
        "frames": summaries,
    }


def capture_can_audit(interface: str, duration: float, database: Any | None = None, dbc_path: Path | None = None) -> dict[str, Any]:
    frame_stats: dict[int, FrameStats] = {}
    if not _interface_exists(interface):
        raise RuntimeError(
            f"CAN interface '{interface}' was not found. Bring the interface up first and run this on the host connected to the bus."
        )
    try:
        bus = can.interface.Bus(interface="socketcan", channel=interface)
    except OSError as exc:
        raise RuntimeError(
            f"Could not open CAN interface '{interface}'. Bring the interface up first and run this on the host connected to the bus."
        ) from exc
    start = time.monotonic()
    deadline = start + duration

    try:
        while time.monotonic() < deadline:
            message = bus.recv(timeout=0.1)
            if message is None:
                continue
            stats = frame_stats.setdefault(
                message.arbitration_id,
                FrameStats(name=KNOWN_MTT_IDS.get(message.arbitration_id, "unknown")),
            )
            stats.update(message, time.monotonic())
    finally:
        bus.shutdown()

    return _build_report(interface, duration, frame_stats, database=database, dbc_path=dbc_path)


def print_report(report: dict[str, Any]) -> None:
    print(f"CAN audit on {report['interface']} for {report['capture_duration_s']:.1f}s")
    print(f"Unique IDs: {report['total_unique_ids']}")
    if report["warnings"]:
        print("Warnings:")
        for warning in report["warnings"]:
            print(f"  - {warning}")

    print("Top frames:")
    for frame in report["frames"][:15]:
        print(
            f"  {frame['id_hex']:>10}  {frame['count']:>6} frames  "
            f"{frame['rate_hz']:>8.2f} Hz  {frame['name']}"
        )
        if frame["decoded_last_frame"]:
            print(f"    last decode ({frame['decode_source']}): {frame['decoded_last_frame']}")
        if frame["derived_last_frame"]:
            print(f"    derived: {frame['derived_last_frame']}")
        print(f"    change mask: {frame['byte_change_mask']}  last: {frame['last_data_hex']}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Passive CAN audit tool for MTT")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface to sniff")
    parser.add_argument("--duration", type=float, default=10.0, help="Capture duration in seconds")
    parser.add_argument("--dbc", default=None, help="DBC path, default is the repo MTT simple DBC")
    parser.add_argument("--no-dbc", action="store_true", help="Disable DBC loading and stay on fallback decoding only")
    parser.add_argument("--json-out", type=Path, default=None, help="Optional JSON output path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    database = None
    dbc_path = None
    if not args.no_dbc:
        database, dbc_path = load_dbc(args.dbc)
    try:
        report = capture_can_audit(interface=args.interface, duration=args.duration, database=database, dbc_path=dbc_path)
    except RuntimeError as exc:
        print(exc)
        return 1
    print_report(report)

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"Saved JSON report to {args.json_out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
