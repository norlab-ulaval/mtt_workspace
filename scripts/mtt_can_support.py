#!/usr/bin/env python3
"""Shared helpers for the local MTT CAN scripts."""

from __future__ import annotations

import os
import re
import struct
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DBC_CANDIDATES = (
    REPO_ROOT / "documentations" / "MTT_CAN_v1_1_simple.dbc",
    REPO_ROOT / "documentations" / "mttDriver.dbc",
    REPO_ROOT / "documentations" / "MTT_CAN_v1.1_simple.dbc",
    REPO_ROOT / "documentations" / "MTT_CAN_v1_1.dbc",
)
DEFAULT_TOOL_VENV_PYTHON = REPO_ROOT / ".venv-tools" / "bin" / "python"


KNOWN_MTT_IDS = {
    0x001: "remote_control",
    0x100: "external_control",
    0x2FF: "main_telemetry",
    0x300: "main_controller_version",
    0x301: "battery_controller_version",
    0x351: "module_351_unknown",
    0x600: "bms_cell_temperatures",
    0x601: "bms_system_temperatures",
    0x602: "bms_core_status",
    0x603: "bms_extra_status",
    0x650: "bms_aux_650",
    0x651: "bms_aux_651",
    0x652: "bms_aux_652",
    0x653: "bms_aux_653",
    0x1806E5F4: "charger_command",
    0x18FF50E5: "charger_status",
}


# Mirrors the current driver parameters used in mtt_driver.py.
MTT_ENCODER_FINAL_RATIO = 324.0
MTT_TRACK_LENGTH_KM = 393.0 / 100000.0
MTT_TRACK_LENGTH_M = 393.0 / 100.0


CSV_FIELDS = [
    "timestamp",
    "interface",
    "id_hex",
    "name",
    "is_extended",
    "dlc",
    "data_hex",
    "decode_source",
    "StateOfCharge",
    "BatteryCurrent",
    "BatteryCurrent_raw",
    "BatteryVoltage_raw",
    "HeatpadA_On",
    "HeatpadB_On",
    "ChargeTimeRemaining",
    "CellTemp1",
    "CellTemp2",
    "CellTemp3",
    "CellTemp4",
    "AmbientTemp",
    "MosfetTemp",
    "HeatpadATemp",
    "HeatpadBTemp",
    "MainSensorTempA",
    "MainSensorTempB",
    "TachometerInstant_ticks_per_s",
    "TachometerCumulative_ticks",
    "speed_kmh_estimate",
    "speed_ms_estimate",
    "absolute_distance_m_estimate",
    "VehicleType",
    "SecurityUnlocked",
    "LightOff_EStopPatch",
    "DirectionReverse",
    "Throttle",
    "Winch",
    "Brake",
    "Steering",
    "SteeringModeClosedLoop",
]


_CANDUMP_HASH_RE = re.compile(
    r"^\((?P<timestamp>[0-9]+\.[0-9]+)\)\s+"
    r"(?P<interface>[A-Za-z0-9_]+)\s+"
    r"(?P<frame>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)$"
)

_CANDUMP_BRACKET_RE = re.compile(
    r"^(?:\((?P<timestamp>[0-9]+\.[0-9]+)\)\s+)?"
    r"(?P<interface>[A-Za-z0-9_]+)\s+"
    r"(?P<frame>[0-9A-Fa-f]+)\s+\[(?P<dlc>[0-9]+)\]\s*"
    r"(?P<data>(?:[0-9A-Fa-f]{2}\s*)*)$"
)


def resolve_repo_path(path: str | Path | None) -> Path:
    """Resolve paths relative to the repo root."""
    if path is None:
        env_path = os.environ.get("MTT_DBC_PATH")
        if env_path:
            resolved = Path(env_path).expanduser()
            return resolved if resolved.is_absolute() else REPO_ROOT / resolved

        for candidate in DEFAULT_DBC_CANDIDATES:
            if candidate.exists():
                return candidate

        return DEFAULT_DBC_CANDIDATES[0]
    resolved = Path(path).expanduser()
    if resolved.is_absolute():
        return resolved
    return REPO_ROOT / resolved


def maybe_reexec_with_tool_venv() -> None:
    """Re-run this script with the local tool venv when needed."""

    if os.environ.get("MTT_CAN_NO_REEXEC") == "1":
        return

    target_python = DEFAULT_TOOL_VENV_PYTHON
    if not target_python.exists():
        return

    try:
        import cantools  # noqa: F401

        return
    except ModuleNotFoundError:
        pass

    current_python = Path(sys.executable)
    if current_python == target_python:
        return

    env = os.environ.copy()
    env["MTT_CAN_NO_REEXEC"] = "1"
    os.execve(str(target_python), [str(target_python), *sys.argv], env)


def load_dbc(path: str | Path | None = None) -> tuple[Any | None, Path]:
    """Load the local DBC when cantools is available."""
    dbc_path = resolve_repo_path(path)
    if not dbc_path.exists():
        return None, dbc_path

    maybe_reexec_with_tool_venv()

    try:
        import cantools
    except ModuleNotFoundError:
        return None, dbc_path

    try:
        database = cantools.database.load_file(str(dbc_path))
    except Exception:
        return None, dbc_path

    return database, dbc_path


def parse_candump_line(line: str) -> dict[str, Any] | None:
    """Parse the common can-utils candump text formats."""

    stripped = line.strip()
    if not stripped:
        return None

    match = _CANDUMP_HASH_RE.match(stripped)
    if match:
        frame_hex = match.group("frame")
        data_hex = match.group("data")
        return {
            "timestamp": float(match.group("timestamp")),
            "interface": match.group("interface"),
            "arbitration_id": int(frame_hex, 16),
            "id_hex": f"0x{int(frame_hex, 16):X}",
            "is_extended": len(frame_hex) > 3,
            "data": bytes.fromhex(data_hex) if data_hex else b"",
            "dlc": len(data_hex) // 2,
            "data_hex": data_hex.upper(),
        }

    match = _CANDUMP_BRACKET_RE.match(stripped)
    if match:
        frame_hex = match.group("frame")
        raw_data = match.group("data") or ""
        data_hex = raw_data.replace(" ", "")
        return {
            "timestamp": float(match.group("timestamp")) if match.group("timestamp") else None,
            "interface": match.group("interface"),
            "arbitration_id": int(frame_hex, 16),
            "id_hex": f"0x{int(frame_hex, 16):X}",
            "is_extended": len(frame_hex) > 3,
            "data": bytes.fromhex(data_hex) if data_hex else b"",
            "dlc": int(match.group("dlc")),
            "data_hex": data_hex.upper(),
        }

    return None


def _safe_float32(data: bytes) -> float | None:
    if len(data) != 4:
        return None
    value = struct.unpack(">f", data)[0]
    if value != value or value in (float("inf"), float("-inf")):
        return None
    return round(value, 6)


def manual_decode_frame(frame_id: int, data: bytes) -> dict[str, Any]:
    """Fallback decoder aligned with the current simple DBC."""
    if frame_id in (0x001, 0x100) and len(data) == 8:
        global_switches = data[1]
        return {
            "VehicleType": data[0],
            "SecurityUnlocked": (global_switches >> 7) & 0x01,
            "LightOff_EStopPatch": (global_switches >> 6) & 0x01,
            "DirectionReverse": (global_switches >> 5) & 0x01,
            "Throttle": data[2],
            "Winch": data[3],
            "Brake": data[4],
            "Steering": data[5],
            "SteeringModeClosedLoop": data[6] & 0x01,
            "ReservedByte7": data[7],
        }

    if frame_id == 0x2FF and len(data) == 8:
        return {
            "MainSensorTempA": struct.unpack("b", data[0:1])[0],
            "MainSensorTempB": struct.unpack("b", data[1:2])[0],
            "TachometerInstant_ticks_per_s": struct.unpack(">H", data[2:4])[0],
            "TachometerCumulative_ticks": struct.unpack(">I", data[4:8])[0],
        }

    if frame_id == 0x300 and len(data) == 8:
        return {
            "MainHardwareRevision": _safe_float32(data[0:4]),
            "MainSoftwareRevision": _safe_float32(data[4:8]),
        }

    if frame_id == 0x301 and len(data) == 8:
        return {
            "BatteryHardwareRevision": _safe_float32(data[0:4]),
            "BatterySoftwareRevision": _safe_float32(data[4:8]),
        }

    if frame_id == 0x600 and len(data) == 8:
        return {
            "CellTemp1": struct.unpack(">h", data[0:2])[0],
            "CellTemp2": struct.unpack(">h", data[2:4])[0],
            "CellTemp3": struct.unpack(">h", data[4:6])[0],
            "CellTemp4": struct.unpack(">h", data[6:8])[0],
        }

    if frame_id == 0x601 and len(data) == 8:
        return {
            "AmbientTemp": struct.unpack(">h", data[0:2])[0],
            "MosfetTemp": struct.unpack(">h", data[2:4])[0],
            "HeatpadATemp": struct.unpack(">h", data[4:6])[0],
            "HeatpadBTemp": struct.unpack(">h", data[6:8])[0],
        }

    if frame_id == 0x602 and len(data) == 8:
        heatpads_state = data[5]
        return {
            "StateOfCharge": data[0],
            "BatteryCurrent_raw": struct.unpack(">h", data[1:3])[0],
            "BatteryVoltage_raw": struct.unpack(">H", data[3:5])[0],
            "HeatpadB_On": heatpads_state & 0x01,
            "HeatpadA_On": (heatpads_state >> 1) & 0x01,
            "HeatpadsReserved": (heatpads_state >> 2) & 0x3F,
            "ChargeTimeRemaining": struct.unpack(">H", data[6:8])[0],
        }

    if frame_id == 0x603 and len(data) == 8:
        return {
            "ChargeTimeRemaining603_raw": struct.unpack(">H", data[0:2])[0],
            "YearMonth_raw": struct.unpack(">H", data[2:4])[0],
            "DayHour_raw": struct.unpack(">H", data[4:6])[0],
            "MinuteSecond_raw": struct.unpack(">H", data[6:8])[0],
        }

    if frame_id == 0x1806E5F4 and len(data) == 8:
        return {
            "MaxVoltage_raw": struct.unpack(">H", data[0:2])[0],
            "MaxCurrent_raw": struct.unpack(">H", data[2:4])[0],
            "ChargerCmdReserved": struct.unpack(">I", data[4:8])[0],
        }

    if frame_id == 0x18FF50E5 and len(data) == 8:
        return {
            "ConfiguredVoltage_raw": struct.unpack(">H", data[0:2])[0],
            "ConfiguredCurrent_raw": struct.unpack(">H", data[2:4])[0],
            "ChargerStatusReserved": struct.unpack(">I", data[4:8])[0],
        }

    return {}


def derive_metrics(frame_id: int, signals: dict[str, Any]) -> dict[str, Any]:
    """Add the tiny subset of derived values we trust enough to expose."""
    derived: dict[str, Any] = {}

    if frame_id == 0x2FF:
        ticks_per_second = signals.get("TachometerInstant_ticks_per_s")
        cumulative_ticks = signals.get("TachometerCumulative_ticks")
        if isinstance(ticks_per_second, (int, float)):
            speed_kmh = (float(ticks_per_second) / MTT_ENCODER_FINAL_RATIO) * MTT_TRACK_LENGTH_KM * 3600.0
            derived["speed_kmh_estimate"] = round(speed_kmh, 6)
            derived["speed_ms_estimate"] = round(speed_kmh / 3.6, 6)
        if isinstance(cumulative_ticks, (int, float)):
            distance_m = (float(cumulative_ticks) / MTT_ENCODER_FINAL_RATIO) * MTT_TRACK_LENGTH_M
            derived["absolute_distance_m_estimate"] = round(distance_m, 6)
    elif frame_id == 0x602:
        battery_current_raw = signals.get("BatteryCurrent_raw")
        battery_current_a = signals.get("BatteryCurrent")
        if isinstance(battery_current_raw, (int, float)) and not isinstance(battery_current_a, (int, float)):
            derived["BatteryCurrent"] = round(float(battery_current_raw) * 0.0103 - 0.72, 6)

    return derived


def decode_frame(frame_id: int, data: bytes, database: Any | None = None) -> dict[str, Any]:
    """Decode one frame with DBC when available, then add derived metrics."""
    decoded: dict[str, Any] = {}
    source = "manual"

    if database is not None:
        try:
            decoded = database.decode_message(frame_id, data, decode_choices=False)
            source = "dbc"
        except Exception:
            decoded = {}

    if not decoded:
        decoded = manual_decode_frame(frame_id, data)
        source = "manual"

    return {
        "source": source,
        "signals": decoded,
        "derived": derive_metrics(frame_id, decoded),
    }


def flatten_decoded_row(base_row: dict[str, Any], decoded_bundle: dict[str, Any]) -> dict[str, Any]:
    """Build one flat row suitable for CSV export."""
    row = {field: "" for field in CSV_FIELDS}
    for key in ("timestamp", "interface", "id_hex", "name", "is_extended", "dlc", "data_hex"):
        if key in base_row:
            row[key] = base_row[key]
    row["decode_source"] = decoded_bundle["source"]

    for source_dict in (decoded_bundle["signals"], decoded_bundle["derived"]):
        for key, value in source_dict.items():
            if key in row:
                row[key] = value

    return row


def format_age(age_seconds: float | None) -> str:
    if age_seconds is None:
        return "n/a"
    if age_seconds < 1.0:
        return f"{age_seconds * 1000.0:.0f} ms"
    return f"{age_seconds:.1f} s"
