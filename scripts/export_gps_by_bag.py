#!/usr/bin/env python3
"""Export external RSplus GPS LLH logs aligned to MTT bag time windows.

This script does not modify bags. It reads bag metadata timestamps, scans
RSplus ZIP/.LLH exports, and writes per-session CSV/YAML files for inspection
and later fusion.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import math
import re
import statistics
import sys
from pathlib import Path
from typing import Any
from zoneinfo import ZoneInfo
from zipfile import ZipFile

import yaml


GPS_CANDIDATE_DIRS = (
    Path("/data/GPS"),
    Path("/data/mtt_bags/GPS"),
    Path("data/GPS"),
)

LOCAL_TZ = ZoneInfo("America/Toronto")

CSV_FIELDS = [
    "t",
    "stamp_utc",
    "bag_time_offset_s",
    "latitude",
    "longitude",
    "altitude",
    "fix_status_raw",
    "satellites",
    "sigma_e",
    "sigma_n",
    "sigma_u",
    "source_zip",
    "source_member",
    "quality_label",
]


def infer_workspace_root(script_path: Path) -> Path:
    for candidate in [script_path.parent, *script_path.parents]:
        if (candidate / "src").exists() and (candidate / "demos").exists():
            return candidate
    return script_path.parent


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


def unix_to_utc_text(t: float | None) -> str | None:
    if t is None or not math.isfinite(t):
        return None
    return datetime.fromtimestamp(t, tz=timezone.utc).isoformat().replace("+00:00", "Z")


def unix_to_date_text(t: float | None, tz: timezone | ZoneInfo = timezone.utc) -> str | None:
    if t is None or not math.isfinite(t):
        return None
    return datetime.fromtimestamp(t, tz=tz).date().isoformat()


def compact_date_to_iso(value: str) -> str | None:
    if not re.match(r"^\d{8}$", value):
        return None
    return f"{value[0:4]}-{value[4:6]}-{value[6:8]}"


def date_from_name(text: str) -> str | None:
    match = re.search(r"(20\d{6})", text)
    return compact_date_to_iso(match.group(1)) if match else None


def session_date_from_name(session_dir: Path) -> str | None:
    match = re.search(r"_(20\d{2}-\d{2}-\d{2})_", session_dir.name)
    return match.group(1) if match else None


def date_match_reasons(
    session_date: str | None,
    bag_utc_date: str | None,
    bag_local_date: str | None,
    file_name_date: str | None,
    gps_utc_date: str | None,
    gps_local_date: str | None,
) -> list[str]:
    reasons: list[str] = []
    if session_date and file_name_date and session_date == file_name_date:
        reasons.append("session_name_file_name")
    if bag_utc_date and gps_utc_date and bag_utc_date == gps_utc_date:
        reasons.append("bag_utc_gps_utc")
    if bag_local_date and gps_local_date and bag_local_date == gps_local_date:
        reasons.append("bag_local_gps_local")
    if session_date and gps_utc_date and session_date == gps_utc_date:
        reasons.append("session_name_gps_utc")
    if session_date and gps_local_date and session_date == gps_local_date:
        reasons.append("session_name_gps_local")
    return reasons


def load_bag_time_window(session_dir: Path) -> tuple[float | None, float | None, dict[str, Any]]:
    metadata_path = session_dir / "bag" / "metadata.yaml"
    if not metadata_path.exists():
        return None, None, {"metadata_path": str(metadata_path), "error": "missing_metadata"}

    data = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    info = data.get("rosbag2_bagfile_information", data)
    start_ns = info.get("starting_time", {}).get("nanoseconds_since_epoch")
    duration_ns = info.get("duration", {}).get("nanoseconds")
    if start_ns is None or duration_ns is None:
        return None, None, {
            "metadata_path": str(metadata_path),
            "error": "missing_start_or_duration",
        }

    start = float(start_ns) / 1e9
    duration = float(duration_ns) / 1e9
    return start, start + duration, {
        "metadata_path": str(metadata_path),
        "duration_s": duration,
        "message_count": int(info.get("message_count", 0)),
    }


def parse_llh_time(date_text: str, time_text: str) -> float:
    dt = datetime.strptime(f"{date_text} {time_text}", "%Y/%m/%d %H:%M:%S.%f")
    return dt.replace(tzinfo=timezone.utc).timestamp()


def parse_llh_line(line: str) -> dict[str, Any] | None:
    parts = line.split()
    if len(parts) < 6 or not re.match(r"^\d{4}/\d{2}/\d{2}$", parts[0]):
        return None
    try:
        return {
            "t": parse_llh_time(parts[0], parts[1]),
            "latitude": float(parts[2]),
            "longitude": float(parts[3]),
            "altitude": float(parts[4]),
            "fix_status_raw": int(float(parts[5])),
            "satellites": int(float(parts[6])) if len(parts) > 6 else None,
            "sigma_e": float(parts[7]) if len(parts) > 7 else None,
            "sigma_n": float(parts[8]) if len(parts) > 8 else None,
            "sigma_u": float(parts[9]) if len(parts) > 9 else None,
        }
    except ValueError:
        return None


def iter_llh_files(gps_dir: Path) -> list[tuple[Path, str | None]]:
    if not gps_dir.exists():
        return []
    direct = [(path, None) for path in sorted(gps_dir.glob("*.LLH"))]
    zipped: list[tuple[Path, str | None]] = []
    for zip_path in sorted(gps_dir.glob("*.zip")):
        try:
            with ZipFile(zip_path) as archive:
                for name in archive.namelist():
                    if name.upper().endswith(".LLH"):
                        zipped.append((zip_path, name))
        except Exception as exc:
            print(f"warning: cannot inspect GPS zip {zip_path}: {exc}", file=sys.stderr)
    return direct + zipped


def load_llh_rows(path: Path, member: str | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    try:
        if member:
            with ZipFile(path) as archive:
                with archive.open(member) as stream:
                    for raw in stream:
                        row = parse_llh_line(raw.decode("utf-8", errors="ignore"))
                        if row:
                            rows.append(row)
        else:
            with path.open("r", encoding="utf-8", errors="ignore") as stream:
                for line in stream:
                    row = parse_llh_line(line)
                    if row:
                        rows.append(row)
    except Exception as exc:
        print(f"warning: cannot read GPS LLH {path}: {exc}", file=sys.stderr)

    rows.sort(key=lambda row: float(row["t"]))
    return rows


def choose_gps_dir(workspace_root: Path, gps_log_dir: str) -> Path | None:
    if gps_log_dir:
        return Path(gps_log_dir).expanduser().resolve()
    for candidate in GPS_CANDIDATE_DIRS:
        path = candidate if candidate.is_absolute() else workspace_root / candidate
        if path.exists():
            return path
    return None


def median_dt(rows: list[dict[str, Any]]) -> float | None:
    if len(rows) < 2:
        return None
    dts = [
        float(rows[i]["t"]) - float(rows[i - 1]["t"])
        for i in range(1, len(rows))
        if float(rows[i]["t"]) >= float(rows[i - 1]["t"])
    ]
    return statistics.median(dts) if dts else None


def value_stats(values: list[float]) -> dict[str, float | None]:
    valid = [v for v in values if v is not None and math.isfinite(v)]
    if not valid:
        return {"min": None, "median": None, "max": None}
    return {"min": min(valid), "median": statistics.median(valid), "max": max(valid)}


def classify_quality(
    selected: list[dict[str, Any]],
    bag_start: float,
    bag_end: float,
    margin_s: float,
    non_monotonic_count: int,
) -> str:
    if non_monotonic_count:
        return "bad_time"
    if not selected:
        return "missing"

    in_bag = [row for row in selected if bag_start <= float(row["t"]) <= bag_end]
    if not in_bag:
        return "partial"

    first = float(in_bag[0]["t"])
    last = float(in_bag[-1]["t"])
    start_gap = max(0.0, first - bag_start)
    end_gap = max(0.0, bag_end - last)
    dts = [float(in_bag[i]["t"]) - float(in_bag[i - 1]["t"]) for i in range(1, len(in_bag))]
    max_gap = max(dts) if dts else 0.0
    if start_gap <= margin_s and end_gap <= margin_s and max_gap <= 1.0:
        return "ok"
    return "partial"


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def process_session(
    session_dir: Path,
    gps_files: list[tuple[Path, str | None]],
    gps_rows_cache: dict[tuple[Path, str | None], list[dict[str, Any]]],
    gps_dir: Path | None,
    margin_s: float,
) -> dict[str, Any]:
    output_dir = session_dir / "gps_export"
    output_dir.mkdir(parents=True, exist_ok=True)

    bag_start, bag_end, bag_info = load_bag_time_window(session_dir)
    result: dict[str, Any] = {
        "session": session_dir.name,
        "session_dir": str(session_dir),
        "gps_dir": str(gps_dir) if gps_dir else None,
        "status": "failed",
        "bag_start": bag_start,
        "bag_start_utc": unix_to_utc_text(bag_start),
        "bag_start_date_utc": unix_to_date_text(bag_start),
        "bag_start_date_local": unix_to_date_text(bag_start, LOCAL_TZ),
        "session_date_from_name": session_date_from_name(session_dir),
        "bag_end": bag_end,
        "bag_end_utc": unix_to_utc_text(bag_end),
        "bag_end_date_utc": unix_to_date_text(bag_end),
        "bag_end_date_local": unix_to_date_text(bag_end, LOCAL_TZ),
        "bag_info": bag_info,
        "margin_s": margin_s,
        "candidates_yaml": str(output_dir / "candidates.yaml"),
        "export_csv": str(output_dir / "external_gps_llh.csv"),
    }

    if bag_start is None or bag_end is None:
        result["status"] = "missing_bag_time"
        (output_dir / "summary.yaml").write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
        return result

    selected: list[dict[str, Any]] = []
    candidates: list[dict[str, Any]] = []
    non_monotonic_count = 0
    same_date_candidate_count = 0
    best_same_date_candidate: dict[str, Any] | None = None

    for path, member in gps_files:
        key = (path, member)
        rows = gps_rows_cache.setdefault(key, load_llh_rows(path, member))
        if not rows:
            continue
        file_non_monotonic = sum(
            1 for i in range(1, len(rows)) if float(rows[i]["t"]) < float(rows[i - 1]["t"])
        )
        non_monotonic_count += file_non_monotonic
        start = float(rows[0]["t"])
        end = float(rows[-1]["t"])
        overlap = max(0.0, min(end, bag_end + margin_s) - max(start, bag_start - margin_s))
        file_name_date = date_from_name(path.name) or (date_from_name(member) if member else None)
        gps_start_date_utc = unix_to_date_text(start)
        gps_start_date_local = unix_to_date_text(start, LOCAL_TZ)
        reasons = date_match_reasons(
            result["session_date_from_name"],
            result["bag_start_date_utc"],
            result["bag_start_date_local"],
            file_name_date,
            gps_start_date_utc,
            gps_start_date_local,
        )
        same_date = bool(reasons)
        if end < bag_start:
            nearest_gap_s = bag_start - end
        elif start > bag_end:
            nearest_gap_s = start - bag_end
        else:
            nearest_gap_s = 0.0
        candidate = {
            "path": str(path),
            "member": member,
            "file_name_date": file_name_date,
            "start": start,
            "start_utc": unix_to_utc_text(start),
            "start_date_utc": gps_start_date_utc,
            "start_date_local": gps_start_date_local,
            "end": end,
            "end_utc": unix_to_utc_text(end),
            "end_date_utc": unix_to_date_text(end),
            "end_date_local": unix_to_date_text(end, LOCAL_TZ),
            "samples": len(rows),
            "median_dt_s": median_dt(rows),
            "overlap_s": overlap,
            "nearest_gap_s": nearest_gap_s,
            "same_date": same_date,
            "date_match_reasons": reasons,
            "non_monotonic_count": file_non_monotonic,
        }
        candidates.append(candidate)
        if same_date:
            same_date_candidate_count += 1
            if best_same_date_candidate is None:
                best_same_date_candidate = candidate
            else:
                current_key = (float(candidate["nearest_gap_s"]), -float(candidate["overlap_s"]))
                best_key = (
                    float(best_same_date_candidate["nearest_gap_s"]),
                    -float(best_same_date_candidate["overlap_s"]),
                )
                if current_key < best_key:
                    best_same_date_candidate = candidate

        if overlap <= 0.0:
            continue
        for row in rows:
            t = float(row["t"])
            if bag_start - margin_s <= t <= bag_end + margin_s:
                out = {
                    "t": f"{t:.9f}",
                    "stamp_utc": unix_to_utc_text(t),
                    "bag_time_offset_s": f"{t - bag_start:.9f}",
                    "latitude": f"{float(row['latitude']):.9f}",
                    "longitude": f"{float(row['longitude']):.9f}",
                    "altitude": f"{float(row['altitude']):.4f}",
                    "fix_status_raw": row["fix_status_raw"],
                    "satellites": row["satellites"] if row["satellites"] is not None else "",
                    "sigma_e": row["sigma_e"] if row["sigma_e"] is not None else "",
                    "sigma_n": row["sigma_n"] if row["sigma_n"] is not None else "",
                    "sigma_u": row["sigma_u"] if row["sigma_u"] is not None else "",
                    "source_zip": path.name,
                    "source_member": member or "",
                    "quality_label": "",
                }
                selected.append(out)

    selected.sort(key=lambda row: float(row["t"]))
    quality_label = classify_quality(selected, bag_start, bag_end, margin_s, non_monotonic_count)
    for row in selected:
        row["quality_label"] = quality_label

    in_bag = [row for row in selected if 0.0 <= float(row["bag_time_offset_s"]) <= (bag_end - bag_start)]
    dts = [float(in_bag[i]["t"]) - float(in_bag[i - 1]["t"]) for i in range(1, len(in_bag))]
    status_counts: dict[int, int] = {}
    for row in selected:
        status = int(row["fix_status_raw"])
        status_counts[status] = status_counts.get(status, 0) + 1

    summary = {
        **result,
        "status": quality_label,
        "candidate_files": len(candidates),
        "same_date_candidate_files": same_date_candidate_count,
        "best_same_date_candidate": best_same_date_candidate,
        "selected_samples": len(selected),
        "in_bag_samples": len(in_bag),
        "gps_start": float(selected[0]["t"]) if selected else None,
        "gps_start_utc": selected[0]["stamp_utc"] if selected else None,
        "gps_end": float(selected[-1]["t"]) if selected else None,
        "gps_end_utc": selected[-1]["stamp_utc"] if selected else None,
        "median_dt_s": statistics.median(dts) if dts else None,
        "max_gap_s": max(dts) if dts else None,
        "gaps_gt_1s": sum(1 for dt in dts if dt > 1.0),
        "fix_status_counts": dict(sorted(status_counts.items())),
        "satellites": value_stats([float(row["satellites"]) for row in selected if row["satellites"] != ""]),
        "latitude": value_stats([float(row["latitude"]) for row in selected]),
        "longitude": value_stats([float(row["longitude"]) for row in selected]),
        "altitude": value_stats([float(row["altitude"]) for row in selected]),
        "non_monotonic_count": non_monotonic_count,
    }

    write_csv(selected, output_dir / "external_gps_llh.csv")
    (output_dir / "candidates.yaml").write_text(yaml.safe_dump(candidates, sort_keys=False), encoding="utf-8")
    (output_dir / "summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    return summary


def parse_args(workspace_root: Path) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export RSplus GPS LLH logs aligned to MTT bag timestamps.")
    parser.add_argument("input_path", nargs="?", default=str(workspace_root / "data"))
    parser.add_argument("--gps-log-dir", default="", help="Directory containing RSplus .LLH files or ZIP exports.")
    parser.add_argument("--margin-s", type=float, default=30.0, help="Time margin around each bag window.")
    return parser.parse_args()


def main() -> int:
    workspace_root = infer_workspace_root(Path(__file__).resolve())
    args = parse_args(workspace_root)
    sessions = resolve_sessions(args.input_path)
    gps_dir = choose_gps_dir(workspace_root, args.gps_log_dir)
    gps_files = iter_llh_files(gps_dir) if gps_dir else []
    gps_rows_cache: dict[tuple[Path, str | None], list[dict[str, Any]]] = {}
    report: list[dict[str, Any]] = []

    if not gps_files:
        print(f"warning: no LLH files found in {gps_dir}", file=sys.stderr)

    for index, session_dir in enumerate(sessions, start=1):
        print(f"[{index}/{len(sessions)}] {session_dir.name}")
        try:
            result = process_session(session_dir, gps_files, gps_rows_cache, gps_dir, args.margin_s)
        except Exception as exc:
            result = {
                "session": session_dir.name,
                "session_dir": str(session_dir),
                "gps_dir": str(gps_dir) if gps_dir else None,
                "status": "failed_exception",
                "error": str(exc),
            }
        report.append(result)
        print(f"  {result['status']} selected={result.get('selected_samples', 0)}")

    report_path = workspace_root / "data" / "gps_export_report.yaml"
    report_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    print(f"Report: {report_path}")
    return 1 if any(item.get("status") in {"failed", "failed_exception", "bad_time"} for item in report) else 0


if __name__ == "__main__":
    raise SystemExit(main())
