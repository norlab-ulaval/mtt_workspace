#!/usr/bin/env python3
"""Score MTT bags for WILN replay and motion-model calibration."""

import argparse
import csv
from pathlib import Path
from typing import Any, Dict, List

import yaml


def as_float(value: Any, default: float = 0.0) -> float:
    if value is None or value == "":
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_int(value: Any, default: int = 0) -> int:
    return int(as_float(value, float(default)))


def load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def load_icp_summary(path: Path) -> Dict[str, Dict[str, str]]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8", newline="") as stream:
        return {row["session"]: row for row in csv.DictReader(stream)}


def motion_row_from_summary(session: Dict[str, Any]) -> Dict[str, Any]:
    fit = session.get("fit", {}) or {}
    stats = session.get("stats", {}) or {}
    labels = stats.get("labels", {}) or {}
    return {
        "session": session.get("session", ""),
        "icp_source": session.get("icp_source", "missing"),
        "row_count": as_int(fit.get("row_count")),
        "good_icp_rows": as_int(fit.get("good_icp_rows")),
        "longitudinal_rows": as_int(fit.get("longitudinal_rows")),
        "yaw_rows": as_int(fit.get("yaw_rows")),
        "observed_motion_only_rows": as_int(fit.get("observed_motion_only_rows")),
        "trusted_for_longitudinal": bool(fit.get("trusted_for_longitudinal")),
        "trusted_for_yaw": bool(fit.get("trusted_for_yaw")),
        "speed_rmse_ms": fit.get("speed_rmse_ms"),
        "yaw_rate_rmse_rad_s": fit.get("yaw_rate_rmse_rad_s"),
        "cmd_linear_physical_sign": fit.get("cmd_linear_physical_sign"),
        "yaw_gain": fit.get("yaw_gain"),
        "icp_max_step_m": stats.get("icp_max_step_m"),
        "labels": labels,
    }


def session_motion_rows(report: Dict[str, Any], data_dir: Path) -> List[Dict[str, Any]]:
    rows = []
    for session in report.get("sessions", []):
        session_name = session.get("session", "")
        per_session_summary = data_dir / session_name / "motion_model_validation" / "model_fit_summary.yaml"
        fresh_summary = load_yaml(per_session_summary)
        rows.append(motion_row_from_summary(fresh_summary or session))
    return rows


def classify(row: Dict[str, Any]) -> Dict[str, Any]:
    coverage = as_float(row.get("icp_coverage"))
    icp_jumps = as_int(row.get("icp_jumps"))
    icp_path_m = as_float(row.get("icp_path_m"))
    good_icp_rows = as_int(row.get("good_icp_rows"))
    yaw_rows = as_int(row.get("yaw_rows"))
    longitudinal_rows = as_int(row.get("longitudinal_rows"))
    icp_max_step_m = as_float(row.get("icp_max_step_m"))
    trusted_long = bool(row.get("trusted_for_longitudinal"))
    trusted_yaw = bool(row.get("trusted_for_yaw"))

    reasons: List[str] = []
    can_test_wiln = (
        coverage >= 0.60
        and good_icp_rows >= 1000
        and icp_path_m >= 5.0
        and icp_jumps <= 30
        and icp_max_step_m <= 0.75
    )
    can_fit_long = trusted_long and longitudinal_rows >= 100
    can_fit_yaw = trusted_yaw and yaw_rows >= 500 and icp_max_step_m <= 1.0

    if can_test_wiln:
        reasons.append("icp_route_usable")
    if can_fit_long:
        reasons.append("longitudinal_model_usable")
    if can_fit_yaw:
        reasons.append("steering_yaw_model_usable")
    if icp_path_m < 1.0 and good_icp_rows > 0:
        reasons.append("static_or_tiny_motion")
    if coverage < 0.60 or good_icp_rows < 1000:
        reasons.append("insufficient_icp")
    if icp_jumps > 30 or icp_max_step_m > 1.0:
        reasons.append("icp_jumpy")

    if can_test_wiln and can_fit_yaw:
        grade = "wiln_steering_candidate"
    elif can_test_wiln:
        grade = "wiln_route_candidate"
    elif can_fit_long or can_fit_yaw:
        grade = "model_calibration_only"
    elif "static_or_tiny_motion" in reasons:
        grade = "static_check_only"
    else:
        grade = "icp_debug_only"

    return {
        "grade": grade,
        "can_test_wiln": can_test_wiln,
        "can_fit_longitudinal": can_fit_long,
        "can_fit_yaw": can_fit_yaw,
        "reasons": reasons,
    }


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "session",
        "grade",
        "can_test_wiln",
        "can_fit_longitudinal",
        "can_fit_yaw",
        "icp_coverage",
        "icp_path_m",
        "icp_jumps",
        "icp_max_step_m",
        "good_icp_rows",
        "longitudinal_rows",
        "yaw_rows",
        "speed_rmse_ms",
        "yaw_rate_rmse_rad_s",
        "cmd_linear_physical_sign",
        "yaw_gain",
        "reasons",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data_dir", nargs="?", default="data", type=Path)
    parser.add_argument("--report-name", default="wiln_readiness_report.yaml")
    parser.add_argument("--csv-name", default="wiln_readiness_summary.csv")
    args = parser.parse_args()

    data_dir = args.data_dir
    motion_report = load_yaml(data_dir / "motion_model_fit_report.yaml")
    icp_rows = load_icp_summary(data_dir / "icp_investigation_summary.csv")

    rows: List[Dict[str, Any]] = []
    for motion_row in session_motion_rows(motion_report, data_dir):
        session = motion_row["session"]
        icp_row = icp_rows.get(session, {})
        row: Dict[str, Any] = {
            **motion_row,
            "icp_grade": icp_row.get("grade", ""),
            "icp_coverage": as_float(icp_row.get("icp_coverage")),
            "icp_path_m": as_float(icp_row.get("icp_path_m")),
            "icp_jumps": as_int(icp_row.get("icp_jumps")),
            "icp_gaps_over_1s": as_int(icp_row.get("icp_gaps_over_1s")),
        }
        row.update(classify(row))
        row["reasons"] = ";".join(row["reasons"])
        rows.append(row)

    grade_order = {
        "wiln_steering_candidate": 0,
        "wiln_route_candidate": 1,
        "model_calibration_only": 2,
        "static_check_only": 3,
        "icp_debug_only": 4,
    }
    rows.sort(
        key=lambda row: (
            grade_order.get(row["grade"], 99),
            -as_float(row.get("icp_path_m")),
            -as_int(row.get("yaw_rows")),
        )
    )

    report = {
        "inputs": {
            "motion_model_fit_report": str(data_dir / "motion_model_fit_report.yaml"),
            "icp_investigation_summary": str(data_dir / "icp_investigation_summary.csv"),
        },
        "counts": {},
        "recommended_sequence": [
            row["session"]
            for row in rows
            if row["grade"] in {"wiln_steering_candidate", "wiln_route_candidate"}
        ][:5],
        "sessions": rows,
    }
    for row in rows:
        report["counts"][row["grade"]] = report["counts"].get(row["grade"], 0) + 1

    yaml_path = data_dir / args.report_name
    csv_path = data_dir / args.csv_name
    yaml_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    write_csv(csv_path, rows)

    print(f"WILN readiness report: {yaml_path}")
    print(f"WILN readiness summary: {csv_path}")
    for session in report["recommended_sequence"]:
        print(f"  candidate: {session}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
