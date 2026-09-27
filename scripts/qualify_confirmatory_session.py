#!/usr/bin/env python3
"""Check required recording topics against an explicit acquisition contract.

Uses bag metadata for presence/counts and audit_bag_timing for timing metrics.
Writes a JSON report and returns nonzero when a required topic or timing check
fails. Optional channels do not determine the result. Run inside the ROS image
when deserialization is required.

This field check does not replace offline scientific qualification against the
canonical dataset in the research repository.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_bag_topics import load_bag_metadata, resolve_bag_dir  # noqa: E402
from audit_bag_timing import compute_topic_stats  # noqa: E402

import yaml  # noqa: E402

def load_required_topics(contract_path: Path) -> list[dict[str, Any]]:
    doc = yaml.safe_load(contract_path.read_text())
    topics = [t for t in doc["topics"] if t.get("required")]
    validate_required_topics(topics)
    return topics


def validate_required_topics(topics: list[dict[str, Any]]) -> None:
    if not topics:
        raise ValueError("topic contract must contain at least one required topic")
    names = [t["topic"] for t in topics]
    if len(set(names)) != len(names) or any(not isinstance(n, str) or not n.startswith("/") for n in names):
        raise ValueError("required topic names must be unique absolute ROS names")
    for entry in topics:
        for key in ("hard_min_hz", "max_gap_s"):
            value = entry.get(key)
            if value is not None and (not math.isfinite(float(value)) or float(value) <= 0):
                raise ValueError(f"{entry['topic']}: {key} must be finite and positive")


def qualify(bag_dir: Path, required_topics: list[dict[str, Any]]) -> dict[str, Any]:
    validate_required_topics(required_topics)
    metadata_path = bag_dir / "metadata.yaml"
    counts, duration_ns, total_messages = load_bag_metadata(metadata_path)
    duration_s = duration_ns / 1e9 if duration_ns else 0.0

    timed_topics = {t["topic"] for t in required_topics if t.get("hard_min_hz") or t.get("max_gap_s")}
    stats, _map_samples = (
        compute_topic_stats(bag_dir, timed_topics) if timed_topics else ({}, [])
    )

    results = []
    overall_pass = True
    for entry in required_topics:
        topic = entry["topic"]
        count = counts.get(topic, 0)
        present = topic in counts
        nonzero = count > 0
        row = {
            "topic": topic,
            "role": entry.get("role", ""),
            "present": present,
            "message_count": count,
            "checks": {},
        }
        if not present:
            row["verdict"] = "FAIL: topic missing from bag metadata"
            overall_pass = False
        elif not nonzero:
            row["verdict"] = "FAIL: topic present but zero messages"
            overall_pass = False
        else:
            verdict = "PASS"
            row_stats = stats.get(topic)
            if topic in timed_topics and (not row_stats or row_stats["n"] < 2):
                verdict = "FAIL: insufficient timing samples"
                overall_pass = False
            elif topic in timed_topics:
                first, last = row_stats["first"], row_stats["last"]
                span = max(0.0, last - first)
                hz = (row_stats["n"] - 1) / span if span > 0 else 0.0
                max_gap = max(row_stats["gaps"]) if row_stats["gaps"] else 0.0
                row["checks"]["measured_hz"] = round(hz, 2)
                row["checks"]["measured_max_gap_s"] = round(max_gap, 3)
                if (not math.isfinite(span) or span <= 0
                        or not row_stats["gaps"]
                        or any(not math.isfinite(g) or g <= 0 for g in row_stats["gaps"])):
                    verdict = "FAIL: nonfinite or non-increasing timestamps"
                    overall_pass = False
                min_hz = entry.get("hard_min_hz")
                max_gap_limit = entry.get("max_gap_s")
                if min_hz is not None and hz < min_hz:
                    verdict = f"FAIL: measured {hz:.2f} Hz below hard_min_hz={min_hz}"
                    overall_pass = False
                if max_gap_limit is not None and max_gap > max_gap_limit:
                    verdict = (
                        f"FAIL: measured max gap {max_gap:.3f}s exceeds max_gap_s={max_gap_limit}"
                    )
                    overall_pass = False
            row["verdict"] = verdict
        results.append(row)

    return {
        "bag_dir": str(bag_dir),
        "duration_s": round(duration_s, 1),
        "total_messages": total_messages,
        "required_topic_count": len(required_topics),
        "overall_pass": overall_pass,
        "topics": results,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", help="Session directory, bag directory, or bag_0.mcap path.")
    parser.add_argument("--topic-contract", type=Path, required=True,
                        help="Explicit topic contract YAML from the research repository.")
    parser.add_argument("--out", type=Path, default=None, help="Write JSON report here (default: <session>/confirmatory_qualification.json).")
    args = parser.parse_args()

    if not args.topic_contract.is_file():
        print(f"ERROR: topic contract not found: {args.topic_contract}", file=sys.stderr)
        return 2

    bag_dir = resolve_bag_dir(Path(args.session))
    required = load_required_topics(args.topic_contract)
    report = qualify(bag_dir, required)

    out_path = args.out or (bag_dir.parent / "confirmatory_qualification.json")
    out_path.write_text(json.dumps(report, indent=2))

    print(f"Bag: {bag_dir}")
    print(f"Duration: {report['duration_s']}s  Total messages: {report['total_messages']}")
    print(f"Required topics: {report['required_topic_count']}")
    for row in report["topics"]:
        print(f"  [{row['verdict']}] {row['topic']}  (n={row['message_count']})  {row.get('checks', {})}")
    print()
    print(f"OVERALL: {'PASS' if report['overall_pass'] else 'FAIL'}")
    print(f"Report written to: {out_path}")

    return 0 if report["overall_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
