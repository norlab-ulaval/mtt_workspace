#!/usr/bin/env python3
"""Export candump text logs to JSONL and CSV with MTT decoding."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from mtt_can_support import CSV_FIELDS
from mtt_can_support import KNOWN_MTT_IDS
from mtt_can_support import decode_frame
from mtt_can_support import flatten_decoded_row
from mtt_can_support import load_dbc
from mtt_can_support import parse_candump_line


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export candump logs to JSONL and CSV with MTT decoding")
    parser.add_argument("input", help="Input candump text file")
    parser.add_argument("--dbc", default=None, help="DBC path, default is the repo MTT simple DBC")
    parser.add_argument("--jsonl-out", default=None, help="Output JSONL path")
    parser.add_argument("--csv-out", default=None, help="Output CSV path")
    parser.add_argument("--ids", nargs="*", default=None, help="Optional CAN IDs to keep, ex: 0x602 0x2FF 0x001")
    return parser.parse_args()


def _normalize_ids(raw_ids: list[str] | None) -> set[int] | None:
    if not raw_ids:
        return None
    normalized: set[int] = set()
    for item in raw_ids:
        normalized.add(int(item, 0))
    return normalized


def _default_output(path: Path, suffix: str) -> Path:
    return path.with_suffix(suffix)


def main() -> int:
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    if not input_path.exists():
        print(f"Input file not found: {input_path}")
        return 1

    database, dbc_path = load_dbc(args.dbc)
    keep_ids = _normalize_ids(args.ids)

    jsonl_path = Path(args.jsonl_out).expanduser().resolve() if args.jsonl_out else _default_output(input_path, ".jsonl")
    csv_path = Path(args.csv_out).expanduser().resolve() if args.csv_out else _default_output(input_path, ".csv")

    rows: list[dict] = []
    parsed_count = 0
    skipped_count = 0

    with input_path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            parsed = parse_candump_line(line)
            if parsed is None:
                skipped_count += 1
                continue
            if keep_ids is not None and parsed["arbitration_id"] not in keep_ids:
                continue

            parsed_count += 1
            parsed["line_number"] = line_number
            parsed["name"] = KNOWN_MTT_IDS.get(parsed["arbitration_id"], "unknown")
            decoded_bundle = decode_frame(parsed["arbitration_id"], parsed["data"], database)

            rows.append(
                {
                    "line_number": line_number,
                    "timestamp": parsed["timestamp"],
                    "interface": parsed["interface"],
                    "arbitration_id": parsed["arbitration_id"],
                    "id_hex": parsed["id_hex"],
                    "name": parsed["name"],
                    "is_extended": parsed["is_extended"],
                    "dlc": parsed["dlc"],
                    "data_hex": parsed["data_hex"],
                    "decode_source": decoded_bundle["source"],
                    "signals": decoded_bundle["signals"],
                    "derived": decoded_bundle["derived"],
                }
            )

    jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    with jsonl_path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=True) + "\n")

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            base_row = {
                "timestamp": row["timestamp"],
                "interface": row["interface"],
                "id_hex": row["id_hex"],
                "name": row["name"],
                "is_extended": row["is_extended"],
                "dlc": row["dlc"],
                "data_hex": row["data_hex"],
            }
            flat_row = flatten_decoded_row(
                base_row=base_row,
                decoded_bundle={
                    "source": row["decode_source"],
                    "signals": row["signals"],
                    "derived": row["derived"],
                },
            )
            writer.writerow(flat_row)

    print(f"input      {input_path}")
    print(f"dbc        {dbc_path} ({'loaded' if database is not None else 'fallback-only'})")
    print(f"rows       {parsed_count}")
    print(f"skipped    {skipped_count}")
    print(f"jsonl_out  {jsonl_path}")
    print(f"csv_out    {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
