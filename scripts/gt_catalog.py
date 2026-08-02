#!/usr/bin/env python3
"""dataset_catalog.csv read/write helpers for build_gt_pipeline.py. Treats
/data/mtt_bags/dataset_catalog.csv as authoritative -- it lives alongside
the actual dataset files this catalog describes. Does NOT touch the repo's
documentations/dataset_catalog.csv snapshot copy; keep that synced by hand.
"""
from __future__ import annotations

import csv
from pathlib import Path


def catalog_row_status(catalog_path: Path, session_id: str) -> str | None:
    """Returns the existing canonical_status for session_id, or None if the
    session has no row yet."""
    if not catalog_path.exists():
        return None
    with catalog_path.open(newline="") as f:
        for row in csv.DictReader(f):
            if row.get("session_id") == session_id:
                return row.get("canonical_status")
    return None


def assert_not_frozen(catalog_path: Path, session_id: str, force: bool) -> None:
    status = catalog_row_status(catalog_path, session_id)
    if status == "ready" and not force:
        raise RuntimeError(
            f"session_id={session_id!r} is already canonical_status=ready in "
            f"{catalog_path} -- refusing to overwrite a frozen dataset. This "
            "protects Campus/Ice-rink (and any future frozen session) from an "
            "accidental re-run. Pass an explicit override if you really intend "
            "to replace an already-ready dataset (not exposed as a CLI flag by "
            "default -- see build_gt_pipeline.py's --force-overwrite-ready).")


def upsert_catalog_row(catalog_path: Path, row: dict) -> None:
    """Replaces the row matching row['session_id'] if present, else appends.
    Preserves the existing column order/set from the file on disk -- never
    silently drops or reorders columns another tool relies on."""
    existing_rows: list[dict] = []
    fieldnames: list[str] = []
    if catalog_path.exists():
        with catalog_path.open(newline="") as f:
            reader = csv.DictReader(f)
            fieldnames = list(reader.fieldnames or [])
            existing_rows = list(reader)
    for key in row:
        if key not in fieldnames:
            fieldnames.append(key)
    replaced = False
    for i, existing in enumerate(existing_rows):
        if existing.get("session_id") == row.get("session_id"):
            existing_rows[i] = {**existing, **row}
            replaced = True
            break
    if not replaced:
        existing_rows.append(row)
    tmp_path = catalog_path.with_suffix(catalog_path.suffix + ".tmp")
    with tmp_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in existing_rows:
            writer.writerow(r)
    tmp_path.replace(catalog_path)  # atomic on the same filesystem
