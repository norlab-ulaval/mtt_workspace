#!/usr/bin/env python3
"""Shared provenance helpers for GT dataset packages -- generalizes the
sha256()/file_record()/manifest.yaml pattern already used by
build_msa_canonical_dataset.py so build_gt_pipeline.py does not reimplement
it. If you are tempted to add a second hashing helper anywhere in this
scripts/ tree, import from here instead.
"""
from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path
from typing import Any


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def file_record(path: Path) -> dict[str, Any]:
    return {
        "path": str(path),
        "sha256": sha256(path),
        "bytes": path.stat().st_size,
    }


def git_commit_hash(repo_root: Path) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo_root), "rev-parse", "HEAD"],
        capture_output=True, text=True, check=True)
    return result.stdout.strip()


def git_is_dirty(repo_root: Path) -> bool:
    result = subprocess.run(
        ["git", "-C", str(repo_root), "status", "--porcelain"],
        capture_output=True, text=True, check=True)
    return bool(result.stdout.strip())
