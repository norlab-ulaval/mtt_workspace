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
    # Some deployments (e.g. an rsync'd copy on a robot, not a git clone)
    # have no .git at all -- record that plainly instead of crashing the
    # whole pipeline at the very last stage after everything else succeeded.
    result = subprocess.run(
        ["git", "-C", str(repo_root), "rev-parse", "HEAD"],
        capture_output=True, text=True)
    if result.returncode != 0:
        return "unknown (not a git repository)"
    return result.stdout.strip()


def git_is_dirty(repo_root: Path) -> bool | None:
    result = subprocess.run(
        ["git", "-C", str(repo_root), "status", "--porcelain"],
        capture_output=True, text=True)
    if result.returncode != 0:
        return None
    return bool(result.stdout.strip())
