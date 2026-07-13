#!/usr/bin/env python3
"""K-fold tuning for MTT command/state motion-model parameters."""

from __future__ import annotations

import argparse
import itertools
import math
from pathlib import Path

import yaml

from lib.mtt_motion_research import (
    ModelParams,
    assign_segments,
    derive_research_rows,
    find_motion_csv,
    fit_residual_model,
    parse_bool,
    prediction_errors,
    read_csv_rows,
    resolve_sessions,
    rmse,
    write_csv_rows,
)


def load_rows(sessions: list[Path]) -> list[dict]:
    rows: list[dict] = []
    for session in sessions:
        research_csv = session / "motion_research" / "dataset.csv"
        if research_csv.exists():
            rows.extend(read_csv_rows(research_csv))
        else:
            rows.extend(assign_segments(derive_research_rows(read_csv_rows(find_motion_csv(session)), session_name=session.name)))
    return rows


def split_folds(rows: list[dict], k: int, mode: str) -> list[tuple[list[dict], list[dict]]]:
    if mode == "session":
        keys = sorted({str(row.get("session", "")) for row in rows})
        fold_keys = [set(keys[i::k]) for i in range(k)]
        return [([r for r in rows if str(r.get("session", "")) not in keys_i], [r for r in rows if str(r.get("session", "")) in keys_i]) for keys_i in fold_keys]
    keys = sorted({(str(row.get("session", "")), str(row.get("segment_id", ""))) for row in rows})
    fold_keys = [set(keys[i::k]) for i in range(k)]
    return [
        (
            [r for r in rows if (str(r.get("session", "")), str(r.get("segment_id", ""))) not in keys_i],
            [r for r in rows if (str(r.get("session", "")), str(r.get("segment_id", ""))) in keys_i],
        )
        for keys_i in fold_keys
    ]


def score_errors(errors: list[dict], horizon_s: float) -> float:
    vals = [float(row["position_error_m"]) for row in errors if abs(float(row["horizon_s"]) - horizon_s) < 1e-9]
    scored = rmse(vals)
    return scored if scored is not None else float("inf")


def parse_float_grid(text: str) -> list[float]:
    return [float(x) for x in text.split(",") if x.strip()]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/results/motion_research/kfold"))
    parser.add_argument("--k", type=int, default=5)
    parser.add_argument("--fold-mode", choices=["segment", "session"], default="segment")
    parser.add_argument("--horizon-s", type=float, default=3.0)
    parser.add_argument("--speed-scales", default="0.8,0.9,1.0,1.1,1.2")
    parser.add_argument("--yaw-gains", default="0.7,0.85,1.0,1.15,1.3")
    parser.add_argument("--tau-v", default="0.2,0.5,0.8,1.2,1.8")
    parser.add_argument("--tau-phi", default="0.2,0.5,0.8,1.2,1.8")
    parser.add_argument("--sample-period-s", type=float, default=1.0)
    parser.add_argument("--allow-synthetic-tacho", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sessions = resolve_sessions(args.path)
    rows = [row for row in load_rows(sessions) if parse_bool(row.get("research_quality_ok"))]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    folds = split_folds(rows, max(2, args.k), args.fold_mode)
    m2_features = ["bias", "abs_v", "phi", "abs_phi", "phi_dot", "abs_phi_dot"]
    grid = list(
        itertools.product(
            parse_float_grid(args.speed_scales),
            parse_float_grid(args.yaw_gains),
            parse_float_grid(args.tau_v),
            parse_float_grid(args.tau_phi),
        )
    )
    results = []
    for speed_scale, yaw_gain, tau_v, tau_phi in grid:
        fold_scores = []
        fold_rows = 0
        for train, test in folds:
            coeff = fit_residual_model(train, m2_features)
            model = ModelParams(
                "m2_cv",
                "m2",
                "cmd",
                "cmd",
                "exact",
                speed_scale=speed_scale,
                yaw_gain=yaw_gain,
                tau_v_s=tau_v,
                tau_phi_s=tau_phi,
                residual_features=m2_features,
                residual_coefficients=coeff,
            )
            errors = prediction_errors(test, model, horizons_s=[args.horizon_s], sample_period_s=args.sample_period_s, require_real_tacho=False)
            fold_scores.append(score_errors(errors, args.horizon_s))
            fold_rows += len(errors)
        finite = [s for s in fold_scores if math.isfinite(s)]
        results.append(
            {
                "speed_scale": speed_scale,
                "yaw_gain": yaw_gain,
                "tau_v_s": tau_v,
                "tau_phi_s": tau_phi,
                "fold_scores": ";".join(f"{s:.6g}" for s in fold_scores),
                "test_prediction_rows": fold_rows,
                "mean_rmse_m": sum(finite) / len(finite) if finite else float("inf"),
                "max_rmse_m": max(finite) if finite else float("inf"),
            }
        )
    results.sort(key=lambda row: (float(row["mean_rmse_m"]), float(row["max_rmse_m"])))
    write_csv_rows(args.output_dir / "kfold_grid_results.csv", results)
    summary = {
        "sessions": [s.name for s in sessions],
        "rows": len(rows),
        "folds": len(folds),
        "fold_mode": args.fold_mode,
        "horizon_s": args.horizon_s,
        "best": results[0] if results else None,
        "outputs": {"grid_csv": str(args.output_dir / "kfold_grid_results.csv")},
    }
    (args.output_dir / "kfold_summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
