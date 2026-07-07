"""
compare_all_models.py

Fit M0 / M2 / VL (fixed & free) / M5 (L0-L2) / M5+hysteresis on a
real bag CSV and report RMS + AIC.  Produces a kappa-vs-phi overlay figure
with all model curves — the "model ladder" for the ICRA paper.

Usage:
    python compare_all_models.py <path/to/dataset.csv> [--save <path>.png]

Dependencies: numpy, matplotlib, and the motion_model library in this folder.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from motion_model import (DEG, VehicleParams, M2Params, M5ContactParams,
                          VirtualLengthParams, kappa_nom_front,
                          M5HysteresisPlant, set_m5_params)
from m5_identify import (BagData, identify_m5, identify_virtual_length,
                         identify_hysteresis)

_MODEL_COLORS = {
    "M0 (nominal)": "#C44E52",
    "M2 (gamma)": "#DD8452",
    "VL (fixed ratio)": "#55A868",
    "VL (free l1,l2)": "#4C72B0",
    "M5 L0": "#937860",
    "M5 L1": "#8172B2",
    "M5 L2": "#CCB974",
    "M5+hysteresis": "#E6A8D7",
}

# ---- CSV loader -----------------------------------------------------------


def load_csv(path: str, v_min: float = 0.25,
             kappa_max: float = 2.0) -> BagData:
    """Load a motion_model_dataset.csv into a BagData, filtering bad rows."""
    rows: List[Dict[str, str]] = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)

    t, phi, theta, v, quality = [], [], [], [], []
    q_ok_col = r.get("research_quality_ok", "0")
    q_ok = q_ok_col in ("True", "true", "1", "1.0")

    for r in rows:
        try:
            t_i = float(r.get("t", 0))
            v_i = float(r.get("icp_speed_ms", r.get("tacho_signed_ms", 0)))
            phi_i = float(r.get("phi_rad", r.get("mtt_articulation_angle", 0)))
            theta_i = float(r.get("icp_yaw", 0))
            # prefer kappa_real; fall back to theta_dot/v
            kappa_i_s = r.get("kappa_real", "")
            if kappa_i_s and kappa_i_s not in ("nan", "inf", "-inf"):
                kappa_i = float(kappa_i_s)
            else:
                th_dot = float(r.get("icp_yaw_rate_rad_s_derived",
                                     r.get("icp_yaw_rate_rad_s", 0)))
                kappa_i = th_dot / v_i if abs(v_i) > v_min else np.nan
            q_i = 1.0 if q_ok else 0.0
        except (ValueError, KeyError, ZeroDivisionError):
            continue

        if not (np.isfinite(t_i) and np.isfinite(phi_i)
                and np.isfinite(theta_i) and np.isfinite(kappa_i)):
            continue
        if abs(v_i) < v_min:
            continue
        if abs(kappa_i) > kappa_max:
            continue
        t.append(t_i)
        phi.append(phi_i)
        v.append(v_i)
        theta.append(theta_i)
        quality.append(q_i)

    if len(t) < 10:
        raise ValueError(f"Only {len(t)} valid rows — need at least 10.")

    return BagData(
        t=np.array(t, float),
        phi=np.array(phi, float),
        theta=np.array(theta, float),
        v=np.array(v, float),
        quality=np.array(quality, float) if quality else None,
    )


# ---- RMS + AIC helpers ----------------------------------------------------


def model_rms(kappa_true: np.ndarray, kappa_pred: np.ndarray,
              w: np.ndarray) -> float:
    m = np.isfinite(kappa_true) & np.isfinite(kappa_pred) & (w > 0)
    if m.sum() == 0:
        return np.nan
    return float(np.sqrt(np.sum(w[m] * (kappa_true[m] - kappa_pred[m]) ** 2)
                         / np.sum(w[m])))


def aic(n: int, rms: float, k: int) -> float:
    """AIC = n*ln(RSS/n) + 2k, where RSS = rms^2 * n."""
    if n <= 0 or rms <= 0 or np.isnan(rms):
        return np.inf
    rss = rms ** 2 * n
    return float(n * np.log(rss / n) + 2 * k)


def aicc(n: int, rms: float, k: int) -> float:
    """AICc = AIC + 2k(k+1)/(n-k-1) for small samples."""
    a = aic(n, rms, k)
    if n - k - 1 <= 0:
        return np.inf
    return a + 2 * k * (k + 1) / (n - k - 1)


# ---- Per-model phi→kappa predictors ---------------------------------------


def predict_m0(phi: np.ndarray, p: VehicleParams) -> np.ndarray:
    return np.array([kappa_nom_front(x, p) for x in phi])


def predict_m2(phi: np.ndarray, p: VehicleParams,
               params: M2Params) -> np.ndarray:
    return np.array([params.kappa(x, p) for x in phi])


def predict_vl(phi: np.ndarray, params: VirtualLengthParams) -> np.ndarray:
    return np.array([params.kappa(x) for x in phi])


def predict_m5(phi: np.ndarray, p: VehicleParams,
               params: M5ContactParams) -> np.ndarray:
    return np.array([params.kappa(x, p) for x in phi])


def predict_m5_hysteresis(phi: np.ndarray, p: VehicleParams,
                          params: M5ContactParams) -> np.ndarray:
    hp = M5HysteresisPlant(params, p)
    hp.reset(0.0)
    return np.array([hp.kappa_step(x) for x in phi])


# ---- Main comparison -----------------------------------------------------


def compare_all(path: str, save_fig: Optional[str] = None) -> Dict[str, Dict]:
    """Fit all models on the CSV at `path` and print a comparison table."""
    p = VehicleParams.mtt154()
    bag = load_csv(path)
    cv = bag.curvature()
    phi = np.asarray(bag.phi, float)
    kappa = cv["kappa"]
    w = cv["weight"]
    m = np.isfinite(kappa) & (w > 0) & (np.abs(kappa) < 2.0)
    phi_m, kap_m, w_m = phi[m], kappa[m], w[m]
    n = int(m.sum())

    print(f"Data: {path}")
    print(f"  Valid points: {n}")
    print(f"  Phi range: [{phi_m.min()*180/np.pi:.1f},"
          f" {phi_m.max()*180/np.pi:.1f}] deg")
    print()

    results: Dict[str, Dict] = {}
    header = f"{'Model':<22} {'#params':>7} {'RMS (e-3)':>10} {'AICc':>10} {'Notes'}"
    print(header)
    print("-" * len(header))

    # 1. M0 — 0 free params
    k0 = predict_m0(phi_m, p)
    r0 = model_rms(kap_m, k0, w_m)
    results["M0 (nominal)"] = dict(n_params=0, rms=r0, aicc=aicc(n, r0, 0))
    print(f"{'M0 (nominal)':<22} {0:>7} {r0*1000:>10.2f} "
          f"{aicc(n, r0, 0):>10.1f} {'nominal l1={:.1f} l2={:.1f}'.format(p.l1, p.l2)}")

    # 2. M2 — 1 param (gamma_vec = gamma only, no regression)
    m2 = M2Params()
    kap_m2_nom = np.array([kappa_nom_front(x, p) for x in phi_m])
    A = kap_m2_nom[:, None]
    from m5_identify import robust_lstsq
    coeff, _ = robust_lstsq(A, kap_m, w_m)
    gamma = float(np.clip(coeff[0], 0.05, 1.5))
    m2_fit = M2Params(gamma=gamma)
    k2 = predict_m2(phi_m, p, m2_fit)
    r2 = model_rms(kap_m, k2, w_m)
    results["M2 (gamma)"] = dict(n_params=1, rms=r2, aicc=aicc(n, r2, 1),
                                  gamma=gamma)
    print(f"{'M2 (gamma)':<22} {1:>7} {r2*1000:>10.2f} "
          f"{aicc(n, r2, 1):>10.1f} gamma={gamma:.4f}")

    # 3. VL fixed ratio — 1 param (L_eff)
    vl_fixed = identify_virtual_length(bag, p, fix_ratio=True)
    k_vlf = predict_vl(phi_m, vl_fixed.vl)
    r_vlf = model_rms(kap_m, k_vlf, w_m)
    results["VL (fixed ratio)"] = dict(n_params=1, rms=r_vlf,
                                        aicc=aicc(n, r_vlf, 1),
                                        L_eff=vl_fixed.vl.L_eff,
                                        gamma_slope=vl_fixed.vl.gamma_slope(p))
    print(f"{'VL (fixed ratio)':<22} {1:>7} {r_vlf*1000:>10.2f} "
          f"{aicc(n, r_vlf, 1):>10.1f} "
          f"L_eff={vl_fixed.vl.L_eff:.3f}")

    # 4. VL free — 2 params (l1_eff, l2_eff)
    vl_free = identify_virtual_length(bag, p, fix_ratio=False)
    k_vl = predict_vl(phi_m, vl_free.vl)
    r_vl = model_rms(kap_m, k_vl, w_m)
    results["VL (free l1,l2)"] = dict(n_params=2, rms=r_vl,
                                       aicc=aicc(n, r_vl, 2),
                                       l1_eff=vl_free.vl.l1_eff,
                                       l2_eff=vl_free.vl.l2_eff)
    print(f"{'VL (free l1,l2)':<22} {2:>7} {r_vl*1000:>10.2f} "
          f"{aicc(n, r_vl, 2):>10.1f} "
          f"l1={vl_free.vl.l1_eff:.3f} l2={vl_free.vl.l2_eff:.3f}")

    # 5. M5 L0 — 1 param (gamma_coeff, phi_dead fixed)
    m5_l0 = identify_m5(bag, p, level="L0", phi_dead_init_deg=1.0)
    k_m5_l0 = predict_m5(phi_m, p, m5_l0.m5)
    r_m5_l0 = model_rms(kap_m, k_m5_l0, w_m)
    results["M5 L0"] = dict(n_params=1, rms=r_m5_l0,
                             aicc=aicc(n, r_m5_l0, 1),
                             gamma_slope=m5_l0.gamma_slope,
                             phi_dead=m5_l0.phi_dead)
    print(f"{'M5 L0':<22} {1:>7} {r_m5_l0*1000:>10.2f} "
          f"{aicc(n, r_m5_l0, 1):>10.1f} "
          f"gamma_slope={m5_l0.gamma_slope:.4f} "
          f"phi_dead={m5_l0.phi_dead/DEG:.2f} deg")

    # 6. M5 L1 — 2 params (gamma_coeff, phi_dead)
    m5_l1 = identify_m5(bag, p, level="L1")
    k_m5_l1 = predict_m5(phi_m, p, m5_l1.m5)
    r_m5_l1 = model_rms(kap_m, k_m5_l1, w_m)
    results["M5 L1"] = dict(n_params=2, rms=r_m5_l1,
                             aicc=aicc(n, r_m5_l1, 2),
                             gamma_slope=m5_l1.gamma_slope,
                             phi_dead=m5_l1.phi_dead)
    print(f"{'M5 L1':<22} {2:>7} {r_m5_l1*1000:>10.2f} "
          f"{aicc(n, r_m5_l1, 2):>10.1f} "
          f"gamma_slope={m5_l1.gamma_slope:.4f} "
          f"phi_dead={m5_l1.phi_dead/DEG:.2f} deg")

    # 7. M5 L2 — 3 params (gamma, a0, a_v)
    m5_l2 = identify_m5(bag, p, level="L2")
    k_m5_l2 = predict_m2(phi_m, p, m5_l2.m2)
    r_m5_l2 = model_rms(kap_m, k_m5_l2, w_m)
    results["M5 L2"] = dict(n_params=3, rms=r_m5_l2,
                             aicc=aicc(n, r_m5_l2, 3),
                             gamma=m5_l2.m2.gamma,
                             a0=m5_l2.m2.a0, a_v=m5_l2.m2.a_v)
    print(f"{'M5 L2':<22} {3:>7} {r_m5_l2*1000:>10.2f} "
          f"{aicc(n, r_m5_l2, 3):>10.1f} "
          f"gamma={m5_l2.m2.gamma:.4f} a0={m5_l2.m2.a0:.4f} "
          f"a_v={m5_l2.m2.a_v:.4f}")

    # 8. M5+hysteresis — 3 params (gamma_coeff, phi_dead_k, loop width)
    if bag.phi.shape[0] > 200:
        hyst = identify_hysteresis(bag, p, m5_l1.m5)
        loop_w = hyst.get("loop_width_deg", np.nan)
        m5_hyst = M5ContactParams.calibrated(
            p, gamma_slope=m5_l1.gamma_slope,
            phi_dead_deg=abs(hyst.get("phi_dead_est_deg", m5_l1.phi_dead / DEG)))
        k_m5_h = predict_m5_hysteresis(phi_m, p, m5_hyst)
        r_m5_h = model_rms(kap_m, k_m5_h, w_m)
        results["M5+hysteresis"] = dict(n_params=3, rms=r_m5_h,
                                         aicc=aicc(n, r_m5_h, 3),
                                         loop_width_deg=loop_w)
        print(f"{'M5+hysteresis':<22} {3:>7} {r_m5_h*1000:>10.2f} "
              f"{aicc(n, r_m5_h, 3):>10.1f} "
              f"loop_width={loop_w:.1f} deg")
    else:
        print(f"{'M5+hysteresis':<22} - skipped (n={n} < 200)")

    # Best model by AICc
    best = min(results, key=lambda k: results[k]["aicc"])
    print(f"\nBest model by AICc: {best}")

    # ---- figure -----------------------------------------------------------
    if save_fig:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

        # Panel 1: data scatter + model curves
        ax = axes[0]
        ax.scatter(phi_m / DEG, kap_m * 1000, s=1.5, c="gray", alpha=0.3,
                   label=f"Data (n={n})", rasterized=True)
        phi_grid = np.linspace(phi_m.min(), phi_m.max(), 501)
        predictions = {
            "M0 (nominal)": predict_m0(phi_grid, p),
            "M2 (gamma)": predict_m2(phi_grid, p, m2_fit),
            "VL (free l1,l2)": predict_vl(phi_grid, vl_free.vl),
            "M5 L1": predict_m5(phi_grid, p, m5_l1.m5),
        }
        for name, kap_p in predictions.items():
            color = _MODEL_COLORS.get(name, "black")
            ls = "--" if "M2" in name or "VL" in name else "-"
            lw = 2.6 if "M5" in name else (2.0 if "M0" in name else 1.6)
            ax.plot(phi_grid / DEG, kap_p * 1000, color=color,
                    ls=ls, lw=lw, label=name)

        if "M5+hysteresis" in results:
            k_h = predict_m5_hysteresis(phi_grid, p, m5_hyst)
            ax.plot(phi_grid / DEG, k_h * 1000,
                    color=_MODEL_COLORS["M5+hysteresis"], ls=":", lw=1.8,
                    label="M5+hysteresis")

        ax.axhline(0, color="k", lw=0.5)
        ax.axvline(0, color="k", lw=0.5)
        ax.set_xlabel("Articulation angle φ (deg)")
        ax.set_ylabel("Curvature κ (×10⁻³ 1/m)")
        ax.set_title("Model comparison — κ(φ) fit")
        ax.legend(fontsize=7.5, loc="best")
        ax.grid(True, alpha=0.2)

        # Panel 2: residual histogram
        ax2 = axes[1]
        for name in ["M0 (nominal)", "M2 (gamma)", "VL (free l1,l2)", "M5 L1"]:
            if name in predictions:
                phi_idx = np.searchsorted(phi_grid, phi_m)
                phi_idx = np.clip(phi_idx, 0, len(phi_grid) - 1)
                resid = (kap_m - predictions[name][phi_idx]) * 1000
                ax2.hist(resid, bins=80, alpha=0.3, density=True,
                         color=_MODEL_COLORS.get(name, "gray"),
                         label=f"{name}  σ={np.std(resid):.2f}")
        ax2.set_xlabel("Residual κ (×10⁻³ 1/m)")
        ax2.set_ylabel("Density")
        ax2.set_title("Residual distribution")
        ax2.legend(fontsize=7, loc="best")
        ax2.grid(True, alpha=0.2)

        fig.suptitle(f"Model Ladder — {Path(path).name}", fontweight="bold")
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        fig.savefig(save_fig, dpi=160)
        print(f"Figure saved: {save_fig}")

    return results


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    csv_path = sys.argv[1]
    save = sys.argv[2] if len(sys.argv) > 2 else None
    compare_all(csv_path, save_fig=save)
