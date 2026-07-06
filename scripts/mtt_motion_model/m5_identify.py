"""
m5_identify.py

Learn the M5 plant parameters from real MTT bags, so the simulation plant
matches the robot. This is what makes the sim predictive instead of merely
illustrative.

WHAT YOU LOG (any subset works; the more, the more parameters identifiable):
    t          (n,)  timestamps [s]
    phi        (n,)  articulation angle [rad]      (encoder)
    theta      (n,)  front-body heading [rad]      (ICP / IMU yaw)
    v          (n,)  front forward speed [m/s]     (wheel/belt odom or ICP)
  optionally:
    quality    (n,)  ICP match quality in [0,1]    (down-weights bad frames)

FROM THESE we form the measured curvature  kappa_meas = theta_dot / v  and
fit, in increasing order of data richness (the identifiability ladder of the
theory doc, Prop. "identifiability ladder"):

  (L0) gamma only            -- 1 steady arc: only the PRODUCT gamma*(phi-phi_dead)
                                is constrained; fit gamma with phi_dead fixed.
  (L1) gamma_slope + phi_dead -- >= 2 arcs at different |phi|: the dead-zone
                                affine law kappa = gamma_coeff * dz(phi,phi_dead)
                                fit by robust linear regression on |phi|>thr.
  (L2) + M2 regression       -- add speed / phidot dependence:
                                kappa = gamma*kappa_nom(phi) + a0 + a_v v + ...
  (L3) + hysteresis loop      -- from triangular sweeps: static/kinetic split
                                and loop width from the two sliding branches.

Outputs are ready-to-use motion_model objects (M5ContactParams via
`.calibrated(...)`, M2Params) so you can drop them straight into the sim.

Pure numpy. Robust (Huber-like IRLS) so a few bad ICP frames don't dominate.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from motion_model import (DEG, VehicleParams, M2Params, M5ContactParams,
                          VirtualLengthParams, kappa_nom_front)


# --------------------------------------------------------------------------- #
# Preprocessing: bag -> measured curvature
# --------------------------------------------------------------------------- #

def _savgol(x: np.ndarray, t: np.ndarray, win: int = 11, poly: int = 2,
            deriv: int = 0) -> np.ndarray:
    """Minimal Savitzky-Golay on possibly-nonuniform t via local polyfit.
    deriv=0 smooths, deriv=1 returns the derivative. O(n*win) but fine here."""
    n = len(x)
    out = np.zeros(n)
    half = win // 2
    for i in range(n):
        a = max(0, i - half); b = min(n, i + half + 1)
        tt = t[a:b] - t[i]
        c = np.polyfit(tt, x[a:b], min(poly, b - a - 1))
        if deriv == 0:
            out[i] = c[-1]
        else:
            out[i] = c[-2] if len(c) >= 2 else 0.0
    return out


@dataclass
class BagData:
    t: np.ndarray
    phi: np.ndarray
    theta: np.ndarray
    v: np.ndarray
    quality: Optional[np.ndarray] = None

    def curvature(self, win: int = 11) -> Dict[str, np.ndarray]:
        """kappa_meas = theta_dot / v, with theta_dot from Savitzky-Golay and
        low-speed frames masked (curvature undefined as v->0)."""
        th_unwrap = np.unwrap(self.theta)
        theta_dot = _savgol(th_unwrap, self.t, win=win, deriv=1)
        phidot = _savgol(np.asarray(self.phi, float), self.t, win=win, deriv=1)
        good = np.abs(self.v) > 0.15
        kappa = np.full(len(self.t), np.nan)
        kappa[good] = theta_dot[good] / self.v[good]
        w = np.ones(len(self.t)) if self.quality is None else np.clip(self.quality, 0, 1)
        w[~good] = 0.0
        return dict(kappa=kappa, theta_dot=theta_dot, phidot=phidot, weight=w,
                    good=good)


# --------------------------------------------------------------------------- #
# Robust linear regression (IRLS with Huber weights)
# --------------------------------------------------------------------------- #

def robust_lstsq(A: np.ndarray, b: np.ndarray, w0: np.ndarray | None = None,
                 iters: int = 12, c: float = 1.345) -> Tuple[np.ndarray, np.ndarray]:
    """Huber IRLS. Returns (coeffs, final_weights)."""
    m = A.shape[0]
    w = np.ones(m) if w0 is None else np.array(w0, float)
    x = np.zeros(A.shape[1])
    for _ in range(iters):
        W = np.sqrt(w)
        x, *_ = np.linalg.lstsq(A * W[:, None], b * W, rcond=None)
        r = b - A @ x
        s = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-9
        u = np.abs(r) / s
        w_new = np.where(u <= c, 1.0, c / np.maximum(u, 1e-9))
        if w0 is not None:
            w_new *= w0
        if np.allclose(w_new, w, atol=1e-3):
            w = w_new; break
        w = w_new
    return x, w


# --------------------------------------------------------------------------- #
# The identification ladder
# --------------------------------------------------------------------------- #

@dataclass
class IdentResult:
    gamma_slope: float
    phi_dead: float           # rad
    m2: M2Params
    m5: M5ContactParams
    rms_residual: float       # 1/m, on the fitted law
    n_used: int
    notes: str


def identify_m5(bag: BagData, p: VehicleParams, level: str = "L1",
                phi_dead_init_deg: float = 1.0) -> IdentResult:
    """Fit the M5 dead-zone affine law kappa = gamma_coeff * dz(phi, phi_dead)
    to a bag. `level`:
      "L0" : fix phi_dead, fit slope only (single-arc regime).
      "L1" : fit slope AND phi_dead jointly (>=2 distinct |phi| needed).
      "L2" : additionally fit the M2 speed/phidot regression on the residual.
    """
    cv = bag.curvature()
    phi = np.asarray(bag.phi, float)
    kappa = cv["kappa"]; w = cv["weight"]
    m = np.isfinite(kappa) & (w > 0)
    phi_m, kap_m, w_m, v_m = phi[m], kappa[m], w[m], np.asarray(bag.v, float)[m]
    L = p.l1 + p.l2

    if level == "L0":
        phi_dead = phi_dead_init_deg * DEG
        dzp = np.sign(phi_m) * np.maximum(np.abs(phi_m) - phi_dead, 0.0)
        A = dzp[:, None]
        coeff, wf = robust_lstsq(A, kap_m, w_m)
        gamma_coeff = float(coeff[0])
        # gamma_coeff = L/(L^2+c2_eff) = gamma_slope/L, so gamma_slope = gamma_coeff*L
        gamma_slope = gamma_coeff * L
        notes = "L0: single-arc, phi_dead fixed"
    else:
        # L1: jointly fit slope and phi_dead. phi_dead enters nonlinearly, so
        # grid-search phi_dead, robust-linear-fit slope at each, pick best.
        best = None
        for pd in np.linspace(0.0, 8 * DEG, 81):
            dzp = np.sign(phi_m) * np.maximum(np.abs(phi_m) - pd, 0.0)
            coeff, wf = robust_lstsq(dzp[:, None], kap_m, w_m)
            resid = kap_m - coeff[0] * dzp
            rms = np.sqrt(np.sum(wf * resid ** 2) / np.sum(wf))
            if best is None or rms < best[0]:
                best = (rms, float(coeff[0]), float(pd), wf)
        rms, gamma_coeff, phi_dead, wf = best
        gamma_slope = gamma_coeff * L
        notes = "L1: joint slope + phi_dead grid/robust fit"

    if level == "L0":
        dzp = np.sign(phi_m) * np.maximum(np.abs(phi_m) - phi_dead, 0.0)
        resid = kap_m - gamma_coeff * dzp
        rms = float(np.sqrt(np.sum(wf * resid ** 2) / np.sum(wf)))

    gamma_slope = float(np.clip(gamma_slope, 0.05, 1.0))
    m5 = M5ContactParams.calibrated(p, gamma_slope=gamma_slope,
                                    phi_dead_deg=phi_dead / DEG)

    # M2 regression (optional L2): fit residual after M0 shape
    m2 = M2Params(gamma=gamma_slope)
    if level == "L2":
        kn = np.array([kappa_nom_front(x, p) for x in phi_m])
        # kappa = gamma*kn + a0 + a_v v ; identify (gamma,a0,a_v)
        A = np.stack([kn, np.ones_like(kn), v_m], axis=1)
        coeff, _ = robust_lstsq(A, kap_m, w_m)
        m2 = M2Params(gamma=float(np.clip(coeff[0], 0.05, 1.5)),
                      a0=float(coeff[1]), a_v=float(coeff[2]))
        notes += " + L2 M2 regression"

    return IdentResult(gamma_slope=gamma_slope, phi_dead=phi_dead, m2=m2, m5=m5,
                       rms_residual=float(rms), n_used=int(m.sum()), notes=notes)


def identify_hysteresis(bag: BagData, p: VehicleParams,
                        m5: M5ContactParams) -> Dict[str, float]:
    """From a triangular phi sweep, estimate the loop width (2*phi_dead_k) and
    the static/kinetic ratio from the offset between up- and down-going
    sliding branches. Returns dict with loop_width_deg, mus_over_muk_est."""
    cv = bag.curvature()
    phi = np.asarray(bag.phi, float); kappa = cv["kappa"]; phidot = cv["phidot"]
    m = np.isfinite(kappa) & (cv["weight"] > 0) & (np.abs(kappa) > 1e-3)
    up = m & (phidot > 0); dn = m & (phidot < 0)
    gc = m5.gamma_coeff(p)
    # for each branch, phi-intercept of the sliding line kappa = gc*(phi - off)
    def intercept(mask):
        if mask.sum() < 5:
            return np.nan
        # robust fit kappa = gc*phi - gc*off  -> off = phi - kappa/gc, take median
        return float(np.median(phi[mask] - kappa[mask] / gc))
    off_up, off_dn = intercept(up), intercept(dn)
    loop_width = abs(off_up - off_dn) if np.isfinite(off_up) and np.isfinite(off_dn) else np.nan
    return dict(offset_up_deg=off_up / DEG if np.isfinite(off_up) else np.nan,
                offset_dn_deg=off_dn / DEG if np.isfinite(off_dn) else np.nan,
                loop_width_deg=loop_width / DEG if np.isfinite(loop_width) else np.nan,
                phi_dead_est_deg=(loop_width / 2) / DEG if np.isfinite(loop_width) else np.nan)


# --------------------------------------------------------------------------- #
# Virtual Length identification: fit l1_eff, l2_eff from measured kappa(phi)
# --------------------------------------------------------------------------- #

@dataclass
class VLIdentResult:
    vl: VirtualLengthParams
    rms_residual: float
    n_used: int
    notes: str


def identify_virtual_length(bag: BagData, p: VehicleParams,
                            fix_ratio: bool = False) -> VLIdentResult:
    """Fit the VL law kappa = sin(phi) / (l1_eff*cos(phi) + l2_eff) to bag data.

    fix_ratio=True  : keep l1_eff/l2_eff = p.l1/p.l2, fit L_eff only (1 param)
    fix_ratio=False : fit both l1_eff and l2_eff independently (2 params, NLS)

    Uses grid search + local refinement (scipy is not required).
    """
    cv = bag.curvature()
    phi = np.asarray(bag.phi, float)
    kappa = cv["kappa"]; w = cv["weight"]
    m = np.isfinite(kappa) & (w > 0) & (np.abs(kappa) < 2.0)  # reject outliers
    phi_m, kap_m, w_m = phi[m], kappa[m], w[m]

    def residual(l1, l2):
        pred = np.sin(phi_m) / (l1 * np.cos(phi_m) + l2 + 1e-12)
        return np.sqrt(np.sum(w_m * (kap_m - pred) ** 2) / np.sum(w_m))

    if fix_ratio:
        ratio = p.l1 / p.l2
        # grid search over L_eff = l1_eff + l2_eff
        best = None
        L_phys = p.l1 + p.l2
        for L in np.linspace(0.5 * L_phys, 3.0 * L_phys, 501):
            l1 = L * ratio / (1 + ratio)
            l2 = L / (1 + ratio)
            r = residual(l1, l2)
            if best is None or r < best[0]:
                best = (r, l1, l2)
        rms, l1_eff, l2_eff = best
        notes = f"VL (fixed ratio={ratio:.3f})"
    else:
        # 2D grid search over l1_eff, l2_eff
        best = None
        for l1 in np.linspace(0.2, 3.0, 57):
            for l2 in np.linspace(0.2, 3.0, 57):
                r = residual(l1, l2)
                if best is None or r < best[0]:
                    best = (r, l1, l2)
        rms0, l1_0, l2_0 = best
        # local refinement: finer grid around the best
        best = None
        for l1 in np.linspace(l1_0 - 0.15, l1_0 + 0.15, 31):
            for l2 in np.linspace(l2_0 - 0.15, l2_0 + 0.15, 31):
                if l1 <= 0 or l2 <= 0:
                    continue
                r = residual(l1, l2)
                if best is None or r < best[0]:
                    best = (r, l1, l2)
        rms, l1_eff, l2_eff = best
        notes = "VL (free l1_eff, l2_eff)"

    return VLIdentResult(
        vl=VirtualLengthParams(l1_eff=float(l1_eff), l2_eff=float(l2_eff)),
        rms_residual=float(rms), n_used=int(m.sum()), notes=notes)


# --------------------------------------------------------------------------- #
# Synthetic-bag self-test: generate data from a KNOWN M5 plant, recover it
# --------------------------------------------------------------------------- #

def _synthesize_bag(p: VehicleParams, m5_true: M5ContactParams,
                    seed: int = 0, noise_deg: float = 0.3) -> BagData:
    """Two constant-radius arcs at different phi + connecting ramps, run
    through the true M5 law, with heading integrated and noise added."""
    rng = np.random.default_rng(seed)
    dt = 0.05; v = 0.8
    phi_prog = np.concatenate([
        np.full(200, 6 * DEG), np.full(200, 14 * DEG),
        np.full(150, 22 * DEG), np.full(100, 3 * DEG)])
    n = len(phi_prog)
    t = np.arange(n) * dt
    theta = np.zeros(n)
    for k in range(1, n):
        kap = m5_true.kappa(phi_prog[k], p)
        theta[k] = theta[k - 1] + v * kap * dt
    theta_noisy = theta + rng.normal(0, noise_deg * DEG, n)
    phi_noisy = phi_prog + rng.normal(0, 0.2 * DEG, n)
    return BagData(t=t, phi=phi_noisy, theta=theta_noisy,
                   v=np.full(n, v), quality=np.ones(n))


if __name__ == "__main__":
    p = VehicleParams.mtt154()
    # ground truth we will try to recover
    m5_true = M5ContactParams.calibrated(p, gamma_slope=0.82, phi_dead_deg=2.5)
    print(f"[truth] gamma_slope=0.820, phi_dead=2.50 deg")

    bag = _synthesize_bag(p, m5_true, seed=1)

    res0 = identify_m5(bag, p, level="L0", phi_dead_init_deg=2.5)
    print(f"[L0] gamma_slope={res0.gamma_slope:.3f} (phi_dead fixed), "
          f"resid={res0.rms_residual*1000:.2f} e-3 1/m, n={res0.n_used}")

    res1 = identify_m5(bag, p, level="L1")
    print(f"[L1] gamma_slope={res1.gamma_slope:.3f}, "
          f"phi_dead={res1.phi_dead/DEG:.2f} deg, "
          f"resid={res1.rms_residual*1000:.2f} e-3 1/m  ({res1.notes})")
    assert abs(res1.gamma_slope - 0.82) < 0.06, "gamma not recovered"
    assert abs(res1.phi_dead / DEG - 2.5) < 1.0, "phi_dead not recovered"

    # the recovered M5 can be dropped straight into the sim:
    print(f"[use] drop-in: M5ContactParams.calibrated(p, "
          f"gamma_slope={res1.gamma_slope:.3f}, "
          f"phi_dead_deg={res1.phi_dead/DEG:.2f})")

    # --- VL recovery test: fit on M5-generated data (NO deadband at large phi) --
    print("\n--- VL identification on M5-generated bag ---")
    vl_fixed = identify_virtual_length(bag, p, fix_ratio=True)
    print(f"[VL-fixed] L_eff={vl_fixed.vl.L_eff:.3f} m "
          f"(gamma_slope={vl_fixed.vl.gamma_slope(p):.3f}), "
          f"resid={vl_fixed.rms_residual*1000:.2f} e-3 1/m")
    vl_free = identify_virtual_length(bag, p, fix_ratio=False)
    print(f"[VL-free] l1={vl_free.vl.l1_eff:.3f}, l2={vl_free.vl.l2_eff:.3f} m, "
          f"resid={vl_free.rms_residual*1000:.2f} e-3 1/m")

    print("All m5_identify checks passed.")
