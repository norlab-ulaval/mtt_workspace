"""
traj_metrics.py

Trajectory-tracking quality metrics for the MTT closed-loop stack.

Two families, matching how the SLAM / odometry community (evo, TUM RGB-D)
and the path-tracking community actually report accuracy:

  1. TRACKING metrics -- reference is a PATH the vehicle was asked to follow.
     The natural error is the signed CROSS-TRACK distance e_y(t) (already
     produced by the controllers), plus heading error. We report RMS / max /
     final / settling-distance, all in the DISTANCE domain (the M5 theory
     doc, Thm. "distance-domain speed invariance", says time-domain specs are
     ill-posed for this rate-independent plant, so settling is in METERS).

  2. APE / RPE -- reference is another TRAJECTORY (e.g. the M0-predicted
     pose stream vs the M5-realized one, or estimated vs ground-truth from
     the drone). These are the standard evo definitions:

       APE_i = || (Q_i)^{-1} P_i ||        (absolute pose error, per frame)
       RPE_{i,d} = || (Q_i^{-1} Q_{i+d})^{-1} (P_i^{-1} P_{i+d}) ||
                                          (relative pose error over lag d)

     with SE(2) poses. APE captures global drift after an optional
     Umeyama alignment; RPE captures LOCAL consistency (drift per meter),
     which is the honest metric when there is no global frame -- exactly the
     drone/AprilTag case where you trust short baselines, not absolute GPS.

Everything is plain numpy. Poses are (x, y, theta). Trajectories are (n,3).
No plotting here; return dicts, plot however you like.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Dict, Optional, Tuple

import numpy as np


# --------------------------------------------------------------------------- #
# SE(2) helpers
# --------------------------------------------------------------------------- #

def _T(pose: np.ndarray) -> np.ndarray:
    """(x,y,theta) -> 3x3 homogeneous SE(2) matrix."""
    x, y, th = pose
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def _wrap(a: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(a), np.cos(a))


def _se2_trans_rot_error(A: np.ndarray, B: np.ndarray) -> Tuple[float, float]:
    """Error of the relative transform A^{-1} B: (translation norm, |rotation|)."""
    E = np.linalg.solve(A, B)
    dt = float(np.hypot(E[0, 2], E[1, 2]))
    dth = float(abs(np.arctan2(E[1, 0], E[0, 0])))
    return dt, dth


# --------------------------------------------------------------------------- #
# 1. Tracking metrics (path reference) -- cross-track / heading
# --------------------------------------------------------------------------- #

@dataclass
class TrackingReport:
    rms_ey: float          # RMS cross-track error (m)
    max_ey: float          # max |cross-track| (m)
    final_ey: float        # |cross-track| at the end (m)
    rms_etheta: float      # RMS heading error (rad)
    settle_distance: float # 2%-settling distance (m), nan if never settles
    overshoot: float       # max excursion past zero on the far side (m)
    arc_length: float      # total distance travelled (m)

    def as_dict(self) -> Dict[str, float]:
        return asdict(self)


def tracking_report(e_y: np.ndarray, e_theta: np.ndarray, s: np.ndarray,
                    settle_frac: float = 0.02) -> TrackingReport:
    """Distance-domain tracking metrics from cross-track/heading series and
    the cumulative arc length s (all same length)."""
    e_y = np.asarray(e_y, float)
    e_theta = np.asarray(e_theta, float)
    s = np.asarray(s, float)
    e0 = abs(e_y[0]) if abs(e_y[0]) > 1e-9 else float(np.max(np.abs(e_y)) + 1e-9)
    tol = max(settle_frac * e0, 0.005)
    settled = np.abs(e_y) < tol
    idx = next((k for k in range(len(settled)) if settled[k:].all()), None)
    settle_distance = float(s[idx]) if idx is not None else np.nan
    overshoot = float(np.max(np.maximum(0.0, -np.sign(e_y[0] + 1e-12) * e_y)))
    return TrackingReport(
        rms_ey=float(np.sqrt(np.mean(e_y ** 2))),
        max_ey=float(np.max(np.abs(e_y))),
        final_ey=float(abs(e_y[-1])),
        rms_etheta=float(np.sqrt(np.mean(_wrap(e_theta) ** 2))),
        settle_distance=settle_distance,
        overshoot=overshoot,
        arc_length=float(s[-1]),
    )


# --------------------------------------------------------------------------- #
# 2. APE / RPE (trajectory-vs-trajectory), evo-style, SE(2)
# --------------------------------------------------------------------------- #

def umeyama_se2(P: np.ndarray, Q: np.ndarray) -> np.ndarray:
    """Least-squares SE(2) alignment mapping P's positions onto Q's (no
    scale): returns the (x,y,theta) of the aligning transform T s.t. T*P ~ Q.
    Standard for APE so global frame offset is not double-counted."""
    Pp, Qp = P[:, :2], Q[:, :2]
    mu_p, mu_q = Pp.mean(0), Qp.mean(0)
    X, Y = Pp - mu_p, Qp - mu_q
    H = X.T @ Y
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, d]) @ U.T
    t = mu_q - R @ mu_p
    th = np.arctan2(R[1, 0], R[0, 0])
    return np.array([t[0], t[1], th])


def _apply_se2(T_pose: np.ndarray, traj: np.ndarray) -> np.ndarray:
    """Left-apply an SE(2) transform (given as a pose) to a whole trajectory."""
    T = _T(T_pose)
    out = np.zeros_like(traj)
    for i, pose in enumerate(traj):
        M = T @ _T(pose)
        out[i] = [M[0, 2], M[1, 2], np.arctan2(M[1, 0], M[0, 0])]
    return out


def ape(P: np.ndarray, Q: np.ndarray, align: bool = True) -> Dict[str, object]:
    """Absolute Pose Error between trajectory P (query) and Q (reference),
    same length, index-aligned. If align, Umeyama-align P to Q first
    (translation part). Returns per-frame translation/rotation errors and
    their statistics."""
    P = np.asarray(P, float); Q = np.asarray(Q, float)
    assert P.shape == Q.shape and P.shape[1] == 3
    if align:
        P = _apply_se2(umeyama_se2(P, Q), P)
    n = len(P)
    et = np.zeros(n); er = np.zeros(n)
    for i in range(n):
        et[i], er[i] = _se2_trans_rot_error(_T(Q[i]), _T(P[i]))
    return dict(
        trans=et, rot=er, aligned=align,
        rmse_trans=float(np.sqrt(np.mean(et ** 2))),
        mean_trans=float(np.mean(et)), max_trans=float(np.max(et)),
        rmse_rot=float(np.sqrt(np.mean(er ** 2))),
    )


def rpe(P: np.ndarray, Q: np.ndarray, delta: int = 1,
        s: Optional[np.ndarray] = None) -> Dict[str, object]:
    """Relative Pose Error over a fixed index lag `delta`. If the cumulative
    arc length `s` is given, also returns drift-per-meter (the honest
    local-consistency number for drone/AprilTag data with no global frame).

        RPE_i = || (Q_i^{-1} Q_{i+d})^{-1} (P_i^{-1} P_{i+d}) ||
    """
    P = np.asarray(P, float); Q = np.asarray(Q, float)
    n = len(P)
    et = np.zeros(n - delta); er = np.zeros(n - delta)
    for i in range(n - delta):
        relP = np.linalg.solve(_T(P[i]), _T(P[i + delta]))
        relQ = np.linalg.solve(_T(Q[i]), _T(Q[i + delta]))
        et[i], er[i] = _se2_trans_rot_error(relQ, relP)
    out = dict(
        trans=et, rot=er, delta=delta,
        rmse_trans=float(np.sqrt(np.mean(et ** 2))),
        rmse_rot=float(np.sqrt(np.mean(er ** 2))),
        mean_trans=float(np.mean(et)),
    )
    if s is not None:
        seg_len = np.array([s[i + delta] - s[i] for i in range(n - delta)])
        good = seg_len > 1e-6
        out["drift_per_m"] = float(np.mean(et[good] / seg_len[good]))
        out["rot_per_m"] = float(np.mean(er[good] / seg_len[good]))
    return out


def rpe_over_lags(P: np.ndarray, Q: np.ndarray, deltas) -> Dict[int, float]:
    """RPE translation RMSE as a function of lag -- the drift curve. A flat
    curve = consistent local accuracy; a rising curve = accumulating drift."""
    return {int(d): rpe(P, Q, delta=int(d))["rmse_trans"] for d in deltas}


# --------------------------------------------------------------------------- #
# Sanity checks
# --------------------------------------------------------------------------- #

if __name__ == "__main__":
    # A reference circle, a query that is the same circle shifted + rotated:
    # after Umeyama alignment APE ~ 0; RPE ~ 0 (locally identical).
    th = np.linspace(0, 2 * np.pi, 400)
    R = 5.0
    Q = np.stack([R * np.cos(th), R * np.sin(th), th + np.pi / 2], axis=1)
    shift = np.array([3.0, -2.0, 0.5])
    P = _apply_se2(shift, Q)

    a = ape(P, Q, align=True)
    assert a["rmse_trans"] < 1e-9, a["rmse_trans"]
    r = rpe(P, Q, delta=1)
    assert r["rmse_trans"] < 1e-9
    print(f"[APE] rigid-shifted copy after alignment: {a['rmse_trans']:.2e} m (==0)")
    print(f"[RPE] delta=1: {r['rmse_trans']:.2e} m (==0, locally identical)")

    # Inject a growing drift into P -> APE grows, RPE stays small & bounded.
    drift = np.cumsum(np.full(len(Q), 0.002))
    Pd = Q.copy(); Pd[:, 0] += drift
    ad = ape(Pd, Q, align=False)
    rd = rpe(Pd, Q, delta=1)
    print(f"[drift test] APE rmse={ad['rmse_trans']*100:.1f} cm (large), "
          f"RPE(1) rmse={rd['rmse_trans']*100:.3f} cm (small) "
          f"-- APE sees global drift, RPE sees local consistency")
    assert ad["rmse_trans"] > 20 * rd["rmse_trans"]

    # Tracking report on a decaying cross-track error
    s = np.linspace(0, 40, 800)
    e_y = 0.5 * np.exp(-s / 8) * np.cos(s / 4)
    e_th = np.gradient(e_y, s)
    rep = tracking_report(e_y, e_th, s)
    print(f"[tracking] rms_ey={rep.rms_ey*100:.1f} cm, "
          f"settle_distance={rep.settle_distance:.1f} m, "
          f"overshoot={rep.overshoot*100:.1f} cm")
    assert rep.rms_ey > 0 and rep.arc_length > 39

    print("All traj_metrics checks passed.")
