"""
traj_tracking_sim.py

Simple path-tracking simulation built on `articulated_kinematics.py`.

Idea: use ONE simple tracking controller (a proportional law on cross-track
+ heading error, closed through the quasi-static kappa_nom(phi) inversion --
this mirrors the "turn: kappa_nom(phi) ... feedforward" block already in the
MTT control diagram) and drive it against TWO different plant realities:

    - the "exact" nonholonomic model (eq. 19, includes phi_dot)
    - the "quasi_static" model (eq. 13/20, phi_dot term dropped)

Same controller, same reference path, same gains -- the only thing that
changes is which physical model the vehicle actually obeys. The gap between
the two resulting tracking-error time series is a direct, simulated measure
of how much the phi_dot-blind controller design costs you in practice.

No plotting here either -- everything is returned as plain numpy arrays.

Plotting functions live at the very end of the file (after the sanity-check
`__main__` block content) and are the only thing added on top of the
original code -- nothing above was changed.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Tuple

import numpy as np

from motion_model import (
    DEG,
    VehicleParams,
    RefPoint,
    kappa_nom_front,
    pose_at,
    simulate,
)

PathFn = Callable[[np.ndarray], Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]
# path_fn(s) -> (x, y, theta, kappa), all arrays, s = arc length (m)


# =========================================================================== #
# Reference paths
# =========================================================================== #

def circular_arc_path(R: float) -> PathFn:
    """Constant-curvature path of radius R (m), starting at the origin,
    heading 0, turning left for R>0 / right for R<0.
    """
    def path_fn(s: np.ndarray):
        s = np.asarray(s, dtype=float)
        theta = s / R
        x = R * np.sin(theta)
        y = R * (1 - np.cos(theta))
        kappa = np.full_like(s, 1.0 / R)
        return x, y, theta, kappa
    return path_fn


def s_curve_path(kappa_max: float, wavelength: float) -> PathFn:
    """Sinusoidal-curvature path: kappa(s) = kappa_max*sin(2*pi*s/wavelength).
    Heading and position obtained by cumulative numerical integration
    (simple trapezoid rule, dense enough to be effectively exact for
    reasonable wavelengths). Useful because an S-curve is exactly the
    maneuver Corke & Ridley / the MTT observability analysis flag as the
    one that separates front/rear effects instead of confounding them.
    """
    s_dense = np.linspace(0, 4 * wavelength, 20000)
    kappa_dense = kappa_max * np.sin(2 * np.pi * s_dense / wavelength)
    theta_dense = np.concatenate([[0.0], np.cumsum(
        0.5 * (kappa_dense[1:] + kappa_dense[:-1]) * np.diff(s_dense))])
    x_dense = np.concatenate([[0.0], np.cumsum(
        0.5 * (np.cos(theta_dense[1:]) + np.cos(theta_dense[:-1])) * np.diff(s_dense))])
    y_dense = np.concatenate([[0.0], np.cumsum(
        0.5 * (np.sin(theta_dense[1:]) + np.sin(theta_dense[:-1])) * np.diff(s_dense))])

    def path_fn(s: np.ndarray):
        s = np.asarray(s, dtype=float)
        x = np.interp(s, s_dense, x_dense)
        y = np.interp(s, s_dense, y_dense)
        theta = np.interp(s, s_dense, theta_dense)
        kappa = np.interp(s, s_dense, kappa_dense)
        return x, y, theta, kappa
    return path_fn


def build_path_table(path_fn: PathFn, s_max: float, n: int = 4000) -> Dict[str, np.ndarray]:
    """Densely sample a path for nearest-point lookup during tracking."""
    s = np.linspace(0, s_max, n)
    x, y, theta, kappa = path_fn(s)
    return dict(s=s, x=x, y=y, theta=theta, kappa=kappa)


# =========================================================================== #
# kappa_nom(phi) inversion (table-based, monotonic over [-phi_max, phi_max])
# =========================================================================== #

def build_kappa_inversion_table(p: VehicleParams, n: int = 2000) -> Dict[str, np.ndarray]:
    phi = np.linspace(-p.phi_max, p.phi_max, n)
    kappa = kappa_nom_front(phi, p)
    return dict(phi=phi, kappa=kappa)  # kappa is monotonic increasing in phi for l1<l2


def phi_from_kappa_nom(kappa_des: float, table: Dict[str, np.ndarray]) -> float:
    """Invert kappa_nom_front(phi) = kappa_des for phi, via 1D interpolation.
    Silently clips to +/-phi_max if kappa_des is outside the achievable range
    (np.interp clamps to the boundary y-value by default).
    """
    return float(np.interp(kappa_des, table["kappa"], table["phi"]))


# =========================================================================== #
# Cross-track / heading error against a path table
# =========================================================================== #

def nearest_path_error(xy: np.ndarray, theta: float, table: Dict[str, np.ndarray]) -> Tuple[float, float, float, int]:
    """Nearest-point search (brute-force argmin, fine for demo-sized tables).
    Returns (cross_track_error e_y, heading_error e_theta, kappa_p, index).
    e_y > 0 means the vehicle is to the LEFT of the path tangent.
    """
    d2 = (table["x"] - xy[0]) ** 2 + (table["y"] - xy[1]) ** 2
    idx = int(np.argmin(d2))
    xp, yp, thp, kp = table["x"][idx], table["y"][idx], table["theta"][idx], table["kappa"][idx]
    e_y = -(xy[0] - xp) * np.sin(thp) + (xy[1] - yp) * np.cos(thp)
    e_theta = np.arctan2(np.sin(theta - thp), np.cos(theta - thp))  # wrapped to [-pi, pi]
    return e_y, e_theta, kp, idx


# =========================================================================== #
# Simple proportional path-tracking controller
# =========================================================================== #

@dataclass
class TrackingGains:
    k_y: float = 0.6       # cross-track error gain (1/m^2, acts on curvature)
    k_theta: float = 1.2   # heading error gain (1/m, acts on curvature)
    k_phi: float = 3.0     # articulation servo gain (1/s), phi -> phi_des tracking
    v_ref: float = 1.5     # constant forward speed command (m/s)


def make_controller(path_table: Dict[str, np.ndarray], p: VehicleParams,
                     gains: TrackingGains):
    """Returns control_fn(t, state) -> [v1, phi_dot], a simple proportional
    tracking law:

        kappa_des = kappa_path - k_y*e_y - k_theta*sin(e_theta)
        phi_des   = kappa_nom_front^{-1}(kappa_des)
        phi_dot   = clip(k_phi*(phi_des - phi), +/- phi_dot_max)
        v1        = v_ref  (constant)

    This is intentionally simple (illustrative, not a certified/optimal
    tracking law) so that the ONLY thing being compared across models is the
    plant response to identical commands, not controller sophistication.
    """
    ktable = build_kappa_inversion_table(p)

    def control_fn(t: float, state: np.ndarray) -> np.ndarray:
        x1, y1, th1, phi = state
        e_y, e_theta, kappa_p, _ = nearest_path_error(np.array([x1, y1]), th1, path_table)
        kappa_des = kappa_p - gains.k_y * e_y - gains.k_theta * np.sin(e_theta)
        phi_des = phi_from_kappa_nom(kappa_des, ktable)
        phi_dot = gains.k_phi * (phi_des - phi)
        phi_dot = np.clip(phi_dot, -p.phi_dot_max, p.phi_dot_max)
        return np.array([gains.v_ref, phi_dot])

    return control_fn


# =========================================================================== #
# Full comparison run: same controller, two plant models
# =========================================================================== #

def run_comparison(path_fn: PathFn, s_max: float, p: VehicleParams,
                    gains: TrackingGains, dt: float, t_final: float,
                    x0: np.ndarray | None = None,
                    ref: RefPoint = RefPoint.FRONT) -> Dict[str, np.ndarray]:
    """Simulate the SAME tracking controller against both plant models and
    return tracking-error time series + summary stats for each.
    """
    if x0 is None:
        x0 = np.zeros(4)
    path_table = build_path_table(path_fn, s_max)
    control_fn = make_controller(path_table, p, gains)

    t, X_exact = simulate(x0, control_fn, p, dt, t_final, model="exact")
    _, X_qs = simulate(x0, control_fn, p, dt, t_final, model="quasi_static")

    def error_series(X):
        e_y = np.zeros(len(X))
        e_th = np.zeros(len(X))
        pts = np.zeros((len(X), 3))
        for k, state in enumerate(X):
            xr, yr, thr = pose_at(state, p, ref)
            pts[k] = (xr, yr, thr)
            e_y[k], e_th[k], _, _ = nearest_path_error(np.array([xr, yr]), thr, path_table)
        return e_y, e_th, pts

    ey_exact, eth_exact, path_exact = error_series(X_exact)
    ey_qs, eth_qs, path_qs = error_series(X_qs)

    def rms(a):
        return float(np.sqrt(np.mean(a ** 2)))

    return dict(
        t=t, path_table=path_table,
        X_exact=X_exact, X_qs=X_qs,
        ref_path_exact=path_exact, ref_path_qs=path_qs,
        cross_track_exact=ey_exact, cross_track_qs=ey_qs,
        heading_err_exact=eth_exact, heading_err_qs=eth_qs,
        rms_cross_track_exact=rms(ey_exact), rms_cross_track_qs=rms(ey_qs),
        max_cross_track_exact=float(np.max(np.abs(ey_exact))),
        max_cross_track_qs=float(np.max(np.abs(ey_qs))),
    )


# =========================================================================== #
# Sanity checks / demo (no plotting) -- run with `python traj_tracking_sim.py`
# =========================================================================== #

if __name__ == "__main__":
    p_mtt = VehicleParams.mtt154()
    gains = TrackingGains(k_y=0.6, k_theta=1.2, k_phi=3.0, v_ref=1.5)

    # --- Circular arc, R = 4 m (tight turn, well within phi_max) -----------
    circ = circular_arc_path(R=4.0)
    res_circ = run_comparison(circ, s_max=25.0, p=p_mtt, gains=gains,
                               dt=0.02, t_final=16.0, ref=RefPoint.FRONT)
    print(f"[circular R=4m] RMS cross-track: exact={res_circ['rms_cross_track_exact']*100:.2f} cm, "
          f"quasi_static={res_circ['rms_cross_track_qs']*100:.2f} cm  "
          f"(max: {res_circ['max_cross_track_exact']*100:.2f} vs {res_circ['max_cross_track_qs']*100:.2f} cm)")
    assert res_circ["rms_cross_track_exact"] >= 0 and res_circ["rms_cross_track_qs"] >= 0

    # --- S-curve, forces sign reversal of phi -> phi_dot term matters most --
    scurve = s_curve_path(kappa_max=1 / 3.0, wavelength=10.0)
    res_s = run_comparison(scurve, s_max=20.0, p=p_mtt, gains=gains,
                            dt=0.02, t_final=16.0, ref=RefPoint.REAR)
    print(f"[S-curve, rear-referenced] RMS cross-track: exact={res_s['rms_cross_track_exact']*100:.2f} cm, "
          f"quasi_static={res_s['rms_cross_track_qs']*100:.2f} cm  "
          f"(max: {res_s['max_cross_track_exact']*100:.2f} vs {res_s['max_cross_track_qs']*100:.2f} cm)")

    # the two plant models must not track identically once phi is moving
    assert not np.allclose(res_s["cross_track_exact"], res_s["cross_track_qs"])

    print("All sanity checks passed.")


# =========================================================================== #
# Plotting (matplotlib only, added on top of the code above -- nothing above
# this line was changed)
# =========================================================================== #

import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams.update({
    "font.size": 10.5,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.grid": True,
    "grid.alpha": 0.25,
})


def plot_tracking_result(res: Dict[str, np.ndarray], title: str = "", ref_label: str = "FRONT"):
    """3-panel figure for one `run_comparison(...)` result:
       (a) reference path + exact/quasi-static trajectory (XY, ref point),
       (b) cross-track error vs time, both models,
       (c) heading error vs time, both models.
    """
    table = res["path_table"]
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15.5, 4.6))

    ax1.plot(table["x"], table["y"], color="gray", lw=2, ls="--", label="reference path")
    ax1.plot(res["ref_path_exact"][:, 0], res["ref_path_exact"][:, 1],
              color="#4C72B0", lw=2, label="exact plant")
    ax1.plot(res["ref_path_qs"][:, 0], res["ref_path_qs"][:, 1],
              color="#DD8452", lw=2, label="quasi-static plant")
    ax1.set_aspect("equal")
    ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
    ax1.set_title(f"{ref_label}-referenced path" + (f" -- {title}" if title else ""))
    ax1.legend(fontsize=8, loc="best")

    ax2.plot(res["t"], res["cross_track_exact"] * 100, color="#4C72B0", lw=2, label="exact")
    ax2.plot(res["t"], res["cross_track_qs"] * 100, color="#DD8452", lw=2, label="quasi-static")
    ax2.axhline(0, color="k", lw=0.6)
    ax2.set_xlabel("t (s)"); ax2.set_ylabel(r"cross-track error $e_y$ (cm)")
    ax2.set_title(f"RMS: {res['rms_cross_track_exact']*100:.1f} vs "
                   f"{res['rms_cross_track_qs']*100:.1f} cm")
    ax2.legend(fontsize=8)

    ax3.plot(res["t"], res["heading_err_exact"] / DEG, color="#4C72B0", lw=2, label="exact")
    ax3.plot(res["t"], res["heading_err_qs"] / DEG, color="#DD8452", lw=2, label="quasi-static")
    ax3.axhline(0, color="k", lw=0.6)
    ax3.set_xlabel("t (s)"); ax3.set_ylabel(r"heading error $e_\theta$ (deg)")
    ax3.set_title("Heading error")
    ax3.legend(fontsize=8)

    fig.tight_layout()
    return fig


# ----------------------------------------------------------------------------- #
# Second entry point: only runs the plots, reusing the results already
# computed in the sanity-check __main__ block above (res_circ, res_s are
# plain module-level globals by the time this runs).
# ----------------------------------------------------------------------------- #

if __name__ == "__main__":
    plot_tracking_result(res_circ, title="circular arc R=4m", ref_label="FRONT")
    plot_tracking_result(res_s, title="S-curve", ref_label="REAR")
    plt.show()


# =========================================================================== #
# Extra diagnostics: front/rear paths, articulation servo, realized speeds
# and yaw rates -- to understand the MODEL itself, independently of how
# "good" this particular (deliberately simple) controller is. The control
# law will be revisited later with something closer to what the MTT stack
# actually needs (Lie-group based, not a flat proportional law) -- these
# plots are only meant to build intuition about *why* the two plants behave
# differently under the SAME commands.
# =========================================================================== #

def reconstruct_controller_diagnostics(t: np.ndarray, X: np.ndarray, p: VehicleParams,
                                        path_table: Dict[str, np.ndarray],
                                        gains: TrackingGains) -> Dict[str, np.ndarray]:
    """Replays the (unchanged) controller math along an already-simulated
    trajectory to recover phi_des(t), kappa_des(t), kappa_path(t). These are
    internal controller quantities `run_comparison` does not return, but
    everything called here (`nearest_path_error`, `build_kappa_inversion_table`,
    `phi_from_kappa_nom`) is the exact same code defined above -- just
    re-evaluated along the logged trajectory for inspection.
    """
    ktable = build_kappa_inversion_table(p)
    n = len(t)
    phi_des = np.zeros(n)
    kappa_des = np.zeros(n)
    kappa_path = np.zeros(n)
    for k in range(n):
        x1, y1, th1, phi = X[k]
        e_y, e_theta, kp, _ = nearest_path_error(np.array([x1, y1]), th1, path_table)
        kd = kp - gains.k_y * e_y - gains.k_theta * np.sin(e_theta)
        phi_des[k] = phi_from_kappa_nom(kd, ktable)
        kappa_des[k] = kd
        kappa_path[k] = kp
    return dict(phi_des=phi_des, phi_actual=X[:, 3], kappa_des=kappa_des, kappa_path=kappa_path)


def plot_front_rear_paths(res: Dict[str, np.ndarray], p: VehicleParams, title: str = ""):
    """XY paths of BOTH reference points (front P1, rear P2) for BOTH plant
    models, next to the commanded reference path -- one subplot per plant so
    the front/rear gap is readable.
    """
    table = res["path_table"]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 5.4))
    for ax, X, label, color in ((ax1, res["X_exact"], "exact", "#4C72B0"),
                                 (ax2, res["X_qs"], "quasi-static", "#DD8452")):
        front_xy = np.array([pose_at(s, p, RefPoint.FRONT)[:2] for s in X])
        rear_xy = np.array([pose_at(s, p, RefPoint.REAR)[:2] for s in X])
        ax.plot(table["x"], table["y"], color="gray", lw=2, ls="--", label="reference path")
        ax.plot(front_xy[:, 0], front_xy[:, 1], color=color, lw=2, label="front (P1)")
        ax.plot(rear_xy[:, 0], rear_xy[:, 1], color=color, lw=1.3, ls=":", label="rear (P2)")
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.set_title(f"{label} plant" + (f" -- {title}" if title else ""))
        ax.legend(fontsize=8)
    fig.tight_layout()
    return fig


def plot_articulation_servo(t: np.ndarray, diag_exact: Dict[str, np.ndarray],
                             diag_qs: Dict[str, np.ndarray], p: VehicleParams, title: str = ""):
    """phi_actual vs phi_des over time, for both plants -- the tracking
    behaviour of the articulation joint itself, isolated from the vehicle's
    path in the world frame.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.6), sharey=True)
    ax1.plot(t, diag_exact["phi_des"] / DEG, "k--", lw=1.3, label=r"$\phi_{des}$")
    ax1.plot(t, diag_exact["phi_actual"] / DEG, color="#4C72B0", lw=2, label=r"$\phi$ actual")
    ax1.axhline(p.phi_max / DEG, color="gray", lw=0.6, ls=":")
    ax1.axhline(-p.phi_max / DEG, color="gray", lw=0.6, ls=":")
    ax1.set_xlabel("t (s)"); ax1.set_ylabel(r"$\phi$ (deg)")
    ax1.set_title("Exact plant" + (f" -- {title}" if title else ""))
    ax1.legend(fontsize=8)

    ax2.plot(t, diag_qs["phi_des"] / DEG, "k--", lw=1.3, label=r"$\phi_{des}$")
    ax2.plot(t, diag_qs["phi_actual"] / DEG, color="#DD8452", lw=2, label=r"$\phi$ actual")
    ax2.axhline(p.phi_max / DEG, color="gray", lw=0.6, ls=":")
    ax2.axhline(-p.phi_max / DEG, color="gray", lw=0.6, ls=":")
    ax2.set_xlabel("t (s)")
    ax2.set_title("Quasi-static plant")
    ax2.legend(fontsize=8)
    fig.tight_layout()
    return fig


def plot_speeds_and_yaw_rates(t: np.ndarray, X: np.ndarray, p: VehicleParams,
                               v1_ref: float, label: str = ""):
    """Front/rear speed and yaw rate ACTUALLY realized along a logged
    trajectory, recovered by numerical differentiation of the logged pose
    (i.e. what really happened in that simulation, not a re-application of
    a formula that might silently assume the wrong plant).
    """
    theta1 = X[:, 2]
    phi = X[:, 3]
    theta2 = theta1 - phi
    th1_dot = np.gradient(theta1, t)
    th2_dot = np.gradient(theta2, t)

    front_xy = np.array([pose_at(s, p, RefPoint.FRONT)[:2] for s in X])
    rear_xy = np.array([pose_at(s, p, RefPoint.REAR)[:2] for s in X])
    v1_num = np.gradient(front_xy[:, 0], t) * np.cos(theta1) + np.gradient(front_xy[:, 1], t) * np.sin(theta1)
    v2_num = np.gradient(rear_xy[:, 0], t) * np.cos(theta2) + np.gradient(rear_xy[:, 1], t) * np.sin(theta2)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.6))
    ax1.plot(t, v1_num, color="#4C72B0", lw=2, label="$v_1$ (front, realized)")
    ax1.plot(t, v2_num, color="#DD8452", lw=2, label="$v_2$ (rear, realized)")
    ax1.axhline(v1_ref, color="k", lw=0.6, ls=":", label="$v_{ref}$ commanded")
    ax1.set_xlabel("t (s)"); ax1.set_ylabel("speed (m/s)")
    ax1.set_title(f"Realized speeds -- {label}")
    ax1.legend(fontsize=8)

    ax2.plot(t, th1_dot / DEG, color="#4C72B0", lw=2, label=r"$\dot\theta_1$ (front)")
    ax2.plot(t, th2_dot / DEG, color="#DD8452", lw=2, label=r"$\dot\theta_2$ (rear)")
    ax2.set_xlabel("t (s)"); ax2.set_ylabel("yaw rate (deg/s)")
    ax2.set_title("Realized yaw rates")
    ax2.legend(fontsize=8)
    fig.tight_layout()
    return fig


# ----------------------------------------------------------------------------- #
# Third entry point: extra diagnostics, reusing res_circ/res_s/gains already
# computed above.
# ----------------------------------------------------------------------------- #

if __name__ == "__main__":
    diag_circ_exact = reconstruct_controller_diagnostics(
        res_circ["t"], res_circ["X_exact"], p_mtt, res_circ["path_table"], gains)
    diag_circ_qs = reconstruct_controller_diagnostics(
        res_circ["t"], res_circ["X_qs"], p_mtt, res_circ["path_table"], gains)
    diag_s_exact = reconstruct_controller_diagnostics(
        res_s["t"], res_s["X_exact"], p_mtt, res_s["path_table"], gains)
    diag_s_qs = reconstruct_controller_diagnostics(
        res_s["t"], res_s["X_qs"], p_mtt, res_s["path_table"], gains)

    plot_front_rear_paths(res_circ, p_mtt, title="circular arc R=4m")
    plot_articulation_servo(res_circ["t"], diag_circ_exact, diag_circ_qs, p_mtt, title="circular arc R=4m")
    plot_speeds_and_yaw_rates(res_circ["t"], res_circ["X_exact"], p_mtt, gains.v_ref, label="exact, circular")
    plot_speeds_and_yaw_rates(res_circ["t"], res_circ["X_qs"], p_mtt, gains.v_ref, label="quasi-static, circular")

    plot_front_rear_paths(res_s, p_mtt, title="S-curve")
    plot_articulation_servo(res_s["t"], diag_s_exact, diag_s_qs, p_mtt, title="S-curve")
    plot_speeds_and_yaw_rates(res_s["t"], res_s["X_exact"], p_mtt, gains.v_ref, label="exact, S-curve")
    plot_speeds_and_yaw_rates(res_s["t"], res_s["X_qs"], p_mtt, gains.v_ref, label="quasi-static, S-curve")

    plt.show()

# =========================================================================== #
# INVERSION LADDER (closed loop): the SAME tracking law, three different
# feedforward inversions, against the M5 plant (the closest simulated
# stand-in for the real MTT before bags):
#
#   A. "m0" inversion: phi = kappa_nom^{-1}(kappa_des)      -- current WILN
#   B. "m2" inversion: phi = (gamma*kappa_nom)^{-1}(kappa_des)  -- fitted gain
#   C. "m5" inversion: deadband inverse (Karnopp)           -- theory eq. 27
#
# Where each fails: A carries both the slope deficit and the deadband; B
# fixes the slope but still hunts in the deadband; C settles clean.
# =========================================================================== #

from motion_model import M2Params, M5ContactParams, set_m5_params


def make_controller_v2(path_table: Dict[str, np.ndarray], p: VehicleParams,
                        gains: TrackingGains, phi_from_kappa) -> Callable:
    """Same law as make_controller, with a pluggable inversion feedforward."""
    def control_fn(t: float, state: np.ndarray) -> np.ndarray:
        x1, y1, th1, phi = state
        e_y, e_theta, kappa_p, _ = nearest_path_error(np.array([x1, y1]), th1, path_table)
        kappa_des = kappa_p - gains.k_y * e_y - gains.k_theta * np.sin(e_theta)
        phi_des = phi_from_kappa(kappa_des)
        phi_dot = np.clip(gains.k_phi * (phi_des - phi),
                          -p.phi_dot_max, p.phi_dot_max)
        return np.array([gains.v_ref, phi_dot])
    return control_fn


def run_inversion_ladder(path_fn: PathFn, s_max: float, p: VehicleParams,
                          gains: TrackingGains, m2: "M2Params",
                          m5: "M5ContactParams", dt: float = 0.02,
                          t_final: float = 16.0,
                          ref: RefPoint = RefPoint.FRONT) -> Dict[str, Dict]:
    """Run the three inversions against the M5 plant. Returns per-variant
    dicts with t, X, e_y series and tail RMS."""
    set_m5_params(m5)
    path_table = build_path_table(path_fn, s_max)
    k0 = build_kappa_inversion_table(p)

    inversions = {
        "m0": lambda kd: phi_from_kappa_nom(kd, k0),
        "m2": lambda kd: phi_from_kappa_nom(kd / m2.gamma, k0),
        "m5": lambda kd: m5.phi_from_kappa(kd, p),
    }
    out: Dict[str, Dict] = {}
    for name, inv in inversions.items():
        ctrl = make_controller_v2(path_table, p, gains, inv)
        t, X = simulate(np.zeros(4), ctrl, p, dt, t_final, model="m5")
        e_y = np.zeros(len(X))
        for k, st in enumerate(X):
            xr, yr, thr = pose_at(st, p, ref)
            e_y[k], _, _, _ = nearest_path_error(np.array([xr, yr]), thr, path_table)
        out[name] = dict(t=t, X=X, e_y=e_y,
                         rms=float(np.sqrt(np.mean(e_y[len(e_y)//2:] ** 2))))
    return out


def plot_inversion_ladder(lad: Dict[str, Dict], title: str = ""):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.6))
    colors = dict(m0="#C44E52", m2="#DD8452", m5="#4C72B0")
    for name, r in lad.items():
        ax1.plot(r["t"], r["e_y"] * 100, color=colors[name], lw=2,
                 label=f"{name} inversion (tail RMS {r['rms']*100:.1f} cm)")
        ax2.plot(r["t"], r["X"][:, 3] / DEG, color=colors[name], lw=1.6,
                 label=f"{name}: articulation")
    ax1.axhline(0, color="k", lw=0.6)
    ax1.set_xlabel("t (s)"); ax1.set_ylabel(r"cross-track $e_y$ (cm)")
    ax1.set_title("Inversion ladder vs M5 plant" + (f" -- {title}" if title else ""))
    ax1.legend(fontsize=8)
    ax2.set_xlabel("t (s)"); ax2.set_ylabel(r"$\phi$ (deg)")
    ax2.set_title("Articulation traces (deadband hunting visible)")
    ax2.legend(fontsize=8)
    fig.tight_layout()
    return fig


if __name__ == "__main__":
    m2 = M2Params()
    m5 = M5ContactParams.calibrated(p_mtt, gamma_slope=0.867, phi_dead_deg=2.0)

    lad = run_inversion_ladder(circular_arc_path(R=4.0), s_max=25.0, p=p_mtt,
                                gains=gains, m2=m2, m5=m5, t_final=16.0)
    print("[ladder] circular R=4m vs M5 plant, tail RMS: "
          + ", ".join(f"{k}={v['rms']*100:.2f} cm" for k, v in lad.items()))
    assert lad["m5"]["rms"] < lad["m2"]["rms"] < lad["m0"]["rms"], \
        "inversion ladder ordering violated"

    plot_inversion_ladder(lad, title="circular R=4m")
    plt.show()
