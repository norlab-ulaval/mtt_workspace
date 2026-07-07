"""
compare_models.py

Analysis script built on top of `motion_model.py`. It does NOT
plot anything -- every function returns plain numpy arrays / dicts so you
can plot whatever subset you want, however you want.

Three independent studies, each in its own section below:

  1. compare_yaw_rate_models(...)   -- exact (eq.19) vs quasi-static (eq.13/20)
                                        model, and the resulting trajectory
                                        drift over a short simulated horizon.

  2. compare_reference_points(...)  -- what changes if you command the vehicle
                                        from the front, the hitch, or the rear
                                        reference point. Includes the front<->
                                        rear inverse-kinematics round trip.

  3. icr_geometry(...) / icr_sensitivity_sweep(...)
                                     -- the instantaneous center of rotation
                                        (ICR) of the FRONT body and of the REAR
                                        body, computed independently. They
                                        coincide (single point, eq.7-9 of the
                                        paper) only when phi_dot = 0. This
                                        section quantifies how far apart they
                                        drift as a function of phi_dot, l1, l2,
                                        and how far each reference point sits
                                        from "its" ICR.

Requires motion_model.py in the same folder (or on PYTHONPATH).

Plotting functions live at the very end of the file (after the sanity-check
`__main__` block content) and are the only thing added on top of the
original analysis code -- nothing above was changed.
"""

from __future__ import annotations

from typing import Dict, Tuple

import numpy as np

from motion_model import (
    DEG,
    VehicleParams,
    RefPoint,
    turning_radius_front,
    turning_radius_rear,
    kappa_nom_front,
    front_yaw_rate,
    rear_yaw_rate,
    rear_speed_from_front,
    front_speed_from_rear,
    hitch_pose,
    rear_pose,
    pose_at,
    simulate,
)


# =========================================================================== #
# 1. Model comparison: exact (eq. 19) vs quasi-static (eq. 13/20)
# =========================================================================== #

def compare_yaw_rate_models(phi_deg: np.ndarray, v1: float, phi_dot: float,
                             p: VehicleParams) -> Dict[str, np.ndarray]:
    """Yaw-rate comparison across an articulation sweep, at fixed (v1, phi_dot).

    Returns arrays (all same length as phi_deg):
        theta1_dot_exact, theta1_dot_qs, delta_theta1_dot
            (= exact - quasi_static: the PURELY KINEMATIC part of a curvature
            residual, see `articulated_kinematics.curvature_residual` docstring)
        kappa_exact_equiv = theta1_dot_exact / v1   (an "effective" curvature,
            only meaningful for interpretation -- kappa_nom itself is only
            defined for the quasi-static model)
    """
    phi = np.asarray(phi_deg) * DEG
    th1_exact = front_yaw_rate(phi, phi_dot, v1, p)
    th1_qs = v1 * kappa_nom_front(phi, p)
    return dict(
        phi_deg=np.asarray(phi_deg),
        theta1_dot_exact=th1_exact,
        theta1_dot_qs=th1_qs,
        delta_theta1_dot=th1_exact - th1_qs,
        kappa_exact_equiv=th1_exact / v1,
    )


def compare_trajectories(control_fn, p: VehicleParams, dt: float, t_final: float,
                          x0: np.ndarray | None = None,
                          ref: RefPoint = RefPoint.FRONT) -> Dict[str, np.ndarray]:
    """Simulate the SAME control_fn(t, state) -> [v1, phi_dot] under both
    models and return both trajectories (at the chosen reference point) plus
    the final position/heading error between them. This turns the abstract
    "delta_theta1_dot" above into a concrete path-drift number.
    """
    if x0 is None:
        x0 = np.zeros(4)
    t, X_exact = simulate(x0, control_fn, p, dt, t_final, model="exact")
    _, X_qs = simulate(x0, control_fn, p, dt, t_final, model="quasi_static")

    def path_at(X):
        pts = np.array([pose_at(x, p, ref) for x in X])
        return pts  # columns: x, y, theta

    path_exact = path_at(X_exact)
    path_qs = path_at(X_qs)
    final_pos_err = np.linalg.norm(path_exact[-1, :2] - path_qs[-1, :2])
    final_heading_err = path_exact[-1, 2] - path_qs[-1, 2]
    return dict(
        t=t, X_exact=X_exact, X_qs=X_qs,
        path_exact=path_exact, path_qs=path_qs,
        final_pos_err=final_pos_err, final_heading_err=final_heading_err,
    )


# =========================================================================== #
# 2. Reference-point comparison (front / hitch / rear control)
# =========================================================================== #

def compare_reference_points(phi_deg: np.ndarray, phi_dot: float, v_cmd: float,
                              p: VehicleParams,
                              cmd_ref: RefPoint = RefPoint.FRONT) -> Dict[str, np.ndarray]:
    """Given a desired forward speed `v_cmd` expressed AT `cmd_ref`
    (front or rear -- hitch shares the front value, see library docstring),
    compute what the resulting speeds/yaw rates are at every reference point.

    This is the "which point do I steer from" comparison: e.g. cmd_ref=REAR
    answers "I want the implement/trailer to move at v_cmd -- what must the
    tractor do, and how does the resulting yaw rate compare to the front-
    referenced quasi-static kappa_nom the MTT report currently uses?"
    """
    phi = np.asarray(phi_deg) * DEG

    if cmd_ref == RefPoint.REAR:
        v2 = np.full_like(phi, v_cmd, dtype=float)
        v1 = front_speed_from_rear(phi, phi_dot, v2, p)
    else:  # FRONT or HITCH command -> identical v1 (see docstring in the lib)
        v1 = np.full_like(phi, v_cmd, dtype=float)
        v2 = rear_speed_from_front(phi, phi_dot, v1, p)

    th1_dot = front_yaw_rate(phi, phi_dot, v1, p)
    th2_dot = rear_yaw_rate(phi, phi_dot, v1, p)

    return dict(
        phi_deg=np.asarray(phi_deg),
        v1=v1, v2=v2,
        theta1_dot=th1_dot, theta2_dot=th2_dot,
        # required front command relative to its own v_max, useful to spot
        # where the inversion demands unreasonable tractor speeds
        v1_over_vmax=v1 / p.v_max,
    )


# =========================================================================== #
# 3. Instantaneous Center of Rotation (ICR) geometry
# =========================================================================== #
#
# Worked in a canonical "hitch frame": H = (0,0), front heading theta1 = 0.
# Results are translation/rotation invariant, so this is the natural frame
# for a parameter study (no absolute world pose needed).
#
#   P1 = (l1, 0)
#   P2 = (-l2*cos(phi), l2*sin(phi))         [theta2 = -phi]
#
# For any rigid body point X moving with velocity v and body angular rate
# omega, the ICR of that body is:  ICR = X + (-vy, vx) / omega
# (standard planar rigid-body kinematics, omega about +z).
# =========================================================================== #

def icr_geometry(phi_deg: float, phi_dot: float, v1: float,
                  p: VehicleParams) -> Dict[str, object]:
    """Full ICR geometry at one (phi, phi_dot, v1) operating point.

    Returns a dict with P1, P2, H, ICR_front, ICR_rear (2-vectors), the
    front/rear turning radii r_front=|v1/th1_dot|, r_rear=|v2/th2_dot|,
    the hitch-to-ICR distances for both bodies, and icr_separation = the
    distance between ICR_front and ICR_rear (0 iff phi_dot == 0, recovering
    the single point O of Corke & Ridley Fig. 3 / eq. 7-9).
    """
    phi = phi_deg * DEG
    H = np.zeros(2)
    P1 = np.array([p.l1, 0.0])
    P2 = np.array([-p.l2 * np.cos(phi), p.l2 * np.sin(phi)])

    th1_dot = float(front_yaw_rate(phi, phi_dot, v1, p))
    th2_dot = float(rear_yaw_rate(phi, phi_dot, v1, p))
    v2 = float(rear_speed_from_front(phi, phi_dot, v1, p))

    def icr_of(point, heading, omega, speed):
        if abs(omega) < 1e-9:
            return np.array([np.nan, np.nan])  # straight line, ICR at infinity
        vx, vy = speed * np.cos(heading), speed * np.sin(heading)
        return point + np.array([-vy, vx]) / omega

    ICR_front = icr_of(P1, 0.0, th1_dot, v1)
    ICR_rear = icr_of(P2, -phi, th2_dot, v2)

    r_front = np.linalg.norm(ICR_front - P1)
    r_rear = np.linalg.norm(ICR_rear - P2)
    r_hitch_front = np.linalg.norm(ICR_front - H)
    r_hitch_rear = np.linalg.norm(ICR_rear - H)
    icr_separation = np.linalg.norm(ICR_front - ICR_rear)

    return dict(
        phi_deg=phi_deg, phi_dot=phi_dot, v1=v1, v2=v2,
        H=H, P1=P1, P2=P2,
        theta1_dot=th1_dot, theta2_dot=th2_dot,
        ICR_front=ICR_front, ICR_rear=ICR_rear,
        r_front=r_front, r_rear=r_rear,
        r_hitch_front=r_hitch_front, r_hitch_rear=r_hitch_rear,
        icr_separation=icr_separation,
    )


def icr_sensitivity_sweep(phi_deg_range: np.ndarray, phi_dot_range: np.ndarray,
                           v1: float, p: VehicleParams) -> Dict[str, np.ndarray]:
    """2D parameter sweep over (phi, phi_dot) at fixed geometry `p` and speed v1.

    Returns 2D arrays (shape = [len(phi_dot_range), len(phi_deg_range)]) for
    r_front, r_rear, r_hitch_front, r_hitch_rear, icr_separation -- ready to
    hand to e.g. plt.pcolormesh / plt.contourf yourself.
    """
    PHI, PHID = np.meshgrid(phi_deg_range, phi_dot_range)  # shape (n_phidot, n_phi)
    shape = PHI.shape
    r_front = np.zeros(shape)
    r_rear = np.zeros(shape)
    r_hitch_front = np.zeros(shape)
    r_hitch_rear = np.zeros(shape)
    icr_sep = np.zeros(shape)

    for i in range(shape[0]):
        for j in range(shape[1]):
            g = icr_geometry(PHI[i, j], PHID[i, j], v1, p)
            r_front[i, j] = g["r_front"]
            r_rear[i, j] = g["r_rear"]
            r_hitch_front[i, j] = g["r_hitch_front"]
            r_hitch_rear[i, j] = g["r_hitch_rear"]
            icr_sep[i, j] = g["icr_separation"]

    return dict(
        phi_deg_grid=PHI, phi_dot_grid=PHID,
        r_front=r_front, r_rear=r_rear,
        r_hitch_front=r_hitch_front, r_hitch_rear=r_hitch_rear,
        icr_separation=icr_sep,
    )


def geometry_sensitivity_sweep(l1_range: np.ndarray, l2_range: np.ndarray,
                                phi_deg: float, phi_dot: float,
                                v1: float, base_p: VehicleParams) -> Dict[str, np.ndarray]:
    """2D parameter sweep over (l1, l2) at fixed (phi, phi_dot, v1) -- the
    "influence of the radii" study. track_width/phi_max/etc. are copied from
    `base_p` unchanged. Returns the same fields as icr_sensitivity_sweep.
    """
    L1, L2 = np.meshgrid(l1_range, l2_range)  # shape (n_l2, n_l1)
    shape = L1.shape
    r_front = np.zeros(shape)
    r_rear = np.zeros(shape)
    icr_sep = np.zeros(shape)
    ratio = np.zeros(shape)

    for i in range(shape[0]):
        for j in range(shape[1]):
            p_ij = VehicleParams(l1=L1[i, j], l2=L2[i, j],
                                  track_width=base_p.track_width,
                                  phi_max=base_p.phi_max,
                                  phi_dot_max=base_p.phi_dot_max,
                                  v_max=base_p.v_max)
            g = icr_geometry(phi_deg, phi_dot, v1, p_ij)
            r_front[i, j] = g["r_front"]
            r_rear[i, j] = g["r_rear"]
            icr_sep[i, j] = g["icr_separation"]
            ratio[i, j] = g["r_rear"] / g["r_front"] if g["r_front"] != 0 else np.nan

    return dict(
        l1_grid=L1, l2_grid=L2,
        r_front=r_front, r_rear=r_rear,
        icr_separation=icr_sep, radius_ratio=ratio,
    )


# =========================================================================== #
# Sanity checks (no plotting) -- run with `python compare_models.py`
# =========================================================================== #

if __name__ == "__main__":
    p_mtt = VehicleParams.mtt154()

    # --- 1. model comparison: exact reduces to quasi-static when phi_dot=0 ---
    res = compare_yaw_rate_models(np.array([10., 30., 50.]), v1=2.0, phi_dot=0.0, p=p_mtt)
    assert np.allclose(res["delta_theta1_dot"], 0.0, atol=1e-12)

    res_dyn = compare_yaw_rate_models(np.linspace(-59, 59, 200), v1=2.0, phi_dot=15 * DEG, p=p_mtt)
    assert np.all(np.abs(res_dyn["delta_theta1_dot"]) > 0)

    def ctrl(t, x):
        return np.array([1.5, 20 * DEG * np.cos(0.5 * t)])

    traj = compare_trajectories(ctrl, p_mtt, dt=0.02, t_final=6.0, ref=RefPoint.REAR)
    assert traj["final_pos_err"] > 0.0  # exact and quasi-static models must diverge
    print(f"[1] exact vs quasi-static, final REAR position drift over 6 s: "
          f"{traj['final_pos_err']*100:.1f} cm")

    # --- 2. reference-point comparison ---
    rp = compare_reference_points(np.linspace(-59, 59, 200), phi_dot=10 * DEG, v_cmd=1.0,
                                   p=p_mtt, cmd_ref=RefPoint.REAR)
    # round-trip: feeding v1 back through rear_speed_from_front must return v_cmd
    v2_check = rear_speed_from_front(rp["phi_deg"] * DEG, 10 * DEG, rp["v1"], p_mtt)
    assert np.allclose(v2_check, 1.0)
    print(f"[2] to hold v2=1.0 m/s at phi_dot=10 deg/s, tractor v1 ranges over "
          f"[{rp['v1'].min():.3f}, {rp['v1'].max():.3f}] m/s")

    # --- 3. ICR geometry: phi_dot=0 collapses front/rear ICR onto one point ---
    g0 = icr_geometry(30.0, 0.0, 2.0, p_mtt)
    assert g0["icr_separation"] < 1e-9
    assert np.isclose(g0["r_front"], abs(turning_radius_front(30 * DEG, p_mtt)))
    assert np.isclose(g0["r_rear"], abs(turning_radius_rear(30 * DEG, p_mtt)))

    g1 = icr_geometry(30.0, 15 * DEG, 2.0, p_mtt)
    assert g1["icr_separation"] > 0.0
    print(f"[3] at phi=30deg, v1=2 m/s: ICR separation is "
          f"{g0['icr_separation']*1000:.1f} mm at phi_dot=0 vs "
          f"{g1['icr_separation']:.3f} m at phi_dot=15 deg/s")

    sweep = icr_sensitivity_sweep(np.linspace(5, 59, 60), np.linspace(0, 25, 40) * DEG,
                                   v1=2.0, p=p_mtt)
    assert sweep["icr_separation"].shape == (40, 60)

    gsweep = geometry_sensitivity_sweep(np.linspace(0.3, 2.0, 40), np.linspace(0.3, 2.0, 40),
                                         phi_deg=30.0, phi_dot=10 * DEG, v1=2.0, base_p=p_mtt)
    assert gsweep["radius_ratio"].shape == (40, 40)

    print("All sanity checks passed.")


# =========================================================================== #
# Plotting (matplotlib only, added on top of the analysis code above)
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


def plot_yaw_rate_comparison(res: Dict[str, np.ndarray], phi_dot: float = 0.0):
    """Figure for `compare_yaw_rate_models` output: exact vs quasi-static
    yaw rate across phi, plus their difference (the kinematic Delta_kappa*v).
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.3))
    ax1.plot(res["phi_deg"], res["theta1_dot_exact"] / DEG, label="exact (eq. 19)", lw=2)
    ax1.plot(res["phi_deg"], res["theta1_dot_qs"] / DEG, "--", label="quasi-static (eq. 13/20)", lw=2)
    ax1.set_xlabel(r"$\phi$ (deg)")
    ax1.set_ylabel(r"$\dot\theta_1$ (deg/s)")
    ax1.set_title(fr"Yaw rate, $\dot\phi={phi_dot/DEG:.0f}^\circ$/s")
    ax1.legend(fontsize=9)

    ax2.plot(res["phi_deg"], res["delta_theta1_dot"] / DEG, color="#C44E52", lw=2)
    ax2.axhline(0, color="k", lw=0.6)
    ax2.set_xlabel(r"$\phi$ (deg)")
    ax2.set_ylabel(r"$\dot\theta_{1,exact}-\dot\theta_{1,qs}$ (deg/s)")
    ax2.set_title("Purely kinematic yaw-rate residual")
    fig.tight_layout()
    return fig


def plot_trajectories(traj: Dict[str, np.ndarray], ref_label: str = "FRONT"):
    """Figure for `compare_trajectories` output: XY path of exact vs
    quasi-static plant under the same control, plus heading over time.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.6))
    ax1.plot(traj["path_exact"][:, 0], traj["path_exact"][:, 1], label="exact", lw=2)
    ax1.plot(traj["path_qs"][:, 0], traj["path_qs"][:, 1], "--", label="quasi-static", lw=2)
    ax1.plot(traj["path_exact"][0, 0], traj["path_exact"][0, 1], "ko", ms=5)
    ax1.set_aspect("equal")
    ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
    ax1.set_title(f"{ref_label}-referenced path, same control input")
    ax1.legend(fontsize=9)

    ax2.plot(traj["t"], traj["path_exact"][:, 2] / DEG, label="exact", lw=2)
    ax2.plot(traj["t"], traj["path_qs"][:, 2] / DEG, "--", label="quasi-static", lw=2)
    ax2.set_xlabel("t (s)"); ax2.set_ylabel(r"$\theta$ (deg)")
    ax2.set_title(f"Heading, final drift = {traj['final_pos_err']*100:.1f} cm")
    ax2.legend(fontsize=9)
    fig.tight_layout()
    return fig


def plot_reference_points(rp: Dict[str, np.ndarray], cmd_ref_label: str = "REAR"):
    """Figure for `compare_reference_points` output."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.3))
    ax1.plot(rp["phi_deg"], rp["v1"], label="$v_1$ (front)", lw=2)
    ax1.plot(rp["phi_deg"], rp["v2"], label="$v_2$ (rear)", lw=2)
    ax1.set_xlabel(r"$\phi$ (deg)"); ax1.set_ylabel("speed (m/s)")
    ax1.set_title(f"Speeds, command referenced at {cmd_ref_label}")
    ax1.legend(fontsize=9)

    ax2.plot(rp["phi_deg"], rp["theta1_dot"] / DEG, label=r"$\dot\theta_1$", lw=2)
    ax2.plot(rp["phi_deg"], rp["theta2_dot"] / DEG, label=r"$\dot\theta_2$", lw=2)
    ax2.set_xlabel(r"$\phi$ (deg)"); ax2.set_ylabel("yaw rate (deg/s)")
    ax2.set_title("Front vs rear body yaw rate")
    ax2.legend(fontsize=9)
    fig.tight_layout()
    return fig


def plot_icr_snapshot(g: Dict[str, object], p: VehicleParams):
    """Single-operating-point geometric snapshot: hitch, P1, P2, ICR_front,
    ICR_rear, and the radii connecting them (from `icr_geometry` output).
    """
    fig, ax = plt.subplots(figsize=(6, 6))
    H, P1, P2 = g["H"], g["P1"], g["P2"]
    ICRf, ICRr = g["ICR_front"], g["ICR_rear"]

    ax.plot(*H, "ks", ms=8, label="H (hitch)")
    ax.plot(*P1, "o", color="#4C72B0", ms=8, label="P1 (front)")
    ax.plot(*P2, "o", color="#DD8452", ms=8, label="P2 (rear)")
    if not np.any(np.isnan(ICRf)):
        ax.plot(*ICRf, "^", color="#4C72B0", ms=10, label="ICR front")
        ax.plot([P1[0], ICRf[0]], [P1[1], ICRf[1]], color="#4C72B0", lw=1, ls=":")
    if not np.any(np.isnan(ICRr)):
        ax.plot(*ICRr, "^", color="#DD8452", ms=10, label="ICR rear")
        ax.plot([P2[0], ICRr[0]], [P2[1], ICRr[1]], color="#DD8452", lw=1, ls=":")
    ax.plot([P1[0], H[0]], [P1[1], H[1]], "k-", lw=1.5)
    ax.plot([P2[0], H[0]], [P2[1], H[1]], "k-", lw=1.5)

    ax.set_aspect("equal")
    ax.set_xlabel("x (m, hitch frame)"); ax.set_ylabel("y (m, hitch frame)")
    ax.set_title(fr"ICR snapshot: $\phi$={g['phi_deg']:.0f}$^\circ$, "
                 fr"$\dot\phi$={g['phi_dot']/DEG:.0f}$^\circ$/s, "
                 fr"separation={g['icr_separation']:.2f} m")
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    return fig


def plot_icr_sweep(sweep: Dict[str, np.ndarray]):
    """Heatmap of ICR separation over (phi, phi_dot), from `icr_sensitivity_sweep`."""
    fig, ax = plt.subplots(figsize=(7, 5))
    pcm = ax.pcolormesh(sweep["phi_deg_grid"], sweep["phi_dot_grid"] / DEG,
                         sweep["icr_separation"], shading="auto", cmap="viridis")
    fig.colorbar(pcm, ax=ax, label="ICR separation (m)")
    ax.set_xlabel(r"$\phi$ (deg)"); ax.set_ylabel(r"$\dot\phi$ (deg/s)")
    ax.set_title("Front/rear ICR separation vs articulation state")
    fig.tight_layout()
    return fig


def plot_geometry_sweep(gsweep: Dict[str, np.ndarray]):
    """Heatmaps of radius ratio and ICR separation over (l1, l2), from
    `geometry_sensitivity_sweep`.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    pcm1 = ax1.pcolormesh(gsweep["l1_grid"], gsweep["l2_grid"], gsweep["radius_ratio"],
                           shading="auto", cmap="coolwarm")
    fig.colorbar(pcm1, ax=ax1, label=r"$r_2/r_1$")
    ax1.set_xlabel("$l_1$ (m)"); ax1.set_ylabel("$l_2$ (m)")
    ax1.set_title("Radius ratio vs geometry")

    pcm2 = ax2.pcolormesh(gsweep["l1_grid"], gsweep["l2_grid"], gsweep["icr_separation"],
                           shading="auto", cmap="viridis")
    fig.colorbar(pcm2, ax=ax2, label="ICR separation (m)")
    ax2.set_xlabel("$l_1$ (m)"); ax2.set_ylabel("$l_2$ (m)")
    ax2.set_title("ICR separation vs geometry")
    fig.tight_layout()
    return fig


# ----------------------------------------------------------------------------- #
# Second entry point: only runs the plots, reusing the results already
# computed in the sanity-check __main__ block above (res_dyn, traj, rp, g1,
# sweep, gsweep, p_mtt are plain module-level globals by the time this runs).
# ----------------------------------------------------------------------------- #

if __name__ == "__main__":
    plot_yaw_rate_comparison(res_dyn, phi_dot=15 * DEG)
    plot_trajectories(traj, ref_label="REAR")
    plot_reference_points(rp, cmd_ref_label="REAR")
    plot_icr_snapshot(g1, p_mtt)
    plot_icr_sweep(sweep)
    plot_geometry_sweep(gsweep)
    plt.show()

# =========================================================================== #
# 4. THE MODEL LADDER: curvature laws M0 / M2 / M5 side by side, and where
#    each one fails against a simulated "ground truth" (M5 + kinetic
#    hysteresis, the most faithful plant available before real bags).
#    Requires the M-hierarchy section of motion_model.py.
# =========================================================================== #

from motion_model import (M2Params, M5ContactParams, M5HysteresisPlant, dz,
                          kappa_nom_front_smallangle)


def compare_curvature_laws(p: VehicleParams, m2: "M2Params", m5: "M5ContactParams",
                            phi_deg_grid: np.ndarray) -> Dict[str, np.ndarray]:
    """kappa(phi) for every model of the hierarchy on a common phi grid."""
    phi = phi_deg_grid * DEG
    return dict(
        phi_deg=phi_deg_grid,
        m0_exact=np.array([kappa_nom_front(x, p) for x in phi]),
        m0_small=np.array([kappa_nom_front_smallangle(x, p) for x in phi]),
        m2=np.array([m2.kappa(x, p) for x in phi]),
        m5=np.array([m5.kappa(x, p) for x in phi]),
    )


def hysteresis_truth(p: VehicleParams, m5: "M5ContactParams",
                      phi_amp_deg: float = 20.0, n: int = 1200) -> Dict[str, np.ndarray]:
    """Triangular phi sweep through the kinetic play plant: the simulated
    'ground truth' (phi_series, kappa_series), rate-independent by
    construction so no speed needs to be specified."""
    a = phi_amp_deg * DEG
    phis = np.concatenate([np.linspace(0, a, n // 4),
                           np.linspace(a, -a, n // 2),
                           np.linspace(-a, a, n // 2)])
    hp = M5HysteresisPlant(m5, p)
    ks = np.array([hp.kappa_step(x) for x in phis])
    return dict(phi=phis, kappa=ks)


def model_ladder_errors(p: VehicleParams, m2: "M2Params", m5: "M5ContactParams",
                         phi_amp_deg: float = 20.0) -> Dict[str, Dict[str, float]]:
    """Two honest comparisons, because hysteresis makes 'the' error ill-posed:

    VIRGIN branch (monotonic first loading from rest, 0 -> amp): here the
    memoryless dead-zone law is the exact M5 prediction, so the ladder reads
    m0 > m2 > m5 ~ 0 -- slope deficit and threshold, each fixed in turn.

    FULL LOOP (triangular sweep with reversals): NO memoryless law can be
    exact -- the best memoryless predictor of a symmetric loop is its
    centerline (which is what M2's fitted-gain line approximates!), while
    only the stateful play operator (M5 + internal state) matches. This is
    the quantitative statement that reversal maneuvers *require* the
    hysteresis state, and explains why a fitted gamma can look better than
    the physically-correct virgin law on S-curve data.
    """
    hp = M5HysteresisPlant(m5, p)
    # --- virgin branch ---
    phi_v = np.linspace(0, phi_amp_deg * DEG, 400)
    hp.reset()
    k_v = np.array([hp.kappa_step(x) for x in phi_v])
    virgin = {
        "m0_exact": np.array([kappa_nom_front(x, p) for x in phi_v]),
        "m2": np.array([m2.kappa(x, p) for x in phi_v]),
        "m5": np.array([m5.kappa(x, p) for x in phi_v]),
    }
    virgin_err = {k: float(np.sqrt(np.mean((v - k_v) ** 2)))
                  for k, v in virgin.items()}
    # --- full loop ---
    tr = hysteresis_truth(p, m5, phi_amp_deg)
    hp2 = M5HysteresisPlant(m5, p)
    loop = {
        "m0_exact": np.array([kappa_nom_front(x, p) for x in tr["phi"]]),
        "m2": np.array([m2.kappa(x, p) for x in tr["phi"]]),
        "m5_memoryless": np.array([m5.kappa(x, p) for x in tr["phi"]]),
        "m5_play": np.array([hp2.kappa_step(x) for x in tr["phi"]]),
    }
    loop_err = {k: float(np.sqrt(np.mean((v - tr["kappa"]) ** 2)))
                for k, v in loop.items()}
    return dict(virgin=virgin_err, loop=loop_err)


def plot_curvature_ladder(laws: Dict[str, np.ndarray], p: VehicleParams,
                           m5: "M5ContactParams", truth: Dict[str, np.ndarray] | None = None):
    fig, ax = plt.subplots(figsize=(8.5, 5.2))
    ax.plot(laws["phi_deg"], laws["m0_exact"], lw=2.2, label="M0 exact (Corke eq.13)")
    ax.plot(laws["phi_deg"], laws["m0_small"], lw=1.2, ls=":", label="M0 small-angle")
    ax.plot(laws["phi_deg"], laws["m2"], lw=2.2, label=f"M2 empirical (gamma)")
    ax.plot(laws["phi_deg"], laws["m5"], lw=2.4, label="M5 deadband-affine (eq.27)")
    if truth is not None:
        ax.plot(truth["phi"] / DEG, truth["kappa"], color="k", lw=1.0, alpha=0.55,
                label="truth: M5 + kinetic play (loop)")
    ax.axvspan(-m5.phi_dead_k / DEG, m5.phi_dead_k / DEG, color="gray", alpha=0.15,
               label=r"deadband $\pm\varphi_{dead}$")
    ax.set_xlabel(r"$\phi$ (deg)"); ax.set_ylabel(r"$\kappa$ (1/m)")
    ax.set_title("The model ladder: curvature laws M0 / M2 / M5 and the hysteretic truth")
    ax.legend(fontsize=8); fig.tight_layout()
    return fig


def plot_ladder_errors(errs: Dict[str, float]):
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    names = list(errs.keys()); vals = [errs[k] for k in names]
    ax.bar(names, vals, color=["#C44E52", "#DD8452", "#4C72B0"])
    ax.set_ylabel("RMS curvature prediction error (1/m)")
    ax.set_title("Where each model fails (vs hysteretic M5 truth)")
    for i, v in enumerate(vals):
        ax.text(i, v, f"{v:.4f}", ha="center", va="bottom", fontsize=9)
    fig.tight_layout()
    return fig


if __name__ == "__main__":
    m2 = M2Params()
    # calibrated truth: slope gain = the MEASURED gamma (0.867 indoor),
    # deadband set to a field-plausible 2 deg (to be identified from bags)
    m5 = M5ContactParams.calibrated(p_mtt, gamma_slope=0.867, phi_dead_deg=2.0)
    errs = model_ladder_errors(p_mtt, m2, m5, phi_amp_deg=20.0)
    print("[4] VIRGIN-branch RMS errors: "
          + ", ".join(f"{k}={v:.4f}" for k, v in errs["virgin"].items()))
    print("[4] FULL-LOOP RMS errors:    "
          + ", ".join(f"{k}={v:.4f}" for k, v in errs["loop"].items()))
    assert errs["virgin"]["m5"] < errs["virgin"]["m2"] < errs["virgin"]["m0_exact"]
    assert errs["loop"]["m5_play"] < 1e-12 < errs["loop"]["m5_memoryless"]

    laws = compare_curvature_laws(p_mtt, m2, m5, np.linspace(-25, 25, 400))
    tr = hysteresis_truth(p_mtt, m5, 20.0)
    plot_curvature_ladder(laws, p_mtt, m5, truth=tr)
    plot_ladder_errors(errs["virgin"])
    plt.show()
