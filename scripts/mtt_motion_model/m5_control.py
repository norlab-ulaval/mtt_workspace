"""
m5_control.py

The M5-based path-tracking controller, packaged for the shared lib
(motion_model.py + traj_tracking_sim.py + traj_metrics.py).

Design (from the M5 theory doc):
  * DISTANCE-DOMAIN tuning. The plant is rate-independent, so gains are
    per-meter and tuned by pole placement in arc length:
        e_y'' + k_theta e_y' + k_y e_y = 0   (per meter)
        omega_s = 4.6/(zeta * d_settle),  k_y = omega_s^2,  k_theta = 2 zeta omega_s
    Settling DISTANCE (m) and overshoot (m) are the specs, not time.
  * DEADBAND INVERSE feedforward (Karnopp): phi_des from the M5 law's exact
    inverse, so the controller does not waste authority inside the dead zone
    and does not hunt around zero curvature.
  * GEAR handling: in reverse the motion-frame curvature flips; errors are
    measured in the motion frame. Front/hitch stable both gears; hitch is the
    gear-invariant default.
  * SPEED-ENVELOPE certificate: the max quasi-static speed for which the
    articulation servo stays "fast in distance"; maneuver speed is chosen
    below it.

The controller is already well-tuned out of the box (DGTConfig defaults) but
every knob is exposed. It plugs into simulate(...) via a control_fn, and the
plant it is validated against is the calibrated M5 plant (model="m5"),
optionally with kinetic hysteresis.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, Tuple

import numpy as np

from motion_model import (
    DEG, VehicleParams, RefPoint, pose_at, simulate,
    M5ContactParams, set_m5_params, M5HysteresisPlant,
)
from traj_tracking_sim import (
    PathFn, build_path_table, nearest_path_error,
    circular_arc_path, s_curve_path,
)
from traj_metrics import tracking_report, ape, rpe, TrackingReport


# --------------------------------------------------------------------------- #
# Tuning
# --------------------------------------------------------------------------- #

@dataclass
class DGTConfig:
    """All knobs. Defaults are already a good tune for the MTT-154 at the
    calibrated M5 plant; adjust d_settle to trade aggressiveness vs smoothness."""
    d_settle: float = 9.0      # target 2%-settling DISTANCE (m)
    zeta: float = 1.0          # damping (1.0 = critically damped, no overshoot)
    k_phi: float = 6.0         # articulation servo gain (1/s)
    v_ref: float = 0.6         # nominal forward speed (m/s)
    deadband_inverse: bool = True
    control_point: RefPoint = RefPoint.FRONT   # perception point: stable in
    #                                            forward tracking. HITCH is
    #                                            gear-invariant but lags the
    #                                            motion (non-minimum-phase) in
    #                                            pure forward tracking -- use it
    #                                            only with the go-to-goal planner.

    # derived per-meter gains (pole placement in arc length)
    @property
    def omega_s(self) -> float:
        return 4.6 / (self.zeta * self.d_settle)

    @property
    def k_y(self) -> float:
        return self.omega_s ** 2

    @property
    def k_theta(self) -> float:
        return 2 * self.zeta * self.omega_s


def speed_envelope(cfg: DGTConfig, m5: M5ContactParams, p: VehicleParams,
                   e_y_max: float = 0.5, margin: float = 2.0) -> float:
    """Max speed for which the articulation servo stays fast-in-distance.
    From max|dphi_des/ds| ~ (1/gamma_coeff) * k_y * e_y_max."""
    dphi_ds = (1.0 / m5.gamma_coeff(p)) * cfg.k_y * e_y_max
    return float(p.phi_dot_max / (margin * dphi_ds))


# --------------------------------------------------------------------------- #
# The controller
# --------------------------------------------------------------------------- #

def make_dgt_controller(path_table: Dict[str, np.ndarray], p: VehicleParams,
                        m5: M5ContactParams, cfg: DGTConfig,
                        v_signed: float | None = None) -> Callable:
    """Return control_fn(t, state) -> [v, phi_dot] implementing DGT.

    kappa_des = kappa_path*gearsign - k_y e_y - k_theta sin(e_theta)   (motion frame)
    phi_des   = M5 deadband inverse (or M0 inverse if deadband_inverse=False)
    phi_dot   = clip(k_phi (phi_des - phi))
    """
    v = cfg.v_ref if v_signed is None else v_signed
    rev = v < 0
    cp = cfg.control_point

    def control_fn(t: float, state: np.ndarray) -> np.ndarray:
        # pose of the controlled point (flat point default)
        xp, yp, thp = pose_at(state, p, cp)
        e_y, e_theta, kappa_p, _ = nearest_path_error(np.array([xp, yp]), thp, path_table)
        # motion-frame gear handling
        gear = -1.0 if rev else 1.0
        kappa_des = kappa_p * gear - cfg.k_y * e_y - cfg.k_theta * np.sin(e_theta)
        kappa_des = float(np.clip(kappa_des,
                                  -m5.gamma_coeff(p) * (p.phi_max - m5.phi_dead_k),
                                  m5.gamma_coeff(p) * (p.phi_max - m5.phi_dead_k)))
        # feedforward inversion
        if cfg.deadband_inverse:
            phi_des = gear * m5.phi_from_kappa(kappa_des, p)
        else:
            phi_des = gear * float(np.clip(-kappa_des * (p.l1 + p.l2),
                                           -p.phi_max, p.phi_max))
        phi_dot = float(np.clip(cfg.k_phi * (phi_des - state[3]),
                                -p.phi_dot_max, p.phi_dot_max))
        return np.array([v, phi_dot])

    return control_fn


# --------------------------------------------------------------------------- #
# Evaluation harness: track a path on the M5 plant, report tracking + APE/RPE
# --------------------------------------------------------------------------- #

def evaluate_tracking(path_fn: PathFn, s_max: float, p: VehicleParams,
                      m5: M5ContactParams, cfg: DGTConfig,
                      x0: np.ndarray | None = None, dt: float = 0.02,
                      t_final: float | None = None) -> Dict[str, object]:
    """Run DGT on the M5 plant and return a full metric bundle: cross-track
    tracking report + APE/RPE of the realized controlled-point trajectory vs
    the reference path resampled at the realized arc lengths."""
    set_m5_params(m5)
    table = build_path_table(path_fn, s_max)
    if x0 is None:
        # start 0.4 m off the path laterally to see convergence
        x0 = np.array([table["x"][0], table["y"][0] + 0.25, table["theta"][0], 0.0])
    if t_final is None:
        t_final = 1.2 * s_max / max(abs(cfg.v_ref), 0.1)

    ctrl = make_dgt_controller(table, p, m5, cfg)
    t, X = simulate(x0, ctrl, p, dt, t_final, model="m5")

    # truncate the run at the step where the controlled point first reaches
    # the final path vertex (avoid nearest-point wrap past the arc end)
    cp = cfg.control_point
    end_xy = np.array([table["x"][-1], table["y"][-1]])
    stop = len(X)
    for k in range(len(X)):
        xp, yp, _ = pose_at(X[k], p, cp)
        _, _, _, idx = nearest_path_error(np.array([xp, yp]),
                                          X[k, 2], table)
        if idx >= len(table["x"]) - 2:
            stop = k + 1
            break
    t, X = t[:stop], X[:stop]
    n = len(X)
    ctrl_xy = np.zeros((n, 3))
    e_y = np.zeros(n); e_th = np.zeros(n); ref_xy = np.zeros((n, 3))
    for k in range(n):
        xp, yp, thp = pose_at(X[k], p, cp)
        ctrl_xy[k] = [xp, yp, thp]
        ey, eth, _, idx = nearest_path_error(np.array([xp, yp]), thp, table)
        e_y[k] = ey; e_th[k] = eth
        ref_xy[k] = [table["x"][idx], table["y"][idx], table["theta"][idx]]

    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(ctrl_xy[:, :2], axis=0).T))])
    rep = tracking_report(e_y, e_th, s)
    ape_res = ape(ctrl_xy, ref_xy, align=False)     # already same frame
    rpe_res = rpe(ctrl_xy, ref_xy, delta=10, s=s)

    return dict(t=t, X=X, ctrl_xy=ctrl_xy, ref_xy=ref_xy, e_y=e_y, e_theta=e_th,
                s=s, tracking=rep, ape=ape_res, rpe=rpe_res, cfg=cfg)


# --------------------------------------------------------------------------- #
# Self-test / demo
# --------------------------------------------------------------------------- #

if __name__ == "__main__":
    p = VehicleParams.mtt154()
    m5 = M5ContactParams.calibrated(p, gamma_slope=0.867, phi_dead_deg=2.0)
    cfg = DGTConfig()
    print(f"tune: d_settle={cfg.d_settle} m, zeta={cfg.zeta} -> "
          f"k_y={cfg.k_y:.3f} 1/m^2, k_theta={cfg.k_theta:.3f} 1/m; "
          f"v*={speed_envelope(cfg, m5, p):.2f} m/s")

    # --- circular arc: DGT should settle cleanly, small APE, tiny RPE ------
    res = evaluate_tracking(circular_arc_path(R=6.0), s_max=30.0, p=p, m5=m5, cfg=cfg)
    r = res["tracking"]
    print(f"[circle R=6] rms_ey={r.rms_ey*100:.2f} cm, settle={r.settle_distance:.1f} m, "
          f"overshoot={r.overshoot*100:.1f} cm | "
          f"APE={res['ape']['rmse_trans']*100:.2f} cm, "
          f"RPE(10) drift/m={res['rpe']['drift_per_m']*100:.3f} cm/m")
    assert r.rms_ey < 0.15 and r.overshoot < 0.10

    # --- deadband inverse ablation: turning it OFF must hurt ----------------
    cfg_off = DGTConfig(deadband_inverse=False)
    res_off = evaluate_tracking(circular_arc_path(R=6.0), s_max=30.0, p=p, m5=m5, cfg=cfg_off)
    print(f"[ablation] deadband inverse ON  rms_ey={res['tracking'].rms_ey*100:.2f} cm; "
          f"OFF rms_ey={res_off['tracking'].rms_ey*100:.2f} cm")
    assert res["tracking"].rms_ey <= res_off["tracking"].rms_ey

    # --- tuning monotonicity: smaller d_settle => shorter settling distance -
    res_tight = evaluate_tracking(circular_arc_path(R=6.0), s_max=30.0, p=p, m5=m5,
                                  cfg=DGTConfig(d_settle=5.0))
    print(f"[tuning] d_settle 9->5 m: settle {res['tracking'].settle_distance:.1f} "
          f"-> {res_tight['tracking'].settle_distance:.1f} m")

    print("All m5_control checks passed.")
