"""
articulated_kinematics.py

Pure kinematic motion-model library for a center-articulated two-body
vehicle (powered tractor + passive/actuated trailer, single hitch DOF),
such as the MTT-154.

Reference:
    P. I. Corke and P. Ridley, "Steering Kinematics for a Center-Articulated
    Mobile Robot," IEEE Trans. Robotics and Automation, 17(2), 2001.
    Equation numbers quoted in docstrings refer to that paper. Items marked
    [ext] are direct extensions/inversions derived from the same geometry
    (rear-referenced curvature, exact v1<->v2 coupling, inverse kinematics),
    not printed in the original paper but consistent with its eq. (12)-(21).

Geometry & conventions
-----------------------
    H   : hitch / articulation point (common to both bodies)
    P1  : front reference point, at distance l1 from H along the FRONT body axis
    P2  : rear reference point,  at distance l2 from H along the REAR body axis
    theta1 : world heading of the front body
    theta2 : world heading of the rear body
    phi    : articulation angle := theta1 - theta2   (rad)
             (matches Corke & Ridley's gamma with this sign convention)

    v1  : forward speed of the front body (== speed of P1 == the "forward"
          velocity component of H measured along the front body axis; these
          three are numerically identical for a rigid body, see derivation
          notes below function `front_yaw_rate`).
    v2  : forward speed of the rear body (== speed of P2).
    phi_dot : articulation rate (rad/s), the second control input (hydraulic
              cylinder / articulation servo).

State convention used throughout this module:

    state = [x1, y1, theta1, phi]     (pose of the FRONT body + articulation)

This is the natural state for the MTT-154 because the front tractor is the
powered/driven unit and phi is a directly actuated/measured joint angle.
Poses of the hitch or of the rear body are obtained on demand with
`hitch_pose`, `rear_pose`, `pose_at` -- no need to carry them in the state.

No plotting and no I/O in this module by design -- only numpy + dataclasses.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Tuple

import numpy as np

DEG = np.pi / 180.0


# --------------------------------------------------------------------------- #
# Parameters
# --------------------------------------------------------------------------- #

@dataclass
class VehicleParams:
    """Kinematic parameters of a center-articulated vehicle.

    l1 : hitch -> front reference point distance (m)   ["Lf" in the MTT report]
    l2 : hitch -> rear reference point distance (m)     ["Lr" in the MTT report]
    track_width : center-to-center track/wheel distance (m), not used by the
        planar models below but kept here since it is needed for wheel/track
        speed differentials (eq. 10-11 of the paper) if you extend the lib.
    phi_max : hard articulation limit (rad), symmetric +/-.
    phi_dot_max : articulation servo rate limit (rad/s), optional, purely
        informative (not enforced anywhere in this module).
    v_max : forward speed limit (m/s), optional, purely informative.
    """
    l1: float
    l2: float
    track_width: float = 1.2
    phi_max: float = 60 * DEG
    phi_dot_max: float = 30 * DEG
    v_max: float = 4.0

    # MTT-154 measured values (internship report, Table 3)
    @classmethod
    def mtt154(cls) -> "VehicleParams":
        return cls(l1=0.9, l2=1.5, track_width=1.2, phi_max=60 * DEG,
                   phi_dot_max=30 * DEG, v_max=4.0)


class RefPoint(str, Enum):
    """Which body-fixed point is used as base_link."""
    FRONT = "front"   # P1  -- tractor reference point
    HITCH = "hitch"   # H   -- articulation center
    REAR = "rear"     # P2  -- trailer / implement reference point


# --------------------------------------------------------------------------- #
# Steady-state geometry (valid for phi_dot = 0, i.e. constant-radius turning)
# --------------------------------------------------------------------------- #

def turning_radius_front(phi: float, p: VehicleParams) -> float:
    """r1, eq. (7): radius of the circle traced by P1 while turning at constant phi."""
    return (p.l1 * np.cos(phi) + p.l2) / np.sin(phi)


def turning_radius_rear(phi: float, p: VehicleParams) -> float:
    """r2, eq. (8): radius of the circle traced by P2 while turning at constant phi."""
    return (p.l2 * np.cos(phi) + p.l1) / np.sin(phi)


def radius_ratio(phi: float, p: VehicleParams) -> float:
    """r2/r1, eq. (9). Equals 1 iff l1 == l2 or phi == 0."""
    return turning_radius_rear(phi, p) / turning_radius_front(phi, p)


# --------------------------------------------------------------------------- #
# Quasi-static ("bicycle") nominal curvature models -- phi_dot term ignored
# --------------------------------------------------------------------------- #

def kappa_nom_front(phi: float, p: VehicleParams) -> float:
    """Exact nominal curvature at P1, eq. (13)/(20): kappa = sin(phi)/(l1 cos(phi)+l2).
    theta1_dot = v1 * kappa_nom_front(phi) under the quasi-static assumption phi_dot=0.
    This is exactly the kappa_nom(phi) used in the MTT curvature-residual framework.
    """
    return np.sin(phi) / (p.l1 * np.cos(phi) + p.l2)


def kappa_nom_front_smallangle(phi: float, p: VehicleParams) -> float:
    """Small-angle approximation, eq. (14): kappa ~= tan(phi) / (l1+l2). Valid |phi| < ~15-30 deg."""
    return np.tan(phi) / (p.l1 + p.l2)


def kappa_nom_rear(phi: float, p: VehicleParams) -> float:
    """[ext] Exact nominal curvature at P2 (mirror of eq. 13, front<->rear, phi->-phi):
    theta2_dot = v2 * kappa_nom_rear(phi) under the quasi-static assumption phi_dot=0.
    """
    return -np.sin(phi) / (p.l2 * np.cos(phi) + p.l1)


# --------------------------------------------------------------------------- #
# Exact dynamic model (includes phi_dot) -- Corke & Ridley eq. (19), (21)
# --------------------------------------------------------------------------- #

def front_yaw_rate(phi: float, phi_dot: float, v1: float, p: VehicleParams) -> float:
    """theta1_dot, eq. (19), full nonholonomic model (no small-angle, no phi_dot=0
    assumption). Reduces to v1*kappa_nom_front(phi) when phi_dot=0.

        theta1_dot = (v1*sin(phi) + l2*phi_dot) / (l1*cos(phi) + l2)

    Derivation sketch (self-contained, matches the paper's result):
    Enforcing zero lateral velocity at P1 and at P2 simultaneously, together
    with the rigid chain P1-H-P2, yields this closed form. See module
    docstring for point/frame definitions.
    """
    return (v1 * np.sin(phi) + p.l2 * phi_dot) / (p.l1 * np.cos(phi) + p.l2)


def rear_yaw_rate(phi: float, phi_dot: float, v1: float, p: VehicleParams) -> float:
    """theta2_dot, eq. (21): theta1_dot - theta2_dot = phi_dot  =>  theta2_dot = theta1_dot - phi_dot."""
    return front_yaw_rate(phi, phi_dot, v1, p) - phi_dot


def rear_speed_from_front(phi: float, phi_dot: float, v1: float, p: VehicleParams) -> float:
    """[ext] v2 as a function of (v1, phi, phi_dot):

        v2 = v1*cos(phi) + l1*sin(phi)*theta1_dot

    This is the *exact* (not just steady-state-ratio) front/rear speed
    coupling; it reduces to v2/v1 = r2/r1 (eq. 11) when phi_dot = 0.
    """
    th1d = front_yaw_rate(phi, phi_dot, v1, p)
    return v1 * np.cos(phi) + p.l1 * np.sin(phi) * th1d


def front_speed_from_rear(phi: float, phi_dot: float, v2: float, p: VehicleParams) -> float:
    """[ext] Inverse of `rear_speed_from_front`: given a DESIRED rear/implement
    speed v2 and articulation rate phi_dot, solve for the required front
    (tractor) command v1.

        v1 = [v2*(l1*cos(phi)+l2) - l1*l2*sin(phi)*phi_dot] / (l1 + l2*cos(phi))

    This is the building block for "control referenced at the rear/implement
    point": you specify the motion you want the tool/trailer to execute, and
    this function tells you what to command the tractor to do.
    """
    num = v2 * (p.l1 * np.cos(phi) + p.l2) - p.l1 * p.l2 * np.sin(phi) * phi_dot
    den = p.l1 + p.l2 * np.cos(phi)
    return num / den


def curvature_residual(theta_dot_measured: float, v: float, phi: float,
                        p: VehicleParams, ref: RefPoint = RefPoint.FRONT) -> float:
    """[ext] Delta_kappa = kappa_real - kappa_nom(phi), the central quantity of
    the MTT curvature-residual framework (report eq. 25), evaluated at the
    chosen reference point.

        kappa_real = theta_dot_measured / v
        kappa_nom  = kappa_nom_front(phi) or kappa_nom_rear(phi)

    Note: part of a nonzero residual can be purely kinematic (missing
    phi_dot term, see `front_yaw_rate` vs `kappa_nom_front`) rather than
    physical wheel/track slip -- see `front_yaw_rate` docstring.
    """
    if v == 0:
        raise ValueError("curvature_residual is undefined at v=0")
    kappa_real = theta_dot_measured / v
    if ref == RefPoint.REAR:
        kappa_nom = kappa_nom_rear(phi, p)
    else:
        kappa_nom = kappa_nom_front(phi, p)
    return kappa_real - kappa_nom


# --------------------------------------------------------------------------- #
# Frame conversions: pose of hitch / rear body from the front-based state
# --------------------------------------------------------------------------- #

def front_pose(state: np.ndarray, p: VehicleParams) -> Tuple[float, float, float]:
    """(x1, y1, theta1) -- identity, provided for symmetry with hitch_pose/rear_pose."""
    x1, y1, th1, _phi = state
    return x1, y1, th1


def hitch_pose(state: np.ndarray, p: VehicleParams) -> Tuple[float, float, float]:
    """(xH, yH, theta1). The hitch is a point, not a rigid body, so its
    'heading' is reported as theta1 (front body heading) by convention --
    use it only as a plotting/bookkeeping label, not a physical heading.
    """
    x1, y1, th1, _phi = state
    xh = x1 - p.l1 * np.cos(th1)
    yh = y1 - p.l1 * np.sin(th1)
    return xh, yh, th1


def rear_pose(state: np.ndarray, p: VehicleParams) -> Tuple[float, float, float]:
    """(x2, y2, theta2), theta2 = theta1 - phi."""
    x1, y1, th1, phi = state
    th2 = th1 - phi
    xh, yh, _ = hitch_pose(state, p)
    x2 = xh - p.l2 * np.cos(th2)
    y2 = yh - p.l2 * np.sin(th2)
    return x2, y2, th2


def pose_at(state: np.ndarray, p: VehicleParams, ref: RefPoint) -> Tuple[float, float, float]:
    """Dispatch to front_pose / hitch_pose / rear_pose."""
    return {
        RefPoint.FRONT: front_pose,
        RefPoint.HITCH: hitch_pose,
        RefPoint.REAR: rear_pose,
    }[ref](state, p)


# --------------------------------------------------------------------------- #
# State-space models: dstate/dt = f(state, u, params)
#   state = [x1, y1, theta1, phi]        (front-based, see module docstring)
#   u     = [v1, phi_dot]                (front speed command, articulation rate)
# --------------------------------------------------------------------------- #

def state_derivative_exact(state: np.ndarray, u: np.ndarray, p: VehicleParams) -> np.ndarray:
    """Full nonholonomic model: eq. (12) position kinematics + eq. (19) yaw rate."""
    x1, y1, th1, phi = state
    v1, phi_dot = u
    th1_dot = front_yaw_rate(phi, phi_dot, v1, p)
    return np.array([v1 * np.cos(th1), v1 * np.sin(th1), th1_dot, phi_dot])


def state_derivative_quasi_static(state: np.ndarray, u: np.ndarray, p: VehicleParams) -> np.ndarray:
    """Quasi-static / 'bicycle' model: eq. (11)-(12) of the MTT report,
    i.e. theta1_dot = v1*kappa_nom_front(phi), the phi_dot contribution to
    yaw rate is DROPPED (this is the model currently used as kappa_nom in
    the curvature-residual framework). phi still integrates at phi_dot.
    """
    x1, y1, th1, phi = state
    v1, phi_dot = u
    th1_dot = v1 * kappa_nom_front(phi, p)
    return np.array([v1 * np.cos(th1), v1 * np.sin(th1), th1_dot, phi_dot])


MODELS: dict[str, Callable] = {
    "exact": state_derivative_exact,
    "quasi_static": state_derivative_quasi_static,
}


# --------------------------------------------------------------------------- #
# Minimal simulator (RK4), no plotting
# --------------------------------------------------------------------------- #

def simulate(x0: np.ndarray, control_fn: Callable[[float, np.ndarray], np.ndarray],
             p: VehicleParams, dt: float, t_final: float,
             model: str = "exact") -> Tuple[np.ndarray, np.ndarray]:
    """Integrate the chosen model with RK4.

    Parameters
    ----------
    x0 : initial state [x1, y1, theta1, phi]
    control_fn : callable(t, state) -> u = [v1, phi_dot]
    p : VehicleParams
    dt, t_final : integration step and horizon (s)
    model : "exact" or "quasi_static"

    Returns
    -------
    t : (N,) time array
    X : (N, 4) state trajectory, columns = [x1, y1, theta1, phi]
    """
    f = MODELS[model]
    n_steps = int(np.round(t_final / dt)) + 1
    t = np.linspace(0, t_final, n_steps)
    X = np.zeros((n_steps, 4))
    X[0] = x0
    for k in range(n_steps - 1):
        tk, xk = t[k], X[k]
        u1 = control_fn(tk, xk)
        k1 = f(xk, u1, p)
        u2 = control_fn(tk + dt / 2, xk + dt / 2 * k1)
        k2 = f(xk + dt / 2 * k1, u2, p)
        u3 = control_fn(tk + dt / 2, xk + dt / 2 * k2)
        k3 = f(xk + dt / 2 * k2, u3, p)
        u4 = control_fn(tk + dt, xk + dt * k3)
        k4 = f(xk + dt * k3, u4, p)
        X[k + 1] = xk + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return t, X


# --------------------------------------------------------------------------- #
# Sanity checks (no plotting; run with `python articulated_kinematics.py`)
# --------------------------------------------------------------------------- #

if __name__ == "__main__":
    p_sym = VehicleParams(l1=1.2, l2=1.2)
    p_mtt = VehicleParams.mtt154()

    for phi_deg in (10, 25, 45):
        phi = phi_deg * DEG
        # symmetric vehicle -> r1 == r2
        assert np.isclose(turning_radius_front(phi, p_sym), turning_radius_rear(phi, p_sym))
        # asymmetric vehicle -> r1 != r2, ratio matches eq.(9) directly
        r1, r2 = turning_radius_front(phi, p_mtt), turning_radius_rear(phi, p_mtt)
        assert np.isclose(radius_ratio(phi, p_mtt), r2 / r1)
        # exact model reduces to quasi-static one when phi_dot = 0
        assert np.isclose(front_yaw_rate(phi, 0.0, 2.0, p_mtt), 2.0 * kappa_nom_front(phi, p_mtt))
        # small-angle approx agrees with exact kappa_nom at small phi
        assert abs(kappa_nom_front(2 * DEG, p_mtt) - kappa_nom_front_smallangle(2 * DEG, p_mtt)) < 1e-3
        # v1<->v2 inversion round-trips
        v1 = 1.7
        phi_dot = 5 * DEG
        v2 = rear_speed_from_front(phi, phi_dot, v1, p_mtt)
        v1_back = front_speed_from_rear(phi, phi_dot, v2, p_mtt)
        assert np.isclose(v1, v1_back)

    # quick simulate() smoke test: constant v1, sinusoidal phi command
    def ctrl(t, x):
        return np.array([1.5, 20 * DEG * np.cos(0.5 * t)])

    t, X = simulate(np.zeros(4), ctrl, p_mtt, dt=0.02, t_final=5.0, model="exact")
    assert X.shape == (t.size, 4)

    print("All sanity checks passed.")

# =========================================================================== #
# THE M-HIERARCHY (report Sec. "model hierarchy" + M5 theory doc, July 2026)
#
#   M0  kinematic (Corke & Ridley)          -- everything above this line
#   M1  articulation actuator dynamics      -- rate-limited servo on phi
#   M2  empirical residual (fitted gamma)   -- report: gamma=0.867 indoor
#   VL  virtual length model                -- fit l1_eff, l2_eff instead of
#       physical l1, l2; captures the slope deficit (gamma<1) via a purely
#       kinematic reparameterization but CANNOT produce a deadband or
#       hysteresis — the professor's "why not just change the lengths?" test.
#   M5  dissipative contact closed form     -- deadband-affine law, theory
#       doc eq.(27)/App.B: kappa = (L/(L^2+c2eff)) * dz(phi), plus the
#       kinetic play (hysteresis) operator of Prop. "stick-slip loop law".
#
# SIGN CONVENTION: this library keeps kappa_nom_front = +sin(phi)/(l1 cos+l2)
# (Corke's convention); all M-laws below use the SAME sign so their kappa(phi)
# curves overlay directly. (The theory doc writes the world-frame minus sign
# explicitly; here it is absorbed in the convention, as in Panel D of
# radius_mtt.py.)
# =========================================================================== #


def dz(x: float, thresh: float) -> float:
    """Symmetric dead-zone: sign(x) * max(|x| - thresh, 0)."""
    return np.sign(x) * np.maximum(np.abs(x) - thresh, 0.0)


@dataclass
class M2Params:
    """Empirical residual model (report): kappa_m2 = gamma * kappa_nom(phi)
    + a0 + a_v v + a_phi phi + a_phidot phi_dot.  Defaults: the indoor fit
    gamma = 0.867, regression terms zero (to be identified from bags)."""
    gamma: float = 0.867
    a0: float = 0.0
    a_v: float = 0.0
    a_phi: float = 0.0
    a_phidot: float = 0.0

    def kappa(self, phi: float, p: VehicleParams,
              v: float = 0.0, phi_dot: float = 0.0) -> float:
        return (self.gamma * kappa_nom_front(phi, p)
                + self.a0 + self.a_v * v + self.a_phi * phi
                + self.a_phidot * phi_dot)


# --------------------------------------------------------------------------- #
# Virtual Length Model (VL) — professor's question: "can we just change the
# effective lengths instead of adding dissipative contact physics?"
#
#   kappa_vl(phi) = sin(phi) / (l1_eff * cos(phi) + l2_eff)
#
# This is PURELY KINEMATIC: it uses the exact same Corke & Ridley formula as
# M0 but with l1_eff, l2_eff as free parameters. It CANNOT produce:
#   - a deadband (flat kappa≈0 region around phi=0)
#   - hysteresis (loop in reversal)
#   - any effect beyond a smooth nonlinear re-shaping of kappa(phi)
#
# If VL fits your data as well as M5, the dissipative-contact interpretation
# is over-constrained — the effect is just effective kinematic parameters.
# If M5 fits better (especially the deadband + hysteresis), the physics is
# real and the ICRA contribution stands.
# =========================================================================== #

@dataclass
class VirtualLengthParams:
    """Effective kinematic lengths, fitted from data instead of measured.
    The model is the exact M0 law evaluated at (l1_eff, l2_eff):
        kappa_vl(phi) = sin(phi) / (l1_eff * cos(phi) + l2_eff)
    """
    l1_eff: float
    l2_eff: float

    @classmethod
    def from_physical(cls, p: VehicleParams) -> "VirtualLengthParams":
        return cls(l1_eff=p.l1, l2_eff=p.l2)

    def kappa(self, phi: float) -> float:
        return np.sin(phi) / (self.l1_eff * np.cos(phi) + self.l2_eff)

    @property
    def L_eff(self) -> float:
        return self.l1_eff + self.l2_eff

    def gamma_slope(self, p: VehicleParams) -> float:
        """Effective small-angle slope gain relative to physical L = l1+l2.
        For small phi: kappa ≈ phi / L_eff = gamma_slope * (phi / L), so
        gamma_slope = L / L_eff.
        """
        return (p.l1 + p.l2) / self.L_eff


@dataclass
class M5ContactParams:
    """Closed-form M5 constants, derived from patch contact quantities
    (theory doc App. B).  Regime B: driven tractor patch near rolling,
    dragged trailer patch (skis / dragged track).

        gamma_coeff = L / (L^2 + c2_eff)            [1/m per rad]
        phi_dead_k  = mu1y N1 cbar1 mu2x / ((mu2y)^2 N2 L)   [rad]
        phi_dead_s  = (mu1_s / mu1_k) * phi_dead_k   (static breakaway)

    kappa_m5(phi) = gamma_coeff * dz(phi, phi_dead_k)   [memoryless law]
    """
    c2_eff: float          # trailer effective gyration^2 (m^2)
    phi_dead_k: float      # kinetic deadband (rad)
    mus_over_muk: float = 1.25

    @classmethod
    def from_contact(cls, p: VehicleParams,
                     mu1y: float = 0.65, N1: float = 300 * 9.81, L1: float = 1.0,
                     mu2x: float = 0.05, mu2y: float = 0.80, N2: float = 150 * 9.81,
                     L2: float = 0.8, mus_over_muk: float = 1.25) -> "M5ContactParams":
        L = p.l1 + p.l2
        cbar1 = L1 / 4.0
        c2_eff = (L2 / 4.0) ** 2
        phi_dead_k = mu1y * N1 * cbar1 * mu2x / ((mu2y ** 2) * N2 * L)
        return cls(c2_eff=c2_eff, phi_dead_k=phi_dead_k,
                   mus_over_muk=mus_over_muk)

    @classmethod
    def calibrated(cls, p: VehicleParams, gamma_slope: float = 0.867,
                   phi_dead_deg: float = 2.0,
                   mus_over_muk: float = 1.25) -> "M5ContactParams":
        """Constructor from MEASURED quantities instead of patch geometry:
        pick c2_eff to reproduce the identified slope gain (report: 0.867
        indoor) and set the deadband directly. Inverse of gamma_slope():
        c2_eff = L^2 (1/gamma_slope - 1)."""
        L = p.l1 + p.l2
        c2_eff = L ** 2 * (1.0 / gamma_slope - 1.0)
        return cls(c2_eff=c2_eff, phi_dead_k=phi_dead_deg * DEG,
                   mus_over_muk=mus_over_muk)

    def gamma_coeff(self, p: VehicleParams) -> float:
        L = p.l1 + p.l2
        return L / (L ** 2 + self.c2_eff)

    def gamma_slope(self, p: VehicleParams) -> float:
        """Slope gain relative to small-angle M0 (kappa_nom' = 1/L):
        gamma_slope = L^2/(L^2 + c2_eff) < 1."""
        L = p.l1 + p.l2
        return L ** 2 / (L ** 2 + self.c2_eff)

    def kappa(self, phi: float, p: VehicleParams) -> float:
        """Memoryless deadband-affine law (theory doc eq. 27, lib sign)."""
        return self.gamma_coeff(p) * dz(phi, self.phi_dead_k)

    def phi_from_kappa(self, kappa_des: float, p: VehicleParams) -> float:
        """Deadband INVERSE (Karnopp feedforward), clipped at phi_max."""
        if kappa_des == 0.0:
            return 0.0
        mag = abs(kappa_des) / self.gamma_coeff(p) + self.phi_dead_k
        return float(np.sign(kappa_des) * min(mag, p.phi_max))


class M5HysteresisPlant:
    """Kinetic play (backlash) operator of half-width phi_dead_k on top of
    the affine slope: the internal state w tracks phi with backlash 2r;
    kappa = gamma_coeff * w.  Produces threshold-from-rest, reversal
    branches offset by 2 phi_dead_k, and rate independence (state update is
    purely geometric in phi).  Static breakaway overshoot (mu_s > mu_k) is
    deliberately not included here -- see theory doc Prop. hysteresis."""

    def __init__(self, m5: M5ContactParams, p: VehicleParams):
        self.m5, self.p = m5, p
        self.w = 0.0

    def reset(self, w0: float = 0.0):
        self.w = w0

    def kappa_step(self, phi: float) -> float:
        r = self.m5.phi_dead_k
        if phi > self.w + r:
            self.w = phi - r
        elif phi < self.w - r:
            self.w = phi + r
        return self.m5.gamma_coeff(self.p) * self.w


# --- plant derivatives registered in MODELS -------------------------------- #

_M2_DEFAULT = M2Params()
_VL_DEFAULT = VirtualLengthParams.from_physical(VehicleParams.mtt154())


def state_derivative_virtual(state: np.ndarray, u: np.ndarray, p: VehicleParams) -> np.ndarray:
    """VL plant: theta1_dot = v * kappa_vl(phi). u=[v, phi_dot].
    Uses module-level VL_ACTIVE (set via set_vl_params)."""
    x1, y1, th1, phi = state
    v1, phi_dot = u
    th1_dot = v1 * VL_ACTIVE.kappa(phi)
    return np.array([v1 * np.cos(th1), v1 * np.sin(th1), th1_dot, phi_dot])


def state_derivative_m2(state: np.ndarray, u: np.ndarray, p: VehicleParams) -> np.ndarray:
    """M2 plant: theta1_dot = v * kappa_m2(phi, v, phi_dot). u=[v, phi_dot]."""
    x1, y1, th1, phi = state
    v1, phi_dot = u
    th1_dot = v1 * _M2_DEFAULT.kappa(phi, p, v1, phi_dot)
    return np.array([v1 * np.cos(th1), v1 * np.sin(th1), th1_dot, phi_dot])


def state_derivative_m5(state: np.ndarray, u: np.ndarray, p: VehicleParams) -> np.ndarray:
    """M5 memoryless plant: theta1_dot = v * kappa_m5(phi). u=[v, phi_dot].
    Uses module-level M5_ACTIVE (set your own via set_m5_params)."""
    x1, y1, th1, phi = state
    v1, phi_dot = u
    th1_dot = v1 * M5_ACTIVE.kappa(phi, p)
    return np.array([v1 * np.cos(th1), v1 * np.sin(th1), th1_dot, phi_dot])


def state_derivative_m1_servo(state: np.ndarray, u: np.ndarray, p: VehicleParams) -> np.ndarray:
    """M1 plant: articulation actuator dynamics. INPUT SEMANTICS DIFFER:
    u = [v, phi_cmd]; phi_dot = clip(K_SERVO*(phi_cmd - phi), +/-phi_dot_max);
    yaw from the exact M0 law with the realized phi_dot."""
    x1, y1, th1, phi = state
    v1, phi_cmd = u
    phi_dot = float(np.clip(K_SERVO * (phi_cmd - phi),
                            -p.phi_dot_max, p.phi_dot_max))
    th1_dot = front_yaw_rate(phi, phi_dot, v1, p)
    return np.array([v1 * np.cos(th1), v1 * np.sin(th1), th1_dot, phi_dot])


K_SERVO = 4.0                       # M1 servo gain (1/s)
M5_ACTIVE = M5ContactParams.from_contact(VehicleParams.mtt154())
VL_ACTIVE = VirtualLengthParams.from_physical(VehicleParams.mtt154())

def set_m5_params(m5: M5ContactParams) -> None:
    global M5_ACTIVE
    M5_ACTIVE = m5

def set_vl_params(vl: VirtualLengthParams) -> None:
    global VL_ACTIVE
    VL_ACTIVE = vl

MODELS["m2"] = state_derivative_m2
MODELS["m5"] = state_derivative_m5
MODELS["m1_servo"] = state_derivative_m1_servo
MODELS["virtual"] = state_derivative_virtual


if __name__ == "__main__":
    # --- M-hierarchy sanity checks (appended) -----------------------------
    p_mtt = VehicleParams.mtt154()
    m2 = M2Params()
    m5 = M5ContactParams.from_contact(p_mtt)
    vl = VirtualLengthParams.from_physical(p_mtt)

    # M2 reduces to M0 at gamma=1, zero regression
    assert np.isclose(M2Params(gamma=1.0).kappa(0.3, p_mtt),
                      kappa_nom_front(0.3, p_mtt))
    # M5: deadband positive, slope gain in (0,1), point-contact limit -> M0
    assert m5.phi_dead_k > 0 and 0 < m5.gamma_slope(p_mtt) < 1
    m5_pc = M5ContactParams(c2_eff=1e-9, phi_dead_k=1e-9)
    assert np.isclose(m5_pc.kappa(0.1, p_mtt), 0.1 / (p_mtt.l1 + p_mtt.l2),
                      rtol=1e-3)
    # deadband inverse round-trips outside the dead zone
    kd = 0.2
    assert np.isclose(m5.kappa(m5.phi_from_kappa(kd, p_mtt), p_mtt), kd)
    # hysteresis: loop width 2*phi_dead_k, rate independent by construction
    hp = M5HysteresisPlant(m5, p_mtt)
    phis = np.concatenate([np.linspace(0, 0.4, 200),
                           np.linspace(0.4, -0.4, 400),
                           np.linspace(-0.4, 0.4, 400)])
    ks = np.array([hp.kappa_step(ph) for ph in phis])
    # on the two sliding branches the phi-intercepts differ by 2 r:
    up = ks[-1] / m5.gamma_coeff(p_mtt)      # w at end of up-sweep
    assert np.isclose(0.4 - up, m5.phi_dead_k, atol=1e-6)

    # VL: physical params reproduce M0
    assert np.isclose(vl.kappa(0.3), kappa_nom_front(0.3, p_mtt))
    # VL: different effective length changes slope
    vl_long = VirtualLengthParams(l1_eff=p_mtt.l1*2, l2_eff=p_mtt.l2*2)
    assert abs(vl_long.kappa(0.3)) < abs(vl.kappa(0.3))  # longer -> less curvature
    # VL: gamma_slope = L / L_eff
    assert np.isclose(vl_long.gamma_slope(p_mtt), 0.5)
    # VL: registering works
    set_vl_params(vl_long)
    assert np.isclose(VL_ACTIVE.kappa(0.3), vl_long.kappa(0.3))

    print(f"[M-hierarchy] gamma_slope={m5.gamma_slope(p_mtt):.4f}, "
          f"phi_dead_k={m5.phi_dead_k/DEG:.2f} deg, "
          f"M2 gamma={m2.gamma} -- all hierarchy checks passed.")
