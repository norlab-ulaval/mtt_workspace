"""Pedagogical reference implementations of MTT-adapted motion-model baselines.

These functions are intentionally compact. They are not drop-in replacements for the
full models in the cited papers. Their purpose is to make the assumptions, inputs,
and parameters explicit so that each family can be implemented under one benchmark.

Conventions
-----------
phi > 0 follows the report convention; kappa may therefore be negative.
Curvature is defined as yaw_rate / longitudinal_speed and should not be evaluated
near zero speed.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Optional
import math
import numpy as np


@dataclass(frozen=True)
class Geometry:
    l1: float  # tractor reference to hitch [m]
    l2: float  # hitch to trailer reference [m]

    @property
    def L(self) -> float:
        return self.l1 + self.l2


@dataclass(frozen=True)
class CalibratedKinematics:
    gamma: float = 1.0
    phi_scale: float = 1.0
    phi_bias: float = 0.0
    l1_offset: float = 0.0
    l2_offset: float = 0.0


@dataclass(frozen=True)
class ActuatorParams:
    delay_s: float = 0.0
    tau_s: float = 0.15
    deadband_rad: float = 0.0
    bias_rad: float = 0.0
    rate_limit_rad_s: float = math.inf


@dataclass(frozen=True)
class ICRParams:
    delta_front: float = 0.0
    delta_rear: float = 0.0


@dataclass(frozen=True)
class SlipAngleParams:
    beta_front: float = 0.0
    beta_rear: float = 0.0


@dataclass(frozen=True)
class M5RParams:
    gamma: float
    deadband_rad: float
    bias_rad: float = 0.0
    delay_s: float = 0.0
    tau_s: float = 0.0


def deadzone(x: np.ndarray | float, radius: float):
    x_arr = np.asarray(x)
    y = np.sign(x_arr) * np.maximum(np.abs(x_arr) - radius, 0.0)
    return float(y) if np.ndim(x_arr) == 0 else y


def kappa_ctrv(yaw_rate0: float, speed0: float, speed_gate: float = 0.05) -> float:
    """Constant-turn-rate-and-velocity baseline."""
    if abs(speed0) < speed_gate:
        return 0.0
    return yaw_rate0 / speed0


def kappa_kin_exact(phi: np.ndarray | float, geom: Geometry):
    """Exact center-articulated point-contact curvature."""
    phi = np.asarray(phi)
    den = geom.l1 * np.cos(phi) + geom.l2
    out = -np.sin(phi) / den
    return float(out) if out.ndim == 0 else out


def kappa_kin_small_angle(phi: np.ndarray | float, geom: Geometry):
    phi = np.asarray(phi)
    out = -phi / geom.L
    return float(out) if out.ndim == 0 else out


def kappa_kin_calibrated(phi: np.ndarray | float, geom: Geometry,
                         p: CalibratedKinematics):
    phi_eff = p.phi_scale * np.asarray(phi) + p.phi_bias
    ge = Geometry(geom.l1 + p.l1_offset, geom.l2 + p.l2_offset)
    out = p.gamma * kappa_kin_exact(phi_eff, ge)
    return float(out) if np.ndim(out) == 0 else out


def kappa_icr_virtual(phi: np.ndarray | float, geom: Geometry, p: ICRParams):
    """Virtual-ICR adaptation: shift the effective lateral-constraint points."""
    ge = Geometry(geom.l1 + p.delta_front, geom.l2 + p.delta_rear)
    return kappa_kin_exact(phi, ge)


def kappa_slip_angle(phi: np.ndarray | float, geom: Geometry,
                     p: SlipAngleParams):
    """Pedagogical relaxed-Pfaff/slip-angle relation.

    The effective articulation is corrected by front/rear patch slip angles. A full
    single-track dynamics model computes these angles from force balance rather than
    treating them as parameters.
    """
    phi_eff = np.asarray(phi) - p.beta_front + p.beta_rear
    return kappa_kin_exact(phi_eff, geom)


def simulate_actuator(phi_cmd: Iterable[float], dt: float, p: ActuatorParams) -> np.ndarray:
    """Delay + dead-zone + first-order lag + rate saturation."""
    u = np.asarray(list(phi_cmd), dtype=float)
    delay_n = max(0, int(round(p.delay_s / dt)))
    ud = np.empty_like(u)
    if delay_n == 0:
        ud[:] = u
    else:
        ud[:delay_n] = u[0]
        ud[delay_n:] = u[:-delay_n]
    target = deadzone(ud - p.bias_rad, p.deadband_rad)
    y = np.empty_like(target)
    y[0] = target[0]
    alpha = 1.0 if p.tau_s <= 0 else min(1.0, dt / p.tau_s)
    max_step = p.rate_limit_rad_s * dt
    for k in range(1, len(y)):
        raw = y[k-1] + alpha * (target[k] - y[k-1])
        y[k] = y[k-1] + np.clip(raw-y[k-1], -max_step, max_step)
    return y


def kappa_m5_reduced(phi_eff: np.ndarray | float, geom: Geometry, p: M5RParams):
    """Reduced dissipative curvature law."""
    x = np.asarray(phi_eff) - p.bias_rad
    out = -(p.gamma / geom.L) * deadzone(x, p.deadband_rad)
    return float(out) if np.ndim(out) == 0 else out


def integrate_planar(x0: np.ndarray, speed: np.ndarray, curvature: np.ndarray,
                     dt: float) -> np.ndarray:
    """Integrate [x, y, yaw] using midpoint yaw."""
    speed = np.asarray(speed); curvature = np.asarray(curvature)
    if speed.shape != curvature.shape:
        raise ValueError("speed and curvature must have the same shape")
    X = np.empty((len(speed)+1, 3), dtype=float); X[0] = x0
    for k, (v, kap) in enumerate(zip(speed, curvature)):
        omega = v * kap
        th_mid = X[k,2] + 0.5 * omega * dt
        X[k+1,0] = X[k,0] + v * math.cos(th_mid) * dt
        X[k+1,1] = X[k,1] + v * math.sin(th_mid) * dt
        X[k+1,2] = X[k,2] + omega * dt
    return X


def quadratic_limit_surface_twist(wrench: np.ndarray, A: np.ndarray) -> np.ndarray:
    """Normal direction of ellipsoidal limit surface H(w)=w^T A w.

    Up to a positive scale, the generalized velocity is grad H = 2 A w.
    """
    wrench = np.asarray(wrench, dtype=float)
    A = np.asarray(A, dtype=float)
    return 2.0 * A @ wrench


def power_dissipation_quadratic(xi: np.ndarray, Q: np.ndarray,
                                affine: Optional[np.ndarray] = None) -> float:
    """Smooth ellipsoidal proxy D=sqrt((B xi+b)^T Q (B xi+b))."""
    xi = np.asarray(xi, dtype=float)
    z = xi if affine is None else xi + np.asarray(affine, dtype=float)
    return float(math.sqrt(max(0.0, z @ Q @ z)))


def solve_m5_full_numeric(objective: Callable[[np.ndarray], float],
                          x0=(1.0, 0.0, 0.0), iterations: int = 200,
                          step: float = 0.05) -> np.ndarray:
    """Dependency-free finite-difference gradient descent for course examples.

    Use scipy.optimize/cvxpy in serious experiments, with constraints and analytic
    gradients. Convexity does not by itself guarantee uniqueness if the objective has
    flat directions.
    """
    x = np.asarray(x0, dtype=float)
    eps = 1e-6
    for _ in range(iterations):
        g = np.zeros(3)
        f0 = objective(x)
        for j in range(3):
            xp = x.copy(); xp[j] += eps
            g[j] = (objective(xp)-f0)/eps
        x -= step * g / max(1.0, np.linalg.norm(g))
    return x


def polynomial_features(phi: np.ndarray, speed: np.ndarray, phi_dot: np.ndarray) -> np.ndarray:
    """Small interpretable feature map for a ridge residual baseline."""
    phi=np.asarray(phi); speed=np.asarray(speed); phi_dot=np.asarray(phi_dot)
    return np.column_stack([
        np.ones_like(phi), phi, speed, phi_dot, phi**2, phi*speed,
        np.abs(phi), np.sign(phi)*phi**2
    ])


def ridge_fit(Phi: np.ndarray, y: np.ndarray, lam: float = 1e-3) -> np.ndarray:
    Phi=np.asarray(Phi); y=np.asarray(y)
    return np.linalg.solve(Phi.T@Phi + lam*np.eye(Phi.shape[1]), Phi.T@y)


def ridge_predict(Phi: np.ndarray, theta: np.ndarray) -> np.ndarray:
    return np.asarray(Phi) @ np.asarray(theta)


if __name__ == "__main__":
    g = Geometry(1.05, 1.15)
    phi = np.deg2rad(np.array([-20, -10, 0, 10, 20]))
    print("KIN", kappa_kin_exact(phi, g))
    print("ICR", kappa_icr_virtual(phi, g, ICRParams(.15,.20)))
    print("M5-R", kappa_m5_reduced(phi, g, M5RParams(.78, math.radians(4))))
