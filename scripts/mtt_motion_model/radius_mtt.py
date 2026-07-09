import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams.update({
    "font.size": 10.5,
    "axes.titlesize": 11.5,
    "axes.titleweight": "bold",
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "axes.grid": True,
    "grid.alpha": 0.25,
})

import os
_PREF = "/home/mohamed/Documents/Project_MTT/Workspace/mtt_workspace/scripts"
DIR = _PREF if os.path.isdir(_PREF) else "."   # fallback to cwd if absent
deg = np.pi / 180.0
phi = np.linspace(-60, 60, 1201) * deg  # articulation angle, MTT hard-stop range

# ---- Vehicle geometries -----------------------------------------------
# Corke & Ridley notation: l1 = front half-length (H->P1), l2 = rear half-length (H->P2)
# MTT-154 measured values (internship report, Table 3): Lf = 0.9 m, Lr = 1.5 m
cases = {
    "Symmetric baseline ($l_1=l_2=1.2$ m)": dict(l1=1.2, l2=1.2, color="#4C72B0", ls="-"),
    "MTT-154 real geometry ($L_f=0.9$, $L_r=1.5$ m)": dict(l1=0.9, l2=1.5, color="#DD8452", ls="-"),
    "Extreme asymmetry ($l_1=0.5$, $l_2=1.9$ m)": dict(l1=0.5, l2=1.9, color="#55A868", ls="--"),
}

def turning_radii(phi, l1, l2):
    s = np.sin(phi)
    r1 = (l1 * np.cos(phi) + l2) / s
    r2 = (l2 * np.cos(phi) + l1) / s
    return r1, r2

def kappa_nom(phi, l1, l2):
    # quasi-static ("bicycle") model, Corke eq.(20) == MTT report eq.(13)
    return np.sin(phi) / (l1 * np.cos(phi) + l2)

def theta_dot_exact(phi, phi_dot, v, l1, l2):
    # full nonholonomic model, Corke eq.(19), extended with articulation rate
    return (v * np.sin(phi) + l2 * phi_dot) / (l1 * np.cos(phi) + l2)

fig, axes = plt.subplots(2, 2, figsize=(12.5, 9.5))
ax1, ax2, ax3, ax4 = axes.flatten()
phid = phi / deg
mask = np.abs(phid) > 1.0  # avoid the 1/sin(phi) singularity at phi=0

# --- Panel A : symmetric baseline, r1 = r2 ---------------------------------
l1, l2 = cases["Symmetric baseline ($l_1=l_2=1.2$ m)"]["l1"], cases["Symmetric baseline ($l_1=l_2=1.2$ m)"]["l2"]
r1, r2 = turning_radii(phi, l1, l2)
ax1.plot(phid[mask], r1[mask], color="#4C72B0", lw=2.5, label=r"$r_1$ (front)")
ax1.plot(phid[mask], r2[mask], color="#C44E52", lw=1.2, ls=":", label=r"$r_2$ (rear)")
ax1.set_title(r"A. Baseline case: $l_1=l_2$  $\Rightarrow$  $r_1=r_2$ (eq. 9)")
ax1.set_xlabel(r"Articulation angle $\phi$ (deg)")
ax1.set_ylabel("Turning radius (m)")
ax1.set_ylim(-15, 15)
ax1.axhline(0, color="k", lw=0.5)
ax1.axvline(0, color="k", lw=0.5)
ax1.legend(loc="upper right", fontsize=9)

# --- Panel B : asymmetric cases -------------------------------------------
for name, c in cases.items():
    if c["l1"] == c["l2"]:
        continue
    r1, r2 = turning_radii(phi, c["l1"], c["l2"])
    ax2.plot(phid[mask], r1[mask], color=c["color"], lw=2.2, ls=c["ls"], label=f"{name} — $r_1$")
    ax2.plot(phid[mask], r2[mask], color=c["color"], lw=1.0, ls="--", alpha=0.6)
ax2.set_title(r"B. Asymmetric cases: $l_1 \neq l_2$  $\Rightarrow$  $r_1 \neq r_2$")
ax2.set_xlabel(r"Articulation angle $\phi$ (deg)")
ax2.set_ylabel("Turning radius (m)")
ax2.set_ylim(-15, 15)
ax2.axhline(0, color="k", lw=0.5)
ax2.axvline(0, color="k", lw=0.5)
ax2.legend(loc="upper right", fontsize=7.5)

# --- Panel C : ratio r2/r1, all cases --------------------------------------
for name, c in cases.items():
    r1, r2 = turning_radii(phi, c["l1"], c["l2"])
    ratio = r2 / r1
    ax3.plot(phid[mask], ratio[mask], color=c["color"], lw=2.2, ls=c["ls"], label=name)
ax3.axhline(1.0, color="k", lw=0.8, alpha=0.5)
ax3.set_title(r"C. Radius ratio $r_2/r_1$ (eq. 9) — differential wheel/track wear")
ax3.set_xlabel(r"Articulation angle $\phi$ (deg)")
ax3.set_ylabel(r"$r_2/r_1$")
ax3.legend(loc="upper right", fontsize=7.5)

# --- Panel D : the MODEL LADDER, kappa(phi), M0 vs M5 (deadband-affine) -----
# Imports the closed-form M5 law from the shared library so this static
# figure shows exactly what the dynamic sims use. This is the "gap" figure:
# M0 (Corke) is the c->0 limit; M5 adds a deadband (gray band) and a slope
# deficit (gamma_slope). Overlaid: the measured indoor gain gamma=0.867.
from motion_model import VehicleParams, M5ContactParams, kappa_nom_front, DEG as _DEG
_p = VehicleParams.mtt154()
_m5 = M5ContactParams.calibrated(_p, gamma_slope=0.867, phi_dead_deg=2.0)
_phi = np.linspace(-30, 30, 601) * deg
k_m0 = np.array([kappa_nom_front(x, _p) for x in _phi])
k_m5 = np.array([_m5.kappa(x, _p) for x in _phi])
k_m2 = 0.867 * k_m0
ax4.plot(_phi / deg, k_m0, color="#C44E52", lw=2.2, label=r"M0 exact (Corke eq.13)")
ax4.plot(_phi / deg, k_m2, color="#DD8452", lw=1.8, ls="--",
         label=r"M2 fitted gain $\gamma=0.867$")
ax4.plot(_phi / deg, k_m5, color="#4C72B0", lw=2.6, label=r"M5 deadband-affine (eq.27)")
ax4.axvspan(-_m5.phi_dead_k / deg, _m5.phi_dead_k / deg, color="gray", alpha=0.15,
            label=r"M5 deadband $\pm\varphi_{dead}$")
ax4.axhline(0, color="k", lw=0.5); ax4.axvline(0, color="k", lw=0.5)
ax4.set_title(r"D. Model ladder $\kappa(\phi)$: M0 vs M2 vs M5 — the ICRA gap figure")
ax4.set_xlabel(r"Articulation angle $\phi$ (deg)")
ax4.set_ylabel(r"Curvature $\kappa$ (1/m)")
ax4.legend(loc="upper left", fontsize=7.5)

fig.suptitle("Steering kinematics of a center-articulated vehicle — theory (Corke & Ridley, 2001) applied to the MTT-154",
             fontsize=13, fontweight="bold", y=0.995)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(f"{DIR}/static_analysis.png", dpi=160)
print("saved static_analysis.png")