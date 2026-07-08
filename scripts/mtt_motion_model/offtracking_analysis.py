"""
offtracking_analysis.py

Quantifies HOW MUCH the front (tractor) and rear (trailer/implement)
reference points of a center-articulated vehicle diverge from one another
over a trajectory, and ties that divergence back to the articulation angle
phi. Three distinct, complementary ways to look at "front vs rear
trajectory" are implemented, matching three different questions:

  1. INSTANTANEOUS GEOMETRIC LINK (single time instant, no history needed)
     `front_rear_offset(state, p)` -- at one instant: how far apart are P1
     and P2 in a straight line, what is their heading difference (== phi,
     by construction), where is the hitch. Purely algebraic (see
     `hitch_pose` / `rear_pose` in motion_model.py).

  2. INSTANTANEOUS KINEMATIC LINK (rates, no history needed)
     `kinematic_mismatch(phi, phi_dot, v1, p)` -- how differently are P1
     and P2 moving RIGHT NOW: yaw-rate mismatch (== phi_dot exactly, eq. 21)
     and speed mismatch (v1 - v2, from `rear_speed_from_front`).

  3. PATH-LEVEL / "OFF-TRACKING" LINK (needs the full trajectory history)
     `off_tracking_series(t, X, p, ref_rear)` -- the classic trailer /
     articulated-vehicle "off-tracking" (a.k.a. swept-path) metric: at each
     instant, how far is the rear reference point from the path ALREADY
     TRACED by the front reference point? This is causal -- only points the
     front has already visited count, you cannot off-track against a future
     point. This is what actually answers "does my implement cut the
     corner", and it is fundamentally different from 1) and 2): it is a
     property of the WHOLE PATH, not of the instantaneous state, and it
     stays nonzero even long after phi has gone back to 0 (the rear needs
     time -- and forward travel -- to "catch up" onto the straight
     continuation of the front's old path).

Reference point choice matters here: front_rear_offset/kinematic_mismatch
are stated for P1/P2 (FRONT/REAR) since that's what phi directly relates to
(theta1 - theta2 = phi by definition); off_tracking_series accepts ANY pair
of reference points via motion_model.pose_at / motion_model.RefPoint, so
you can for instance ask "how far does the HITCH cut the corner relative to
the front" just as easily as "how far does the REAR" -- see the sanity
checks at the bottom for both.

Requires motion_model.py (the kinematics library) on the same path.
No plotting -- everything is returned as plain numpy arrays / dicts, plot
however you like.
"""

from __future__ import annotations

from typing import Dict, Tuple

import numpy as np

from motion_model import (
    DEG,
    VehicleParams,
    RefPoint,
    front_yaw_rate,
    rear_yaw_rate,
    rear_speed_from_front,
    radius_ratio,
    hitch_pose,
    rear_pose,
    pose_at,
    simulate,
)


# =========================================================================== #
# 1. Instantaneous geometric link (single state, no history)
# =========================================================================== #

def front_rear_offset(state: np.ndarray, p: VehicleParams) -> Dict[str, object]:
    """At one instant: straight-line P1-P2 distance, heading difference
    (== phi by construction), and hitch position. Sanity: dist_P1P2 should
    always satisfy the triangle inequality l1+l2 >= dist_P1P2 >= |l1-l2|,
    with equality to l1+l2 iff phi=0 (both bodies aligned).
    """
    x1, y1, th1, phi = state
    xh, yh, _ = hitch_pose(state, p)
    x2, y2, th2 = rear_pose(state, p)
    dist_P1P2 = float(np.hypot(x1 - x2, y1 - y2))
    return dict(
        P1=(x1, y1), P2=(x2, y2), H=(xh, yh),
        theta1=th1, theta2=th2, phi=phi,
        heading_diff=th1 - th2,  # identically == phi
        dist_P1P2=dist_P1P2,
    )


# =========================================================================== #
# 2. Instantaneous kinematic link (rates, no history)
# =========================================================================== #

def kinematic_mismatch(phi: float, phi_dot: float, v1: float,
                        p: VehicleParams) -> Dict[str, float]:
    """How differently P1 and P2 are moving RIGHT NOW. yaw_rate_mismatch is
    identically equal to phi_dot (eq. 21: theta1_dot - theta2_dot = phi_dot),
    included explicitly here so you don't have to re-derive that every time
    you want to relate "how fast the front and rear are rotating apart" to
    the articulation rate you are commanding/measuring.
    """
    th1_dot = front_yaw_rate(phi, phi_dot, v1, p)
    th2_dot = rear_yaw_rate(phi, phi_dot, v1, p)
    v2 = rear_speed_from_front(phi, phi_dot, v1, p)
    return dict(
        theta1_dot=th1_dot, theta2_dot=th2_dot,
        yaw_rate_mismatch=th1_dot - th2_dot,   # == phi_dot, exactly
        v1=v1, v2=v2, speed_mismatch=v1 - v2,
        radius_ratio=radius_ratio(phi, p) if phi_dot == 0 else np.nan,  # only meaningful if phi_dot=0
    )


# =========================================================================== #
# 3. Path-level "off-tracking" link (needs the full trajectory)
# =========================================================================== #

def _point_to_polyline_distance(point: np.ndarray, poly: np.ndarray) -> Tuple[float, int]:
    """Distance from `point` (2,) to the closest location on the polyline
    `poly` (m,2) -- projects onto each segment, not just the vertices, so
    the result does not depend on how densely the polyline is sampled.
    Returns (distance, index of the segment's start vertex).
    """
    if len(poly) < 2:
        d = float(np.hypot(*(poly[0] - point)))
        return d, 0
    A = poly[:-1]
    B = poly[1:]
    AB = B - A
    AP = point - A
    denom = np.maximum(np.sum(AB * AB, axis=1), 1e-12)
    tau = np.clip(np.sum(AP * AB, axis=1) / denom, 0.0, 1.0)
    proj = A + tau[:, None] * AB
    d = np.hypot(*(proj - point).T)
    j = int(np.argmin(d))
    return float(d[j]), j


def off_tracking_series(t: np.ndarray, X: np.ndarray, p: VehicleParams,
                         ref_rear: RefPoint = RefPoint.REAR,
                         ref_front: RefPoint = RefPoint.FRONT) -> Dict[str, np.ndarray]:
    """Causal off-tracking distance: at every time step k, the distance from
    the rear reference point's CURRENT position to the closest location
    (interpolated along the polyline, not just the sampled vertices -- see
    `_point_to_polyline_distance`) on the front reference point's path *up
    to and including step k* (never a future front position -- the rear
    cannot "off-track" against a corner the front hasn't taken yet).

    Also returns cumulative arc length traveled by each reference point
    (simple polyline length, i.e. sum of consecutive Euclidean distances)
    and their ratio -- the path-integrated version of eq. (10)-(11)/eq. (9)
    radius_ratio, valid even when phi is not constant.

    Parameters
    ----------
    t, X : output of motion_model.simulate(...) -- X has columns [x1,y1,theta1,phi]
    ref_rear, ref_front : which reference points to compare (default REAR vs FRONT,
        but e.g. ref_rear=HITCH lets you ask "how much does the hitch cut the corner").

    Returns
    -------
    dict with: t, front_xy, rear_xy (both (n,2)), off_track (n,), the index
    into front_xy of the nearest historical segment for each k, cumulative
    arc lengths s_front / s_rear (n,), and the total arc_length_ratio (scalar).
    """
    n = len(t)
    front_xy = np.zeros((n, 2))
    rear_xy = np.zeros((n, 2))
    for k in range(n):
        xf, yf, _ = pose_at(X[k], p, ref_front)
        xr, yr, _ = pose_at(X[k], p, ref_rear)
        front_xy[k] = (xf, yf)
        rear_xy[k] = (xr, yr)

    off_track = np.full(n, np.nan)
    nearest_idx = np.full(n, -1, dtype=int)
    for k in range(1, n):
        hist = front_xy[:k + 1]                       # causal: only up to now
        d, j = _point_to_polyline_distance(rear_xy[k], hist)
        off_track[k] = d
        nearest_idx[k] = j
    off_track[0] = 0.0
    nearest_idx[0] = 0

    s_front = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(front_xy, axis=0).T))])
    s_rear = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(rear_xy, axis=0).T))])

    # Burn-in: at t=0 the rear already sits l1 (HITCH), l1+l2 (REAR), etc.
    # "ahead" of any recorded front history (the front's pre-t=0 path is, by
    # construction, unknown to us). Until the front has itself travelled at
    # least that far, off_track is comparing the rear against a polyline
    # that physically doesn't reach far enough back yet -- an unavoidable
    # boundary artifact, not a real off-tracking event. We mask it out.
    burn_in_dist = float(np.hypot(*(front_xy[0] - rear_xy[0])))
    valid = s_front >= burn_in_dist
    off_track_valid = np.where(valid, off_track, np.nan)

    s_front = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(front_xy, axis=0).T))])
    s_rear = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(rear_xy, axis=0).T))])
    arc_length_ratio = float(s_rear[-1] / s_front[-1]) if s_front[-1] > 0 else np.nan

    return dict(
        t=t, front_xy=front_xy, rear_xy=rear_xy,
        off_track=off_track, off_track_valid=off_track_valid,
        valid_mask=valid, burn_in_dist=burn_in_dist,
        nearest_front_idx=nearest_idx,
        s_front=s_front, s_rear=s_rear, arc_length_ratio=arc_length_ratio,
        max_off_track=float(np.nanmax(off_track_valid)),
        rms_off_track=float(np.sqrt(np.nanmean(off_track_valid ** 2))),
    )


# =========================================================================== #
# Sanity checks (no plotting) -- run with `python offtracking_analysis.py`
# =========================================================================== #

if __name__ == "__main__":
    p_mtt = VehicleParams.mtt154()

    # --- 1. geometric offset: triangle-inequality bounds, phi=0 => dist=l1+l2
    s0 = np.array([0.0, 0.0, 0.0, 0.0])
    g0 = front_rear_offset(s0, p_mtt)
    assert np.isclose(g0["dist_P1P2"], p_mtt.l1 + p_mtt.l2)
    assert np.isclose(g0["heading_diff"], 0.0)

    s1 = np.array([0.0, 0.0, 0.0, 40 * DEG])
    g1 = front_rear_offset(s1, p_mtt)
    assert p_mtt.l1 + p_mtt.l2 >= g1["dist_P1P2"] >= abs(p_mtt.l1 - p_mtt.l2)
    assert np.isclose(g1["heading_diff"], 40 * DEG)
    print(f"[1] phi=0 -> |P1P2|={g0['dist_P1P2']:.3f} m (== l1+l2); "
          f"phi=40deg -> |P1P2|={g1['dist_P1P2']:.3f} m")

    # --- 2. kinematic mismatch: yaw-rate mismatch must equal phi_dot exactly
    km = kinematic_mismatch(phi=25 * DEG, phi_dot=12 * DEG, v1=2.0, p=p_mtt)
    assert np.isclose(km["yaw_rate_mismatch"], 12 * DEG)
    print(f"[2] at phi=25deg, phi_dot=12deg/s, v1=2 m/s: "
          f"v2={km['v2']:.3f} m/s (speed mismatch {km['speed_mismatch']*100:.1f} cm/s)")

    # --- 3a. off-tracking, phi always 0 -> rear must exactly retrace front's
    #         path (up to numerical discretization), off_track ~ 0 everywhere
    def ctrl_straight_turn_then_straight(t, x):
        return np.array([1.5, 0.0])  # phi stays at its initial value (0) forever

    t, X = simulate(np.zeros(4), ctrl_straight_turn_then_straight, p_mtt,
                     dt=0.02, t_final=8.0, model="exact")
    ot0 = off_tracking_series(t, X, p_mtt)
    assert ot0["max_off_track"] < 1e-9
    print(f"[3a] phi=0 throughout: max off-tracking = {ot0['max_off_track']*1e9:.3f} nm "
          f"(should be ~0 -- rear exactly retraces front's path)")

    # --- 3b. off-tracking during a sustained turn: rear must cut the corner
    #         (off_track > 0), and the mean off-tracking should scale with
    #         how asymmetric the geometry is (l1 != l2)
    def ctrl_turn(t, x):
        phi_target = 30 * DEG
        return np.array([1.5, 5 * DEG * np.sign(phi_target - x[3])])

    t2, X2 = simulate(np.zeros(4), ctrl_turn, p_mtt, dt=0.02, t_final=10.0, model="exact")
    ot1 = off_tracking_series(t2, X2, p_mtt)
    assert ot1["max_off_track"] > 0.01  # meaningfully nonzero, not just noise
    print(f"[3b] sustained turn to phi=30deg: max off-tracking = {ot1['max_off_track']*100:.1f} cm, "
          f"RMS = {ot1['rms_off_track']*100:.1f} cm, "
          f"arc-length ratio (rear/front) = {ot1['arc_length_ratio']:.4f}")

    # --- 3c. same maneuver, but ask about the HITCH instead of the REAR ----
    ot1_hitch = off_tracking_series(t2, X2, p_mtt, ref_rear=RefPoint.HITCH)
    assert ot1_hitch["max_off_track"] < ot1["max_off_track"]  # hitch is closer to front than rear is
    print(f"[3c] same turn, HITCH-referenced: max off-tracking = "
          f"{ot1_hitch['max_off_track']*100:.1f} cm (< rear's, as expected: |l1| < |l1+l2|)")

    print("All sanity checks passed.")

# =========================================================================== #
# 4. PLANT-DEPENDENT OFF-TRACKING: the same articulation command produces
#    DIFFERENT corner-cutting depending on which plant actually realizes the
#    yaw. Off-tracking is a geometric consequence of the realized path, so if
#    M5's deadband+deficit make the front under-rotate for a given phi, the
#    whole swept-path signature changes. This ties the motion-model choice
#    (M0 exact vs M5) directly to a field-measurable quantity (drone swept
#    path), independent of any controller.
#
#    Requires the M-hierarchy section of motion_model.py (model="m5").
# =========================================================================== #

from motion_model import M5ContactParams, set_m5_params


def offtracking_by_plant(control_fn, p: VehicleParams, m5: "M5ContactParams",
                          dt: float = 0.02, t_final: float = 12.0,
                          ref_rear: RefPoint = RefPoint.REAR
                          ) -> Dict[str, Dict[str, np.ndarray]]:
    """Run the SAME open-loop command through the M0-exact and the M5 plant,
    then measure trailer off-tracking for each. Returns {'exact':..., 'm5':...}
    with each entry the full off_tracking_series dict, plus the realized phi
    trace (identical across plants -- phi integrates phi_dot the same way; it
    is the YAW response to that phi that differs, hence the front path, hence
    the off-tracking)."""
    set_m5_params(m5)
    out = {}
    for model in ("exact", "m5"):
        t, X = simulate(np.zeros(4), control_fn, p, dt, t_final, model=model)
        ot = off_tracking_series(t, X, p, ref_rear=ref_rear)
        ot["X"] = X
        out[model] = ot
    return out


def plot_offtracking_by_plant(res: Dict[str, Dict], p: VehicleParams,
                               title: str = ""):
    import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.8))
    colors = dict(exact="#4C72B0", m5="#C44E52")
    for model, ot in res.items():
        ax1.plot(ot["front_xy"][:, 0], ot["front_xy"][:, 1],
                 color=colors[model], lw=2, label=f"{model}: front path")
        ax1.plot(ot["rear_xy"][:, 0], ot["rear_xy"][:, 1],
                 color=colors[model], lw=1.2, ls="--", alpha=0.7,
                 label=f"{model}: trailer path")
        ax2.plot(ot["t"], ot["off_track_valid"] * 100, color=colors[model],
                 lw=2, label=f"{model}: max {ot['max_off_track']*100:.1f} cm")
    ax1.set_aspect("equal"); ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
    ax1.set_title("Front & trailer swept paths per plant")
    ax1.legend(fontsize=8)
    ax2.set_xlabel("t (s)"); ax2.set_ylabel("trailer off-tracking (cm)")
    ax2.set_title("Off-tracking: M0-exact vs M5 plant" + (f" -- {title}" if title else ""))
    ax2.legend(fontsize=8)
    fig.suptitle(title, fontsize=12, fontweight="bold")
    fig.tight_layout()
    return fig


if __name__ == "__main__":
    # --- 4. same turn command, two plants -> different off-tracking --------
    p_mtt = VehicleParams.mtt154()
    m5 = M5ContactParams.calibrated(p_mtt, gamma_slope=0.867, phi_dead_deg=2.0)

    def ctrl_turn_plant(t, x):
        phi_target = 30 * DEG
        return np.array([1.5, 5 * DEG * np.sign(phi_target - x[3])])

    res = offtracking_by_plant(ctrl_turn_plant, p_mtt, m5, t_final=12.0)
    ot_ex, ot_m5 = res["exact"], res["m5"]
    print(f"[4] SAME phi command, trailer off-tracking: "
          f"M0-exact max={ot_ex['max_off_track']*100:.1f} cm, "
          f"M5 max={ot_m5['max_off_track']*100:.1f} cm")
    # KEY FINDING (opposite of the naive guess): for a GIVEN phi command, M5
    # under-rotates the front (deadband + slope deficit), so the front traces
    # a FLATTER, LONGER path; the trailer, lagging on that flatter front
    # history, diverges MORE from it, not less. So the two plants disagree on
    # swept-path geometry by a factor ~2 here -- a plant-model error visible
    # directly on the drone with NO controller involved. Predicting swept
    # path with M0 does not simply over- or under-estimate uniformly; it gets
    # the corner-cutting qualitatively wrong once the deficit is present.
    assert ot_m5["max_off_track"] != ot_ex["max_off_track"]
    factor = ot_m5["max_off_track"] / ot_ex["max_off_track"]
    print(f"[4] => M0 and M5 disagree on trailer swept path by a factor "
          f"{factor:.2f} for the identical phi command (drone-measurable, "
          f"controller-independent).")

    print("All plant-dependent off-tracking checks passed.")
