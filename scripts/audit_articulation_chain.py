#!/usr/bin/env python3
"""
audit_articulation_chain.py

Offline audit of the articulation-angle sensing/command chain from a bag:
command -> servo -> hardware encoder -> LiDAR/KF fused estimate.

For each topic: message count, frequency (Hz), gaps/dropouts (availability).
For hardware vs trailer (LiDAR/KF): time-aligned error stats (mean/std/max),
correlation, and Mahalanobis-style agreement using the STM confidence signal.
For command/setpoint vs measured/hardware: response-lag estimate (cross-
correlation) — is the servo tracking commands with a consistent delay, or is
something drifting/sticking?
For LiDAR processing latency: nearest-preceding /rsairy_ns/points bag-time vs
/trailer/articulation_angle publish bag-time (std_msgs/Float64 has no header,
so this is a bag-receive-time proxy, not a header-stamp-exact latency).

Usage
-----
  python3 scripts/audit_articulation_chain.py --bag data/hitch_angle_sweep

Dependencies: numpy, rosbag2_py, sensor_msgs_py (same as the other scripts
in this directory — no open3d/scipy needed here).
"""

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ============================================================
#  Bag reading (same minimal pattern as export_trailer_roi_cloud.py)
# ============================================================

class BagReader:
    def __init__(self, bag_path: str, storage_id: str = "mcap"):
        self.bag_path = str(bag_path)
        self.storage_id = storage_id
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(
            rosbag2_py.StorageOptions(uri=self.bag_path, storage_id=self.storage_id),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        self.topic_types = {t.name: t.type for t in self.reader.get_all_topics_and_types()}

    def read_all(self, topics=None):
        """Returns {topic: [(bag_time_s, msg), ...]} sorted by bag_time."""
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=self.bag_path, storage_id=self.storage_id),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        if topics:
            reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
        out = {}
        while reader.has_next():
            topic, data, ts_ns = reader.read_next()
            msg_type = self.topic_types.get(topic)
            if msg_type is None:
                continue
            try:
                cls = get_message(msg_type)
                msg = deserialize_message(data, cls)
            except Exception:
                continue
            out.setdefault(topic, []).append((ts_ns * 1e-9, msg))
        return out


# ============================================================
#  Per-topic availability stats
# ============================================================

def topic_stats(name, series, expected_hz=None):
    if not series:
        print(f"  {name:45s}  ABSENT (0 messages)")
        return None
    t = np.array([s[0] for s in series])
    n = len(t)
    duration = t[-1] - t[0] if n > 1 else 0.0
    hz = (n - 1) / duration if duration > 0 else 0.0
    dt = np.diff(t)
    max_gap = dt.max() if len(dt) else 0.0
    dropout_note = ""
    if expected_hz and max_gap > 3.0 / expected_hz:
        dropout_note = f"  <-- GAP: {max_gap:.2f}s (expected ~{1.0/expected_hz:.2f}s between msgs)"
    print(f"  {name:45s}  n={n:5d}  {hz:6.2f} Hz  span={duration:6.2f}s  "
          f"max_gap={max_gap:.3f}s{dropout_note}")
    return {"t": t, "n": n, "hz": hz, "max_gap": max_gap}


# ============================================================
#  Nearest-neighbour time alignment
# ============================================================

def nearest_match(t_ref, v_ref, t_query):
    """For each t_query, find the nearest t_ref sample; returns (aligned_v, dt)."""
    idx = np.searchsorted(t_ref, t_query)
    idx = np.clip(idx, 1, len(t_ref) - 1)
    left = idx - 1
    right = idx
    use_left = np.abs(t_query - t_ref[left]) <= np.abs(t_ref[right] - t_query)
    chosen = np.where(use_left, left, right)
    return v_ref[chosen], t_query - t_ref[chosen]


def float64_series(series):
    t = np.array([s[0] for s in series])
    v = np.array([s[1].data for s in series], dtype=np.float64)
    order = np.argsort(t)
    return t[order], v[order]


def normalize_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))


# ============================================================
#  Main
# ============================================================

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bag", required=True)
    p.add_argument("--storage-id", default="mcap")
    p.add_argument("--max-lag-s", type=float, default=1.0,
                    help="Max |lag| to search when cross-correlating command vs hardware (s)")
    args = p.parse_args()

    topics = [
        "/hardware/articulation_angle",
        "/trailer/articulation_angle",
        "/trailer/articulation_detected",
        "/trailer/articulation_stm_confidence",
        "/mtt_articulation_angle",
        "articulation_servo/setpoint_rad",
        "articulation_servo/measured_rad",
        "articulation_servo/steer_cmd",
        "articulation_servo/error_rad",
        "/rsairy_ns/points",
    ]

    bag = BagReader(args.bag, args.storage_id)
    print(f"Bag: {args.bag}")
    print(f"Topics present: {sorted(bag.topic_types.keys())}\n")
    data = bag.read_all(topics)

    print("=" * 90)
    print("AVAILABILITY / FREQUENCY")
    print("=" * 90)
    stats = {}
    expected = {
        "/hardware/articulation_angle": None,
        "/trailer/articulation_angle": 10.0,   # RS-Airy rate
        "/rsairy_ns/points": 10.0,
        "articulation_servo/setpoint_rad": None,
        "articulation_servo/measured_rad": None,
    }
    for t in topics:
        stats[t] = topic_stats(t, data.get(t, []), expected.get(t))

    # ── Hardware vs trailer (LiDAR/KF) agreement ──
    print("\n" + "=" * 90)
    print("HARDWARE ENCODER vs TRAILER (LiDAR/KF fused) — agreement")
    print("=" * 90)
    if "/hardware/articulation_angle" in data and "/trailer/articulation_angle" in data:
        t_hw, v_hw = float64_series(data["/hardware/articulation_angle"])
        t_tr, v_tr = float64_series(data["/trailer/articulation_angle"])
        if len(t_hw) > 2 and len(t_tr) > 2:
            v_hw_aligned, dt = nearest_match(t_hw, v_hw, t_tr)
            err = normalize_angle(v_tr - v_hw_aligned)  # SO(2)-correct residual
            print(f"  n_compared = {len(err)}")
            print(f"  mean error  = {np.rad2deg(err.mean()):+.3f} deg  "
                  f"(bias — consistent offset would show here)")
            print(f"  std error   = {np.rad2deg(err.std()):.3f} deg")
            print(f"  max |error| = {np.rad2deg(np.abs(err).max()):.3f} deg")
            print(f"  RMS error   = {np.rad2deg(np.sqrt((err**2).mean())):.3f} deg")
            # Range covered, to know if the sweep actually exercised the full range
            print(f"  hardware angle range covered: "
                  f"[{np.rad2deg(v_hw.min()):.1f}, {np.rad2deg(v_hw.max()):.1f}] deg")
            if np.rad2deg(v_hw.max() - v_hw.min()) < 60.0:
                print("  WARNING: covered range < 60 deg total — this bag may not have swept "
                      "close to the +-42/45 deg limits. Re-check before trusting the ROI at "
                      "the extremes.")
        else:
            print("  Not enough samples on one of the two topics.")
    else:
        print("  Missing /hardware/articulation_angle or /trailer/articulation_angle in this bag.")

    # ── Command/setpoint vs measured/hardware — response lag ──
    print("\n" + "=" * 90)
    print("COMMAND/SETPOINT vs MEASURED — response lag (cross-correlation)")
    print("=" * 90)
    cmd_topic = "articulation_servo/setpoint_rad" if "articulation_servo/setpoint_rad" in data else "/mtt_articulation_angle"
    meas_topic = "articulation_servo/measured_rad" if "articulation_servo/measured_rad" in data else "/hardware/articulation_angle"
    if cmd_topic in data and meas_topic in data and len(data[cmd_topic]) > 5 and len(data[meas_topic]) > 5:
        t_cmd, v_cmd = float64_series(data[cmd_topic])
        t_meas, v_meas = float64_series(data[meas_topic])
        # Resample both onto a common uniform grid for cross-correlation.
        t0, t1 = max(t_cmd[0], t_meas[0]), min(t_cmd[-1], t_meas[-1])
        if t1 > t0:
            dt_grid = 0.02  # 50 Hz
            grid = np.arange(t0, t1, dt_grid)
            cmd_g = np.interp(grid, t_cmd, v_cmd)
            meas_g = np.interp(grid, t_meas, v_meas)
            max_lag_n = int(args.max_lag_s / dt_grid)
            xcorr = np.correlate(meas_g - meas_g.mean(), cmd_g - cmd_g.mean(), mode="full")
            lags = np.arange(-len(cmd_g) + 1, len(cmd_g))
            center = len(cmd_g) - 1
            window = slice(center - max_lag_n, center + max_lag_n + 1)
            best = np.argmax(xcorr[window]) - max_lag_n
            lag_s = best * dt_grid
            print(f"  comparing '{cmd_topic}' -> '{meas_topic}'")
            print(f"  estimated lag = {lag_s*1000:+.1f} ms "
                  f"(positive = measured lags behind command)")
        else:
            print("  No overlapping time window between command and measured topics.")
    else:
        print(f"  Missing '{cmd_topic}' or '{meas_topic}' (or too few samples).")

    # ── LiDAR processing latency proxy ──
    print("\n" + "=" * 90)
    print("LiDAR PROCESSING LATENCY (proxy: nearest-preceding /rsairy_ns/points bag-time)")
    print("=" * 90)
    print("  NOTE: std_msgs/Float64 has no header — this is bag-receive-time based, not an")
    print("  exact scan-to-publish latency. Treat as an order-of-magnitude check.")
    if "/rsairy_ns/points" in data and "/trailer/articulation_angle" in data:
        t_cloud = np.array([s[0] for s in data["/rsairy_ns/points"]])
        t_cloud.sort()
        t_tr, _ = float64_series(data["/trailer/articulation_angle"])
        idx = np.searchsorted(t_cloud, t_tr) - 1
        valid = idx >= 0
        latency = t_tr[valid] - t_cloud[idx[valid]]
        latency = latency[(latency >= 0) & (latency < 1.0)]  # drop obviously-wrong matches
        if len(latency):
            print(f"  n={len(latency)}  mean={latency.mean()*1000:.1f}ms  "
                  f"p95={np.percentile(latency,95)*1000:.1f}ms  max={latency.max()*1000:.1f}ms")
        else:
            print("  Not enough valid matches.")
    else:
        print("  Missing /rsairy_ns/points or /trailer/articulation_angle.")

    # ── STM confidence during the sweep ──
    print("\n" + "=" * 90)
    print("MOTOR-FEEDBACK CONFIDENCE (trailer_detector_node's cross-check health)")
    print("=" * 90)
    if "/trailer/articulation_stm_confidence" in data and data["/trailer/articulation_stm_confidence"]:
        _, v_conf = float64_series(data["/trailer/articulation_stm_confidence"])
        print(f"  mean={v_conf.mean():.2f}  min={v_conf.min():.2f}  "
              f"fraction below min_confidence(0.20)={np.mean(v_conf < 0.20)*100:.1f}%")
    else:
        print("  Missing /trailer/articulation_stm_confidence.")

    print("\nDone.")


if __name__ == "__main__":
    main()
