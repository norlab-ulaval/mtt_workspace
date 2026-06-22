#!/usr/bin/env python3
"""Deep IMU+odometry audit — bias, drift, model error.

For each IMU topic found in the bag, reports:
  1. Stationary gyro bias (mean angular_velocity.z) during first N seconds
  2. Heading drift from quaternion (deg/min)
  3. Recommended bias value for runtime_odometry

Usage:
  python3 scripts/audit_imu_deep.py <bag_or_session_path>
"""

import argparse
import math
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rosidl_runtime_py.utilities import get_message


IMU_TOPICS = ["/mti100/data", "/mti100/data_unbiased", "/mti100/data_raw"]


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def resolve_bag(path):
    p = Path(path)
    if (p / "metadata.yaml").exists():
        return p
    if (p / "bag" / "metadata.yaml").exists():
        return p / "bag"
    for sub in p.iterdir():
        candidate = sub / "metadata.yaml"
        if candidate.exists():
            return candidate.parent
    raise FileNotFoundError(f"No bag found at {path}")


def read_imu_data(bag_path, topic, max_samples=10000):
    """Read IMU data from a single topic. Returns list of (t, gyro_z, yaw)."""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="mcap"),
        ConverterOptions("cdr", "cdr"),
    )
    all_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in all_types:
        return []
    
    reader.set_filter(StorageFilter(topics=[topic]))
    mt = get_message(all_types[topic])
    
    records = []
    while reader.has_next() and len(records) < max_samples:
        _, raw, t_ns = reader.read_next()
        msg = deserialize_message(raw, mt)
        yaw = quat_to_yaw(msg.orientation)
        records.append((
            float(t_ns) * 1e-9,
            msg.angular_velocity.z,
            yaw,
        ))
    return records


def analyze_imu_bias_and_drift(records, topic_name, window_s=60.0):
    """Analyze records = list of (t, gyro_z, yaw)."""
    n = len(records)
    if n == 0:
        print(f"  {topic_name}: no messages")
        return None
    
    first_t = records[0][0]
    last_t = records[-1][0]
    duration = last_t - first_t
    
    print(f"\n─── {topic_name} ───")
    print(f"  Messages: {n} over {duration:.1f}s ({n/duration:.1f} Hz)")
    
    # Find stationary period: first N seconds where gyro_z variation is low
    # Use first window_s seconds
    window_end = first_t + window_s
    window_records = [r for r in records if r[0] <= window_end]
    window_n = len(window_records)
    
    if window_n < 10:
        print(f"  ⚠️  Less than 10 samples in first {window_s}s window")
        return None
    
    # Compute mean and std of gyro_z in window
    z_vals = [r[1] for r in window_records]
    mean_z = sum(z_vals) / len(z_vals)
    var_z = sum((v - mean_z) ** 2 for v in z_vals) / len(z_vals)
    std_z = math.sqrt(var_z)
    
    # Heading drift during window
    first_yaw = window_records[0][2]
    last_yaw = window_records[-1][2]
    dyaw = wrap_pi(last_yaw - first_yaw)
    dt = window_records[-1][0] - window_records[0][0]
    drift_deg_s = math.degrees(dyaw) / max(dt, 1e-6)
    
    print(f"\n  🟢 Stationary gyro bias (first {dt:.1f}s):")
    print(f"     angular_velocity.z mean: {mean_z:+.5f} rad/s")
    print(f"     angular_velocity.z std : {std_z:.5f} rad/s")
    print(f"     heading drift          : {math.degrees(dyaw):+.3f}°")
    print(f"     drift rate             : {drift_deg_s:+.4f}°/s = {drift_deg_s*60:+.2f}°/min")
    
    # Full bag heading drift
    full_dyaw = wrap_pi(records[-1][2] - records[0][2])
    full_drift = math.degrees(full_dyaw) / duration
    print(f"\n  🔵 Full bag heading drift ({duration:.0f}s):")
    print(f"     yaw {math.degrees(records[0][2]):+.2f}° → {math.degrees(records[-1][2]):+.2f}°")
    print(f"     total drift: {math.degrees(full_dyaw):+.1f}° ({full_drift:+.4f}°/s = {full_drift*60:+.2f}°/min)")
    
    # Integrate gyro_z to get heading and compare to quaternion
    # Initialize integration from first quaternion heading
    heading_int = records[0][2]
    max_div = 0.0
    max_div_at = 0.0
    for i in range(1, len(records)):
        dt_i = records[i][0] - records[i-1][0]
        if dt_i <= 0 or dt_i > 0.5:
            continue
        heading_int = wrap_pi(heading_int + records[i][1] * dt_i)
        div = abs(wrap_pi(records[i][2] - heading_int))
        if div > max_div:
            max_div = div
            max_div_at = records[i][0] - records[0][0]
    
    final_div = abs(wrap_pi(records[-1][2] - heading_int))
    print(f"\n  📊 Gyro integration vs quaternion heading:")
    print(f"     max divergence : {math.degrees(max_div):.2f}° at t={max_div_at:.0f}s")
    print(f"     final div      : {math.degrees(final_div):.2f}°")
    
    # Bias sign convention: runtime_odometry uses: sign * z - bias
    # sign=-1.0, so effective = -1.0 * mean_z - bias
    # For effective to be 0 during stationary: bias = -sign * mean_z = 1.0 * mean_z
    # With sign=-1.0: bias = -mean_z  (because -z - bias = 0 → bias = -z = -mean_z)
    # No wait: eff = sign * z - bias = -1 * mean_z - bias
    # For eff to be 0: -mean_z - bias = 0 → bias = -mean_z
    # Hmm, that doesn't sound right. Let me re-derive.
    # In on_imu(): yaw_rate = imu_yaw_rate_sign_ * msg->angular_velocity.z - imu_yaw_rate_bias_rad_s_
    # With sign=-1.0: yaw_rate = -z - bias
    # Stationary: z = mean_z, and we want yaw_rate = 0
    # So: -mean_z - bias = 0 → bias = -mean_z
    # If mean_z = -0.0195 (negative), then bias = 0.0195 → -(-0.0195) - 0.0195 = 0 ✓ (matches our earlier finding)
    
    bias_pr = -mean_z  # For sign=-1.0
    print(f"\n  🎯 RECOMMENDED BIAS (for sign=-1.0):")
    print(f"     imu_yaw_rate_bias_rad_s = {bias_pr:+.5f}")
    print(f"     (To apply: set REPLAY_ODOM_IMU_BIAS={bias_pr:+.5f})")
    print(f"     (To enable IMU: set REPLAY_ODOM_IMU_TOPIC=/mti100/data)")
    
    return {
        "bias": bias_pr,
        "mean_z": mean_z,
        "std_z": std_z,
        "drift_deg_s": drift_deg_s,
        "drift_window_deg": math.degrees(dyaw),
        "full_drift_deg": math.degrees(full_dyaw),
        "max_divergence_deg": math.degrees(max_div),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", help="Path to bag or session directory")
    parser.add_argument("--window-s", type=float, default=60.0,
                        help="Stationary analysis window (default 60s)")
    args = parser.parse_args()
    
    bag_path = resolve_bag(args.bag_dir)
    print(f"\n{'='*60}")
    print(f" DEEP IMU AUDIT")
    print(f" Bag: {bag_path}")
    print(f"{'='*60}")
    
    found_any = False
    for topic in IMU_TOPICS:
        records = read_imu_data(bag_path, topic, max_samples=10000)
        if records:
            found_any = True
            analyze_imu_bias_and_drift(records, topic, args.window_s)
    
    if not found_any:
        print("  No IMU topics found. Available topics:")
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(bag_path), storage_id="mcap"),
            ConverterOptions("cdr", "cdr"),
        )
        for t in reader.get_all_topics_and_types():
            if "mti100" in t.name or "imu" in t.name:
                print(f"    {t.name}")
    
    print(f"\n{'='*60}")
    print(" AUDIT COMPLETE")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
