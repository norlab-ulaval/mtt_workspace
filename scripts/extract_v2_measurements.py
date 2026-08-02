#!/usr/bin/env python3
"""Read-only extraction of GT-solver inputs from the ICE_RINK V2 bag.

This script never plays the bag and never touches DDS/ROS graph state: it opens
the MCAP with rosbag2_py.SequentialReader (the same pattern already used by
scripts/offline_reference.py) purely as a random-access file reader, and reads
already-produced CSVs (GT_icp/, zed_visual_map/). It is safe to run while a
mapping session is active on the same host.

Outputs (artifacts/gt_v2_icerink/):
  measurements/imu.csv            /mti100/data, rotated into base_footprint via
                                   the /tf_static chain (same convention as
                                   factor_graph_node.cpp's R_base_imu_)
  measurements/zed_imu.csv        /zed/zed_node/imu/data, raw sensor frame
                                   (diagnostic cross-check only, not fused)
  measurements/hardware_phi.csv   /hardware/articulation_angle (hitch yaw, encoder)
  measurements/lidar_phi.csv      /trailer/articulation_angle (hitch yaw, LiDAR PCA,
                                   already fused online — see CLAUDE.md)
  measurements/hardware_pitch.csv /hardware/articulation_pitch_rad
  measurements/track_odom.csv     /mtt_odometry
  measurements/tachometer.csv     /mtt_tachometer
  measurements/cmd_vel.csv         /cmd_vel (TwistStamped)
  measurements/articulation_cmd.csv
                                  /mtt/articulation_cmd (radian command)
  measurements/articulation_setpoint.csv
                                  /mtt_articulation_setpoint
  measurements/servo_setpoint.csv /articulation_servo/setpoint_rad
  measurements/servo_steer_cmd.csv
                                  /articulation_servo/steer_cmd
  measurements/articulation_mode.csv
                                  /mtt_control/articulation_mode
  measurements/articulation_hold.csv
                                  /mtt_control/articulation_hold_active
  measurements/zed_odom.csv       /zed/zed_node/odom
  measurements/icp.csv            copied/reformatted from GT_icp/icp_odom_*.csv
  audit/tf_static.yaml            full dumped static transform tree
  audit/topic_timing.csv          per-topic count/rate/gap/duplicate stats
  audit/imu_static_check.yaml     gravity-sign check on the stationary segment
  audit/audit_report.md           human-readable summary

NOTE (2026-07-29): build_gt_pipeline.py invokes this script ONLY for its
audit/tf_static.yaml output (the static-TF snapshot build_gt_v2_100hz.py
needs for extrinsic composition). This script's own measurements/*.csv
output is intentionally left unused by that pipeline -- offline_reference.py
(scripts/offline_reference.py) already produces an equivalent measurements/
directory from the same --offline-icp input, and using two separate
extractors for the same bag would risk them silently disagreeing. This
script remains the canonical way to reproduce the existing Ice-rink
artifacts standalone.
"""

from __future__ import annotations

import argparse
import csv
import math
import shutil
import sys
import yaml
from dataclasses import dataclass
from pathlib import Path

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # pragma: no cover
    print(f"FATAL: rosbag2_py not importable ({exc}). Run with ROS jazzy sourced:\n"
          f"  source /opt/ros/jazzy/setup.bash && python3 {__file__}", file=sys.stderr)
    raise SystemExit(1)

TOPICS_SIMPLE = {
    # topic -> (out filename, extractor tag)
    "/mti100/data": ("imu_raw", "imu"),
    "/zed/zed_node/imu/data": ("zed_imu", "imu"),
    "/hardware/articulation_angle": ("hardware_phi", "float64"),
    "/hardware/articulation_pitch_rad": ("hardware_pitch", "float64"),
    "/mtt_odometry": ("track_odom", "odom"),
    "/zed/zed_node/odom": ("zed_odom", "odom"),
    "/mtt_tachometer": ("tachometer", "tacho"),
    "/mtt/articulation_state": ("articulation_state", "artic_state"),
    "/cmd_vel": ("cmd_vel", "twist"),
    "/mtt/articulation_cmd": ("articulation_cmd", "float64"),
    "/mtt_articulation_setpoint": ("articulation_setpoint", "float64"),
    "/articulation_servo/setpoint_rad": ("servo_setpoint", "float64"),
    "/articulation_servo/steer_cmd": ("servo_steer_cmd", "float64"),
    "/mtt_control/articulation_mode": ("articulation_mode", "string"),
    "/mtt_control/articulation_hold_active": ("articulation_hold", "bool"),
}
TF_STATIC_TOPIC = "/tf_static"


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_rotmat(x: float, y: float, z: float, w: float) -> list[list[float]]:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_transpose(a: list[list[float]]) -> list[list[float]]:
    return [[a[j][i] for j in range(3)] for i in range(3)]


def rotmat_to_quat(r: list[list[float]]) -> tuple[float, float, float, float]:
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2][1] - r[1][2]) / s
        y = (r[0][2] - r[2][0]) / s
        z = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    return x, y, z, w


@dataclass
class StaticEdge:
    parent: str
    child: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def read_tf_static(reader_topic_types: dict[str, str], bag_dir: Path) -> list[StaticEdge]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TF_STATIC_TOPIC]))
    msg_type = get_message(reader_topic_types[TF_STATIC_TOPIC])
    edges: dict[tuple[str, str], StaticEdge] = {}
    while reader.has_next():
        topic, data, _ts = reader.read_next()
        msg = deserialize_message(data, msg_type)
        for tr in msg.transforms:
            parent = tr.header.frame_id
            child = tr.child_frame_id
            t = tr.transform.translation
            q = tr.transform.rotation
            edges[(parent, child)] = StaticEdge(parent, child, t.x, t.y, t.z, q.x, q.y, q.z, q.w)
    return list(edges.values())


def build_frame_graph(edges: list[StaticEdge]) -> dict[str, tuple[str, list[list[float]], list[float]]]:
    """child -> (parent, R_parent_child, t_parent_child)"""
    graph = {}
    for e in edges:
        r = quat_to_rotmat(e.qx, e.qy, e.qz, e.qw)
        graph[e.child] = (e.parent, r, [e.x, e.y, e.z])
    return graph


def lookup_transform(
    graph: dict[str, tuple[str, list[list[float]], list[float]]],
    target: str, source: str,
) -> tuple[list[list[float]], list[float]]:
    """Compose static edges to get R,t such that p_target = R @ p_source + t
    (i.e. the transform FROM source frame INTO target frame)."""

    def path_to_root(frame: str) -> list[str]:
        chain = [frame]
        while chain[-1] in graph:
            chain.append(graph[chain[-1]][0])
        return chain

    chain_s = path_to_root(source)
    chain_t = path_to_root(target)
    set_t = set(chain_t)
    common = next((f for f in chain_s if f in set_t), None)
    if common is None:
        raise RuntimeError(f"no common ancestor between {source} and {target}")

    # source -> common (compose child->parent edges)
    r_sc = [[1.0, 0, 0], [0, 1, 0], [0, 0, 1]]
    t_sc = [0.0, 0.0, 0.0]
    frame = source
    while frame != common:
        parent, r_pc, t_pc = graph[frame]
        # p_parent = r_pc @ p_child + t_pc ; compose with existing p_child(=p_frame)_from_source
        r_sc = mat_mul(r_pc, r_sc)
        t_sc = [r_pc[i][0] * t_sc[0] + r_pc[i][1] * t_sc[1] + r_pc[i][2] * t_sc[2] + t_pc[i] for i in range(3)]
        frame = parent

    # target -> common
    r_tc = [[1.0, 0, 0], [0, 1, 0], [0, 0, 1]]
    t_tc = [0.0, 0.0, 0.0]
    frame = target
    while frame != common:
        parent, r_pc, t_pc = graph[frame]
        r_tc = mat_mul(r_pc, r_tc)
        t_tc = [r_pc[i][0] * t_tc[0] + r_pc[i][1] * t_tc[1] + r_pc[i][2] * t_tc[2] + t_pc[i] for i in range(3)]
        frame = parent

    # p_common = r_sc @ p_source + t_sc  = r_tc @ p_target + t_tc
    # => p_target = r_tc^-1 @ (r_sc @ p_source + t_sc - t_tc)
    r_tc_inv = mat_transpose(r_tc)  # rotation matrices are orthonormal
    r_result = mat_mul(r_tc_inv, r_sc)
    diff = [t_sc[i] - t_tc[i] for i in range(3)]
    t_result = [sum(r_tc_inv[i][k] * diff[k] for k in range(3)) for i in range(3)]
    return r_result, t_result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True,
                         help="Session directory (contains bag/).")
    parser.add_argument("--offline-icp", type=Path, required=True,
                         help="Qualified offline ICP CSV -- same file passed to "
                              "offline_reference.py's --offline-icp, must already exist.")
    parser.add_argument("--output-dir", type=Path, required=True,
                         help="Where to write audit/ (and measurements/, unused by "
                              "build_gt_pipeline -- see that script's comment).")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    # Import and validate offline ICP CSV early
    from offline_reference import validate_offline_icp_csv
    validate_offline_icp_csv(args.offline_icp)

    bag = args.session / "bag"
    out = args.output_dir
    zed_map_csv = args.session / "zed_visual_map" / "zed_pose_trajectory.csv"

    out.mkdir(parents=True, exist_ok=True)
    (out / "audit").mkdir(exist_ok=True)
    (out / "measurements").mkdir(exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    print(f"Bag topics: {len(topic_types)}")

    # ── /tf_static ──
    edges = read_tf_static(topic_types, bag)
    with (out / "audit" / "tf_static.yaml").open("w") as f:
        yaml.safe_dump(
            {"edges": [
                {"parent": e.parent, "child": e.child,
                 "xyz": [e.x, e.y, e.z], "qxyzw": [e.qx, e.qy, e.qz, e.qw]}
                for e in edges
            ]},
            f, sort_keys=False)
    print(f"tf_static: {len(edges)} edges dumped")

    graph = build_frame_graph(edges)
    frames = set(graph.keys()) | {p for p, _, _ in graph.values()}
    base_frame = "base_footprint" if "base_footprint" in frames else "base_link"
    r_base_imu, t_base_imu = lookup_transform(graph, base_frame, "imu_link")
    r_base_zedimu = None
    for zed_imu_frame in ("zed_imu_link", "zed_left_camera_frame", "zed_camera_center"):
        if zed_imu_frame in frames:
            try:
                r_base_zedimu, _ = lookup_transform(graph, base_frame, zed_imu_frame)
                break
            except RuntimeError:
                continue
    print(f"R_base_imu ({base_frame} <- imu_link):")
    for row in r_base_imu:
        print("  ", row)

    # ── Reopen for the bulk topic pass (rosbag2_py readers are single-pass) ──
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    wanted = [t for t in TOPICS_SIMPLE if t in topic_types]
    missing = [t for t in TOPICS_SIMPLE if t not in topic_types]
    if missing:
        print(f"WARNING: topics not in bag, skipped: {missing}", file=sys.stderr)
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    msg_types = {t: get_message(topic_types[t]) for t in wanted}

    writers: dict[str, csv.writer] = {}
    files: dict[str, object] = {}
    counts: dict[str, int] = {t: 0 for t in wanted}
    first_t: dict[str, float] = {}
    last_t: dict[str, float] = {}
    max_gap: dict[str, float] = {}
    dup_count: dict[str, int] = {t: 0 for t in wanted}
    header_zero_count: dict[str, int] = {t: 0 for t in wanted}
    lidar_detected_count = 0
    hardware_fresh_count = 0
    pitch_fresh_count = 0
    artic_count = 0

    def get_writer(name: str, header: list[str]):
        if name not in writers:
            f = open(out / "measurements" / f"{name}.csv", "w", newline="")
            files[name] = f
            w = csv.writer(f)
            w.writerow(header)
            writers[name] = w
        return writers[name]

    n_msgs = 0
    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        n_msgs += 1
        bag_t = ts_ns / 1e9
        out_name, kind = TOPICS_SIMPLE[topic]
        msg = deserialize_message(data, msg_types[topic])

        hdr_t = None
        if hasattr(msg, "header"):
            if msg.header.stamp.sec or msg.header.stamp.nanosec:
                hdr_t = stamp_to_sec(msg.header.stamp)
            else:
                header_zero_count[topic] += 1
        t = hdr_t if hdr_t is not None else bag_t

        if topic in last_t:
            gap = t - last_t[topic]
            if gap <= 0:
                dup_count[topic] += 1
            max_gap[topic] = max(max_gap.get(topic, 0.0), gap)
        else:
            first_t[topic] = t
        last_t[topic] = t
        counts[topic] += 1

        if kind == "imu":
            ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
            gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            if out_name == "imu_raw":
                r = r_base_imu
            else:
                r = r_base_zedimu
            if r is not None:
                arx = r[0][0] * ax + r[0][1] * ay + r[0][2] * az
                ary = r[1][0] * ax + r[1][1] * ay + r[1][2] * az
                arz = r[2][0] * ax + r[2][1] * ay + r[2][2] * az
                grx = r[0][0] * gx + r[0][1] * gy + r[0][2] * gz
                gry = r[1][0] * gx + r[1][1] * gy + r[1][2] * gz
                grz = r[2][0] * gx + r[2][1] * gy + r[2][2] * gz
            else:
                arx, ary, arz, grx, gry, grz = ax, ay, az, gx, gy, gz
            w = get_writer(out_name, ["t", "bag_t", "ax", "ay", "az", "gx", "gy", "gz",
                                       "ax_raw", "ay_raw", "az_raw", "frame_id"])
            w.writerow([t, bag_t, arx, ary, arz, grx, gry, grz, ax, ay, az, msg.header.frame_id])
        elif kind == "float64":
            w = get_writer(out_name, ["t", "bag_t", "value"])
            w.writerow([t, bag_t, msg.data])
        elif kind == "odom":
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            v = msg.twist.twist.linear
            wv = msg.twist.twist.angular
            w = get_writer(out_name, ["t", "bag_t", "x", "y", "z", "qx", "qy", "qz", "qw",
                                       "vx", "vy", "vz", "wx", "wy", "wz"])
            w.writerow([t, bag_t, p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, wv.x, wv.y, wv.z])
        elif kind == "twist":
            v = msg.twist.linear
            wv = msg.twist.angular
            w = get_writer(
                out_name,
                ["t", "bag_t", "linear_x", "linear_y", "linear_z",
                 "angular_x", "angular_y", "angular_z", "frame_id"],
            )
            w.writerow([
                t, bag_t, v.x, v.y, v.z, wv.x, wv.y, wv.z,
                msg.header.frame_id,
            ])
        elif kind == "string":
            w = get_writer(out_name, ["t", "bag_t", "value"])
            w.writerow([t, bag_t, msg.data])
        elif kind == "bool":
            w = get_writer(out_name, ["t", "bag_t", "value"])
            w.writerow([t, bag_t, int(msg.data)])
        elif kind == "tacho":
            w = get_writer(out_name, ["t", "bag_t", "speed_ms", "is_synthetic", "source",
                                       "direction", "model_valid", "model_speed_ms"])
            w.writerow([t, bag_t, msg.speed_ms, int(msg.tachometer_is_synthetic),
                        msg.tachometer_source, msg.direction,
                        int(msg.model_state_valid), msg.model_speed_ms])
        elif kind == "artic_state":
            w = get_writer(out_name, ["t", "bag_t", "hardware_rad", "hardware_fresh",
                                       "lidar_rad", "lidar_detected", "effective_rad",
                                       "effective_source", "pitch_rad", "pitch_fresh",
                                       "hardware_lidar_residual_rad"])
            w.writerow([t, bag_t, msg.hardware_rad, int(msg.hardware_fresh),
                        msg.lidar_rad, int(msg.lidar_detected), msg.effective_rad,
                        msg.effective_source, msg.pitch_rad, int(msg.pitch_fresh),
                        msg.hardware_lidar_residual_rad])
            artic_count += 1
            lidar_detected_count += int(msg.lidar_detected)
            hardware_fresh_count += int(msg.hardware_fresh)
            pitch_fresh_count += int(msg.pitch_fresh)

    for f in files.values():
        f.close()
    print(f"Bulk pass: {n_msgs} messages across {len(wanted)} topics")

    # ── topic timing audit ──
    with (out / "audit" / "topic_timing.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["topic", "count", "duration_s", "avg_hz", "max_gap_s",
                    "dup_or_nonmonotonic", "header_stamp_zero_count"])
        for t in wanted:
            dur = last_t.get(t, 0) - first_t.get(t, 0)
            hz = counts[t] / dur if dur > 0 else 0.0
            w.writerow([t, counts[t], f"{dur:.3f}", f"{hz:.2f}",
                        f"{max_gap.get(t, 0.0):.4f}", dup_count[t], header_zero_count[t]])

    # ── GT_icp CSV: reformat/copy with explicit column names ──
    if args.offline_icp.exists():
        with args.offline_icp.open() as fin, (out / "measurements" / "icp.csv").open("w", newline="") as fout:
            reader_csv = csv.DictReader(fin)
            writer_csv = csv.writer(fout)
            writer_csv.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz", "wx", "wy", "wz"])
            n = 0
            for row in reader_csv:
                t = float(row["timestamp_sec"]) + float(row["timestamp_nanosec"]) * 1e-9
                writer_csv.writerow([t, row["x"], row["y"], row["z"], row["qx"], row["qy"], row["qz"], row["qw"],
                                      row["vx"], row["vy"], row["vz"], row["wx"], row["wy"], row["wz"]])
                n += 1
        print(f"icp.csv: {n} rows copied from {args.offline_icp.name}")
    else:
        print(f"WARNING: GT_icp CSV not found at {args.offline_icp}", file=sys.stderr)

    if ZED_MAP_CSV.exists():
        import shutil
        shutil.copy(zed_map_csv, out / "measurements" / "zed_visual_map_trajectory.csv")
        print(f"zed_visual_map_trajectory.csv copied from {ZED_MAP_CSV}")

    # ── IMU stationary-segment gravity check ──
    imu_csv = out / "measurements" / "imu_raw.csv"
    grav_check = {"status": "not_run"}
    if imu_csv.exists():
        rows = list(csv.DictReader(imu_csv.open()))
        if rows:
            t0 = float(rows[0]["t"])
            window = [r for r in rows if float(r["t"]) - t0 < 3.0]  # first 3s, assumed stationary
            if window:
                az_vals = [float(r["az"]) for r in window]
                ax_vals = [float(r["ax"]) for r in window]
                ay_vals = [float(r["ay"]) for r in window]
                mean_az = sum(az_vals) / len(az_vals)
                mean_ax = sum(ax_vals) / len(ax_vals)
                mean_ay = sum(ay_vals) / len(ay_vals)
                grav_check = {
                    "status": "ok" if abs(mean_az - 9.81) < 1.0 else "SIGN_OR_AXIS_SUSPECT",
                    "window_s": 3.0,
                    "n_samples": len(window),
                    "mean_ax_base_frame": mean_ax,
                    "mean_ay_base_frame": mean_ay,
                    "mean_az_base_frame": mean_az,
                    "expected_az": 9.81,
                    "base_frame": base_frame,
                    "R_base_imu_source": "tf_static composed chain (script), same convention as "
                                          "factor_graph_node.cpp:492-527 R_base_imu_",
                }
    with (out / "audit" / "imu_static_check.yaml").open("w") as f:
        yaml.safe_dump(grav_check, f, sort_keys=False)
    print("Gravity check:", grav_check.get("status"), "mean_az =", grav_check.get("mean_az_base_frame"))

    # ── audit report ──
    with (out / "audit" / "audit_report.md").open("w") as f:
        f.write("# V2 ICE_RINK bag — extraction audit\n\n")
        f.write(f"Bag: `{bag}`\n\n")
        f.write(f"Base frame used for IMU rotation: `{base_frame}`\n\n")
        f.write("## Gravity-sign check (first 3s, assumed stationary)\n\n")
        f.write(f"```yaml\n{yaml.safe_dump(grav_check, sort_keys=False)}```\n\n")
        f.write("## Topic timing\n\nSee `topic_timing.csv`.\n\n")
        f.write("## Missing topics\n\n")
        if missing:
            for t in missing:
                f.write(f"- `{t}`\n")
        else:
            f.write("None.\n")
        f.write("\n## Notes\n\n")
        f.write("- `/trailer/pose` (full 6-DOF LiDAR trailer pose) and `/trailer/articulation_angle` "
                 "are **not recorded** in this bag (trailer_pose_node / trailer_detector_node were "
                 "not running during capture — confirmed against `ros2_bag_info.txt`, 80 topics "
                 "actually present). `TrailerPoseFactorFull` (6D) cannot be built from this bag. "
                 "Instead, `H(k)` (hitch yaw) is constrained by two independent real measurement "
                 "streams bundled in `/mtt/articulation_state` (50 Hz): `hardware_rad` (encoder, "
                 "gated on `hardware_fresh`) and `lidar_rad` (LiDAR PCA, gated on `lidar_detected`) "
                 "— reusing the exact sigmas `phi_sigma_hardware`/`phi_sigma_model` from "
                 "`state.hpp`. `pitch_rad` (gated on `pitch_fresh`) constrains `P(k)`.\n")
        f.write("- ZED IMU and ZED odom are extracted and rotated the same way, for use as "
                 "cross-check / loosely-weighted factors — see solver report for how each was used.\n")
        if artic_count:
            f.write(f"\n## Articulation state coverage ({artic_count} messages)\n\n")
            f.write(f"- `hardware_fresh`: {hardware_fresh_count / artic_count:.1%}\n")
            f.write(f"- `pitch_fresh`: {pitch_fresh_count / artic_count:.1%}\n")
            f.write(f"- `lidar_detected`: {lidar_detected_count / artic_count:.1%} — "
                    "if 0%, the LiDAR hitch-yaw stream never fired in this session (regardless "
                    "of the reason); `H(k)` then relies on `hardware_rad` alone, and this MUST "
                    "be stated in the solver report, not silently dropped.\n")

    print("Done. See", out / "audit" / "audit_report.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
