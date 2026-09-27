#!/usr/bin/env python3
"""
extract_mathis_topics.py
------------------------
Script d'extraction de données de télémétrie depuis un rosbag ROS 2 (MCAP).
Conçu pour Mathis afin d'évaluer la vitesse de la chenille, l'angle d'articulation et l'IMU.

Topics extraits principaux:
  - /articulation_servo/measured_rad (std_msgs/msg/Float64)
  - /mtt_tachometer (mtt_msgs/msg/MttTachometerData)
  - /mti100/data (sensor_msgs/msg/Imu)

Topics additionnels (bonus):
  - /hardware/articulation_angle (std_msgs/msg/Float64)
  - /cmd_vel (geometry_msgs/msg/TwistStamped)
  - /mtt_odometry (nav_msgs/msg/Odometry)
"""

import sys
import math
import csv
import argparse
from contextlib import ExitStack
from pathlib import Path

def find_mcap_bag_directory(input_path: str) -> str:
    """Vérifie l'existence du dossier du bag et retourne le chemin correct contenant metadata.yaml ou .mcap."""
    p = Path(input_path).resolve()
    if not p.exists():
        raise FileNotFoundError(f"Le chemin spécifié n'existe pas: {input_path}")
    if p.is_file():
        if p.suffix != ".mcap":
            raise ValueError("Le fichier doit être un .mcap")
        return str(p.parent)

    # Si le dossier passé contient direct un sous-dossier 'bag' avec metadata.yaml
    if (p / "bag" / "metadata.yaml").exists() or (p / "bag" / "bag_0.mcap").exists():
        return str(p / "bag")
    elif (p / "metadata.yaml").exists() or any(p.glob("*.mcap")):
        return str(p)
    else:
        # Essayer de chercher récursivement ou utiliser p
        candidates = {f.parent for f in p.rglob("*.mcap")}
        if len(candidates) > 1:
            raise ValueError("Plusieurs bags trouvés; préciser un dossier de bag unique")
        if candidates:
            return str(candidates.pop())
        raise FileNotFoundError(f"Aucun fichier .mcap ou metadata.yaml trouvé dans {input_path}")


def main():
    parser = argparse.ArgumentParser(description="Extrait des données de télémétrie d'un ROS 2 bag MCAP vers CSV.")
    parser.add_argument("--bag-path", type=str, required=True,
                        help="Chemin vers le dossier du bag (ou du dossier racine).")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Dossier où sauvegarder les fichiers CSV (par défaut: <bag_root>/output_csv).")
    parser.add_argument("--include-bonus", action=argparse.BooleanOptionalAction, default=True,
                        help="Extraire aussi les topics additionnels intéressants (/hardware/articulation_angle, /cmd_vel, /mtt_odometry).")
    args = parser.parse_args()
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import Float64
        from sensor_msgs.msg import Imu
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from mtt_msgs.msg import MttTachometerData
    except ImportError as error:
        parser.error(f"Sourcer ROS 2 et le workspace compilé : {error}")

    # 1. Validation du dossier du bag
    root_path = Path(args.bag_path).resolve()
    try:
        bag_dir = find_mcap_bag_directory(root_path)
    except (FileNotFoundError, ValueError) as err:
        print(f"❌ Erreur: {err}")
        sys.exit(1)

    print(f"📁 Bag localisé dans: {bag_dir}")

    # Déterminer le dossier de sortie pour les CSV
    if args.output_dir:
        out_dir = Path(args.output_dir).resolve()
    else:
        if root_path.is_dir() and not (root_path / "metadata.yaml").exists():
            out_dir = root_path / "output_csv"
        else:
            out_dir = root_path.parent / "output_csv"

    print(f"📂 Les fichiers CSV seront sauvegardés dans: {out_dir}")

    # 2. Configuration des topics à extraire
    topics_config = {
        "/articulation_servo/measured_rad": {
            "type": Float64,
            "filename": "articulation_servo_measured_rad.csv",
            "columns": ["timestamp_sec", "timestamp_nanosec", "time_relative_sec", "measured_rad", "measured_deg"],
            "handler": lambda timestamp, msg, t0: [
                f"{timestamp * 1e-9:.9f}",
                timestamp,
                f"{(timestamp - t0) * 1e-9:.9f}",
                f"{msg.data:.6f}",
                f"{math.degrees(msg.data):.4f}"
            ]
        },
        "/mtt_tachometer": {
            "type": MttTachometerData,
            "filename": "mtt_tachometer.csv",
            "columns": [
                "timestamp_sec", "timestamp_nanosec", "time_relative_sec", "header_stamp_sec",
                "speed_ms", "speed_kmh", "tachometer_instant_rps", "tachometer_cumulative",
                "distance_km", "direction", "steer_cmd", "telemetry_fresh", "telemetry_age_ms",
                "tachometer_is_synthetic", "tachometer_source"
            ],
            "handler": lambda timestamp, msg, t0: [
                f"{timestamp * 1e-9:.9f}",
                timestamp,
                f"{(timestamp - t0) * 1e-9:.9f}",
                f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.9f}",
                f"{msg.speed_ms:.6f}",
                f"{msg.speed_kmh:.6f}",
                msg.tachometer_instant,
                msg.tachometer_cumulative,
                f"{msg.distance_km:.6f}",
                msg.direction,
                f"{msg.steer_cmd:.6f}",
                msg.telemetry_fresh,
                f"{msg.telemetry_age_ms:.2f}",
                msg.tachometer_is_synthetic,
                msg.tachometer_source
            ]
        },
        "/mti100/data": {
            "type": Imu,
            "filename": "mti100_imu.csv",
            "columns": [
                "timestamp_sec", "timestamp_nanosec", "time_relative_sec", "header_stamp_sec",
                "orient_x", "orient_y", "orient_z", "orient_w",
                "ang_vel_x", "ang_vel_y", "ang_vel_z",
                "lin_acc_x", "lin_acc_y", "lin_acc_z"
            ],
            "handler": lambda timestamp, msg, t0: [
                f"{timestamp * 1e-9:.9f}",
                timestamp,
                f"{(timestamp - t0) * 1e-9:.9f}",
                f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.9f}",
                f"{msg.orientation.x:.8f}", f"{msg.orientation.y:.8f}", f"{msg.orientation.z:.8f}", f"{msg.orientation.w:.8f}",
                f"{msg.angular_velocity.x:.8f}", f"{msg.angular_velocity.y:.8f}", f"{msg.angular_velocity.z:.8f}",
                f"{msg.linear_acceleration.x:.8f}", f"{msg.linear_acceleration.y:.8f}", f"{msg.linear_acceleration.z:.8f}"
            ]
        }
    }

    if args.include_bonus:
        topics_config["/hardware/articulation_angle"] = {
            "type": Float64,
            "filename": "hardware_articulation_angle.csv",
            "columns": ["timestamp_sec", "timestamp_nanosec", "time_relative_sec", "articulation_angle_rad", "articulation_angle_deg"],
            "handler": lambda timestamp, msg, t0: [
                f"{timestamp * 1e-9:.9f}",
                timestamp,
                f"{(timestamp - t0) * 1e-9:.9f}",
                f"{msg.data:.6f}",
                f"{math.degrees(msg.data):.4f}"
            ]
        }
        topics_config["/cmd_vel"] = {
            "type": TwistStamped,
            "filename": "cmd_vel.csv",
            "columns": [
                "timestamp_sec", "timestamp_nanosec", "time_relative_sec", "header_stamp_sec",
                "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"
            ],
            "handler": lambda timestamp, msg, t0: [
                f"{timestamp * 1e-9:.9f}",
                timestamp,
                f"{(timestamp - t0) * 1e-9:.9f}",
                f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.9f}",
                f"{msg.twist.linear.x:.6f}", f"{msg.twist.linear.y:.6f}", f"{msg.twist.linear.z:.6f}",
                f"{msg.twist.angular.x:.6f}", f"{msg.twist.angular.y:.6f}", f"{msg.twist.angular.z:.6f}"
            ]
        }
        topics_config["/mtt_odometry"] = {
            "type": Odometry,
            "filename": "mtt_odometry.csv",
            "columns": [
                "timestamp_sec", "timestamp_nanosec", "time_relative_sec", "header_stamp_sec",
                "pos_x", "pos_y", "pos_z", "orient_x", "orient_y", "orient_z", "orient_w",
                "twist_linear_x", "twist_linear_y", "twist_angular_z"
            ],
            "handler": lambda timestamp, msg, t0: [
                f"{timestamp * 1e-9:.9f}",
                timestamp,
                f"{(timestamp - t0) * 1e-9:.9f}",
                f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.9f}",
                f"{msg.pose.pose.position.x:.6f}", f"{msg.pose.pose.position.y:.6f}", f"{msg.pose.pose.position.z:.6f}",
                f"{msg.pose.pose.orientation.x:.8f}", f"{msg.pose.pose.orientation.y:.8f}", f"{msg.pose.pose.orientation.z:.8f}", f"{msg.pose.pose.orientation.w:.8f}",
                f"{msg.twist.twist.linear.x:.6f}", f"{msg.twist.twist.linear.y:.6f}", f"{msg.twist.twist.angular.z:.6f}"
            ]
        }

    # 3. Ouvrir les fichiers CSV en écriture
    csv_writers = {}
    counters = {topic: 0 for topic in topics_config}

    # 4. Lecture du Rosbag avec rosbag2_py
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)
    collisions = [out_dir / cfg["filename"] for cfg in topics_config.values()
                  if (out_dir / cfg["filename"]).exists()]
    if collisions:
        parser.error(f"Les fichiers de sortie existent déjà : {collisions}")
    out_dir.mkdir(parents=True, exist_ok=True)

    # Filtrer uniquement les topics désirés pour une vitesse optimale
    target_topics = list(topics_config.keys())
    reader.set_filter(rosbag2_py.StorageFilter(topics=target_topics))

    print(f"⚡ Lecture en cours du bag MCAP pour {len(target_topics)} topics...")
    t_start_bag = None
    processed_count = 0

    with ExitStack() as stack:
        for topic, cfg in topics_config.items():
            stream = stack.enter_context((out_dir / cfg["filename"]).open(
                "x", newline="", encoding="utf-8"))
            writer = csv.writer(stream)
            writer.writerow(cfg["columns"])
            csv_writers[topic] = writer
        while reader.has_next():
            topic, raw_data, timestamp = reader.read_next()
            if topic in topics_config:
                if t_start_bag is None:
                    t_start_bag = timestamp

                cfg = topics_config[topic]
                msg = deserialize_message(raw_data, cfg["type"])
                row = cfg["handler"](timestamp, msg, t_start_bag)
                csv_writers[topic].writerow(row)
                counters[topic] += 1

                processed_count += 1
                if processed_count % 50000 == 0:
                    print(f"  -> {processed_count:,} messages traités...", flush=True)

    # 5. Résumé final
    print("\n✅ extraction terminée avec succès !")
    print("--------------------------------------------------")
    print(f"Fichiers générés dans {out_dir}:")
    for topic, cfg in topics_config.items():
        filepath = out_dir / cfg["filename"]
        size_mb = filepath.stat().st_size / (1024 * 1024)
        print(f"  • {cfg['filename']:<35} : {counters[topic]:>8,} lignes ({size_mb:.2f} MB)")
    print("--------------------------------------------------")


if __name__ == "__main__":
    main()
