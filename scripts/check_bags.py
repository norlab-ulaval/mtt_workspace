#!/usr/bin/env python3
"""
check_bags.py — Vérifie les message counts de tous les bags MTT.
Lit les metadata.yaml directement — aucun ROS requis, marche partout.

Usage:
  python3 Workspace/mtt_workspace/scripts/check_bags.py
  python3 ~/Project/mtt_ws/scripts/check_bags.py --data-dir /chemin/vers/data
"""

import sys, argparse
from pathlib import Path

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

def infer_workspace_root(script_path: Path) -> Path:
    for candidate in [script_path.parent, *script_path.parents]:
        if (candidate / "src").exists() and (candidate / "demos").exists():
            return candidate
    return script_path.parent


DATA_DIR = infer_workspace_root(Path(__file__).resolve()) / "data"

BOLD  = "\033[1m"; RED   = "\033[91m"; YELLOW = "\033[93m"
GREEN = "\033[92m"; CYAN  = "\033[96m"; DIM   = "\033[2m"
ORANGE = "\033[38;5;208m"; RESET = "\033[0m"

# ── Topics surveillés ────────────────────────────────────────────────────────
# Format: { group_label: [(topic, is_primary)] }
# is_primary=True → topic DOIT avoir des msgs pour que le groupe soit OK
WATCH = {
    "ZED": [
        ("/zed/zed_node/rgb/color/rect/image/compressed",          True,  "rgb/compressed"),
        ("/zed/zed_node/depth/depth_registered/compressedDepth",   True,  "depth/compressedDepth"),
        ("/zed/zed_node/imu/data",                                 False, "imu/data"),
    ],
    "OAK": [
        ("/oak/rgb/image_rect",                                    True,  "rgb/image_rect"),
        ("/oak/stereo/image_raw",                                  True,  "stereo/image_raw"),
        ("/oak/points",                                            False, "points"),
    ],
    "GPS Single": [
        ("/gps/fix",                                               True,  "gps/fix"),
        ("/gps/time_reference",                                    False, "gps/time_reference"),
        ("/gps/nmea_sentence",                                     False, "gps/nmea_sentence"),
    ],
    "GPS Dual": [
        ("/gps_left/fix",                                          True,  "gps_left/fix"),
        ("/gps_right/fix",                                         True,  "gps_right/fix"),
        ("/gps/heading",                                           True,  "gps/heading"),
        ("/gps_left/nmea_sentence",                                False, "gps_left/nmea"),
        ("/gps_right/nmea_sentence",                               False, "gps_right/nmea"),
    ],
    "LiDAR": [
        ("/hesai_lidar/points",                                    True,  "hesai/points"),
        ("/rsairy_ns/points",                                      True,  "rsairy/points"),
    ],
    "IMU": [
        ("/mti100/data",                                           True,  "mti100/data"),
        ("/mti10/data",                                            True,  "mti10/data"),
    ],
    "CAN/Odom": [
        ("/mtt_tachometer",                                        True,  "mtt_tachometer"),
        ("/mtt_status",                                            False, "mtt_status"),
        ("/mtt_odometry",                                          False, "mtt_odometry"),
    ],
    "ICP": [
        ("/merged_points_filtered",                                  False, "merged_points_filtered"),
        ("/mapping/icp_odom",                                      True,  "mapping/icp_odom"),
        ("/trailer/angle",                                         False, "trailer/angle"),
    ],
    "BMS": [
        ("/mtt_battery/status",                                    True,  "mtt_battery/status"),
        ("/from_can_bus",                                          False, "from_can_bus"),
    ],
}

# ── Causes racines connues ────────────────────────────────────────────────────
KNOWN_CAUSES = {
    "/zed/zed_node/rgb/color/rect/image/compressed":
        "QoS mismatch: ZED SDK force BEST_EFFORT, recorder attend RELIABLE\n"
        "     FIX: qos_override.yaml + --qos-profile-overrides-path dans compose.yaml ✅ FAIT",
    "/zed/zed_node/depth/depth_registered/compressedDepth":
        "QoS mismatch: même cause ZED\n"
        "     FIX: inclus dans qos_override.yaml ✅ FAIT",
    "/zed/zed_node/imu/data":
        "QoS mismatch: même cause ZED (2-3 msgs = bruit, pas réel)\n"
        "     FIX: inclus dans qos_override.yaml ✅ FAIT",
    "/gps_left/fix":
        "Driver GPS tente TCP port 5001 → Reach RS écoute sur 9001 ou 9696\n"
        "     FIX: corriger host/port dans gps_tcp.yaml ⚠️  À FAIRE",
    "/gps_right/fix":
        "Driver GPS tente TCP port 5001 → Reach RS écoute sur 9001 ou 9696\n"
        "     FIX: corriger host/port dans gps_tcp.yaml ⚠️  À FAIRE",
    "/gps/fix":
        "Le Reach RS publie du NMEA mais aucun GGA valide n'est converti en NavSatFix\n"
        "     FIX: vérifier GGA 5 Hz + RMC 1 Hz côté ReachView3 et lire les compteurs du driver GPS",
    "/gps/time_reference":
        "Pas de date RMC valide ou pas de GGA valide → pas de GPS UTC publié\n"
        "     FIX: vérifier la sortie RMC côté Reach et les diagnostics du parser",
    "/gps/heading":
        "Dépend de gps_left/fix + gps_right/fix → mort si GPS morts",
    "/gps_left/nmea_sentence":
        "Reach RS n'émet pas NMEA sur ce port (config ReachView3 requise)",
    "/gps_right/nmea_sentence":
        "Reach RS n'émet pas NMEA sur ce port (config ReachView3 requise)",
    "/mtt_tachometer":
        "Encodeur inductif mort avant 14h58 (disque 10 dents a frotté face capteur)\n"
        "     FIX: remplacement matériel + vérifier gap 1-2mm 🔧 MATÉRIEL",
    "/mapping/icp_odom":
        "ICP absent ou inutilisable — vérifier le topic, le délai de lancement, et surtout le mode deskew\n"
        "     Avec tachometer_mode=cmd_sim, traiter /mapping/icp_odom comme référence locale, pas comme ground truth",
    "/merged_points_filtered":
        "Le cloud merger ne publie pas ce que le mapper consomme\n"
        "     FIX: vérifier TF lidar->base_link, /hesai_lidar/points, /rsairy_ns/points et perception.launch.py",
    "/oak/stereo/image_raw":
        "Le pipeline RGBD OAK ne sort pas — souvent câble USB desserré après vibration\n"
        "     FIX: rebrancher l'OAK, vérifier /dev/bus/usb et relancer le driver",
    "/oak/points":
        "Le point cloud OAK n'est pas activé ou l'entrée depth RGBD ne sort pas\n"
        "     FIX: activer pointcloud.enable=true et vérifier /oak/stereo/image_raw",
    "/mtt_battery/status":
        "BMS decoding non compile avant aujourd'hui — rebuild mtt_driver et redéployer\n"
        "     FIX: colcon build --packages-select mtt_msgs mtt_driver && dc up --build robot",
    "/from_can_bus":
        "socketcan_bridge non lance — service 'socketcan_bridge' absent du compose ou can0 down\n"
        "     FIX: dc up socketcan_bridge (ou vérifier que can0 est up: ip link show can0)",
}


def parse_metadata(meta_path: Path):
    """Parse metadata.yaml → (counts dict, duration_s, total_msgs)."""
    text = meta_path.read_text()

    if HAS_YAML:
        data  = yaml.safe_load(text)
        info  = data.get("rosbag2_bagfile_information", data)
        counts = {
            e["topic_metadata"]["name"]: e["message_count"]
            for e in info.get("topics_with_message_count", [])
        }
        dur_s = info.get("duration", {}).get("nanoseconds", 0) / 1e9
        total = info.get("message_count", 0)
        return counts, dur_s, total
    else:
        # Minimal parser — no yaml module
        counts: dict = {}
        current_name = None
        dur_s, total = 0, 0
        for line in text.splitlines():
            s = line.strip()
            if s.startswith("name:") and "/" in s:
                current_name = s.split("name:", 1)[1].strip().strip('"')
            elif s.startswith("message_count:") and current_name:
                try:
                    counts[current_name] = int(s.split(":", 1)[1].strip())
                except ValueError:
                    pass
                current_name = None
            elif s.startswith("nanoseconds:") and dur_s == 0:
                try:
                    dur_s = int(s.split(":", 1)[1].strip()) / 1e9
                except ValueError:
                    pass
        return counts, dur_s, sum(counts.values())


def group_status(entries, counts):
    """
    Returns 'ok', 'dead', 'partial', 'missing'.
    ok      = tous les topics primaires ont des msgs
    partial = certains topics ont des msgs, mais des primaires en manquent
    dead    = tous les primaires à 0 (mais présents dans le bag)
    missing = aucun topic du groupe n'est dans le bag
    """
    primary_counts   = [(t, counts.get(t, -1)) for t, primary, _ in entries if primary]
    secondary_counts = [(t, counts.get(t, -1)) for t, primary, _ in entries if not primary]

    if all(c == -1 for t, c in primary_counts):
        return "missing"

    primary_ok = [c for t, c in primary_counts if c > 0]

    if all(c > 0 for t, c in primary_counts):
        return "ok"
    if primary_ok:
        return "partial"
    return "dead"


STATUS_ICON = {
    "ok":      f"{GREEN}✓ OK     {RESET}",
    "partial": f"{ORANGE}~ PARTIAL{RESET}",
    "dead":    f"{RED}✗ DEAD   {RESET}",
    "missing": f"{DIM}— NOT REC{RESET}",
}


def analyze_session(session_dir: Path) -> set:
    """Returns set of broken (0 msgs) primary topic names."""
    bag_dir   = session_dir / "bag"
    meta_path = bag_dir / "metadata.yaml"

    name = session_dir.name
    ts   = name.rsplit("_", 2)
    ts_str = (ts[-2] + " " + ts[-1].replace("-", ":")) if len(ts) >= 3 else "?"

    if not bag_dir.exists() or not meta_path.exists():
        print(f"\n  {DIM}{name}{RESET}")
        print(f"    {YELLOW}⚠  pas de bag/ ou metadata.yaml{RESET}")
        return set()

    counts, dur_s, total = parse_metadata(meta_path)
    total_k = total / 1000

    print(f"\n  {BOLD}{name}{RESET}")
    print(f"  {DIM}{ts_str}  {dur_s:.0f}s  {total_k:.0f}k msgs{RESET}")

    broken = set()

    for group, entries in WATCH.items():
        status = group_status(entries, counts)
        icon   = STATUS_ICON[status]
        print(f"    {BOLD}{group:12s}{RESET} {icon}")

        for topic, is_primary, short in entries:
            c = counts.get(topic, -1)
            marker = f"{'[P]' if is_primary else '   '}"
            if c == -1:
                print(f"      {DIM}{marker} {short:<40s} —{RESET}")
            elif c == 0:
                col = RED if is_primary else YELLOW
                print(f"      {col}{marker} {short:<40s} 0 msgs{RESET}")
                if is_primary:
                    broken.add(topic)
            else:
                print(f"      {GREEN}{marker} {short:<40s} {c:,}{RESET}")

    return broken


def print_root_causes(all_broken: set):
    print(f"\n{BOLD}{RED}══ CAUSES RACINES (topics primaires à 0) ══{RESET}\n")

    explained   = {}
    unexplained = []

    for topic in sorted(all_broken):
        cause = KNOWN_CAUSES.get(topic)
        if cause:
            explained.setdefault(cause, []).append(topic)
        else:
            unexplained.append(topic)

    for cause, topics in explained.items():
        print(f"  {RED}●{RESET} {topics[0].split('/')[-1] if len(topics)==1 else ', '.join(t.split('/')[-1] for t in topics)}")
        print(f"    {cause}")
        print()

    if unexplained:
        print(f"  {YELLOW}● Cause inconnue :{RESET}")
        for t in unexplained:
            print(f"    {t}")
        print()


def print_summary_table(session_results: list):
    """One-line per session summary."""
    groups = list(WATCH.keys())
    header = f"  {'Session':42s}" + "".join(f" {g[:5]:5s}" for g in groups)
    print(f"\n{BOLD}{CYAN}══ Tableau récapitulatif ══{RESET}\n")
    print(f"{DIM}{header}{RESET}")
    print(f"  {'-'*42}" + "-" * (len(groups) * 6))

    icons = {"ok": f"{GREEN}  ✓  {RESET}", "partial": f"{ORANGE}  ~  {RESET}",
             "dead": f"{RED}  ✗  {RESET}", "missing": f"{DIM}  —  {RESET}"}

    for name, statuses in session_results:
        short = name[-42:]
        row = f"  {short:42s}" + "".join(icons.get(s, "  ?  ") for s in statuses)
        print(row)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=Path, default=DATA_DIR)
    args = parser.parse_args()

    data_dir = args.data_dir
    if not data_dir.exists():
        print(f"{RED}Data dir not found: {data_dir}{RESET}")
        sys.exit(1)

    print(f"\n{BOLD}{CYAN}══════════════════════════════════════════════════════{RESET}")
    print(f"{BOLD}{CYAN}   MTT Bag Health Check — {data_dir}{RESET}")
    print(f"{BOLD}{CYAN}══════════════════════════════════════════════════════{RESET}")
    print(f"  {DIM}[P] = topic primaire (doit avoir des msgs pour que le groupe soit OK){RESET}")

    sessions = sorted([d for d in data_dir.iterdir()
                       if d.is_dir() and d.name.startswith("mtt_")])

    all_broken: set = set()
    session_results = []

    for s in sessions:
        broken = analyze_session(s)
        all_broken.update(broken)

        # collect per-group status for summary table
        bag_dir   = s / "bag"
        meta_path = bag_dir / "metadata.yaml"
        if meta_path.exists():
            counts, _, _ = parse_metadata(meta_path)
            statuses = [group_status(entries, counts) for entries in WATCH.values()]
        else:
            statuses = ["missing"] * len(WATCH)
        session_results.append((s.name, statuses))

    print_summary_table(session_results)
    print()
    print_root_causes(all_broken)


if __name__ == "__main__":
    main()
