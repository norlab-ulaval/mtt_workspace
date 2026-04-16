#!/usr/bin/env python3
"""
analyze_temps.py — Forensics thermiques sur tous les bags MTT.

Lit le topic /mtt_status (ou /mtt_tachometer) dans chaque session,
extrait temp_A et temp_B, et identifie quand l'encodeur a surchauffé.

Usage (sur le robot, ROS sourcé ou dans le container):
  python3 ~/Project/mtt_ws/scripts/analyze_temps.py
  python3 ~/Project/mtt_ws/scripts/analyze_temps.py --plot   # courbe matplotlib si dispo
"""

import os, sys, argparse
from pathlib import Path
from datetime import datetime

DATA_DIR = Path(os.environ.get("DATA_DIR",
                Path.home() / "Project" / "mtt_ws" / "data"))

BOLD   = "\033[1m"
RED    = "\033[91m"
YELLOW = "\033[93m"
GREEN  = "\033[92m"
CYAN   = "\033[96m"
DIM    = "\033[2m"
RESET  = "\033[0m"

THRESH_WARN  = 50    # °C — temp_B commence à monter
THRESH_BAD   = 80    # °C — danger
THRESH_CRIT  = 100   # °C — seuil de dommage

# ─────────────────────────────────────────────────────────────────────────────

def color_temp(t: float) -> str:
    s = f"{t:+.0f}°C"
    if t >= THRESH_CRIT: return f"{RED}{BOLD}{s}{RESET}"
    if t >= THRESH_BAD:  return f"{YELLOW}{BOLD}{s}{RESET}"
    if t >= THRESH_WARN: return f"{YELLOW}{s}{RESET}"
    return f"{GREEN}{s}{RESET}"


def read_bag_temps(bag_dir: Path):
    """
    Returns list of (timestamp_s, temp_a, temp_b) from /mtt_status or /mtt_tachometer.
    Raises ImportError if ROS is not sourced.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_path = bag_dir / "bag"
    if not bag_path.exists():
        return None, "no bag/ subdir"

    if not list(bag_path.glob("*.mcap")):
        return None, "no .mcap file"

    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )

        type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

        # Prefer /mtt_status, fall back to /mtt_tachometer
        if "/mtt_status" in type_map:
            topic = "/mtt_status"
            get_a = lambda m: m.temperature_a
            get_b = lambda m: m.temperature_b
        elif "/mtt_tachometer" in type_map:
            topic = "/mtt_tachometer"
            get_a = lambda m: m.main_sensor_temp_a
            get_b = lambda m: m.main_sensor_temp_b
        else:
            return None, "no temperature topic found"

        msg_type = get_message(type_map[topic])
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

        readings = []
        while reader.has_next():
            _, data, ts_ns = reader.read_next()
            msg = deserialize_message(data, msg_type)
            readings.append((ts_ns / 1e9, get_a(msg), get_b(msg)))

        if not readings:
            return None, f"topic {topic} present but 0 messages"

        return readings, topic

    except Exception as e:
        return None, str(e)


def mini_bar(val, max_val=120, width=30, thresh=THRESH_CRIT) -> str:
    filled = int(round(val / max_val * width))
    bar = "█" * filled + "░" * (width - filled)
    color = RED if val >= thresh else (YELLOW if val >= THRESH_BAD else GREEN)
    return f"{color}{bar}{RESET}"


def analyze_session(session_dir: Path):
    name = session_dir.name

    readings, info = read_bag_temps(session_dir)

    if readings is None:
        print(f"  {DIM}{name}{RESET}")
        print(f"    {YELLOW}⚠  Skipped: {info}{RESET}\n")
        return None

    temps_a = [r[1] for r in readings]
    temps_b = [r[2] for r in readings]
    t_start = readings[0][0]
    t_end   = readings[-1][0]
    duration = t_end - t_start

    max_b_val  = max(temps_b)
    max_b_idx  = temps_b.index(max_b_val)
    max_b_t    = readings[max_b_idx][0] - t_start

    # Find first crossing of each threshold
    def first_cross(series, thresh):
        for r, v in zip(readings, series):
            if v >= thresh:
                return r[0] - t_start
        return None

    cross_50  = first_cross(temps_b, THRESH_WARN)
    cross_80  = first_cross(temps_b, THRESH_BAD)
    cross_100 = first_cross(temps_b, THRESH_CRIT)

    # Session timestamp from dir name  (mtt_TYPE_NAME_YYYY-MM-DD_HH-MM-SS)
    parts = name.rsplit("_", 2)
    session_time = parts[-2] + " " + parts[-1].replace("-", ":") if len(parts) >= 3 else "?"

    print(f"  {BOLD}{name}{RESET}  {DIM}({session_time}){RESET}")
    print(f"    Duration : {duration:.0f}s   |   {len(readings)} samples   |   topic: {info}")
    print(f"    temp_A : {color_temp(min(temps_a))} → {color_temp(max(temps_a))}  "
          f"(min / max)")
    print(f"    temp_B : {color_temp(min(temps_b))} → {color_temp(max_b_val)}  "
          f"(min / max)")
    print(f"    temp_B max at t+{max_b_t:.0f}s  {mini_bar(max_b_val)}")

    if cross_100 is not None:
        print(f"    {RED}{BOLD}🔥  temp_B ≥ 100°C at t+{cross_100:.0f}s  ← POINT DE RUPTURE{RESET}")
    elif cross_80 is not None:
        print(f"    {YELLOW}⚠   temp_B ≥ 80°C  at t+{cross_80:.0f}s{RESET}")
    elif cross_50 is not None:
        print(f"    {YELLOW}    temp_B ≥ 50°C  at t+{cross_50:.0f}s{RESET}")
    else:
        print(f"    {GREEN}    temp_B stable < 50°C{RESET}")

    print()
    return readings


def try_plot(all_sessions: list):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.dates as mdates

        fig, (ax_a, ax_b) = plt.subplots(2, 1, figsize=(14, 8), sharex=False)
        fig.suptitle("MTT Temperature Forensics — 2026-04-15", fontsize=13, fontweight="bold")

        colors = plt.cm.tab10.colors

        for i, (name, readings) in enumerate(all_sessions):
            if not readings:
                continue
            color = colors[i % len(colors)]
            t_start = readings[0][0]
            ts = [(r[0] - t_start) for r in readings]
            ta = [r[1] for r in readings]
            tb = [r[2] for r in readings]
            label = name.split("_", 3)[-1][:30]  # shorten

            ax_a.plot(ts, ta, color=color, linewidth=1.2, label=label)
            ax_b.plot(ts, tb, color=color, linewidth=1.2, label=label)

        for ax, title in [(ax_a, "temp_A (°C)"), (ax_b, "temp_B (°C) — encodeur")]:
            ax.set_ylabel(title)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7, loc="upper left")

        ax_b.axhline(THRESH_WARN, color="orange", linestyle="--", alpha=0.7, label="50°C warn")
        ax_b.axhline(THRESH_BAD,  color="red",    linestyle="--", alpha=0.7, label="80°C bad")
        ax_b.axhline(THRESH_CRIT, color="darkred",linestyle="-",  alpha=0.9, label="100°C CRITICAL")
        ax_b.legend(fontsize=7, loc="upper left")

        ax_b.set_xlabel("Temps dans la session (s)")
        plt.tight_layout()

        out = Path("/tmp/mtt_temp_forensics.png")
        plt.savefig(out, dpi=150)
        print(f"  {GREEN}Plot saved → {out}{RESET}")
        plt.show()

    except ImportError:
        print(f"  {DIM}(matplotlib non disponible — install avec: pip3 install matplotlib){RESET}")


# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="MTT temperature forensics")
    parser.add_argument("--data-dir", type=Path, default=DATA_DIR)
    parser.add_argument("--plot", action="store_true", help="Générer un graphe matplotlib")
    args = parser.parse_args()

    data_dir = args.data_dir
    if not data_dir.exists():
        print(f"{RED}Data dir not found: {data_dir}{RESET}")
        sys.exit(1)

    # Check ROS is available
    try:
        import rosbag2_py  # noqa
    except ImportError:
        print(f"{RED}rosbag2_py non disponible.{RESET}")
        print("Lance depuis le container docker:")
        print("  dc exec robot python3 /home/mtt/Project/mtt_ws/scripts/analyze_temps.py")
        print("Ou source ROS d'abord:")
        print("  source /opt/ros/jazzy/setup.bash && python3 scripts/analyze_temps.py")
        sys.exit(1)

    print(f"\n{BOLD}{CYAN}══════════════════════════════════════════════════{RESET}")
    print(f"{BOLD}{CYAN}   MTT Temperature Forensics — {data_dir.name}{RESET}")
    print(f"{BOLD}{CYAN}══════════════════════════════════════════════════{RESET}\n")
    print(f"  Seuils : warn={THRESH_WARN}°C  bad={THRESH_BAD}°C  critical={THRESH_CRIT}°C\n")

    sessions = sorted([d for d in data_dir.iterdir()
                       if d.is_dir() and d.name.startswith("mtt_")])

    if not sessions:
        print(f"  {YELLOW}Aucune session trouvée dans {data_dir}{RESET}")
        return

    all_data = []
    for s in sessions:
        readings = analyze_session(s)
        all_data.append((s.name, readings))

    # Global summary
    print(f"\n{BOLD}══ Résumé global ══{RESET}")
    any_crit = False
    for name, readings in all_data:
        if readings is None:
            continue
        tb = [r[2] for r in readings]
        max_b = max(tb)
        flag = f"{RED}{BOLD}🔥 CRITIQUE{RESET}" if max_b >= THRESH_CRIT else \
               (f"{YELLOW}⚠  chaud{RESET}" if max_b >= THRESH_BAD else
                f"{GREEN}OK{RESET}")
        print(f"  {name[-30:]:30s}  temp_B max={color_temp(max_b)}  {flag}")
        if max_b >= THRESH_CRIT:
            any_crit = True

    if any_crit:
        print(f"\n  {RED}{BOLD}→ L'encodeur a subi une temperature critique.{RESET}")
        print(f"  {RED}  Vérifier les sessions marquées 🔥 pour le moment exact.{RESET}")

    print()

    if args.plot:
        try_plot(all_data)


if __name__ == "__main__":
    main()
