#!/bin/bash
# pre_session_check.sh
#
# Pre-flight checklist before a data collection session.
# Run this on the robot before starting the Docker compose.
#
# Checks:
#   1. NTP / clock synchronization
#   2. Network (LiDAR, GPS reach)
#   3. USB devices (Reach RS)
#   4. Disk space
#   5. Workspace / bag storage
#
# Usage: bash scripts/pre_session_check.sh [--gps-mode serial|tcp]

set -uo pipefail

# ── Colors ────────────────────────────────────────────────────────────────────
OK="\033[92m✓\033[0m"
FAIL="\033[91m✗\033[0m"
WARN="\033[93m⚠\033[0m"
BOLD="\033[1m"
RESET="\033[0m"

# ── Config (can be overridden by env vars) ────────────────────────────────────
GPS_MODE="${GPS_MODE:-serial}"
HESAI_IP="${HESAI_IP:-192.168.2.201}"
# NOTE: enp8s0 on the robot is 192.168.1.102 — make sure RS_IP is the Bpearl's actual IP,
# not the robot's own address (pinging yourself always succeeds, masking a real failure).
RS_IP="${RS_IP:-192.168.1.102}"
# Reach RS device IPs (the IP of each Reach RS box on its USB-ethernet interface).
# Confirmed: gps_left (Reach+) = 192.168.2.59 / gps_right (Reach) = 192.168.2.241
# TCP output ports are currently CLOSED on both units — serial mode is used.
REACH_LEFT_IP="${REACH_LEFT_IP:-192.168.2.59}"
REACH_RIGHT_IP="${REACH_RIGHT_IP:-192.168.2.241}"
REACH_LEFT_TCP_PORT="${REACH_LEFT_TCP_PORT:-9001}"
REACH_RIGHT_TCP_PORT="${REACH_RIGHT_TCP_PORT:-9696}"
WORKSPACE="${WORKSPACE:-$(cd "$(dirname "$0")/.." && pwd)}"
DATA_DIR="${WORKSPACE}/data"
# Minimum free space in GB
MIN_FREE_GB="${MIN_FREE_GB:-50}"
# Estimated MB/s for the full sensor stack
EST_MB_PER_S=20

# ── Parse args ────────────────────────────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        --gps-mode=*) GPS_MODE="${arg#*=}" ;;
        --gps-mode) shift; GPS_MODE="$1" ;;
    esac
done

# Serial IDs — match sensors_interfaces.yaml
MTI100_ID="${MTI100_ID:-usb-Xsens_MTi-100_IMU_017824F1-if01-port0}"
MTI10_ID="${MTI10_ID:-usb-Xsens_MTi-10_IMU_016820DB-if01-port0}"
OAK_USB_ID="${OAK_USB_ID:-03e7:2485}"   # Intel Movidius MyriadX (OAK-D)

# Clock offset thresholds (ms).
# MTT uses NTP only (no GPS PPS hardware sync), so 50 ms is an acceptable hard limit.
# Warn at 20 ms to flag marginal sync before it causes timestamp misalignment.
GPS_SYNC_WARN_MS="${GPS_SYNC_WARN_MS:-20}"
GPS_SYNC_FAIL_MS="${GPS_SYNC_FAIL_MS:-50}"

pass=0; fail=0; warn=0

check() {
    local label="$1"
    local result="$2"  # ok | fail | warn
    local detail="${3:-}"
    case "$result" in
        ok)   echo -e "  $OK  $label${detail:+  ($detail)}"; ((pass++)) ;;
        fail) echo -e "  $FAIL  $label${detail:+  ($detail)}"; ((fail++)) ;;
        warn) echo -e "  $WARN  $label${detail:+  ($detail)}"; ((warn++)) ;;
    esac
}

ping_host() {
    local ip="$1"
    ping -c1 -W2 "$ip" &>/dev/null && echo "ok" || echo "fail"
}

# Check if an IP is assigned to a local interface (USB-ethernet device connected).
echo ""
echo -e "${BOLD}══════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}   MTT Pre-Session Checklist${RESET}"
echo -e "${BOLD}══════════════════════════════════════════════════${RESET}"
echo ""

# ─── 1. Clock / NTP ──────────────────────────────────────────────────────────
echo -e "${BOLD}[1] Clock synchronization${RESET}"

if command -v timedatectl &>/dev/null; then
    ntp_ok=$(timedatectl show --property=NTPSynchronized --value 2>/dev/null)
    if [ "$ntp_ok" = "yes" ]; then
        check "NTP synchronized" "ok"
    else
        check "NTP synchronized" "fail" "Run: sudo systemctl restart chronyd"
    fi
elif command -v chronyc &>/dev/null; then
    if chronyc tracking &>/dev/null; then
        check "chrony running" "ok"
    else
        check "chrony running" "fail" "sudo systemctl start chronyd"
    fi
else
    check "NTP check" "warn" "timedatectl/chrony not found — clock may drift"
fi

# Clock offset quality — critical for IMU/GPS temporal alignment
if command -v chronyc &>/dev/null && chronyc tracking &>/dev/null 2>&1; then
    rms_line=$(chronyc tracking 2>/dev/null | grep "RMS offset")
    rms_val=$(echo "$rms_line" | awk '{print $4}')
    rms_unit=$(echo "$rms_line" | awk '{print $5}')
    # Convert to ms for comparison
    rms_ms=$(awk "BEGIN {v=\"${rms_val:-999}\"; u=\"${rms_unit:-seconds}\"; \
        if(u==\"seconds\") print v*1000; \
        else if(u==\"milliseconds\") print v; \
        else if(u==\"microseconds\") print v/1000; \
        else print 999}" 2>/dev/null)
    rms_display="${rms_val:-?} ${rms_unit:-}"
    int_ms=$(printf "%.0f" "${rms_ms:-999}" 2>/dev/null || echo 999)
    if [ "$int_ms" -le "$GPS_SYNC_WARN_MS" ]; then
        check "Clock offset quality" "ok" "RMS ${rms_display} (< ${GPS_SYNC_WARN_MS} ms ✓)"
    elif [ "$int_ms" -le "$GPS_SYNC_FAIL_MS" ]; then
        check "Clock offset quality" "warn" "RMS ${rms_display} — marginal for IMU/GPS sync (threshold: ${GPS_SYNC_WARN_MS} ms)"
    else
        check "Clock offset quality" "fail" "RMS ${rms_display} — too high for reliable IMU/GPS timestamps"
    fi
else
    check "Clock offset quality" "warn" "chronyc not available — cannot measure offset"
fi

dt_utc=$(date -u +"%Y-%m-%d %H:%M:%S UTC")
echo "    System time: $dt_utc"
echo ""

# ─── 2. Network / LiDAR ──────────────────────────────────────────────────────
echo -e "${BOLD}[2] Network (LiDAR + Reach RS ping)${RESET}"

r=$(ping_host "$HESAI_IP")
check "Hesai XT-32 ($HESAI_IP)" "$r"

r=$(ping_host "$RS_IP")
check "RS Bpearl ($RS_IP)" "$r"

# Reach RS devices expose a USB-ethernet interface — ping them directly.
# If not reachable: device not plugged in, or no USB-ethernet link.
r=$(ping_host "$REACH_LEFT_IP")
check "Reach+ left  ($REACH_LEFT_IP)" "$r" \
    "$([ "$r" = "ok" ] && echo "device reachable" || echo "not reachable — USB not plugged in?")"
r=$(ping_host "$REACH_RIGHT_IP")
check "Reach  right ($REACH_RIGHT_IP)" "$r" \
    "$([ "$r" = "ok" ] && echo "device reachable" || echo "not reachable — USB not plugged in?")"

# GPS TCP port test — only useful if GPS_MODE=tcp (currently ports are closed).
# Kept here for when TCP output is configured in ReachView3.
tcp_port_test() {
    local ip="$1" port="$2"
    python3 - <<PYEOF 2>/dev/null
import socket
try:
    with socket.create_connection(("$ip", $port), timeout=2.5) as s:
        s.settimeout(2.0)
        data = s.recv(256)
        if data:
            snippet = data.decode("ascii", errors="replace")[:60].strip()
            nmea = "NMEA ok" if "\$" in snippet else f"data: {snippet}"
            print(f"ok:{nmea}")
        else:
            print("ok:connected, no data")
except ConnectionRefusedError:
    print("warn:port $port refused — enable NMEA TCP output in ReachView3")
except Exception as e:
    print(f"warn:{e}")
PYEOF
}

if [ "$GPS_MODE" = "tcp" ]; then
    gps_test_left=$(tcp_port_test "$REACH_LEFT_IP" "$REACH_LEFT_TCP_PORT" 2>/dev/null)
    check "Reach+ left  TCP $REACH_LEFT_IP:$REACH_LEFT_TCP_PORT" "${gps_test_left%%:*}" "${gps_test_left#*:}"

    gps_test_right=$(tcp_port_test "$REACH_RIGHT_IP" "$REACH_RIGHT_TCP_PORT" 2>/dev/null)
    check "Reach  right TCP $REACH_RIGHT_IP:$REACH_RIGHT_TCP_PORT" "${gps_test_right%%:*}" "${gps_test_right#*:}"
else
    check "GPS TCP ports" "ok" "serial mode — TCP not needed"
fi

echo ""

# ─── 3. USB / Serial ─────────────────────────────────────────────────────────
echo -e "${BOLD}[3] USB devices${RESET}"

# Reach RS serial devices confirmed:
#   gps_left  (Reach+) → /dev/ttyACM1   (or /dev/reach_left  if udev configured)
#   gps_right (Reach)  → /dev/ttyACM0   (or /dev/reach_right if udev configured)
if [ "$GPS_MODE" = "serial" ]; then
    if [ -e /dev/reach_left ]; then
        check "GPS left  (/dev/reach_left → udev symlink)" "ok"
    elif [ -e /dev/ttyACM1 ]; then
        check "GPS left  (/dev/ttyACM1)" "ok" "udev symlink not set up — using ttyACM1 directly"
    else
        check "GPS left  (/dev/ttyACM1)" "fail" "not found — Reach+ not plugged in"
    fi
    if [ -e /dev/reach_right ]; then
        check "GPS right (/dev/reach_right → udev symlink)" "ok"
    elif [ -e /dev/ttyACM0 ]; then
        check "GPS right (/dev/ttyACM0)" "ok" "udev symlink not set up — using ttyACM0 directly"
    else
        check "GPS right (/dev/ttyACM0)" "fail" "not found — Reach not plugged in"
    fi
else
    check "GPS serial devices" "ok" "GPS mode=tcp — NMEA over network, no ttyACM needed"
fi

# CAN interface
if ip link show can0 &>/dev/null; then
    can_state=$(ip link show can0 | grep -o "state [A-Z]*" | awk '{print $2}')
    if [ "$can_state" = "UP" ]; then
        check "CAN interface can0" "ok"
        # CAN liveness — verify MTT is actually publishing frames
        if command -v candump &>/dev/null; then
            # Use a temp file to avoid || echo 0 firing when timeout exits 124
            frame_count=$(timeout 2 candump -n 20 can0 2>/dev/null | wc -l | tr -d ' \n')
            frame_count=${frame_count:-0}
            if [ "$frame_count" -ge 10 ]; then
                check "CAN bus liveness (can0)" "ok" "${frame_count} frames in 2s — MTT active"
            elif [ "$frame_count" -gt 0 ]; then
                check "CAN bus liveness (can0)" "warn" "Only ${frame_count} frames — MTT may be in standby"
            else
                check "CAN bus liveness (can0)" "warn" "No CAN frames detected — MTT off or not transmitting"
            fi
        else
            check "CAN liveness check" "warn" "candump not found — install can-utils"
        fi
    else
        check "CAN interface can0 ($can_state)" "warn" "Run: sudo ip link set can0 up type can bitrate 250000"
    fi
else
    check "CAN interface can0" "warn" "Not found — will be configured by Docker startup"
fi

# IMU devices
MTI100_PATH="/dev/serial/by-id/${MTI100_ID}"
if [ -e "$MTI100_PATH" ]; then
    check "IMU MTi-100 (primary)" "ok" "$MTI100_PATH"
else
    check "IMU MTi-100 (primary)" "fail" "Not found at $MTI100_PATH — check USB"
fi

MTI10_PATH="/dev/serial/by-id/${MTI10_ID}"
if [ -e "$MTI10_PATH" ]; then
    check "IMU MTi-10 (secondary)" "ok" "$MTI10_PATH"
else
    check "IMU MTi-10 (secondary)" "warn" "Not found — secondary IMU, optional"
fi

# ZED camera
if ls /dev/video* &>/dev/null 2>&1; then
    zed_count=$(ls /dev/video* 2>/dev/null | wc -l)
    check "ZED camera (/dev/video*)" "ok" "${zed_count} video device(s)"
else
    check "ZED camera (/dev/video*)" "warn" "No /dev/video* — ZED not connected"
fi

# OAK-D camera (Intel Movidius MyriadX)
if command -v lsusb &>/dev/null; then
    if lsusb 2>/dev/null | grep -qi "$OAK_USB_ID"; then
        check "OAK-D camera ($OAK_USB_ID)" "ok"
    else
        check "OAK-D camera ($OAK_USB_ID)" "warn" "Not found — OAK not connected or not powered"
    fi
else
    check "OAK-D USB check" "warn" "lsusb not found"
fi
echo ""

# ─── 4. Disk space ───────────────────────────────────────────────────────────
echo -e "${BOLD}[4] Disk space${RESET}"

mkdir -p "$DATA_DIR"
avail_kb=$(df -k "$DATA_DIR" | awk 'NR==2{print $4}')
avail_gb=$((avail_kb / 1024 / 1024))
avail_mb=$((avail_kb / 1024))
minutes=$((avail_mb / EST_MB_PER_S / 60))

if [ "$avail_gb" -ge "$MIN_FREE_GB" ]; then
    check "Disk space ($DATA_DIR)" "ok" "${avail_gb} GB free (~${minutes} min at ${EST_MB_PER_S} MB/s)"
elif [ "$avail_gb" -ge 10 ]; then
    check "Disk space ($DATA_DIR)" "warn" "${avail_gb} GB free (~${minutes} min) — below ${MIN_FREE_GB}GB target"
else
    check "Disk space ($DATA_DIR)" "fail" "${avail_gb} GB free — too low for a meaningful session"
fi

echo "    Storage path: $DATA_DIR"
echo ""

# ─── 5. Docker image ──────────────────────────────────────────────────────────
echo -e "${BOLD}[5] Docker${RESET}"

MTT_IMAGE="${MTT_IMAGE:-mtt_workspace:devel}"
if docker image inspect "$MTT_IMAGE" &>/dev/null; then
    img_created=$(docker image inspect "$MTT_IMAGE" --format '{{.Created}}' | cut -c1-10)
    check "Docker image ($MTT_IMAGE)" "ok" "created $img_created"
else
    check "Docker image ($MTT_IMAGE)" "fail" "Run: dcr compile (or dcu + dcr compile)"
fi
echo ""

# ─── Summary ──────────────────────────────────────────────────────────────────
echo -e "${BOLD}══════════════════════════════════════════════════${RESET}"
echo -e "  ${BOLD}Results:${RESET}  ✓ $pass  ⚠ $warn  ✗ $fail"
echo ""
if [ "$fail" -gt 0 ]; then
    echo -e "  \033[91m${BOLD}❌  GO/NO-GO: NO-GO — fix the errors above first.${RESET}"
    exit 1
elif [ "$warn" -gt 0 ]; then
    echo -e "  \033[93m${BOLD}⚠   GO/NO-GO: CONDITIONAL GO — check warnings.${RESET}"
    exit 2
else
    echo -e "  \033[92m${BOLD}✅  GO/NO-GO: GO — everything OK.${RESET}"
    echo ""
    echo "  Next: cd demos/data_collection && dcr up"
    exit 0
fi
