#!/bin/bash
# post_launch_check.sh
#
# Fast go/no-go check of the RUNNING data_collection stack, meant to be run
# right after `docker compose --profile record --profile experiment up` and
# right before pressing deadman+A. Complements pre_session_check.sh (which
# checks hardware *before* the stack is up) -- this checks the *live* stack
# against the specific failure signatures hit during the 2026-08-04 ice-rink
# session, not a generic sensor sweep:
#
#   - Hesai LiDAR "wedge": ping succeeds, driver claims a socket, but zero
#     UDP packets arrive on /lidar_packets. `ros2 topic hz` on /hesai_lidar/points
#     alone does NOT catch this early -- /lidar_packets is the raw layer.
#   - rosbag2_recorder hang: container stays "running", CPU time frozen,
#     mcap file byte-identical for minutes. A single `ros2 bag info` can't
#     see this on an open bag; this checks bag mtime advancing over a short
#     window instead.
#   - deadman/estop/mode chain, and whether the zone map actually loaded
#     (silently fails open -- geofence/auto-reverse disable without erroring).
#
# Usage: bash scripts/post_launch_check.sh [demo_name]
#   demo_name defaults to data_collection. Pass mathis_com_shift, live_robot, etc.
# Exit: 0 = GO, 1 = NO-GO (see printed reasons)

set -uo pipefail

DEMO="${1:-data_collection}"

OK="\033[92m✓\033[0m"; FAIL="\033[91m✗\033[0m"; WARN="\033[93m⚠\033[0m"
BOLD="\033[1m"; RESET="\033[0m"
fail=0; warn=0

check() {
  local label="$1" result="$2" detail="${3:-}"
  case "$result" in
    ok)   echo -e "  $OK  $label${detail:+  ($detail)}" ;;
    fail) echo -e "  $FAIL  $label${detail:+  ($detail)}"; ((fail++)) ;;
    warn) echo -e "  $WARN  $label${detail:+  ($detail)}"; ((warn++)) ;;
  esac
}

ROS_EXEC="docker exec ${DEMO}-sensors-1 bash -c"
SRC="source /opt/ros/jazzy/setup.bash; source \$(docker exec ${DEMO}-sensors-1 printenv WORKSPACE)/install/setup.bash 2>/dev/null;"

echo -e "${BOLD}══════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}   MTT Post-Launch Go/No-Go — ${DEMO} (running stack)${RESET}"
echo -e "${BOLD}══════════════════════════════════════════════════${RESET}"

# ── 1. Containers ──
n_down=$(docker compose -f "$(dirname "$0")/../demos/${DEMO}/compose.yaml" ps --format '{{.Names}} {{.State}}' 2>/dev/null | grep -vc running)
if [ "${n_down:-1}" -eq 0 ]; then check "All containers running" "ok"; else check "All containers running" "fail" "${n_down} not running — check docker compose ps"; fi

# ── 2. Hesai wedge detector: raw UDP layer, not just the parsed cloud ──
lp_hz=$($ROS_EXEC "$SRC timeout 4 ros2 topic hz /lidar_packets --window 10 2>&1 | grep -c 'average rate'")
if [ "${lp_hz:-0}" -gt 0 ]; then check "Hesai raw UDP (/lidar_packets)" "ok"; else check "Hesai raw UDP (/lidar_packets)" "fail" "ping-alive-but-silent wedge signature -- needs full stack down/up, not just container restart"; fi

# ── 3. Recorder liveness: bag mtime must advance over a short window, not just container status ──
if ! docker ps --format '{{.Names}}' | grep -q "^${DEMO}-record-1$"; then
  check "Recorder writing" "warn" "record not started (no --profile record/ice yet)"
else
BAG_DIR=$(ls -dt ~/Project/mtt_ws/data/mtt_*/bag 2>/dev/null | head -1)
if [ -n "$BAG_DIR" ]; then
  MCAP=$(ls -t "$BAG_DIR"/*.mcap 2>/dev/null | head -1)
  if [ -n "$MCAP" ]; then
    S1=$(stat -c %s "$MCAP" 2>/dev/null); sleep 3; S2=$(stat -c %s "$MCAP" 2>/dev/null)
    if [ "$S1" != "$S2" ]; then check "Recorder writing (bag growing)" "ok"; else check "Recorder writing (bag growing)" "warn" "no growth in 3s -- could be chunk buffering, recheck in ~60s; if still flat, it's the known hang, SIGINT+reindex+relaunch"; fi
  else
    check "Recorder writing" "warn" "no .mcap in latest bag dir yet"
  fi
else
  check "Recorder writing" "warn" "record not started yet"
fi
fi

# ── 4. ZED/VSLAM chain actually delivering, not just topic-exists ──
vs=$($ROS_EXEC "$SRC timeout 3 ros2 topic echo /isaac/vslam/odometry --once 2>&1 | grep -c 'pose:'")
if [ "${vs:-0}" -gt 0 ]; then check "Isaac VSLAM odometry flowing" "ok"; else check "Isaac VSLAM odometry flowing" "fail"; fi

# ── 5. Safety chain ──
mode=$($ROS_EXEC "$SRC timeout 2 ros2 topic echo /mtt_control/selected_mode --once 2>&1 | grep -oP 'data: \K.*'")
estop=$($ROS_EXEC "$SRC timeout 2 ros2 topic echo /mtt_control/teleop_estop --once 2>&1 | grep -oP 'data: \K.*'")
check "Mode / ESTOP" "ok" "mode=${mode:-?} estop=${estop:-?}"

# ── 6. Zone map loaded (fails open, silently) — only applies where experiment_conductor exists ──
if ! docker ps --format '{{.Names}}' | grep -q "^${DEMO}-experiment_conductor-1$"; then
  check "Zone map / geofence" "warn" "no experiment_conductor in this demo — n/a"
elif docker logs "${DEMO}-experiment_conductor-1" --tail 100 2>&1 | grep -qi "geofence.*disabled"; then
  check "Zone map / geofence" "fail" "DISABLED -- check zone_map.npz path/parse before driving"
else
  check "Zone map / geofence" "ok"
fi

echo ""
if [ "$fail" -gt 0 ]; then
  echo -e "  \033[91m${BOLD}❌  NO-GO — fix failures above before deadman+A.${RESET}"
  exit 1
elif [ "$warn" -gt 0 ]; then
  echo -e "  \033[93m${BOLD}⚠   CONDITIONAL GO — check warnings.${RESET}"
  exit 0
else
  echo -e "  \033[92m${BOLD}✅  GO.${RESET}"
  exit 0
fi
