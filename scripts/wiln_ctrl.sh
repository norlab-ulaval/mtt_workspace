#!/bin/bash
# Helper around the live WILN services.
# Prefer the Compose one-shot services in demos/data_collection and demos/live_robot.
# Keep runtime tuning in the demo YAML files, not here.
#
# Usage:
#   bash scripts/wiln_ctrl.sh teach_start
#   bash scripts/wiln_ctrl.sh teach_stop
#   bash scripts/wiln_ctrl.sh save   [/path/to/route.ltr]
#   bash scripts/wiln_ctrl.sh load   [/path/to/route.ltr]
#   bash scripts/wiln_ctrl.sh replay
#   bash scripts/wiln_ctrl.sh replay_loop <n_loops>
#   bash scripts/wiln_ctrl.sh stop
#   bash scripts/wiln_ctrl.sh clear

set -euo pipefail

BOLD="\033[1m"; GREEN="\033[92m"; RED="\033[91m"; YELLOW="\033[93m"; RESET="\033[0m"

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "$0")/.." && pwd)}"
DEFAULT_LTR="${WILN_LTR_DIR:-${WORKSPACE}/data}/route.ltr"
export ROS_DOMAIN_ID="${WILN_ROS_DOMAIN_ID:-2}"
export RMW_IMPLEMENTATION="${WILN_RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ZENOH_ROUTER_CONFIG_URI="${ZENOH_ROUTER_CONFIG_URI:-${WORKSPACE}/src/external/norlab_robot/scripts/config/zenoh_router.json5}"
export ZENOH_SESSION_CONFIG_URI="${ZENOH_SESSION_CONFIG_URI:-${WORKSPACE}/src/external/norlab_robot/scripts/config/zenoh_session.json5}"
export RMW_ZENOH_CONFIG_FILE="${RMW_ZENOH_CONFIG_FILE:-${WORKSPACE}/src/external/norlab_robot/scripts/config/zenoh_session.json5}"

_svc() {
    local service="$1" type="$2" args="${3:-}"
    if [ -z "$args" ]; then
        args='{}'
    fi
    echo -e "${BOLD}→ ros2 service call $service $type $args${RESET}"
    ros2 service call "$service" "$type" "$args"
}

_wait_for_wiln() {
    local timeout_s="${1:-8}"
    local i
    for i in $(seq 1 "$timeout_s"); do
        if ros2 service list 2>/dev/null | grep -qx "/start_recording"; then
            return 0
        fi
        sleep 1
    done
    return 1
}

_check_wiln() {
    if ! _wait_for_wiln 8; then
        echo -e "${RED}✗ WILN services not found.${RESET}"
        echo "  Start WILN first: dc up --profile wiln"
        exit 1
    fi
}

_topic_present() {
    ros2 topic list 2>/dev/null | grep -qx "$1"
}

CMD="${1:-help}"

case "$CMD" in
    teach_start|start)
        _check_wiln
        echo -e "${GREEN}Starting teach phase — drive the route now.${RESET}"
        _svc /mtt_repeat/teach_start std_srvs/srv/Trigger
        ;;

    teach_stop|stop_teach)
        _check_wiln
        echo -e "${YELLOW}Stopping teach — saving trajectory to memory.${RESET}"
        _svc /mtt_repeat/teach_stop std_srvs/srv/Trigger
        ;;

    save)
        _check_wiln
        LTR="${2:-$DEFAULT_LTR}"
        mkdir -p "$(dirname "$LTR")"
        echo -e "Saving map+trajectory → ${BOLD}$LTR${RESET}"
        _svc /save_map_traj wiln/srv/SaveMapTraj "{file_name: {data: \"$LTR\"}}"
        ;;

    load)
        _check_wiln
        LTR="${2:-$DEFAULT_LTR}"
        if [ ! -f "$LTR" ]; then
            echo -e "${RED}✗ File not found: $LTR${RESET}"
            exit 1
        fi
        echo -e "Loading map+trajectory ← ${BOLD}$LTR${RESET}"
        _svc /load_map_traj wiln/srv/LoadMapTraj "{file_name: {data: \"$LTR\"}}"
        ;;

    ready)
        _check_wiln
        echo -e "${GREEN}Marking loaded trajectory ready for replay.${RESET}"
        _svc /mtt_repeat/mark_ready std_srvs/srv/Trigger
        ;;

    replay|play)
        _check_wiln
        echo -e "${GREEN}Starting repeat phase — robot will follow the trajectory.${RESET}"
        _svc /mtt_repeat/play_line std_srvs/srv/Trigger
        ;;

    replay_loop|loop)
        _check_wiln
        N="${2:-1}"
        echo -e "${GREEN}Starting loop repeat — ${N} loops.${RESET}"
        _svc /mtt_repeat/play_loop wiln/srv/PlayLoop "{nb_loops: {data: $N}}"
        ;;

    stop|cancel)
        _check_wiln
        echo -e "${YELLOW}Cancelling trajectory.${RESET}"
        _svc /mtt_repeat/cancel std_srvs/srv/Trigger
        ;;

    clear)
        _check_wiln
        echo -e "${YELLOW}Clearing trajectory from memory.${RESET}"
        _svc /clear_trajectory std_srvs/srv/Empty
        ;;

    status)
        _wait_for_wiln 5 || true
        echo -e "${BOLD}WILN service availability:${RESET}"
        for svc in /start_recording /stop_recording /play_line /cancel_trajectory /save_map_traj /load_map_traj /play_loop /mtt_repeat/teach_start /mtt_repeat/teach_stop /mtt_repeat/play_line /mtt_repeat/cancel; do
            if ros2 service list 2>/dev/null | grep -q "$svc"; then
                echo -e "  ${GREEN}✓${RESET}  $svc"
            else
                echo -e "  ${RED}✗${RESET}  $svc"
            fi
        done
        echo ""
        echo -e "${BOLD}Topics / action:${RESET}"
        for topic in /mapping/icp_odom /mtt_health /mtt_repeat/state /mtt_repeat/ready /cmd_vel/manual /controller/cmd_vel /mtt_control/selected_source; do
            if _topic_present "$topic"; then
                echo -e "  ${GREEN}✓${RESET}  $topic"
            else
                echo -e "  ${RED}✗${RESET}  $topic"
            fi
        done
        if ros2 action list 2>/dev/null | grep -qx "/follow_path"; then
            echo -e "  ${GREEN}✓${RESET}  /follow_path"
        else
            echo -e "  ${RED}✗${RESET}  /follow_path"
        fi
        echo ""
        echo -e "${BOLD}Latched repeat state:${RESET}"
        timeout 2s ros2 topic echo --once /mtt_repeat/state 2>/dev/null || echo "  /mtt_repeat/state unavailable"
        timeout 2s ros2 topic echo --once /mtt_repeat/ready 2>/dev/null || echo "  /mtt_repeat/ready unavailable"
        ;;

    help|*)
        echo ""
        echo -e "${BOLD}wiln_ctrl.sh — WILN service helper${RESET}"
        echo ""
        echo "  teach_start           Start recording a new route"
        echo "  teach_stop            Stop recording"
        echo "  save   [file.ltr]     Save map+trajectory to disk (default: $DEFAULT_LTR)"
        echo "  load   [file.ltr]     Load map+trajectory from disk"
        echo "  ready                 Mark a loaded trajectory ready for replay"
        echo "  replay                Start autonomous repeat"
        echo "  replay_loop <N>       Repeat N times"
        echo "  stop                  Cancel current repeat"
        echo "  clear                 Clear trajectory from memory"
        echo "  status                Check if WILN services are available"
        echo ""
        echo "  Fresh route workflow:"
        echo "    1. dc up robot                   (mapping must be running)"
        echo "    2. dc up --profile wiln          (in one terminal)"
        echo "    2. bash scripts/wiln_ctrl.sh teach_start"
        echo "       ... drive the route ..."
        echo "    3. bash scripts/wiln_ctrl.sh teach_stop"
        echo "    4. bash scripts/wiln_ctrl.sh replay"
        echo "    5. bash scripts/wiln_ctrl.sh save /path/to/route.ltr   (optional)"
        echo ""
        echo "  Existing route workflow:"
        echo "    1. dc up robot"
        echo "    2. dc up --profile wiln"
        echo "    3. bash scripts/wiln_ctrl.sh load /path/to/route.ltr"
        echo "    4. bash scripts/wiln_ctrl.sh replay"
        echo ""
        echo "  Notes:"
        echo "    - Prefer the Compose one-shot services when available."
        echo "    - Do not use 'ros2 launch wiln' directly for the MTT stack."
        echo "    - You do need live /mapping/icp_odom before WILN can teach or replay."
        echo ""
        ;;
esac
