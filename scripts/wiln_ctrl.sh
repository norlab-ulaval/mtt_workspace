#!/bin/bash
# wiln_ctrl.sh — WILN teach-and-repeat service controller.
#
# Wraps ROS2 service calls for the common WILN operations.
# Run this inside the Docker container (via 'dcr bash wiln_ctrl.sh <cmd>')
# or from a shell that has ROS2 sourced.
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

_svc() {
    local service="$1" type="$2" args="${3:-'{}'}"
    echo -e "${BOLD}→ ros2 service call $service $type $args${RESET}"
    ros2 service call "$service" "$type" "$args"
}

_check_wiln() {
    if ! ros2 service list 2>/dev/null | grep -q "/start_recording"; then
        echo -e "${RED}✗ WILN services not found.${RESET}"
        echo "  Start WILN first: dc up --profile wiln"
        exit 1
    fi
}

CMD="${1:-help}"

case "$CMD" in
    teach_start|start)
        _check_wiln
        echo -e "${GREEN}Starting teach phase — drive the route now.${RESET}"
        _svc /start_recording std_srvs/srv/Trigger
        ;;

    teach_stop|stop_teach)
        _check_wiln
        echo -e "${YELLOW}Stopping teach — saving trajectory to memory.${RESET}"
        _svc /stop_recording std_srvs/srv/Trigger
        ;;

    save)
        _check_wiln
        LTR="${2:-$DEFAULT_LTR}"
        mkdir -p "$(dirname "$LTR")"
        echo -e "Saving map+trajectory → ${BOLD}$LTR${RESET}"
        _svc /save_ltr wiln_msgs/srv/SaveLtr "{file_name: \"$LTR\"}"
        ;;

    load)
        _check_wiln
        LTR="${2:-$DEFAULT_LTR}"
        if [ ! -f "$LTR" ]; then
            echo -e "${RED}✗ File not found: $LTR${RESET}"
            exit 1
        fi
        echo -e "Loading map+trajectory ← ${BOLD}$LTR${RESET}"
        _svc /load_ltr wiln_msgs/srv/LoadLtr "{file_name: \"$LTR\"}"
        ;;

    replay|play)
        _check_wiln
        echo -e "${GREEN}Starting repeat phase — robot will follow the trajectory.${RESET}"
        _svc /play_trajectory std_srvs/srv/Trigger
        ;;

    replay_loop|loop)
        _check_wiln
        N="${2:-1}"
        echo -e "${GREEN}Starting loop repeat — ${N} loops.${RESET}"
        _svc /play_loop_trajectory wiln_msgs/srv/PlayLoopTrajectory "{nbLoops: $N}"
        ;;

    stop|cancel)
        _check_wiln
        echo -e "${YELLOW}Cancelling trajectory.${RESET}"
        _svc /cancel_trajectory std_srvs/srv/Trigger
        ;;

    clear)
        _check_wiln
        echo -e "${YELLOW}Clearing trajectory from memory.${RESET}"
        _svc /clear_trajectory std_srvs/srv/Trigger
        ;;

    status)
        echo -e "${BOLD}WILN service availability:${RESET}"
        for svc in /start_recording /stop_recording /play_trajectory /cancel_trajectory /save_ltr /load_ltr; do
            if ros2 service list 2>/dev/null | grep -q "$svc"; then
                echo -e "  ${GREEN}✓${RESET}  $svc"
            else
                echo -e "  ${RED}✗${RESET}  $svc"
            fi
        done
        ;;

    help|*)
        echo ""
        echo -e "${BOLD}wiln_ctrl.sh — WILN teach-and-repeat controller${RESET}"
        echo ""
        echo "  teach_start           Start recording a new route"
        echo "  teach_stop            Stop recording"
        echo "  save   [file.ltr]     Save map+trajectory to disk (default: $DEFAULT_LTR)"
        echo "  load   [file.ltr]     Load map+trajectory from disk"
        echo "  replay                Start autonomous repeat"
        echo "  replay_loop <N>       Repeat N times"
        echo "  stop                  Cancel current repeat"
        echo "  clear                 Clear trajectory from memory"
        echo "  status                Check if WILN services are available"
        echo ""
        echo "  Typical workflow:"
        echo "    1. dc up --profile wiln          (in one terminal)"
        echo "    2. bash scripts/wiln_ctrl.sh teach_start"
        echo "       ... drive the route ..."
        echo "    3. bash scripts/wiln_ctrl.sh teach_stop"
        echo "    4. bash scripts/wiln_ctrl.sh save /path/to/route.ltr"
        echo "    5. bash scripts/wiln_ctrl.sh load /path/to/route.ltr  (optional, if reloading)"
        echo "    6. bash scripts/wiln_ctrl.sh replay"
        echo ""
        ;;
esac
