#!/usr/bin/env bash
# sync_bags.sh — pull bag sessions from the robot to the local workstation.
#
# Source : ROBOT_HOST:ROBOT_WORKSPACE/data/   (read from demos/data_collection/.env)
# Dest   : LOCAL_DATA_DIR/                    (default: <workspace>/data/)
#
# Usage:
#   bash scripts/sync_bags.sh                         # sync everything new
#   bash scripts/sync_bags.sh --dry-run               # preview only, nothing copied
#   bash scripts/sync_bags.sh mtt_nominal_exp_*       # sync one or more specific sessions
#   bash scripts/sync_bags.sh --list                  # list sessions on the robot
#
# Environment overrides (take priority over .env):
#   ROBOT_HOST       SSH host / IP of the robot
#   ROBOT_WS         Workspace path on the robot
#   LOCAL_DATA_DIR   Local destination directory

set -uo pipefail

# ── Config ────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$(cd "${SCRIPT_DIR}/.." && pwd)}"

# Load .env so ROBOT_HOST, ROBOT_WORKSPACE, etc. are available as defaults.
ENV_FILE="${WORKSPACE}/demos/data_collection/.env"
if [[ -f "$ENV_FILE" ]]; then
  set -a; source "$ENV_FILE" 2>/dev/null || true; set +a
fi

# Connectivity and targets will be resolved after argument parsing to handle --eth
ROBOT_WS="${ROBOT_WS:-${ROBOT_WORKSPACE:-/home/mohamed/Project/mtt_ws}}"
LOCAL_DATA_DIR="${LOCAL_DATA_DIR:-${WORKSPACE}/data}"

# ── Colors ────────────────────────────────────────────────────────────────────
BOLD="\033[1m"; CYAN="\033[96m"; GREEN="\033[92m"; YELLOW="\033[93m"
RED="\033[91m"; RESET="\033[0m"

# ── Args ──────────────────────────────────────────────────────────────────────
dry_run=false
list_only=false
use_eth=false
use_remote=false
filters=()

for arg in "$@"; do
  case "$arg" in
    --dry-run)   dry_run=true ;;
    --list)      list_only=true ;;
    --eth)       use_eth=true ;;
    --remote)    use_remote=true ;;
    --help|-h)
      echo "Usage: bash scripts/sync_bags.sh [--dry-run] [--list] [--eth] [--remote] [session_pattern...]"
      echo ""
      echo "  --dry-run             Show what would be copied without transferring"
      echo "  --list                List sessions available on the robot"
      echo "  --eth                 Use Ethernet connection (ROBOT_HOST_ETH)"
      echo "  --remote              Use Tailscale remote connection (ROBOT_HOST_REMOTE)"
      echo "  session_pattern       Sync only matching sessions (e.g. mtt_nominal_exp_*)"
      echo ""
      echo "  ROBOT_HOST=${ROBOT_HOST:-192.168.2.2}   (override with env var)"
      echo "  ROBOT_WS=${ROBOT_WS}   (override with env var)"
      exit 0
      ;;
    -*)
      echo -e "${RED}Unknown option: $arg${RESET}" >&2; exit 1 ;;
    *)
      filters+=("$arg") ;;
  esac
done

# ── Connectivity Setup ────────────────────────────────────────────────────────
# ROBOT_HOST and SSH_TARGET: env var > .env > hardcoded fallback
if [[ "${use_remote}" == true ]]; then
  ROBOT_HOST="${ROBOT_HOST_REMOTE:-100.77.55.41}"
  if [[ -n "${ROBOT_SSH_TARGET_REMOTE:-}" ]]; then
    SSH_TARGET="${ROBOT_SSH_TARGET_REMOTE}"
  elif [[ -n "${ROBOT_USER:-}" ]]; then
    SSH_TARGET="${ROBOT_USER}@${ROBOT_HOST}"
  else
    SSH_TARGET="${ROBOT_HOST}"
  fi
elif [[ "${use_eth}" == true ]]; then
  ROBOT_HOST="${ROBOT_HOST_ETH:-192.168.3.102}"
  if [[ -n "${ROBOT_SSH_TARGET_ETH:-}" ]]; then
    SSH_TARGET="${ROBOT_SSH_TARGET_ETH}"
  elif [[ -n "${ROBOT_USER:-}" ]]; then
    SSH_TARGET="${ROBOT_USER}@${ROBOT_HOST}"
  else
    SSH_TARGET="${ROBOT_HOST}"
  fi
else
  ROBOT_HOST="${ROBOT_HOST:-192.168.2.2}"
  if [[ -n "${ROBOT_SSH_TARGET:-}" ]]; then
    SSH_TARGET="${ROBOT_SSH_TARGET}"
  elif [[ -n "${ROBOT_USER:-}" ]]; then
    SSH_TARGET="${ROBOT_USER}@${ROBOT_HOST}"
  else
    SSH_TARGET="${ROBOT_HOST}"
  fi
fi

ROBOT_DATA="${SSH_TARGET}:${ROBOT_WS}/data/"



echo ""
echo -e "${BOLD}${CYAN}══════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}${CYAN}   MTT Bag Sync  (robot → workstation)${RESET}"
echo -e "${BOLD}${CYAN}══════════════════════════════════════════════════${RESET}"
echo ""
echo -e "  Robot :  ${BOLD}${ROBOT_DATA}${RESET}"
echo -e "  Local :  ${BOLD}${LOCAL_DATA_DIR}/${RESET}"
echo ""

# ── Connectivity check ────────────────────────────────────────────────────────
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "${SSH_TARGET}" true 2>/dev/null; then
  echo -e "${RED}✗  Cannot reach ${SSH_TARGET} via SSH.${RESET}"
  echo "   Check: ssh ${SSH_TARGET}  (key auth must be set up)"
  echo "   Or override: ROBOT_SSH_TARGET=user@192.168.2.2 bash scripts/sync_bags.sh"
  exit 1
fi
echo -e "  ${GREEN}✓  SSH connection to ${SSH_TARGET} OK${RESET}"
echo ""

# ── List mode ─────────────────────────────────────────────────────────────────
if $list_only; then
  echo -e "${BOLD}Sessions on robot (${SSH_TARGET}:${ROBOT_WS}/data/):${RESET}"
  echo ""
  # shellcheck disable=SC2029
  ssh "${SSH_TARGET}" "ls -1t ${ROBOT_WS}/data/ 2>/dev/null | grep -v '^\.' | grep -v gitkeep" \
    | while read -r session; do
        size=$(ssh "${SSH_TARGET}" "du -sh '${ROBOT_WS}/data/${session}' 2>/dev/null | cut -f1" 2>/dev/null || echo "?")
        printf "  %-55s %s\n" "$session" "$size"
      done
  echo ""
  exit 0
fi

# ── Build rsync include/exclude rules ─────────────────────────────────────────
mkdir -p "${LOCAL_DATA_DIR}"

rsync_opts=(
  --recursive         # copy directories recursively
  --times             # preserve modification times (crucial for analysis)
  --human-readable    # sizes in KB/MB/GB
  --progress          # per-file progress
  --partial           # resume interrupted transfers (important for large mcap)
  --partial-dir=".rsync_partial"  # store partials in hidden dir
  --info=progress2    # overall progress bar
  --exclude=".gitkeep"
  --exclude=".rsync_partial/"
  --exclude="__pycache__/"
)

if $dry_run; then
  rsync_opts+=(--dry-run)
  echo -e "${YELLOW}${BOLD}DRY RUN — nothing will be copied${RESET}"
  echo ""
fi

# If specific session patterns were given, build include/exclude list
if [ "${#filters[@]}" -gt 0 ]; then
  echo -e "  Filtering to sessions matching: ${BOLD}${filters[*]}${RESET}"
  echo ""
  for pattern in "${filters[@]}"; do
    rsync_opts+=("--include=${pattern}/")
    rsync_opts+=("--include=${pattern}/**")
  done
  rsync_opts+=("--exclude=*")
fi

# ── Run rsync ─────────────────────────────────────────────────────────────────
echo -e "${BOLD}Starting transfer...${RESET}"
echo ""

start_time=$(date +%s)

rsync "${rsync_opts[@]}" \
  "${ROBOT_DATA}" \
  "${LOCAL_DATA_DIR}/" \
  2>&1

exit_code=$?
end_time=$(date +%s)
elapsed=$(( end_time - start_time ))

echo ""
if [ $exit_code -eq 0 ]; then
  echo -e "${GREEN}${BOLD}✅  Sync complete  (${elapsed}s)${RESET}"
  echo ""
  echo -e "  Local data dir: ${LOCAL_DATA_DIR}/"
  echo -e "  Sessions synced:"
  ls -1t "${LOCAL_DATA_DIR}" 2>/dev/null \
    | grep -v '^\.' | grep -v gitkeep \
    | head -10 \
    | while read -r session; do
        size=$(du -sh "${LOCAL_DATA_DIR}/${session}" 2>/dev/null | cut -f1)
        printf "    %-55s %s\n" "$session" "$size"
      done
  echo ""
elif [ $exit_code -eq 23 ] || [ $exit_code -eq 24 ]; then
  # exit 24 = some files vanished during transfer (normal for active recording)
  # exit 23 = partial transfer due to error (e.g. symlinks on exFAT)
  echo -e "${YELLOW}${BOLD}⚠   Sync complete with warnings (some files changed or symlinks ignored).${RESET}"
  echo "   This is normal if the robot is still recording or if saving to an exFAT drive."
  echo ""
  echo -e "  Local data dir: ${LOCAL_DATA_DIR}/"
  echo -e "  Sessions synced:"
  ls -1t "${LOCAL_DATA_DIR}" 2>/dev/null \
    | grep -v '^\.' | grep -v gitkeep \
    | head -10 \
    | while read -r session; do
        size=$(du -sh "${LOCAL_DATA_DIR}/${session}" 2>/dev/null | cut -f1)
        printf "    %-55s %s\n" "$session" "$size"
      done
  echo ""
else
  echo -e "${RED}${BOLD}✗   rsync failed (exit code ${exit_code}).${RESET}"
  exit $exit_code
fi
