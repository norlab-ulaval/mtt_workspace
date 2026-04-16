#!/usr/bin/env bash
# annotate_session.sh — interactive session annotation
#
# Prompts the operator for experiment context, then writes:
#   demos/data_collection/.env   — picked up automatically by docker compose
#   /tmp/mtt_session_info.txt    — human-readable summary printed before launch
#
# Usage:
#   bash scripts/annotate_session.sh
#   bash scripts/annotate_session.sh --noninteractive  # uses env vars as-is, no prompts
#
# After running this, start the session:
#   cd demos/data_collection && dc up
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE="${WORKSPACE:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
ENV_FILE="${WORKSPACE}/demos/data_collection/.env"
SUMMARY_FILE="/tmp/mtt_session_info.txt"

BOLD="\033[1m"; CYAN="\033[96m"; GREEN="\033[92m"; YELLOW="\033[93m"; RESET="\033[0m"

non_interactive=false
for arg in "$@"; do
  [[ "$arg" == "--noninteractive" ]] && non_interactive=true
done

# ── Prompt helper ─────────────────────────────────────────────────────────────
prompt() {
  local var="$1" prompt_text="$2" default="$3"
  if $non_interactive; then
    eval "$var=\"\${$var:-$default}\""
    return
  fi
  local current; current=$(eval "echo \"\${$var:-}\"")
  local show_default="${current:-$default}"
  printf "  %s [%s]: " "$prompt_text" "$show_default"
  read -r input
  eval "$var=\"\${input:-$show_default}\""
}

choose() {
  local var="$1" prompt_text="$2" default="$3"; shift 3
  local options=("$@")
  if $non_interactive; then
    eval "$var=\"\${$var:-$default}\""
    return
  fi
  echo -e "  ${BOLD}$prompt_text${RESET}"
  for i in "${!options[@]}"; do
    printf "    %d) %s\n" "$((i+1))" "${options[$i]}"
  done
  printf "  Choice [%s]: " "$default"
  read -r choice
  if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le "${#options[@]}" ]; then
    eval "$var=\"${options[$((choice-1))]}\""
  else
    eval "$var=\"\${$var:-$default}\""
  fi
}

# ── Load existing .env as defaults ────────────────────────────────────────────
if [[ -f "$ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  set -a; source "$ENV_FILE" 2>/dev/null || true; set +a
fi

echo ""
echo -e "${BOLD}${CYAN}══════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}${CYAN}   MTT Session Annotation${RESET}"
echo -e "${BOLD}${CYAN}══════════════════════════════════════════════════${RESET}"
echo ""

# ── Collect fields ────────────────────────────────────────────────────────────

# Experiment name — becomes part of the bag directory name
prompt EXPERIMENT_NAME "Experiment name (slug, e.g. straight_snow_v1)" "exp_$(date +%H%M)"

# Session type
choose SESSION_TYPE "Session type:" "motion_model" \
  "motion_model" "calibration" "nominal" "edge_case" "trailer_test" "localization"

# Terrain
choose TERRAIN "Terrain:" "neige_dure" \
  "neige_dure" "sloche" "boue" "mixte" "herbe" "gravier" "asphalte"

# Trailer
choose TRAILER_ATTACHED "Trailer attached?" "false" "false" "true"

# Weather
choose WEATHER "Weather:" "ensoleille" \
  "ensoleille" "nuageux" "neige_legere" "neige_forte" "pluie" "brouillard" "vent"

# Temperature
prompt TEMPERATURE "Temperature (°C)" "0"

# Operator
prompt OPERATOR "Operator name" "${USER:-unknown}"

# Notes — free text
if ! $non_interactive; then
  printf "  Notes / observations (Enter to skip): "
  read -r NOTES
  NOTES="${NOTES:-}"
fi
NOTES="${NOTES:-}"

# ── Write .env — only update session vars, preserve everything else ───────────
mkdir -p "$(dirname "$ENV_FILE")"

# Set or update a single KEY=VALUE in the .env file.
# - If the key exists: update it in-place (preserves position and other vars).
# - If it doesn't exist yet: append it under the session metadata section.
set_env_var() {
  local key="$1" value="$2"
  if grep -q "^${key}=" "$ENV_FILE" 2>/dev/null; then
    sed -i "s|^${key}=.*|${key}=${value}|" "$ENV_FILE"
  else
    printf '%s=%s\n' "${key}" "${value}" >> "$ENV_FILE"
  fi
}

# If no .env exists yet, create a minimal one so set_env_var can append to it.
if [[ ! -f "$ENV_FILE" ]]; then
  printf '# MTT data collection — run annotate_session.sh to populate session metadata\n' > "$ENV_FILE"
fi

# Update only session metadata — infrastructure vars (WORKSPACE, ROBOT_HOST, etc.) are untouched.
set_env_var "EXPERIMENT_NAME"  "${EXPERIMENT_NAME}"
set_env_var "SESSION_TYPE"     "${SESSION_TYPE}"
set_env_var "TERRAIN"          "${TERRAIN}"
set_env_var "TRAILER_ATTACHED" "${TRAILER_ATTACHED}"
set_env_var "WEATHER"          "${WEATHER}"
set_env_var "TEMPERATURE"      "${TEMPERATURE}"
set_env_var "OPERATOR"         "${OPERATOR}"
set_env_var "NOTES"            "${NOTES}"

# ── Write human-readable summary ──────────────────────────────────────────────
cat > "$SUMMARY_FILE" << SUMEOF
MTT Session Summary — $(date '+%Y-%m-%d %H:%M:%S')
══════════════════════════════════════════════════
  Experiment : ${EXPERIMENT_NAME}
  Type       : ${SESSION_TYPE}
  Terrain    : ${TERRAIN}
  Trailer    : ${TRAILER_ATTACHED}
  Weather    : ${WEATHER}
  Temp       : ${TEMPERATURE}°C
  Operator   : ${OPERATOR}
  Notes      : ${NOTES:-—}
══════════════════════════════════════════════════
SUMEOF

echo ""
echo -e "${GREEN}${BOLD}✅  Annotation saved → ${ENV_FILE}${RESET}"
echo ""
cat "$SUMMARY_FILE"
echo ""
echo -e "${BOLD}Session directory will be:${RESET}"
echo "  data/mtt_${SESSION_TYPE}_${EXPERIMENT_NAME}_<YYYY-MM-DD_HH-MM-SS>/"
echo "       ├── session_info.yaml    ← metadata"
echo "       ├── ros_params.yaml      ← ROS node params snapshot"
echo "       ├── topic_list.txt       ← active topics at start"
echo "       └── bag/                 ← mcap bag files"
echo ""
echo -e "${BOLD}Next steps:${RESET}"
echo "  1. bash scripts/pre_session_check.sh"
echo "  2. cd demos/data_collection && dc up"
echo "  3. During recording: bash scripts/event_mark.sh \"<event description>\""
echo "  4. After recording:  python3 scripts/post_session_report.py data/<session_dir>"
echo ""
