#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/replay_bag.sh <session_dir|bag_dir|session.mcap|bag_0.mcap> [docker compose args...]

Examples:
  ./scripts/replay_bag.sh /path/to/session
  ./scripts/replay_bag.sh /path/to/session up rviz
  ./scripts/replay_bag.sh /path/to/session/session.mcap --profile localization up
EOF
}

if [ $# -lt 1 ]; then
  usage
  exit 1
fi

INPUT_PATH="$1"
shift || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPLAY_DIR="${WORKSPACE_ROOT}/demos/bag_replay"

resolve_bag_dir() {
  local path="$1"
  if [ -f "$path" ]; then
    dirname "$path"
    return 0
  fi
  if [ -d "$path/bag" ] && [ -f "$path/bag/metadata.yaml" ]; then
    printf '%s\n' "$path/bag"
    return 0
  fi
  if [ -d "$path" ] && [ -f "$path/metadata.yaml" ]; then
    printf '%s\n' "$path"
    return 0
  fi
  return 1
}

if ! BAG_DIR="$(resolve_bag_dir "$INPUT_PATH")"; then
  echo "ERROR: cannot resolve a bag directory from '$INPUT_PATH'" >&2
  usage
  exit 1
fi

cd "$REPLAY_DIR"
if [ $# -eq 0 ]; then
  exec env BAG_PATH="$BAG_DIR" docker compose up
else
  exec env BAG_PATH="$BAG_DIR" docker compose "$@"
fi
