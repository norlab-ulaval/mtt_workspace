#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/replay_bag.sh <session_dir|bag_dir|session.mcap|bag_0.mcap> [docker compose args...]
  ./scripts/replay_bag.sh <session_dir|bag_dir|session.mcap|bag_0.mcap> mathis [docker compose up options...]

Examples:
  ./scripts/replay_bag.sh /path/to/session
  ./scripts/replay_bag.sh /path/to/session up rviz
  ./scripts/replay_bag.sh /path/to/session/session.mcap --profile localization up
  ./scripts/replay_bag.sh /path/to/session mathis
  ./scripts/replay_bag.sh /path/to/session mathis --build
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

# The containers run install/, not src/: a stale binary silently ignores every
# C++ change. install/ mtimes are unreliable (CMake preserves old timestamps on
# copy), so compare source vs the BUILD binary mtime, and install vs build by
# content.
MAPPER_BUILD_BIN="${WORKSPACE_ROOT}/build/norlab_icp_mapper_ros/mapper_node"
MAPPER_INSTALL_BIN="${WORKSPACE_ROOT}/install/norlab_icp_mapper_ros/lib/norlab_icp_mapper_ros/mapper_node"
MAPPER_SRC="${WORKSPACE_ROOT}/src/external/norlab_icp_mapper_ros/src"
if [ -x "$MAPPER_BUILD_BIN" ] && [ -d "$MAPPER_SRC" ]; then
  NEWER_SRC="$(find "$MAPPER_SRC" \( -name '*.cpp' -o -name '*.h' \) -newer "$MAPPER_BUILD_BIN" -print -quit)"
  if [ -n "$NEWER_SRC" ]; then
    echo "WARNING: mapper source newer than last build:" >&2
    echo "  $NEWER_SRC" >&2
    echo "  Rebuild before trusting this replay: docker compose run --rm compile" >&2
  elif [ -x "$MAPPER_INSTALL_BIN" ] && ! cmp -s "$MAPPER_BUILD_BIN" "$MAPPER_INSTALL_BIN"; then
    echo "WARNING: install/ mapper binary differs from build/ — rerun:" >&2
    echo "  docker compose run --rm compile" >&2
  fi
fi

cd "$REPLAY_DIR"
if [ $# -gt 0 ] && [ "$1" = "mathis" ]; then
  shift
  exec env BAG_PATH="$BAG_DIR" docker compose --profile mathis_mapping up "$@" \
    bag_player description imu_odom_mathis mapping_mathis foxglove
fi

if [ $# -eq 0 ]; then
  exec env BAG_PATH="$BAG_DIR" docker compose up
else
  exec env BAG_PATH="$BAG_DIR" docker compose "$@"
fi
