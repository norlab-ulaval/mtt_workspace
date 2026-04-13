#!/bin/bash
set -euo pipefail

cd /workspaces/mtt_workspace
DEVCONTAINER_HOME="${HOME:-/home/mtt}"
DEVCONTAINER_COLCON_ROOT="${DEVCONTAINER_HOME}/.cache/mtt_workspace_devcontainer"
mkdir -p "${DEVCONTAINER_COLCON_ROOT}/build" "${DEVCONTAINER_COLCON_ROOT}/install" "${DEVCONTAINER_COLCON_ROOT}/log"
mapfile -t workspace_paths < <(bash ./scripts/workspace_source_paths)
colcon --log-base "${DEVCONTAINER_COLCON_ROOT}/log" build \
  --base-paths "${workspace_paths[@]}" \
  --build-base "${DEVCONTAINER_COLCON_ROOT}/build" \
  --install-base "${DEVCONTAINER_COLCON_ROOT}/install" \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
