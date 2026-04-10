#!/bin/bash
set -e

cd /workspaces/mtt_tools
DEVCONTAINER_HOME="${HOME:-/home/mtt}"
DEVCONTAINER_COLCON_ROOT="${DEVCONTAINER_HOME}/.cache/mtt_tools_devcontainer"
mkdir -p "${DEVCONTAINER_COLCON_ROOT}/build" "${DEVCONTAINER_COLCON_ROOT}/install" "${DEVCONTAINER_COLCON_ROOT}/log"
colcon --log-base "${DEVCONTAINER_COLCON_ROOT}/log" build \
  --base-paths src \
  --build-base "${DEVCONTAINER_COLCON_ROOT}/build" \
  --install-base "${DEVCONTAINER_COLCON_ROOT}/install" \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
