#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -r "${WORKSPACE}/install/local_setup.bash" ]]; then
  source "${WORKSPACE}/install/local_setup.bash"
fi

export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {name}: {message}"

if [[ -d "${WORKSPACE}/mtt_description/models" ]]; then
  export GZ_SIM_RESOURCE_PATH="${WORKSPACE}/mtt_description/models:${GZ_SIM_RESOURCE_PATH}"
fi

if [[ $# -eq 0 ]]; then
  exec bash
fi

exec "$@"
