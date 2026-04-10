#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -r "${WORKSPACE}/install/local_setup.bash" ]]; then
  source "${WORKSPACE}/install/local_setup.bash"
fi

export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {name}: {message}"

resource_paths=()
for candidate in \
  "${WORKSPACE}/src/mtt_description/models" \
  "${WORKSPACE}/src/mtt_description/worlds" \
  "${WORKSPACE}/gazebo/models" \
  "${WORKSPACE}/gazebo/worlds"; do
  if [[ -d "${candidate}" ]]; then
    resource_paths+=("${candidate}")
  fi
done

if [[ ${#resource_paths[@]} -gt 0 ]]; then
  export GZ_SIM_RESOURCE_PATH="$(IFS=:; echo "${resource_paths[*]}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}")"
fi

if [[ $# -eq 0 ]]; then
  exec bash
fi

exec "$@"
