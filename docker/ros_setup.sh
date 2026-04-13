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
  "${WORKSPACE}/src/mtt_core/mtt_description/models" \
  "${WORKSPACE}/src/mtt_core/mtt_description/worlds" \
  "${WORKSPACE}/gazebo/models" \
  "${WORKSPACE}/gazebo/worlds"; do
  if [[ -d "${candidate}" ]]; then
    resource_paths+=("${candidate}")
  fi
done

if [[ ${#resource_paths[@]} -gt 0 ]]; then
  export GZ_SIM_RESOURCE_PATH="$(IFS=:; echo "${resource_paths[*]}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}")"
fi

# Ensure interactive bash sessions (dcr bash, docker exec bash) auto-source ROS.
# Written once to ~/.bashrc; survives for the container lifetime without rebuilding.
if [[ -f "${HOME}/.bashrc" ]] && ! grep -q 'opt/ros' "${HOME}/.bashrc" 2>/dev/null; then
    cat >> "${HOME}/.bashrc" << 'BASHRC_EOF'

# ROS auto-source — added by ros_setup.sh (tab completion, ros2 CLI)
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
[ -r "${WORKSPACE}/install/local_setup.bash" ] && source "${WORKSPACE}/install/local_setup.bash"
BASHRC_EOF
fi

if [[ $# -eq 0 ]]; then
  exec bash
fi

exec "$@"
