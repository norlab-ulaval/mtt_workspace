#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname -- "${BASH_SOURCE[0]}")/.."

repo_root="$(pwd)"
ros_distro="${ROS_DISTRO:-jazzy}"
with_ros=false
skip_rosdep=false
skip_vcs=false
run_build=false
force=false
dry_run=false
check_only=false

print_usage() {
  cat <<'EOF'
Usage: ./scripts/install_host.sh [options]

Purpose:
  Prepare a host machine to build this workspace without Docker.

Options:
  --with-ros        install ROS 2 (ros-<distro>-desktop) if missing
  --check           verify requirements only (no installs)
  --dry-run         print commands without executing
  --skip-rosdep     skip rosdep init/update/install
  --skip-vcs        skip workspace import (create_ws)
  --build           run colcon build at the end
  --force           continue even if Ubuntu version is unexpected
  -h, --help        show this help

Notes:
  - This script targets Ubuntu 24.04 for ROS 2 Jazzy.
  - It uses apt for installs. You need sudo access.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-ros)
      with_ros=true
      shift
      ;;
    --check)
      check_only=true
      shift
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    --skip-rosdep)
      skip_rosdep=true
      shift
      ;;
    --skip-vcs)
      skip_vcs=true
      shift
      ;;
    --build)
      run_build=true
      shift
      ;;
    --force)
      force=true
      shift
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      print_usage >&2
      exit 1
      ;;
  esac
done

log() {
  echo "$@"
}

warn() {
  echo "$@" >&2
}

run_cmd() {
  if [[ "${dry_run}" == true ]]; then
    echo "[dry-run] $*"
    return 0
  fi
  "$@"
}

check_cmd() {
  local name="$1"
  if ! command -v "${name}" >/dev/null 2>&1; then
    warn "[install_host] missing command: ${name}"
    return 1
  fi
  return 0
}

check_path() {
  local label="$1"
  local path="$2"
  if [[ ! -e "${path}" ]]; then
    warn "[install_host] missing ${label}: ${path}"
    return 1
  fi
  return 0
}

check_lib() {
  local label="$1"
  local pattern="$2"
  if command -v ldconfig >/dev/null 2>&1; then
    if ! ldconfig -p 2>/dev/null | grep -q "${pattern}"; then
      warn "[install_host] missing ${label} (${pattern})"
      return 1
    fi
  else
    warn "[install_host] ldconfig not available; cannot verify ${label}"
  fi
  return 0
}

if [[ -f /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
else
  warn "[install_host] /etc/os-release not found"
  exit 1
fi

if [[ "${ID:-}" != "ubuntu" ]]; then
  warn "[install_host] This script targets Ubuntu. Detected: ${ID:-unknown}"
  if [[ "${force}" != true ]]; then
    exit 1
  fi
fi

if [[ "${VERSION_ID:-}" != "24.04" ]]; then
  warn "[install_host] Expected Ubuntu 24.04 for ROS 2 Jazzy. Detected: ${VERSION_ID:-unknown}"
  if [[ "${force}" != true ]]; then
    warn "[install_host] Use --force to continue anyway."
    exit 1
  fi
fi

base_packages=(
  git
  curl
  gnupg
  lsb-release
  python3-pip
  python3-argcomplete
  python3-vcstool
  python3-rosdep
  python3-colcon-common-extensions
)

missing=0
check_cmd git || missing=1
check_cmd curl || missing=1
check_cmd python3 || missing=1
check_cmd vcs || missing=1
check_cmd rosdep || missing=1
check_cmd colcon || missing=1

check_path "ROS setup" "/opt/ros/${ros_distro}/setup.bash" || true

# Heavy runtime dependencies that are not covered by rosdep.
# These are installed in the Docker base image and may require vendor SDKs.
check_path "ZED SDK" "/usr/local/zed" || missing=1
check_path "ZED tools" "/usr/local/zed/tools/ZED_Explorer" || true
check_path "depthai headers" "/usr/local/include/depthai/depthai.hpp" || missing=1
check_lib "libpointmatcher" "libpointmatcher" || missing=1
check_lib "libnabo" "libnabo" || missing=1

if [[ "${check_only}" == true ]]; then
  if [[ "${missing}" -ne 0 ]]; then
    warn "[install_host] missing requirements detected"
    exit 1
  fi
  log "[install_host] all checks passed"
  exit 0
fi

if ! command -v sudo >/dev/null 2>&1; then
  warn "[install_host] sudo is required"
  exit 1
fi

run_cmd sudo apt-get update
run_cmd sudo apt-get install -y --no-install-recommends "${base_packages[@]}"

if [[ "${with_ros}" == true ]]; then
  if [[ ! -d "/opt/ros/${ros_distro}" ]]; then
    run_cmd sudo apt-get install -y --no-install-recommends software-properties-common
    run_cmd sudo add-apt-repository universe
    run_cmd sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      | run_cmd sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    run_cmd sudo apt-get update
    run_cmd sudo apt-get install -y --no-install-recommends "ros-${ros_distro}-desktop"
  fi
fi

if [[ "${skip_vcs}" != true ]]; then
  if [[ ! -d "${repo_root}/src/mtt_core" || ! -d "${repo_root}/src/external" ]]; then
    ./scripts/create_ws
  fi
fi

if [[ "${skip_rosdep}" != true ]]; then
  run_cmd sudo rosdep init 2>/dev/null || true
  run_cmd rosdep update
  run_cmd rosdep install -iyr --from-paths src/mtt_core src/external --ignore-src
fi

if [[ "${run_build}" == true ]]; then
  if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
    warn "[install_host] Missing /opt/ros/${ros_distro}/setup.bash"
    exit 1
  fi
  # shellcheck disable=SC1091
  source "/opt/ros/${ros_distro}/setup.bash"
  run_cmd colcon build --base-paths src/mtt_core src/external
fi

cat <<EOF
[install_host] Done.

Next steps:
  source /opt/ros/${ros_distro}/setup.bash
  colcon build --base-paths src/mtt_core src/external
EOF
