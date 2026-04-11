#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  cat <<'EOF' >&2
Usage: demos/live_robot/scripts/robot_snapshot.sh <output_dir> [ssh_target]
EOF
  exit 1
fi

output_dir="$1"
target="${2:-${ROBOT_SSH_TARGET:-robot@192.168.2.2}}"

mkdir -p "${output_dir}"

run_remote() {
  local name="$1"
  local command="$2"

  if ! ssh -o BatchMode=yes "${target}" "${command}" >"${output_dir}/${name}.txt" 2>"${output_dir}/${name}.err"; then
    echo "Could not collect ${name} from ${target}" >&2
  fi
}

run_remote date "date --iso-8601=seconds"
run_remote hostname "hostnamectl || hostname"
run_remote network "ip -br addr && printf '\n' && ip route"
run_remote can0 "ip -details link show can0"
run_remote screens "screen -ls || true"
run_remote sockets "ss -ltnup || true"
run_remote processes "ps -ef | grep -E 'ros2|screen|zenoh|foxglove|mtt' | grep -v grep || true"
