#!/usr/bin/env bash
set -euo pipefail

duration="${AUDIT_DURATION:-6}"
interfaces="${AUDIT_INTERFACES:-enp4s0 enp8s0 enp5s0 enp0s31f6}"
heavy_topics="${AUDIT_HEAVY_TOPICS:-/hesai_lidar/points /mapping/map /mapping/aligned_scan /wiln/obstacles /debug/hesai_points_voxel /zed/zed_node/point_cloud/cloud_registered}"
hz_topics="${AUDIT_HZ_TOPICS:-/tf /joint_states /mtt_odometry /mtt_tachometer /mapping/icp_odom /mapping/trajectory_path /mtt_obstacle/hazard_status /wiln/trajectory}"

section() {
  printf '\n========== %s ==========\n' "$1"
}

run() {
  printf '\n$ %s\n' "$*"
  "$@" 2>&1 || true
}

run_timeout() {
  local seconds="$1"
  shift
  printf '\n$ timeout %ss %s\n' "$seconds" "$*"
  timeout "${seconds}s" "$@" 2>&1 || true
}

section "Host"
run hostname
run date -Iseconds

section "Routes and interfaces"
run ip -br addr
run ip route
run ip route get 1.1.1.1
run ip neigh
run nmcli device status
run nmcli connection show

section "Ethernet link state"
for iface in ${interfaces}; do
  if ip link show "${iface}" >/dev/null 2>&1; then
    run ethtool "${iface}"
    run ip -s link show "${iface}"
  else
    printf '\n%s: not present\n' "${iface}"
  fi
done

section "Docker"
run docker ps --format '{{.Names}} {{.Status}} {{.Ports}}'
run docker network ls
if [ -f demos/data_collection/compose.yaml ]; then
  run docker compose -f demos/data_collection/compose.yaml ps
fi
if [ -f demos/live_robot/compose.yaml ]; then
  run docker compose -f demos/live_robot/compose.yaml ps
fi

section "Processes"
run ps -eo pid,pcpu,pmem,comm,args --sort=-pcpu

section "ROS graph"
run_timeout 4 ros2 topic list
run_timeout 4 ros2 node list

section "ROS bandwidth"
for topic in ${heavy_topics}; do
  if timeout 2s ros2 topic info "${topic}" 2>/dev/null | grep -qE "Publisher count: [1-9]"; then
    run_timeout "${duration}" ros2 topic bw "${topic}"
  else
    printf '\n%s: no publisher\n' "${topic}"
  fi
done

section "ROS frequency"
for topic in ${hz_topics}; do
  if timeout 2s ros2 topic info "${topic}" 2>/dev/null | grep -qE "Publisher count: [1-9]"; then
    run_timeout "${duration}" ros2 topic hz "${topic}"
  else
    printf '\n%s: no publisher\n' "${topic}"
  fi
done

section "Traffic monitor hints"
if command -v iftop >/dev/null 2>&1; then
  for iface in ${interfaces}; do
    if ip link show "${iface}" >/dev/null 2>&1; then
      printf 'Interactive: sudo iftop -i %s\n' "${iface}"
    fi
  done
else
  echo "iftop not installed in this environment"
fi

section "Field conclusions to check"
echo "- No default route should use a LiDAR-only interface."
echo "- The Doodle interface should be the only one carrying 192.168.50.0/24."
echo "- /hesai_lidar/points should stay local; remote Foxglove should use /debug/hesai_points_voxel."
echo "- After any route/interface change: ip route get 1.1.1.1 && ping -c 3 1.1.1.1"
