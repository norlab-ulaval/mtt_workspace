#!/usr/bin/env bash
# wait_for_mapping_ready.sh — TF / odometry readiness gate for ICP mapper startup.
#
# Called from live_robot.launch.py as a pre-condition before starting the mapper.
# Sleeps for the minimum delay, then polls until the odom->base_footprint chain is
# alive (proxy: /mtt_odometry topic has at least one publisher).
#
# Always exits 0 — mapping starts regardless; the gate only delays it until TF is
# ready or until the poll timeout expires.
#
# Usage: wait_for_mapping_ready.sh <min_delay_s> [poll_timeout_s]
#   min_delay_s    : minimum sleep before polling (same as old mapping_delay_seconds)
#   poll_timeout_s : additional time to poll after min_delay (default 20)
#
# The total maximum wait is min_delay_s + poll_timeout_s.

set -euo pipefail

MIN_DELAY_S="${1:-10}"
POLL_TIMEOUT_S="${2:-20}"

echo "[mapping_ready] minimum wait ${MIN_DELAY_S}s ..."
sleep "${MIN_DELAY_S}"

echo "[mapping_ready] polling for /mtt_odometry publisher (proxy for odom->base_footprint TF) ..."
DEADLINE=$(( $(date +%s) + POLL_TIMEOUT_S ))

while true; do
  NOW=$(date +%s)
  if [[ ${NOW} -ge ${DEADLINE} ]]; then
    echo "[mapping_ready] poll timeout after ${POLL_TIMEOUT_S}s — starting mapper anyway"
    echo "[mapping_ready] WARNING: mapper may start before odom->base_footprint TF is ready"
    echo "[mapping_ready] Run: docker compose run --rm audit_tf  to diagnose startup race"
    exit 0
  fi

  # Check if /mtt_odometry has any publisher in the ROS graph.
  # ros2 topic info exits 0 even if the topic doesn't exist, so we check the output.
  PUB_COUNT=$(ros2 topic info /mtt_odometry --verbose 2>/dev/null \
    | grep -c "Publisher count:" || true)
  if [[ "${PUB_COUNT}" -gt 0 ]]; then
    PUB_LINE=$(ros2 topic info /mtt_odometry 2>/dev/null | grep "Publisher count:" || echo "")
    COUNT_VAL=$(echo "${PUB_LINE}" | grep -oP '\d+' || echo "0")
    if [[ "${COUNT_VAL}" -gt 0 ]]; then
      ELAPSED=$(( $(date +%s) - ( DEADLINE - POLL_TIMEOUT_S ) ))
      echo "[mapping_ready] /mtt_odometry publisher detected (waited ${ELAPSED}s after min_delay)"
      echo "[mapping_ready] odom->base_footprint TF should be available — starting mapper"
      exit 0
    fi
  fi

  sleep 1
done
