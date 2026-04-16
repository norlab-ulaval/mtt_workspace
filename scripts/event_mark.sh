#!/usr/bin/env bash
# event_mark.sh — publish a timestamped annotation into the running bag.
#
# Call this from INSIDE the container (e.g. in a second dcr bash tab) during
# a recording session to mark the start/end of manoeuvres, terrain changes, etc.
#
# Usage:
#   bash scripts/event_mark.sh "straight line on hard snow — start"
#   bash scripts/event_mark.sh "sharp turn with trailer — boue"
#   bash scripts/event_mark.sh "brake test from 0.6 m/s"
#
# The annotation lands in /session/events (std_msgs/String) and is recorded
# in the bag alongside all sensor data. Each message is prefixed with ISO timestamp.
#
# Inside the container it can be aliased for speed:
#   alias mark='bash ${WORKSPACE}/scripts/event_mark.sh'
#   mark "event description"

set -uo pipefail

if [[ $# -eq 0 ]]; then
  echo "Usage: bash scripts/event_mark.sh \"<event description>\""
  exit 1
fi

ANNOTATION="$(date -Iseconds) | $*"

# Source ROS if not already in PATH
if ! command -v ros2 &>/dev/null; then
  ROS_DISTRO="${ROS_DISTRO:-jazzy}"
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

echo "  📌 Marking event: ${ANNOTATION}"
ros2 topic pub --once /session/events std_msgs/msg/String \
  "data: '${ANNOTATION}'" \
  --qos-reliability reliable \
  --qos-durability transient_local \
  > /dev/null 2>&1

echo "  ✓ Annotation published to /session/events"
