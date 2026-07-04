#!/usr/bin/env bash
set -euo pipefail

template="${MTT_ZENOH_CONFIG_TEMPLATE:-/config/zenoh_session.template.json5}"

# Auto-detect field/Doodle network mode.
# When FIELD_NETWORK_MODE=true, override the Zenoh endpoint to the Doodle IP
# so that every service (monitor, teleop_pc, bash, etc.) uses the Doodle link.
if [ "${FIELD_NETWORK_MODE:-false}" = "true" ] && [ -n "${DOODLE_ROBOT_IP:-}" ]; then
  doodle_port="${ZENOH_FIELD_PORT:-7447}"
  export ROBOT_ZENOH_ENDPOINT="tcp/${DOODLE_ROBOT_IP}:${doodle_port}"
  echo "[zenoh] FIELD_NETWORK_MODE=true — endpoint overridden to tcp/${DOODLE_ROBOT_IP}:${doodle_port}"
fi

zenoh_endpoint="${ROBOT_ZENOH_ENDPOINT:-${ZENOH_ROUTER_ENDPOINT:-tcp/192.168.2.2:7447}}"
live_domain_id="${LIVE_ROBOT_DOMAIN_ID:-${ROS_DOMAIN_ID:-2}}"

if [[ ! -f "${template}" ]]; then
  echo >&2 "[zenoh] missing template: ${template}"
  exit 1
fi

if [[ "${zenoh_endpoint}" != tcp/*:* ]]; then
  echo >&2 "[zenoh] expected tcp/<host>:<port>, got '${zenoh_endpoint}'"
  exit 1
fi

endpoint_payload="${zenoh_endpoint#tcp/}"
robot_host="${endpoint_payload%:*}"
robot_zenoh_port="${endpoint_payload##*:}"

mkdir -p "${HOME}/.ros"

safe_host="${robot_host//[^A-Za-z0-9_.-]/_}"
rendered_config="${HOME}/.ros/zenoh_session_${safe_host}_${robot_zenoh_port}.json5"

sed \
  -e "s|__ROBOT_HOST__|${robot_host}|g" \
  -e "s|__ROBOT_ZENOH_PORT__|${robot_zenoh_port}|g" \
  "${template}" > "${rendered_config}"

export ROS_DOMAIN_ID="${live_domain_id}"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Keep both names for compatibility, but use the same variable family as
# norlab_robot on the robot side.
export ZENOH_SESSION_CONFIG_URI="${rendered_config}"
export RMW_ZENOH_CONFIG_FILE="${rendered_config}"

echo "[zenoh] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[zenoh] router=tcp/${robot_host}:${robot_zenoh_port}"

exec "$@"
