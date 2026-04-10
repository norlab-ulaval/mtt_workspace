#!/usr/bin/env bash
set -e

if [[ -z "${WORKSPACE}" ]]; then
  echo >&2 "Missing environment variable 'WORKSPACE'"
  exit 1
fi

cd -- "${WORKSPACE}"
mkdir -p "${HOME}/.ros"

exec /ros_setup.sh "$@"
