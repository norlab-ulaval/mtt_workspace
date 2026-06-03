#!/usr/bin/env bash
set -euo pipefail

MODE="load"
CREATE_DIR="false"
UPDATE_LATEST="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2:?missing mode}"
      shift 2
      ;;
    --create-dir)
      CREATE_DIR="true"
      shift
      ;;
    --update-latest)
      UPDATE_LATEST="true"
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

WORKSPACE_DIR="${WORKSPACE:-/home/mohamed/Documents/Project_MTT/Workspace/mtt_workspace}"
ROUTES_ROOT="${WILN_ROUTES_ROOT:-${WORKSPACE_DIR}/data/wiln_routes}"
mkdir -p "${ROUTES_ROOT}"

sanitize_route_name() {
  local raw="$1"
  raw="${raw#/}"
  raw="${raw%/}"
  if [[ -z "${raw}" || "${raw}" == *".."* || "${raw}" == */* ]]; then
    echo "invalid route name '${raw}'" >&2
    exit 2
  fi
  printf '%s' "${raw}"
}

if [[ -n "${WILN_LTR_DIR:-}" ]]; then
  ROUTE_DIR="${WILN_LTR_DIR}"
  ROUTE_NAME="${ROUTE:-$(basename "${ROUTE_DIR}")}"
elif [[ -n "${ROUTE:-}" ]]; then
  ROUTE_NAME="$(sanitize_route_name "${ROUTE}")"
  ROUTE_DIR="${ROUTES_ROOT}/${ROUTE_NAME}"
elif [[ "${MODE}" == "save" ]]; then
  ROUTE_NAME="${ROUTE_PREFIX:-route}_$(date +%Y%m%d_%H%M%S)"
  ROUTE_DIR="${ROUTES_ROOT}/${ROUTE_NAME}"
elif [[ -e "${ROUTES_ROOT}/latest" ]]; then
  ROUTE_DIR="$(readlink -f "${ROUTES_ROOT}/latest")"
  ROUTE_NAME="$(basename "${ROUTE_DIR}")"
else
  ROUTE_NAME="garage_1559"
  ROUTE_DIR="${ROUTES_ROOT}/${ROUTE_NAME}"
fi

ROUTE_NAME="$(sanitize_route_name "${ROUTE_NAME}")"
ROUTE_FILE="${ROUTE_DIR}/route.ltr"

if [[ "${CREATE_DIR}" == "true" ]]; then
  mkdir -p "${ROUTE_DIR}"
fi

if [[ "${UPDATE_LATEST}" == "true" ]]; then
  ln -sfn "${ROUTE_DIR}" "${ROUTES_ROOT}/latest"
fi

printf 'export ROUTE_NAME=%q\n' "${ROUTE_NAME}"
printf 'export WILN_ROUTES_ROOT=%q\n' "${ROUTES_ROOT}"
printf 'export WILN_LTR_DIR=%q\n' "${ROUTE_DIR}"
printf 'export ROUTE_FILE=%q\n' "${ROUTE_FILE}"
