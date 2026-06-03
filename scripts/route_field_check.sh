#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE:-/home/mohamed/Documents/Project_MTT/Workspace/mtt_workspace}"
eval "$("${WORKSPACE_DIR}/scripts/wiln_route_env.sh" --mode load)"

echo "== WILN Route Field Check =="
echo "route: ${ROUTE_NAME}"
echo "file:  ${ROUTE_FILE}"

if [[ ! -f "${ROUTE_FILE}" ]]; then
  echo "verdict: FAIL route file missing"
  echo "hint: run ROUTE=${ROUTE_NAME} docker compose run --rm wiln_save after teach_stop"
  exit 2
fi

set +e
python3 "${WORKSPACE_DIR}/scripts/validate_wiln_route.py" "${ROUTE_FILE}"
VALIDATE_RC=$?
set -e
if [[ "${VALIDATE_RC}" -ne 0 && "${VALIDATE_RC}" -ne 2 ]]; then
  exit "${VALIDATE_RC}"
fi

set +e
MPLCONFIGDIR=/tmp/matplotlib python3 "${WORKSPACE_DIR}/scripts/preview_wiln_route.py" "${ROUTE_FILE}" --output-dir "${WILN_LTR_DIR}"
PREVIEW_RC=$?
set -e
if [[ "${PREVIEW_RC}" -ne 0 && "${PREVIEW_RC}" -ne 2 ]]; then
  exit "${PREVIEW_RC}"
fi

echo ""
echo "outputs:"
for path in "${WILN_LTR_DIR}/preview.png" "${WILN_LTR_DIR}/preview.yaml" "${WILN_LTR_DIR}/metadata.yaml"; do
  if [[ -f "${path}" ]]; then
    echo "  OK   ${path}"
  else
    echo "  WARN ${path} missing"
  fi
done

# Live pose vs route start check.
# Skipped if /mapping/icp_odom is not available (mapping not running).
echo ""
echo "== Route Start Position Check =="
START_CHECK_RC=0
set +e
python3 "${WORKSPACE_DIR}/scripts/route_start_check.py" \
  --route "${ROUTE_FILE}" \
  --icp-samples 5 \
  --timeout 8
START_CHECK_RC=$?
set -e

if [[ "${VALIDATE_RC}" -eq 0 && "${PREVIEW_RC}" -eq 0 && "${START_CHECK_RC}" -eq 0 ]]; then
  echo ""
  echo "verdict: OK route ready for route_load/route_replay"
  exit 0
fi

echo ""
if [[ "${START_CHECK_RC}" -eq 2 ]]; then
  echo "verdict: WARN/FAIL route geometry OK but live pose check failed"
  echo "  hint: run 'docker compose run --rm route_align' to align route to current map frame"
elif [[ "${START_CHECK_RC}" -eq 1 ]]; then
  echo "verdict: WARN robot is outside start threshold — consider route_align before replay"
else
  echo "verdict: WARN/FAIL inspect route before replay"
fi
exit 2
