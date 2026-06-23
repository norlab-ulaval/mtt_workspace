#!/usr/bin/env bash
# ============================================================
# run_mapping_rate_test.sh
#
# Runs 3 sequential ICP mapping passes on the calibration bag
# at rates 1.0, 0.5 and 0.25 to compare quality vs speed.
#
# Usage:
#   ./scripts/run_mapping_rate_test.sh [bag_path]
#
# Default bag_path:
#   ${WORKSPACE}/data/mtt_calibration_test_garage_2026-06-02_09-01-53
#
# Outputs (per run):
#   <bag>/mapping_rate_1p0/map.vtk
#   <bag>/mapping_rate_1p0/trajectory.vtk
#   <bag>/mapping_rate_0p5/map.vtk
#   <bag>/mapping_rate_0p5/trajectory.vtk
#   <bag>/mapping_rate_0p25/map.vtk
#   <bag>/mapping_rate_0p25/trajectory.vtk
# ============================================================
set -uo pipefail

WORKSPACE=/home/mohamed/Documents/Project_MTT/Workspace/mtt_workspace

BAG_PATH="${1:-${WORKSPACE}/data/mtt_calibration_test_garage_2026-06-02_09-01-53}"
BAG_DURATION_S=2170   # from ros2 bag info (36.2 min)

COMPOSE_DIR="${WORKSPACE}/demos/bag_replay"
SERVICES="bag_player description runtime_odometry joint_state_builder mapping"

LOG_FILE="${BAG_PATH}/mapping_rate_test_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "${BAG_PATH}"

log() {
    echo "[$(date '+%H:%M:%S')] $*" | tee -a "${LOG_FILE}"
}

run_mapping() {
    local rate="$1"
    local label="$2"           # e.g. "1p0", "0p5", "0p25"
    local outdir="${BAG_PATH}/mapping_rate_${label}"

    # Stop slightly before the loop restarts (bag_duration/rate - 90s margin).
    # The mapper destructor saves on SIGINT.
    local wait_s
    wait_s=$(python3 -c "import math; w=int(math.ceil(${BAG_DURATION_S}/${rate}))-90; print(max(60,w))")

    log "============================================================"
    log "Run: rate=${rate}x  wait=${wait_s}s (~$(( wait_s/60 )) min)  out=${outdir}"
    log "============================================================"

    mkdir -p "${outdir}"

    cd "${COMPOSE_DIR}"

    # Clean up any leftover containers from a previous run.
    BAG_PATH="${BAG_PATH}" REPLAY_RATE="${rate}" docker compose down --timeout 10 2>/dev/null || true
    sleep 3

    log "Starting stack..."
    BAG_PATH="${BAG_PATH}" REPLAY_RATE="${rate}" \
        docker compose up -d ${SERVICES} 2>&1 | tee -a "${LOG_FILE}"

    log "Waiting ${wait_s}s for bag to finish one pass..."
    sleep "${wait_s}"

    log "Stopping stack (SIGINT → mapper destructor saves map)..."
    BAG_PATH="${BAG_PATH}" REPLAY_RATE="${rate}" \
        docker compose down --timeout 90 2>&1 | tee -a "${LOG_FILE}"

    # Give the filesystem a moment to flush.
    sleep 5

    # The mapper saves to WORKSPACE/map.vtk and WORKSPACE/trajectory.vtk
    if [ -f "${WORKSPACE}/map.vtk" ]; then
        cp "${WORKSPACE}/map.vtk"        "${outdir}/map.vtk"
        log "Saved map:        ${outdir}/map.vtk"
    else
        log "WARNING: ${WORKSPACE}/map.vtk not found — mapper may not have saved"
    fi

    if [ -f "${WORKSPACE}/trajectory.vtk" ]; then
        cp "${WORKSPACE}/trajectory.vtk" "${outdir}/trajectory.vtk"
        log "Saved trajectory: ${outdir}/trajectory.vtk"
    else
        log "WARNING: ${WORKSPACE}/trajectory.vtk not found"
    fi

    # Keep the originals for the next run (they will be overwritten).
    log "Run rate=${rate}x done."
    echo ""
    sleep 10  # cooldown before next run
}

log "Starting mapping rate test"
log "Bag:  ${BAG_PATH}"
log "Runs: 1.0x (~35 min)  0.5x (~72 min)  0.25x (~143 min)"
log "Total estimated: ~4.2 hours"
log "Log:  ${LOG_FILE}"
echo ""

run_mapping "1.0"  "1p0"
run_mapping "0.5"  "0p5"
run_mapping "0.25" "0p25"

log "============================================================"
log "All 3 runs complete."
log "Results:"
log "  ${BAG_PATH}/mapping_rate_1p0/"
log "  ${BAG_PATH}/mapping_rate_0p5/"
log "  ${BAG_PATH}/mapping_rate_0p25/"
log "Open in Foxglove by loading each map.vtk or replaying the bag"
log "and checking /mapping/aligned_scan vs /mapping/map."
log "============================================================"
