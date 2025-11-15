#!/usr/bin/env bash
set -Eeuo pipefail

# Sweep symmetric Y bounds for the crop/bounding box and summarize detections.
# - Varies min_y/max_y over ±10, ±8, ±6, ±4, ±3 (meters)
# - Uses benchmark.launch.py and writes per-run logs + a CSV summary.
# - Each run uses run_seconds=10 by default

# Default bag path (override by passing as $1)
DEFAULT_BAG="/media/ingrid/Extreme_Pro/2025-10-15-Messdaten/2025-10-15-VK-OL-004/2025-10-15-VK-OL-004_0.db3"
BAG_PATH="${1:-${DEFAULT_BAG}}"

# Validate bag path; fallback to directory with metadata.yaml if given a .db3 file
if [[ ! -e "${BAG_PATH}" ]]; then
  dir="$(dirname -- "${BAG_PATH}")"
  if [[ -f "${dir}/metadata.yaml" ]]; then
    echo "[WARN] Bag path not found: ${BAG_PATH}. Using directory: ${dir}" >&2
    BAG_PATH="${dir}"
  else
    echo "[ERROR] Bag path does not exist: ${BAG_PATH}" >&2
    exit 2
  fi
else
  if [[ -f "${BAG_PATH}" && "${BAG_PATH}" == *.db3 ]]; then
    dir="$(dirname -- "${BAG_PATH}")"
    if [[ -f "${dir}/metadata.yaml" ]]; then
      echo "[INFO] Switching bag path to directory containing metadata.yaml: ${dir}" >&2
      BAG_PATH="${dir}"
    fi
  fi
fi

RATE="${RATE:-1}"
RUN_SECS="${RUN_SECS:-10}"
# Start playback at 145s offset by default (override via env START_OFFSET)
START_OFFSET="${START_OFFSET:-145}"
RMW_IMPL="${RMW_IMPL:-}"

# Optional RMW argument
RMW_ARG=()
if [[ -n "${RMW_IMPL}" ]]; then
  RMW_ARG+=("rmw:=${RMW_IMPL}")
fi

# Fixed algorithm parameters for this sweep (override via env)
VOXEL_SIZE="${VOXEL_SIZE:-0.10}"
DISTANCE_THRESHOLD="${DISTANCE_THRESHOLD:-0.15}"
CLUSTER_TOLERANCE="${CLUSTER_TOLERANCE:-0.50}"

# Y half-widths in meters (symmetric min_y=-Y, max_y=+Y)
YS=(10 8 6 4 3)

CSV="bbox-y-vs-bench.csv"
echo "y_abs,real_s,exit_code,pkw,mensch,fahrradfahrer,lkw,total" > "${CSV}"

for y in "${YS[@]}"; do
  MIN_Y="-$y"
  MAX_Y="$y"
  LOG="result-bboxy-${y}.log"

  {
    echo "=== Start bbox y=±${y} ==="
    date -Is
    echo "cmd: ros2 launch ouster_cpp benchmark.launch.py bag=\"${BAG_PATH}\" rate=${RATE} start_offset=${START_OFFSET} run_seconds=${RUN_SECS} voxel_size=${VOXEL_SIZE} distance_threshold=${DISTANCE_THRESHOLD} cluster_tolerance=${CLUSTER_TOLERANCE} min_y=${MIN_Y} max_y=${MAX_Y} ${RMW_ARG[*]:-}"

    LC_ALL=C LANG=C time -p \
      ros2 launch ouster_cpp benchmark.launch.py \
        bag:="${BAG_PATH}" \
        rate:="${RATE}" \
        start_offset:="${START_OFFSET}" \
        run_seconds:="${RUN_SECS}" \
        voxel_size:="${VOXEL_SIZE}" \
        distance_threshold:="${DISTANCE_THRESHOLD}" \
        cluster_tolerance:="${CLUSTER_TOLERANCE}" \
        min_y:="${MIN_Y}" \
        max_y:="${MAX_Y}" \
        "${RMW_ARG[@]}"

    RC=$?
    echo "exit_code: ${RC}"
    date -Is
    echo "=== Done bbox y=±${y} ==="
  } &> "${LOG}" || true

  # Extract metrics from log
  pkw=$(grep -c "Pkw erkannt" "${LOG}" 2>/dev/null || true)
  mensch=$(grep -c "Mensch erkannt" "${LOG}" 2>/dev/null || true)
  fahr=$(grep -c "Fahrradfahrer erkannt" "${LOG}" 2>/dev/null || true)
  lkw=$(grep -c "Lkw erkannt" "${LOG}" 2>/dev/null || true)
  total=$(( pkw + mensch + fahr + lkw ))

  # Timing and exit code
  real_time=$(grep -E "^real[[:space:]]" "${LOG}" | tail -n1 | awk '{print $2}' | tr ',' '.')
  rc=$(awk -F ': ' '/^exit_code:/ {ec=$2} END {print ec+0}' "${LOG}" 2>/dev/null || true)

  echo "${y},${real_time:-0},${rc:-0},${pkw:-0},${mensch:-0},${fahr:-0},${lkw:-0},${total:-0}" >> "${CSV}"

  echo "[INFO] bbox y=±${y} -> pkw=${pkw} mensch=${mensch} fahr=${fahr} lkw=${lkw} total=${total} (real=${real_time}s, rc=${rc})"
done

echo "Résumé écrit dans ${CSV}. Logs conservés (result-bboxy-*.log)."
