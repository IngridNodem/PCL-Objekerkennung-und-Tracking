#!/usr/bin/env bash
set -Eeuo pipefail

# Iterate distance_threshold values for benchmark.launch.py and summarize detections.
# - Each run uses run_seconds=10 by default
# - Per-run logs: result-dt-<dt>.log (kept)
# - Summary CSV: dt-vs-bench.csv
# - Measures elapsed time with `time -p`

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

# Fixed voxel size for this sweep (can override via env VOXEL_SIZE)
VOXEL_SIZE="${VOXEL_SIZE:-0.20}"

# Grid of distance thresholds
DISTS=(0.10 0.15 0.20 0.25)

CSV="dt-vs-bench.csv"
echo "distance_threshold,real_s,exit_code,pkw,mensch,fahrradfahrer,lkw,total" > "${CSV}"

for dt in "${DISTS[@]}"; do
  DT="${dt}"
  LOG="result-dt-${DT}.log"

  {
    echo "=== Start dt=${DT} ==="
    date -Is
    echo "cmd: ros2 launch ouster_cpp benchmark.launch.py bag=\"${BAG_PATH}\" rate=${RATE} start_offset=${START_OFFSET} run_seconds=${RUN_SECS} voxel_size=${VOXEL_SIZE} distance_threshold=${DT} ${RMW_ARG[*]:-}"

    LC_ALL=C LANG=C time -p \
      ros2 launch ouster_cpp benchmark.launch.py \
        bag:="${BAG_PATH}" \
        rate:="${RATE}" \
        start_offset:="${START_OFFSET}" \
        run_seconds:="${RUN_SECS}" \
        voxel_size:="${VOXEL_SIZE}" \
        distance_threshold:="${DT}" \
        "${RMW_ARG[@]}"

    RC=$?
    echo "exit_code: ${RC}"
    date -Is
    echo "=== Done dt=${DT} ==="
  } &> "${LOG}" || true

  # Extract metrics from log
  # Counts per class (match exact phrases from tracking_node output)
  pkw=$(grep -c "Pkw erkannt" "${LOG}" 2>/dev/null || true)
  mensch=$(grep -c "Mensch erkannt" "${LOG}" 2>/dev/null || true)
  fahr=$(grep -c "Fahrradfahrer erkannt" "${LOG}" 2>/dev/null || true)
  lkw=$(grep -c "Lkw erkannt" "${LOG}" 2>/dev/null || true)
  total=$(( pkw + mensch + fahr + lkw ))

  # Extract timing (last 'real' from time -p), normalize decimal comma->dot
  real_time=$(grep -E "^real[[:space:]]" "${LOG}" | tail -n1 | awk '{print $2}' | tr ',' '.' )
  # Extract exit code (last occurrence)
  rc=$(awk -F ': ' '/^exit_code:/ {ec=$2} END {print ec+0}' "${LOG}" 2>/dev/null || true)

  echo "${DT},${real_time:-0},${rc:-0},${pkw:-0},${mensch:-0},${fahr:-0},${lkw:-0},${total:-0}" >> "${CSV}"

  echo "[INFO] dt=${DT} -> pkw=${pkw} mensch=${mensch} fahr=${fahr} lkw=${lkw} total=${total} (real=${real_time}s, rc=${rc})"
done

echo "Résumé écrit dans ${CSV}. Logs conservés (result-dt-*.log)."
