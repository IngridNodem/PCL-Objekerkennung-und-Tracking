#!/usr/bin/env bash
set -Eeuo pipefail

# Sweep voxel_size and distance_threshold for benchmark.launch.py
# - Each run uses run_seconds=10
# - Logs saved as result-<vs>-<dt>.log in current directory
# - Uses `time -p` to record elapsed time

# Usage:
#   bash ouster_cpp/scripts/run_benchmark_grid.sh /path/to/data.db3
# Optional env vars:
#   RATE=1.5 RUN_SECS=10 RMW_IMPL=rmw_fastrtps_cpp bash ...

BAG_PATH="/media/ingrid/Extreme_Pro/2025-10-15-Messdaten/2025-10-15-VK-OL-005/2025-10-15-VK-OL-005_0.db3"

RATE="${RATE:-1.5}"
# Force 10 seconds per run unless overridden explicitly
RUN_SECS="${RUN_SECS:-10}"
RMW_IMPL="${RMW_IMPL:-}"

# Safety timeout: RUN_SECS + margin
TIMEOUT_SEC=$(( RUN_SECS + 20 ))

# Optional RMW argument
RMW_ARG=()
if [[ -n "${RMW_IMPL}" ]]; then
  RMW_ARG+=("rmw:=${RMW_IMPL}")
fi

VOXELS=(0.10 0.15 0.20 0.25)
DISTS=(0.10 0.15 0.18 0.22)

for vs in "${VOXELS[@]}"; do
  for dt in "${DISTS[@]}"; do
    # Use the literal strings as provided in arrays to avoid printf float portability issues
    VS="${vs}"
    DT="${dt}"
    LOG="result-${VS}-${DT}.log"

    {
      echo "=== Start vs=${VS} dt=${DT} ==="
      date -Is
      echo "cmd: ros2 launch ouster_cpp benchmark.launch.py bag=\"${BAG_PATH}\" rate=${RATE} run_seconds=${RUN_SECS} voxel_size=${VS} distance_threshold=${DT} ${RMW_ARG[*]:-}"

      # Run with safety timeout and measure time; append both stdout/stderr
      time -p timeout "${TIMEOUT_SEC}" \
        ros2 launch ouster_cpp benchmark.launch.py \
          bag:="${BAG_PATH}" \
          rate:="${RATE}" \
          run_seconds:="${RUN_SECS}" \
          voxel_size:="${VS}" \
          distance_threshold:="${DT}" \
          "${RMW_ARG[@]}"

      RC=$?
      echo "exit_code: ${RC}"
      date -Is
      echo "=== Done vs=${VS} dt=${DT} ==="
    } &> "${LOG}" || {
      # Also annotate failures in the log
      echo "[WARN] Execution ended non-zero or timed out for vs=${VS} dt=${DT}. See ${LOG}" >&2
    }

    # Small pause between runs
    sleep 2
  done
done
