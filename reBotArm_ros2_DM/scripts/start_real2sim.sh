#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/source_rebotarm_env.sh"

REAL2SIM_MODEL="${REAL2SIM_MODEL:-colored}"
REAL2SIM_MODEL_PATH="${REAL2SIM_MODEL_PATH:-}"
REAL2SIM_JOINT_MAP_FILE="${REAL2SIM_JOINT_MAP_FILE:-}"
REAL2SIM_JOINT_STATE_TOPIC="${REAL2SIM_JOINT_STATE_TOPIC:-/rebotarm/joint_states}"
REAL2SIM_OPEN_VIEWER="${REAL2SIM_OPEN_VIEWER:-true}"
REAL2SIM_SYNC_HZ="${REAL2SIM_SYNC_HZ:-60.0}"
REAL2SIM_SMOOTHING_ALPHA="${REAL2SIM_SMOOTHING_ALPHA:-1.0}"
REAL2SIM_STALE_TIMEOUT="${REAL2SIM_STALE_TIMEOUT:-1.0}"

if [[ "${REAL2SIM_MODEL}" == "kinematic" || "${REAL2SIM_MODEL}" == "stl" || "${REAL2SIM_MODEL}" == "colored" || "${REAL2SIM_MODEL}" == "legacy" ]]; then
  package_prefix="$(ros2 pkg prefix rebotarm_mujoco)"
  if [[ "${REAL2SIM_MODEL}" == "colored" || "${REAL2SIM_MODEL}" == "stl" ]]; then
    REAL2SIM_MODEL_PATH="${REAL2SIM_MODEL_PATH:-${package_prefix}/share/rebotarm_mujoco/models/rebotarm_b601_colored.xml}"
  elif [[ "${REAL2SIM_MODEL}" == "legacy" ]]; then
    REAL2SIM_MODEL_PATH="${REAL2SIM_MODEL_PATH:-${package_prefix}/share/rebotarm_mujoco/models/rebotarm_b601_stl.xml}"
  else
    REAL2SIM_MODEL_PATH="${REAL2SIM_MODEL_PATH:-${package_prefix}/share/rebotarm_mujoco/models/rebotarm_b601_kinematic.xml}"
  fi
  REAL2SIM_JOINT_MAP_FILE="${REAL2SIM_JOINT_MAP_FILE:-${package_prefix}/share/rebotarm_mujoco/config/joint_map_kinematic.yaml}"
fi

launch_args=(
  "joint_state_topic:=${REAL2SIM_JOINT_STATE_TOPIC}"
  "open_viewer:=${REAL2SIM_OPEN_VIEWER}"
  "sync_hz:=${REAL2SIM_SYNC_HZ}"
  "smoothing_alpha:=${REAL2SIM_SMOOTHING_ALPHA}"
  "stale_timeout:=${REAL2SIM_STALE_TIMEOUT}"
)

if [[ -n "${REAL2SIM_MODEL_PATH}" ]]; then
  launch_args+=("model_path:=${REAL2SIM_MODEL_PATH}")
fi

if [[ -n "${REAL2SIM_JOINT_MAP_FILE}" ]]; then
  launch_args+=("joint_map_file:=${REAL2SIM_JOINT_MAP_FILE}")
fi

exec ros2 launch rebotarm_mujoco real2sim.launch.py "${launch_args[@]}" "$@"
