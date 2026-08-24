#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/source_rebotarm_env.sh"

MUJOCO_TORQUE_MODEL="${MUJOCO_TORQUE_MODEL:-colored}"
MUJOCO_TORQUE_MODEL_PATH="${MUJOCO_TORQUE_MODEL_PATH:-}"
MUJOCO_TORQUE_JOINT_MAP_FILE="${MUJOCO_TORQUE_JOINT_MAP_FILE:-}"
MUJOCO_TORQUE_TARGET_TOPIC="${MUJOCO_TORQUE_TARGET_TOPIC:-/rebotarm/mujoco/target_joint_states}"
MUJOCO_TORQUE_COMPARE_TOPIC="${MUJOCO_TORQUE_COMPARE_TOPIC:-/rebotarm/joint_states}"
MUJOCO_TORQUE_SIM_TOPIC="${MUJOCO_TORQUE_SIM_TOPIC:-/rebotarm/mujoco/joint_states}"
MUJOCO_TORQUE_OPEN_VIEWER="${MUJOCO_TORQUE_OPEN_VIEWER:-true}"
MUJOCO_TORQUE_CONTROL_HZ="${MUJOCO_TORQUE_CONTROL_HZ:-500.0}"
MUJOCO_TORQUE_PUBLISH_HZ="${MUJOCO_TORQUE_PUBLISH_HZ:-60.0}"
MUJOCO_TORQUE_COMPARE_LOG_HZ="${MUJOCO_TORQUE_COMPARE_LOG_HZ:-2.0}"
MUJOCO_TORQUE_SDK_COMPARE="${MUJOCO_TORQUE_SDK_COMPARE:-true}"
MUJOCO_TORQUE_LIMIT="${MUJOCO_TORQUE_LIMIT:-18.0}"

if [[ "${MUJOCO_TORQUE_MODEL}" == "kinematic" || "${MUJOCO_TORQUE_MODEL}" == "stl" || "${MUJOCO_TORQUE_MODEL}" == "colored" || "${MUJOCO_TORQUE_MODEL}" == "legacy" ]]; then
  package_prefix="$(ros2 pkg prefix rebotarm_mujoco)"
  if [[ "${MUJOCO_TORQUE_MODEL}" == "colored" || "${MUJOCO_TORQUE_MODEL}" == "stl" ]]; then
    MUJOCO_TORQUE_MODEL_PATH="${MUJOCO_TORQUE_MODEL_PATH:-${package_prefix}/share/rebotarm_mujoco/models/rebotarm_b601_colored.xml}"
  elif [[ "${MUJOCO_TORQUE_MODEL}" == "legacy" ]]; then
    MUJOCO_TORQUE_MODEL_PATH="${MUJOCO_TORQUE_MODEL_PATH:-${package_prefix}/share/rebotarm_mujoco/models/rebotarm_b601_stl.xml}"
  else
    MUJOCO_TORQUE_MODEL_PATH="${MUJOCO_TORQUE_MODEL_PATH:-${package_prefix}/share/rebotarm_mujoco/models/rebotarm_b601_kinematic.xml}"
  fi
  MUJOCO_TORQUE_JOINT_MAP_FILE="${MUJOCO_TORQUE_JOINT_MAP_FILE:-${package_prefix}/share/rebotarm_mujoco/config/joint_map_kinematic.yaml}"
fi

launch_args=(
  "target_joint_state_topic:=${MUJOCO_TORQUE_TARGET_TOPIC}"
  "compare_joint_state_topic:=${MUJOCO_TORQUE_COMPARE_TOPIC}"
  "sim_joint_state_topic:=${MUJOCO_TORQUE_SIM_TOPIC}"
  "open_viewer:=${MUJOCO_TORQUE_OPEN_VIEWER}"
  "control_hz:=${MUJOCO_TORQUE_CONTROL_HZ}"
  "publish_hz:=${MUJOCO_TORQUE_PUBLISH_HZ}"
  "compare_log_hz:=${MUJOCO_TORQUE_COMPARE_LOG_HZ}"
  "sdk_compare_enabled:=${MUJOCO_TORQUE_SDK_COMPARE}"
  "torque_limit:=${MUJOCO_TORQUE_LIMIT}"
)

if [[ -n "${MUJOCO_TORQUE_MODEL_PATH}" ]]; then
  launch_args+=("model_path:=${MUJOCO_TORQUE_MODEL_PATH}")
fi

if [[ -n "${MUJOCO_TORQUE_JOINT_MAP_FILE}" ]]; then
  launch_args+=("joint_map_file:=${MUJOCO_TORQUE_JOINT_MAP_FILE}")
fi

exec ros2 launch rebotarm_mujoco mujoco_torque_control.launch.py "${launch_args[@]}" "$@"
