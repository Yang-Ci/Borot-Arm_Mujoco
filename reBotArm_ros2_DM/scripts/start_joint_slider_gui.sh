#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/source_rebotarm_env.sh"

JOINT_SLIDER_TOPIC="${JOINT_SLIDER_TOPIC:-/rebotarm/joint_states}"
JOINT_SLIDER_HZ="${JOINT_SLIDER_HZ:-30.0}"
REBOTARM_NAMESPACE="${REBOTARM_NAMESPACE:-rebotarm}"

exec ros2 launch rebotarm_mujoco joint_slider_gui.launch.py \
  joint_state_topic:="${JOINT_SLIDER_TOPIC}" \
  publish_hz:="${JOINT_SLIDER_HZ}" \
  arm_namespace:="${REBOTARM_NAMESPACE}" \
  "$@"
