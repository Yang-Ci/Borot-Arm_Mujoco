#!/usr/bin/env bash
set -Eeuo pipefail

# Start the fake reBotArm driver, MuJoCo viewer, simulation tools, and rosbridge.
# Press Ctrl+C to stop all processes.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/source_rebotarm_env.sh"

ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
ROSBRIDGE_ADDRESS="${ROSBRIDGE_ADDRESS:-0.0.0.0}"
REAL2SIM_MODEL="${REAL2SIM_MODEL:-colored}"
export REAL2SIM_MODEL
MUJOCO_GRASP_MODE="${MUJOCO_GRASP_MODE:-physics}"
MUJOCO_PHYSICS_JOINT_TOPIC="${MUJOCO_PHYSICS_JOINT_TOPIC:-/rebotarm/mujoco/physics_joint_states}"
MUJOCO_OBJECT_STATES_TOPIC="${MUJOCO_OBJECT_STATES_TOPIC:-/rebotarm/mujoco/object_states}"
SIM_IK_TOLERANCE="${SIM_IK_TOLERANCE:-0.020}"
SIM_IK_ORIENTATION_TOLERANCE="${SIM_IK_ORIENTATION_TOLERANCE:-0.120}"
ROSBRIDGE_MANAGED=0

PIDS=()
NAMES=()
SHUTTING_DOWN=0

log() {
  printf '[rebot-mujoco-all] %s\n' "$*"
}

is_running() {
  kill -0 "$1" >/dev/null 2>&1
}

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

port_in_use() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -ltn "sport = :${port}" 2>/dev/null | grep -q LISTEN
    return
  fi
  if command -v lsof >/dev/null 2>&1; then
    lsof -nP -iTCP:"${port}" -sTCP:LISTEN >/dev/null 2>&1
    return
  fi
  return 1
}

detect_host_ip() {
  local ip
  if command -v hostname >/dev/null 2>&1 && hostname -I >/dev/null 2>&1; then
    ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
    [[ -n "${ip}" ]] && { printf '%s' "${ip}"; return 0; }
  fi
  if command -v ip >/dev/null 2>&1; then
    ip="$(ip -4 route get 1 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
    [[ -n "${ip}" ]] && { printf '%s' "${ip}"; return 0; }
  fi
  printf '%s' "${HOST_IP:-}"
}

stop_existing_rebot_stack() {
  local pattern
  # Only stop simulation-owned processes.  rosbridge may belong to
  # `./rebotarm start web`, and robot_state_publisher may belong to the DM
  # hardware stack, so neither is safe to match globally here.
  pattern="fake_bringup.launch.py|fake_rebotarm_driver|real2sim.launch.py|real2sim_sync|mujoco_physics_grasp.launch.py|rebotarm_mujoco_physics_grasp|mujoco_physics_grasp|sim_task_server.launch.py|rebotarm_mujoco_sim_task_server|sim_rgb_camera.launch.py|rebotarm_mujoco_sim_rgb_camera|sim_color_detector.launch.py|rebotarm_mujoco_sim_color_detector|sim_color_detector.py"

  local pids=()
  mapfile -t pids < <(
    pgrep -f "${pattern}" 2>/dev/null | awk -v self="$$" -v parent="${PPID}" '$1 != self && $1 != parent' || true
  )

  if [[ "${#pids[@]}" -eq 0 ]]; then
    return 0
  fi

  log "stopping existing reBot simulation stack: ${pids[*]}"
  for pid in "${pids[@]}"; do
    if is_running "${pid}"; then
      kill -INT "${pid}" >/dev/null 2>&1 || true
    fi
  done

  local deadline=$((SECONDS + 8))
  while (( SECONDS < deadline )); do
    local any_running=0
    for pid in "${pids[@]}"; do
      if is_running "${pid}"; then
        any_running=1
        break
      fi
    done
    [[ "${any_running}" -eq 0 ]] && return 0
    sleep 1
  done

  log "existing stack did not stop cleanly; sending SIGTERM."
  for pid in "${pids[@]}"; do
    if is_running "${pid}"; then
      kill -TERM "${pid}" >/dev/null 2>&1 || true
    fi
  done

  deadline=$((SECONDS + 5))
  while (( SECONDS < deadline )); do
    local any_running=0
    for pid in "${pids[@]}"; do
      if is_running "${pid}"; then
        any_running=1
        break
      fi
    done
    [[ "${any_running}" -eq 0 ]] && return 0
    sleep 1
  done

  local stubborn=()
  for pid in "${pids[@]}"; do
    if is_running "${pid}"; then
      stubborn+=("${pid}")
      kill -KILL "${pid}" >/dev/null 2>&1 || true
    fi
  done
  if [[ "${#stubborn[@]}" -gt 0 ]]; then
    log "force-stopped stale simulation processes: ${stubborn[*]}"
  fi
}

start_process() {
  local name="$1"
  shift

  log "starting ${name}: $*"
  "$@" &
  PIDS+=("$!")
  NAMES+=("${name}")
}

stop_all() {
  local reason="${1:-shutdown}"

  if [[ "${SHUTTING_DOWN}" -eq 1 ]]; then
    return
  fi
  SHUTTING_DOWN=1

  log "${reason}; stopping processes..."
  for pid in "${PIDS[@]}"; do
    if is_running "${pid}"; then
      kill -INT "${pid}" >/dev/null 2>&1 || true
    fi
  done

  local deadline=$((SECONDS + 10))
  while (( SECONDS < deadline )); do
    local any_running=0
    for pid in "${PIDS[@]}"; do
      if is_running "${pid}"; then
        any_running=1
        break
      fi
    done
    [[ "${any_running}" -eq 0 ]] && break
    sleep 1
  done

  for pid in "${PIDS[@]}"; do
    if is_running "${pid}"; then
      kill -TERM "${pid}" >/dev/null 2>&1 || true
    fi
  done

  wait "${PIDS[@]}" >/dev/null 2>&1 || true
  log "stopped."
}

trap 'stop_all "Ctrl+C received"; exit 130' INT
trap 'stop_all "termination signal received"; exit 143' TERM
trap 'stop_all "script exiting"' EXIT

if truthy "${CLEAN_OLD_SIM:-true}"; then
  stop_existing_rebot_stack
fi

start_process "fake bringup" \
  "${SCRIPT_DIR}/start_fake_bringup.sh"

if [[ "${MUJOCO_GRASP_MODE}" == "physics" ]]; then
  start_process "MuJoCo physics grasp" \
    ros2 launch rebotarm_mujoco mujoco_physics_grasp.launch.py \
      sim_joint_state_topic:="${MUJOCO_PHYSICS_JOINT_TOPIC}" \
      object_states_topic:="${MUJOCO_OBJECT_STATES_TOPIC}"
else
  start_process "MuJoCo STL real2sim" \
    "${SCRIPT_DIR}/start_real2sim.sh"
fi

start_process "MuJoCo sim task server" \
  ros2 launch rebotarm_mujoco sim_task_server.launch.py \
    ik_tolerance:="${SIM_IK_TOLERANCE}" \
    ik_orientation_tolerance:="${SIM_IK_ORIENTATION_TOLERANCE}"

if [[ "${MUJOCO_GRASP_MODE}" == "physics" ]]; then
  start_process "MuJoCo overhead RGB camera" \
    ros2 launch rebotarm_mujoco sim_rgb_camera.launch.py \
      joint_state_topic:="${MUJOCO_PHYSICS_JOINT_TOPIC}" \
      object_states_topic:="${MUJOCO_OBJECT_STATES_TOPIC}" \
      virtual_grasp_enabled:=false
else
  start_process "MuJoCo overhead RGB camera" \
    ros2 launch rebotarm_mujoco sim_rgb_camera.launch.py
fi

start_process "MuJoCo color detector" \
  ros2 launch rebotarm_mujoco sim_color_detector.launch.py

if port_in_use "${ROSBRIDGE_PORT}"; then
  log "rosbridge port ${ROSBRIDGE_PORT} is already listening; reusing it."
else
  start_process "rosbridge websocket" \
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
      port:="${ROSBRIDGE_PORT}" \
      address:="${ROSBRIDGE_ADDRESS}"
  ROSBRIDGE_MANAGED=1
fi

log "startup complete."
HOST_IP="${HOST_IP:-$(detect_host_ip)}"
log "web URL: ws://${HOST_IP}:${ROSBRIDGE_PORT}"
log "MuJoCo grasp mode: ${MUJOCO_GRASP_MODE}"
log "IK tolerance: ${SIM_IK_TOLERANCE} m, orientation tolerance: ${SIM_IK_ORIENTATION_TOLERANCE}"
log "RGB camera topic: /rebotarm/mujoco/overhead_rgb/image_raw"
log "Color detections topic: /rebotarm/vision/color_blocks/detections"
if [[ "${ROSBRIDGE_MANAGED}" -eq 1 ]]; then
  log "Ctrl+C stops fake driver, MuJoCo, camera, vision, and rosbridge."
else
  log "Ctrl+C stops fake driver, MuJoCo, camera, and vision; reused rosbridge stays running."
fi

while true; do
  for index in "${!PIDS[@]}"; do
    pid="${PIDS[${index}]}"
    if ! is_running "${pid}"; then
      status=0
      name="${NAMES[${index}]}"
      wait "${pid}" || status="$?"
      log "${name} exited with status ${status}; stopping the remaining processes."
      stop_all "${name} exited"
      exit "${status}"
    fi
  done
  sleep 1
done
