#!/usr/bin/env bash
set -Eeuo pipefail

# One-key ROS2 startup for reBotArm. Press Ctrl+C to stop every launch safely.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." >/dev/null 2>&1 && pwd)"

SERIAL_CHANNEL="${SERIAL_CHANNEL:-/dev/ttyACM0}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
USE_RVIZ="${USE_RVIZ:-true}"

PIDS=()
NAMES=()
SHUTTING_DOWN=0

log() {
  printf '[rebotarm-start] %s\n' "$*"
}

source_ros_environment() {
  local env_script="${SCRIPT_DIR}/source_rebotarm_env.sh"

  if [[ -f "${env_script}" ]]; then
    # shellcheck source=/dev/null
    source "${env_script}"
  else
    log "environment helper not found: ${env_script}"
  fi

  if ! command -v ros2 >/dev/null 2>&1; then
    log "ros2 command not found. Source your ROS2 environment first, or build/source this workspace."
    exit 1
  fi
}

start_launch() {
  local name="$1"
  shift

  log "starting ${name}: $*"
  "$@" &
  PIDS+=("$!")
  NAMES+=("${name}")
}

is_running() {
  local pid="$1"
  kill -0 "${pid}" >/dev/null 2>&1
}

stop_all() {
  local reason="${1:-shutdown}"

  if [[ "${SHUTTING_DOWN}" -eq 1 ]]; then
    return
  fi
  SHUTTING_DOWN=1

  log "${reason}; sending SIGINT to ROS launch processes..."
  for pid in "${PIDS[@]}"; do
    if is_running "${pid}"; then
      kill -INT "${pid}" >/dev/null 2>&1 || true
    fi
  done

  local deadline=$((SECONDS + 12))
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
      log "process ${pid} did not stop after SIGINT; sending SIGTERM..."
      kill -TERM "${pid}" >/dev/null 2>&1 || true
    fi
  done

  wait "${PIDS[@]}" >/dev/null 2>&1 || true
  log "all launch processes stopped."
}

trap 'stop_all "Ctrl+C received"; exit 130' INT
trap 'stop_all "termination signal received"; exit 143' TERM
trap 'stop_all "script exiting"' EXIT

source_ros_environment

start_launch "fake bringup" \
  ros2 launch rebotarm_bringup fake_bringup.launch.py

# rosbridge is owned by `./rebotarm start web`; this script only reuses it.
if ss -ltn "sport = :${ROSBRIDGE_PORT}" 2>/dev/null | grep -q LISTEN; then
  log "rosbridge port ${ROSBRIDGE_PORT} is already listening; reusing it."
else
  log "WARNING: rosbridge port ${ROSBRIDGE_PORT} is not listening."
  log "Run './rebotarm start web' first so the web UI can talk to ROS."
fi

start_launch "hardware bringup" \
  ros2 launch rebotarm_bringup bringup.launch.py \
    channel:="${SERIAL_CHANNEL}" \
    use_rviz:="${USE_RVIZ}"

log "startup complete."
log "Ctrl+C will safely stop fake bringup, hardware bringup, and RViz."

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
