#!/usr/bin/env bash
# Source this file before running reBotArm ROS2 commands.
# It loads ROS2, the local Python virtual environment, and this workspace.

_rebotarm_env_is_sourced() {
  [[ "${BASH_SOURCE[0]}" != "$0" ]]
}

_rebotarm_env_log() {
  printf '[rebotarm-env] %s\n' "$*"
}

_rebotarm_env_fail() {
  _rebotarm_env_log "$*"
  if _rebotarm_env_is_sourced; then
    return 1
  fi
  exit 1
}

_rebotarm_env_source_file() {
  local setup_file="$1"
  local label="$2"
  local had_errexit=0
  local had_nounset=0

  case "$-" in
    *e*) had_errexit=1 ;;
  esac
  case "$-" in
    *u*) had_nounset=1 ;;
  esac

  set +e
  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  local status="$?"
  if [[ "${had_errexit}" -eq 1 ]]; then
    set -e
  else
    set +e
  fi
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  else
    set +u
  fi

  if [[ "${status}" -ne 0 ]]; then
    _rebotarm_env_fail "failed to source ${label}: ${setup_file}"
    return 1
  fi
  _rebotarm_env_log "sourced ${label}: ${setup_file}"
}

_rebotarm_env_prepend_pythonpath() {
  local path="$1"

  case ":${PYTHONPATH:-}:" in
    *":${path}:"*) ;;
    *) export PYTHONPATH="${path}${PYTHONPATH:+:${PYTHONPATH}}" ;;
  esac
}

_REBOTARM_ENV_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
_REBOTARM_ENV_WORKSPACE_DIR="$(cd -- "${_REBOTARM_ENV_SCRIPT_DIR}/.." >/dev/null 2>&1 && pwd)"

export REBOTARM_ROS2_WORKSPACE="${_REBOTARM_ENV_WORKSPACE_DIR}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
REBOTARM_VENV="${REBOTARM_VENV:-${_REBOTARM_ENV_WORKSPACE_DIR}/.venv}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ ! -f "${ROS_SETUP}" ]]; then
    for setup_file in /opt/ros/*/setup.bash; do
      if [[ -f "${setup_file}" ]]; then
        ROS_SETUP="${setup_file}"
        break
      fi
    done
  fi

  if [[ -f "${ROS_SETUP}" ]]; then
    _rebotarm_env_source_file "${ROS_SETUP}" "ROS2" || return 1 2>/dev/null || exit 1
  else
    _rebotarm_env_log "ROS2 setup not found; set ROS_SETUP=/opt/ros/<distro>/setup.bash if needed"
  fi
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [[ -f "${REBOTARM_VENV}/bin/activate" ]]; then
  _rebotarm_env_source_file "${REBOTARM_VENV}/bin/activate" "venv" || return 1 2>/dev/null || exit 1
else
  _rebotarm_env_log "venv not found: ${REBOTARM_VENV}"
fi

_rebotarm_env_site_packages_found=0
for site_packages in "${REBOTARM_VENV}"/lib/python*/site-packages; do
  if [[ -d "${site_packages}" ]]; then
    _rebotarm_env_prepend_pythonpath "${site_packages}"
    _rebotarm_env_site_packages_found=1
    _rebotarm_env_log "PYTHONPATH includes: ${site_packages}"
  fi
done

if [[ "${_rebotarm_env_site_packages_found}" -eq 0 && -d "${REBOTARM_VENV}" ]]; then
  _rebotarm_env_log "no venv site-packages directory found under: ${REBOTARM_VENV}/lib"
fi

# cmeel packages (pin/pinocchio, coal, eigenpy, etc.) install Python modules
# and shared libraries under cmeel.prefix; add them so the system Python used
# by ROS 2 entry-point scripts can import pinocchio and load its C extensions.
for _cmeel_sp in "${REBOTARM_VENV}"/lib/python*/site-packages/cmeel.prefix/lib/python*/site-packages; do
  if [[ -d "${_cmeel_sp}" ]]; then
    _rebotarm_env_prepend_pythonpath "${_cmeel_sp}"
    _rebotarm_env_log "PYTHONPATH includes: ${_cmeel_sp}"
  fi
done
for _cmeel_lib in "${REBOTARM_VENV}"/lib/python*/site-packages/cmeel.prefix/lib; do
  if [[ -d "${_cmeel_lib}" ]]; then
    case ":${LD_LIBRARY_PATH:-}:" in
      *":${_cmeel_lib}:"*) ;;
      *) export LD_LIBRARY_PATH="${_cmeel_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
    esac
    _rebotarm_env_log "LD_LIBRARY_PATH includes: ${_cmeel_lib}"
  fi
done

if [[ -f "${_REBOTARM_ENV_WORKSPACE_DIR}/install/setup.bash" ]]; then
  _rebotarm_env_source_file "${_REBOTARM_ENV_WORKSPACE_DIR}/install/setup.bash" "workspace" || return 1 2>/dev/null || exit 1
else
  _rebotarm_env_log "workspace setup not found; run colcon build first if packages are unavailable"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  _rebotarm_env_fail "ros2 command not found after environment setup"
  return 1 2>/dev/null || exit 1
fi

if ! _rebotarm_env_is_sourced; then
  _rebotarm_env_log "environment ready for child commands"
  _rebotarm_env_log "to keep it in your shell, run: source scripts/source_rebotarm_env.sh"
fi
