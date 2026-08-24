#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/source_rebotarm_env.sh"

PYTHON_EXECUTABLE="${REBOTARM_VENV:-${REBOTARM_ROS2_WORKSPACE}/.venv}/bin/python3"

if [[ ! -x "${PYTHON_EXECUTABLE}" ]]; then
  echo "[rebotarm-text-agent] Python not found: ${PYTHON_EXECUTABLE}" >&2
  exit 1
fi

exec "${PYTHON_EXECUTABLE}" -m rebotarm_agent.rebotarm_text_agent "$@"
