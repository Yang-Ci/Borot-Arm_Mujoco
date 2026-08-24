#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/source_rebotarm_env.sh"

exec ros2 launch rebotarm_bringup fake_bringup.launch.py "$@"
