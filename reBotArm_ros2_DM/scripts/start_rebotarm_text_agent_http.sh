#!/usr/bin/env bash
# 启动 rebotarm_text_agent 的 HTTP 服务模式（供 Web UI 调用）
# 用法：./scripts/start_rebotarm_text_agent_http.sh

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

# shellcheck source=/dev/null
source "${SCRIPT_DIR}/source_rebotarm_env.sh"

PYTHON_EXECUTABLE="${REBOTARM_VENV:-${REBOTARM_ROS2_WORKSPACE}/.venv}/bin/python3"

if [[ ! -x "${PYTHON_EXECUTABLE}" ]]; then
  echo "[rebotarm-text-agent-http] Python not found: ${PYTHON_EXECUTABLE}" >&2
  exit 1
fi

HOST="${REBOTARM_AGENT_HTTP_HOST:-0.0.0.0}"
PORT="${REBOTARM_AGENT_HTTP_PORT:-8082}"

echo "[rebotarm-text-agent-http] MCP=${REBOTARM_MCP_URL:-http://127.0.0.1:8081/mcp}"
echo "[rebotarm-text-agent-http] model=${REBOTARM_LLM_MODEL:-qwen-plus}"
echo "[rebotarm-text-agent-http] base_url=${REBOTARM_LLM_BASE_URL:-https://dashscope.aliyuncs.com/compatible-mode/v1}"

exec "${PYTHON_EXECUTABLE}" -m rebotarm_agent.rebotarm_text_agent \
  --http-server \
  --http-host "${HOST}" \
  --http-port "${PORT}" \
  --yes \
  "$@"
