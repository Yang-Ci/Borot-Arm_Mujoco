# reBotArm Agent MCP Server

`rebotarm_agent` provides a Model Context Protocol (MCP) tool server for the
existing reBotArm ROS2 API. It is designed as a high-level task layer for LLM,
voice, and web-agent integrations.

The server does not replace `rebotarmcontroller` or `rebotarm_mujoco`. It calls
their existing ROS topics, services, and actions.

## Safety Model

The default mode is locked:

```text
motion_mode:=locked
```

In locked mode, read-only tools and IK checks work, but tools that can move the
arm return a blocked response. To allow motion, start the server with:

```text
motion_mode:=allow
```

Use `motion_mode:=allow` only when you are connected to a safe simulation, fake
driver, or intentionally controlling real hardware.

## Dependencies

Install the Python MCP runtime in the same Python environment used by ROS2:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install fastmcp
```

Then build the package:

```bash
cd ~/reBotArm_ros2_DM
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rebotarm_agent
source install/setup.bash
```

If this is a fresh workspace, build the message and controller packages first or
build the whole workspace.

## Start

Locked mode:

```bash
ros2 launch rebotarm_agent rebotarm_mcp.launch.py
```

The launch file runs the MCP server through:

```text
~/reBot_Arm_Mujoco-DM/reBotArm_ros2_DM/.venv/bin/python3
```

This keeps `fastmcp` inside the workspace virtual environment instead of the
externally managed Ubuntu system Python. If your virtual environment lives
elsewhere, pass `python_executable:=/path/to/python3`.

Simulation motion mode:

```bash
ros2 launch rebotarm_agent rebotarm_mcp.launch.py motion_mode:=allow
```

Custom namespace and port:

```bash
ros2 launch rebotarm_agent rebotarm_mcp.launch.py \
  arm_namespace:=rebotarm \
  host:=127.0.0.1 \
  port:=8081 \
  motion_mode:=locked
```

The default transport is `streamable-http`. Most MCP clients can connect to:

```text
http://127.0.0.1:8081/mcp
```

Opening that URL directly in a browser is not a health page. A `Not Acceptable:
Client must accept text/event-stream` response only means the browser is not an
MCP streamable-http client.

For stdio-based clients, run the executable directly instead of using launch:

```bash
ros2 run rebotarm_agent rebotarm_mcp_server --transport stdio
```

## Tools

Read-only and diagnostic tools:

- `get_robot_status`
- `diagnose_ros`
- `gravity_compensation_status`
- `ik_check`
- `detect_blocks`

Controller tools:

- `enable_robot`
- `disable_robot`
- `gravity_compensation_stop`
- `record_start`
- `record_stop`
- `record_clear`

Motion-locked tools:

- `safe_home`
- `gravity_compensation_start`
- `set_gripper_opening_mm`
- `move_to_pose`
- `move_joints`
- `pick_color`
- `record_replay`

## Example Agent Intents

These are the natural-language commands the MCP layer is meant to support after
you connect it to an LLM or voice frontend:

```text
Check whether ROS is ready.
Is the arm enabled?
Can the arm reach x 0.32, y 0.00, z 0.25?
Open the gripper to 90 mm.
Pick the red block in simulation.
Start recording this demonstration.
Replay the last recorded trajectory.
Return to safe home.
```

## Recommended First Demo

Start the full simulation stack:

```bash
cd ~/reBotArm_ros2_DM
source scripts/source_rebotarm_env.sh
./scripts/start_rebot_mujoco_all.sh
```

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_agent rebotarm_mcp.launch.py motion_mode:=allow
```

Then connect an MCP client and call:

```text
diagnose_ros
detect_blocks
pick_color(color="red")
```

Quick local client smoke test:

```bash
python - <<'PY'
import asyncio
from fastmcp import Client

async def main():
    async with Client("http://127.0.0.1:8081/mcp") as client:
        print([tool.name for tool in await client.list_tools()])
        print(await client.call_tool("diagnose_ros", {}))
        print(await client.call_tool("pick_color", {"color": "red"}))

asyncio.run(main())
PY
```

## Text LLM Agent

`rebotarm_text_agent` is a command-line LLM agent that connects to the MCP
server, exposes the MCP tools to an OpenAI-compatible chat model, and lets you
control the robot with short Chinese or English instructions.

Start the simulation stack in one terminal:

```bash
cd ~/reBotArm_ros2_DM
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
./scripts/start_rebot_mujoco_all.sh
```

Start the MCP server in a second terminal:

```bash
cd ~/reBotArm_ros2_DM
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch rebotarm_agent rebotarm_mcp.launch.py motion_mode:=allow
```

Start the text agent in a third terminal:

```bash
cd ~/reBotArm_ros2_DM
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash

export OPENAI_API_KEY="sk-..."
export REBOTARM_LLM_MODEL="gpt-4.1-mini"
./scripts/start_rebotarm_text_agent.sh
```

For Qwen through Alibaba Cloud Model Studio / DashScope compatible mode:

```bash
export REBOTARM_LLM_BASE_URL="https://dashscope.aliyuncs.com/compatible-mode/v1"
export DASHSCOPE_API_KEY="your-dashscope-api-key"
export REBOTARM_LLM_MODEL="qwen-plus"
./scripts/start_rebotarm_text_agent.sh
```

For a local or gateway-hosted OpenAI-compatible endpoint:

```bash
export REBOTARM_LLM_BASE_URL="http://127.0.0.1:11434/v1"
export REBOTARM_LLM_API_KEY="local"
export REBOTARM_LLM_MODEL="your-model-name"
./scripts/start_rebotarm_text_agent.sh --yes
```

`ros2 run rebotarm_agent rebotarm_text_agent` uses the Python interpreter from
the generated ROS entry point. On Ubuntu 24.04 this can miss packages installed
inside `.venv`, such as `fastmcp`. The helper script always uses the workspace
virtual environment and keeps stdin attached for the interactive prompt.

Useful prompts:

```text
先诊断 ROS 状态。
现在能看到哪些颜色块？
抓取红色方块。
把夹爪打开到 90 毫米。
回到安全位置。
```

By default, the agent asks for confirmation before motion tools. Use `--yes`
only in a safe simulation.

Local commands that do not call the LLM:

```text
/status
/detect
/detect red
/pick red
/gripper 90
```

If natural-language prompts fail with `Temporary failure in name resolution`,
the VM cannot resolve the LLM host. Check DNS/network from the VM:

```bash
getent hosts dashscope.aliyuncs.com
curl -I https://dashscope.aliyuncs.com/compatible-mode/v1
```
