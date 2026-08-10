from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time
import urllib.error
import urllib.request
from typing import Any
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from fastmcp import Client


DEFAULT_MCP_URL = "http://127.0.0.1:8081/mcp"
DEFAULT_BASE_URL = "https://api.openai.com/v1"
DEFAULT_MODEL = "gpt-4.1-mini"
MOTION_TOOLS = {
    "safe_home",
    "gravity_compensation_start",
    "set_gripper_opening_mm",
    "move_to_pose",
    "move_joints",
    "pick_color",
    "record_replay",
}

TOOL_CATEGORIES = {
    "get_robot_status": ("状态与诊断", "#5fa8ff"),
    "diagnose_ros": ("状态与诊断", "#5fa8ff"),
    "enable_robot": ("使能控制", "#77c96b"),
    "disable_robot": ("使能控制", "#77c96b"),
    "safe_home": ("运动控制", "#33d6b0"),
    "move_to_pose": ("运动控制", "#33d6b0"),
    "move_joints": ("运动控制", "#33d6b0"),
    "ik_check": ("运动控制", "#33d6b0"),
    "set_gripper_opening_mm": ("夹爪控制", "#f2a541"),
    "gravity_compensation_status": ("重力补偿", "#a78bfa"),
    "gravity_compensation_start": ("重力补偿", "#a78bfa"),
    "gravity_compensation_stop": ("重力补偿", "#a78bfa"),
    "detect_blocks": ("视觉抓取", "#ef5a4d"),
    "pick_color": ("视觉抓取", "#ef5a4d"),
    "record_start": ("录制回放", "#e879f9"),
    "record_stop": ("录制回放", "#e879f9"),
    "record_replay": ("录制回放", "#e879f9"),
    "record_clear": ("录制回放", "#e879f9"),
}

SYSTEM_PROMPT = """你是 reBotArm 机械臂的智能控制助手。你必须使用 MCP tools 执行用户的指令，而不是解释如何执行。

## 核心规则：
1. **直接执行，不要解释**：用户说"摆姿势"就调用 move_to_pose，说"抓红色"就调用 pick_color，不要输出步骤说明或教程。
2. **使用工具获取真实信息**：用户问状态、问看到什么、问色块位置，必须调用 get_robot_status 或 detect_blocks，禁止编造数据。
3. **参数必须合理**：move_to_pose 的 x 在 [-0.4, 0.4] 之间，y 在 [-0.3, 0.3] 之间，z 在 [0.1, 0.5] 之间。
4. **抓取流程**：明确颜色时直接 pick_color；颜色不明确时先 detect_blocks。
5. **安全第一**：未知的目标或危险操作先询问用户确认。
6. **随机生成**：用户要求"摆姿势"、"动一动"等非具体指令时，每次生成不同的随机坐标，不要重复使用相同的数值。

## 可用工具：
- get_robot_status: 获取机械臂状态
- diagnose_ros: 诊断 ROS 连接
- enable_robot / disable_robot: 启用/禁用机械臂
- safe_home: 回到安全位置
- move_to_pose: 移动到指定位置（x, y, z, roll_deg, pitch_deg, yaw_deg, duration）
- move_joints: 控制关节角度
- set_gripper_opening_mm: 设置夹爪开度（0-90mm）
- detect_blocks: 检测颜色物块
- pick_color: 抓取指定颜色物块
- record_start / record_stop / record_replay: 录制/重放动作

## 回复格式：
- 直接调用工具，输出尽量少的自然语言，最多一句话说明你的操作。
- 不要输出数学公式、方框、代码块或教程。
"""


class ChatCompletionsLLM:
    def __init__(
        self,
        *,
        base_url: str,
        api_key: str,
        model: str,
        timeout_sec: float,
        temperature: float,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.api_key = api_key or "local"
        self.model = model
        self.timeout_sec = max(float(timeout_sec), 5.0)
        self.temperature = float(temperature)

    def complete(
        self,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]],
    ) -> dict[str, Any]:
        payload = {
            "model": self.model,
            "messages": messages,
            "tools": tools,
            "tool_choice": "auto",
            "temperature": self.temperature,
        }
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        request = urllib.request.Request(
            f"{self.base_url}/chat/completions",
            data=body,
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"LLM HTTP {exc.code}: {detail}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"LLM request failed: {exc}") from exc


async def run_repl(args: argparse.Namespace) -> int:
    llm = ChatCompletionsLLM(
        base_url=args.base_url,
        api_key=args.api_key,
        model=args.model,
        timeout_sec=args.timeout_sec,
        temperature=args.temperature,
    )

    async with Client(args.mcp_url) as mcp:
        mcp_tools = await mcp.list_tools()
        tools = [_mcp_tool_to_chat_tool(tool) for tool in mcp_tools]
        tool_names = [tool["function"]["name"] for tool in tools]
        messages: list[dict[str, Any]] = [{"role": "system", "content": SYSTEM_PROMPT}]

        print(f"Connected MCP: {args.mcp_url}")
        print(f"LLM model: {args.model}")
        print(f"Tools: {', '.join(tool_names)}")
        print(
            "输入中文指令；/tools 查看工具，/status 诊断，/detect 查看色块，"
            "/pick red 抓取，/gripper 90 控制夹爪，/reset 清空上下文，/exit 退出。"
            "在本地命令后加 --json 可显示原始数据。"
        )

        while True:
            try:
                user_text = input("\n你 > ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return 0

            if not user_text:
                continue
            if user_text in {"/exit", "/quit", "exit", "quit"}:
                return 0
            if user_text == "/tools":
                print(", ".join(tool_names))
                continue
            if user_text == "/reset":
                messages = [{"role": "system", "content": SYSTEM_PROMPT}]
                print("上下文已清空。")
                continue
            if user_text == "/status":
                result = await mcp.call_tool("diagnose_ros", {})
                print(_compact_json(_mcp_result_to_json(result), limit=args.result_chars))
                continue
            if await _try_local_command(
                mcp,
                user_text,
                confirm_motion=not args.yes,
                result_chars=args.result_chars,
                verbose_tools=args.verbose_tools,
            ):
                continue

            messages.append({"role": "user", "content": user_text})
            await _run_agent_turn(
                llm,
                mcp,
                messages,
                tools,
                confirm_motion=not args.yes,
                max_rounds=args.max_tool_rounds,
                result_chars=args.result_chars,
            )


async def _run_agent_turn(
    llm: ChatCompletionsLLM,
    mcp: Client,
    messages: list[dict[str, Any]],
    tools: list[dict[str, Any]],
    *,
    confirm_motion: bool,
    max_rounds: int,
    result_chars: int,
) -> None:
    for _ in range(max(1, int(max_rounds))):
        try:
            response = llm.complete(messages, tools)
        except Exception as exc:
            print(f"\n助手 > LLM 请求失败：{exc}")
            print("提示：先检查 VM 的 DNS/网络；MCP 本地工具仍可用，例如 /status、/detect、/pick red。")
            messages.append(
                {
                    "role": "assistant",
                    "content": f"LLM request failed: {exc}",
                }
            )
            return
        choice = (response.get("choices") or [{}])[0]
        message = choice.get("message") or {}
        content = message.get("content") or ""
        tool_calls = message.get("tool_calls") or []

        if content:
            print(f"\n助手 > {content}")

        if not tool_calls:
            parsed_calls = _parse_tool_calls_from_text(content)
            if parsed_calls:
                tool_calls = parsed_calls
            else:
                messages.append({"role": "assistant", "content": content})
                return

        assistant_message = {"role": "assistant", "content": content, "tool_calls": tool_calls}
        messages.append(assistant_message)

        for call in tool_calls:
            function = call.get("function") or {}
            name = str(function.get("name") or "")
            arguments = _parse_tool_arguments(function.get("arguments"), name)
            tool_call_id = str(call.get("id") or f"tool-{int(time.time() * 1000)}")

            if not name:
                tool_result = {"ok": False, "message": "LLM emitted a tool call without a name."}
            elif confirm_motion and name in MOTION_TOOLS:
                if not _confirm_tool(name, arguments):
                    tool_result = {
                        "ok": False,
                        "tool": name,
                        "cancelled": True,
                        "message": "User declined this motion tool call.",
                    }
                else:
                    tool_result = await _call_mcp_tool(mcp, name, arguments)
            else:
                tool_result = await _call_mcp_tool(mcp, name, arguments)

            print(f"\n工具 > {name}({_compact_json(arguments, limit=260)})")
            print(f"结果 > {_compact_json(tool_result, limit=result_chars)}")
            messages.append(
                {
                    "role": "tool",
                    "tool_call_id": tool_call_id,
                    "content": _compact_json(tool_result, limit=result_chars),
                }
            )

    print("\n助手 > 工具调用轮次已到上限，我先停在这里。")


async def _try_local_command(
    mcp: Client,
    text: str,
    *,
    confirm_motion: bool,
    result_chars: int,
    verbose_tools: bool,
) -> bool:
    parts = text.split()
    if not parts:
        return False
    command = parts[0].lower()
    show_json = "--json" in parts or verbose_tools
    parts = [part for part in parts if part != "--json"]
    intent = _parse_builtin_intent(text)
    if intent is not None:
        command = intent["command"]
        parts = [command, *intent.get("args", [])]
        show_json = show_json or verbose_tools

    if command == "/detect":
        color = parts[1] if len(parts) > 1 else "auto"
        result = await _call_mcp_tool(mcp, "detect_blocks", {"preferred_color": color})
        print(_summarize_detect_blocks(result))
    elif command == "/pick":
        color = parts[1] if len(parts) > 1 else "auto"
        arguments = {"color": color}
        if confirm_motion and not _confirm_tool("pick_color", arguments):
            result = {"ok": False, "tool": "pick_color", "cancelled": True}
        else:
            result = await _call_mcp_tool(mcp, "pick_color", arguments)
        print(_summarize_pick_color(result))
    elif command == "/gripper":
        if len(parts) < 2:
            print("用法：/gripper 90")
            return True
        try:
            opening_mm = float(parts[1])
        except ValueError:
            print("夹爪开度需要是数字，单位 mm。")
            return True
        arguments = {"opening_mm": opening_mm}
        if confirm_motion and not _confirm_tool("set_gripper_opening_mm", arguments):
            result = {"ok": False, "tool": "set_gripper_opening_mm", "cancelled": True}
        else:
            result = await _call_mcp_tool(mcp, "set_gripper_opening_mm", arguments)
        print(_summarize_gripper(result))
    elif command == "/pose":
        arguments = _generate_random_pose()
        if confirm_motion and not _confirm_tool("move_to_pose", arguments):
            result = {"ok": False, "tool": "move_to_pose", "cancelled": True}
        else:
            result = await _call_mcp_tool_with_retry(mcp, "move_to_pose", arguments, max_retries=3)
        print(f"助手 > 正在摆姿势...")
        print(f"工具 > move_to_pose({_compact_json(arguments, limit=200)})")
        print(_summarize_pose(result))
        if not result.get("ok"):
            print(f"结果 > {_compact_json(result, limit=2000)}")
    else:
        return False

    if show_json:
        print(_compact_json(result, limit=result_chars))
    return True


def _parse_builtin_intent(text: str) -> dict[str, Any] | None:
    normalized = text.strip().lower()
    compact = "".join(normalized.split())
    color = _extract_color(compact)

    if any(
        token in compact
        for token in (
            "看到哪些",
            "看到了哪些",
            "能看到什么",
            "看到什么",
            "有哪些颜色",
            "哪些颜色",
            "色块",
            "检测",
        )
    ):
        return {"command": "/detect", "args": [color] if color else []}

    if any(token in compact for token in ("抓取", "抓一下", "夹取", "拿起", "抓住", "抓", "捡", "pick")):
        return {"command": "/pick", "args": [color or "auto"]}

    if "打开夹爪" in compact or "张开夹爪" in compact:
        opening = _extract_number(compact, default=90.0)
        return {"command": "/gripper", "args": [str(opening)]}

    if "关闭夹爪" in compact or "闭合夹爪" in compact or "夹爪关闭" in compact:
        opening = _extract_number(compact, default=0.0)
        return {"command": "/gripper", "args": [str(opening)]}

    if any(token in compact for token in ("姿势", "pose", "摆个", "动一动", "运动", "move")):
        return {"command": "/pose", "args": []}

    return None


def _extract_color(text: str) -> str | None:
    color_map = {
        "red": ("red", "红", "红色"),
        "blue": ("blue", "蓝", "蓝色"),
        "yellow": ("yellow", "黄", "黄色"),
    }
    for color, tokens in color_map.items():
        if any(token in text for token in tokens):
            return color
    return None


def _extract_number(text: str, *, default: float) -> float:
    digits = []
    started = False
    for char in text:
        if char.isdigit() or (char == "." and started):
            digits.append(char)
            started = True
        elif started:
            break
    try:
        return float("".join(digits)) if digits else float(default)
    except ValueError:
        return float(default)


def _summarize_detect_blocks(result: dict[str, Any]) -> str:
    if not result.get("ok"):
        return f"助手 > 视觉检测失败：{result.get('message', 'unknown error')}"
    detections = result.get("detections") or []
    items = [item for item in detections if isinstance(item, dict) and item.get("color")]
    if not items:
        return "助手 > 当前没有检测到颜色块。"
    target = result.get("target") or {}
    target_color = target.get("color") or result.get("target_color") or items[0].get("color")
    lines = [f"助手 > 当前看到 {len(items)} 个颜色块，优先目标是 {target_color}："]
    for item in items:
        lines.append(
            "  - "
            f"{item.get('color')}: "
            f"x={_format_number(item.get('x'))}, "
            f"y={_format_number(item.get('y'))}, "
            f"z={_format_number(item.get('z'))}"
        )
    return "\n".join(lines)


def _summarize_pick_color(result: dict[str, Any]) -> str:
    if result.get("cancelled"):
        return "助手 > 已取消抓取。"
    target = result.get("target") or {}
    color = target.get("color") or "目标"
    if result.get("ok"):
        return f"助手 > 抓取 {color} 的流程已完成。"
    failed = result.get("failed_step")
    message = result.get("message") or (f"失败步骤：{failed}" if failed else "unknown error")
    return f"助手 > 抓取 {color} 失败：{message}"


def _summarize_gripper(result: dict[str, Any]) -> str:
    if result.get("cancelled"):
        return "助手 > 已取消夹爪动作。"
    if result.get("ok"):
        reached = result.get("reached_opening_mm")
        if reached is not None:
            return f"助手 > 夹爪命令已发送，到达开度约 {float(reached):.1f} mm。"
        return "助手 > 夹爪命令已发送。"
    return f"助手 > 夹爪动作失败：{result.get('message', 'unknown error')}"


def _summarize_pose(result: dict[str, Any]) -> str:
    if result.get("cancelled"):
        return "助手 > 已取消摆姿势。"
    if result.get("ok"):
        final = result.get("final_pose", {}).get("position", {})
        return f"助手 > 姿势已摆好，位置: x={_format_number(final.get('x'))} y={_format_number(final.get('y'))} z={_format_number(final.get('z'))}"
    return f"助手 > 摆姿势失败：{result.get('message', 'unknown error')}"


def _generate_random_pose() -> dict[str, Any]:
    import random
    x = round(random.uniform(0.15, 0.35), 2)
    y = round(random.uniform(-0.1, 0.1), 2)
    z = round(random.uniform(0.25, 0.4), 2)
    return {
        "x": x,
        "y": y,
        "z": z,
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "yaw_deg": round(random.uniform(-30, 30), 1),
        "duration": 2.0,
    }


async def _call_mcp_tool_with_retry(mcp, tool_name: str, arguments: dict, max_retries: int = 5) -> dict[str, Any]:
    result = await _call_mcp_tool(mcp, tool_name, arguments)
    if result.get("ok"):
        return result
    for attempt in range(max_retries - 1):
        arguments = _generate_random_pose()
        print(f"助手 > 重试中... 尝试 {attempt + 2}/{max_retries}")
        result = await _call_mcp_tool(mcp, tool_name, arguments)
        if result.get("ok"):
            return result
    return result


def _format_number(value: Any) -> str:
    try:
        return f"{float(value):.3f}"
    except (TypeError, ValueError):
        return "--"


async def _call_mcp_tool(mcp: Client, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
    try:
        result = await mcp.call_tool(name, arguments)
    except Exception as exc:
        return {"ok": False, "tool": name, "message": str(exc), "error_type": exc.__class__.__name__}
    payload = _mcp_result_to_json(result)
    return payload if isinstance(payload, dict) else {"ok": True, "tool": name, "result": payload}


def _confirm_tool(name: str, arguments: dict[str, Any]) -> bool:
    print(f"\n即将执行运动工具: {name} {_compact_json(arguments, limit=260)}")
    answer = input("确认执行？输入 y 继续，其它取消 > ").strip().lower()
    return answer in {"y", "yes", "是", "确认"}


def _mcp_tool_to_chat_tool(tool: Any) -> dict[str, Any]:
    name = str(getattr(tool, "name", ""))
    description = str(getattr(tool, "description", "") or f"MCP tool {name}")
    schema = (
        getattr(tool, "inputSchema", None)
        or getattr(tool, "input_schema", None)
        or getattr(tool, "parameters", None)
        or {"type": "object", "properties": {}}
    )
    schema = _plain_json(schema)
    if not isinstance(schema, dict):
        schema = {"type": "object", "properties": {}}
    schema.setdefault("type", "object")
    schema.setdefault("properties", {})
    return {
        "type": "function",
        "function": {
            "name": name,
            "description": description,
            "parameters": schema,
        },
    }


_TOOL_PARAMS = {
    "move_to_pose": {"x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg", "duration"},
    "move_joints": {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "duration"},
    "set_gripper_opening_mm": {"opening_mm"},
    "pick_color": {"color"},
    "detect_blocks": {"preferred_color"},
    "get_robot_status": set(),
    "safe_home": set(),
}

def _parse_tool_arguments(value: Any, tool_name: str = "") -> dict[str, Any]:
    if isinstance(value, dict):
        parsed = value
    elif not value:
        return {}
    else:
        try:
            parsed = json.loads(str(value))
        except json.JSONDecodeError:
            return {}
    if not isinstance(parsed, dict):
        return {}
    allowed_params = _TOOL_PARAMS.get(tool_name, set())
    if allowed_params:
        return {k: v for k, v in parsed.items() if k in allowed_params}
    return parsed


def _mcp_result_to_json(result: Any) -> Any:
    structured = getattr(result, "structured_content", None)
    if structured is not None:
        return _plain_json(structured)
    data = getattr(result, "data", None)
    if data is not None:
        return _plain_json(data)

    payload: dict[str, Any] = {}
    content = getattr(result, "content", None)
    if content is not None:
        payload["content"] = [
            _plain_json(getattr(item, "text", item)) for item in list(content)
        ]
    if hasattr(result, "is_error"):
        payload["is_error"] = bool(getattr(result, "is_error"))
    return payload


def _plain_json(value: Any) -> Any:
    if hasattr(value, "model_dump"):
        value = value.model_dump()
    try:
        return json.loads(json.dumps(value, ensure_ascii=False, default=str))
    except TypeError:
        return str(value)


def _compact_json(value: Any, *, limit: int) -> str:
    text = json.dumps(_plain_json(value), ensure_ascii=False, separators=(",", ":"), default=str)
    if len(text) <= limit:
        return text
    return text[: max(int(limit) - 20, 20)] + "...<truncated>"


def _parse_tool_calls_from_text(text: str) -> list[dict[str, Any]]:
    import re
    calls = []
    text = text.strip()
    tool_pattern = r"(move_to_pose|move_joints|set_gripper_opening_mm|pick_color|detect_blocks|get_robot_status|safe_home)\s+([^;]+)"
    matches = re.finditer(tool_pattern, text, re.IGNORECASE)
    for match in matches:
        name = match.group(1).lower()
        args_text = match.group(2)
        arguments = {}
        kv_pattern = r"(\w+)\s*=\s*([\d.]+)"
        kv_matches = re.finditer(kv_pattern, args_text)
        for kv in kv_matches:
            key = kv.group(1)
            try:
                value = float(kv.group(2))
            except ValueError:
                value = kv.group(2)
            arguments[key] = value
        if arguments:
            calls.append({
                "function": {
                    "name": name,
                    "arguments": json.dumps(arguments),
                },
                "id": f"tool-{int(time.time() * 1000)}-{len(calls)}",
            })
    return calls


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a text LLM agent for reBotArm MCP tools.")
    parser.add_argument("--mcp-url", default=os.getenv("REBOTARM_MCP_URL", DEFAULT_MCP_URL))
    parser.add_argument(
        "--base-url",
        default=os.getenv("REBOTARM_LLM_BASE_URL", DEFAULT_BASE_URL),
        help="OpenAI-compatible base URL.",
    )
    parser.add_argument(
        "--api-key",
        default=(
            os.getenv("REBOTARM_LLM_API_KEY")
            or os.getenv("DASHSCOPE_API_KEY")
            or os.getenv("OPENAI_API_KEY")
            or ""
        ),
    )
    parser.add_argument(
        "--model",
        default=os.getenv("REBOTARM_LLM_MODEL", DEFAULT_MODEL),
        help="Chat Completions model name.",
    )
    parser.add_argument("--timeout-sec", type=float, default=float(os.getenv("REBOTARM_LLM_TIMEOUT_SEC", "60")))
    parser.add_argument("--temperature", type=float, default=float(os.getenv("REBOTARM_LLM_TEMPERATURE", "0.7")))
    parser.add_argument("--max-tool-rounds", type=int, default=8)
    parser.add_argument("--result-chars", type=int, default=6000)
    parser.add_argument(
        "--verbose-tools",
        action="store_true",
        help="Print raw JSON for local MCP shortcut commands.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Do not ask before motion tools. Use only in a safe simulation.",
    )
    parser.add_argument(
        "--http-server",
        action="store_true",
        help="Run as an HTTP server for web UI integration (POST /chat).",
    )
    parser.add_argument(
        "--http-host",
        default=os.getenv("REBOTARM_AGENT_HTTP_HOST", "0.0.0.0"),
        help="HTTP server bind host.",
    )
    parser.add_argument(
        "--http-port",
        type=int,
        default=int(os.getenv("REBOTARM_AGENT_HTTP_PORT", "8082")),
        help="HTTP server bind port.",
    )
    args, _ros_args = parser.parse_known_args(argv)
    return args


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    has_api_key = bool(args.api_key)
    is_local = "localhost" in args.base_url or "127.0.0.1" in args.base_url
    # HTTP server mode: start even without API key (Dashboard/tools work, /chat returns error)
    if getattr(args, "http_server", False):
        if not has_api_key and not is_local:
            print(
                "[text-agent-http] WARNING: No API key set. /chat endpoint will return errors. "
                "Set REBOTARM_LLM_API_KEY, DASHSCOPE_API_KEY, or OPENAI_API_KEY to enable LLM chat. "
                "Dashboard, /tools and /call_tool work without an API key.",
                file=sys.stderr,
            )
        return asyncio.run(run_http_server(args))
    # REPL mode: API key required
    if not has_api_key and not is_local:
        print(
            "Missing API key. Set REBOTARM_LLM_API_KEY, DASHSCOPE_API_KEY, or OPENAI_API_KEY, "
            "or point --base-url to a local OpenAI-compatible server.",
            file=sys.stderr,
        )
        return 2
    try:
        return asyncio.run(run_repl(args))
    except KeyboardInterrupt:
        print()
        return 130



# ============ MCP Dashboard ============

_MCP_DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>reBotArm MCP Dashboard</title>
<style>
:root{
  --bg:#111211;--surface:#191b1a;--surface-2:#202321;--line:rgba(255,255,255,.12);
  --text:#f4f1ea;--muted:#a7ada7;--teal:#33d6b0;--amber:#f2a541;--red:#ef5a4d;
  --green:#77c96b;--blue:#5fa8ff;--purple:#a78bfa;--pink:#e879f9;
}
*{box-sizing:border-box;margin:0;padding:0}
body{height:100vh;background:var(--bg);color:var(--text);font-family:Inter,"Segoe UI","Microsoft YaHei",Arial,sans-serif;display:flex;flex-direction:column;overflow:hidden}

/* Top bar */
.topbar{display:flex;align-items:center;justify-content:space-between;padding:12px 20px;border-bottom:1px solid var(--line);background:rgba(17,18,17,.78);backdrop-filter:blur(14px);z-index:10}
.brand{display:flex;align-items:center;gap:10px}
.brand-dot{width:10px;height:10px;border-radius:3px;background:var(--teal);box-shadow:0 0 12px rgba(51,214,176,.6)}
.brand h1{font-size:18px;font-weight:700;letter-spacing:0}
.topbar-actions{display:flex;align-items:center;gap:12px}
.lang-btn{background:rgba(255,255,255,.06);border:1px solid var(--line);color:var(--text);border-radius:6px;padding:5px 12px;font-size:12px;font-weight:600;cursor:pointer;transition:.2s}
.lang-btn:hover{border-color:var(--teal);color:var(--teal)}
.status-pill{display:inline-flex;align-items:center;gap:6px;padding:5px 12px;border-radius:999px;background:rgba(255,255,255,.06);font-size:12px;color:var(--muted);white-space:nowrap}
.status-dot{width:8px;height:8px;border-radius:999px;background:var(--red);box-shadow:0 0 10px rgba(239,90,77,.5);transition:.3s}
.status-pill.online .status-dot{background:var(--green);box-shadow:0 0 12px rgba(119,201,107,.6)}

/* Main layout */
.main{flex:1;min-height:0;display:grid;grid-template-columns:1fr 360px;overflow:hidden}

/* Tools panel (left) */
.tools-panel{display:flex;flex-direction:column;overflow:hidden;min-height:0}
.panel-header{padding:14px 20px;border-bottom:1px solid var(--line);background:var(--surface)}
.search-row{display:flex;gap:10px;align-items:center}
.search-input{flex:1;background:var(--bg);border:1px solid var(--line);border-radius:6px;padding:7px 12px;color:var(--text);font-size:13px;min-width:0}
.search-input:focus{outline:none;border-color:var(--teal)}
.btn-register{background:var(--teal);color:#111211;border:none;border-radius:6px;padding:7px 14px;font-size:12px;font-weight:700;cursor:pointer;white-space:nowrap;transition:.2s}
.btn-register:hover{opacity:.85}
.tools-container{flex:1;overflow-y:auto;padding:18px 20px}
.loading{display:flex;align-items:center;justify-content:center;height:100%;color:var(--muted);font-size:14px}

/* Category sections */
.cat-section{margin-bottom:22px}
.cat-title{display:flex;align-items:center;gap:8px;font-size:12px;font-weight:700;margin-bottom:10px;text-transform:uppercase;letter-spacing:.5px;color:var(--text)}
.cat-badge{width:10px;height:10px;border-radius:3px}
.cat-count{color:var(--muted);font-weight:400;font-size:11px;margin-left:auto}
.tools-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:10px}

/* Tool cards */
.tool-card{background:var(--surface-2);border:1px solid var(--line);border-radius:8px;padding:14px;transition:.2s}
.tool-card:hover{border-color:rgba(255,255,255,.22)}
.tool-card.custom{border-color:rgba(167,139,250,.3)}
.tool-head{display:flex;align-items:center;gap:6px;margin-bottom:6px;flex-wrap:wrap}
.tool-name{font-size:13px;font-weight:600;font-family:"Cascadia Mono",Consolas,monospace}
.tag{display:inline-block;font-size:10px;padding:1px 6px;border-radius:3px;font-weight:600}
.tag.motion{background:rgba(242,165,65,.15);color:var(--amber)}
.tag.custom{background:rgba(167,139,250,.15);color:var(--purple)}
.tool-desc{font-size:12px;color:var(--muted);line-height:1.4;margin-bottom:10px}
.tool-params{display:flex;flex-direction:column;gap:5px;margin-bottom:10px}
.param-row{display:flex;align-items:center;gap:8px}
.param-row label{font-size:11px;color:var(--muted);min-width:80px;font-family:"Cascadia Mono",Consolas,monospace}
.param-row input{flex:1;background:var(--bg);border:1px solid var(--line);border-radius:4px;padding:4px 8px;color:var(--text);font-size:12px;min-width:0}
.param-row input:focus{outline:none;border-color:var(--teal)}
.no-params{font-size:11px;color:var(--muted);font-style:italic;margin-bottom:10px}
.btn-call{background:var(--teal);color:#111211;border:none;border-radius:4px;padding:5px 14px;font-size:12px;font-weight:600;cursor:pointer;transition:.2s}
.btn-call:hover{opacity:.85}
.btn-call:disabled{opacity:.4;cursor:not-allowed}
.btn-del{background:rgba(239,90,77,.12);color:var(--red);border:1px solid rgba(239,90,77,.25);border-radius:4px;padding:5px 10px;font-size:11px;cursor:pointer;margin-left:6px;transition:.2s}
.btn-del:hover{background:rgba(239,90,77,.2)}

/* Chat panel (right) */
.chat-panel{border-left:1px solid var(--line);background:rgba(21,23,22,.6);backdrop-filter:blur(10px);display:flex;flex-direction:column;overflow:hidden;min-height:0}
.chat-header{padding:14px 16px;border-bottom:1px solid var(--line);font-size:13px;font-weight:700;color:var(--teal)}
.chat-log{flex:1;overflow-y:auto;padding:12px 16px;font-size:13px;line-height:1.5}
.chat-log .msg{margin-bottom:8px;padding:8px 10px;border-radius:6px;word-break:break-word}
.chat-log .msg.user{background:var(--surface-2)}
.chat-log .msg.assistant{background:rgba(51,214,176,.08);border-left:2px solid var(--teal)}
.chat-log .msg.tool{background:rgba(95,168,255,.08);border-left:2px solid var(--blue);font-family:"Cascadia Mono",Consolas,monospace;font-size:11px}
.chat-log .msg.error{background:rgba(239,90,77,.08);border-left:2px solid var(--red)}
.chat-log .msg.info{background:rgba(242,165,65,.08);border-left:2px solid var(--amber);font-size:12px}
.chat-input-row{display:flex;gap:8px;padding:12px 16px;border-top:1px solid var(--line)}
.chat-input-row input{flex:1;background:var(--bg);border:1px solid var(--line);border-radius:6px;padding:7px 12px;color:var(--text);font-size:13px;min-width:0}
.chat-input-row input:focus{outline:none;border-color:var(--teal)}
.chat-input-row button{background:var(--teal);color:#111211;border:none;border-radius:6px;padding:7px 16px;font-size:13px;font-weight:600;cursor:pointer;white-space:nowrap}
.chat-input-row button:disabled{opacity:.4;cursor:not-allowed}

/* Modal */
.modal-overlay{display:none;position:fixed;inset:0;background:rgba(0,0,0,.6);backdrop-filter:blur(4px);z-index:100;align-items:center;justify-content:center}
.modal-overlay.active{display:flex}
.modal{background:var(--surface);border:1px solid var(--line);border-radius:8px;width:min(520px,90vw);max-height:88vh;overflow-y:auto;box-shadow:0 24px 80px rgba(0,0,0,.5)}
.modal-header{display:flex;align-items:center;justify-content:space-between;padding:16px 20px;border-bottom:1px solid var(--line)}
.modal-header h2{font-size:16px;font-weight:700}
.modal-close{background:none;border:none;color:var(--muted);font-size:22px;cursor:pointer;line-height:1;padding:0 4px}
.modal-close:hover{color:var(--text)}
.modal-body{padding:16px 20px;display:flex;flex-direction:column;gap:14px}
.form-row{display:flex;flex-direction:column;gap:5px}
.form-row label{font-size:12px;font-weight:600;color:var(--text)}
.form-row input,.form-row textarea{background:var(--bg);border:1px solid var(--line);border-radius:6px;padding:8px 10px;color:var(--text);font-size:13px;font-family:inherit}
.form-row textarea{font-family:"Cascadia Mono",Consolas,monospace;font-size:12px;resize:vertical}
.form-row input:focus,.form-row textarea:focus{outline:none;border-color:var(--teal)}
.form-hint{font-size:11px;color:var(--muted)}
.modal-footer{display:flex;justify-content:flex-end;gap:10px;padding:14px 20px;border-top:1px solid var(--line)}
.btn-secondary{background:rgba(255,255,255,.06);color:var(--text);border:1px solid var(--line);border-radius:6px;padding:8px 16px;font-size:13px;cursor:pointer}
.btn-secondary:hover{border-color:rgba(255,255,255,.2)}
.btn-primary{background:var(--teal);color:#111211;border:none;border-radius:6px;padding:8px 16px;font-size:13px;font-weight:600;cursor:pointer}
.btn-primary:hover{opacity:.85}
.btn-primary:disabled{opacity:.4;cursor:not-allowed}

/* Scrollbar */
::-webkit-scrollbar{width:6px}
::-webkit-scrollbar-track{background:transparent}
::-webkit-scrollbar-thumb{background:rgba(255,255,255,.12);border-radius:3px}
::-webkit-scrollbar-thumb:hover{background:rgba(255,255,255,.2)}

@media(max-width:768px){.main{grid-template-columns:1fr;grid-template-rows:1fr 240px}.chat-panel{border-left:none;border-top:1px solid var(--line)}}
</style>
</head>
<body>

<div class="topbar">
  <div class="brand">
    <span class="brand-dot"></span>
    <h1 data-i18n="title">reBotArm MCP Dashboard</h1>
  </div>
  <div class="topbar-actions">
    <button class="lang-btn" id="lang-toggle">EN</button>
    <div class="status-pill" id="status-pill">
      <span class="status-dot"></span>
      <span id="status-text" data-i18n="status.connecting">Connecting</span>
    </div>
  </div>
</div>

<div class="main">
  <section class="tools-panel">
    <div class="panel-header">
      <div class="search-row">
        <input class="search-input" type="text" id="search" data-i18n-ph="search.ph" placeholder="Search tools..."/>
        <button class="btn-register" id="btn-register" data-i18n="btn.register">Register Tool</button>
      </div>
    </div>
    <div class="tools-container" id="tools-container">
      <div class="loading" data-i18n="loading">Loading tools...</div>
    </div>
  </section>

  <aside class="chat-panel">
    <div class="chat-header" data-i18n="chat.title">Natural Language Control</div>
    <div class="chat-log" id="chat-log"></div>
    <div class="chat-input-row">
      <input type="text" id="chat-input" data-i18n-ph="chat.placeholder" placeholder="Enter a command..." disabled/>
      <button id="chat-btn" disabled data-i18n="chat.send">Send</button>
    </div>
  </aside>
</div>

<div class="modal-overlay" id="register-modal">
  <div class="modal">
    <div class="modal-header">
      <h2 data-i18n="modal.title">Register Custom MCP Tool</h2>
      <button class="modal-close" id="modal-close">&times;</button>
    </div>
    <div class="modal-body">
      <div class="form-row">
        <label data-i18n="modal.name">Tool Name</label>
        <input type="text" id="reg-name" placeholder="my_custom_tool"/>
        <span class="form-hint" data-i18n="modal.nameHint">Lowercase, underscores, no spaces</span>
      </div>
      <div class="form-row">
        <label data-i18n="modal.desc">Description</label>
        <textarea id="reg-desc" rows="2" data-i18n-ph="modal.descPh" placeholder="What does this tool do?"></textarea>
      </div>
      <div class="form-row">
        <label data-i18n="modal.category">Category</label>
        <input type="text" id="reg-category" placeholder="Custom" data-i18n-ph="modal.catPh"/>
      </div>
      <div class="form-row">
        <label data-i18n="modal.webhook">Webhook URL</label>
        <input type="text" id="reg-webhook" placeholder="http://localhost:3000/my-tool"/>
        <span class="form-hint" data-i18n="modal.webhookHint">Tool arguments will be POSTed here as JSON</span>
      </div>
      <div class="form-row">
        <label data-i18n="modal.schema">Parameter Schema (JSON)</label>
        <textarea id="reg-params" rows="6" placeholder='{"type":"object","properties":{"value":{"type":"number","description":"..."}},"required":["value"]}'></textarea>
      </div>
    </div>
    <div class="modal-footer">
      <button class="btn-secondary" id="modal-cancel" data-i18n="modal.cancel">Cancel</button>
      <button class="btn-primary" id="modal-submit" data-i18n="modal.submit">Register</button>
    </div>
  </div>
</div>

<script>
/* ===== i18n ===== */
const LANG_KEY='rebotarm.mcp.lang';
const DICT={
  'title':{zh:'reBotArm MCP 控制台',en:'reBotArm MCP Dashboard'},
  'status.connecting':{zh:'连接中',en:'Connecting'},
  'status.connected':{zh:'已连接',en:'Connected'},
  'status.failed':{zh:'连接失败',en:'Failed'},
  'search.ph':{zh:'搜索工具...',en:'Search tools...'},
  'btn.register':{zh:'注册新工具',en:'Register Tool'},
  'loading':{zh:'正在加载工具列表...',en:'Loading tools...'},
  'loadFail':{zh:'加载失败',en:'Load failed'},
  'retry':{zh:'重试',en:'Retry'},
  'call':{zh:'调用',en:'Call'},
  'running':{zh:'执行中...',en:'Running...'},
  'motion':{zh:'运动',en:'Motion'},
  'custom':{zh:'自定义',en:'Custom'},
  'noParams':{zh:'无参数',en:'No parameters'},
  'tools':{zh:'个工具',en:'tools'},
  'chat.title':{zh:'自然语言控制',en:'Natural Language Control'},
  'chat.placeholder':{zh:'输入指令，如：回到零位、打开夹爪、抓红色方块',en:'Enter a command, e.g. go home, open gripper, pick red'},
  'chat.send':{zh:'发送',en:'Send'},
  'chat.waiting':{zh:'等待...',en:'Waiting...'},
  'callTool':{zh:'调用',en:'Call'},
  'result':{zh:'结果',en:'Result'},
  'callFail':{zh:'调用失败',en:'Call failed'},
  'delConfirm':{zh:'确定删除此工具？',en:'Delete this tool?'},
  'modal.title':{zh:'注册自定义 MCP 工具',en:'Register Custom MCP Tool'},
  'modal.name':{zh:'工具名称',en:'Tool Name'},
  'modal.nameHint':{zh:'小写字母、下划线，不含空格',en:'Lowercase, underscores, no spaces'},
  'modal.desc':{zh:'描述',en:'Description'},
  'modal.descPh':{zh:'这个工具做什么？',en:'What does this tool do?'},
  'modal.category':{zh:'分类',en:'Category'},
  'modal.catPh':{zh:'自定义',en:'Custom'},
  'modal.webhook':{zh:'Webhook URL',en:'Webhook URL'},
  'modal.webhookHint':{zh:'工具参数将以 JSON 格式 POST 到此地址',en:'Tool arguments will be POSTed here as JSON'},
  'modal.schema':{zh:'参数 Schema (JSON)',en:'Parameter Schema (JSON)'},
  'modal.cancel':{zh:'取消',en:'Cancel'},
  'modal.submit':{zh:'注册',en:'Register'},
  'regSuccess':{zh:'工具注册成功',en:'Tool registered successfully'},
  'regFail':{zh:'注册失败',en:'Registration failed'},
  'invalidJson':{zh:'JSON 格式错误',en:'Invalid JSON'},
  'fillRequired':{zh:'请填写工具名称和 Webhook URL',en:'Please fill in tool name and webhook URL'},
};
const TOOL_I18N={
  'get_robot_status':{zh:{name:'获取机器人状态',desc:'返回最新的机器人状态、关节反馈、夹爪反馈和视觉新鲜度'}},
  'diagnose_ros':{zh:{name:'ROS 诊断',desc:'检查预期的 reBotArm ROS 服务、动作和反馈话题是否可用'}},
  'enable_robot':{zh:{name:'启用机器人',desc:'启用机器人控制器。此操作本身不会发送运动目标'}},
  'disable_robot':{zh:{name:'禁用机器人',desc:'禁用机器人控制器'}},
  'safe_home':{zh:{name:'安全归位',desc:'将机械臂移动到配置的安全归位位置。需要 motion_mode=allow'}},
  'move_to_pose':{zh:{name:'移动到位姿',desc:'将末端执行器移动到笛卡尔位姿。需要 motion_mode=allow'}},
  'move_joints':{zh:{name:'移动关节',desc:'使用安全两点轨迹移动一个或多个机械臂关节（弧度）。需要 motion_mode=allow'}},
  'ik_check':{zh:{name:'IK 可达性检查',desc:'对目标位姿运行 IK 可达性检查，不执行运动'}},
  'set_gripper_opening_mm':{zh:{name:'设置夹爪开度',desc:'命令夹爪开度（毫米），0 闭合至 90 全开。需要 motion_mode=allow'}},
  'gravity_compensation_status':{zh:{name:'重力补偿状态',desc:'查询控制器端重力补偿是否激活'}},
  'gravity_compensation_start':{zh:{name:'启动重力补偿',desc:'启动控制器端重力补偿。需要 motion_mode=allow'}},
  'gravity_compensation_stop':{zh:{name:'停止重力补偿',desc:'停止控制器端重力补偿'}},
  'detect_blocks':{zh:{name:'检测方块',desc:'返回最新的模拟色块检测结果，可按颜色过滤'}},
  'pick_color':{zh:{name:'拾取色块',desc:'用"靠近-闭合-提升"序列拾取最新检测到的色块。需要 motion_mode=allow'}},
  'record_start':{zh:{name:'开始录制',desc:'如果模拟任务服务器正在运行，则开始 MuJoCo 任务录制'}},
  'record_stop':{zh:{name:'停止录制',desc:'停止 MuJoCo 任务录制并保存已捕获的 CSV'}},
  'record_replay':{zh:{name:'回放录制',desc:'回放内存中最新的 MuJoCo 任务录制。需要 motion_mode=allow'}},
  'record_clear':{zh:{name:'清除录制',desc:'清除当前 MuJoCo 任务录制缓冲区'}},
};
const CAT_I18N={'状态与诊断':'Status & Diagnostics','使能控制':'Enable Control','运动控制':'Motion Control','夹爪控制':'Gripper Control','重力补偿':'Gravity Compensation','视觉抓取':'Vision & Pick','录制回放':'Record & Replay'};
let curLang='zh';
function t(key){const e=DICT[key];return e?e[curLang]:key;}
function tt(name){const ti=TOOL_I18N[name];return(curLang==='zh'&&ti)?ti:null;}
function catName(zh){return(curLang==='en'&&CAT_I18N[zh])?CAT_I18N[zh]:zh;}
function applyI18n(){document.querySelectorAll('[data-i18n]').forEach(el=>{const k=el.dataset.i18n;const e=DICT[k];if(e)el.textContent=e[curLang];});document.querySelectorAll('[data-i18n-ph]').forEach(el=>{const k=el.dataset.i18nPh;const e=DICT[k];if(e)el.placeholder=e[curLang];});}
function setLang(l){curLang=l;try{localStorage.setItem(LANG_KEY,l);}catch(_){}applyI18n();document.getElementById('lang-toggle').textContent=l==='zh'?'EN':'中文';if(typeof allTools!=='undefined'&&allTools.length)renderTools(allTools);}
function initLang(){try{curLang=localStorage.getItem(LANG_KEY)||'zh';}catch(_){curLang='zh';}setLang(curLang);}

/* ===== State ===== */
const MOTION_TOOLS=["safe_home","gravity_compensation_start","set_gripper_opening_mm","move_to_pose","move_joints","pick_color","record_replay"];
const PARAMS_FILTER={"move_to_pose":["x","y","z","duration"]};
let allTools=[];

/* ===== Load tools ===== */
async function loadTools(){
  try{
    const r=await fetch('/tools');
    const data=await r.json();
    if(!data.ok)throw new Error(data.error||t('loadFail'));
    allTools=data.tools||[];
    renderTools(allTools);
    const pill=document.getElementById('status-pill');
    pill.classList.add('online');
    document.getElementById('status-text').textContent=t('status.connected')+' '+allTools.length+' '+t('tools');
    document.getElementById('chat-input').disabled=false;
    document.getElementById('chat-btn').disabled=false;
  }catch(e){
    document.getElementById('tools-container').innerHTML='<div class="loading" style="color:var(--red)">'+t('loadFail')+': '+e.message+'<br><button onclick="loadTools()" style="margin-top:8px;background:var(--teal);border:none;border-radius:4px;padding:4px 12px;cursor:pointer">'+t('retry')+'</button></div>';
    document.getElementById('status-text').textContent=t('status.failed');
  }
}

/* ===== Render tools ===== */
function renderTools(tools){
  const filter=document.getElementById('search').value.toLowerCase();
  const filtered=filter?tools.filter(tl=>{const ti=TOOL_I18N[tl.name];const dn=(ti&&ti.zh)?ti.zh.name:tl.name;const dd=(ti&&ti.zh)?ti.zh.desc:tl.description;return dn.toLowerCase().includes(filter)||dd.toLowerCase().includes(filter);}):tools;
  const cats={};
  filtered.forEach(t=>{
    const info=t.category||['Other','#a7ada7'];
    const cn=info[0];
    if(!cats[cn])cats[cn]={color:info[1],tools:[]};
    cats[cn].tools.push(t);
  });
  let html='';
  for(const[name,info]of Object.entries(cats)){
 html+='<div class="cat-section"><div class="cat-title"><span class="cat-badge" style="background:'+info.color+'"></span>'+catName(name)+'<span class="cat-count">'+info.tools.length+' '+t('tools')+'</span></div><div class="tools-grid">';
    for(const tool of info.tools){
     const isMotion=MOTION_TOOLS.includes(tool.name)||tool.is_motion;
      const isCustom=tool.custom;
      const tInfo=tt(tool.name);
      const dn=tInfo&&tInfo.zh?tInfo.zh.name:tool.name;
      const dd=tInfo&&tInfo.zh?tInfo.zh.desc:(tool.description||'');
      html+='<div class="tool-card'+(isCustom?' custom':'')+'"><div class="tool-head"><span class="tool-name">'+dn+'</span>';
      if(isMotion)html+='<span class="tag motion">'+t('motion')+'</span>';
      if(isCustom)html+='<span class="tag custom">'+t('custom')+'</span>';
      html+='</div><div class="tool-desc">'+dd+'</div>';
      const params=tool.parameters&&tool.parameters.properties||{};
      const req=tool.parameters&&tool.parameters.required||[];
      if(Object.keys(params).length>0){
        html+='<div class="tool-params">';
        const allowed=PARAMS_FILTER[tool.name];const paramEntries=allowed?Object.entries(params).filter(([k])=>allowed.includes(k)):Object.entries(params);for(const[pn,pi]of paramEntries){
          const isReq=req.includes(pn);
          const def=pi.default!==undefined?pi.default:'';
          const pt=pi.type||'string';
          html+='<div class="param-row"><label>'+pn+(isReq?'*':'')+'</label><input type="'+(pt==='number'||pt==='integer'?'number':'text')+'" data-tool="'+tool.name+'" data-param="'+pn+'" value="'+def+'" placeholder="'+pt+'"/></div>';
        }
        html+='</div>';
      }else{
        html+='<div class="no-params">'+t('noParams')+'</div>';
      }
      html+='<button class="btn-call" onclick="callTool(\''+tool.name+'\')">'+t('call')+'</button>';
      if(isCustom)html+='<button class="btn-del" onclick="delTool(\''+tool.name+'\')">&times;</button>';
      html+='</div>';
    }
    html+='</div></div>';
  }
  document.getElementById('tools-container').innerHTML=html||'<div class="loading">'+t('noParams')+'</div>';
}

/* ===== Call tool ===== */
async function callTool(name){
  const inputs=document.querySelectorAll('input[data-tool="'+name+'"]');
  const args={};
  inputs.forEach(inp=>{
    const val=inp.value.trim();
    if(val==='')return;
    const pt=inp.placeholder;
    if(pt==='number'||pt==='integer')args[inp.dataset.param]=parseFloat(val);
    else if(pt==='boolean')args[inp.dataset.param]=val==='true';
    else args[inp.dataset.param]=val;
  });
  const btn=event.target;
  btn.disabled=true;
 btn.textContent=t('running');
  addLog('tool',t('callTool')+' '+(tt(name)?tt(name).zh.name:name)+'('+JSON.stringify(args)+')');
 try{
   const r=await fetch('/call_tool',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name,arguments:args})});
   const data=await r.json();
    addLog('tool',(tt(name)?tt(name).zh.name:name)+' '+t('result')+': '+JSON.stringify(data).slice(0,500));
 }catch(e){
    addLog('error',t('callFail')+': '+e.message);
  }
  btn.disabled=false;
  btn.textContent=t('call');
}

/* ===== Delete custom tool ===== */
async function delTool(name){
  if(!confirm(t('delConfirm')))return;
  try{
    await fetch('/unregister_tool',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name})});
    loadTools();
  }catch(e){addLog('error',e.message);}
}

/* ===== Chat ===== */
function addLog(type,text){
  const log=document.getElementById('chat-log');
  const div=document.createElement('div');
  div.className='msg '+type;
  div.textContent=text;
  log.appendChild(div);
  log.scrollTop=log.scrollHeight;
}
async function sendChat(){
  const input=document.getElementById('chat-input');
  const btn=document.getElementById('chat-btn');
  const text=input.value.trim();
  if(!text)return;
  addLog('user',text);
  input.value='';
  btn.disabled=true;
  btn.textContent=t('chat.waiting');
  try{
    const r=await fetch('/chat',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({text})});
    const data=await r.json();
    if(data.ok){
      addLog('assistant',data.text||'(no response)');
      if(data.events){
        for(const ev of data.events){
          if(ev.type==='tool')addLog('tool',(tt(ev.name)?tt(ev.name).zh.name:ev.name)+'('+JSON.stringify(ev.arguments)+') -> '+JSON.stringify(ev.result).slice(0,300));
          else if(ev.type==='error')addLog('error',ev.message);
          else if(ev.type==='info')addLog('info',ev.message);
        }
      }
    }else{
      addLog('error',data.error||'Request failed');
    }
  }catch(e){
    addLog('error',e.message);
  }
  btn.disabled=false;
  btn.textContent=t('chat.send');
}

/* ===== Register modal ===== */
function openModal(){document.getElementById('register-modal').classList.add('active');}
function closeModal(){document.getElementById('register-modal').classList.remove('active');}

async function submitRegistration(){
  const name=document.getElementById('reg-name').value.trim();
  const desc=document.getElementById('reg-desc').value.trim();
  const cat=document.getElementById('reg-category').value.trim()||t('modal.catPh');
  const webhook=document.getElementById('reg-webhook').value.trim();
  const paramsRaw=document.getElementById('reg-params').value.trim();
  if(!name||!webhook){alert(t('fillRequired'));return;}
  let params={};
  if(paramsRaw){
    try{params=JSON.parse(paramsRaw);}catch(e){alert(t('invalidJson')+': '+e.message);return;}
  }
  const btn=document.getElementById('modal-submit');
  btn.disabled=true;
  try{
    const r=await fetch('/register_tool',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name,description:desc,category:cat,webhook_url:webhook,parameters:params})});
    const data=await r.json();
    if(data.ok){
      addLog('info',t('regSuccess')+': '+name);
      closeModal();
      loadTools();
      ['reg-name','reg-desc','reg-category','reg-webhook','reg-params'].forEach(id=>document.getElementById(id).value='');
    }else{
      alert(t('regFail')+': '+(data.error||''));
    }
  }catch(e){
    alert(t('regFail')+': '+e.message);
  }
  btn.disabled=false;
}

/* ===== Wire up ===== */
document.getElementById('lang-toggle').addEventListener('click',()=>setLang(curLang==='zh'?'en':'zh'));
document.getElementById('search').addEventListener('input',()=>renderTools(allTools));
document.getElementById('btn-register').addEventListener('click',openModal);
document.getElementById('modal-close').addEventListener('click',closeModal);
document.getElementById('modal-cancel').addEventListener('click',closeModal);
document.getElementById('modal-submit').addEventListener('click',submitRegistration);
document.getElementById('register-modal').addEventListener('click',e=>{if(e.target.id==='register-modal')closeModal();});
document.getElementById('chat-btn').addEventListener('click',sendChat);
document.getElementById('chat-input').addEventListener('keydown',e=>{if(e.key==='Enter')sendChat();});

initLang();
loadTools();
</script>
</body>
</html>
"""


# Per-request custom-tool registry (name -> definition).
# Custom tools are user-defined MCP tools backed by a webhook URL.
_CUSTOM_TOOLS: list[dict[str, Any]] = []


async def _http_register_tool(payload: dict[str, Any]) -> dict[str, Any]:
    """Register a user-defined MCP tool backed by a webhook URL."""
    name = str(payload.get("name", "")).strip()
    if not name:
        return {"ok": False, "error": "missing tool name"}
    webhook_url = str(payload.get("webhook_url", "")).strip()
    if not webhook_url:
        return {"ok": False, "error": "missing webhook_url"}
    description = str(payload.get("description", "") or "")
    category = str(payload.get("category", "") or "Custom")
    parameters = payload.get("parameters") or {}
    if isinstance(parameters, str):
        try:
            parameters = json.loads(parameters)
        except json.JSONDecodeError as exc:
            return {"ok": False, "error": f"invalid parameters JSON: {exc}"}
    # Remove any existing tool with the same name, then append.
    _CUSTOM_TOOLS[:] = [t for t in _CUSTOM_TOOLS if t["name"] != name]
    _CUSTOM_TOOLS.append({
        "name": name,
        "description": description,
        "category": (category, "#a78bfa"),
        "parameters": parameters,
        "webhook_url": webhook_url,
        "custom": True,
    })
    return {"ok": True, "name": name}


async def _http_unregister_tool(payload: dict[str, Any]) -> dict[str, Any]:
    """Remove a previously registered custom tool by name."""
    name = str(payload.get("name", "")).strip()
    before = len(_CUSTOM_TOOLS)
    _CUSTOM_TOOLS[:] = [t for t in _CUSTOM_TOOLS if t["name"] != name]
    removed = before - len(_CUSTOM_TOOLS)
    return {"ok": True, "removed": removed}


async def _http_list_tools(args: argparse.Namespace) -> dict[str, Any]:
    """List MCP tools with categories for the dashboard."""
    try:
        async with Client(args.mcp_url) as mcp:
            mcp_tools = await mcp.list_tools()
            tools = []
            for tool in mcp_tools:
                name = str(getattr(tool, "name", ""))
                cat_info = TOOL_CATEGORIES.get(name, ("其他", "#a7ada7"))
                chat_tool = _mcp_tool_to_chat_tool(tool)
                tools.append({
                    "name": name,
                    "description": str(getattr(tool, "description", "") or ""),
                    "parameters": chat_tool["function"]["parameters"],
                    "category": cat_info,
                    "is_motion": name in MOTION_TOOLS,
                })
        # Append user-registered custom tools.
        for ct in _CUSTOM_TOOLS:
            tools.append({
                "name": ct["name"],
                "description": ct["description"],
                "parameters": ct.get("parameters") or {"type": "object", "properties": {}},
                "category": ct["category"],
                "is_motion": False,
                "custom": True,
            })
        return {"ok": True, "tools": tools}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


async def _http_call_tool(args: argparse.Namespace, payload: dict[str, Any]) -> dict[str, Any]:
    """Call a single MCP tool directly."""
    name = str(payload.get("name", "")).strip()
    arguments = payload.get("arguments") or {}
    if not name:
        return {"ok": False, "error": "missing tool name"}
    # Check if this is a user-registered custom tool (webhook-backed).
    custom = next((t for t in _CUSTOM_TOOLS if t["name"] == name), None)
    if custom:
        try:
            req_data = json.dumps(arguments).encode("utf-8")
            req = urllib.request.Request(
                custom["webhook_url"],
                data=req_data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=30) as resp:
                body = resp.read().decode("utf-8", errors="replace")
                try:
                    result = json.loads(body)
                except json.JSONDecodeError:
                    result = {"raw": body}
                return {"ok": True, "result": result, "custom": True}
        except urllib.error.URLError as exc:
            return {"ok": False, "error": f"webhook call failed: {exc}", "custom": True}
        except Exception as exc:
            return {"ok": False, "error": str(exc), "custom": True}
    try:
        async with Client(args.mcp_url) as mcp:
            result = await _call_mcp_tool(mcp, name, arguments)
            return result
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


# ============ HTTP Server mode ============

# Per-request state holder for the HTTP server
_HTTP_STATE: dict[str, Any] = {}


async def _http_handle_chat(args: argparse.Namespace, payload: dict[str, Any]) -> dict[str, Any]:
    """Process a single chat turn over HTTP."""
    if not args.api_key and "localhost" not in args.base_url and "127.0.0.1" not in args.base_url:
        return {
            "ok": False,
            "error": "LLM API key not configured. Set REBOTARM_LLM_API_KEY, DASHSCOPE_API_KEY, or OPENAI_API_KEY. Tool listing and direct tool calls still work.",
        }
    user_text = str(payload.get("text") or payload.get("message") or "").strip()
    reset = bool(payload.get("reset", False))
    if not user_text:
        return {"ok": False, "error": "empty text"}

    if reset or "messages" not in _HTTP_STATE:
        _HTTP_STATE["messages"] = [{"role": "system", "content": SYSTEM_PROMPT}]

    events: list[dict[str, Any]] = []
    final_text = ""

    async with Client(args.mcp_url) as mcp:
        intent = _parse_builtin_intent(user_text)
        if intent is not None:
            command = intent["command"]
            parts = [command, *intent.get("args", [])]
            if command == "/pose":
                arguments = _generate_random_pose()
                result = await _call_mcp_tool_with_retry(mcp, "move_to_pose", arguments, max_retries=5)
                events.append({"type": "assistant", "content": "正在摆姿势..."})
                events.append({
                    "type": "tool",
                    "name": "move_to_pose",
                    "arguments": arguments,
                    "result": result,
                })
                if result.get("ok"):
                    final = result.get("final_pose", {}).get("position", {})
                    final_text = f"姿势已摆好，位置: x={_format_number(final.get('x'))} y={_format_number(final.get('y'))} z={_format_number(final.get('z'))}"
                else:
                    final_text = f"摆姿势失败：{result.get('message', 'unknown error')}"
                events.append({"type": "assistant", "content": final_text})
                return {"ok": True, "text": final_text, "events": events}
            elif command == "/detect":
                color = parts[1] if len(parts) > 1 else "auto"
                result = await _call_mcp_tool(mcp, "detect_blocks", {"preferred_color": color})
                events.append({"type": "assistant", "content": _summarize_detect_blocks(result)})
                return {"ok": True, "text": _summarize_detect_blocks(result), "events": events}
            elif command == "/pick":
                color = parts[1] if len(parts) > 1 else "auto"
                result = await _call_mcp_tool(mcp, "pick_color", {"color": color})
                events.append({"type": "assistant", "content": _summarize_pick_color(result)})
                return {"ok": True, "text": _summarize_pick_color(result), "events": events}
            elif command == "/gripper":
                if len(parts) >= 2:
                    try:
                        opening_mm = float(parts[1])
                        result = await _call_mcp_tool(mcp, "set_gripper_opening_mm", {"opening_mm": opening_mm})
                        events.append({"type": "assistant", "content": _summarize_gripper(result)})
                        return {"ok": True, "text": _summarize_gripper(result), "events": events}
                    except ValueError:
                        pass

        llm = ChatCompletionsLLM(
            base_url=args.base_url,
            api_key=args.api_key,
            model=args.model,
            timeout_sec=args.timeout_sec,
            temperature=args.temperature,
        )

        mcp_tools = await mcp.list_tools()
        tools = [_mcp_tool_to_chat_tool(tool) for tool in mcp_tools]

        _HTTP_STATE["messages"].append({"role": "user", "content": user_text})

        for _ in range(max(1, int(args.max_tool_rounds))):
            try:
                response = llm.complete(_HTTP_STATE["messages"], tools)
            except Exception as exc:
                events.append({"type": "error", "message": str(exc)})
                return {"ok": False, "error": str(exc), "events": events}

            choice = (response.get("choices") or [{}])[0]
            message = choice.get("message") or {}
            content = message.get("content") or ""
            tool_calls = message.get("tool_calls") or []

            if content:
                events.append({"type": "assistant", "content": content})
                final_text = content

            if not tool_calls:
                if content:
                    _HTTP_STATE["messages"].append({"role": "assistant", "content": content})
                break

            assistant_message = {"role": "assistant", "content": content, "tool_calls": tool_calls}
            _HTTP_STATE["messages"].append(assistant_message)

            for call in tool_calls:
                function = call.get("function") or {}
                name = str(function.get("name") or "")
                arguments = _parse_tool_arguments(function.get("arguments"), name)
                tool_call_id = str(call.get("id") or f"tool-{int(time.time() * 1000)}")

                if not name:
                    tool_result = {"ok": False, "message": "LLM emitted a tool call without a name."}
                elif args.yes is False and name in MOTION_TOOLS:
                    # In HTTP mode, always run motion tools (auto-yes by default)
                    tool_result = await _call_mcp_tool(mcp, name, arguments)
                else:
                    tool_result = await _call_mcp_tool(mcp, name, arguments)

                events.append({
                    "type": "tool",
                    "name": name,
                    "arguments": arguments,
                    "result": tool_result,
                })

                _HTTP_STATE["messages"].append({
                    "role": "tool",
                    "tool_call_id": tool_call_id,
                    "content": _compact_json(tool_result, limit=args.result_chars),
                })
        else:
            events.append({"type": "info", "message": "工具调用轮次已到上限。"})

    return {"ok": True, "text": final_text, "events": events}


class _ChatHTTPHandler(BaseHTTPRequestHandler):
    server_args: argparse.Namespace = None  # set in run_http_server

    def log_message(self, format, *args):  # noqa: A002
        sys.stderr.write("[text-agent-http] " + (format % args) + "\n")

    def _write_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):  # noqa: N802
        self._write_json(204, {})

    def _write_html(self, status: int, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):  # noqa: N802
        if self.path == "/" or self.path == "/dashboard":
            self._write_html(200, _MCP_DASHBOARD_HTML.encode("utf-8"))
        elif self.path == "/health":
            self._write_json(200, {"ok": True, "service": "rebotarm-text-agent"})
        elif self.path == "/tools":
            result = asyncio.run(_http_list_tools(self.server_args))
            self._write_json(200, result)
        else:
            self._write_json(404, {"ok": False, "error": "not found"})

    def do_POST(self):  # noqa: N802
        if self.path not in ("/chat", "/call_tool", "/register_tool", "/unregister_tool"):
            self._write_json(404, {"ok": False, "error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length > 0 else b"{}"
            payload = json.loads(raw.decode("utf-8"))
        except Exception as exc:
            self._write_json(400, {"ok": False, "error": f"invalid json: {exc}"})
            return

        try:
            if self.path == "/chat":
                result = asyncio.run(_http_handle_chat(self.server_args, payload))
            elif self.path == "/register_tool":
                result = asyncio.run(_http_register_tool(payload))
            elif self.path == "/unregister_tool":
                result = asyncio.run(_http_unregister_tool(payload))
            else:
                result = asyncio.run(_http_call_tool(self.server_args, payload))
        except Exception as exc:
            self._write_json(500, {"ok": False, "error": str(exc)})
            return

        self._write_json(200, result)


async def run_http_server(args: argparse.Namespace) -> int:
    host = str(getattr(args, "http_host", "0.0.0.0"))
    port = int(getattr(args, "http_port", 8082))

    _ChatHTTPHandler.server_args = args
    httpd = ThreadingHTTPServer((host, port), _ChatHTTPHandler)
    print(f"[text-agent-http] listening on http://{host}:{port}/ (dashboard) /chat (llm) /tools (list) /call_tool (invoke) /register_tool /unregister_tool", flush=True)
    print(f"[text-agent-http] MCP={args.mcp_url} model={args.model}", flush=True)

    loop = asyncio.get_event_loop()

    def _serve():
        httpd.serve_forever()

    serve_task = loop.run_in_executor(None, _serve)
    try:
        await asyncio.Event().wait()  # run forever
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        serve_task.cancel()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
