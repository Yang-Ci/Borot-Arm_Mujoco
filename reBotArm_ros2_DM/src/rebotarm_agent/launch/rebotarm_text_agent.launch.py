from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mcp_url = LaunchConfiguration("mcp_url")
    base_url = LaunchConfiguration("base_url")
    model = LaunchConfiguration("model")
    timeout_sec = LaunchConfiguration("timeout_sec")
    temperature = LaunchConfiguration("temperature")
    python_executable = LaunchConfiguration("python_executable")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mcp_url", default_value="http://127.0.0.1:8081/mcp"),
            DeclareLaunchArgument(
                "base_url",
                default_value=EnvironmentVariable(
                    "REBOTARM_LLM_BASE_URL",
                    default_value="https://api.openai.com/v1",
                ),
            ),
            DeclareLaunchArgument(
                "model",
                default_value=EnvironmentVariable(
                    "REBOTARM_LLM_MODEL",
                    default_value="gpt-4.1-mini",
                ),
            ),
            DeclareLaunchArgument("timeout_sec", default_value="60"),
            DeclareLaunchArgument("temperature", default_value="0.1"),
            DeclareLaunchArgument(
                "python_executable",
                default_value=[
                    EnvironmentVariable("HOME"),
                    "/reBot_Arm_Mujoco-DM/reBotArm_ros2_DM/.venv/bin/python3",
                ],
                description=(
                    "Python interpreter used to run the text agent. The default "
                    "points at the workspace virtualenv so fastmcp is importable."
                ),
            ),
            Node(
                package="rebotarm_agent",
                executable="rebotarm_text_agent",
                name="rebotarm_text_agent",
                output="screen",
                prefix=[python_executable],
                arguments=[
                    "--mcp-url",
                    mcp_url,
                    "--base-url",
                    base_url,
                    "--model",
                    model,
                    "--timeout-sec",
                    timeout_sec,
                    "--temperature",
                    temperature,
                ],
            ),
        ]
    )
