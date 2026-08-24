from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mujoco_share = FindPackageShare("rebotarm_mujoco")

    model_path = LaunchConfiguration("model_path")
    joint_map_file = LaunchConfiguration("joint_map_file")
    target_joint_state_topic = LaunchConfiguration("target_joint_state_topic")
    sim_joint_state_topic = LaunchConfiguration("sim_joint_state_topic")
    object_states_topic = LaunchConfiguration("object_states_topic")
    object_names = LaunchConfiguration("object_names")
    open_viewer = LaunchConfiguration("open_viewer")
    control_hz = LaunchConfiguration("control_hz")
    publish_hz = LaunchConfiguration("publish_hz")
    arm_torque_limit = LaunchConfiguration("arm_torque_limit")
    gripper_force_limit = LaunchConfiguration("gripper_force_limit")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [mujoco_share, "models", "rebotarm_b601_colored.xml"]
                ),
            ),
            DeclareLaunchArgument(
                "joint_map_file",
                default_value=PathJoinSubstitution(
                    [mujoco_share, "config", "joint_map_kinematic.yaml"]
                ),
            ),
            DeclareLaunchArgument("target_joint_state_topic", default_value="/rebotarm/joint_states"),
            DeclareLaunchArgument(
                "sim_joint_state_topic",
                default_value="/rebotarm/mujoco/physics_joint_states",
            ),
            DeclareLaunchArgument("object_states_topic", default_value="/rebotarm/mujoco/object_states"),
            DeclareLaunchArgument("object_names", default_value="red_cube,blue_block,yellow_cylinder"),
            DeclareLaunchArgument("open_viewer", default_value="true"),
            DeclareLaunchArgument("control_hz", default_value="500.0"),
            DeclareLaunchArgument("publish_hz", default_value="30.0"),
            DeclareLaunchArgument("arm_torque_limit", default_value="30.0"),
            DeclareLaunchArgument("gripper_force_limit", default_value="32.0"),
            Node(
                package="rebotarm_mujoco",
                executable="mujoco_physics_grasp",
                name="rebotarm_mujoco_physics_grasp",
                output="screen",
                parameters=[
                    {
                        "model_path": model_path,
                        "joint_map_file": joint_map_file,
                        "target_joint_state_topic": target_joint_state_topic,
                        "sim_joint_state_topic": sim_joint_state_topic,
                        "object_states_topic": object_states_topic,
                        "object_names": object_names,
                        "open_viewer": open_viewer,
                        "control_hz": control_hz,
                        "publish_hz": publish_hz,
                        "arm_torque_limit": arm_torque_limit,
                        "gripper_force_limit": gripper_force_limit,
                    }
                ],
            ),
        ]
    )
