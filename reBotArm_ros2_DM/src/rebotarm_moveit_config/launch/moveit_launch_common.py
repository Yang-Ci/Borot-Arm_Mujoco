import os
from importlib.machinery import SourceFileLoader
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def apply_rviz_urdf_compat(moveit_config):
    """Apply the shared Jazzy multi-visual workaround to robot_description."""
    compat_path = (
        Path(get_package_share_directory("rebotarm_bringup"))
        / "launch"
        / "rviz_urdf_compat.py"
    )
    compat = SourceFileLoader("rviz_urdf_compat", str(compat_path)).load_module()
    description = moveit_config.robot_description["robot_description"]
    moveit_config.robot_description["robot_description"] = (
        compat.make_rviz_compatible(description)
    )
    return moveit_config


def moveit_parameters(moveit_config):
    parameters = moveit_config.to_dict()
    ompl = parameters.setdefault("ompl", {})
    ompl["planning_plugin"] = "ompl_interface/OMPLPlanner"

    if os.environ.get("ROS_DISTRO") == "humble":
        ompl["request_adapters"] = " ".join(
            [
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/ResolveConstraintFrames",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
            ]
        )
        ompl.pop("response_adapters", None)

    return parameters
