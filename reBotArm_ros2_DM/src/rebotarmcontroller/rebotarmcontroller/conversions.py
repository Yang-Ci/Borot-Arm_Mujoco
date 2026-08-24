from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion, quaternion_from_matrix


def pose_to_xyz_rpy(pose: Pose) -> tuple[float, float, float, float, float, float]:
    quat = [
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    ]
    roll, pitch, yaw = euler_from_quaternion(quat)
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(roll),
        float(pitch),
        float(yaw),
    )


def fk_to_pose(position: np.ndarray, rotation: np.ndarray) -> Pose:
    mat = np.eye(4)
    mat[:3, :3] = rotation
    quat = quaternion_from_matrix(mat)

    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose
