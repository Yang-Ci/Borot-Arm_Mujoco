#!/usr/bin/env python3
"""Demo 2: 逆运动学 -- 给定目标位置，求解关节角

使用 ROS2 Service /<ns>/move_to_pose_ik。
需要先启动仿真栈。
自动检测 RS / DM 命名空间。
"""
import os
import sys

import rclpy
from rclpy.node import Node
from rebotarm_msgs.srv import MoveToPoseIK
from geometry_msgs.msg import Pose


def detect_namespace():
    """检测 RS 或 DM 命名空间。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("rebotarm_mujoco_rs")
        return "rebotarm_rs"
    except Exception:
        pass
    if "Mujoco-RS" in os.getcwd():
        return "rebotarm_rs"
    return "rebotarm"


class IKClient(Node):
    def __init__(self, namespace):
        super().__init__("ik_client")
        self.ns = namespace
        self.cli = self.create_client(MoveToPoseIK, f"/{namespace}/move_to_pose_ik")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"等待 /{namespace}/move_to_pose_ik 服务...")

    def solve(self, x, y, z):
        req = MoveToPoseIK.Request()
        req.target_pose.position.x = float(x)
        req.target_pose.position.y = float(y)
        req.target_pose.position.z = float(z)
        req.target_pose.orientation.w = 0.0

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        resp = future.result()
        print(f"成功: {resp.success}")
        print(f"消息: {resp.message}")
        print(f"关节解: {list(resp.q_solution)}")
        return resp


def main():
    rclpy.init()
    ns = detect_namespace()
    print(f"命名空间: /{ns}")

    node = IKClient(ns)

    # 目标：红色方块上方
    if ns == "rebotarm_rs":
        node.solve(0.31, -0.09, 0.20)
    else:
        node.solve(0.34, -0.13, 0.20)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
