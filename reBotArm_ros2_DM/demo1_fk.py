#!/usr/bin/env python3
"""Demo 1: 正运动学 -- 已知关节角，求末端位姿

不需要启动 ROS2 仿真，纯 MuJoCo Python 调用。
自动检测 RS / DM 模型。
"""
import os
import sys

import mujoco
import numpy as np

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# 三组测试姿态（弧度）
TEST_POSES = [
    [0, 0, 0, 0, 0, 0],
    [0, -0.5, -1.0, 0, 0, 0],
    [0, -0.8, -1.5, 0.5, 0, 0],
]


def find_model():
    """自动检测 RS 或 DM 模型文件路径。"""
    try:
        from ament_index_python.packages import get_package_share_directory

        for pkg, fname, ver in [
            ("rebotarm_mujoco_rs", "rs_arm.xml", "RS"),
            ("rebotarm_mujoco", "rebotarm_b601_stl.xml", "DM"),
        ]:
            try:
                share = get_package_share_directory(pkg)
                path = os.path.join(share, "models", fname)
                if os.path.isfile(path):
                    return path, ver
            except Exception:
                pass
    except ImportError:
        pass

    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        ("install/rebotarm_mujoco/share/rebotarm_mujoco/models/rebotarm_b601_stl.xml", "DM"),
        ("install/rebotarm_mujoco_rs/share/rebotarm_mujoco_rs/models/rs_arm.xml", "RS"),
        ("rebotarm_ros2/install/rebotarm_mujoco_rs/share/rebotarm_mujoco_rs/models/rs_arm.xml", "RS"),
    ]
    for rel, ver in candidates:
        p = os.path.join(script_dir, rel)
        if os.path.isfile(p):
            return p, ver

    print("Error: 找不到模型文件，请先 source 工作空间环境。", file=sys.stderr)
    sys.exit(1)


def main():
    xml_path, version = find_model()
    print(f"模型: {version} ({os.path.basename(xml_path)})")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")

    for i, angles in enumerate(TEST_POSES):
        # 按关节名设置 qpos 地址，兼容 RS (nq=9) 和 DM (nq=29)
        for j, name in enumerate(ARM_JOINTS):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                data.qpos[int(model.jnt_qposadr[jid])] = angles[j]

        mujoco.mj_forward(model, data)

        if tcp_id >= 0:
            pos = data.site_xpos[tcp_id]
            mat = data.site_xmat[tcp_id].reshape(3, 3)
        else:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_link")
            if bid < 0:
                bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper_end")
            pos = data.body(bid).xpos
            mat = data.body(bid).xmat.reshape(3, 3)

        print(f"\n--- 姿态 {i + 1} ---")
        print(f"关节角: {angles}")
        print(f"TCP 位置: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
        print("TCP 旋转矩阵:")
        for r in range(3):
            print(f"  [{mat[r, 0]:.4f} {mat[r, 1]:.4f} {mat[r, 2]:.4f}]")


if __name__ == "__main__":
    main()
