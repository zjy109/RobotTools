#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立重解算脚本
读取 samples.json，用实测相机高度作为绝对约束，重新求解 T_base_camera。

用法：
    直接修改下面的用户配置区，然后 python refine_solve.py
"""

import json
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


# =========================================================
# 用户配置区
# =========================================================

SAMPLE_FILE = "samples.json"
OUTPUT_FILE = "T_base_camera_refined.yaml"

# ---- 实测值（非常重要）----
# 相机光心距离地面的真实高度（米），由深度相机 + 地面测量得到
CAMERA_HEIGHT = 0.347

# 标定板中心高度（米）。如果你知道就填，不知道填 None（会用第一个样本推算）
BOARD_HEIGHT = None         # 例如 0.72

# ---- 初始猜测 ----
# T_base_camera 的初始平移 [x, y, z]（米）
# X0_T = [0.25, 0.0, CAMERA_HEIGHT]   # z 直接用实测值
X0_T = [0.0, 0.0, CAMERA_HEIGHT]
# T_base_camera 的初始旋转 [roll, pitch, yaw]（弧度）
X0_R = [0.0, 0.0, 0.0]

# ---- 约束权重 ----
# 越大越"硬"。相机高度约束建议 1e3 ~ 1e4。
W_CAMERA_Z = 1.0e4
# 如果 BOARD_HEIGHT 不为 None，也可以加一个板子高度约束（一般不需要同时加）
W_BOARD_Z = 0.0


# =========================================================
# 数学工具
# =========================================================

def vec_to_T(v):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[3:6]).as_matrix()
    T[:3, 3] = v[:3]
    return T


def T_to_vec(T):
    return np.r_[
        T[:3, 3],
        Rotation.from_matrix(T[:3, :3]).as_rotvec()
    ]


# =========================================================
# 主流程
# =========================================================

def main():

    samples = json.load(open(SAMPLE_FILE))
    print(f"[info] loaded {len(samples)} samples from {SAMPLE_FILE}")

    if len(samples) < 5:
        raise RuntimeError("Need at least 5 samples to solve")

    # ---- 组合初始值 ----
    X0 = np.r_[X0_T, X0_R]

    # 标定板初始猜测：用第一个样本 + 当前 X0 推算
    T_map_board_0 = (
        np.array(samples[0]["T_map_base"])
        @ vec_to_T(X0)
        @ np.array(samples[0]["T_camera_board"])
    )
    B0 = T_to_vec(T_map_board_0)

    if BOARD_HEIGHT is not None:
        B0[2] = BOARD_HEIGHT

    x0 = np.r_[X0, B0]

    print("[info] initial guess:")
    print(f"   T_base_camera  t = {X0[:3]}")
    print(f"   T_base_camera rpy = {X0[3:6]}")
    print(f"   T_map_board    t = {B0[:3]}")
    print(f"   T_map_board   rpy = {B0[3:6]}")

    # ---- 残差函数 ----
    def residual(x):
        T_base_camera = vec_to_T(x[:6])
        T_map_board = vec_to_T(x[6:])

        res = []

        # (1) 绝对高度约束：钉死相机 Z
        if W_CAMERA_Z > 0:
            res.append((x[2] - CAMERA_HEIGHT) * W_CAMERA_Z)

        # (2) 可选：钉死标定板 Z
        if W_BOARD_Z > 0 and BOARD_HEIGHT is not None:
            res.append((x[8] - BOARD_HEIGHT) * W_BOARD_Z)

        # (3) 主残差：所有样本的几何一致性
        for s in samples:
            T_map_base = np.array(s["T_map_base"])
            T_camera_board = np.array(s["T_camera_board"])

            pred = T_map_base @ T_base_camera @ T_camera_board
            delta = np.linalg.inv(T_map_board) @ pred

            res.extend(delta[:3, 3])
            res.extend(
                Rotation.from_matrix(delta[:3, :3]).as_rotvec()
            )

        return np.array(res)

    # ---- 求解 ----
    result = least_squares(residual, x0, verbose=1)

    T = vec_to_T(result.x[:6])
    T_board = vec_to_T(result.x[6:])

    # ---- 输出 ----
    print("\n================ RESULT ================")
    print("T_base_camera =")
    print(np.array2string(T, precision=6, suppress_small=True))

    print(f"\nCamera position  (m): {T[:3, 3]}")
    print(
        "Camera orientation (deg): "
        f"{Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)}"
    )

    print(f"\nBoard position  (m): {T_board[:3, 3]}")
    print(
        "Board orientation (deg): "
        f"{Rotation.from_matrix(T_board[:3, :3]).as_euler('xyz', degrees=True)}"
    )

    print(f"\ncost    : {result.cost:.6e}")
    print(f"samples : {len(samples)}")
    print(f"camera height (constraint) : {CAMERA_HEIGHT} m")
    print(f"camera height (solved)     : {T[2, 3]:.4f} m")

    # ---- 保存 ----
    with open(OUTPUT_FILE, "w") as f:
        yaml.dump(
            {
                "T_base_camera": T.tolist(),
                "T_map_board":   T_board.tolist(),
                "camera_height_measured": CAMERA_HEIGHT,
                "camera_height_solved":   float(T[2, 3]),
                "cost":    float(result.cost),
                "samples": len(samples),
            },
            f,
        )

    print(f"\n[saved] {OUTPUT_FILE}")


if __name__ == "__main__":
    main()