"""Calibration solvers shared by the interactive application."""

from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def vec_to_T(v):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[3:6]).as_matrix()
    T[:3, 3] = v[:3]
    return T


def T_to_vec(T):
    return np.r_[
        T[:3, 3],
        Rotation.from_matrix(T[:3, :3]).as_rotvec(),
    ]


def _write_yaml(output_file, payload):
    output_path = Path(output_file)
    with output_path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(payload, file, sort_keys=False)


def solve_basic(samples, output_file):
    """Run the original unconstrained solver and save its result."""
    if len(samples) < 5:
        raise RuntimeError("Need at least 5 samples to solve")

    # Keep the original initial camera-base guess.
    X0 = np.zeros(6)
    X0[2] = 0.35

    # Keep the original board initial guess.
    B0 = T_to_vec(
        np.array(samples[0]["T_map_base"])
        @ np.array(samples[0]["T_camera_board"])
    )
    x0 = np.r_[X0, B0]

    def residual(x):
        T_base_camera = vec_to_T(x[:6])
        T_map_board = vec_to_T(x[6:])
        res = []

        for sample in samples:
            T_map_base = np.array(sample["T_map_base"])
            T_camera_board = np.array(sample["T_camera_board"])
            pred = T_map_base @ T_base_camera @ T_camera_board
            delta = np.linalg.inv(T_map_board) @ pred
            res.extend(delta[:3, 3])
            res.extend(Rotation.from_matrix(delta[:3, :3]).as_rotvec())

        return np.array(res)

    result = least_squares(residual, x0, verbose=1)
    T_base_camera = vec_to_T(result.x[:6])

    _write_yaml(
        output_file,
        {
            "method": "basic",
            "T_base_camera": T_base_camera.tolist(),
            "cost": float(result.cost),
            "samples": len(samples),
        },
    )
    return T_base_camera, result.cost


def solve_refined(
    samples,
    output_file,
    camera_height,
    board_height=None,
    initial_x=0.0,
    initial_y=0.0,
    initial_roll=0.0,
    initial_pitch=0.0,
    initial_yaw=0.0,
    camera_z_weight=1.0e4,
    board_z_weight=0.0,
):
    """Run the original height-constrained solver and save its result."""
    if len(samples) < 5:
        raise RuntimeError("Need at least 5 samples to solve")

    X0 = np.array(
        [
            initial_x,
            initial_y,
            camera_height,
            initial_roll,
            initial_pitch,
            initial_yaw,
        ],
        dtype=float,
    )

    T_map_board_0 = (
        np.array(samples[0]["T_map_base"])
        @ vec_to_T(X0)
        @ np.array(samples[0]["T_camera_board"])
    )
    B0 = T_to_vec(T_map_board_0)
    if board_height is not None:
        B0[2] = board_height
    x0 = np.r_[X0, B0]

    def residual(x):
        T_base_camera = vec_to_T(x[:6])
        T_map_board = vec_to_T(x[6:])
        res = []

        if camera_z_weight > 0:
            res.append((x[2] - camera_height) * camera_z_weight)
        if board_z_weight > 0 and board_height is not None:
            res.append((x[8] - board_height) * board_z_weight)

        for sample in samples:
            T_map_base = np.array(sample["T_map_base"])
            T_camera_board = np.array(sample["T_camera_board"])
            pred = T_map_base @ T_base_camera @ T_camera_board
            delta = np.linalg.inv(T_map_board) @ pred
            res.extend(delta[:3, 3])
            res.extend(Rotation.from_matrix(delta[:3, :3]).as_rotvec())

        return np.array(res)

    result = least_squares(residual, x0, verbose=1)
    T_base_camera = vec_to_T(result.x[:6])
    T_map_board = vec_to_T(result.x[6:])

    _write_yaml(
        output_file,
        {
            "method": "height_constrained",
            "T_base_camera": T_base_camera.tolist(),
            "T_map_board": T_map_board.tolist(),
            "camera_height_measured": float(camera_height),
            "camera_height_solved": float(T_base_camera[2, 3]),
            "cost": float(result.cost),
            "samples": len(samples),
        },
    )
    return T_base_camera, T_map_board, result.cost
