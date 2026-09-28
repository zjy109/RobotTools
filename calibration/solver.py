
# -*- coding: utf-8 -*-

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def vec_to_T(v):
    T = np.eye(4)
    T[:3,:3] = Rotation.from_rotvec(v[3:6]).as_matrix()
    T[:3,3] = v[:3]
    return T


def T_to_vec(T):
    return np.r_[
        T[:3,3],
        Rotation.from_matrix(T[:3,:3]).as_rotvec()
    ]


def solve(samples, output_file):

    if len(samples) < 5:
        raise RuntimeError("Need more samples")

    # initial camera-base guess
    X0 = np.zeros(6)
    X0[2] = 0.35

    # board initial guess from first sample
    B0 = T_to_vec(
        np.array(samples[0]["T_map_base"])
        @ np.array(samples[0]["T_camera_board"])
    )

    x0 = np.r_[X0, B0]

    def residual(x):

        T_base_camera = vec_to_T(x[:6])
        T_map_board = vec_to_T(x[6:])

        res=[]

        for s in samples:

            T_map_base = np.array(
                s["T_map_base"]
            )

            T_camera_board = np.array(
                s["T_camera_board"]
            )

            pred = (
                T_map_base
                @ T_base_camera
                @ T_camera_board
            )

            delta = np.linalg.inv(T_map_board) @ pred

            res.extend(delta[:3,3])

            rot = Rotation.from_matrix(
                delta[:3,:3]
            ).as_rotvec()

            res.extend(rot)

        return np.array(res)


    result = least_squares(
        residual,
        x0,
        verbose=1
    )

    T = vec_to_T(
        result.x[:6]
    )

    with open(output_file,"w") as f:
        yaml.dump(
            {
                "T_base_camera":
                    T.tolist(),
                "cost":
                    float(result.cost),
                "samples":
                    len(samples)
            },
            f
        )

    return T, result.cost
