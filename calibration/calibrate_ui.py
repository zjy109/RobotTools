
#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
import json
import time
import urllib.request
import numpy as np
import pyrealsense2 as rs

from scipy.spatial.transform import Rotation

import config
from solver import solve


class Hermes:

    def __init__(self):
        self.url = (
            f"http://{config.ROBOT_IP}:1448"
            f"{config.POSE_API}"
        )

    def read(self):

        with urllib.request.urlopen(
            self.url, timeout=2
        ) as f:
            p=json.loads(f.read())

        T=np.eye(4)

        T[:3,:3]=Rotation.from_euler(
            "xyz",
            [
                p["roll"],
                p["pitch"],
                p["yaw"]
            ]
        ).as_matrix()

        T[:3,3]=[
            p["x"],
            p["y"],
            p["z"]
        ]

        return T


class Camera:

    def __init__(self):

        self.pipe=rs.pipeline()

        cfg=rs.config()

        cfg.enable_stream(
            rs.stream.color,
            config.RGB_WIDTH,
            config.RGB_HEIGHT,
            rs.format.bgr8,
            config.FPS
        )

        profile=self.pipe.start(cfg)

        intr=profile.get_stream(
            rs.stream.color
        ).as_video_stream_profile().get_intrinsics()

        self.K=np.array([
            [intr.fx,0,intr.ppx],
            [0,intr.fy,intr.ppy],
            [0,0,1]
        ])

        self.D=np.array(
            intr.coeffs
        )


    def read(self):

        frames=self.pipe.wait_for_frames()

        return np.asarray(
            frames.get_color_frame().get_data()
        ).copy()


class Board:

    def __init__(self,K,D):

        dic=cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco,config.ARUCO_DICT)
        )

        self.board=cv2.aruco.CharucoBoard(
            (
                config.SQUARES_X,
                config.SQUARES_Y
            ),
            config.SQUARE_LENGTH,
            config.MARKER_LENGTH,
            dic
        )

        self.det=cv2.aruco.CharucoDetector(
            self.board
        )

        self.K=K
        self.D=D


    def detect(self,img):

        corners,ids,_,_=self.det.detectBoard(
            cv2.cvtColor(
                img,
                cv2.COLOR_BGR2GRAY
            )
        )

        if ids is None or len(ids)<config.MIN_CORNERS:
            return None

        ids=ids.flatten()

        obj=np.asarray(
            self.board.getChessboardCorners()
        )[ids]

        ok,r,t=cv2.solvePnP(
            obj,
            corners,
            self.K,
            self.D
        )

        if not ok:
            return None

        T=np.eye(4)
        T[:3,:3]=cv2.Rodrigues(r)[0]
        T[:3,3]=t[:,0]

        return T


# class Board:

#     def __init__(self, K, D):
#         dic = cv2.aruco.getPredefinedDictionary(
#             getattr(cv2.aruco, config.ARUCO_DICT)
#         )
#         self.board = cv2.aruco.CharucoBoard(
#             (config.SQUARES_X, config.SQUARES_Y),
#             config.SQUARE_LENGTH,
#             config.MARKER_LENGTH,
#             dic
#         )
#         # 改用旧版 API 组合
#         self.detector = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters())
#         self.K = K
#         self.D = D

#     def detect(self, img):
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
#         # 1. 先检测 ArUco 码
#         corners, ids, rejected = self.detector.detectMarkers(gray)
        
#         if ids is None or len(ids) < 2: # 至少要有2个码才能插值
#             return None

#         # 2. 插值 ChArUco 角点
#         ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
#             corners, ids, gray, self.board
#         )

#         if ret is None or ret < config.MIN_CORNERS:
#             return None

#         # 3. PnP 求解
#         obj = np.asarray(self.board.getChessboardCorners())[charuco_ids]
        
#         ok, r, t = cv2.solvePnP(
#             obj, charuco_corners, self.K, self.D
#         )

#         if not ok:
#             return None

#         T = np.eye(4)
#         T[:3, :3] = cv2.Rodrigues(r)[0]
#         T[:3, 3] = t[:, 0]

#         return T


def main():

    cam=Camera()
    robot=Hermes()
    board=Board(cam.K,cam.D)

    samples=[]

    while True:

        img=cam.read()

        obs=board.detect(img)

        try:
            Tbase=robot.read()
            robot_ok=True
        except:
            Tbase=None
            robot_ok=False


        panel=np.zeros(
            (config.RGB_HEIGHT,420,3),
            np.uint8
        )

        texts=[
            f"Samples: {len(samples)}",
            "Board: "+("OK" if obs is not None else "FAIL"),
            "Robot: "+("OK" if robot_ok else "FAIL"),
            "",
            "SPACE capture",
            "D delete",
            "S save",
            "L load",
            "ENTER solve",
            "R reset",
            "ESC quit"
        ]

        y=40
        for t in texts:
            cv2.putText(
                panel,t,
                (20,y),
                cv2.FONT_HERSHEY_SIMPLEX,
                .7,
                (0,255,0),
                2
            )
            y+=45


        view=np.hstack(
            [img,panel]
        )

        cv2.imshow(
            config.WINDOW_NAME,
            view
        )

        key=cv2.waitKey(1)&255


        if key==27:
            break

        elif key==32:

            if obs is not None and robot_ok:

                samples.append(
                    {
                    "T_map_base":
                        Tbase.tolist(),
                    "T_camera_board":
                        obs.tolist(),
                    "time":
                        time.time()
                    }
                )

                print(
                    "capture",
                    len(samples)
                )


        elif key==ord("d"):

            if samples:
                samples.pop()


        elif key==ord("s"):

            json.dump(
                samples,
                open(config.SAMPLE_FILE,"w"),
                indent=2
            )

            print("saved")


        elif key==ord("l"):

            samples=json.load(
                open(config.SAMPLE_FILE)
            )

            print(
                "loaded",
                len(samples)
            )


        elif key==ord("r"):

            samples=[]


        elif key==13:

            T,cost=solve(
                samples,
                config.RESULT_FILE
            )

            print(T)
            print("cost",cost)


    cv2.destroyAllWindows()


if __name__=="__main__":
    main()
