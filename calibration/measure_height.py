#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RealSense D435i 深度测高脚本（相机接近水平版）

适用场景:
    相机光轴接近水平，地面出现在画面下方。
    通过下方区域深度反投影，计算相机光心到地面的垂直高度。

原理:
    像素 (u,v) 深度 Z(米) 反投影:
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = Z
    光轴向下俯仰角为 theta 时:
        h = Y * cos(theta) + Z * sin(theta)
    相机接近水平时 theta≈0，h≈Y。
"""

import cv2
import numpy as np
import pyrealsense2 as rs


# ==========================================================
#                    用户配置区域
# ==========================================================

WIDTH = 640
HEIGHT = 480
FPS = 30

# ---------- 地面采样区域 ----------
# 相机接近水平时，地面在画面下方，所以 ROI 放在下半部分
ROI_X1_RATIO = 0.20
ROI_X2_RATIO = 0.80
ROI_Y1_RATIO = 0.60
ROI_Y2_RATIO = 0.95

# ---------- 相机俯仰角 ----------
# 光轴向下俯仰角，单位：度。
# 相机接近水平：填 0.0 或实际的小角度（例如 5.0）
TILT_ANGLE_DEG = 0.0

# ---------- 有效深度范围（米） ----------
MIN_DEPTH = 0.20
MAX_DEPTH = 5.00

# ---------- 滤波 ----------
ENABLE_FILTERS = True

# ---------- 平滑 ----------
SMOOTH_ALPHA = 0.2


# ==========================================================
#                       初始化
# ==========================================================

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
intr = color_profile.get_intrinsics()

fx = intr.fx
fy = intr.fy
cx = intr.ppx
cy = intr.ppy

print("==============================")
print("RealSense D435i Height (near-horizontal)")
print(f"Depth scale : {depth_scale:.6f} m/unit")
print(f"Intrinsics  : fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
print("==============================")

align = rs.align(rs.stream.color)

spatial_filter = rs.spatial_filter() if ENABLE_FILTERS else None
temporal_filter = rs.temporal_filter() if ENABLE_FILTERS else None
hole_filter = rs.hole_filling_filter() if ENABLE_FILTERS else None

x1 = int(WIDTH * ROI_X1_RATIO)
x2 = int(WIDTH * ROI_X2_RATIO)
y1 = int(HEIGHT * ROI_Y1_RATIO)
y2 = int(HEIGHT * ROI_Y2_RATIO)

tilt_rad = np.deg2rad(TILT_ANGLE_DEG)
cos_t = np.cos(tilt_rad)
sin_t = np.sin(tilt_rad)

smoothed_height = None


# ==========================================================
#                       主循环
# ==========================================================

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)

        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        if ENABLE_FILTERS:
            depth_frame = spatial_filter.process(depth_frame)
            depth_frame = temporal_filter.process(depth_frame)
            depth_frame = hole_filter.process(depth_frame)

        color_image = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        depth_m = depth_raw * depth_scale

        # ---------- 反投影计算高度 ----------
        heights = []

        # 为了效率，只遍历 ROI 内像素
        for v in range(y1, y2):
            for u in range(x1, x2):
                Z = depth_m[v, u]
                if Z < MIN_DEPTH or Z > MAX_DEPTH:
                    continue

                # 反投影得到相机坐标系下的 Y（向下）
                Y = (v - cy) * Z / fy

                # 通用高度公式
                h = Y * cos_t + Z * sin_t
                heights.append(h)

        height_text = "N/A"
        valid_ratio = 0.0
        h_med = None

        if len(heights) > 0:
            h_med = float(np.median(heights))
            valid_ratio = len(heights) / float((y2 - y1) * (x2 - x1))

            if smoothed_height is None:
                smoothed_height = h_med
            else:
                smoothed_height = (1 - SMOOTH_ALPHA) * smoothed_height + SMOOTH_ALPHA * h_med

            height_text = f"{smoothed_height:.3f} m"

        print(
            f"[HEIGHT] {height_text}   "
            f"(points {len(heights)}, valid {valid_ratio*100:.1f}%)",
            end="\r"
        )

        # ---------- 可视化 ----------
        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.putText(
            color_image,
            f"Height: {height_text}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2
        )

        cv2.putText(
            color_image,
            f"Tilt: {TILT_ANGLE_DEG:.1f} deg  Valid: {valid_ratio*100:.0f}%",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2
        )

        cv2.putText(
            color_image,
            "ESC to quit",
            (20, HEIGHT - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (200, 200, 200),
            1
        )

        depth_vis = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_raw, alpha=0.03),
            cv2.COLORMAP_JET
        )

        cv2.imshow("RealSense Height (Color)", color_image)
        cv2.imshow("Depth", depth_vis)

        key = cv2.waitKey(1)
        if key == 27:
            break

finally:
    print()
    pipeline.stop()
    cv2.destroyAllWindows()
