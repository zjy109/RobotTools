# -*- coding: utf-8 -*-

# ==========================
# Hermes Slamware
# ==========================

ROBOT_IP = "192.168.11.1"

POSE_API = (
    "/api/core/slam/v1/localization/pose"
)


# ==========================
# RealSense D435i
# ==========================

RGB_WIDTH = 640
RGB_HEIGHT = 480
FPS = 30



# ==========================
# ChArUco Board
# ==========================

ARUCO_DICT = "DICT_5X5_100"

SQUARES_X = 7
SQUARES_Y = 5

# meter
SQUARE_LENGTH = 0.05

# meter
# 必须和实际打印板一致
MARKER_LENGTH = 0.035



# ==========================
# Calibration
# ==========================

SAMPLE_FILE = "samples.json"

RESULT_FILE = (
    "T_base_camera.yaml"
)


MIN_CORNERS = 9


# ==========================
# UI
# ==========================

WINDOW_NAME = (
    "Hermes ChArUco Calibration"
)



# # -*- coding: utf-8 -*-

# ROBOT_IP = "192.168.11.1"
# POSE_API = "/api/core/slam/v1/localization/pose"

# RGB_WIDTH = 1280
# RGB_HEIGHT = 720
# FPS = 15

# ARUCO_DICT = "DICT_5X5_100"
# SQUARES_X = 5
# SQUARES_Y = 7
# SQUARE_LENGTH = 0.05
# MARKER_LENGTH = 0.025

# SAMPLE_FILE = "samples.json"
# RESULT_FILE = "T_base_camera.yaml"

# MIN_CORNERS = 8

# WINDOW_NAME = "Hermes ChArUco Calibration"

