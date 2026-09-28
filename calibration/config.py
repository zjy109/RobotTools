"""Configuration for the Hermes/RealSense calibration tool."""

from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent

# Hermes / SLAMWARE
ROBOT_IP = "192.168.11.1"
POSE_HTTP_PORT = 1448
POSE_API = "/api/core/slam/v1/localization/pose"
POSE_TIMEOUT_S = 2.0

# RealSense D435i
RGB_WIDTH = 640
RGB_HEIGHT = 480
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
FPS = 30

# ChArUco board
ARUCO_DICT = "DICT_5X5_100"
SQUARES_X = 7
SQUARES_Y = 5
SQUARE_LENGTH = 0.05
MARKER_LENGTH = 0.035
MIN_CORNERS = 9

# Input/output
SAMPLE_FILE = BASE_DIR / "samples.json"
BASIC_RESULT_FILE = BASE_DIR / "T_base_camera_basic.yaml"
REFINED_RESULT_FILE = BASE_DIR / "T_base_camera_refined.yaml"

# Height measurement
ROI_X1_RATIO = 0.20
ROI_X2_RATIO = 0.80
ROI_Y1_RATIO = 0.60
ROI_Y2_RATIO = 0.95
TILT_ANGLE_DEG = 0.0
MIN_DEPTH = 0.20
MAX_DEPTH = 5.00
ENABLE_DEPTH_FILTERS = True
SMOOTH_ALPHA = 0.2

# Confirmed camera height, written by the height-measurement UI.
CAMERA_HEIGHT = 0.347000
BOARD_HEIGHT = None
REFINED_INITIAL_X = 0.0
REFINED_INITIAL_Y = 0.0
REFINED_INITIAL_ROLL = 0.0
REFINED_INITIAL_PITCH = 0.0
REFINED_INITIAL_YAW = 0.0
W_CAMERA_Z = 1.0e4
W_BOARD_Z = 0.0

# UI
WINDOW_NAME = "Hermes Camera Calibration Tool"
PANEL_WIDTH = 420
