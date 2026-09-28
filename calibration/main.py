#!/usr/bin/env python3
"""Single OpenCV entry point for camera/base calibration workflows."""

import json
import os
import re
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation

import config
from solver import solve_basic, solve_refined


def _put_lines(image, lines, x=20, y=40, step=36, scale=0.65):
    for text, color in lines:
        cv2.putText(
            image,
            str(text),
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            2,
            cv2.LINE_AA,
        )
        y += step


def _is_key(key, char):
    return key in (ord(char.lower()), ord(char.upper()))


def load_samples(path=config.SAMPLE_FILE):
    with Path(path).open("r", encoding="utf-8") as file:
        samples = json.load(file)
    if not isinstance(samples, list):
        raise ValueError("sample file must contain a JSON list")
    return samples


def save_samples(samples, path=config.SAMPLE_FILE):
    output_path = Path(path)
    temp_path = output_path.with_suffix(output_path.suffix + ".tmp")
    with temp_path.open("w", encoding="utf-8") as file:
        json.dump(samples, file, indent=2)
        file.write("\n")
    os.replace(temp_path, output_path)


def update_camera_height(height):
    """Atomically update CAMERA_HEIGHT in config.py and this process."""
    config_path = Path(config.__file__).resolve()
    source = config_path.read_text(encoding="utf-8")
    pattern = re.compile(
        r"^CAMERA_HEIGHT\s*=\s*[-+]?(?:\d+(?:\.\d*)?|\.\d+)"
        r"(?:[eE][-+]?\d+)?\s*(?:#.*)?$",
        re.MULTILINE,
    )
    updated, count = pattern.subn(f"CAMERA_HEIGHT = {height:.6f}", source)
    if count != 1:
        raise RuntimeError(f"expected one CAMERA_HEIGHT assignment, found {count}")

    temp_path = config_path.with_suffix(".py.tmp")
    try:
        temp_path.write_text(updated, encoding="utf-8")
        os.chmod(temp_path, config_path.stat().st_mode)
        os.replace(temp_path, config_path)
    finally:
        if temp_path.exists():
            temp_path.unlink()
    config.CAMERA_HEIGHT = float(height)


class HermesPoseClient:
    def __init__(self):
        self.url = (
            f"http://{config.ROBOT_IP}:{config.POSE_HTTP_PORT}"
            f"{config.POSE_API}"
        )

    def read(self):
        with urllib.request.urlopen(self.url, timeout=config.POSE_TIMEOUT_S) as response:
            pose = json.loads(response.read())

        T_map_base = np.eye(4)
        T_map_base[:3, :3] = Rotation.from_euler(
            "xyz",
            [pose["roll"], pose["pitch"], pose["yaw"]],
        ).as_matrix()
        T_map_base[:3, 3] = [pose["x"], pose["y"], pose["z"]]
        return T_map_base


class CalibrationCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.started = False
        stream_config = rs.config()
        stream_config.enable_stream(
            rs.stream.color,
            config.RGB_WIDTH,
            config.RGB_HEIGHT,
            rs.format.bgr8,
            config.FPS,
        )
        try:
            profile = self.pipeline.start(stream_config)
            self.started = True
            intrinsics = (
                profile.get_stream(rs.stream.color)
                .as_video_stream_profile()
                .get_intrinsics()
            )
            self.K = np.array(
                [
                    [intrinsics.fx, 0, intrinsics.ppx],
                    [0, intrinsics.fy, intrinsics.ppy],
                    [0, 0, 1],
                ]
            )
            self.D = np.array(intrinsics.coeffs)
        except Exception:
            if self.started:
                self.pipeline.stop()
            raise

    def read(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asarray(color_frame.get_data()).copy()

    def stop(self):
        if self.started:
            self.pipeline.stop()
            self.started = False


class HeightCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.started = False
        stream_config = rs.config()
        stream_config.enable_stream(
            rs.stream.depth,
            config.DEPTH_WIDTH,
            config.DEPTH_HEIGHT,
            rs.format.z16,
            config.FPS,
        )
        stream_config.enable_stream(
            rs.stream.color,
            config.RGB_WIDTH,
            config.RGB_HEIGHT,
            rs.format.bgr8,
            config.FPS,
        )
        try:
            profile = self.pipeline.start(stream_config)
            self.started = True
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            intrinsics = (
                profile.get_stream(rs.stream.color)
                .as_video_stream_profile()
                .get_intrinsics()
            )
            self.fx = intrinsics.fx
            self.fy = intrinsics.fy
            self.cx = intrinsics.ppx
            self.cy = intrinsics.ppy
            self.align = rs.align(rs.stream.color)
            self.spatial_filter = (
                rs.spatial_filter() if config.ENABLE_DEPTH_FILTERS else None
            )
            self.temporal_filter = (
                rs.temporal_filter() if config.ENABLE_DEPTH_FILTERS else None
            )
            self.hole_filter = (
                rs.hole_filling_filter() if config.ENABLE_DEPTH_FILTERS else None
            )
        except Exception:
            if self.started:
                self.pipeline.stop()
            raise

    def read(self):
        frames = self.align.process(self.pipeline.wait_for_frames())
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None

        if config.ENABLE_DEPTH_FILTERS:
            depth_frame = self.spatial_filter.process(depth_frame)
            depth_frame = self.temporal_filter.process(depth_frame)
            depth_frame = self.hole_filter.process(depth_frame)

        color = np.asanyarray(color_frame.get_data()).copy()
        depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        return color, depth_raw

    def stop(self):
        if self.started:
            self.pipeline.stop()
            self.started = False


class CharucoBoardDetector:
    def __init__(self, K, D):
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, config.ARUCO_DICT)
        )
        self.board = cv2.aruco.CharucoBoard(
            (config.SQUARES_X, config.SQUARES_Y),
            config.SQUARE_LENGTH,
            config.MARKER_LENGTH,
            dictionary,
        )
        self.detector = cv2.aruco.CharucoDetector(self.board)
        self.K = K
        self.D = D

    def detect(self, image):
        corners, ids, _, _ = self.detector.detectBoard(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        )
        if ids is None or len(ids) < config.MIN_CORNERS:
            return None

        ids = ids.flatten()
        object_points = np.asarray(self.board.getChessboardCorners())[ids]
        ok, rotation, translation = cv2.solvePnP(
            object_points,
            corners,
            self.K,
            self.D,
        )
        if not ok:
            return None

        T_camera_board = np.eye(4)
        T_camera_board[:3, :3] = cv2.Rodrigues(rotation)[0]
        T_camera_board[:3, 3] = translation[:, 0]
        return T_camera_board


def select_mode():
    canvas = np.zeros((600, 900, 3), dtype=np.uint8)
    buttons = [
        ((90, 150, 810, 245), "calibration"),
        ((90, 275, 810, 370), "height"),
        ((90, 400, 810, 495), "refine"),
    ]
    clicked = {"mode": None}

    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONUP:
            return
        for (x1, y1, x2, y2), mode in buttons:
            if x1 <= x <= x2 and y1 <= y <= y2:
                clicked["mode"] = mode

    sample_text = "missing"
    if config.SAMPLE_FILE.exists():
        try:
            sample_text = str(len(load_samples()))
        except Exception:
            sample_text = "invalid"
    basic_status = "yes" if config.BASIC_RESULT_FILE.exists() else "no"
    refined_status = "yes" if config.REFINED_RESULT_FILE.exists() else "no"

    cv2.namedWindow(config.WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(config.WINDOW_NAME, on_mouse)

    while True:
        canvas.fill(18)
        _put_lines(
            canvas,
            [("Hermes Camera Calibration Tool", (0, 220, 255))],
            x=175,
            y=75,
            scale=1.0,
        )

        labels = [
            ("[1] ChArUco Calibration", "Capture / load / save / basic solve"),
            ("[2] Camera Height Measurement", "Measure and confirm CAMERA_HEIGHT"),
            ("[3] Refine Calibration", "Solve with the confirmed height"),
        ]
        for ((x1, y1, x2, y2), _mode), (title, subtitle) in zip(buttons, labels):
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (70, 70, 70), -1)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 180, 220), 2)
            cv2.putText(canvas, title, (x1 + 25, y1 + 38), cv2.FONT_HERSHEY_SIMPLEX,
                        0.75, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(canvas, subtitle, (x1 + 25, y1 + 70), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (190, 190, 190), 1, cv2.LINE_AA)

        status_line = (
            f"Samples: {sample_text}    Height: {config.CAMERA_HEIGHT:.3f} m    "
            f"Basic: {basic_status}    Refined: {refined_status}"
        )
        cv2.putText(canvas, status_line, (90, 545), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (170, 170, 170), 1, cv2.LINE_AA)
        cv2.putText(canvas, "ESC/Q: Quit", (90, 575), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (140, 140, 140), 1, cv2.LINE_AA)
        cv2.imshow(config.WINDOW_NAME, canvas)

        if clicked["mode"]:
            return clicked["mode"]
        key = cv2.waitKey(30) & 255
        if key == ord("1"):
            return "calibration"
        if key == ord("2"):
            return "height"
        if key == ord("3"):
            return "refine"
        if key == 27 or _is_key(key, "q"):
            return "quit"


def run_calibration_mode():
    camera = CalibrationCamera()
    try:
        robot = HermesPoseClient()
        board = CharucoBoardDetector(camera.K, camera.D)
        samples = []
        status = "Ready"

        while True:
            image = camera.read()
            if image is None:
                continue
            observation = board.detect(image)

            robot_error = ""
            try:
                T_map_base = robot.read()
                robot_ok = True
            except (urllib.error.URLError, TimeoutError, ValueError, KeyError) as exc:
                T_map_base = None
                robot_ok = False
                robot_error = str(exc)

            panel = np.zeros((image.shape[0], config.PANEL_WIDTH, 3), np.uint8)
            lines = [
                (f"Samples: {len(samples)}", (0, 255, 0)),
                ("Board: " + ("OK" if observation is not None else "FAIL"),
                 (0, 255, 0) if observation is not None else (0, 0, 255)),
                ("Robot: " + ("OK" if robot_ok else "FAIL"),
                 (0, 255, 0) if robot_ok else (0, 0, 255)),
                ("", (255, 255, 255)),
                ("SPACE  Capture", (255, 255, 255)),
                ("D      Delete last", (255, 255, 255)),
                ("S      Save samples", (255, 255, 255)),
                ("L      Load samples", (255, 255, 255)),
                ("ENTER  Basic solve", (255, 255, 255)),
                ("R      Reset", (255, 255, 255)),
                ("B/ESC  Back", (255, 255, 255)),
                ("", (255, 255, 255)),
                (status, (0, 220, 255)),
            ]
            _put_lines(panel, lines, step=33, scale=0.58)
            if robot_error:
                cv2.putText(panel, robot_error[:48], (20, image.shape[0] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            cv2.imshow(config.WINDOW_NAME, np.hstack([image, panel]))
            key = cv2.waitKey(1) & 255

            if key == 27 or _is_key(key, "b"):
                return
            if key == 32:
                if observation is not None and robot_ok:
                    samples.append(
                        {
                            "T_map_base": T_map_base.tolist(),
                            "T_camera_board": observation.tolist(),
                            "time": time.time(),
                        }
                    )
                    status = f"Captured sample {len(samples)}"
                else:
                    status = "Capture rejected: board/robot unavailable"
            elif _is_key(key, "d"):
                if samples:
                    samples.pop()
                    status = "Deleted last sample"
            elif _is_key(key, "s"):
                try:
                    save_samples(samples)
                    status = f"Saved {len(samples)} samples"
                except OSError as exc:
                    status = f"Save failed: {exc}"
            elif _is_key(key, "l"):
                try:
                    samples = load_samples()
                    status = f"Loaded {len(samples)} samples"
                except (OSError, ValueError, json.JSONDecodeError) as exc:
                    status = f"Load failed: {exc}"
            elif _is_key(key, "r"):
                samples = []
                status = "Samples reset"
            elif key in (10, 13):
                try:
                    T_base_camera, cost = solve_basic(samples, config.BASIC_RESULT_FILE)
                    print("T_base_camera (basic) =")
                    print(T_base_camera)
                    print("cost", cost)
                    status = f"Basic result saved; cost={cost:.3e}"
                except Exception as exc:
                    status = f"Solve failed: {exc}"
    finally:
        camera.stop()


def run_height_mode():
    camera = HeightCamera()
    x1 = int(config.RGB_WIDTH * config.ROI_X1_RATIO)
    x2 = int(config.RGB_WIDTH * config.ROI_X2_RATIO)
    y1 = int(config.RGB_HEIGHT * config.ROI_Y1_RATIO)
    y2 = int(config.RGB_HEIGHT * config.ROI_Y2_RATIO)
    tilt_rad = np.deg2rad(config.TILT_ANGLE_DEG)
    cos_t = np.cos(tilt_rad)
    sin_t = np.sin(tilt_rad)
    smoothed_height = None
    pending_height = None
    status = "C: confirm current height"

    try:
        while True:
            color_image, depth_raw = camera.read()
            if color_image is None or depth_raw is None:
                continue
            depth_m = depth_raw * camera.depth_scale
            heights = []

            # Preserve the original per-pixel height calculation.
            for v in range(y1, y2):
                for u in range(x1, x2):
                    Z = depth_m[v, u]
                    if Z < config.MIN_DEPTH or Z > config.MAX_DEPTH:
                        continue
                    Y = (v - camera.cy) * Z / camera.fy
                    heights.append(Y * cos_t + Z * sin_t)

            valid_ratio = 0.0
            if heights:
                median_height = float(np.median(heights))
                valid_ratio = len(heights) / float((y2 - y1) * (x2 - x1))
                if smoothed_height is None:
                    smoothed_height = median_height
                else:
                    smoothed_height = (
                        (1 - config.SMOOTH_ALPHA) * smoothed_height
                        + config.SMOOTH_ALPHA * median_height
                    )

            height_text = "N/A" if smoothed_height is None else f"{smoothed_height:.3f} m"
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(color_image, f"Height: {height_text}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.putText(color_image,
                        f"Tilt: {config.TILT_ANGLE_DEG:.1f} deg  Valid: {valid_ratio*100:.0f}%",
                        (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 0), 2)

            depth_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_raw, alpha=0.03),
                cv2.COLORMAP_JET,
            )
            panel = np.zeros((color_image.shape[0], config.PANEL_WIDTH, 3), np.uint8)
            lines = [
                ("Camera Height", (0, 220, 255)),
                (f"Current: {height_text}", (0, 255, 0)),
                (f"Configured: {config.CAMERA_HEIGHT:.3f} m", (255, 255, 255)),
                (f"Valid points: {valid_ratio*100:.1f}%", (255, 255, 255)),
                ("", (255, 255, 255)),
                ("C      Confirm height", (255, 255, 255)),
                ("Y      Save when prompted", (255, 255, 255)),
                ("N      Cancel confirmation", (255, 255, 255)),
                ("B/ESC  Back", (255, 255, 255)),
                ("", (255, 255, 255)),
                (status, (0, 220, 255)),
            ]
            _put_lines(panel, lines, step=37, scale=0.57)
            cv2.imshow(config.WINDOW_NAME, np.hstack([color_image, depth_vis, panel]))
            key = cv2.waitKey(1) & 255

            if key == 27 or _is_key(key, "b"):
                return
            if _is_key(key, "c"):
                if smoothed_height is None:
                    status = "No valid height to confirm"
                else:
                    pending_height = float(smoothed_height)
                    status = f"Write {pending_height:.3f} m? Y/N"
            elif _is_key(key, "n"):
                pending_height = None
                status = "Confirmation cancelled"
            elif _is_key(key, "y") and pending_height is not None:
                try:
                    update_camera_height(pending_height)
                    status = f"Saved CAMERA_HEIGHT={pending_height:.3f} m"
                    pending_height = None
                except (OSError, RuntimeError) as exc:
                    status = f"Config update failed: {exc}"
    finally:
        camera.stop()


def run_refine_mode():
    canvas = np.zeros((650, 980, 3), dtype=np.uint8)
    try:
        samples = load_samples()
        T_base_camera, T_map_board, cost = solve_refined(
            samples=samples,
            output_file=config.REFINED_RESULT_FILE,
            camera_height=config.CAMERA_HEIGHT,
            board_height=config.BOARD_HEIGHT,
            initial_x=config.REFINED_INITIAL_X,
            initial_y=config.REFINED_INITIAL_Y,
            initial_roll=config.REFINED_INITIAL_ROLL,
            initial_pitch=config.REFINED_INITIAL_PITCH,
            initial_yaw=config.REFINED_INITIAL_YAW,
            camera_z_weight=config.W_CAMERA_Z,
            board_z_weight=config.W_BOARD_Z,
        )
        translation = T_base_camera[:3, 3]
        orientation = Rotation.from_matrix(T_base_camera[:3, :3]).as_euler(
            "xyz", degrees=True
        )
        print("T_base_camera (refined) =")
        print(T_base_camera)
        print("T_map_board =")
        print(T_map_board)
        print("cost", cost)
        lines = [
            ("Refined Calibration Complete", (0, 255, 0)),
            (f"Samples: {len(samples)}", (255, 255, 255)),
            (f"Cost: {cost:.6e}", (255, 255, 255)),
            (f"Height constraint: {config.CAMERA_HEIGHT:.6f} m", (255, 255, 255)),
            ("", (255, 255, 255)),
            ("T_base_camera translation (m)", (0, 220, 255)),
            (f"x={translation[0]: .6f}  y={translation[1]: .6f}  z={translation[2]: .6f}",
             (255, 255, 255)),
            ("T_base_camera roll/pitch/yaw (deg)", (0, 220, 255)),
            (f"r={orientation[0]: .3f}  p={orientation[1]: .3f}  y={orientation[2]: .3f}",
             (255, 255, 255)),
            ("", (255, 255, 255)),
            (f"Saved: {config.REFINED_RESULT_FILE.name}", (255, 255, 255)),
            ("B/ESC/ENTER  Back", (180, 180, 180)),
        ]
    except Exception as exc:
        lines = [
            ("Refined Calibration Failed", (0, 0, 255)),
            (str(exc), (255, 255, 255)),
            ("", (255, 255, 255)),
            ("B/ESC/ENTER  Back", (180, 180, 180)),
        ]

    while True:
        canvas.fill(18)
        _put_lines(canvas, lines, x=45, y=65, step=45, scale=0.7)
        cv2.imshow(config.WINDOW_NAME, canvas)
        key = cv2.waitKey(30) & 255
        if key in (10, 13, 27) or _is_key(key, "b"):
            return


def show_mode_error(mode, error):
    canvas = np.zeros((400, 900, 3), dtype=np.uint8)
    lines = [
        (f"Could not start {mode} mode", (0, 0, 255)),
        (str(error), (255, 255, 255)),
        ("", (255, 255, 255)),
        ("B/ESC/ENTER  Back", (180, 180, 180)),
    ]
    while True:
        canvas.fill(18)
        _put_lines(canvas, lines, x=35, y=70, step=55, scale=0.7)
        cv2.imshow(config.WINDOW_NAME, canvas)
        key = cv2.waitKey(30) & 255
        if key in (10, 13, 27) or _is_key(key, "b"):
            return


def main():
    try:
        while True:
            mode = select_mode()
            if mode == "quit":
                break
            try:
                if mode == "calibration":
                    run_calibration_mode()
                elif mode == "height":
                    run_height_mode()
                elif mode == "refine":
                    run_refine_mode()
            except Exception as exc:
                show_mode_error(mode, exc)
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
