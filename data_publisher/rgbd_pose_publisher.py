#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense RGB-D + Hermes pose time synchronization publisher

Architecture
------------
Main thread:
    RealSense -> camera capture timestamp -> interpolate base pose -> apply extrinsic
    -> ZMQ publish [topic, msgpack meta, rgb bytes, depth bytes]

Child thread:
    Poll Hermes /pose API -> timestamp each returned pose on PC clock
    -> append to shared pose buffer

Time conventions
----------------
1) Camera timestamp:
   RealSense GLOBAL_TIME / SYSTEM_TIME, converted to Unix nanoseconds.
   This is the master timestamp of each published observation.

2) Robot pose timestamp:
   Estimated as midpoint of HTTP request/response wall-clock timestamps:
       t_pose = (t_request_unix + t_response_unix) / 2
   RTT itself is measured by monotonic clock.

3) Synchronized pose:
   For each camera timestamp tc, find P0 and P1 such that:
       P0.t <= tc <= P1.t
   Interpolate translation linearly and rotation with quaternion SLERP.

IMPORTANT
---------
- Fill T_BASE_CAMERA_CONFIG with your calibrated T_base_camera matrix,
  or set EXTRINSIC_FILE to a .npy/.json file.
- The SLAMTEC REST pose itself does NOT include a measurement timestamp;
  HTTP midpoint is therefore an estimate. Keep the sync diagnostics.
"""

from __future__ import annotations

# ============================================================================
# USER CONFIGURATION
# ============================================================================

# ---------- ZMQ ----------
ZMQ_ENDPOINT = "ipc:///tmp/rgbd_pose.ipc"
# ZMQ_ENDPOINT = "tcp://0.0.0.0:5555"
ZMQ_TOPIC = "rgbd.pose"
ZMQ_SNDHWM = 2

# ---------- Hermes / SLAMTEC ----------
ROBOT_IP = "192.168.11.1"
POSE_API = "/api/core/slam/v1/localization/pose"
POSE_HTTP_PORT = 1448
POSE_HTTP_TIMEOUT_S = 1.0
POSE_POLL_HZ = 30.0                 # Desired polling rate; actual rate is API/RTT limited.
POSE_BUFFER_SIZE = 300              # Number of pose samples kept in memory.

# ---------- Pose timing / quality ----------
POSE_SYNC_WAIT_TIMEOUT_S = 0.12     # Max wait for the pose after a camera frame.
MAX_POSE_SIDE_DT_MS = 100.0         # Max |camera-pose| on either side for valid interpolation.
MAX_POSE_RTT_MS = 40.0              # Reject pose samples whose HTTP RTT exceeds this.
PUBLISH_UNSYNCED_FRAMES = False     # False: only output frames with a valid synchronized pose.

# ---------- RealSense ----------
CAMERA_SERIAL = None                # None = first RealSense found.
RGB_WIDTH = 640
RGB_HEIGHT = 480
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
FPS = 30
ALIGN_DEPTH_TO_COLOR = True
CAMERA_WARMUP_FRAMES = 30
REQUIRE_GLOBAL_OR_SYSTEM_TIME = True

# Keep publisher-side depth processing conservative: configure the D435i sensor,
# but do not apply software spatial/temporal/hole-filling/decimation filters here.
# Downstream consumers (LingBot, nvblox, etc.) can preprocess the same aligned
# sensor depth differently for their own needs.
DEPTH_VISUAL_PRESET = "Medium Density"   # None = keep device default.
DEPTH_EMITTER_ENABLED = True              # Enable the D4xx IR projector when supported.
DEPTH_AUTO_EXPOSURE = True                # Keep stereo depth auto exposure enabled.

# ---------- Metadata / diagnostics ----------
PUBLISH_CAMERA_DEBUG = True         # User-requested switch for camera_debug metadata.
PRINT_EVERY_N_FRAMES = 30

# ---------- Extrinsic: T_base_camera ----------
# This calibration was obtained from RGB images, therefore ``camera`` here is
# the RealSense color optical frame. With ALIGN_DEPTH_TO_COLOR=True the published
# depth image is expressed in the same projection geometry.
# Convention:
#   p_base = T_base_camera @ p_camera
# Then:
#   T_map_camera = T_map_base @ T_base_camera
#
# Option A: set EXTRINSIC_FILE to .npy or .json.
# JSON may be:
#   1) a raw 4x4 list
#   2) {"T_base_camera": [[...], ...]}
#   3) {"T": [[...], ...]}
#   4) {"matrix": [[...], ...]}
#
# Option B: leave EXTRINSIC_FILE=None and paste your calibrated matrix below.
EXTRINSIC_FILE = None
EXTRINSIC_FILE_IS_T_BASE_CAMERA = True
T_BASE_CAMERA_CONFIG = [
    [0.001996648567855641, -0.010413978966730186, 0.9999437796379237, 0.2490650981407886],
    [-0.9996866750515288, 0.024929164929483688, 0.0022557615870130165, 0.047238149392801],
    [-0.02495125485652616, -0.9996349762678368, -0.010360941225852738, 0.347],
    [0.0, 0.0, 0.0, 1.0],
]

# ============================================================================
# IMPORTS
# ============================================================================

import bisect
import json
import os
import signal
import threading
import time
import urllib.request
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import msgpack
import numpy as np
import pyrealsense2 as rs
import zmq
from scipy.spatial.transform import Rotation, Slerp


# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass(frozen=True)
class PoseSample:
    seq: int
    timestamp_ns: int          # Estimated pose time on PC Unix/realtime clock.
    request_time_ns: int
    response_time_ns: int
    rtt_ns: int                # HTTP round-trip duration from monotonic clock.

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class SyncResult:
    valid: bool
    reason: str
    T_map_base: Optional[np.ndarray]

    p0: Optional[PoseSample] = None
    p1: Optional[PoseSample] = None
    alpha: Optional[float] = None
    dt_before_ms: Optional[float] = None
    dt_after_ms: Optional[float] = None


# ============================================================================
# EXTRINSIC
# ============================================================================

def _validate_transform(T: np.ndarray, name: str) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4, got {T.shape}")
    if not np.all(np.isfinite(T)):
        raise ValueError(f"{name} contains NaN/Inf")
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise ValueError(f"{name} last row must be [0, 0, 0, 1]")
    return T


def load_extrinsic() -> np.ndarray:
    if EXTRINSIC_FILE is None:
        T = _validate_transform(np.array(T_BASE_CAMERA_CONFIG), "T_BASE_CAMERA_CONFIG")
        return T

    path = os.path.expanduser(EXTRINSIC_FILE)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Extrinsic file not found: {path}")

    ext = os.path.splitext(path)[1].lower()
    if ext == ".npy":
        T = np.load(path)
    elif ext == ".json":
        with open(path, "r", encoding="utf-8") as f:
            obj = json.load(f)
        if isinstance(obj, list):
            T = np.asarray(obj, dtype=np.float64)
        elif isinstance(obj, dict):
            for key in ("T_base_camera", "T", "matrix"):
                if key in obj:
                    T = np.asarray(obj[key], dtype=np.float64)
                    break
            else:
                raise ValueError(
                    "JSON extrinsic must be a 4x4 list or contain one of: "
                    "T_base_camera / T / matrix"
                )
        else:
            raise ValueError("Unsupported JSON extrinsic structure")
    else:
        raise ValueError("EXTRINSIC_FILE supports only .npy or .json")

    T = _validate_transform(T, "extrinsic")
    if not EXTRINSIC_FILE_IS_T_BASE_CAMERA:
        T = np.linalg.inv(T)
    return T


# ============================================================================
# HERMES POSE READER + SHARED BUFFER
# ============================================================================

class PoseBuffer:
    """Thread-safe time-ordered pose ring buffer."""

    def __init__(self, maxlen: int):
        self._buf = deque(maxlen=maxlen)
        self._cv = threading.Condition()

    def append(self, sample: PoseSample) -> None:
        with self._cv:
            # HTTP samples should already be ordered. If clock adjustment causes an
            # out-of-order sample, ignore it instead of corrupting interpolation.
            if self._buf and sample.timestamp_ns <= self._buf[-1].timestamp_ns:
                return
            self._buf.append(sample)
            self._cv.notify_all()

    def snapshot(self):
        with self._cv:
            return list(self._buf)

    def wait_for_bracket(
        self,
        target_ns: int,
        timeout_s: float,
    ) -> Optional[Tuple[PoseSample, PoseSample]]:
        """Wait until buffer contains p0.t <= target <= p1.t, or timeout."""
        deadline = time.monotonic() + timeout_s

        with self._cv:
            while True:
                if len(self._buf) >= 2:
                    first_t = self._buf[0].timestamp_ns
                    last_t = self._buf[-1].timestamp_ns

                    # Target is older than retained history: waiting cannot fix it.
                    if target_ns < first_t:
                        return None

                    if target_ns <= last_t:
                        samples = list(self._buf)
                        times = [p.timestamp_ns for p in samples]
                        idx = bisect.bisect_left(times, target_ns)

                        if idx == 0:
                            return None
                        if idx >= len(samples):
                            # Should be impossible because target <= last_t, but safe.
                            return None

                        p0 = samples[idx - 1]
                        p1 = samples[idx]
                        return p0, p1

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cv.wait(timeout=remaining)


class HermesPoseReader(threading.Thread):
    def __init__(self, pose_buffer: PoseBuffer, stop_event: threading.Event):
        super().__init__(name="HermesPoseReader", daemon=True)
        self.pose_buffer = pose_buffer
        self.stop_event = stop_event
        self.url = f"http://{ROBOT_IP}:{POSE_HTTP_PORT}{POSE_API}"

        self.seq = 0
        self.success_count = 0
        self.error_count = 0
        self.last_error = None

    def _read_pose_json(self) -> dict:
        with urllib.request.urlopen(self.url, timeout=POSE_HTTP_TIMEOUT_S) as f:
            return json.loads(f.read())

    def run(self) -> None:
        period_s = 0.0 if POSE_POLL_HZ <= 0 else 1.0 / POSE_POLL_HZ

        while not self.stop_event.is_set():
            loop_start_mono = time.monotonic()

            request_time_ns = time.time_ns()
            request_mono_ns = time.monotonic_ns()

            try:
                p = self._read_pose_json()

                response_mono_ns = time.monotonic_ns()
                response_time_ns = time.time_ns()

                rtt_ns = response_mono_ns - request_mono_ns
                timestamp_ns = (request_time_ns + response_time_ns) // 2

                sample = PoseSample(
                    seq=self.seq,
                    timestamp_ns=timestamp_ns,
                    request_time_ns=request_time_ns,
                    response_time_ns=response_time_ns,
                    rtt_ns=rtt_ns,
                    x=float(p["x"]),
                    y=float(p["y"]),
                    z=float(p.get("z", 0.0)),
                    roll=float(p.get("roll", 0.0)),
                    pitch=float(p.get("pitch", 0.0)),
                    yaw=float(p["yaw"]),
                )

                self.pose_buffer.append(sample)
                self.seq += 1
                self.success_count += 1
                self.last_error = None

            except Exception as e:
                self.error_count += 1
                self.last_error = repr(e)

            if period_s > 0:
                elapsed = time.monotonic() - loop_start_mono
                sleep_s = period_s - elapsed
                if sleep_s > 0:
                    self.stop_event.wait(sleep_s)


# ============================================================================
# POSE INTERPOLATION
# ============================================================================

def pose_sample_to_transform(p: PoseSample) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_euler(
        "xyz", [p.roll, p.pitch, p.yaw], degrees=False
    ).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def interpolate_pose(
    pose_buffer: PoseBuffer,
    target_ns: int,
) -> SyncResult:
    bracket = pose_buffer.wait_for_bracket(target_ns, POSE_SYNC_WAIT_TIMEOUT_S)
    if bracket is None:
        return SyncResult(
            valid=False,
            reason="no_pose_bracket",
            T_map_base=None,
        )

    p0, p1 = bracket
    dt_ns = p1.timestamp_ns - p0.timestamp_ns
    if dt_ns <= 0:
        return SyncResult(
            valid=False,
            reason="invalid_pose_time_order",
            T_map_base=None,
            p0=p0,
            p1=p1,
        )

    dt_before_ms = (target_ns - p0.timestamp_ns) / 1e6
    dt_after_ms = (p1.timestamp_ns - target_ns) / 1e6

    if dt_before_ms < 0 or dt_after_ms < 0:
        return SyncResult(
            valid=False,
            reason="camera_not_bracketed",
            T_map_base=None,
            p0=p0,
            p1=p1,
            dt_before_ms=dt_before_ms,
            dt_after_ms=dt_after_ms,
        )

    if dt_before_ms > MAX_POSE_SIDE_DT_MS or dt_after_ms > MAX_POSE_SIDE_DT_MS:
        return SyncResult(
            valid=False,
            reason="pose_gap_too_large",
            T_map_base=None,
            p0=p0,
            p1=p1,
            dt_before_ms=dt_before_ms,
            dt_after_ms=dt_after_ms,
        )

    rtt0_ms = p0.rtt_ns / 1e6
    rtt1_ms = p1.rtt_ns / 1e6
    if rtt0_ms > MAX_POSE_RTT_MS or rtt1_ms > MAX_POSE_RTT_MS:
        return SyncResult(
            valid=False,
            reason="pose_rtt_too_large",
            T_map_base=None,
            p0=p0,
            p1=p1,
            dt_before_ms=dt_before_ms,
            dt_after_ms=dt_after_ms,
        )

    alpha = (target_ns - p0.timestamp_ns) / dt_ns
    alpha = float(np.clip(alpha, 0.0, 1.0))

    # Translation: linear interpolation.
    xyz0 = np.array([p0.x, p0.y, p0.z], dtype=np.float64)
    xyz1 = np.array([p1.x, p1.y, p1.z], dtype=np.float64)
    xyz = (1.0 - alpha) * xyz0 + alpha * xyz1

    # Rotation: quaternion SLERP.
    r0 = Rotation.from_euler("xyz", [p0.roll, p0.pitch, p0.yaw], degrees=False)
    r1 = Rotation.from_euler("xyz", [p1.roll, p1.pitch, p1.yaw], degrees=False)
    slerp = Slerp([0.0, 1.0], Rotation.concatenate([r0, r1]))
    r = slerp([alpha])[0]

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = r.as_matrix()
    T[:3, 3] = xyz

    return SyncResult(
        valid=True,
        reason="ok",
        T_map_base=T,
        p0=p0,
        p1=p1,
        alpha=alpha,
        dt_before_ms=dt_before_ms,
        dt_after_ms=dt_after_ms,
    )


# ============================================================================
# REALSENSE
# ============================================================================

def _normalized_option_text(value: str) -> str:
    return "".join(ch.lower() for ch in value if ch.isalnum())


def _read_option(sensor, option):
    if not sensor.supports(option):
        return None
    try:
        return float(sensor.get_option(option))
    except Exception:
        return None


def _option_description(sensor, option, value):
    if value is None:
        return None
    try:
        return str(sensor.get_option_value_description(option, float(value)))
    except Exception:
        return None


def _set_visual_preset_by_name(depth_sensor, desired_name: Optional[str]):
    if desired_name is None or not depth_sensor.supports(rs.option.visual_preset):
        return

    target = _normalized_option_text(desired_name)
    option_range = depth_sensor.get_option_range(rs.option.visual_preset)
    step = option_range.step if option_range.step > 0 else 1.0
    value = option_range.min
    matched_value = None

    # D4xx presets are discrete values. Search descriptions instead of relying on
    # Python enum member names, which vary slightly across librealsense releases.
    while value <= option_range.max + 1e-6:
        desc = _option_description(depth_sensor, rs.option.visual_preset, value)
        if desc is not None and _normalized_option_text(desc) == target:
            matched_value = value
            break
        value += step

    if matched_value is None:
        print(
            f"[depth] warning: visual preset '{desired_name}' not found; "
            "keeping the device's current preset"
        )
        return

    try:
        depth_sensor.set_option(rs.option.visual_preset, float(matched_value))
    except Exception as exc:
        print(f"[depth] warning: failed to set visual preset '{desired_name}': {exc}")


def configure_depth_sensor(depth_sensor) -> dict:
    """Apply conservative hardware-side D435i settings and report actual state."""
    # Set the preset first because presets may modify multiple stereo options.
    _set_visual_preset_by_name(depth_sensor, DEPTH_VISUAL_PRESET)

    if DEPTH_EMITTER_ENABLED is not None:
        if depth_sensor.supports(rs.option.emitter_enabled):
            try:
                depth_sensor.set_option(
                    rs.option.emitter_enabled,
                    1.0 if DEPTH_EMITTER_ENABLED else 0.0,
                )
            except Exception as exc:
                print(f"[depth] warning: failed to set emitter_enabled: {exc}")
        else:
            print("[depth] warning: emitter_enabled is unsupported on this device")

    if DEPTH_AUTO_EXPOSURE is not None:
        if depth_sensor.supports(rs.option.enable_auto_exposure):
            try:
                depth_sensor.set_option(
                    rs.option.enable_auto_exposure,
                    1.0 if DEPTH_AUTO_EXPOSURE else 0.0,
                )
            except Exception as exc:
                print(f"[depth] warning: failed to set auto exposure: {exc}")
        else:
            print("[depth] warning: depth auto exposure is unsupported on this device")

    preset_value = _read_option(depth_sensor, rs.option.visual_preset)
    emitter_value = _read_option(depth_sensor, rs.option.emitter_enabled)
    auto_exposure_value = _read_option(depth_sensor, rs.option.enable_auto_exposure)

    state = {
        "visual_preset": _option_description(
            depth_sensor, rs.option.visual_preset, preset_value
        ),
        "visual_preset_value": preset_value,
        "emitter_enabled": None if emitter_value is None else bool(emitter_value > 0.5),
        "auto_exposure_enabled": (
            None if auto_exposure_value is None else bool(auto_exposure_value > 0.5)
        ),
        # Deliberately no librealsense post-processing filters in the publisher.
        "software_filters": [],
    }

    print(
        "[depth] sensor config: "
        f"preset={state['visual_preset']!r} "
        f"emitter={state['emitter_enabled']} "
        f"auto_exposure={state['auto_exposure_enabled']} "
        "software_filters=none"
    )
    return state


def build_pipeline():
    pipe = rs.pipeline()
    cfg = rs.config()

    if CAMERA_SERIAL:
        cfg.enable_device(CAMERA_SERIAL)

    cfg.enable_stream(
        rs.stream.depth,
        DEPTH_WIDTH,
        DEPTH_HEIGHT,
        rs.format.z16,
        FPS,
    )
    cfg.enable_stream(
        rs.stream.color,
        RGB_WIDTH,
        RGB_HEIGHT,
        rs.format.bgr8,
        FPS,
    )

    profile = pipe.start(cfg)
    dev = profile.get_device()

    global_time_status = []
    for sensor in dev.query_sensors():
        name = sensor.get_info(rs.camera_info.name)
        if sensor.supports(rs.option.global_time_enabled):
            try:
                sensor.set_option(rs.option.global_time_enabled, 1)
                enabled = sensor.get_option(rs.option.global_time_enabled) == 1
                global_time_status.append((name, enabled))
            except Exception as e:
                global_time_status.append((name, f"error: {e}"))
        else:
            global_time_status.append((name, "unsupported"))

    print(
        f"[camera] device={dev.get_info(rs.camera_info.name)} "
        f"serial={dev.get_info(rs.camera_info.serial_number)}"
    )
    for name, st in global_time_status:
        print(f"[camera] global_time on '{name}': {st}")

    depth_sensor = dev.first_depth_sensor()
    depth_sensor_config = configure_depth_sensor(depth_sensor)
    depth_scale = float(depth_sensor.get_depth_scale())

    color_intr = (
        profile.get_stream(rs.stream.color)
        .as_video_stream_profile()
        .get_intrinsics()
    )
    raw_depth_intr = (
        profile.get_stream(rs.stream.depth)
        .as_video_stream_profile()
        .get_intrinsics()
    )

    aligner = rs.align(rs.stream.color) if ALIGN_DEPTH_TO_COLOR else None

    return (
        pipe,
        profile,
        aligner,
        depth_scale,
        color_intr,
        raw_depth_intr,
        depth_sensor_config,
    )


def intrinsics_to_dict(intr) -> dict:
    return {
        "width": int(intr.width),
        "height": int(intr.height),
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "ppx": float(intr.ppx),
        "ppy": float(intr.ppy),
        "model": str(intr.model),
        "coeffs": [float(v) for v in intr.coeffs],
    }


def is_host_time_domain(domain) -> bool:
    """True when RealSense timestamp is mapped to the host/system timeline."""
    return domain in (
        rs.timestamp_domain.global_time,
        rs.timestamp_domain.system_time,
    )


def timestamp_source_name(domain) -> str:
    if domain == rs.timestamp_domain.global_time:
        return "realsense_global_time"
    if domain == rs.timestamp_domain.system_time:
        return "realsense_system_time"
    return f"realsense_{str(domain)}"


# ============================================================================
# MAIN PUBLISHER
# ============================================================================

def main():
    stop_event = threading.Event()

    def _handle_signal(signum, frame):
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    T_base_camera = load_extrinsic()
    print("[extrinsic] T_base_camera (color optical -> base) =")
    print(T_base_camera)

    # This publisher's calibrated T_base_camera comes from RGB images, i.e. the
    # RealSense color optical frame. For the nvblox-ready protocol below, keep
    # depth aligned to color so depth_intr and T_map_camera refer to the same frame.
    if not ALIGN_DEPTH_TO_COLOR:
        raise RuntimeError(
            "ALIGN_DEPTH_TO_COLOR must be True for this RGB-calibrated publisher: "
            "T_map_camera refers to the color optical frame. If native depth is "
            "required, also publish/compute T_map_depth using RealSense depth-to-color "
            "extrinsics instead of reusing T_map_camera."
        )

    # Shared pose source.
    pose_buffer = PoseBuffer(maxlen=POSE_BUFFER_SIZE)
    pose_reader = HermesPoseReader(pose_buffer, stop_event)
    pose_reader.start()
    print(
        f"[pose] polling {pose_reader.url} target_hz={POSE_POLL_HZ:.1f} "
        f"buffer={POSE_BUFFER_SIZE}"
    )

    # ZMQ publisher.
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, ZMQ_SNDHWM)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind(ZMQ_ENDPOINT)
    topic_bytes = ZMQ_TOPIC.encode("utf-8")
    print(f"[zmq] bound {ZMQ_ENDPOINT} topic={ZMQ_TOPIC}")

    pipe = None

    try:
        (
            pipe,
            profile,
            aligner,
            depth_scale,
            color_intr,
            raw_depth_intr,
            depth_sensor_config,
        ) = build_pipeline()

        # Warmup allows exposure + Global Time mapping to settle.
        for _ in range(max(0, CAMERA_WARMUP_FRAMES)):
            if stop_event.is_set():
                break
            pipe.wait_for_frames()

        frame_id = 0
        published_count = 0
        skipped_unsynced = 0
        skip_reasons = {
            "bad_camera_clock": 0,
            "no_pose_bracket": 0,
            "invalid_pose_time_order": 0,
            "camera_not_bracketed": 0,
            "pose_gap_too_large": 0,
            "pose_rtt_too_large": 0,
            "other_sync_error": 0,
        }
        dropped_zmq = 0
        last_print_mono = time.monotonic()

        # Intrinsics corresponding to the depth image that is actually published.
        # With rs.align(depth -> color), this is the aligned depth frame profile,
        # i.e. color-camera projection geometry. The stream profile is fixed for
        # the lifetime of this process, so cache it after the first valid frame.
        published_depth_intr = None

        def record_skip(reason: str) -> None:
            nonlocal skipped_unsynced
            skipped_unsynced += 1
            if reason in skip_reasons:
                skip_reasons[reason] += 1
            else:
                skip_reasons["other_sync_error"] += 1

        while not stop_event.is_set():
            # -----------------------------------------------------------------
            # 1) Acquire raw RGB-D frames.
            # -----------------------------------------------------------------
            frames = pipe.wait_for_frames()
            raw_color = frames.get_color_frame()
            raw_depth = frames.get_depth_frame()
            if not raw_color or not raw_depth:
                continue

            enqueue_time_unix_ns = time.time_ns()

            # IMPORTANT: take original timestamps before rs.align().
            color_ts_ms = float(raw_color.get_timestamp())
            depth_ts_ms = float(raw_depth.get_timestamp())
            color_domain = raw_color.get_frame_timestamp_domain()
            depth_domain = raw_depth.get_frame_timestamp_domain()
            color_fnum = int(raw_color.get_frame_number())
            depth_fnum = int(raw_depth.get_frame_number())

            color_host_time = is_host_time_domain(color_domain)
            depth_host_time = is_host_time_domain(depth_domain)

            if REQUIRE_GLOBAL_OR_SYSTEM_TIME and not color_host_time:
                record_skip("bad_camera_clock")
                if skipped_unsynced % max(1, PRINT_EVERY_N_FRAMES) == 0:
                    print(
                        f"[warn] color timestamp domain={color_domain}; "
                        "cannot align it with PC-timestamped robot pose"
                    )
                continue

            # Master timestamp: RGB capture time on host/system timeline.
            t_cam_ns = int(round(color_ts_ms * 1_000_000.0))

            # -----------------------------------------------------------------
            # 2) Synchronize robot pose to the camera timestamp.
            # -----------------------------------------------------------------
            sync = interpolate_pose(pose_buffer, t_cam_ns)

            if not sync.valid and not PUBLISH_UNSYNCED_FRAMES:
                record_skip(sync.reason)
                continue

            if sync.valid:
                T_map_base = sync.T_map_base
                T_map_camera = T_map_base @ T_base_camera
                pose_valid = True
            else:
                T_map_base = None
                T_map_camera = None
                pose_valid = False

            # -----------------------------------------------------------------
            # 3) Align depth to color, then materialize image arrays.
            # -----------------------------------------------------------------
            aligned_frames = aligner.process(frames) if aligner is not None else frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            bgr = np.asanyarray(color_frame.get_data())
            rgb = np.ascontiguousarray(bgr[:, :, ::-1])
            depth_raw = np.ascontiguousarray(np.asanyarray(depth_frame.get_data()))

            if published_depth_intr is None:
                published_depth_intr = (
                    depth_frame.get_profile()
                    .as_video_stream_profile()
                    .get_intrinsics()
                )

                print(
                    "[camera] published depth geometry: "
                    f"aligned_to={'color' if ALIGN_DEPTH_TO_COLOR else 'native'} "
                    f"size={published_depth_intr.width}x{published_depth_intr.height} "
                    f"fx={published_depth_intr.fx:.3f} "
                    f"fy={published_depth_intr.fy:.3f} "
                    f"cx={published_depth_intr.ppx:.3f} "
                    f"cy={published_depth_intr.ppy:.3f}"
                )
                print(
                    "[camera] native depth geometry: "
                    f"size={raw_depth_intr.width}x{raw_depth_intr.height} "
                    f"fx={raw_depth_intr.fx:.3f} "
                    f"fy={raw_depth_intr.fy:.3f} "
                    f"cx={raw_depth_intr.ppx:.3f} "
                    f"cy={raw_depth_intr.ppy:.3f}"
                )

                if ALIGN_DEPTH_TO_COLOR:
                    geom_pairs = (
                        ("width", published_depth_intr.width, color_intr.width),
                        ("height", published_depth_intr.height, color_intr.height),
                        ("fx", published_depth_intr.fx, color_intr.fx),
                        ("fy", published_depth_intr.fy, color_intr.fy),
                        ("ppx", published_depth_intr.ppx, color_intr.ppx),
                        ("ppy", published_depth_intr.ppy, color_intr.ppy),
                    )
                    mismatches = []
                    for name, aligned_value, color_value in geom_pairs:
                        if name in ("width", "height"):
                            same = int(aligned_value) == int(color_value)
                        else:
                            same = abs(float(aligned_value) - float(color_value)) <= 1e-4
                        if not same:
                            mismatches.append(
                                f"{name}: aligned={aligned_value}, color={color_value}"
                            )
                    if mismatches:
                        raise RuntimeError(
                            "rs.align(depth->color) returned depth geometry that does "
                            "not match the color profile: " + "; ".join(mismatches)
                        )

            # Sanity checks: published image bytes and published intrinsics must
            # describe exactly the same raster. This is especially important for
            # nvblox, where depth rays are reconstructed from these intrinsics.
            if depth_raw.ndim != 2:
                raise RuntimeError(f"Expected depth image HxW, got shape={depth_raw.shape}")

            if ALIGN_DEPTH_TO_COLOR and depth_raw.shape[:2] != rgb.shape[:2]:
                raise RuntimeError(
                    "Aligned depth/RGB size mismatch: "
                    f"depth={depth_raw.shape[:2]}, rgb={rgb.shape[:2]}"
                )

            if (
                int(published_depth_intr.width) != int(depth_raw.shape[1])
                or int(published_depth_intr.height) != int(depth_raw.shape[0])
            ):
                raise RuntimeError(
                    "Published depth intrinsics/image size mismatch: "
                    f"intr={published_depth_intr.width}x{published_depth_intr.height}, "
                    f"image={depth_raw.shape[1]}x{depth_raw.shape[0]}"
                )

            publish_time_unix_ns = time.time_ns()

            # -----------------------------------------------------------------
            # 4) Build compact metadata.
            # -----------------------------------------------------------------
            meta = {
                "version": 2,
                "type": "rgbd_pose",
                "frame_id": frame_id,

                # Master timestamp of this observation.
                "timestamp_ns": t_cam_ns,
                "timestamp_source": timestamp_source_name(color_domain),

                "width": int(rgb.shape[1]),
                "height": int(rgb.shape[0]),
                "depth_width": int(depth_raw.shape[1]),
                "depth_height": int(depth_raw.shape[0]),
                "rgb_dtype": str(rgb.dtype),
                "depth_dtype": str(depth_raw.dtype),
                "rgb_encoding": "rgb8",
                "depth_encoding": "z16",
                "depth_scale_m": depth_scale,

                "pose_valid": pose_valid,
                "T_map_base": None if T_map_base is None else T_map_base.tolist(),
                "T_map_camera": None if T_map_camera is None else T_map_camera.tolist(),

                "sync": {
                    "valid": sync.valid,
                    "reason": sync.reason,
                    "method": "linear_translation+slerp_rotation",
                    "dt_before_ms": sync.dt_before_ms,
                    "dt_after_ms": sync.dt_after_ms,
                    "rtt_before_ms": None if sync.p0 is None else sync.p0.rtt_ns / 1e6,
                    "rtt_after_ms": None if sync.p1 is None else sync.p1.rtt_ns / 1e6,
                },

                "publisher_stats": {
                    "published": published_count,
                    "skipped_unsynced": skipped_unsynced,
                    "skip_reasons": dict(skip_reasons),
                    "zmq_drop": dropped_zmq,
                    "pose_api_ok": pose_reader.success_count,
                    "pose_api_err": pose_reader.error_count,
                },

                # Camera geometry. ``depth_intr`` always corresponds to the
                # depth bytes in this packet. ``depth_intr_raw`` is the native
                # depth-sensor profile retained only for diagnostics/advanced use.
                "color_intr": intrinsics_to_dict(color_intr),
                "depth_intr": intrinsics_to_dict(published_depth_intr),
                "depth_intr_raw": intrinsics_to_dict(raw_depth_intr),
                "depth_aligned_to": "color" if ALIGN_DEPTH_TO_COLOR else None,
                "depth_sensor_config": depth_sensor_config,

                # The calibrated extrinsic/T_map_camera is RGB-based and therefore
                # always refers to the RealSense color optical frame. When depth is
                # aligned to color (the configured/default path), depth_intr and
                # T_map_camera describe the same camera frame and can be consumed
                # directly by nvblox.
                "T_map_camera_frame": "color_optical",
                "depth_frame": "color_optical" if ALIGN_DEPTH_TO_COLOR else "depth_optical",
            }

            if PUBLISH_CAMERA_DEBUG:
                meta["camera_debug"] = {
                    "color_timestamp_ms": color_ts_ms,
                    "color_timestamp_domain": str(color_domain),
                    "color_frame_number": color_fnum,

                    "depth_timestamp_ms": depth_ts_ms,
                    "depth_timestamp_domain": str(depth_domain),
                    "depth_frame_number": depth_fnum,

                    "rgb_depth_timestamp_diff_ms": depth_ts_ms - color_ts_ms,
                    "color_host_time_compatible": color_host_time,
                    "depth_host_time_compatible": depth_host_time,

                    "enqueue_time_unix_ns": enqueue_time_unix_ns,
                    "publish_time_unix_ns": publish_time_unix_ns,
                    "camera_to_user_ms": (
                        enqueue_time_unix_ns / 1e6 - color_ts_ms
                        if color_host_time
                        else None
                    ),
                    "pack_and_sync_ms": (
                        publish_time_unix_ns - enqueue_time_unix_ns
                    ) / 1e6,
                }

            meta_bytes = msgpack.packb(meta, use_bin_type=True)

            # -----------------------------------------------------------------
            # 5) Publish multipart ZMQ message.
            # -----------------------------------------------------------------
            try:
                sock.send_multipart(
                    [
                        topic_bytes,
                        meta_bytes,
                        rgb.tobytes(),
                        depth_raw.tobytes(),
                    ],
                    flags=zmq.NOBLOCK,
                )
            except zmq.Again:
                dropped_zmq += 1
                frame_id += 1
                continue

            frame_id += 1
            published_count += 1

            # -----------------------------------------------------------------
            # 6) Periodic diagnostics.
            # -----------------------------------------------------------------
            if published_count % max(1, PRINT_EVERY_N_FRAMES) == 0:
                now_mono = time.monotonic()
                dt = now_mono - last_print_mono
                last_print_mono = now_mono
                pub_fps = PRINT_EVERY_N_FRAMES / dt if dt > 0 else 0.0

                if sync.valid:
                    rtt0 = sync.p0.rtt_ns / 1e6 if sync.p0 else float("nan")
                    rtt1 = sync.p1.rtt_ns / 1e6 if sync.p1 else float("nan")
                    sync_text = (
                        f"dt=(-{sync.dt_before_ms:.1f},+{sync.dt_after_ms:.1f})ms "
                        f"rtt=({rtt0:.1f},{rtt1:.1f})ms"
                    )
                else:
                    sync_text = f"sync={sync.reason}"

                sr = skip_reasons
                print(
                    f"[pub] frame={frame_id} fps={pub_fps:.1f} "
                    f"pose_ok={pose_valid} {sync_text} "
                    f"pose_api_ok={pose_reader.success_count} "
                    f"pose_api_err={pose_reader.error_count} "
                    f"skip={skipped_unsynced}"
                    f"(bracket={sr['no_pose_bracket']},"
                    f" gap={sr['pose_gap_too_large']},"
                    f" rtt={sr['pose_rtt_too_large']},"
                    f" clock={sr['bad_camera_clock']},"
                    f" other={sr['invalid_pose_time_order'] + sr['camera_not_bracketed'] + sr['other_sync_error']}) "
                    f"zmq_drop={dropped_zmq}"
                )

    finally:
        stop_event.set()

        if pipe is not None:
            try:
                pipe.stop()
            except Exception:
                pass

        pose_reader.join(timeout=2.0)
        sock.close(0)
        ctx.term()

        print("[publisher] stopped")
        if pose_reader.last_error:
            print(f"[pose] last_error={pose_reader.last_error}")


if __name__ == "__main__":
    main()