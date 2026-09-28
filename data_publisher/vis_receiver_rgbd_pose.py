#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compact Rerun viewer for synchronized RGB-D + camera pose.

Target:
    rerun-sdk == 0.31.4
    numpy      == 1.26.4

Layout:
    left:
        RGB only
        Depth only

    right:
        Large 3D view
        Small bottom diagnostics:
            Sync timing | Camera XYZ | Skip counters

Important:
    - RGB and Depth are logged as separate root-level entities:
          /images/rgb
          /images/depth
      They are explicitly filtered into separate 2D views.
    - RGB/Depth are NOT children of the 3D camera pinhole entity, so they will
      not be projected into the 3D scene.
    - The blueprint disables auto views/layout and is explicitly activated.
"""

from __future__ import annotations

# ============================================================================
# USER CONFIGURATION
# ============================================================================

# ---------- ZMQ ----------
ZMQ_ENDPOINT = "ipc:///tmp/rgbd_pose.ipc"
# ZMQ_ENDPOINT = "tcp://10.113.45.27:5555"
ZMQ_TOPIC = "rgbd.pose"
ZMQ_RCVHWM = 4

# ---------- Rerun ----------
# Use a fresh app id so old viewer-side blueprint state is not reused.
RERUN_APP_ID = "rgbd_pose_monitor_compact_v2"

# "connect" connects to an existing Rerun server; "spawn" starts the native
# Rerun viewer on this machine. Command-line arguments can override both.
RERUN_MODE = "connect"
RERUN_DESTINATION = "rerun+http://127.0.0.1:9876/proxy"
RERUN_GRPC_PORT = 9876
RERUN_SERVER_MEMORY_LIMIT = "128MiB"

# ---------- Visualization rate ----------
RGB_EVERY_N = 6          # ~5 Hz if source is 30 FPS
DEPTH_EVERY_N = 10       # ~3 Hz
POSE_EVERY_N = 1         # full-rate pose
METRICS_EVERY_N = 1      # full-rate metrics

# ---------- 3D ----------
SHOW_MAP_ORIGIN = True
SHOW_BASE = True
SHOW_CAMERA = True
SHOW_CAMERA_TRAJECTORY = True

MAP_AXIS_LENGTH = 0.40
BASE_AXIS_LENGTH = 0.28
CAMERA_AXIS_LENGTH = 0.20

CAMERA_FOV_PLANE_DISTANCE = 0.20

# ---------- Trajectory ----------
TRAJECTORY_MAX_POINTS = 1000
TRAJECTORY_UPDATE_EVERY_N = 150

# ---------- RGB ----------
COMPRESS_RGB = True
RGB_JPEG_QUALITY = 80

# ---------- Console ----------
PRINT_EVERY_N_FRAMES = 60

# ---------- Shutdown ----------
FORCE_EXIT_AFTER_CLEANUP = True


# ============================================================================
# IMPORTS
# ============================================================================

from collections import deque
import argparse
import os
import sys
import time

import msgpack
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import zmq


# ============================================================================
# HELPERS
# ============================================================================

def as_transform(meta: dict, key: str) -> np.ndarray | None:
    value = meta.get(key)
    if value is None:
        return None

    T = np.asarray(value, dtype=np.float64)

    if T.shape != (4, 4):
        return None

    if not np.all(np.isfinite(T)):
        return None

    return T


def as_float(value, default=np.nan) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def get_stat(stats: dict, *names):
    for name in names:
        if name in stats:
            return stats[name]
    return None


def log_scalar(rec: rr.RecordingStream, path: str, value) -> None:
    v = as_float(value)
    if np.isfinite(v):
        rec.log(path, rr.Scalars(v))


def log_camera_fov(rec: rr.RecordingStream, meta: dict) -> bool:
    """
    Log only the camera optical model/FOV in the 3D hierarchy.

    RealSense optical frame:
        +X right
        +Y down
        +Z forward
    """
    intr = meta.get("color_intr")
    if not isinstance(intr, dict):
        return False

    try:
        fx = float(intr["fx"])
        fy = float(intr["fy"])
        cx = float(intr["ppx"])
        cy = float(intr["ppy"])
        width = int(meta["width"])
        height = int(meta["height"])
    except Exception:
        return False

    rec.log(
        "world/camera/fov",
        rr.Pinhole(
            image_from_camera=[
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            resolution=[width, height],
            camera_xyz=rr.ViewCoordinates.RDF,
            image_plane_distance=CAMERA_FOV_PLANE_DISTANCE,
        ),
        static=True,
    )

    return True


# ============================================================================
# BLUEPRINT
# ============================================================================

def build_blueprint() -> rrb.Blueprint:
    # Explicit absolute contents: each 2D view shows one image entity only.
    rgb_view = rrb.Spatial2DView(
        origin="/",
        name="RGB",
        contents=[
            "/images/rgb",
        ],
    )

    depth_view = rrb.Spatial2DView(
        origin="/",
        name="Depth",
        contents=[
            "/images/depth",
        ],
    )

    scene_view = rrb.Spatial3DView(
        origin="/world",
        name="Map / Base / Camera",
        contents=[
            "/world/**",
        ],
    )

    sync_view = rrb.TimeSeriesView(
        origin="/metrics/sync",
        name="Sync",
        contents=[
            "/metrics/sync/**",
        ],
    )

    xyz_view = rrb.TimeSeriesView(
        origin="/metrics/camera_xyz",
        name="Camera XYZ",
        contents=[
            "/metrics/camera_xyz/**",
        ],
    )

    skip_view = rrb.TimeSeriesView(
        origin="/metrics/skip",
        name="Skip",
        contents=[
            "/metrics/skip/**",
        ],
    )

    diagnostics = rrb.Horizontal(
        sync_view,
        xyz_view,
        skip_view,
        column_shares=[1, 1, 1],
    )

    left = rrb.Vertical(
        rgb_view,
        depth_view,
        row_shares=[1, 1],
    )

    right = rrb.Vertical(
        scene_view,
        diagnostics,
        row_shares=[4, 1],
    )

    return rrb.Blueprint(
        rrb.Horizontal(
            left,
            right,
            column_shares=[2, 3],
        ),
        rrb.SelectionPanel(state="collapsed"),
        rrb.BlueprintPanel(state="collapsed"),
        rrb.TimePanel(state="collapsed"),
        auto_views=False,
        auto_layout=False,
    )


def log_static_scene(rec: rr.RecordingStream) -> None:
    # map/base convention: +X forward, +Y left, +Z up
    rec.log(
        "world",
        rr.ViewCoordinates.FLU,
        static=True,
    )

    if SHOW_MAP_ORIGIN:
        rec.log(
            "world/map_origin",
            rr.Transform3D(),
            rr.TransformAxes3D(
                MAP_AXIS_LENGTH,
                show_frame=True,
            ),
            static=True,
        )

        rec.log(
            "world/map_origin/marker",
            rr.Points3D(
                [[0.0, 0.0, 0.0]],
                radii=0.045,
            ),
            static=True,
        )

    # ---------------- Sync ----------------
    rec.log(
        "metrics/sync/dt_before_ms",
        rr.SeriesLines(names="dt before"),
        static=True,
    )

    rec.log(
        "metrics/sync/dt_after_ms",
        rr.SeriesLines(names="dt after"),
        static=True,
    )

    rec.log(
        "metrics/sync/rtt_before_ms",
        rr.SeriesLines(names="RTT before"),
        static=True,
    )

    rec.log(
        "metrics/sync/rtt_after_ms",
        rr.SeriesLines(names="RTT after"),
        static=True,
    )

    # ---------------- Camera XYZ ----------------
    rec.log(
        "metrics/camera_xyz/x",
        rr.SeriesLines(names="x"),
        static=True,
    )

    rec.log(
        "metrics/camera_xyz/y",
        rr.SeriesLines(names="y"),
        static=True,
    )

    rec.log(
        "metrics/camera_xyz/z",
        rr.SeriesLines(names="z"),
        static=True,
    )

    # ---------------- Skip ----------------
    for path, name in (
        ("metrics/skip/total", "total"),
        ("metrics/skip/bracket", "bracket"),
        ("metrics/skip/gap", "gap"),
        ("metrics/skip/rtt", "RTT"),
        ("metrics/skip/clock", "clock"),
        ("metrics/skip/other", "other"),
    ):
        rec.log(
            path,
            rr.SeriesLines(names=name),
            static=True,
        )


# ============================================================================
# MAIN
# ============================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize synchronized RGB-D and camera poses with Rerun."
    )
    parser.add_argument(
        "--rerun-mode",
        choices=("connect", "spawn"),
        default=RERUN_MODE,
        help=(
            "connect to an existing Rerun server (default), or spawn the "
            "native viewer locally"
        ),
    )
    parser.add_argument(
        "--rerun-destination",
        default=RERUN_DESTINATION,
        help="gRPC URL used in connect mode",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    print(
        f"[version] rerun={getattr(rr, '__version__', 'unknown')} "
        f"numpy={np.__version__}"
    )

    rec = rr.RecordingStream(RERUN_APP_ID)

    blueprint = build_blueprint()

    if args.rerun_mode == "spawn":
        rec.spawn(
            port=RERUN_GRPC_PORT,
            memory_limit=RERUN_SERVER_MEMORY_LIMIT,
            default_blueprint=blueprint,
        )
        print(f"[rerun] spawned local viewer on port {RERUN_GRPC_PORT}")
    else:
        rec.connect_grpc(
            args.rerun_destination,
            default_blueprint=blueprint,
        )
        print(f"[rerun] connected to {args.rerun_destination}")

    # Force this blueprint to become the current active/default blueprint.
    try:
        rec.send_blueprint(
            blueprint,
            make_active=True,
            make_default=True,
        )
    except Exception as exc:
        print(
            f"[rerun] send_blueprint warning: {exc}",
            file=sys.stderr,
        )

    log_static_scene(rec)

    print("")
    print("============================================================")
    print(" Rerun RGB-D Pose Monitor")
    print("============================================================")
    print(f"[rerun] mode: {args.rerun_mode}")
    print("============================================================")
    print("")

    # ------------------------------------------------------------------------
    # ZMQ
    # ------------------------------------------------------------------------
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)

    sock.setsockopt(zmq.RCVHWM, ZMQ_RCVHWM)
    sock.setsockopt(zmq.LINGER, 0)

    sock.connect(ZMQ_ENDPOINT)

    sock.setsockopt(
        zmq.SUBSCRIBE,
        ZMQ_TOPIC.encode("utf-8"),
    )

    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)

    print(
        f"[zmq] connected {ZMQ_ENDPOINT} "
        f"topic={ZMQ_TOPIC}"
    )
    print("[recv] Ctrl+C to stop")

    # ------------------------------------------------------------------------
    # Runtime state
    # ------------------------------------------------------------------------
    fov_logged = False

    trajectory = deque(
        maxlen=max(2, TRAJECTORY_MAX_POINTS)
    )

    received_count = 0
    malformed_count = 0

    last_print_time = time.monotonic()
    last_print_received = 0

    try:
        while True:
            events = dict(
                poller.poll(timeout=100)
            )

            if sock not in events:
                continue

            parts = sock.recv_multipart()

            if len(parts) != 4:
                malformed_count += 1
                continue

            _, meta_bytes, rgb_bytes, depth_bytes = parts

            # ----------------------------------------------------------------
            # Decode
            # ----------------------------------------------------------------
            try:
                meta = msgpack.unpackb(
                    meta_bytes,
                    raw=False,
                )

                width = int(meta["width"])
                height = int(meta["height"])

                rgb_dtype = np.dtype(
                    meta.get(
                        "rgb_dtype",
                        "uint8",
                    )
                )

                depth_dtype = np.dtype(
                    meta.get(
                        "depth_dtype",
                        "uint16",
                    )
                )

                rgb = np.frombuffer(
                    rgb_bytes,
                    dtype=rgb_dtype,
                ).reshape(
                    height,
                    width,
                    3,
                )

                depth = np.frombuffer(
                    depth_bytes,
                    dtype=depth_dtype,
                ).reshape(
                    height,
                    width,
                )

            except Exception as exc:
                malformed_count += 1
                print(
                    f"[warn] failed to decode frame: {exc}"
                )
                continue

            frame_id = int(
                meta.get(
                    "frame_id",
                    received_count,
                )
            )

            timestamp_ns = int(
                meta.get(
                    "timestamp_ns",
                    0,
                )
            )

            pose_valid = bool(
                meta.get(
                    "pose_valid",
                    False,
                )
            )

            # ----------------------------------------------------------------
            # Timeline
            # ----------------------------------------------------------------
            rec.set_time(
                "frame",
                sequence=frame_id,
            )

            if timestamp_ns > 0:
                rec.set_time(
                    "camera_time",
                    timestamp=np.datetime64(
                        timestamp_ns,
                        "ns",
                    ),
                )

            if not fov_logged:
                fov_logged = log_camera_fov(
                    rec,
                    meta,
                )

            # ----------------------------------------------------------------
            # RGB
            # ----------------------------------------------------------------
            if (
                received_count
                % max(1, RGB_EVERY_N)
                == 0
            ):
                image = rr.Image(rgb)

                if COMPRESS_RGB:
                    image = image.compress(
                        jpeg_quality=RGB_JPEG_QUALITY
                    )

                rec.log(
                    "images/rgb",
                    image,
                )

            # ----------------------------------------------------------------
            # Depth
            # ----------------------------------------------------------------
            if (
                received_count
                % max(1, DEPTH_EVERY_N)
                == 0
            ):
                depth_scale_m = float(
                    meta.get(
                        "depth_scale_m",
                        0.001,
                    )
                )

                units_per_meter = (
                    1.0 / depth_scale_m
                    if depth_scale_m > 0
                    else 1000.0
                )

                rec.log(
                    "images/depth",
                    rr.DepthImage(
                        depth,
                        meter=units_per_meter,
                        colormap="turbo",
                    ),
                )

            # ----------------------------------------------------------------
            # Pose
            # ----------------------------------------------------------------
            T_map_base = as_transform(
                meta,
                "T_map_base",
            )

            T_map_camera = as_transform(
                meta,
                "T_map_camera",
            )

            if (
                received_count
                % max(1, POSE_EVERY_N)
                == 0
            ):
                if (
                    SHOW_BASE
                    and T_map_base is not None
                ):
                    rec.log(
                        "world/base",
                        rr.Transform3D(
                            translation=T_map_base[:3, 3],
                            mat3x3=T_map_base[:3, :3],
                        ),
                        rr.TransformAxes3D(
                            BASE_AXIS_LENGTH,
                            show_frame=True,
                        ),
                    )

                if (
                    SHOW_CAMERA
                    and pose_valid
                    and T_map_camera is not None
                ):
                    camera_xyz = T_map_camera[:3, 3]
                    camera_R = T_map_camera[:3, :3]

                    rec.log(
                        "world/camera",
                        rr.Transform3D(
                            translation=camera_xyz,
                            mat3x3=camera_R,
                        ),
                        rr.TransformAxes3D(
                            CAMERA_AXIS_LENGTH,
                            show_frame=True,
                        ),
                    )

                    if SHOW_CAMERA_TRAJECTORY:
                        trajectory.append(
                            camera_xyz.astype(
                                np.float32
                            ).copy()
                        )

                        if (
                            len(trajectory) >= 2
                            and received_count
                            % max(
                                1,
                                TRAJECTORY_UPDATE_EVERY_N,
                            )
                            == 0
                        ):
                            rec.log(
                                "world/camera_trajectory",
                                rr.LineStrips3D(
                                    [
                                        np.asarray(
                                            trajectory,
                                            dtype=np.float32,
                                        )
                                    ],
                                    radii=0.006,
                                ),
                            )

            # ----------------------------------------------------------------
            # Compact diagnostics
            # ----------------------------------------------------------------
            if (
                received_count
                % max(1, METRICS_EVERY_N)
                == 0
            ):
                if (
                    pose_valid
                    and T_map_camera is not None
                ):
                    p = T_map_camera[:3, 3]

                    log_scalar(
                        rec,
                        "metrics/camera_xyz/x",
                        p[0],
                    )

                    log_scalar(
                        rec,
                        "metrics/camera_xyz/y",
                        p[1],
                    )

                    log_scalar(
                        rec,
                        "metrics/camera_xyz/z",
                        p[2],
                    )

                sync = meta.get(
                    "sync",
                    {},
                )

                if isinstance(sync, dict):
                    log_scalar(
                        rec,
                        "metrics/sync/dt_before_ms",
                        sync.get("dt_before_ms"),
                    )

                    log_scalar(
                        rec,
                        "metrics/sync/dt_after_ms",
                        sync.get("dt_after_ms"),
                    )

                    log_scalar(
                        rec,
                        "metrics/sync/rtt_before_ms",
                        sync.get("rtt_before_ms"),
                    )

                    log_scalar(
                        rec,
                        "metrics/sync/rtt_after_ms",
                        sync.get("rtt_after_ms"),
                    )

                stats = meta.get(
                    "publisher_stats",
                    {},
                )

                if isinstance(stats, dict) and stats:
                    log_scalar(
                        rec,
                        "metrics/skip/total",
                        get_stat(
                            stats,
                            "skip_total",
                            "skipped_unsynced",
                            "skip",
                        ),
                    )

                    log_scalar(
                        rec,
                        "metrics/skip/bracket",
                        get_stat(
                            stats,
                            "skip_bracket",
                            "skip_no_bracket",
                            "bracket",
                        ),
                    )

                    log_scalar(
                        rec,
                        "metrics/skip/gap",
                        get_stat(
                            stats,
                            "skip_gap",
                            "gap",
                        ),
                    )

                    log_scalar(
                        rec,
                        "metrics/skip/rtt",
                        get_stat(
                            stats,
                            "skip_rtt",
                            "rtt",
                        ),
                    )

                    log_scalar(
                        rec,
                        "metrics/skip/clock",
                        get_stat(
                            stats,
                            "skip_clock",
                            "clock",
                        ),
                    )

                    log_scalar(
                        rec,
                        "metrics/skip/other",
                        get_stat(
                            stats,
                            "skip_other",
                            "other",
                        ),
                    )

            received_count += 1

            # ----------------------------------------------------------------
            # Console
            # ----------------------------------------------------------------
            if (
                received_count
                % max(
                    1,
                    PRINT_EVERY_N_FRAMES,
                )
                == 0
            ):
                now = time.monotonic()

                elapsed = (
                    now
                    - last_print_time
                )

                recent_count = (
                    received_count
                    - last_print_received
                )

                fps = (
                    recent_count / elapsed
                    if elapsed > 0
                    else 0.0
                )

                last_print_time = now
                last_print_received = received_count

                if T_map_camera is not None:
                    x, y, z = (
                        T_map_camera[:3, 3]
                    )

                    camera_text = (
                        f"camera="
                        f"({x:+.3f},"
                        f"{y:+.3f},"
                        f"{z:+.3f})"
                    )

                else:
                    camera_text = "camera=(invalid)"

                print(
                    f"[recv] "
                    f"frame={frame_id} "
                    f"fps={fps:.1f} "
                    f"pose_valid={pose_valid} "
                    f"{camera_text} "
                    f"malformed={malformed_count}"
                )

    except KeyboardInterrupt:
        print("\n[receiver] Ctrl+C received")

    finally:
        try:
            poller.unregister(sock)
        except Exception:
            pass

        sock.close(0)

        try:
            ctx.term()
        except Exception:
            pass

        try:
            rec.flush(blocking=True)
        except Exception:
            pass

        try:
            rec.disconnect()
        except Exception as exc:
            print(
                f"[rerun] disconnect warning: {exc}",
                file=sys.stderr,
            )

        print(
            "[receiver] stopped",
            flush=True,
        )

        if FORCE_EXIT_AFTER_CLEANUP:
            os._exit(0)


if __name__ == "__main__":
    main()
