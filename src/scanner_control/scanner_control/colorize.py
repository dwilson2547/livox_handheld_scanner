#!/usr/bin/env python3
"""
colorize — project RealSense color frames onto a reconstructed PLY mesh.

Algorithm
---------
For each mesh vertex (in world frame):
  - For each sampled camera keyframe:
      1. Look up the nearest odometry pose (T_world_lidar).
      2. Transform vertex to camera frame via T_cam_lidar @ inv(T_world_lidar).
      3. Project through the calibrated camera intrinsics.
      4. Score = 1/depth (closer, more head-on => higher score).
  - Assign the best-scoring frame's bilinearly-sampled color to the vertex.

Writes mesh_colored.ply (binary little-endian, float32 xyz + uint8 rgb per vertex).

Requires:
  - ROS2 environment sourced (for rosbag2_py / rclpy serialization)
  - cv2 (for image decode fallback, though we decode manually for speed)
  - numpy, yaml
"""

from __future__ import annotations

import re
import sys
import time
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False

# --------------------------------------------------------------------------- #
#  PLY I/O
# --------------------------------------------------------------------------- #

def _read_binary_ply(path: Path) -> tuple[np.ndarray, Optional[np.ndarray]]:
    """Return (verts float32 N×3, faces int32 M×3-or-None) from a binary PLY."""
    data = path.read_bytes()
    end = data.index(b"end_header\n") + len(b"end_header\n")
    header = data[:end].decode("ascii")
    n_verts = int(re.search(r"element vertex (\d+)", header).group(1))
    face_m = re.search(r"element face (\d+)", header)
    n_faces = int(face_m.group(1)) if face_m else 0

    body = data[end:]
    verts = np.frombuffer(body[: n_verts * 12], dtype="<f4").reshape(-1, 3).copy()
    faces: Optional[np.ndarray] = None
    if n_faces:
        face_dt = np.dtype(
            [("cnt", np.uint8), ("i0", "<i4"), ("i1", "<i4"), ("i2", "<i4")]
        )
        fa = np.frombuffer(body[n_verts * 12 : n_verts * 12 + n_faces * 13], dtype=face_dt)
        faces = np.stack([fa["i0"], fa["i1"], fa["i2"]], axis=1)
    return verts, faces


def write_colored_ply(
    path: Path,
    verts: np.ndarray,
    colors: np.ndarray,
    faces: Optional[np.ndarray],
) -> None:
    """Write binary little-endian PLY with float32 xyz + uint8 rgb vertex colors."""
    n_verts, n_faces = len(verts), len(faces) if faces is not None else 0
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {n_verts}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        f"element face {n_faces}\n"
        "property list uchar int vertex_indices\n"
        "end_header\n"
    ).encode("ascii")

    # 15 bytes per vertex: 3×float32 + 3×uint8, tightly packed (no alignment padding in PLY)
    vdt = np.dtype(
        [("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
         ("r", np.uint8), ("g", np.uint8), ("b", np.uint8)]
    )
    va = np.empty(n_verts, dtype=vdt)
    va["x"], va["y"], va["z"] = verts[:, 0], verts[:, 1], verts[:, 2]
    va["r"], va["g"], va["b"] = colors[:, 0], colors[:, 1], colors[:, 2]

    with path.open("wb") as fh:
        fh.write(header)
        fh.write(va.tobytes())
        if n_faces:
            fdt = np.dtype(
                [("cnt", np.uint8), ("i0", "<i4"), ("i1", "<i4"), ("i2", "<i4")]
            )
            fa = np.empty(n_faces, dtype=fdt)
            fa["cnt"] = 3
            fa["i0"], fa["i1"], fa["i2"] = faces[:, 0], faces[:, 1], faces[:, 2]
            fh.write(fa.tobytes())


# --------------------------------------------------------------------------- #
#  Bag helpers
# --------------------------------------------------------------------------- #

def _odom_to_se3(msg) -> np.ndarray:
    """nav_msgs/Odometry → 4×4 SE3 (T_world_lidar)."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    n = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if n > 1e-9:
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _decode_image_msg(msg) -> np.ndarray:
    """sensor_msgs/Image → uint8 (H, W, 3) RGB array."""
    h, w = msg.height, msg.width
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding.lower()
    if "rgb" in enc and "a" not in enc:
        return raw.reshape(h, w, 3).copy()
    if "bgr" in enc and "a" not in enc:
        return raw.reshape(h, w, 3)[:, :, ::-1].copy()
    if enc in ("rgba8",):
        return raw.reshape(h, w, 4)[:, :, :3].copy()
    if enc in ("bgra8",):
        return raw.reshape(h, w, 4)[:, :, 2::-1].copy()
    raise ValueError(f"Unsupported image encoding: {msg.encoding!r}")


# --------------------------------------------------------------------------- #
#  Core colorization
# --------------------------------------------------------------------------- #

TOPICS = {
    "odom": "/aft_mapped_to_init",
    "tf": "/tf",
    "image": "/camera/d435i/color/image_raw",
    "info": "/camera/d435i/color/camera_info",
}
# TF frame pair published by Point-LIO (world → body)
_TF_PARENT = "camera_init"
_TF_CHILD = "aft_mapped"


def colorize_mesh(
    session_dir: Path,
    T_cam_lidar: np.ndarray,
    out_path: Optional[Path] = None,
    keyframe_interval: float = 0.1,
    log: Callable[[str], None] = print,
) -> Path:
    """
    Project camera color onto the session's dense replay mesh.

    Parameters
    ----------
    session_dir      : path to session directory (contains *.db3 + mesh_dense_replay.ply)
    T_cam_lidar      : 4×4 SE3 — transforms points FROM LiDAR frame TO color camera frame
    out_path         : where to write the colored PLY (default: session_dir/mesh_colored.ply)
    keyframe_interval: seconds between sampled camera frames (0.5 = 2 fps)
    log              : callable for progress messages

    Returns
    -------
    Path to the written colored PLY.
    """
    if not _HAS_ROS:
        raise RuntimeError(
            "rosbag2_py not available — source the ROS2 environment before colorizing."
        )

    if out_path is None:
        out_path = session_dir / "mesh_colored.ply"

    # --- find bag and mesh -------------------------------------------------- #
    db3_files = sorted(session_dir.glob("*.db3"))
    if not db3_files:
        raise FileNotFoundError(f"No bag files in {session_dir}")
    bag_dir = db3_files[0].parent  # SequentialReader wants the directory

    mesh_path = session_dir / "mesh_dense_replay.ply"
    if not mesh_path.exists():
        raise FileNotFoundError(f"Mesh not found: {mesh_path}")

    log(f"Loading mesh {mesh_path.name} …")
    verts, faces = _read_binary_ply(mesh_path)
    n_verts = len(verts)
    verts_h = np.hstack([verts.astype(np.float64), np.ones((n_verts, 1))]).T  # (4,N)
    log(f"  {n_verts} vertices, {len(faces) if faces is not None else 0} faces")

    def _open_reader(topics):
        r = rosbag2_py.SequentialReader()
        r.open(
            rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""),
        )
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
        return r

    # Discover which topics are available
    _probe = rosbag2_py.SequentialReader()
    _probe.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    available_topics = {t.name for t in _probe.get_all_topics_and_types()}
    for required in [TOPICS["image"], TOPICS["info"]]:
        if required not in available_topics:
            raise RuntimeError(f"Required topic {required!r} not in bag. Available: {sorted(available_topics)}")

    use_odom_topic = TOPICS["odom"] in available_topics
    use_tf = (not use_odom_topic) and (TOPICS["tf"] in available_topics)
    if not use_odom_topic and not use_tf:
        raise RuntimeError(
            f"No odometry source found in bag. Need either {TOPICS['odom']!r} or {TOPICS['tf']!r}. "
            f"Available: {sorted(available_topics)}"
        )

    # --- pass 1: collect odometry + camera intrinsics ----------------------- #
    odom_source = TOPICS["odom"] if use_odom_topic else TOPICS["tf"]
    log(f"Pass 1: reading odometry from {odom_source!r} and camera intrinsics …")
    CameraInfo = get_message("sensor_msgs/msg/CameraInfo")
    TFMessage = get_message("tf2_msgs/msg/TFMessage")
    Odometry = get_message("nav_msgs/msg/Odometry")

    odom_times: list[int] = []
    odom_mats: list[np.ndarray] = []
    cam_info = None

    r1 = _open_reader([odom_source, TOPICS["info"]])
    while r1.has_next():
        topic, data, ts = r1.read_next()
        if topic == TOPICS["odom"] and use_odom_topic:
            odom_times.append(ts)
            odom_mats.append(_odom_to_se3(deserialize_message(data, Odometry)))
        elif topic == TOPICS["tf"] and use_tf:
            msg = deserialize_message(data, TFMessage)
            for xf in msg.transforms:
                if xf.header.frame_id == _TF_PARENT and xf.child_frame_id == _TF_CHILD:
                    tr = xf.transform.translation
                    q = xf.transform.rotation
                    qx, qy, qz, qw = q.x, q.y, q.z, q.w
                    n = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
                    if n > 1e-9:
                        qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
                    R = np.array([
                        [1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                        [2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2), 2*(qy*qz-qx*qw)],
                        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
                    ], dtype=np.float64)
                    T = np.eye(4, dtype=np.float64)
                    T[:3, :3] = R
                    T[:3, 3] = [tr.x, tr.y, tr.z]
                    odom_times.append(ts)
                    odom_mats.append(T)
        elif topic == TOPICS["info"] and cam_info is None:
            cam_info = deserialize_message(data, CameraInfo)

    if not odom_times:
        raise RuntimeError("No odometry messages in bag")
    if cam_info is None:
        raise RuntimeError("No camera_info messages in bag")

    odom_t = np.array(odom_times, dtype=np.int64)
    log(f"  {len(odom_t)} odometry frames")

    K = np.array(cam_info.k, dtype=np.float64).reshape(3, 3)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    W, H = int(cam_info.width), int(cam_info.height)
    log(f"  Camera: {W}×{H}, fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

    # --- pass 2: stream images, project, update vertex colors --------------- #
    log("Pass 2: projecting vertices onto camera keyframes …")
    Image = get_message("sensor_msgs/msg/Image")

    best_scores = np.full(n_verts, -np.inf, dtype=np.float64)
    vertex_colors = np.zeros((n_verts, 3), dtype=np.uint8)

    last_kf_ts: int = -(int(1e18))
    n_kf = 0
    interval_ns = int(keyframe_interval * 1e9)

    r2 = _open_reader([TOPICS["image"]])
    while r2.has_next():
        topic, data, ts = r2.read_next()
        if topic != TOPICS["image"]:
            continue
        if ts - last_kf_ts < interval_ns:
            continue
        last_kf_ts = ts

        # nearest odometry (nearest-neighbour)
        idx = int(np.searchsorted(odom_t, ts))
        if idx > 0 and (
            idx == len(odom_t) or abs(odom_t[idx - 1] - ts) < abs(odom_t[idx] - ts)
        ):
            idx -= 1
        if abs(odom_t[idx] - ts) > int(5e8):  # skip if odom > 500ms away
            continue

        T_world_lidar = odom_mats[idx]
        # T_cam_world = T_cam_lidar @ inv(T_world_lidar)
        T_cam_world = T_cam_lidar @ np.linalg.inv(T_world_lidar)

        # transform all vertices to camera frame at once
        v_cam = T_cam_world @ verts_h  # (4, N)
        depth = v_cam[2]

        vis = depth > 0.3  # must be in front of camera, at least 30cm

        # project to image plane
        safe_d = np.where(vis, depth, 1.0)
        u = fx * v_cam[0] / safe_d + cx
        v = fy * v_cam[1] / safe_d + cy

        in_bounds = (u >= 0.5) & (u < W - 0.5) & (v >= 0.5) & (v < H - 0.5)

        # score: 1/depth — closer = higher; invisible = -inf (already initialized)
        score = np.where(vis & in_bounds, 1.0 / depth, -np.inf)

        improve = score > best_scores
        if not improve.any():
            continue

        # decode image only when at least one vertex improves
        img = _decode_image_msg(deserialize_message(data, Image))

        imp = np.where(improve)[0]
        ui = u[imp]
        vi = v[imp]
        x0 = np.floor(ui).astype(np.int32).clip(0, W - 2)
        y0 = np.floor(vi).astype(np.int32).clip(0, H - 2)
        wx = (ui - x0).reshape(-1, 1)
        wy = (vi - y0).reshape(-1, 1)

        # bilinear sample
        c00 = img[y0, x0].astype(np.float32)
        c10 = img[y0, x0 + 1].astype(np.float32)
        c01 = img[y0 + 1, x0].astype(np.float32)
        c11 = img[y0 + 1, x0 + 1].astype(np.float32)
        sampled = (
            (1 - wy) * ((1 - wx) * c00 + wx * c10)
            + wy * ((1 - wx) * c01 + wx * c11)
        ).clip(0, 255).astype(np.uint8)

        vertex_colors[imp] = sampled
        best_scores[imp] = score[imp]
        n_kf += 1

        if n_kf % 20 == 0:
            colored = int((best_scores > -np.inf).sum())
            log(f"  keyframe {n_kf}: {colored}/{n_verts} vertices colored")

    colored = int((best_scores > -np.inf).sum())
    log(f"Processed {n_kf} keyframes — {colored}/{n_verts} vertices colored "
        f"({100*colored//n_verts if n_verts else 0}%)")

    if colored == 0:
        raise RuntimeError(
            "No vertices were colored. Check that T_cam_lidar is correct and "
            "that the camera images overlap with the scan area."
        )

    log(f"Writing {out_path.name} …")
    write_colored_ply(out_path, verts, vertex_colors, faces)
    log(f"Done → {out_path}")
    return out_path


# --------------------------------------------------------------------------- #
#  Calibration helpers
# --------------------------------------------------------------------------- #

DEFAULT_CALIB_TEMPLATE = """\
# LiDAR-to-camera extrinsic calibration
# T_cam_lidar: 4x4 SE3 matrix that transforms a point from the Livox Horizon
# LiDAR frame into the D435i color camera frame.
#
# HOW TO FILL THIS IN
# -------------------
# Option A (quick, ~1-2 cm accuracy):
#   Measure the physical offset between the Horizon's aperture center and the
#   D435i color camera's principal point with calipers. Estimate rotation from
#   inspection (usually near-identity if both sensors face forward).
#
# Option B (accurate, ~1-5 mm):
#   Record a calibration bag of a scene with sharp straight edges at multiple
#   depths (a doorway corner or box works well), then use:
#     livox_camera_calib (HKU MaRS) — targetless edge-based calibration
#     OR direct_visual_lidar_calibration — available on ROS2 Humble
#   Both tools output a 4x4 T_cam_lidar matrix.
#
# Until this is filled in, colorized meshes will use identity (= offset = 0,
# same frame as LiDAR), which produces visually misaligned colors.
#
T_cam_lidar:
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
"""


def load_calibration(path: Optional[Path]) -> np.ndarray:
    """
    Load T_cam_lidar from a YAML file. Returns identity with a warning if path is None.
    """
    if path is None or not path.exists():
        print(
            "WARNING: no calibration file found — using identity.\n"
            "  Colors will be misaligned until T_cam_lidar is measured/calibrated.\n"
            "  See scripts/calib_lidar_camera.yaml for the required format.",
            file=sys.stderr,
        )
        return np.eye(4, dtype=np.float64)

    with path.open() as f:
        data = yaml.safe_load(f)

    T = np.array(data["T_cam_lidar"], dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"T_cam_lidar must be 4×4, got {T.shape}")
    return T
