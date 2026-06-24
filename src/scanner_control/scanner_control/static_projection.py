#!/usr/bin/env python3
"""
static_projection — single-frame LiDAR↔camera projection test (the calibration gate).

This is build-step 1 / the "non-negotiable" prerequisite from
voxel_color_map_handoff.md and the whole subject of cam_lidar_calib_handoff.md:
before building any color-confidence system, confirm the camera→LiDAR extrinsic
is actually correct. A robust color accumulator still places color in the WRONG
location if the extrinsic rotation is off — it just does so confidently.

What it does
------------
Take ONE LiDAR sweep and ONE RGB frame (no odometry — points are projected
straight through T_cam_lidar, so this isolates the extrinsic from any trajectory
/ timestamp confound) and produce two artefacts:

  1. <out>_overlay.png  — the RGB image with LiDAR points splatted on top,
     colored by depth. **Read this for geometric alignment:** depth
     discontinuities in the overlay (a wall edge, doorway, window frame) should
     line up with the same visual edges in the photo. If the LiDAR edge sits
     shifted-but-parallel to the visual edge → extrinsic rotation/translation
     error (recalibrate). If warped/curved vs. a straight visual edge →
     intrinsics/distortion. If it lines up crisply → extrinsic is fine, the
     blotchy mesh is a temporal/outlier problem and the voxel map is the fix.

  2. <out>_cloud.ply    — the sweep's points (LiDAR frame) colored by the image
     pixel each projects to. View in Potree/MeshLab: a window's color landing on
     the wall beside it is the same diagnosis, in 3D.

Requires a sourced ROS 2 environment (rosbag2_py + livox_interfaces) and cv2.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, List, Optional, Tuple

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False

from scanner_control.colorize import _decode_image_msg  # reuse the tested decoder

_COLOR_IMG = "/camera/d435i/color/image_raw"
_COLOR_INFO = "/camera/d435i/color/camera_info"
_LIDAR = "/livox/lidar"


def _open_reader(bag_dir: Path, topics: Optional[List[str]] = None):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    if topics:
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def _custommsg_to_xyz(msg) -> np.ndarray:
    """livox_interfaces/CustomMsg → (N,3) float32 points in the LiDAR frame."""
    n = msg.point_num
    xyz = np.empty((n, 3), dtype=np.float32)
    pts = msg.points
    for i in range(n):
        p = pts[i]
        xyz[i, 0] = p.x
        xyz[i, 1] = p.y
        xyz[i, 2] = p.z
    return xyz


def _depth_to_color(depth: np.ndarray, dmin: float, dmax: float) -> np.ndarray:
    """Map depths → BGR via a turbo-ish ramp (near=warm, far=cool). (N,3) uint8."""
    t = np.clip((depth - dmin) / max(dmax - dmin, 1e-6), 0.0, 1.0)
    # simple blue→green→red ramp
    r = np.clip(1.5 - abs(2.0 * t - 2.0), 0, 1)
    g = np.clip(1.5 - abs(2.0 * t - 1.0), 0, 1)
    b = np.clip(1.5 - abs(2.0 * t - 0.0), 0, 1)
    return (np.stack([b, g, r], axis=1) * 255).astype(np.uint8)  # BGR for cv2


def run_static_projection(
    bag_dir: Path,
    T_cam_lidar: np.ndarray,
    out_prefix: Path,
    time_fraction: float = 0.5,
    accumulate_window: float = 0.15,
    point_radius: int = 2,
    overlay_stride: int = 1,
    overlay_alpha: float = 1.0,
    log: Callable[[str], None] = print,
) -> Tuple[Path, Path]:
    """
    Parameters
    ----------
    bag_dir           : directory containing the *.db3 bag
    T_cam_lidar       : 4×4 SE3, LiDAR-frame point → camera-frame point
    out_prefix        : output path prefix (suffixes _overlay.png / _cloud.ply added)
    time_fraction     : where in the bag to grab the reference RGB frame (0..1)
    accumulate_window : seconds of LiDAR sweeps to accumulate around the frame
                        (one sweep is sparse; widen only if the rig was near-still)
    point_radius      : splat radius in the overlay PNG

    Returns (overlay_png_path, cloud_ply_path).
    """
    if not _HAS_ROS:
        raise RuntimeError("rosbag2_py unavailable — source the ROS 2 environment first.")
    import cv2

    db3 = sorted(bag_dir.glob("*.db3"))
    if not db3:
        raise FileNotFoundError(f"No *.db3 in {bag_dir}")
    bag_dir = db3[0].parent

    Image = get_message("sensor_msgs/msg/Image")
    CameraInfo = get_message("sensor_msgs/msg/CameraInfo")
    CustomMsg = get_message("livox_interfaces/msg/CustomMsg")

    # -- pass 1: time bounds + intrinsics + image timestamp index ------------- #
    log("Pass 1: indexing image/info timestamps …")
    img_times: List[int] = []
    cam_info = None
    r = _open_reader(bag_dir, [_COLOR_IMG, _COLOR_INFO])
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic == _COLOR_IMG:
            img_times.append(ts)
        elif topic == _COLOR_INFO and cam_info is None:
            cam_info = deserialize_message(data, CameraInfo)
    if not img_times:
        raise RuntimeError(f"No {_COLOR_IMG} messages in bag")
    if cam_info is None:
        raise RuntimeError(f"No {_COLOR_INFO} messages in bag")

    img_t = np.array(sorted(img_times), dtype=np.int64)
    t0, t1 = img_t[0], img_t[-1]
    target_ts = int(t0 + (t1 - t0) * float(np.clip(time_fraction, 0.0, 1.0)))
    # snap to the nearest actual image timestamp
    j = int(np.searchsorted(img_t, target_ts))
    if j > 0 and (j == len(img_t) or abs(img_t[j - 1] - target_ts) < abs(img_t[j] - target_ts)):
        j -= 1
    frame_ts = int(img_t[j])
    log(f"  reference RGB frame at t={frame_ts} ({time_fraction:.0%} through {len(img_t)} frames)")

    K = np.array(cam_info.k, dtype=np.float64).reshape(3, 3)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    dist = np.array(cam_info.d, dtype=np.float64) if len(cam_info.d) else None
    W, H = int(cam_info.width), int(cam_info.height)
    log(f"  intrinsics {W}×{H} fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}"
        + (f"  dist={np.round(dist,4).tolist()}" if dist is not None and np.any(dist) else "  dist=none"))

    # -- pass 2: fetch the exact RGB frame + nearby LiDAR sweeps -------------- #
    log("Pass 2: reading reference image + LiDAR sweeps …")
    win = int(accumulate_window * 1e9)
    image = None
    sweeps: List[np.ndarray] = []
    r = _open_reader(bag_dir, [_COLOR_IMG, _LIDAR])
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic == _COLOR_IMG and ts == frame_ts and image is None:
            image = _decode_image_msg(deserialize_message(data, Image))  # RGB
        elif topic == _LIDAR and abs(ts - frame_ts) <= win:
            sweeps.append(_custommsg_to_xyz(deserialize_message(data, CustomMsg)))
    if image is None:
        raise RuntimeError("Could not re-read the reference image frame")
    if not sweeps:
        raise RuntimeError(
            f"No LiDAR sweeps within ±{accumulate_window}s of the frame — widen accumulate_window"
        )
    pts = np.concatenate(sweeps, axis=0)
    log(f"  {len(pts)} LiDAR points from {len(sweeps)} sweep(s); image {image.shape[1]}×{image.shape[0]}")

    # -- project LiDAR (frame) → camera → pixels ----------------------------- #
    ph = np.hstack([pts.astype(np.float64), np.ones((len(pts), 1))]).T  # (4,N)
    cam = (T_cam_lidar @ ph)[:3]                                        # (3,N)
    z = cam[2]
    front = z > 0.05
    x = cam[0][front] / z[front]
    y = cam[1][front] / z[front]
    zf = z[front]
    pts_front = pts[front]

    # apply radial-tangential distortion if the camera reports it (plumb-bob)
    if dist is not None and np.any(dist):
        k1, k2, p1, p2 = dist[0], dist[1], dist[2], dist[3]
        k3 = dist[4] if len(dist) > 4 else 0.0
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        x_d = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        y_d = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x, y = x_d, y_d

    u = fx * x + cx
    v = fy * y + cy
    inb = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    ui = u[inb].astype(np.int32)
    vi = v[inb].astype(np.int32)
    zi = zf[inb]
    pts_in = pts_front[inb]
    log(f"  {inb.sum()}/{len(pts)} points project into the image")
    if inb.sum() == 0:
        raise RuntimeError(
            "No LiDAR points landed in the image. Either the extrinsic is badly wrong "
            "or the camera was not looking at the scanned surface for this frame."
        )

    # -- artefact 1: overlay PNG (depth-colored points on the RGB frame) ------ #
    overlay = image[:, :, ::-1].copy()  # RGB→BGR for cv2
    dmin, dmax = float(np.percentile(zi, 2)), float(np.percentile(zi, 98))
    cols = _depth_to_color(zi, dmin, dmax)
    # Draw points onto a separate layer, then alpha-blend so the photo shows through;
    # subsample with overlay_stride so dense sweeps don't paint over every edge.
    sel = slice(None, None, max(1, overlay_stride))
    layer = overlay.copy()
    drew = np.zeros((H, W), dtype=bool)
    for px, py, col in zip(ui[sel], vi[sel], cols[sel]):
        cv2.circle(layer, (int(px), int(py)), point_radius, tuple(int(c) for c in col), -1)
        drew[max(0, py - point_radius):py + point_radius + 1,
             max(0, px - point_radius):px + point_radius + 1] = True
    a = float(np.clip(overlay_alpha, 0.0, 1.0))
    overlay[drew] = (a * layer[drew] + (1 - a) * overlay[drew]).astype(np.uint8)
    cv2.putText(overlay, f"depth {dmin:.2f}-{dmax:.2f}m  pts={inb.sum()}",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    overlay_path = out_prefix.with_name(out_prefix.name + "_overlay.png")
    cv2.imwrite(str(overlay_path), overlay)
    log(f"  wrote {overlay_path.name}")

    # -- artefact 2: colored cloud PLY (sweep colored by projected pixel) ----- #
    sampled_rgb = image[vi, ui]  # (M,3) RGB
    cloud_path = out_prefix.with_name(out_prefix.name + "_cloud.ply")
    _write_cloud_ply(cloud_path, pts_in, sampled_rgb)
    log(f"  wrote {cloud_path.name}")

    return overlay_path, cloud_path


def _write_cloud_ply(path: Path, xyz: np.ndarray, rgb: np.ndarray) -> None:
    n = len(xyz)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    ).encode("ascii")
    vdt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                    ("r", np.uint8), ("g", np.uint8), ("b", np.uint8)])
    va = np.empty(n, dtype=vdt)
    va["x"], va["y"], va["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    va["r"], va["g"], va["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with path.open("wb") as fh:
        fh.write(header)
        fh.write(va.tobytes())
