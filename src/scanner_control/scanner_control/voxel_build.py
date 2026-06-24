#!/usr/bin/env python3
"""
voxel_build — drive the probabilistic voxel map from session bags.

Two inputs, correlated by header timestamp (the sensor-time domain, NOT bag
receive time):

  * geometry: Point-LIO's /cloud_registered (deskewed, world-frame) + odometry
    (/aft_mapped_to_init). This is the source the project doctrine mandates —
    raw /livox/lidar single-pose accumulation produces "disconnected blobs".
  * color:    the original session bag's RGB image, aligned depth image, and
    camera_info, plus the same trajectory.

Pipeline (handoff §Per-Frame Update Loop):
  occupancy : integrate each /cloud_registered sweep (vectorized endpoint hits;
              optional ray-clearing for stronger noise rejection).
  color     : for each RGB keyframe, interpolate the trajectory to THAT image's
              timestamp (the temporal fix), project visible occupied voxels,
              reject occluded ones via the depth image, weight each sample by
              view-angle · range · motion-ω, and feed the robust median.

This module is the ROS-facing front-end; the math/data-structures live in the
ROS-free voxel_map.py so they stay unit-testable.
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

from scanner_control.colorize import _decode_image_msg, _odom_to_se3
from scanner_control.voxel_map import (
    VoxelMap,
    VoxelMapConfig,
    motion_weight,
    range_weight,
    write_voxel_ply,
)

_CLOUD = "/cloud_registered"
_ODOM = "/aft_mapped_to_init"
_COLOR_IMG = "/camera/d435i/color/image_raw"
_DEPTH_IMG = "/camera/d435i/aligned_depth_to_color/image_raw"
_COLOR_INFO = "/camera/d435i/color/camera_info"


# --------------------------------------------------------------------------- #
#  Bag helpers
# --------------------------------------------------------------------------- #

def _reader(bag_dir: Path, topics: Optional[List[str]] = None):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    if topics:
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def _bag_dir(p: Path) -> Path:
    db3 = sorted(p.glob("*.db3"))
    if not db3:
        raise FileNotFoundError(f"No *.db3 in {p}")
    return db3[0].parent


def _topics(bag_dir: Path) -> set:
    r = _reader(bag_dir)
    return {t.name for t in r.get_all_topics_and_types()}


def _stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _pointcloud2_to_xyz(msg) -> np.ndarray:
    """
    sensor_msgs/PointCloud2 → (N,3) float32 xyz. Parses by field offset / point_step
    (the project's documented gotcha: do NOT byte-stride; read full float32 fields).
    """
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        raise ValueError(f"PointCloud2 missing xyz fields; has {list(fields)}")
    n = msg.width * msg.height
    step = msg.point_step
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)
    out = np.empty((n, 3), dtype=np.float32)
    for i, name in enumerate(("x", "y", "z")):
        off = fields[name].offset
        out[:, i] = np.frombuffer(raw[:, off:off + 4].tobytes(), dtype="<f4")
    finite = np.isfinite(out).all(axis=1)
    return out[finite]


class _Trajectory:
    """Time-indexed SE3 poses (T_world_lidar) with linear interpolation."""

    def __init__(self, times_ns: np.ndarray, mats: List[np.ndarray]):
        order = np.argsort(times_ns)
        self.t = times_ns[order]
        self.T = [mats[i] for i in order]

    def __len__(self):
        return len(self.t)

    def pose_at(self, ts: int) -> np.ndarray:
        """Interpolate to ts: lerp translation, nlerp-ish slerp on rotation (small dt)."""
        i = int(np.searchsorted(self.t, ts))
        if i <= 0:
            return self.T[0]
        if i >= len(self.t):
            return self.T[-1]
        t0, t1 = self.t[i - 1], self.t[i]
        T0, T1 = self.T[i - 1], self.T[i]
        if t1 == t0:
            return T0
        a = (ts - t0) / (t1 - t0)
        out = np.eye(4)
        out[:3, 3] = (1 - a) * T0[:3, 3] + a * T1[:3, 3]
        out[:3, :3] = _slerp_rot(T0[:3, :3], T1[:3, :3], a)
        return out

    def omega_at(self, ts: int) -> float:
        """Angular speed magnitude (rad/s) from adjacent poses around ts."""
        i = int(np.searchsorted(self.t, ts))
        i = max(1, min(i, len(self.t) - 1))
        dt = (self.t[i] - self.t[i - 1]) * 1e-9
        if dt <= 0:
            return 0.0
        dR = self.T[i][:3, :3].T @ self.T[i - 1][:3, :3]
        ang = np.arccos(np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0))
        return float(ang / dt)


def _slerp_rot(R0: np.ndarray, R1: np.ndarray, a: float) -> np.ndarray:
    """Slerp between two rotation matrices via the relative axis-angle."""
    dR = R0.T @ R1
    ang = np.arccos(np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0))
    if ang < 1e-8:
        return R0
    ax = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
    ax = ax / (2 * np.sin(ang))
    th = ang * a
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    R_step = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
    return R0 @ R_step


# --------------------------------------------------------------------------- #
#  Build
# --------------------------------------------------------------------------- #

def build_voxel_map(
    cloud_bag: Path,
    source_bag: Path,
    T_cam_lidar: np.ndarray,
    config: Optional[VoxelMapConfig] = None,
    keyframe_interval: float = 0.2,
    ray_clear: bool = False,
    clear_subsample: int = 8,
    occlusion_eps: float = 0.05,
    log: Callable[[str], None] = print,
) -> VoxelMap:
    """
    Parameters
    ----------
    cloud_bag         : bag dir with /cloud_registered + /aft_mapped_to_init
    source_bag        : original session bag dir (color/depth/camera_info)
    T_cam_lidar       : 4×4, LiDAR-frame point → camera-frame point
    keyframe_interval : seconds between sampled RGB frames
    ray_clear         : enable per-ray miss integration (slow; stronger denoise)
    clear_subsample   : when ray_clear, clear rays for every Nth point
    occlusion_eps     : metres a voxel may be behind the depth surface and still pass
    """
    if not _HAS_ROS:
        raise RuntimeError("rosbag2_py unavailable — source the ROS 2 environment first.")
    cfg = config or VoxelMapConfig()
    cloud_bag = _bag_dir(cloud_bag)
    source_bag = _bag_dir(source_bag)

    PointCloud2 = get_message("sensor_msgs/msg/PointCloud2")
    Odometry = get_message("nav_msgs/msg/Odometry")
    Image = get_message("sensor_msgs/msg/Image")
    CameraInfo = get_message("sensor_msgs/msg/CameraInfo")

    # -- trajectory ---------------------------------------------------------- #
    log("Reading trajectory …")
    traj = _read_trajectory(cloud_bag, source_bag, Odometry)
    log(f"  {len(traj)} poses spanning {(traj.t[-1]-traj.t[0])*1e-9:.1f}s")

    # -- occupancy ----------------------------------------------------------- #
    vm = VoxelMap(cfg)
    log(f"Integrating occupancy from {_CLOUD} (voxel {cfg.voxel_size*100:.0f}mm, "
        f"ray_clear={ray_clear}) …")
    n_sweeps = 0
    n_pts = 0
    r = _reader(cloud_bag, [_CLOUD])
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != _CLOUD:
            continue
        msg = deserialize_message(data, PointCloud2)
        pts = _pointcloud2_to_xyz(msg)
        if len(pts) == 0:
            continue
        if ray_clear:
            origin = traj.pose_at(_stamp_ns(msg.header))[:3, 3]
            for p in pts[::max(1, clear_subsample)]:
                vm.integrate_ray(origin, p)
            # also fold in all endpoints densely (clearing pass was subsampled)
            vm.integrate_hits_batch(pts)
        else:
            vm.integrate_hits_batch(pts)
        n_sweeps += 1
        n_pts += len(pts)
        if n_sweeps % 50 == 0:
            log(f"  {n_sweeps} sweeps, {len(vm)} voxels touched")
    occ = sum(1 for _ in vm.occupied_keys())
    log(f"  {n_sweeps} sweeps, {n_pts} pts → {len(vm)} voxels, {occ} occupied "
        f"(≥ L_OCC_MIN={cfg.l_occ_min})")

    # -- color --------------------------------------------------------------- #
    _colorize_voxels(
        vm, source_bag, traj, T_cam_lidar,
        Image, CameraInfo, keyframe_interval, occlusion_eps, log,
    )
    return vm


def _read_trajectory(cloud_bag: Path, source_bag: Path, Odometry) -> _Trajectory:
    """Prefer /aft_mapped_to_init from the cloud bag; fall back to the source bag."""
    for bag in (cloud_bag, source_bag):
        if _ODOM not in _topics(bag):
            continue
        times, mats = [], []
        r = _reader(bag, [_ODOM])
        while r.has_next():
            topic, data, _ = r.read_next()
            if topic != _ODOM:
                continue
            msg = deserialize_message(data, Odometry)
            times.append(_stamp_ns(msg.header))
            mats.append(_odom_to_se3(msg))
        if times:
            return _Trajectory(np.array(times, dtype=np.int64), mats)
    raise RuntimeError(
        f"No {_ODOM} in either bag. Record it alongside /cloud_registered during replay."
    )


def _colorize_voxels(
    vm: VoxelMap,
    source_bag: Path,
    traj: _Trajectory,
    T_cam_lidar: np.ndarray,
    Image,
    CameraInfo,
    keyframe_interval: float,
    occlusion_eps: float,
    log: Callable[[str], None],
) -> None:
    # camera intrinsics
    cam_info = None
    r = _reader(source_bag, [_COLOR_INFO])
    while r.has_next() and cam_info is None:
        topic, data, _ = r.read_next()
        if topic == _COLOR_INFO:
            cam_info = deserialize_message(data, CameraInfo)
    if cam_info is None:
        raise RuntimeError("No camera_info in source bag")
    K = np.array(cam_info.k, dtype=np.float64).reshape(3, 3)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    W, H = int(cam_info.width), int(cam_info.height)

    # snapshot occupied voxel centers once (occupancy is frozen before coloring)
    occ_keys = list(vm.occupied_keys())
    if not occ_keys:
        log("  no occupied voxels — skipping color")
        return
    centers = np.array([vm.center_of(k) for k in occ_keys], dtype=np.float64)  # (M,3) world
    centers_h = np.hstack([centers, np.ones((len(centers), 1))]).T             # (4,M)
    # crude surface normal per voxel = direction toward the trajectory centroid
    # (good enough for the view-angle weight; PCA normals come with plane detection).
    traj_centroid = np.mean([T[:3, 3] for T in traj.T], axis=0)

    log(f"Coloring {len(occ_keys)} occupied voxels from RGB keyframes "
        f"(every {keyframe_interval}s) …")
    interval_ns = int(keyframe_interval * 1e9)
    last_kf = -(1 << 62)
    n_kf = 0

    # depth frames indexed by stamp for occlusion lookup
    depth_index = _index_depth(source_bag, Image)

    r = _reader(source_bag, [_COLOR_IMG])
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != _COLOR_IMG:
            continue
        msg = deserialize_message(data, Image)
        ts = _stamp_ns(msg.header)
        if ts - last_kf < interval_ns:
            continue
        last_kf = ts

        # pose interpolated to THIS image's timestamp (the temporal fix)
        T_world_lidar = traj.pose_at(ts)
        T_cam_world = T_cam_lidar @ np.linalg.inv(T_world_lidar)
        cam = (T_cam_world @ centers_h)[:3]   # (3,M) voxel centers in camera frame
        z = cam[2]
        front = z > 0.1
        if not front.any():
            continue
        u = fx * cam[0] / z + cx
        v = fy * cam[1] / z + cy
        inb = front & (u >= 0) & (u < W) & (v >= 0) & (v < H)
        if not inb.any():
            continue

        idx = np.where(inb)[0]
        ui = u[idx].astype(np.int32)
        vi = v[idx].astype(np.int32)
        zv = z[idx]

        # occlusion: compare voxel depth to the aligned depth image (metric, mm→m)
        depth_img = _nearest_depth(depth_index, ts)
        keep = np.ones(len(idx), dtype=bool)
        if depth_img is not None:
            d_cam = depth_img[vi, ui].astype(np.float32) * 0.001  # D435 depth is uint16 mm
            valid = d_cam > 0.05
            occluded = valid & (d_cam < (zv - occlusion_eps))
            keep = ~occluded

        if not keep.any():
            continue
        img = _decode_image_msg(msg)  # RGB
        cam_pos = T_world_lidar[:3, 3]
        omega = traj.omega_at(ts)
        w_motion = motion_weight(omega, vm.cfg.motion_k)

        kidx = idx[keep]
        kui, kvi, kzv = ui[keep], vi[keep], zv[keep]
        rgb = img[kvi, kui]  # (K,3)
        for j, vox_i in enumerate(kidx):
            key = occ_keys[vox_i]
            # view-angle weight via crude normal (voxel→trajectory direction)
            n = centers[vox_i] - traj_centroid
            ray = centers[vox_i] - cam_pos
            w_view = _abs_cos(n, ray, vm.cfg.view_angle_min_cos)
            w = w_motion * w_view * range_weight(float(kzv[j]), vm.cfg.range_falloff)
            vm.add_color(key, rgb[j], w)
        n_kf += 1
        if n_kf % 25 == 0:
            log(f"  keyframe {n_kf}: last added {keep.sum()} samples")
    log(f"  colored from {n_kf} keyframes")


def _abs_cos(n: np.ndarray, r: np.ndarray, min_cos: float) -> float:
    nn = np.linalg.norm(n)
    rr = np.linalg.norm(r)
    if nn < 1e-9 or rr < 1e-9:
        return 1.0
    c = abs(float(np.dot(n, r)) / (nn * rr))
    return c if c >= min_cos else 0.0


def _index_depth(source_bag: Path, Image) -> List[Tuple[int, object]]:
    if _DEPTH_IMG not in _topics(source_bag):
        return []
    out = []
    r = _reader(source_bag, [_DEPTH_IMG])
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == _DEPTH_IMG:
            msg = deserialize_message(data, Image)
            out.append((_stamp_ns(msg.header), msg))
    out.sort(key=lambda x: x[0])
    return out


def _nearest_depth(index, ts: int, max_dt_ns: int = 80_000_000):
    if not index:
        return None
    times = [t for t, _ in index]
    i = int(np.searchsorted(times, ts))
    best = None
    for cand in (i - 1, i):
        if 0 <= cand < len(index) and abs(index[cand][0] - ts) <= max_dt_ns:
            if best is None or abs(index[cand][0] - ts) < abs(index[best][0] - ts):
                best = cand
    if best is None:
        return None
    msg = index[best][1]
    h, w = msg.height, msg.width
    return np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(h, w)


def export(vm: VoxelMap, out_path: Path, with_color: bool = True) -> Path:
    centers, colors = vm.export_points(with_color=with_color)
    write_voxel_ply(out_path, centers, colors)
    return out_path
