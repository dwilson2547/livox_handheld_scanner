#!/usr/bin/env python3
"""
export_pointcloud.py — Export a session's full LiDAR scan as a LAS point cloud.

Preferred path: reads /cloud_registered from a bag produced by the "Process"
step in the control panel (already deskewed, world-frame, from Point-LIO).

Fallback: if no /cloud_registered bag exists, reads raw /livox/lidar frames
and applies per-frame TF poses.  This is fast but omits per-point IMU
deskewing, so moving scans will show smear at frame boundaries.

Usage:
  source ~/ros2_ws/install/setup.bash
  python3 scripts/export_pointcloud.py sessions/my_scan
  python3 scripts/export_pointcloud.py sessions/my_scan --out /tmp/cloud.las
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:
    sys.exit(f"ROS2 environment not sourced — run: source ~/ros2_ws/install/setup.bash\n({exc})")

try:
    import laspy
except ImportError:
    sys.exit("laspy not installed — run: pip3 install laspy")


_TF_PARENT = "camera_init"
_TF_CHILD = "aft_mapped"


def _tf_to_se3(xf) -> np.ndarray:
    tr = xf.transform.translation
    q = xf.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    n = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if n > 1e-9:
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)],
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [tr.x, tr.y, tr.z]
    return T


def export_pointcloud(session_dir: Path, out_path: Path) -> int:
    db3_files = sorted(session_dir.glob("*.db3"))
    if not db3_files:
        raise FileNotFoundError(f"No bag files in {session_dir}")
    bag_dir = db3_files[0].parent

    CustomMsg = get_message("livox_interfaces/msg/CustomMsg")
    TFMessage = get_message("tf2_msgs/msg/TFMessage")

    def _open(topics):
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
               rosbag2_py.ConverterOptions("", ""))
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
        return r

    # Pass 1: collect TF poses
    print("Pass 1: reading odometry …")
    odom_times: list[int] = []
    odom_mats: list[np.ndarray] = []
    r1 = _open(["/tf"])
    while r1.has_next():
        _, data, ts = r1.read_next()
        for xf in deserialize_message(data, TFMessage).transforms:
            if xf.header.frame_id == _TF_PARENT and xf.child_frame_id == _TF_CHILD:
                odom_times.append(ts)
                odom_mats.append(_tf_to_se3(xf))
    if not odom_times:
        raise RuntimeError(
            "No TF odometry (camera_init→aft_mapped) found — bag may predate Point-LIO output")
    odom_t = np.array(odom_times, dtype=np.int64)
    print(f"  {len(odom_t)} odometry frames")

    # Pass 2: accumulate LiDAR frames in world coordinates
    print("Pass 2: accumulating LiDAR frames …")
    chunks_xyz: list[np.ndarray] = []
    chunks_intensity: list[np.ndarray] = []
    n_frames = n_skipped = 0

    r2 = _open(["/livox/lidar"])
    while r2.has_next():
        _, data, ts = r2.read_next()
        msg = deserialize_message(data, CustomMsg)
        if msg.point_num == 0:
            continue

        # nearest odometry pose
        idx = int(np.searchsorted(odom_t, ts))
        if idx > 0 and (idx == len(odom_t) or
                        abs(odom_t[idx - 1] - ts) < abs(odom_t[idx] - ts)):
            idx -= 1
        if abs(odom_t[idx] - ts) > int(5e8):  # skip if pose > 500ms away
            n_skipped += 1
            continue

        T = odom_mats[idx]
        pts = msg.points

        xyz = np.array([[p.x, p.y, p.z] for p in pts], dtype=np.float32)
        tags = np.array([p.tag for p in pts], dtype=np.uint8)
        refl = np.array([p.reflectivity for p in pts], dtype=np.uint8)

        # tag & 0x30 == 0x00: LiDAR return is normal (not noise/edge artifact)
        valid = (tags & 0x30) == 0
        xyz = xyz[valid].astype(np.float64)
        refl = refl[valid]

        if len(xyz) == 0:
            continue

        # transform to world frame
        xyz_h = np.hstack([xyz, np.ones((len(xyz), 1))]).T
        world = (T @ xyz_h)[:3].T

        chunks_xyz.append(world.astype(np.float32))
        chunks_intensity.append(refl)
        n_frames += 1

        if n_frames % 100 == 0:
            total = sum(len(c) for c in chunks_xyz)
            print(f"  frame {n_frames}: {total:,} points …", end="\r", flush=True)

    print()
    if not chunks_xyz:
        raise RuntimeError("No points accumulated — check bag contents")

    xyz_all = np.vstack(chunks_xyz).astype(np.float64)
    int_all = np.concatenate(chunks_intensity).astype(np.uint16) * 256  # → uint16 range

    print(f"  {len(xyz_all):,} points from {n_frames} frames "
          f"({n_skipped} frames skipped — no nearby pose)")

    # Write LAS 1.4
    print(f"Writing {out_path} …")
    header = laspy.LasHeader(point_format=0, version="1.4")
    header.offsets = xyz_all.mean(axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])  # 1mm precision

    las = laspy.LasData(header=header)
    las.x = xyz_all[:, 0]
    las.y = xyz_all[:, 1]
    las.z = xyz_all[:, 2]
    las.intensity = int_all
    las.write(str(out_path))

    size_mb = out_path.stat().st_size / 1e6
    print(f"Done → {out_path}  ({size_mb:.0f} MB)")
    return len(xyz_all)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir", help="Session directory containing the bag")
    ap.add_argument("--out", "-o", default=None,
                    help="Output path (default: <session_dir>/pointcloud.las)")
    args = ap.parse_args()

    session_dir = Path(args.session_dir).expanduser()
    if not session_dir.is_dir():
        # try relative to workspace sessions/
        workspace_sessions = Path(__file__).resolve().parents[2] / "sessions"
        candidate = workspace_sessions / session_dir.name
        if candidate.is_dir():
            session_dir = candidate
        else:
            sys.exit(f"ERROR: session not found: {args.session_dir}\n"
                     f"  Tried: {session_dir.resolve()}\n"
                     f"  Tried: {candidate}")
    session_dir = session_dir.resolve()

    out = (Path(args.out).expanduser().resolve()
           if args.out else session_dir / "pointcloud.las")

    try:
        export_pointcloud(session_dir, out)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
