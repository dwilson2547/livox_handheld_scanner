#!/usr/bin/env python3
"""
colorize_mesh.py — Project RealSense color onto a reconstructed LiDAR mesh.

Usage:
  # Source ROS2 first, then:
  python3 scripts/colorize_mesh.py <session_dir> [options]

  --calibration / -c   YAML file with T_cam_lidar (default: scripts/calib_lidar_camera.yaml
                        next to this script, then identity if not found)
  --out / -o           Output PLY path  (default: <session_dir>/mesh_colored.ply)
  --interval / -i      Camera keyframe interval in seconds  (default: 0.5 = 2 fps)

Example:
  source ~/ros2_ws/install/setup.bash
  python3 scripts/colorize_mesh.py sessions/my_scan
"""

import argparse
import sys
from pathlib import Path

# Make scanner_control importable without installing it first (develop-mode build).
_REPO_ROOT = Path(__file__).resolve().parent.parent
_BUILD_CTRL = Path.home() / "ros2_ws/build/scanner_control"
for _p in [
    _REPO_ROOT / "src/scanner_control",
    _BUILD_CTRL,
]:
    if _p.exists() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from scanner_control.colorize import colorize_mesh, load_calibration  # noqa: E402


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("session_dir", help="Session directory containing mesh_dense_replay.ply + bag")
    ap.add_argument(
        "--calibration", "-c", default=None,
        help="YAML file with T_cam_lidar 4×4 matrix "
             "(default: scripts/calib_lidar_camera.yaml next to this script)",
    )
    ap.add_argument("--out", "-o", default=None, help="Output PLY path")
    ap.add_argument(
        "--interval", "-i", type=float, default=0.1,
        help="Camera keyframe interval in seconds (default: 0.1)",
    )
    args = ap.parse_args()

    session_dir = Path(args.session_dir).expanduser().resolve()
    if not session_dir.is_dir():
        print(f"ERROR: not a directory: {session_dir}", file=sys.stderr)
        sys.exit(1)

    # Default calibration: next to this script
    calib_path = (
        Path(args.calibration).expanduser().resolve()
        if args.calibration
        else Path(__file__).parent / "calib_lidar_camera.yaml"
    )
    T_cam_lidar = load_calibration(calib_path)
    print(f"Calibration from: {calib_path if calib_path.exists() else 'identity (placeholder)'}")

    out_path = Path(args.out).expanduser().resolve() if args.out else None

    try:
        result = colorize_mesh(
            session_dir=session_dir,
            T_cam_lidar=T_cam_lidar,
            out_path=out_path,
            keyframe_interval=args.interval,
        )
        print(f"\nColored mesh written to: {result}")
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
