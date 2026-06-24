#!/usr/bin/env python3
"""
static_projection_test.py — run the single-frame LiDAR↔camera projection test.

This is the calibration GATE (cam_lidar_calib_handoff.md / voxel_color_map_handoff.md
build-step 1). It projects one LiDAR sweep through the current T_cam_lidar onto one
RGB frame so you can eyeball whether the extrinsic is correct BEFORE investing in
the voxel color map.

Usage:
  source ~/ros2_ws/install/setup.bash
  python3 scripts/static_projection_test.py <session_dir> [options]

  --calibration / -c   YAML with T_cam_lidar (default: scripts/calib_lidar_camera.yaml)
  --out / -o           Output prefix (default: <session_dir>/static_test)
  --time / -t          Fraction through the bag to grab the RGB frame (default 0.5)
  --window / -w        Seconds of LiDAR sweeps to accumulate (default 0.15)
  --radius / -r        Overlay splat radius in px (default 2)

Outputs <prefix>_overlay.png and <prefix>_cloud.ply. See the module docstring for
how to read them.
"""

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
for _p in [_REPO_ROOT / "src/scanner_control", Path.home() / "ros2_ws/build/scanner_control"]:
    if _p.exists() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from scanner_control.colorize import load_calibration  # noqa: E402
from scanner_control.static_projection import run_static_projection  # noqa: E402


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("session_dir", help="Session directory containing the *.db3 bag")
    ap.add_argument("--calibration", "-c", default=None)
    ap.add_argument("--out", "-o", default=None)
    ap.add_argument("--time", "-t", type=float, default=0.5)
    ap.add_argument("--window", "-w", type=float, default=0.15)
    ap.add_argument("--radius", "-r", type=int, default=2)
    ap.add_argument("--stride", "-s", type=int, default=1,
                    help="draw every Nth point in the overlay (lets the photo show through)")
    ap.add_argument("--alpha", "-a", type=float, default=1.0,
                    help="overlay point opacity 0..1 (default 1.0)")
    args = ap.parse_args()

    session_dir = Path(args.session_dir).expanduser().resolve()
    if not session_dir.is_dir():
        print(f"ERROR: not a directory: {session_dir}", file=sys.stderr)
        sys.exit(1)

    calib_path = (
        Path(args.calibration).expanduser().resolve()
        if args.calibration
        else Path(__file__).parent / "calib_lidar_camera.yaml"
    )
    T_cam_lidar = load_calibration(calib_path)
    print(f"Calibration: {calib_path if calib_path.exists() else 'identity (placeholder)'}")

    out_prefix = (
        Path(args.out).expanduser().resolve() if args.out else session_dir / "static_test"
    )

    try:
        overlay, cloud = run_static_projection(
            bag_dir=session_dir,
            T_cam_lidar=T_cam_lidar,
            out_prefix=out_prefix,
            time_fraction=args.time,
            accumulate_window=args.window,
            point_radius=args.radius,
            overlay_stride=args.stride,
            overlay_alpha=args.alpha,
        )
        print(f"\nOverlay (read for edge alignment): {overlay}")
        print(f"Colored cloud (read for color-on-geometry): {cloud}")
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
