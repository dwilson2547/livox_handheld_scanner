#!/usr/bin/env python3
"""
build_voxel_map.py — build the probabilistic voxel color map for a session.

Prerequisite: a "cloud bag" containing Point-LIO's /cloud_registered AND
/aft_mapped_to_init, produced by replaying the raw session through Point-LIO:

  source ~/ros2_ws/install/setup.bash
  scripts/replay_to_cloud_bag.sh <session_dir>        # writes <session>/cloud_bag/

Then:
  python3 scripts/build_voxel_map.py <session_dir> [options]

  --cloud-bag          cloud bag dir (default: <session>/cloud_bag)
  --calibration / -c   T_cam_lidar YAML (default: scripts/calib_lidar_camera.yaml)
  --out / -o           output PLY (default: <session>/voxel_color_map.ply)
  --voxel-size         metres (default 0.02)
  --interval / -i      RGB keyframe interval seconds (default 0.2)
  --l-occ-min          occupancy export threshold / noise knob (default 0.85)
  --ray-clear          enable per-ray miss integration (slow; stronger denoise)
"""

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
for _p in [_REPO_ROOT / "src/scanner_control", Path.home() / "ros2_ws/build/scanner_control"]:
    if _p.exists() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from scanner_control.colorize import load_calibration  # noqa: E402
from scanner_control.voxel_build import build_voxel_map, export  # noqa: E402
from scanner_control.voxel_map import VoxelMapConfig  # noqa: E402


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("session_dir")
    ap.add_argument("--cloud-bag", default=None)
    ap.add_argument("--calibration", "-c", default=None)
    ap.add_argument("--out", "-o", default=None)
    ap.add_argument("--voxel-size", type=float, default=0.02)
    ap.add_argument("--interval", "-i", type=float, default=0.2)
    ap.add_argument("--l-occ-min", type=float, default=0.85)
    ap.add_argument("--ray-clear", action="store_true")
    args = ap.parse_args()

    session = Path(args.session_dir).expanduser().resolve()
    if not session.is_dir():
        print(f"ERROR: not a directory: {session}", file=sys.stderr)
        sys.exit(1)

    cloud_bag = Path(args.cloud_bag).expanduser().resolve() if args.cloud_bag else session / "cloud_bag"
    if not cloud_bag.is_dir():
        print(f"ERROR: cloud bag not found: {cloud_bag}\n"
              f"  Generate it first:  scripts/replay_to_cloud_bag.sh {session}", file=sys.stderr)
        sys.exit(1)

    calib_path = (
        Path(args.calibration).expanduser().resolve()
        if args.calibration else Path(__file__).parent / "calib_lidar_camera.yaml"
    )
    T_cam_lidar = load_calibration(calib_path)
    out_path = Path(args.out).expanduser().resolve() if args.out else session / "voxel_color_map.ply"

    cfg = VoxelMapConfig(voxel_size=args.voxel_size, l_occ_min=args.l_occ_min)

    try:
        vm = build_voxel_map(
            cloud_bag=cloud_bag,
            source_bag=session,
            T_cam_lidar=T_cam_lidar,
            config=cfg,
            keyframe_interval=args.interval,
            ray_clear=args.ray_clear,
        )
        export(vm, out_path, with_color=True)
        print(f"\nVoxel color map written to: {out_path}")
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
