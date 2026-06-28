#!/usr/bin/env python3
"""
voxel_ply_to_las.py — convert a colored voxel-map PLY (from build_voxel_map.py) to a
colored LAS so it can go through the proven LAS→Potree pipeline (potree.sh).

PotreeConverter's PLY reader is flaky (it crashes indexing some valid binary PLYs);
LAS point-format 2 (XYZ + 16-bit RGB) is the format the rest of this project already
trusts, so we route colored voxel clouds through it.

  python3 scripts/voxel_ply_to_las.py <voxel.ply> [-o out.las]
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import laspy
except ImportError:
    sys.exit("laspy not installed — run: pip3 install laspy")


def read_voxel_ply(path: Path):
    """Read the binary-little-endian PLY written by voxel_map.write_voxel_ply:
    x,y,z float32 + red,green,blue uint8. Returns (xyz Nx3 float32, rgb Nx3 uint8)."""
    with path.open("rb") as fh:
        magic = fh.readline().strip()
        if magic != b"ply":
            sys.exit(f"{path} is not a PLY file")
        n = 0
        fmt = None
        while True:
            line = fh.readline()
            if not line:
                sys.exit("unexpected EOF in PLY header")
            t = line.strip()
            if t.startswith(b"format"):
                fmt = t.split()[1]
            elif t.startswith(b"element vertex"):
                n = int(t.split()[2])
            elif t == b"end_header":
                break
        if fmt != b"binary_little_endian":
            sys.exit(f"unsupported PLY format {fmt!r}; expected binary_little_endian")
        dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                       ("r", np.uint8), ("g", np.uint8), ("b", np.uint8)])
        va = np.frombuffer(fh.read(n * dt.itemsize), dtype=dt, count=n)
    xyz = np.column_stack([va["x"], va["y"], va["z"]]).astype(np.float64)
    rgb = np.column_stack([va["r"], va["g"], va["b"]]).astype(np.uint16)
    return xyz, rgb


def write_colored_las(xyz: np.ndarray, rgb8: np.ndarray, out_path: Path) -> None:
    header = laspy.LasHeader(point_format=2, version="1.4")   # fmt 2 = XYZ + RGB
    header.offsets = xyz.mean(axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])           # 1 mm precision
    las = laspy.LasData(header=header)
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    # LAS RGB is 16-bit; scale 8-bit (0..255) to fill 0..65535 so viewers read it right.
    scaled = (rgb8 * 257).astype(np.uint16)
    las.red, las.green, las.blue = scaled[:, 0], scaled[:, 1], scaled[:, 2]
    las.write(str(out_path))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ply")
    ap.add_argument("--out", "-o", default=None)
    args = ap.parse_args()

    ply = Path(args.ply).expanduser().resolve()
    if not ply.is_file():
        sys.exit(f"not a file: {ply}")
    out = Path(args.out).expanduser().resolve() if args.out else ply.with_suffix(".las")

    xyz, rgb = read_voxel_ply(ply)
    write_colored_las(xyz, rgb, out)
    print(f"{len(xyz):,} points → {out}  ({out.stat().st_size/1e6:.1f} MB)")


if __name__ == "__main__":
    main()
