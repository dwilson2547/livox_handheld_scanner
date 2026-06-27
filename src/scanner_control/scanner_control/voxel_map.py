#!/usr/bin/env python3
"""
voxel_map — probabilistic voxel map with log-odds occupancy and robust per-voxel
color accumulation.

This is the geometry+color core described in voxel_color_map_handoff.md. It is
deliberately ROS-free and depends only on numpy, so the occupancy logic can be
unit-tested without hardware or a sourced ROS environment. Bag I/O and the
camera-projection front-end (which feed this core) live elsewhere and are gated
on a verified camera→LiDAR extrinsic — see cam_lidar_calib_handoff.md.

Build order (handoff §"Build / Validation Order"):
  2. occupancy: VoxelMap + log-odds + ray clearing   ← THIS MODULE (geometry)
  3-5. color: ColorAccumulator + weights + occlusion  ← THIS MODULE (data structures;
       the projection that feeds add_color() is wired separately, after calib)

The two halves are intentionally separable: you can build and visualize occupancy
with zero camera involvement, exactly as the handoff prescribes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterator, Optional, Tuple

import numpy as np

VoxelKey = Tuple[int, int, int]


# --------------------------------------------------------------------------- #
#  Config
# --------------------------------------------------------------------------- #

@dataclass
class VoxelMapConfig:
    """Tunable parameters. Defaults follow the handoff's recommended starting values."""

    # Geometry --------------------------------------------------------------- #
    voxel_size: float = 0.02          # metres. Handoff: start at 2cm (≥ LIO pose error).

    # Log-odds occupancy (OctoMap-style) ------------------------------------- #
    l_hit: float = 0.85               # ln(0.7/0.3): endpoint evidence
    l_miss: float = -0.40             # ln(0.4/0.6): ray pass-through evidence
    l_min: float = -2.0               # clamp (allows recovery from transient errors)
    l_max: float = 3.5                # clamp (prevents over-confidence)
    l_occ_min: float = 0.85           # threshold to call a voxel "occupied" / export it
                                      #   → this is the noise-floor knob (raise = stricter)

    # Color accumulation ----------------------------------------------------- #
    color_reservoir: int = 64         # bounded best-N sample buffer per voxel (median;
                                      #   lowest-weight eviction — handoff pt_2 §2)
    n_min_color: int = 3              # min samples for a confident exported color

    # Per-sample color weights ---------------------------------------------- #
    view_angle_min_cos: float = 0.34  # drop samples grazing beyond ~70° (cos 70° ≈ 0.34)
    range_falloff: float = 1.0        # range weight = 1/(1 + range_falloff*range²)
    motion_k: float = 1.0             # angular-velocity weight = exp(-motion_k*|ω|)


# --------------------------------------------------------------------------- #
#  Robust color accumulator (handoff §ColorAccumulator, Option A)
# --------------------------------------------------------------------------- #

class ColorAccumulator:
    """
    Bounded reservoir of weighted RGB samples; reports the per-channel weighted
    median. Robust to rolling-shutter fliers / reflections in a way a running
    mean is not (a single bad frame cannot drag the result).

    Eviction keeps the *best-N* observations, not the most recent (handoff pt_2
    §2). A voxel on a thorough scan is seen far more than `capacity` times, so on
    overflow we drop the **lowest-weight** retained sample rather than the oldest
    — otherwise a clean face-on early frame could be evicted in favour of a later
    grazing-angle / fast-pan one, degrading the surviving set over the scan.
    """

    __slots__ = ("_rgb", "_w", "_n", "_cap")

    def __init__(self, capacity: int = 64):
        self._cap = capacity
        self._rgb = np.zeros((capacity, 3), dtype=np.float32)
        self._w = np.zeros(capacity, dtype=np.float32)
        self._n = 0          # number of valid samples (<= cap)

    def add(self, rgb, weight: float) -> None:
        if weight <= 0.0:
            return
        if self._n < self._cap:
            self._rgb[self._n] = rgb
            self._w[self._n] = weight
            self._n += 1
            return
        # Full: replace the lowest-weight sample, but only if the incoming sample
        # outranks it (otherwise it's worse than everything we kept → drop it).
        j = int(np.argmin(self._w))
        if weight > self._w[j]:
            self._rgb[j] = rgb
            self._w[j] = weight

    @property
    def sample_count(self) -> int:
        return self._n

    def result(self) -> np.ndarray:
        """Per-channel weighted median as uint8 RGB. Zeros if empty."""
        if self._n == 0:
            return np.zeros(3, dtype=np.uint8)
        rgb = self._rgb[: self._n]
        w = self._w[: self._n]
        out = np.empty(3, dtype=np.uint8)
        for c in range(3):
            out[c] = _weighted_median(rgb[:, c], w)
        return out


def _weighted_median(values: np.ndarray, weights: np.ndarray) -> float:
    """Weighted median: smallest v where cumulative weight crosses half the total."""
    order = np.argsort(values, kind="stable")
    v = values[order]
    cw = np.cumsum(weights[order])
    half = 0.5 * cw[-1]
    idx = int(np.searchsorted(cw, half))
    idx = min(idx, len(v) - 1)
    return float(v[idx])


# --------------------------------------------------------------------------- #
#  Voxel + map
# --------------------------------------------------------------------------- #

@dataclass
class Voxel:
    log_odds: float = 0.0
    hit_count: int = 0
    color: Optional[ColorAccumulator] = None


class VoxelMap:
    """
    Sparse spatial-hash voxel map. Most of the world is empty, so we only ever
    allocate voxels we touch.
    """

    def __init__(self, config: Optional[VoxelMapConfig] = None):
        self.cfg = config or VoxelMapConfig()
        self._voxels: Dict[VoxelKey, Voxel] = {}
        self._inv_size = 1.0 / self.cfg.voxel_size

    # -- keys / centers ------------------------------------------------------ #

    def key_of(self, point) -> VoxelKey:
        """World coordinate → integer voxel key (floored division by voxel size)."""
        return (
            int(math.floor(point[0] * self._inv_size)),
            int(math.floor(point[1] * self._inv_size)),
            int(math.floor(point[2] * self._inv_size)),
        )

    def center_of(self, key: VoxelKey) -> np.ndarray:
        s = self.cfg.voxel_size
        return np.array([(key[0] + 0.5) * s, (key[1] + 0.5) * s, (key[2] + 0.5) * s])

    def get(self, key: VoxelKey) -> Optional[Voxel]:
        return self._voxels.get(key)

    def _voxel_at(self, key: VoxelKey) -> Voxel:
        v = self._voxels.get(key)
        if v is None:
            v = Voxel()
            self._voxels[key] = v
        return v

    def __len__(self) -> int:
        return len(self._voxels)

    # -- occupancy ----------------------------------------------------------- #

    def _apply_hit(self, key: VoxelKey) -> Voxel:
        v = self._voxel_at(key)
        v.log_odds = min(v.log_odds + self.cfg.l_hit, self.cfg.l_max)
        v.hit_count += 1
        return v

    def _apply_miss(self, key: VoxelKey) -> None:
        v = self._voxel_at(key)
        v.log_odds = max(v.log_odds + self.cfg.l_miss, self.cfg.l_min)

    def integrate_ray(self, origin, endpoint, clear: bool = True) -> Voxel:
        """
        Integrate one LiDAR return: apply L_MISS to every voxel the ray passes
        through (miss evidence) and L_HIT to the endpoint voxel (hit evidence).

        Ray-clearing is the noise-rejection core: a spurious one-off return gets a
        single +L_HIT, but later rays to real surfaces pass *through* that empty
        space and repeatedly drive it back below threshold. Returns the endpoint
        voxel so a caller can attach color to it.
        """
        end_key = self.key_of(endpoint)
        if clear:
            for k in _voxel_traversal(origin, endpoint, self.cfg.voxel_size):
                if k == end_key:
                    break  # never apply a miss to the endpoint itself
                self._apply_miss(k)
        return self._apply_hit(end_key)

    def integrate_hit_only(self, endpoint) -> Voxel:
        """Endpoint hit with no ray clearing (cheaper; weaker noise rejection)."""
        return self._apply_hit(self.key_of(endpoint))

    def integrate_hits_batch(self, points: np.ndarray) -> None:
        """
        Vectorized endpoint integration for a whole sweep (no ray clearing).

        Floors all points to keys at once and folds in l_hit·(count) per voxel with
        clamping. Orders of magnitude faster than per-point Python; this is the path
        used for interactive whole-session occupancy. Ray clearing (the stronger
        noise rejection) is the per-ray integrate_ray() path, opt-in for offline runs.
        """
        if len(points) == 0:
            return
        keys = np.floor(np.asarray(points, dtype=np.float64) * self._inv_size).astype(np.int64)
        uniq, counts = np.unique(keys, axis=0, return_counts=True)
        lh, lmax = self.cfg.l_hit, self.cfg.l_max
        for row, c in zip(uniq, counts):
            k = (int(row[0]), int(row[1]), int(row[2]))
            v = self._voxel_at(k)
            v.log_odds = min(v.log_odds + lh * int(c), lmax)
            v.hit_count += int(c)

    # -- color --------------------------------------------------------------- #

    def add_color(self, key: VoxelKey, rgb, weight: float) -> None:
        v = self._voxels.get(key)
        if v is None:
            return  # only color voxels that exist (i.e. have geometry evidence)
        if v.color is None:
            v.color = ColorAccumulator(self.cfg.color_reservoir)
        v.color.add(rgb, weight)

    # -- iteration / export -------------------------------------------------- #

    def occupied_keys(self) -> Iterator[VoxelKey]:
        thr = self.cfg.l_occ_min
        for k, v in self._voxels.items():
            if v.log_odds >= thr:
                yield k

    def export_points(self, with_color: bool = False):
        """
        Return (centers Nx3 float32, colors Nx3 uint8 | None) for occupied voxels.
        Colors are the per-voxel weighted median; voxels below n_min_color samples
        get a flag color (magenta) so low-confidence color is visible, not silent.
        """
        centers = []
        colors = [] if with_color else None
        flag = np.array([255, 0, 255], dtype=np.uint8)
        for k in self.occupied_keys():
            centers.append(self.center_of(k))
            if with_color:
                v = self._voxels[k]
                if v.color is not None and v.color.sample_count >= self.cfg.n_min_color:
                    colors.append(v.color.result())
                else:
                    colors.append(flag)
        c = np.asarray(centers, dtype=np.float32).reshape(-1, 3)
        col = np.asarray(colors, dtype=np.uint8).reshape(-1, 3) if with_color else None
        return c, col


# --------------------------------------------------------------------------- #
#  Amanatides–Woo voxel traversal (exact DDA over a uniform grid)
# --------------------------------------------------------------------------- #

def _voxel_traversal(origin, endpoint, voxel_size: float) -> Iterator[VoxelKey]:
    """
    Yield every voxel key the segment origin→endpoint passes through, in order,
    including both the origin voxel and the endpoint voxel. Exact integer grid
    traversal (Amanatides & Woo, 1987) — no stepping artefacts or skipped voxels.
    """
    o = np.asarray(origin, dtype=np.float64)
    e = np.asarray(endpoint, dtype=np.float64)
    inv = 1.0 / voxel_size

    ix, iy, iz = (int(math.floor(o[0] * inv)),
                  int(math.floor(o[1] * inv)),
                  int(math.floor(o[2] * inv)))
    ex, ey, ez = (int(math.floor(e[0] * inv)),
                  int(math.floor(e[1] * inv)),
                  int(math.floor(e[2] * inv)))

    d = e - o
    step = [0, 0, 0]
    t_max = [math.inf, math.inf, math.inf]
    t_delta = [math.inf, math.inf, math.inf]
    cur = [ix, iy, iz]

    for a in range(3):
        if d[a] > 0:
            step[a] = 1
            next_boundary = (cur[a] + 1) * voxel_size
            t_max[a] = (next_boundary - o[a]) / d[a]
            t_delta[a] = voxel_size / d[a]
        elif d[a] < 0:
            step[a] = -1
            next_boundary = cur[a] * voxel_size
            t_max[a] = (next_boundary - o[a]) / d[a]
            t_delta[a] = -voxel_size / d[a]

    target = (ex, ey, ez)
    yield (cur[0], cur[1], cur[2])
    # Bound iterations so a degenerate ray can never spin forever.
    max_steps = abs(ex - ix) + abs(ey - iy) + abs(ez - iz) + 1
    for _ in range(max_steps):
        if (cur[0], cur[1], cur[2]) == target:
            return
        axis = 0 if t_max[0] <= t_max[1] and t_max[0] <= t_max[2] else (
            1 if t_max[1] <= t_max[2] else 2)
        cur[axis] += step[axis]
        t_max[axis] += t_delta[axis]
        yield (cur[0], cur[1], cur[2])


# --------------------------------------------------------------------------- #
#  Per-sample color weights (handoff §Per-Sample Color Weight)
# --------------------------------------------------------------------------- #

def view_angle_weight(surface_normal, cam_ray, min_cos: float) -> float:
    """cos(theta) between voxel surface normal and camera ray; 0 below the cutoff.
    Face-on surfaces (cos→1) project cleanly; grazing angles smear and alias."""
    n = np.asarray(surface_normal, dtype=np.float64)
    r = np.asarray(cam_ray, dtype=np.float64)
    nn = np.linalg.norm(n)
    nr = np.linalg.norm(r)
    if nn < 1e-9 or nr < 1e-9:
        return 0.0
    c = abs(float(np.dot(n, r)) / (nn * nr))
    return c if c >= min_cos else 0.0


def range_weight(range_m: float, falloff: float) -> float:
    """Down-weight distant samples (D435 RGB↔depth alignment degrades with range)."""
    return 1.0 / (1.0 + falloff * range_m * range_m)


def motion_weight(omega_mag: float, k: float) -> float:
    """Down-weight high angular-velocity frames (rolling-shutter skew).
    The highest-leverage weight and nearly free — ω comes from the trajectory."""
    return math.exp(-k * abs(omega_mag))


# --------------------------------------------------------------------------- #
#  PLY export (reuse colorize's writer when available; fallback otherwise)
# --------------------------------------------------------------------------- #

def write_voxel_ply(path: Path, centers: np.ndarray, colors: Optional[np.ndarray]) -> None:
    """Write occupied voxel centers as a colored point-cloud PLY for inspection."""
    n = len(centers)
    if colors is None:
        colors = np.full((n, 3), 200, dtype=np.uint8)  # neutral grey for geometry-only
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
    va["x"], va["y"], va["z"] = centers[:, 0], centers[:, 1], centers[:, 2]
    va["r"], va["g"], va["b"] = colors[:, 0], colors[:, 1], colors[:, 2]
    with path.open("wb") as fh:
        fh.write(header)
        fh.write(va.tobytes())
