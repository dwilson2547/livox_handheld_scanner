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
    n_min_hits: int = 1               # export gate: drop voxels hit < N times. The cheap
                                      #   denoise (pt_2 / handoff) — a one-off flier has
                                      #   hit_count==1, so n_min_hits=2 rejects it without
                                      #   any ray-clearing. 1 = no gating (default).

    # Color accumulation ----------------------------------------------------- #
    color_reservoir: int = 64         # bounded best-N sample buffer per voxel (median;
                                      #   lowest-weight eviction — handoff pt_2 §2)
    n_min_color: int = 3              # min samples for a confident exported color
    color_vector_median: bool = False # handoff pt_2 §3: use the weighted vector medoid
                                      #   (an actually-observed sample) instead of the
                                      #   per-channel median, which on a mixed-color
                                      #   voxel can output a triplet no sample had.

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

    def result(self, vector_median: bool = False) -> np.ndarray:
        """Robust per-voxel color as uint8 RGB. Zeros if empty.

        Default is the per-channel weighted median (cheap, robust to fliers). With
        ``vector_median`` it returns the weighted **medoid** — the retained sample
        minimizing the weighted sum of distances to all others (handoff pt_2 §3).
        The medoid is guaranteed to be a color that was actually observed, so it
        cannot invent a hue on a mixed-color voxel the way independent per-channel
        medians can."""
        if self._n == 0:
            return np.zeros(3, dtype=np.uint8)
        rgb = self._rgb[: self._n]
        w = self._w[: self._n]
        if vector_median:
            return _weighted_medoid(rgb, w)
        out = np.empty(3, dtype=np.uint8)
        for c in range(3):
            out[c] = _weighted_median(rgb[:, c], w)
        return out


def _weighted_medoid(rgb: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """Weighted vector medoid: the sample minimizing Σ_j w_j·‖rgb_i − rgb_j‖.
    Returns an actually-observed color (never an invented per-channel mix). O(n²)
    over the bounded reservoir (n ≤ capacity), evaluated only at export."""
    diff = rgb[:, None, :] - rgb[None, :, :]          # (n, n, 3)
    dist = np.sqrt((diff * diff).sum(axis=2))         # (n, n) pairwise distances
    cost = dist @ weights                             # (n,) weighted distance sum
    return rgb[int(np.argmin(cost))].astype(np.uint8)


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
    Sparse voxel map with a **columnar** backing store: occupancy lives in parallel
    numpy arrays (`_keys` int64 packed coords, kept sorted; `_log_odds`; `_hit_count`)
    rather than a dict of per-voxel objects. This is what makes ray-clearing
    affordable — a whole sweep's misses are applied with one vectorized
    searchsorted + clip, instead of a Python loop over hundreds of thousands of
    free-space voxels (the old dict design's hard floor).

    Color is sparse and only ever touched in a per-voxel Python loop anyway, so it
    stays a dict keyed by the same packed int64 id.
    """

    def __init__(self, config: Optional[VoxelMapConfig] = None):
        self.cfg = config or VoxelMapConfig()
        self._inv_size = 1.0 / self.cfg.voxel_size
        self._keys = np.zeros(0, dtype=np.int64)        # packed coords, sorted ascending
        self._log_odds = np.zeros(0, dtype=np.float64)
        self._hit_count = np.zeros(0, dtype=np.int64)
        self._color: Dict[int, ColorAccumulator] = {}   # packed id → accumulator

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

    def _row_of(self, pid: int) -> int:
        """Row index of a packed id, or -1 if absent (binary search on sorted keys)."""
        if len(self._keys) == 0:
            return -1
        i = int(np.searchsorted(self._keys, pid))
        if i < len(self._keys) and int(self._keys[i]) == pid:
            return i
        return -1

    def get(self, key: VoxelKey) -> Optional[Voxel]:
        """Snapshot view of a voxel (reads the columns), or None if untouched.
        The `color` field is the live ColorAccumulator reference, not a copy."""
        i = self._row_of(_pack_one(*key))
        if i < 0:
            return None
        return Voxel(float(self._log_odds[i]), int(self._hit_count[i]),
                     self._color.get(_pack_one(*key)))

    def __len__(self) -> int:
        return len(self._keys)

    def iter_voxels(self) -> Iterator[Tuple[VoxelKey, Voxel]]:
        """Yield (key, snapshot Voxel) for every touched voxel. For tests/inspection."""
        kx, ky, kz = _unpack_keys(self._keys)
        for i in range(len(self._keys)):
            key = (int(kx[i]), int(ky[i]), int(kz[i]))
            yield key, Voxel(float(self._log_odds[i]), int(self._hit_count[i]),
                             self._color.get(int(self._keys[i])))

    # -- occupancy (all updates funnel through the vectorized _apply) -------- #

    def _apply(self, ids: np.ndarray, deltas: np.ndarray,
               hit_inc: Optional[np.ndarray], insert_new: bool = True) -> None:
        """Add `deltas` to the log-odds of voxels `ids` (assumed UNIQUE), clamped to
        [l_min, l_max]; optionally bump hit_count by `hit_inc`. Existing voxels are
        updated in place by vectorized index; new ones are merge-inserted to keep
        `_keys` sorted. Both hits (delta>0, clamp at l_max) and misses (delta<0,
        clamp at l_min) reduce to a single symmetric clip.

        `insert_new=False` updates only voxels that already exist and drops the rest.
        Misses use this: a miss to a never-hit voxel only creates a negative voxel
        that can never be exported (we export occupied voxels only), so storing free
        space would balloon the map to the whole scanned volume for zero benefit.
        Clearing only needs to suppress voxels that carry hit evidence — surfaces and
        fliers — which by definition already exist. Late single-hit fliers with no
        subsequent clearing ray are caught instead by the n_min_hits export gate."""
        ids = np.asarray(ids, dtype=np.int64)
        if len(ids) == 0:
            return
        deltas = np.asarray(deltas, dtype=np.float64)
        hi = None if hit_inc is None else np.asarray(hit_inc, dtype=np.int64)
        lmin, lmax = self.cfg.l_min, self.cfg.l_max
        m = len(self._keys)

        if m:
            idx = np.searchsorted(self._keys, ids)
            idxc = np.minimum(idx, m - 1)
            exists = self._keys[idxc] == ids
        else:
            exists = np.zeros(len(ids), dtype=bool)
            idxc = np.zeros(len(ids), dtype=np.int64)

        if exists.any():
            er = idxc[exists]
            self._log_odds[er] = np.clip(self._log_odds[er] + deltas[exists], lmin, lmax)
            if hi is not None:
                self._hit_count[er] += hi[exists]

        new = ~exists
        if insert_new and new.any():
            nids = ids[new]
            nlo = np.clip(deltas[new], lmin, lmax)
            nhc = hi[new] if hi is not None else np.zeros(int(new.sum()), dtype=np.int64)
            order = np.argsort(nids, kind="stable")     # insert positions must be sorted
            nids, nlo, nhc = nids[order], nlo[order], nhc[order]
            pos = np.searchsorted(self._keys, nids)
            self._keys = np.insert(self._keys, pos, nids)
            self._log_odds = np.insert(self._log_odds, pos, nlo)
            self._hit_count = np.insert(self._hit_count, pos, nhc)

    def integrate_ray(self, origin, endpoint, clear: bool = True) -> Voxel:
        """
        Integrate one LiDAR return: apply L_MISS to every voxel the ray passes
        through (miss evidence) and L_HIT to the endpoint voxel (hit evidence).

        Exact Amanatides–Woo traversal — the reference path, used in tests and where
        precision matters. For whole sweeps use integrate_misses_batch (clearing) +
        integrate_hits_batch (hits), which are vectorized. Returns the endpoint
        voxel snapshot so a caller can attach color to it.
        """
        end_key = self.key_of(endpoint)
        end_id = _pack_one(*end_key)
        if clear:
            mids = [_pack_one(*k)
                    for k in _voxel_traversal(origin, endpoint, self.cfg.voxel_size)
                    if k != end_key]
            if mids:
                uids = np.unique(np.asarray(mids, dtype=np.int64))
                self._apply(uids, np.full(len(uids), self.cfg.l_miss), None,
                            insert_new=False)
        self._apply(np.array([end_id], dtype=np.int64),
                    np.array([self.cfg.l_hit]), np.array([1], dtype=np.int64))
        return self.get(end_key)

    def integrate_hit_only(self, endpoint) -> Voxel:
        """Endpoint hit with no ray clearing (cheaper; weaker noise rejection)."""
        key = self.key_of(endpoint)
        self._apply(np.array([_pack_one(*key)], dtype=np.int64),
                    np.array([self.cfg.l_hit]), np.array([1], dtype=np.int64))
        return self.get(key)

    def integrate_misses_batch(
        self,
        origin,
        endpoints: np.ndarray,
        subsample: int = 1,
        max_chunk_samples: int = 1_000_000,
    ) -> int:
        """
        Vectorized ray-clearing for a whole sweep: apply L_MISS to every *existing*
        voxel that any ray origin→endpoint passes through (excluding each ray's own
        endpoint voxel), folding all misses for a voxel into a single clamped update.
        Free (never-hit) voxels are NOT created — see _apply(insert_new=False): the
        map stays surface-sized instead of filling the whole scanned volume, which is
        what turns a multi-minute, multi-million-voxel run into a seconds-long one.

        Fast *ray-marching* approximation of the exact Amanatides–Woo traversal in
        integrate_ray(): rays are point-sampled at one-voxel steps and keys packed
        into a single int64 (a 1-D np.unique is ~15× faster than np.unique(axis=0)).
        A sample can skip a voxel a ray only clips at a corner (or double-count one
        along a diagonal); for *clearing* — where slightly weaker/stronger is
        harmless and only presence matters — that is an accepted trade. The per-voxel
        update is vectorized via the columnar store (one searchsorted + clip, ~10 ms
        for a whole sweep over a 280k-voxel map — the old dict loop was the floor and
        is gone).

        What's left is the *sampling* cost: generating ~(rays × range/voxel) points
        per sweep. `subsample` trades that against completeness — and a voxel crossed
        by many rays only needs a few to be cleared, so quality is nearly flat: on a
        room sweep, subsample 1→4→8 costs ~770→176→116 ms with negligible difference
        in what clears. So subsample is a real *speed* knob, not just memory.

        Returns the number of distinct candidate voxels the rays traversed.
        """
        o = np.asarray(origin, dtype=np.float64).reshape(3)
        P = np.asarray(endpoints, dtype=np.float64)
        if P.ndim != 2 or P.shape[1] != 3 or len(P) == 0:
            return 0
        if subsample > 1:
            P = P[::subsample]

        vs = self.cfg.voxel_size
        d = P - o
        L = np.sqrt((d * d).sum(axis=1))
        keep = L > vs                       # rays shorter than a voxel clear nothing
        if not keep.any():
            return 0
        d, L = d[keep], L[keep]
        # Sort by length so each chunk is length-homogeneous (the per-chunk sample
        # grid is sized to the longest ray; mixing lengths would waste work/memory).
        order = np.argsort(L)
        u = d[order] / L[order, None]       # unit directions (N,3)
        L = L[order]

        step = vs                           # one-voxel steps (corner skips OK for clearing)
        inv = self._inv_size
        smax_all = max(1, int(np.floor((L[-1] - vs) / step)))
        rows_per_chunk = max(1, max_chunk_samples // smax_all)

        ids_parts = []
        for c0 in range(0, len(L), rows_per_chunk):
            c1 = min(len(L), c0 + rows_per_chunk)
            uc, Lc = u[c0:c1], L[c0:c1]
            smax = int(np.floor((Lc[-1] - vs) / step))  # Lc sorted asc → last is longest
            if smax < 1:
                continue
            ts = np.arange(0, smax + 1, dtype=np.float64) * step          # incl. origin voxel
            samples = o + uc[:, None, :] * ts[None, :, None]             # (C,S,3)
            keys = np.floor(samples * inv).astype(np.int64)             # (C,S,3)
            valid = ts[None, :] <= (Lc[:, None] - vs)                   # stop 1 voxel short
            ids_parts.append(_pack_keys(keys[valid]))                  # (M,) int64
        if not ids_parts:
            return 0

        uniq, counts = np.unique(np.concatenate(ids_parts), return_counts=True)
        self._apply(uniq, self.cfg.l_miss * counts.astype(np.float64), None,
                    insert_new=False)
        return len(uniq)

    def integrate_hits_batch(self, points: np.ndarray) -> None:
        """
        Vectorized endpoint integration for a whole sweep (no ray clearing).

        Floors all points to packed keys, folds l_hit·count per voxel, and applies
        with a single vectorized update. This is the default whole-session occupancy
        path; ray clearing (integrate_misses_batch) is the opt-in stronger denoise.
        """
        if len(points) == 0:
            return
        keys = np.floor(np.asarray(points, dtype=np.float64) * self._inv_size).astype(np.int64)
        ids = _pack_keys(keys)
        uniq, counts = np.unique(ids, return_counts=True)
        self._apply(uniq, self.cfg.l_hit * counts.astype(np.float64),
                    counts.astype(np.int64))

    # -- color --------------------------------------------------------------- #

    def add_color(self, key: VoxelKey, rgb, weight: float) -> None:
        pid = _pack_one(*key)
        if self._row_of(pid) < 0:
            return  # only color voxels that exist (i.e. have geometry evidence)
        acc = self._color.get(pid)
        if acc is None:
            acc = ColorAccumulator(self.cfg.color_reservoir)
            self._color[pid] = acc
        acc.add(rgb, weight)

    # -- iteration / export -------------------------------------------------- #

    def _occupied_mask(self) -> np.ndarray:
        """Boolean mask over the columns: occupied AND hit at least n_min_hits times."""
        return (self._log_odds >= self.cfg.l_occ_min) & (self._hit_count >= self.cfg.n_min_hits)

    def occupied_keys(self) -> Iterator[VoxelKey]:
        kx, ky, kz = _unpack_keys(self._keys[self._occupied_mask()])
        for a, b, c in zip(kx, ky, kz):
            yield (int(a), int(b), int(c))

    def compute_normals(self, radius: int = 2, min_neighbors: int = 6):
        """Per-voxel PCA surface normals over the occupied set (plane_detection
        handoff Stage 1). For each occupied voxel, gather occupied neighbours within
        `radius` voxels and take the smallest-eigenvalue eigenvector of their center
        covariance — the local plane normal. Computed on the *denoised* occupancy, so
        far cleaner than per-frame raw normals.

        Sign is left ambiguous (PCA gives an axis): downstream uses |cos|, which is
        sign-free, so no orientation pass is needed. Voxels with fewer than
        `min_neighbors` supporting neighbours are marked invalid (normal left zero).

        Returns (normals (M,3) float64, valid (M,) bool), aligned to the iteration
        order of occupied_keys() / export_points() so callers can index by position.
        Fully vectorized: one searchsorted membership test per neighbour offset, then
        a single batched eigh over all voxels — no per-voxel Python loop.
        """
        ids = self._keys[self._occupied_mask()]
        n = len(ids)
        normals = np.zeros((n, 3), dtype=np.float64)
        valid = np.zeros(n, dtype=bool)
        if n == 0:
            return normals, valid

        kx, ky, kz = _unpack_keys(ids)
        base = np.column_stack([kx, ky, kz]).astype(np.int64)   # (n,3) int coords
        s = self.cfg.voxel_size
        centers = (base + 0.5) * s                               # (n,3) world centers

        cnt = np.zeros(n, dtype=np.int64)
        ssum = np.zeros((n, 3), dtype=np.float64)                # Σ neighbour center
        souter = np.zeros((n, 3, 3), dtype=np.float64)           # Σ pᵀp (for covariance)
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
                    nid = _pack_keys(base + (dx, dy, dz))
                    pos = np.minimum(np.searchsorted(ids, nid), n - 1)
                    ok = ids[pos] == nid                         # neighbour occupied?
                    if not ok.any():
                        continue
                    src = np.where(ok)[0]
                    nb = centers[pos[ok]]                        # neighbour centers
                    cnt[src] += 1
                    ssum[src] += nb
                    souter[src] += nb[:, :, None] * nb[:, None, :]

        valid = cnt >= min_neighbors
        if valid.any():
            c = cnt[valid].astype(np.float64)
            mean = ssum[valid] / c[:, None]
            cov = souter[valid] / c[:, None, None] - mean[:, :, None] * mean[:, None, :]
            _, vecs = np.linalg.eigh(cov)                        # ascending eigenvalues
            normals[valid] = vecs[:, :, 0]                       # smallest-λ eigenvector
        return normals, valid

    def export_points(self, with_color: bool = False):
        """
        Return (centers Nx3 float32, colors Nx3 uint8 | None) for occupied voxels.
        Occupancy = log-odds ≥ l_occ_min AND hit_count ≥ n_min_hits (the cheap
        flier gate). Colors are the per-voxel weighted median; voxels below
        n_min_color samples get a flag color (magenta) so low-confidence color is
        visible, not silent.
        """
        mask = self._occupied_mask()
        ids = self._keys[mask]
        kx, ky, kz = _unpack_keys(ids)
        s = self.cfg.voxel_size
        centers = np.empty((len(ids), 3), dtype=np.float32)
        centers[:, 0] = (kx + 0.5) * s
        centers[:, 1] = (ky + 0.5) * s
        centers[:, 2] = (kz + 0.5) * s
        if not with_color:
            return centers, None
        flag = np.array([255, 0, 255], dtype=np.uint8)
        vec_med = self.cfg.color_vector_median
        colors = np.empty((len(ids), 3), dtype=np.uint8)
        for i in range(len(ids)):
            acc = self._color.get(int(ids[i]))
            if acc is not None and acc.sample_count >= self.cfg.n_min_color:
                colors[i] = acc.result(vec_med)
            else:
                colors[i] = flag
        return centers, colors


# --------------------------------------------------------------------------- #
#  Voxel-key packing (3 int21 → 1 int64) for fast 1-D set/count operations
# --------------------------------------------------------------------------- #

_KEY_BITS = 21                       # per axis → range ±2^20 voxels (±20 km @ 2cm)
_KEY_OFF = 1 << (_KEY_BITS - 1)      # bias to make every coord non-negative
_KEY_MASK = (1 << _KEY_BITS) - 1


def _pack_keys(keys: np.ndarray) -> np.ndarray:
    """(N,3) int voxel keys → (N,) int64, biased so negatives pack cleanly.
    Lets us use a 1-D np.unique (sort of int64) instead of np.unique(axis=0),
    which is the difference between ~0.06 s and ~1 s for a sweep's free voxels."""
    k = keys.astype(np.int64)
    return ((k[:, 0] + _KEY_OFF)
            | ((k[:, 1] + _KEY_OFF) << _KEY_BITS)
            | ((k[:, 2] + _KEY_OFF) << (2 * _KEY_BITS)))


def _pack_one(kx: int, ky: int, kz: int) -> int:
    """Scalar form of _pack_keys for single-voxel lookups."""
    return int((kx + _KEY_OFF)
               | ((ky + _KEY_OFF) << _KEY_BITS)
               | ((kz + _KEY_OFF) << (2 * _KEY_BITS)))


def _unpack_keys(ids: np.ndarray):
    """Inverse of _pack_keys → (kx, ky, kz) int64 arrays."""
    kx = (ids & _KEY_MASK) - _KEY_OFF
    ky = ((ids >> _KEY_BITS) & _KEY_MASK) - _KEY_OFF
    kz = ((ids >> (2 * _KEY_BITS)) & _KEY_MASK) - _KEY_OFF
    return kx, ky, kz


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
