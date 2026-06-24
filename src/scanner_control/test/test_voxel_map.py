#!/usr/bin/env python3
"""
Unit tests for the voxel-map geometry+color core. Pure numpy — no ROS, no
hardware — so the occupancy logic can be validated offline. Run with:

    python3 -m pytest src/scanner_control/test/test_voxel_map.py -v
    # or, with no pytest installed:
    python3 src/scanner_control/test/test_voxel_map.py
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scanner_control.voxel_map import (  # noqa: E402
    ColorAccumulator,
    VoxelMap,
    VoxelMapConfig,
    _voxel_traversal,
    _weighted_median,
    motion_weight,
    range_weight,
    view_angle_weight,
)


def test_key_and_center_roundtrip():
    m = VoxelMap(VoxelMapConfig(voxel_size=0.02))
    p = np.array([0.031, -0.005, 0.119])
    k = m.key_of(p)
    assert k == (1, -1, 5)
    c = m.center_of(k)
    # center must be within half a voxel of the original point on every axis
    assert np.all(np.abs(c - p) <= 0.02)


def test_traversal_is_exact_and_ordered():
    # A straight ray along +x crossing several voxels hits each integer cell once.
    keys = list(_voxel_traversal([0.001, 0.001, 0.001], [0.099, 0.001, 0.001], 0.02))
    xs = [k[0] for k in keys]
    assert xs == [0, 1, 2, 3, 4]                 # no skips, in order
    assert all(k[1] == 0 and k[2] == 0 for k in keys)
    assert keys[0] == (0, 0, 0) and keys[-1] == (4, 0, 0)


def test_traversal_diagonal_terminates_at_endpoint():
    keys = list(_voxel_traversal([0.001] * 3, [0.099] * 3, 0.02))
    assert keys[0] == (0, 0, 0)
    assert keys[-1] == (4, 4, 4)
    # consecutive keys differ by exactly one step on exactly one axis
    for a, b in zip(keys, keys[1:]):
        diff = sum(abs(x - y) for x, y in zip(a, b))
        assert diff == 1


def test_ray_clearing_rejects_one_off_noise():
    """Handoff's central claim: a spurious return that later rays pass through is
    driven back below threshold, while a repeatedly-hit real surface persists."""
    cfg = VoxelMapConfig(voxel_size=0.02)
    m = VoxelMap(cfg)
    origin = np.array([0.0, 0.0, 0.0])

    # One spurious return floating in mid-air at x=0.10 (a reflection / flier).
    noise_pt = np.array([0.10, 0.0, 0.0])
    m.integrate_ray(origin, noise_pt)
    noise_key = m.key_of(noise_pt)
    assert m.get(noise_key).log_odds > 0  # initially looks occupied

    # 30 later returns to a real wall at x=0.40 all pass THROUGH the noise voxel.
    wall_pt = np.array([0.40, 0.0, 0.0])
    for _ in range(30):
        m.integrate_ray(origin, wall_pt)

    noise_lo = m.get(noise_key).log_odds
    wall_lo = m.get(m.key_of(wall_pt)).log_odds
    assert noise_lo < cfg.l_occ_min, "one-off noise should be cleared below threshold"
    assert wall_lo >= cfg.l_occ_min, "repeatedly-hit real surface should persist"
    # And the export reflects it: wall in, noise out.
    centers, _ = m.export_points()
    occ = {m.key_of(c) for c in centers}
    assert m.key_of(wall_pt) in occ
    assert noise_key not in occ


def test_log_odds_clamped():
    cfg = VoxelMapConfig(voxel_size=0.02)
    m = VoxelMap(cfg)
    pt = np.array([0.5, 0.5, 0.5])
    for _ in range(1000):
        m.integrate_hit_only(pt)
    assert m.get(m.key_of(pt)).log_odds <= cfg.l_max + 1e-9


def test_weighted_median_rejects_flier():
    # 10 clean ~green samples, weight 1; one bright-red flier with weight 1.
    vals = np.array([120] * 10 + [255], dtype=np.float32)
    w = np.ones(11, dtype=np.float32)
    assert _weighted_median(vals, w) == 120  # flier does not move the median

    # Even with a heavy flier, median resists until weight dominates.
    w2 = np.array([1.0] * 10 + [5.0], dtype=np.float32)
    assert _weighted_median(vals, w2) == 120


def test_color_accumulator_median_vs_mean():
    acc = ColorAccumulator(capacity=64)
    for _ in range(20):
        acc.add(np.array([10, 200, 10], dtype=np.float32), 1.0)
    acc.add(np.array([255, 0, 0], dtype=np.float32), 1.0)  # single bad frame
    r = acc.result()
    # median stays on the clean green; a running mean would be dragged toward red.
    assert r[1] == 200 and r[0] == 10 and r[2] == 10
    assert acc.sample_count == 21


def test_color_reservoir_is_bounded():
    acc = ColorAccumulator(capacity=8)
    for i in range(100):
        acc.add(np.array([i % 256, 0, 0], dtype=np.float32), 1.0)
    assert acc.sample_count == 8  # bounded memory regardless of input length


def test_add_color_only_on_existing_voxels():
    m = VoxelMap(VoxelMapConfig(voxel_size=0.02))
    pt = np.array([0.2, 0.2, 0.2])
    # No geometry yet → color is dropped, nothing allocated.
    m.add_color(m.key_of(pt), np.array([10, 20, 30]), 1.0)
    assert len(m) == 0
    # After a hit, color attaches.
    m.integrate_hit_only(pt)
    m.add_color(m.key_of(pt), np.array([10, 20, 30]), 1.0)
    assert m.get(m.key_of(pt)).color.sample_count == 1


def test_weights_are_sane():
    # face-on (cos 1) > grazing; beyond cutoff → 0
    assert view_angle_weight([0, 0, 1], [0, 0, 1], 0.34) == 1.0
    assert view_angle_weight([1, 0, 0], [0, 0, 1], 0.34) == 0.0  # 90° → below cutoff
    # range weight decreases with distance
    assert range_weight(0.0, 1.0) == 1.0
    assert range_weight(5.0, 1.0) < range_weight(1.0, 1.0)
    # motion weight: still frame ~1, fast pan down-weighted
    assert abs(motion_weight(0.0, 1.0) - 1.0) < 1e-9
    assert motion_weight(2.0, 1.0) < 0.2


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL  {fn.__name__}: {e}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"  ERROR {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    return failed


if __name__ == "__main__":
    sys.exit(1 if _run_all() else 0)
