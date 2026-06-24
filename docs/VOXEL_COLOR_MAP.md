# Voxel Color Map (experimental)

Probabilistic voxel map that replaces the blotchy last-write-wins colorizer with
log-odds occupancy + robust per-voxel color accumulation. Grounded in the design
in [`../voxel_color_map_handoff.md`](../voxel_color_map_handoff.md); calibration
prerequisite in [`../cam_lidar_calib_handoff.md`](../cam_lidar_calib_handoff.md).

## Why

The old `colorize.py` projects camera color onto mesh vertices with
`score = 1/depth`, last-write-wins, no outlier rejection, no occlusion test, and
snaps each image to the *nearest* odometry pose rather than its own timestamp. A
single rolling-shutter / reflection frame permanently wins a vertex → muddy color.

The voxel map fixes this with three independent levers:
- **Per-image-timestamp pose** — interpolate the trajectory to each RGB frame's
  own stamp (the temporal fix for motion smear).
- **Robust weighted median** per voxel — a bad frame cannot drag the result.
- **Occlusion + per-sample weights** — reject hidden voxels (depth test) and
  down-weight grazing-angle / distant / fast-motion (rolling-shutter) samples.

## Components

| File | Role |
|---|---|
| `src/scanner_control/scanner_control/voxel_map.py` | ROS-free core: log-odds occupancy, Amanatides–Woo ray-clearing, weighted-median `ColorAccumulator`, view-angle/range/motion weights, PLY export |
| `src/scanner_control/test/test_voxel_map.py` | Unit tests for the core (no ROS/hardware needed) |
| `src/scanner_control/scanner_control/voxel_build.py` | Bag adapter + color front-end (occupancy from `/cloud_registered`, per-keyframe projection, occlusion, weighting) |
| `src/scanner_control/scanner_control/static_projection.py` | Calibration **gate**: project one LiDAR sweep through `T_cam_lidar` onto one RGB frame |
| `scripts/static_projection_test.py` | Runner for the calibration gate |
| `scripts/replay_to_cloud_bag.sh` | Replay a raw session → record deskewed `/cloud_registered` + `/aft_mapped_to_init` |
| `scripts/build_voxel_map.py` | Build + export the colored voxel map |

The math lives in `voxel_map.py` (ROS-free, unit-testable); the ROS/bag I/O lives
in `voxel_build.py` and `static_projection.py`.

## Workflow

```bash
source ~/ros2_ws/install/setup.bash

# 0. (recommended) confirm the extrinsic before trusting any color
python3 scripts/static_projection_test.py sessions/<name> -t 0.5 -s 4 -a 0.85
#    → <name>/static_test_overlay.png  (LiDAR depth points over the RGB frame)
#      Depth discontinuities should land on the matching photo edges. Shifted-but-
#      straight → extrinsic; warped → intrinsics/distortion; crisp → good.

# 1. regenerate the deskewed cloud (build needs /cloud_registered, NOT raw sweeps)
scripts/replay_to_cloud_bag.sh sessions/<name>      # → sessions/<name>/cloud_bag/

# 2. build the colored voxel map
python3 scripts/build_voxel_map.py sessions/<name> --voxel-size 0.02 -i 0.2
#    → sessions/<name>/voxel_color_map.ply
```

### Key parameters (`VoxelMapConfig` / CLI)
- `--voxel-size` (default **0.02 m**) — start at 2 cm; handheld LIO pose error is
  ~1–2 cm, so smaller voxels scatter repeated hits across neighbours.
- `--l-occ-min` (default 0.85) — occupancy export threshold / **noise-floor knob**.
- `--interval` — seconds between sampled RGB keyframes.
- `--ray-clear` — enable per-ray miss integration (stronger denoise; slow in Python).

## Status (2026-06-24)

End-to-end validated on `viewer_fix_20260621_075144`: ~60 s replay + 92 s build,
570 deskewed sweeps → 157 k voxels, colored from 271 keyframes. Result is a
**color-coherent surface** — a clear improvement over the last-write-wins mesh —
with residual blotches confined to grazing-angle boundary voxels. Core unit tests
pass (ray-clearing rejects one-off noise, median resists fliers, DDA exact, clamps).

**Calibration:** the static-frame gate shows the extrinsic is approximately correct
(scene-coherent, straight lines) with a small residual offset; the blotch was
dominated by the temporal/outlier path, confirming the voxel map is the right fix.
A proper target-based / `direct_visual_lidar_calibration` run is still pending (needs
the rig connected). The session bags' `camera_info` carries no distortion coeffs.

## Known limitations / next steps

1. **Noise rejection not active by default.** `build_voxel_map.py` uses vectorized
   *endpoint* hits for speed, so a single hit lands a voxel exactly at
   `L_OCC_MIN=0.85` and the threshold filters nothing. The ray-clearing path
   (`--ray-clear`) works and is unit-tested but is ~40 min/session in pure Python.
   **Next:** vectorize ray-clearing or add a cheap min-hit-count gate.
2. **Surface normals are crude** (voxel→trajectory-centroid direction) for the
   view-angle weight. Proper PCA normals arrive with plane detection
   (`../plane_detection_handoff.md`).
3. **Dim, low-contrast captures** with no distortion model — worth a fixed-exposure
   recapture and a real extrinsic calibration on the next hardware session.
4. Color quality is gated on the calibration refinement above.

## Replay gotcha

`ros2 launch scanner.launch.py use_bag:=true` both **plays** the bag and keeps
Point-LIO/meshing nodes alive **indefinitely** after playback — it never self-exits.
`replay_to_cloud_bag.sh` handles this by watching the launch log for the player's
"process has finished cleanly" line, then tearing down the launch group.
