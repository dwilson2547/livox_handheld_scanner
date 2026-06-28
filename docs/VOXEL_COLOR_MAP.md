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
| `src/scanner_control/scanner_control/voxel_map.py` | ROS-free core: log-odds occupancy, Amanatides–Woo ray-clearing, best-N weighted-median `ColorAccumulator` (lowest-weight eviction), view-angle/range/motion weights, PLY export |
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
- `--min-hits N` (default 1) — export gate: drop voxels hit < N times. `--min-hits 2`
  is the cheapest denoise; it rejects one-off fliers without any ray-clearing.
- `--ray-clear` — enable miss-based clearing (stronger denoise). Vectorized columnar
  store + surface-only updates make it minutes/session, not ~40 min (see limitation 1).
  `clear_subsample` (default 4) trades sampling cost against completeness.
- `--vector-median` — robust color via the weighted vector medoid instead of
  per-channel medians (avoids invented hues on mixed-color voxels).

### Viewing the colored voxel map
The map exports a colored PLY (default `<session>/voxel_color_map.ply`). To browse it
in Potree: `bash scripts/potree.sh voxel <session>` — converts PLY → colored LAS
(`voxel_ply_to_las.py`, LAS point-format 2 / 16-bit RGB) → Potree octree and serves at
`localhost:8087`. (PotreeConverter's PLY reader is unreliable; the LAS hop is why.) The
raw point cloud still uses `potree.sh start <session>`.

## Part 2 refinements applied (2026-06-27)

Implementing [`../voxel_color_map_pt_2.md`](../voxel_color_map_pt_2.md):

- **§2 lowest-weight eviction (the flagship cheap upgrade).** `ColorAccumulator`
  now keeps the *best-N* samples: on overflow it drops the lowest-weight retained
  sample (and ignores an incoming sample that is worse than everything kept),
  instead of evicting the oldest. A clean face-on early frame can no longer be
  pushed out by a later grazing-angle / fast-pan one. Unit-tested
  (`test_color_accumulator_keeps_highest_weight`).
- **§1 ω from the trajectory, centered on the image stamp.** `_Trajectory.omega_at`
  was already trajectory-derived (not raw IMU — correct for this rig); it now uses
  a centered finite-difference `ω ≈ angle(R(t-h)ᵀR(t+h))/2h` around the image
  timestamp rather than whichever two native poses happened to bracket it, making
  the rolling-shutter weight robust to uneven native pose spacing.

**§3 vector median — implemented (2026-06-28).** `--vector-median` switches the
per-voxel color from independent per-channel medians (which can output a hue no
sample had on a mixed-color voxel) to the weighted **vector medoid** — the retained
sample minimizing Σ wⱼ·‖rgbᵢ−rgbⱼ‖, guaranteed to be an actually-observed color.
Off by default; flip it on if you see odd tints on high-contrast-edge voxels.
**Open decisions** (not algorithm changes): §4 VDBFusion (this map is standalone,
emits its own PLY — it does *not* run a second grid alongside VDBFusion, so the
"two grids" trap is already avoided); §6 process-then-delete bag ops.

### Capture checklist (pt_2 §5 — systematic color drift the median CANNOT fix)
Lock the D435 RGB **exposure and white balance** before recording. The robust
accumulator rejects *outliers*; it faithfully preserves a *consistent* wrong color
if auto-exposure/WB drifts across the scan. The lock is **now scaffolded** in
`src/scanner_bringup/config/realsense.yaml` (`rgb_camera.enable_auto_exposure: false`
+ `rgb_camera.exposure`, `rgb_camera.enable_auto_white_balance: false` +
`rgb_camera.white_balance`). **Action remaining on next hardware session:** verify
the param names against the installed `realsense2_camera` (`ros2 param list
/camera/d435i`) and tune the two placeholder values to the room. At minimum lock WB.

## Status (2026-06-28)

End-to-end validated on `viewer_fix_20260621_075144`: ~60 s replay + build,
570 deskewed sweeps → 157 k voxels, colored from 271 keyframes. Result is a
**color-coherent surface** — a clear improvement over the last-write-wins mesh —
with residual blotches confined to grazing-angle boundary voxels. **21 unit tests
pass** (ray-clearing, robust median + vector medoid, PCA normals, columnar-store
sorted-key invariant, min-hit gate, DDA exact, clamps).

**Performance + denoise are done (2026-06-28).** `VoxelMap` is now a columnar store
(int64-packed sorted keys + numpy log-odds/hit-count), so `--ray-clear` runs in
minutes/session instead of ~40 min (the old per-voxel Python dict loop is gone), and
two denoise knobs exist: `--min-hits N` (cheap flier gate) and `--ray-clear` (miss
clearing, surface-only). Re-validated on real data: baseline reproduces the prior
output **byte-for-byte**; `--min-hits 2` removes 57 % of voxels (single-hit fliers),
`--ray-clear` removes 31 % (cleared ghosts); builds run 58–166 s on the 2.9 GB bag.

**Color quality (2026-06-28):** view-angle weight now uses real **PCA surface
normals** (`compute_normals`) instead of the crude centroid direction; `--vector-median`
adds the weighted medoid for mixed-color voxels. Colored voxel maps are browsable via
`bash scripts/potree.sh voxel <session>`.

**Calibration (unchanged, hardware-gated):** the static-frame gate shows the extrinsic
is approximately correct (scene-coherent, straight lines) with a small residual offset;
the blotch was dominated by the temporal/outlier path, confirming the voxel map is the
right fix. A proper target-based / `direct_visual_lidar_calibration` run is still pending
(needs the rig connected). The session bags' `camera_info` carries no distortion coeffs.

### Open / next (all hardware-gated — waiting on the rig)
- **§5 RealSense exposure/WB lock** — scaffolded in `scanner_bringup/config/realsense.yaml`;
  verify param names (`ros2 param list /camera/d435i`) and tune the placeholder values
  to the room on the next capture. At minimum lock WB.
- **Target-based extrinsic calibration** — `direct_visual_lidar_calibration` run to remove
  the residual offset.
- **Plane detection Stages 2+** — gravity-based ground/wall/ceiling classification and
  plane extraction on top of the now-available PCA normals (`../plane_detection_handoff.md`).

## Known limitations / next steps

1. **Noise rejection is opt-in but now cheap (reworked 2026-06-28).** By default
   `build_voxel_map.py` uses vectorized *endpoint* hits, so a single hit lands a voxel
   exactly at `L_OCC_MIN=0.85` and the threshold filters nothing. Two complementary
   denoise paths now exist:
   - **`--min-hits N`** (cheap gate): export only voxels hit ≥ N times. A one-off
     flier has `hit_count==1`, so `--min-hits 2` drops it for ~free — no ray-clearing
     needed. This is the recommended first knob.
   - **`--ray-clear`** (stronger): miss-based clearing, fully reworked. `VoxelMap` now
     uses a **columnar store** (int64-packed sorted `_keys` + numpy `_log_odds`/
     `_hit_count`), so the per-voxel update is one vectorized `searchsorted` + `clip`
     (~10 ms/sweep over a 280k-voxel map — the old per-voxel **dict loop**, the true
     ~40 min/session bottleneck, is gone). Clearing is also **surface-only**
     (`_apply(insert_new=False)`): misses update existing voxels but never create
     free-space ones, since those can't be exported anyway — this keeps the map
     surface-sized instead of filling the whole scanned volume (~13 M voxels at 2 cm
     for a room). The geometry is an int64-packed numpy ray-march approximating the
     exact Amanatides–Woo `integrate_ray` (kept for tests). Remaining cost is purely
     ray *sampling*, tuned by `clear_subsample` (default 4 ≈ ~1.7 min/session; 1 =
     every ray ≈ ~7 min; quality is nearly flat in it). Net vs the old path: a usable
     full-session ray-clear in minutes instead of tens of minutes.
2. **Surface normals — now PCA (2026-06-28).** The view-angle weight previously used
   a crude voxel→trajectory-centroid direction; it now uses real per-voxel **PCA
   normals** (`VoxelMap.compute_normals`, plane_detection handoff Stage 1): the
   smallest-eigenvalue eigenvector of each occupied voxel's occupied-neighbour
   covariance, computed on the *denoised* occupancy and fully vectorized (one batched
   `eigh`, no per-voxel loop). Sign-free (downstream uses |cos|). Voxels with too few
   neighbours fall back to the old centroid direction. Stages 2+ of the handoff
   (gravity-based ground/wall/ceiling classification, plane extraction) are still TODO.
3. **Dim, low-contrast captures** with no distortion model — worth a fixed-exposure
   recapture and a real extrinsic calibration on the next hardware session.
4. Color quality is gated on the calibration refinement above.

## Replay gotcha

`ros2 launch scanner.launch.py use_bag:=true` both **plays** the bag and keeps
Point-LIO/meshing nodes alive **indefinitely** after playback — it never self-exits.
`replay_to_cloud_bag.sh` handles this by watching the launch log for the player's
"process has finished cleanly" line, then tearing down the launch group.
