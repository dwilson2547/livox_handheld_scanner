# HANDOFF — implementation brief for GitHub Copilot

This repo is a **scaffold**. The ROS 2 plumbing, configs, launch wiring, and node
skeletons are in place; the actual sensor integration and the meshing math are
stubbed. This document is the implementation plan. Work the sections roughly in
order — each builds on the previous.

Target platform: **Ubuntu 22.04 + ROS 2 Humble**, NUC-class x86, CPU-only.

Search the codebase for `TODO(copilot)` — every marker corresponds to a task here.

---

## §1. Context you need before touching anything

**Hardware:** Livox Horizon LiDAR (primary), Intel RealSense D435i (color only),
NUC compute. **The LIO IMU is the Horizon's built-in BMI088** — do not add the
WT901C or the D435i's BMI055 to the odometry pipeline. They were evaluated and
deliberately rejected (sync/quality). The camera is for optional colorization
only.

**Pipeline:** driver → Point-LIO → (VDBFusion meshing ∥ coverage/health) →
Foxglove, with a parallel rosbag of raw topics for offline refinement.

**The non-negotiable product requirement:** live coverage feedback. The operator
must see a coverage heatmap and a tracking-health indicator while scanning.

---

## §2. Dependency vendoring (`scripts/setup_workspace.sh`)

Mostly done. Validate that:
- `Livox-SDK` builds and installs (the SDK1 C library).
- `livox_ros2_driver` builds and the launch/executable names referenced in
  `scanner.launch.py` actually exist (`livox_ros2_driver_node`, the config-driven
  launch). Fix names if the upstream repo differs.
- `point_lio` builds **after** the §4 patch.
- `foxglove_bridge`, `sensor_msgs_py`, `vdbfusion` install.

Deliverable: `./scripts/setup_workspace.sh` on a clean 22.04 box ends in a green
`colcon build`.

---

## §3. Bring up the Horizon driver (SDK1)

- Put the real broadcast code in `config/horizon.json`.
- Confirm the driver publishes **CustomMsg** (`xfer_format: 1`) so per-point
  timestamps survive for deskewing. PointCloud2 output loses them — do not use it
  for LIO input.
- Confirm `/livox/imu` actually streams (the BMI088 is a separate topic; it's
  commonly missed). `ros2 topic hz /livox/imu` should show ~200 Hz.

Acceptance: `ros2 topic hz` on `/livox/lidar` (~10 Hz) and `/livox/imu` (~200 Hz)
both healthy.

---

## §4. ⭐ Point-LIO + SDK1 CustomMsg — the critical integration risk

This is the single most likely thing to eat time. **Read carefully.**

The Horizon (SDK1) driver publishes `livox_interfaces/msg/CustomMsg`. Stock
Point-LIO ROS2 forks (e.g. `dfloreaa/point_lio_ros2`) are written against
`livox_ros_driver2/msg/CustomMsg` (the HAP/Mid-360 SDK2 message). The two
messages are structurally near-identical but are **different ROS types**, so
Point-LIO will not subscribe to the driver's output as-is.

Two viable fixes — **prefer Option A:**

**Option A — patch Point-LIO's preprocess to accept the SDK1 message.**
- In Point-LIO's `preprocess.{h,cpp}` (the `livox_pcl_cbk` / CustomMsg path),
  change the include and the callback signature from the driver2 CustomMsg to
  `livox_interfaces/msg/CustomMsg`. The field layout (`points[].x/y/z/reflectivity/offset_time`)
  matches, so the body of the deskew loop should be unchanged.
- Update the package.xml/CMakeLists dependency from `livox_ros_driver2` to
  `livox_interfaces` (or whatever the SDK1 driver names its interfaces package —
  verify; some forks call it `livox_ros2_driver_interfaces`).
- Rebuild.

**Option B — relay node.** Write a tiny node that subscribes to the SDK1
CustomMsg and republishes the driver2 CustomMsg. Avoids touching Point-LIO but
adds a copy per frame and a second message dependency. Only do this if Option A
proves messy.

Then, in `config/point_lio_horizon.yaml`:
- Set `lidar_type: 1` (Livox), `scan_line: 6` (Horizon), `blind`, and timestamp
  unit to match the SDK1 driver's offset_time units (**verify** — guessing here
  causes deskew smear).
- ⚠️ **Fill in the real LiDAR↔IMU extrinsic** (`extrinsic_T` / `extrinsic_R`)
  from the Horizon manual. The placeholders are identity/zero and WILL produce
  smeared maps.
- Fill in real BMI088 saturation (`satu_acc`, `satu_gyro`) and noise covariances.

Acceptance: walk the rig around a room; `/cloud_registered` accumulates a crisp,
non-smeared map; odom on `/aft_mapped_to_init` is continuous. **Verify the actual
output topic names** of your fork and update `meshing.yaml` / `coverage.yaml` if
they differ from `/cloud_registered` and `/aft_mapped_to_init`.

---

## §5. Coverage / health node (`scanner_coverage/coverage_node.py`)

Coverage heatmap path is **implemented** (hash-voxel hit counting →
PointCloud2 with `intensity` = normalized observation count). Validate it against
the real registered cloud and tune `voxel_size` / `observations_for_full`.

Health path needs work:
- Current implementation is a **fallback**: it derives a `[0,1]` health score from
  the trace of the odometry pose covariance. This is crude.
- **Better:** Point-LIO's degeneracy is best read from the condition number of the
  iEKF update's information matrix (the classic LIO degeneracy signal — e.g. when
  scanning a featureless corridor the geometry under-constrains certain DOF).
  Either (a) lightly patch the Point-LIO fork to publish a `std_msgs/Float32`
  health/condition score, or (b) subscribe to whatever covariance/diagnostics it
  already emits and compute a smoothed score here.
- Map the score to the thresholds already in `coverage.yaml`
  (`health_warn_threshold`, `health_bad_threshold`).

Performance note: `read_points` over a dense cloud every frame at 10 Hz is heavy
in Python. If it can't keep up on the NUC, decimate (every Nth point or every Nth
cloud), or port the hot loop to numpy-vectorized reads / C++.

Acceptance: heatmap visibly fills in as areas are scanned; health indicator drops
when you point the rig at a blank wall / featureless space and recovers in
feature-rich areas.

---

## §6. VDBFusion meshing node (`scanner_meshing/vdb_meshing_node.py`)

Currently a **no-op stub** with full ROS plumbing. Implement:
- Instantiate `VDBVolume(voxel_size, sdf_trunc, space_carving)`.
- On each registered cloud: convert to an `(N,3)` float64 numpy array, get the
  sensor origin from the latest odom (already cached as `self._last_origin`), and
  call `self.vdb.integrate(points, origin)`. Origin matters — space carving needs
  the true sensor position per integration.
- Every `mesh_every_n_clouds`, call `extract_triangle_mesh(min_weight=...)` and
  publish a `visualization_msgs/Marker` TRIANGLE_LIST on `/scanner/mesh`.
  (For large meshes, consider downsampling the published marker and keeping the
  full-res volume only for the on-shutdown PLY save.)
- On shutdown, write the PLY to `save_path`.

If `pip install vdbfusion` fails on 22.04, build from source (OpenVDB + pybind);
note the steps in `docs/SETUP.md` once solved.

Acceptance: a live surface mesh appears in Foxglove and refines as coverage
improves; a `.ply` is written on shutdown.

---

## §7. Offline refinement (separate, later — EPYC/Altra)

Not part of the live NUC pipeline. Out of scope for the first pass, but keep the
bag schema (`/livox/lidar /livox/imu /tf /tf_static`) intact so it's possible to:
- replay the bag,
- run Point-LIO (or a pose-graph backend) with loop closure,
- re-integrate VDBFusion at a finer `voxel_size` for the print mesh.

Don't optimize the live path in ways that break offline replay (e.g. don't drop
raw topics from the recorder).

---

## §8. Things NOT to do (guardrails)

- Do **not** add the WT901C or D435i IMU to the LIO pipeline.
- Do **not** switch the driver to `livox_ros_driver2` (HAP/Mid-360 only — it does
  not support the Horizon).
- Do **not** feed PointCloud2 (xfer_format 2) into Point-LIO; you lose per-point
  timestamps and deskewing breaks.
- Do **not** trust the placeholder extrinsics/saturation values in
  `point_lio_horizon.yaml` — they are marked PLACEHOLDER and must be replaced with
  manual values before the maps mean anything dimensionally.

---

## Task checklist

- [ ] §2 clean-box `setup_workspace.sh` → green colcon build
- [ ] §3 Horizon driver: CustomMsg + IMU topic confirmed
- [ ] §4 Point-LIO SDK1 CustomMsg patch (Option A) builds + subscribes
- [ ] §4 real LiDAR↔IMU extrinsic + BMI088 params filled in
- [ ] §4 verify real output topic names, update meshing/coverage configs
- [ ] §5 validate coverage heatmap on real cloud; tune voxels
- [ ] §5 real LIO health signal (replace covariance-trace fallback)
- [ ] §6 VDBFusion integrate + extract + publish + PLY save
- [ ] §6 Foxglove layout shows mesh + coverage + health together
- [ ] end-to-end room scan produces crisp map + live feedback + saved bag
