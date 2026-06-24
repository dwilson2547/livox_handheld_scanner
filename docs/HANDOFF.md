# HANDOFF — system state and known decisions

This documents what is built, what the key decisions were, and what remains
open. It replaces the original Copilot implementation brief (the scaffold is
fully implemented).

---

## What's working

- **Full end-to-end pipeline:** capture → process → colorize → Potree viewer
- **Control panel** at `:8090` — start/stop scan, trigger processing, colorize,
  launch Potree per session
- **Autostart on boot** — `scanner-control.service` systemd unit runs the control
  panel as user `daniel` on every boot; available ~5 s after power-on, no login needed.
  Service file at `scripts/scanner-control.service`; wrapper script at
  `scripts/start_control_panel.sh`. Logs via `journalctl -u scanner-control`.
- **Point-LIO** running on Horizon (SDK1 patch applied, correct CustomMsg type)
- **VDBFusion TSDF meshing** — dense replay config produces clean meshes
- **Colorization** — D435i frames projected onto mesh vertices via calibrated `T_cam_lidar`
- **Point cloud export** — `/cloud_registered` recorded during replay, parsed
  to LAS with correct numpy field extraction (`reshape(n, point_step)`, not `raw[off::step]`)
- **Potree** — PotreeConverter 2.1.1 with bundled viewer; start/stop via UI or `scripts/potree.sh`

---

## Settled decisions — do not relitigate

**IMU source:** Horizon built-in BMI088 only. D435i BMI055 and WT901C are excluded.
The BMI088 is already in the Livox time domain; no cross-sensor sync needed.

**Driver:** `livox_ros2_driver` (SDK1). `livox_ros_driver2` is HAP/Mid-360 only.
CustomMsg type: `livox_interfaces/msg/CustomMsg`.

**Point-LIO SDK1 patch:** changes the CustomMsg include from `livox_ros_driver2`
to `livox_interfaces`. Field layout is identical. Patch in `scripts/vendor_patches/`.

**Point cloud source:** always `/cloud_registered` (Point-LIO's deskewed,
world-frame output). Raw `/livox/lidar` per-frame accumulation without deskewing
produces disconnected blobs at frame boundaries and was explicitly rejected.

**D435i depth:** not integrated into the LIO pipeline or meshing. Color only,
for mesh colorization. The sensor density/range profile doesn't complement the
Horizon at room scale.

**T_cam_lidar:** `R = [[0,-1,0],[0,0,-1],[1,0,0]]` — maps Livox frame
(X=fwd, Y=left, Z=up) to D435i frame (X=right, Y=down, Z=fwd). Translation:
camera is ~100mm above LiDAR, ~4mm forward, center-aligned laterally.
Physical values in `scripts/calib_lidar_camera.yaml`.

---

## Known gotchas

**PointCloud2 numpy parsing:** `raw[off::step][:n*4]` extracts 1 byte per point,
not a complete float32. Correct: `pts = np.frombuffer(raw, uint8).reshape(n, step);
field = np.frombuffer(pts[:, off:off+4].tobytes(), dtype='<f4')`.

**Sky-pointing causes silent LIO divergence:** Point-LIO silently loses track when
the LiDAR is aimed at open sky (near-zero returns → no plane residuals → iEKF
diverges). The estimator freezes on the last valid frame; no error is thrown;
processing completes but produces no mesh. The control panel now shows a live
pts/frame metric and amber warning banner when density drops below 5,000 pts/frame.
During processing replay, a stalled `/cloud_registered` (>5 s without a message
after having been active) turns the processing status line orange with a divergence
message. See `docs/issues/2026_06_24_point_lio_sky_divergence.md`.

**Stale server process:** `kill $(fuser 8090/tcp)` can silently fail if the old
process doesn't respond to SIGTERM. Use `kill -9 <pid>` or `fuser -k 8090/tcp`
when restarting after a rebuild.

**PotreeConverter version:** 2.x generates `metadata.json + octree.bin` format,
incompatible with Potree viewer 1.8.x (`cloud.js` format). The 2.x release zip
bundles its own compatible viewer under `resources/page_template/libs/`. Use
`--generate-page` and serve the output directory directly. `liblaszip.so` symlink
required: `ln -sf /usr/lib/.../liblaszip.so.8 vendor/PotreeConverter/liblaszip.so`.

**Sessions directory:** lives at `<workspace-root>/sessions/`, not inside the
scanner project. Scripts resolve this via `$REPO_ROOT/../../sessions/`.

---

## What's not done / future work

- **LIO health signal:** coverage node derives a `[0,1]` score from odom pose
  covariance trace. A real degeneracy signal (condition number of iEKF information
  matrix) would be more meaningful but requires patching Point-LIO.

- **Color calibration:** physical measurements are approximate. A targetless
  calibration pass (e.g. using a checkerboard) would improve colorization accuracy
  beyond the current ~72% vertex coverage.

- **Offline refinement:** the raw bag contains `/livox/lidar + /livox/imu + /tf`.
  Loop-closure or pose-graph optimization from a longer session is possible via
  replay but not wired up.

- **Outlier filtering:** TSDF meshing is already robust to sparse outlier frames
  (volumetric averaging rejects single-frame artifacts). If point cloud aesthetics
  matter, statistical outlier removal (Open3D `remove_statistical_outlier`) or a
  voxel frame-count filter could be added as a post-processing pass on the LAS.

---

## Task checklist (original scaffold — all completed)

- [x] §2 clean-box `setup_workspace.sh` → green colcon build
- [x] §3 Horizon driver: CustomMsg + IMU topic confirmed
- [x] §4 Point-LIO SDK1 CustomMsg patch (Option A) builds + subscribes
- [x] §4 real LiDAR↔IMU extrinsic + BMI088 params filled in
- [x] §4 verify real output topic names, update meshing/coverage configs
- [x] §5 validate coverage heatmap on real cloud; tune voxels
- [x] §6 VDBFusion integrate + extract + publish + PLY save
- [x] end-to-end room scan produces crisp map + saved bag
- [x] browser control panel (scan/process/colorize/Potree)
- [x] D435i colorization with correct T_cam_lidar
- [x] point cloud export (LAS from /cloud_registered)
- [x] Potree viewer integration
