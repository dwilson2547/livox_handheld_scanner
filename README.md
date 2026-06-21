# livox_handheld_scanner

Handheld 3D mapping rig built around a **Livox Horizon** LiDAR with real-time
odometry, TSDF meshing, colorization, and a browser control panel.

## Hardware

| Item | Part | Role |
|------|------|------|
| LiDAR | Livox Horizon | Primary 3D sensor (non-repetitive scan pattern) |
| IMU | Horizon built-in BMI088 | LIO attitude source (time-synced to LiDAR clock) |
| Camera | Intel RealSense D435i | Optional colorization only — IMU not used |
| Compute | NUC-class x86, Ubuntu 22.04 | Live capture + odometry + meshing |

**IMU note:** only the Horizon's built-in BMI088 is used for LIO. The D435i's
BMI055 and any external IMU are deliberately excluded — poor sync, no benefit.

## Software stack

- **OS / middleware:** Ubuntu 22.04 + ROS 2 Humble
- **Driver:** `livox_ros2_driver` (SDK1 — required for the Horizon; `livox_ros_driver2` is HAP/Mid-360 only and will not work)
- **Odometry:** Point-LIO (point-by-point iEKF, handles aggressive hand motion)
- **Meshing:** VDBFusion TSDF integrator (live incremental surface + offline dense replay)
- **Colorization:** per-vertex D435i projection using calibrated `T_cam_lidar`
- **Point cloud:** `/cloud_registered` recorded during processing replay → LAS export
- **Control panel:** browser UI at `:8090` for scan/process/colorize/Potree workflows
- **Potree viewer:** octree point cloud browser at `:8087` (per-session, on demand)

See `docs/ARCHITECTURE.md` for the node graph and `docs/SETUP.md` for setup instructions.

## ⚠️ Critical driver note

The **Horizon is an SDK1 device.** Use `livox_ros2_driver`, not `livox_ros_driver2`.
The CustomMsg type is `livox_interfaces/msg/CustomMsg`. Stock Point-LIO forks expect
the SDK2 message type and must be patched — `setup_workspace.sh` handles this automatically.

## Quick start

```bash
# One-time setup (clones deps, applies patches, builds workspace, downloads PotreeConverter)
cd ~/ros2_ws/src/livox_handheld_scanner
bash scripts/setup_workspace.sh
bash scripts/setup_potree.sh

# Start the control panel (primary interface)
source ~/ros2_ws/install/setup.bash
ros2 launch scanner_bringup control_panel.launch.py
# → open http://<scanner-ip>:8090
```

From the control panel you can:
- **Start / Stop scan** — captures raw bag to `sessions/<timestamp>/`
- **Process** — replays through Point-LIO + VDBFusion, produces mesh + point cloud LAS
- **Colorize** — projects D435i frames onto mesh vertices
- **Launch Potree** — converts point cloud to octree and serves it at `:8087`

## Repo layout

```
src/scanner_bringup/     launch files, sensor configs
src/scanner_control/     HTTP control server + browser UI
src/scanner_coverage/    per-voxel observation density + LIO health node
src/scanner_meshing/     VDBFusion TSDF wrapper node
scripts/                 setup, calibration, Potree lifecycle, export tools
docs/                    ARCHITECTURE.md, HANDOFF.md, SETUP.md
```

## Sessions

Each scan is saved to `<workspace>/sessions/<session-name>/`:

```
<session>/
  *.db3 / metadata.yaml    raw bag (LiDAR + IMU)
  mesh_dense_replay.ply    processed TSDF mesh
  mesh_colored.ply         colorized mesh (if colorized)
  pointcloud.las           deskewed point cloud from /cloud_registered
  potree/                  Potree octree (generated on first "Launch Potree")
  dense_replay.log         processing log
```

## Key gotchas

- **Point-LIO SDK1 patch** — applied automatically by `setup_workspace.sh`; see `scripts/vendor_patches/`
- **Broadcast code** — put the Horizon's 15-char code in `src/scanner_bringup/config/horizon.json`; use `scripts/discover_horizon_broadcast_code.sh` to find it
- **T_cam_lidar** — physical calibration in `scripts/calib_lidar_camera.yaml`; adjust offsets if you remount the camera
- **Point cloud source** — always `/cloud_registered` (Point-LIO deskewed output), never raw `/livox/lidar` per-frame accumulation
