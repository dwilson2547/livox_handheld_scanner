# livox_handheld_scanner

Live LiDAR-inertial handheld 3D mapping rig built around a **Livox Horizon**, with
real-time coverage feedback and parallel rosbag logging for offline print-quality
refinement.

This is a **scaffold / starter repo**. Several nodes are deliberately stubbed and
marked `TODO(copilot)` — see `docs/HANDOFF.md` for the full implementation brief.

## Hardware

| Item | Part | Role |
|------|------|------|
| LiDAR | Livox Horizon | Primary 3D sensor (non-repetitive scan) |
| IMU | Horizon built-in **BMI088** | LIO attitude source (already time-synced to LiDAR clock) |
| Camera | Intel RealSense D435i | Colorization / visual cross-check only — **IMU NOT used** |
| Compute | NUC-class x86, Ubuntu 22.04 | Live capture + LIO + meshing |

**Sensors deliberately dropped from the LIO pipeline:** WT901C (poor sync) and the
D435i's BMI055 (weak, internally unsynced gyro/accel). Do not re-add them.

## Software stack

- **OS / middleware:** Ubuntu 22.04 + ROS 2 Humble
- **Driver:** `livox_ros2_driver` (SDK1 — required for the Horizon; NOT `livox_ros_driver2`, which is HAP/Mid-360 only)
- **Odometry:** Point-LIO (point-by-point iEKF, handles aggressive hand motion better than FAST-LIO2)
- **Meshing:** VDBFusion TSDF integrator (live incremental surface)
- **Coverage:** custom node — per-voxel observation density + LIO health
- **Viewer:** Foxglove Studio (coverage heatmap + health indicator)
- **Logging:** parallel `ros2 bag` of raw topics for offline pose-graph refinement

See `docs/ARCHITECTURE.md` for the node graph and `docs/HANDOFF.md` for the build-out plan.

## ⚠️ Critical hardware/driver note — read before building

The **Horizon is an SDK1 device.** The widely-documented `livox_ros_driver2` only
supports HAP and Mid-360 and **will not drive a Horizon.** This repo uses
`livox_ros2_driver` instead. The practical consequences:

- CustomMsg type is `livox_interfaces/msg/CustomMsg`, **not** `livox_ros_driver2/msg/CustomMsg`.
- Stock Point-LIO ROS2 forks expect the driver2 message type and must be patched to
  accept the SDK1 message. This repo's workspace bootstrap applies that patch automatically;
  see HANDOFF §4.

## Quick start (once dependencies are vendored — see HANDOFF §2)

```bash
# build
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash

# bring up the full live pipeline (driver + point-lio + meshing + coverage + bag)
ros2 launch scanner_bringup scanner.launch.py

# same, with the optional D435i color/depth driver enabled
ros2 launch scanner_bringup scanner.launch.py enable_camera:=true

# config-portal / dry run without hardware (plays a bag instead of the live driver)
ros2 launch scanner_bringup scanner.launch.py use_bag:=true bag_path:=/path/to/session

# denser offline replay / reconstruction pass for a saved session
ros2 launch scanner_bringup scanner.launch.py \
  use_bag:=true \
  bag_path:=/path/to/session \
  record:=false foxglove:=false rviz:=false \
  point_lio_config:=/home/daniel/ros2_ws/install/scanner_bringup/share/scanner_bringup/config/point_lio_horizon_dense.yaml \
  meshing_config:=/home/daniel/ros2_ws/install/scanner_bringup/share/scanner_bringup/config/meshing_dense.yaml
```

## Repo layout

```
src/scanner_bringup/    launch files, sensor configs, rviz/foxglove layouts
src/scanner_control/    browser-hosted control surface and scan start/stop API
src/scanner_coverage/   coverage node (per-voxel density + LIO health)
src/scanner_meshing/    VDBFusion wrapper node
docs/                   ARCHITECTURE.md, HANDOFF.md, SETUP.md
scripts/                env setup, extrinsic calibration helpers
foxglove/               operator layout JSON
viewer/                 browser-based offline scan viewers
```

## Offline scan viewer

The live pipeline already writes `sessions/mesh_live.ply` on shutdown. To inspect
that mesh in a browser:

```bash
cd ~/documents/workspace/robotics/livox_handheld_scanner
python3 scripts/serve_scan_viewer.py
```

Then open `http://127.0.0.1:8081`.

You can also point it at a specific mesh:

```bash
python3 scripts/serve_scan_viewer.py --scan /path/to/scan.ply --bind 0.0.0.0 --port 8081
```

This first pass targets the saved `.ply` mesh directly. Potree is a better fit for
registered point clouds, which can be added later once bag replay/export of
`/cloud_registered` is formalized.

## Browser control panel

To run the scanner-hosted control surface:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch scanner_bringup control_panel.launch.py
```

Then open `http://<scanner-ip>:8090`.

This first pass provides:
- **Start scan / Stop scan** buttons
- live **D435i camera preview**
- live **LiDAR/IMU rate + health + odom** status
- a lightweight **live mesh/point preview** fed from `/scanner/mesh`

The control panel starts and stops the existing `scanner.launch.py` stack as a
child process, keeping the “operator UI” separate from the “scanner pipeline.”

Current limitations:
- the camera preview is only live while the scan stack is running
- the LiDAR preview is intentionally lightweight and should be treated as an
  operator preview, not a final-quality reconstruction
- the recommended final artifact path is still **post-scan replay** using the
  dense replay configs
