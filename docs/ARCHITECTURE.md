# Architecture

## Design goals

1. **Live coverage feedback** — operator sees what's been scanned and tracking health in real time.
2. **Clean offline output** — dense mesh and point cloud produced by replaying through Point-LIO
   post-scan, not constrained by live compute budget.
3. **Self-contained** — NUC-class x86, CPU-only, browser UI, no external tooling required.

## Two-stage pipeline

**Capture (live):** Point-LIO odometry + coarse TSDF mesh + coverage heatmap. Parallel raw bag.
The live mesh is a guide; the bag is the source of truth.

**Process (offline replay):** replay the bag through Point-LIO + VDBFusion at dense settings.
Concurrently records `/cloud_registered` to a temp bag, converts to LAS on completion.

## Node graph

```
                ┌─────────────────────────┐
  Horizon ──────► livox_ros2_driver (SDK1) │
  (Ethernet)    │  /livox/lidar (CustomMsg)│
                │  /livox/imu  (BMI088)    │
                └──────────┬──────────────┘
                           │ (also recorded to raw bag)
                           ▼
                ┌─────────────────────────┐
                │        Point-LIO        │
                │  iEKF, point-by-point   │
                │  /cloud_registered      │ ← deskewed, world-frame
                │  /aft_mapped_to_init    │ ← odometry
                │  /tf                    │
                └───────┬──────────┬──────┘
                        │          │
             ┌──────────▼──┐   ┌───▼────────────────┐
             │ vdb_meshing │   │ coverage_node       │
             │ TSDF→mesh   │   │ voxel obs density   │
             │ /scanner/   │   │ + LIO health score  │
             │   mesh      │   │ /scanner/coverage   │
             └──────┬──────┘   │ /scanner/health     │
                    │          └────────┬─────────────┘
                    └─────────┬─────────┘
                              ▼
                    ┌──────────────────┐
                    │ foxglove_bridge  │ ◄── operator (optional, ws://:8765)
                    └──────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ scanner_control (control_server.py)                     http://:8090        │
│                                                                             │
│  start_scan() ──► ros2 launch scanner.launch.py (record:=true)             │
│  stop_scan()  ──► SIGINT                                                    │
│                                                                             │
│  start_processing() ──► ros2 launch scanner.launch.py (use_bag:=true)      │
│                         + ros2 bag record /cloud_registered (temp)          │
│                         → on completion: _cloud_bag_to_las() → pointcloud.las│
│                                                                             │
│  start_colorize() ──► colorize.py (D435i frames → mesh vertex colors)      │
│                       T_cam_lidar from calib_lidar_camera.yaml              │
│                                                                             │
│  start_potree() ──► PotreeConverter (LAS → octree)                         │
│                     python3 -m http.server → http://:8087                  │
└─────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────┐
│ ros2 bag record (during capture)                     │
│   /livox/lidar  /livox/imu  /tf  /tf_static          │
└──────────────────────────────────────────────────────┘
```

## Frames

- `camera_init` — Point-LIO world frame (odom origin)
- `body` — IMU body frame
- `livox_frame` — LiDAR sensor frame

Coverage and meshing consume `/cloud_registered` which is already in `camera_init`,
so they don't need the LiDAR↔IMU extrinsic. Point-LIO handles that internally.

## Key data flows

**Live capture:** `livox_ros2_driver → Point-LIO → vdb_meshing + coverage_node + foxglove_bridge`
plus raw bag in parallel.

**Processing replay:** `ros2 bag play → Point-LIO → vdb_meshing` with dense configs. The
control server concurrently runs `ros2 bag record /cloud_registered` and converts the result
to LAS after the replay finishes.

**Colorization:** loads the D435i color bag, projects each frame through `T_cam_lidar` onto
mesh vertices using per-vertex depth testing, writes `mesh_colored.ply`.

**Potree:** `pointcloud.las → PotreeConverter (--generate-page) → potree/ dir → http.server`.

## Sensor decisions

| Sensor | Decision | Reason |
|--------|----------|--------|
| Horizon BMI088 | ✅ used for LIO | Built-in, in Livox time domain, no sync needed |
| D435i BMI055 | ❌ dropped from LIO | Weaker part, gyro/accel internally unsynced |
| WT901C (external) | ❌ dropped | Poor sync, no advantage over BMI088 |
| D435i depth | ❌ not used in meshing | Doesn't complement Horizon at room scale |
| D435i color | ✅ used for colorization | Post-processing only, not in live pipeline |
