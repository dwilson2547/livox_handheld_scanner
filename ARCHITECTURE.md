# Architecture

## Design goals (in priority order)

1. **Live coverage feedback is non-negotiable.** The operator must see, in real
   time, what's been scanned and how well, plus a tracking-health indicator.
   Waving an invisible laser around blind is unacceptable. This requirement
   drives everything toward a live pipeline rather than log-and-process-later.
2. **Dimensionally accurate output suitable for 3D printing.** Final quality
   comes from offline refinement on the big iron, not the live preview.
3. **Self-contained, handheld.** NUC-class compute on the rig, CPU-only.

## Two-stage philosophy

- **Live (on the NUC):** Point-LIO odometry + a coarse live TSDF mesh + coverage
  heatmap, just good enough to guide the operator. Everything also goes to a
  rosbag.
- **Offline (EPYC / Altra):** replay the bag, run loop closure / pose-graph
  optimization, re-integrate the TSDF at a finer voxel size for the print mesh.

The live mesh is a guide; the bag is the source of truth.

## Node graph

```
                 ┌─────────────────────────┐
   Horizon ─────►│ livox_ros2_driver (SDK1) │
   (USB/Eth)     │  /livox/lidar (CustomMsg)│
                 │  /livox/imu  (BMI088)    │
                 └──────────┬───────────────┘
                            │ (raw, also recorded to bag)
                            ▼
                 ┌─────────────────────────┐
                 │       Point-LIO         │
                 │  iEKF, point-by-point   │
                 │  /cloud_registered      │
                 │  /aft_mapped_to_init    │ (odom)
                 │  /tf                    │
                 └───────┬──────────┬──────┘
                         │          │
              ┌──────────▼──┐   ┌───▼───────────────┐
              │ vdb_meshing │   │ coverage          │
              │ TSDF→mesh   │   │ voxel obs density │
              │ /scanner/   │   │ + LIO health      │
              │   mesh      │   │ /scanner/coverage │
              └──────┬──────┘   │ /scanner/health   │
                     │          └────────┬──────────┘
                     └─────────┬─────────┘
                               ▼
                     ┌──────────────────┐
                     │ foxglove_bridge  │ ◄── operator (laptop/tablet)
                     │  :8765           │     coverage heatmap + health
                     └──────────────────┘

   ┌──────────────────────────────────────────────┐
   │ ros2 bag record  (parallel, raw topics)       │
   │  /livox/lidar /livox/imu /tf /tf_static       │  → offline refinement
   └──────────────────────────────────────────────┘
```

## Sensor decisions (settled — do not relitigate)

- **LIO IMU = Horizon's built-in BMI088.** Already rigidly co-mounted and in the
  Livox time domain; Livox publishes the LiDAR↔IMU extrinsic. This sidesteps the
  cross-sensor time-sync problem entirely.
- **WT901C: dropped.** Poor sync, no advantage over the BMI088.
- **D435i BMI055: dropped from LIO.** Weaker part, gyro/accel not well synced
  internally. Camera kept only for optional colorization / visual cross-check.

## Frames

- `camera_init` / world frame: Point-LIO's default `odom_header_frame_id`.
- `body`: IMU body frame.
- `livox_frame`: LiDAR frame.
- Coverage and meshing both consume `/cloud_registered` already in the world frame,
  so they don't need the LiDAR↔IMU extrinsic themselves — but Point-LIO does, and
  getting it right is what makes the map crisp.
