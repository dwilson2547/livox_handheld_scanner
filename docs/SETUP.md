# Setup (Ubuntu 22.04, NUC-class x86)

## 1. OS and ROS 2

Install **Ubuntu 22.04 LTS**, then ROS 2 Humble:

```bash
# ROS 2 Humble (desktop)
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init && rosdep update
```

## 2. Workspace setup

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this-repo-url> livox_handheld_scanner
cd ~/ros2_ws
bash src/livox_handheld_scanner/scripts/setup_workspace.sh
```

`setup_workspace.sh` does everything in one pass:
- clones Livox-SDK, livox_ros2_driver, Point-LIO into `src/`
- applies Ubuntu 22.04 fixes to Livox-SDK (`<memory>` includes + PIC build)
- applies the SDK1 CustomMsg patch to Point-LIO (changes include from `livox_ros_driver2` to `livox_interfaces`)
- installs apt/pip dependencies (foxglove_bridge, sensor_msgs_py, vdbfusion, laspy, open3d, etc.)
- runs `colcon build`

## 3. PotreeConverter (one-time)

```bash
bash src/livox_handheld_scanner/scripts/setup_potree.sh
```

Downloads PotreeConverter 2.1.1 with bundled viewer into `vendor/PotreeConverter/`.
Creates the `liblaszip.so` symlink needed at runtime.

## 4. Horizon network and broadcast code

The Horizon communicates over Ethernet via the Livox converter box. Set a static
IP on the NUC's wired interface in the Livox subnet (default `192.168.1.x`).

Find the broadcast code:

```bash
bash src/livox_handheld_scanner/scripts/discover_horizon_broadcast_code.sh
```

Copy the 15-character code into `src/scanner_bringup/config/horizon.json` and set
`enable_connect: true`.

## 5. Hardware smoke test

```bash
source ~/ros2_ws/install/setup.bash
bash src/livox_handheld_scanner/scripts/smoke_test_live_sensors.sh          # LiDAR + IMU only
bash src/livox_handheld_scanner/scripts/smoke_test_live_sensors.sh true     # + D435i
```

Check that `/livox/imu` is publishing at ~200 Hz — the BMI088 topic is separate
from the point cloud and easy to miss. If it's silent, check `imu_rate` in `horizon.json`.

If using the D435i, run `lsusb -t` and confirm it enumerates at **5000M** (USB 3).
USB 2 fallback degrades color/depth throughput significantly.

## 6. Autostart on boot (systemd service)

The control panel can run as a system service so it starts automatically when the
NUC boots — no login required, just plug in and open the browser.

```bash
# Install (one-time, run from the workspace root)
sudo cp src/livox_handheld_scanner/scripts/scanner-control.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now scanner-control
```

The service file is version-controlled at `scripts/scanner-control.service`.
After a re-image or workspace change, just re-run the three lines above.

**Useful commands:**

```bash
systemctl status scanner-control          # is it running?
journalctl -u scanner-control -f          # live log
sudo systemctl restart scanner-control    # restart after a rebuild
sudo systemctl disable scanner-control    # revert to manual start
```

**When the service is installed** the control panel is available at
`http://<scanner-ip>:8090` within ~5 seconds of boot.

## 7. Control panel (manual start / development)

If the systemd service is not installed, or you want to run a second instance
for testing, start the control panel manually:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch scanner_bringup control_panel.launch.py
```

Open `http://<scanner-ip>:8090`. From here:

1. **Start scan** — begins capturing raw bag to `<workspace>/sessions/<timestamp>/`
2. **Stop scan** — finalizes the bag
3. **Process** — replays through Point-LIO + VDBFusion; produces `mesh_dense_replay.ply`
   and `pointcloud.las` from `/cloud_registered`
4. **Colorize** — projects D435i frames onto mesh vertices (requires `enable_camera:=true`
   during capture, or a camera-enabled bag); produces `mesh_colored.ply`
5. **Launch Potree** — converts `pointcloud.las` to octree and serves at `:8087`;
   button changes to **Open Potree ↗** and **Stop Potree** when running

## 8. Potree CLI (alternative to UI)

```bash
bash scripts/potree.sh                        # show status + list sessions
bash scripts/potree.sh start                  # start viewer for most recent session
bash scripts/potree.sh start living-room      # fuzzy-match session name
bash scripts/potree.sh stop                   # stop running viewer
```

## 9. Offline replay without hardware

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch scanner_bringup scanner.launch.py use_bag:=true bag_path:=sessions/<session>
```

This is what the control panel "Process" button does internally, with the dense configs.

## 10. Calibration

Physical `T_cam_lidar` values are in `scripts/calib_lidar_camera.yaml`. The
rotation `R = [[0,-1,0],[0,0,-1],[1,0,0]]` maps Livox frame to D435i frame —
do not change this unless you remount the sensors. The translation reflects the
physical offset (camera ~100mm above LiDAR, ~4mm forward, laterally centered).

For a more accurate calibration, use a targetless method (e.g. `direct_visual_lidar_calibration`)
and update the YAML. The colorization node picks it up on next run without rebuilding.

## Session output layout

```
sessions/<session-name>/
  <session>_0.db3       raw bag (LiDAR + IMU)
  metadata.yaml         bag metadata
  mesh_dense_replay.ply processed TSDF mesh
  mesh_colored.ply      colorized mesh
  pointcloud.las        deskewed point cloud (from /cloud_registered)
  potree/               Potree octree (generated on demand)
  dense_replay.log      processing log
```
