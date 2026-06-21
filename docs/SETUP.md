# Setup (dedicated NUC, Ubuntu 22.04)

## 0. OS

Install **Ubuntu 22.04 LTS**. The whole stack targets 22.04 + ROS 2 Humble
because that's where the SDK1 Livox driver and Point-LIO have the best coverage.

## 1. ROS 2 Humble

Follow the official ROS 2 Humble Debian install (desktop variant). Then:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
```

## 2. Workspace + dependencies

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this-repo-url> livox_handheld_scanner
cd ~/ros2_ws
./src/livox_handheld_scanner/scripts/setup_workspace.sh
```

`setup_workspace.sh` clones and builds the external deps (Livox-SDK,
livox_ros2_driver, Point-LIO), applies the repo's Ubuntu 22.04 / SDK1 vendor
patches, installs the build/tooling packages it needs, installs
foxglove_bridge / sensor_msgs_py / vdbfusion, then runs `colcon build`.

## 3. Network / USB for the Horizon

The Horizon talks over Ethernet (via the Livox converter). Set a static IP on
the NUC's wired interface in the Livox subnet, confirm you can ping the unit,
and put the unit's broadcast code into
`src/scanner_bringup/config/horizon.json`.

If you don't know the code yet, use the helper that starts the driver in
auto-discovery mode without modifying the checked-in config:

```bash
cd ~/ros2_ws/src/livox_handheld_scanner
./scripts/discover_horizon_broadcast_code.sh
```

When the driver prints the Horizon's 15-character broadcast code, copy it into
`src/scanner_bringup/config/horizon.json` and set `enable_connect` back to
`true`.

## 4. Smoke test (no LIO)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch livox_ros2_driver livox_lidar_launch.py   # or the SDK1 launch name
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu     # CONFIRM the IMU topic is actually publishing
```

If `/livox/imu` is silent, the IMU isn't enabled — check `imu_rate` in
`horizon.json`. The BMI088 is a separate topic from the cloud and is easy to miss.

For a quick combined hardware sanity check after starting the live sensors, use:

```bash
cd ~/ros2_ws/src/livox_handheld_scanner
./scripts/smoke_test_live_sensors.sh         # Horizon only
./scripts/smoke_test_live_sensors.sh true    # Horizon + D435i
```

## 5. Full pipeline

```bash
ros2 launch scanner_bringup scanner.launch.py
```

To bring up the optional D435i color/depth driver alongside the scanner stack:

```bash
ros2 launch scanner_bringup scanner.launch.py enable_camera:=true
```

Then connect Foxglove Studio to `ws://<nuc-ip>:8765` and import
`foxglove/operator_layout.json`.

**Note:** check `lsusb -t` if RealSense performance looks wrong. The D435i should
enumerate at **5000M** on a USB 3 path; if it falls back to USB 2.x, color/depth
throughput will suffer.

## 6. View a saved scan mesh in the browser

When the scanner stack shuts down cleanly it writes `sessions/mesh_live.ply`. To
inspect that mesh locally:

```bash
cd ~/documents/workspace/robotics/livox_handheld_scanner
python3 scripts/serve_scan_viewer.py
```

Open `http://127.0.0.1:8081` in a browser.

To stage a specific PLY instead of the default `sessions/mesh_live.ply`:

```bash
python3 scripts/serve_scan_viewer.py --scan /path/to/scan.ply --bind 0.0.0.0 --port 8081
```

## 7. Start the browser control panel

The scanner can now host a lightweight browser UI for scan control and live
preview:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch scanner_bringup control_panel.launch.py
```

Open `http://<scanner-ip>:8090`.

Current features:
- Start/stop scan buttons
- live D435i preview
- LiDAR / IMU / health status
- lightweight live mesh preview from `/scanner/mesh`

Current limitations:
- the camera preview only updates while the scan stack is running
- the LiDAR preview is a lightweight operator view and is expected to be sparser
  than the post-scan replay result

## 8. Offline replay / dev without hardware

```bash
ros2 launch scanner_bringup scanner.launch.py use_bag:=true bag_path:=sessions/<session>
```

For a denser post-scan reconstruction pass, use the replay-specific configs:

```bash
ros2 launch scanner_bringup scanner.launch.py \
  use_bag:=true \
  bag_path:=sessions/<session> \
  record:=false foxglove:=false rviz:=false \
  point_lio_config:=/home/daniel/ros2_ws/install/scanner_bringup/share/scanner_bringup/config/point_lio_horizon_dense.yaml \
  meshing_config:=/home/daniel/ros2_ws/install/scanner_bringup/share/scanner_bringup/config/meshing_dense.yaml
```

This is the recommended path for the fuller mesh artifact after an on-device scan.
