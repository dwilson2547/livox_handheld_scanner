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
livox_ros2_driver, Point-LIO), installs foxglove_bridge / sensor_msgs_py /
vdbfusion, then runs `colcon build`. Read its output — it prints the SDK1
CustomMsg action item for Point-LIO.

## 3. Network / USB for the Horizon

The Horizon talks over Ethernet (via the Livox converter). Set a static IP on
the NUC's wired interface in the Livox subnet, confirm you can ping the unit,
and put the unit's broadcast code into
`src/scanner_bringup/config/horizon.json`.

## 4. Smoke test (no LIO)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch livox_ros2_driver livox_lidar_launch.py   # or the SDK1 launch name
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu     # CONFIRM the IMU topic is actually publishing
```

If `/livox/imu` is silent, the IMU isn't enabled — check `imu_rate` in
`horizon.json`. The BMI088 is a separate topic from the cloud and is easy to miss.

## 5. Full pipeline

```bash
ros2 launch scanner_bringup scanner.launch.py
```

Then connect Foxglove Studio to `ws://<nuc-ip>:8765` and import
`foxglove/operator_layout.json`.

## 6. Offline replay / dev without hardware

```bash
ros2 launch scanner_bringup scanner.launch.py use_bag:=true bag_path:=sessions/<session>
```
