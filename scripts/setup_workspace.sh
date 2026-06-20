#!/usr/bin/env bash
#
# setup_workspace.sh
# Vendors the external ROS 2 dependencies the scanner needs that are NOT part of
# this repo, into a colcon workspace, then builds everything.
#
# Target: Ubuntu 22.04 + ROS 2 Humble.
#
# Run from an empty/colcon workspace root that contains this repo under src/, e.g.:
#   ~/ros2_ws/src/livox_handheld_scanner   <- this repo
#   ~/ros2_ws$  ./src/livox_handheld_scanner/scripts/setup_workspace.sh
#
# IMPORTANT (Horizon = SDK1):
#   The Horizon is an SDK1 device. We use livox_ros2_driver (Mid-40/70/Tele-15/
#   Horizon/Avia), NOT livox_ros_driver2 (HAP/Mid-360 only). The Point-LIO fork
#   must accept the SDK1 CustomMsg (livox_interfaces/msg/CustomMsg). See HANDOFF §4.
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SRC="${WS_ROOT}/src"
echo "Workspace root: ${WS_ROOT}"
echo "Source dir:     ${SRC}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: ROS 2 Humble not found at /opt/ros/humble. Install it first (see docs/SETUP.md)."
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "${SRC}"

# --- 1. Livox SDK (C lib the driver links against) ----------------------------
if [ ! -d "Livox-SDK" ]; then
  echo ">> Cloning + building Livox-SDK (SDK1)"
  git clone https://github.com/Livox-SDK/Livox-SDK.git
  pushd Livox-SDK/build >/dev/null 2>&1 || (mkdir -p Livox-SDK/build && pushd Livox-SDK/build >/dev/null)
  cmake .. && make -j"$(nproc)" && sudo make install
  popd >/dev/null
fi

# --- 2. livox_ros2_driver (SDK1 ROS2 driver, supports Horizon) ----------------
if [ ! -d "livox_ros2_driver" ]; then
  echo ">> Cloning livox_ros2_driver (SDK1 -- Horizon-capable)"
  git clone https://github.com/Livox-SDK/livox_ros2_driver.git
  # TODO(copilot): some forks rename the package dir; verify the inner package
  # name resolves to 'livox_ros2_driver' for the launch file in scanner.launch.py.
fi

# --- 3. Point-LIO (ROS2) ------------------------------------------------------
if [ ! -d "point_lio" ]; then
  echo ">> Cloning Point-LIO (ROS2)"
  git clone https://github.com/dfloreaa/point_lio_ros2.git point_lio
  echo ""
  echo "   !! ACTION REQUIRED (copilot): this fork expects livox_ros_driver2's"
  echo "      CustomMsg. The Horizon driver publishes livox_interfaces/msg/CustomMsg"
  echo "      (SDK1). Patch the Point-LIO preprocess to accept the SDK1 message"
  echo "      type, OR add a small relay node that converts SDK1 CustomMsg ->"
  echo "      the type Point-LIO expects. See HANDOFF §4 for both options."
  echo ""
fi

# --- 4. foxglove_bridge (apt) -------------------------------------------------
echo ">> Installing foxglove_bridge + sensor_msgs_py (apt)"
sudo apt-get update
sudo apt-get install -y ros-humble-foxglove-bridge ros-humble-sensor-msgs-py

# --- 5. VDBFusion (pip) -------------------------------------------------------
echo ">> Installing VDBFusion (pip)"
pip install --user vdbfusion || echo "   (vdbfusion pip install failed; see HANDOFF §6 for source build)"

# --- 6. rosdep + build --------------------------------------------------------
cd "${WS_ROOT}"
rosdep install --from-paths src --ignore-src -y || true
echo ">> colcon build"
colcon build --symlink-install

echo ""
echo "Done. Source the overlay:  source ${WS_ROOT}/install/setup.bash"
echo "Then:                      ros2 launch scanner_bringup scanner.launch.py"
