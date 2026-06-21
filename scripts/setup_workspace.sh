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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SRC="${WS_ROOT}/src"
echo "Workspace root: ${WS_ROOT}"
echo "Source dir:     ${SRC}"

apply_vendor_patch() {
  local repo_dir="$1"
  local patch_file="$2"
  local label="$3"

  if patch --forward --dry-run -p1 -d "${repo_dir}" < "${patch_file}" >/dev/null 2>&1; then
    echo ">> Applying ${label} patch"
    patch --forward -p1 -d "${repo_dir}" < "${patch_file}"
    return
  fi

  if patch --reverse --dry-run -p1 -d "${repo_dir}" < "${patch_file}" >/dev/null 2>&1; then
    echo ">> ${label} patch already applied"
    return
  fi

  echo "ERROR: ${label} patch no longer applies cleanly: ${patch_file}"
  exit 1
}

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: ROS 2 Humble not found at /opt/ros/humble. Install it first (see docs/SETUP.md)."
  exit 1
fi
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u

cd "${SRC}"

# --- 1. Livox SDK (C lib the driver links against) ----------------------------
if [ ! -d "Livox-SDK" ]; then
  echo ">> Cloning + building Livox-SDK (SDK1)"
  git clone https://github.com/Livox-SDK/Livox-SDK.git
fi
apply_vendor_patch \
  "${SRC}/Livox-SDK" \
  "${SCRIPT_DIR}/vendor_patches/livox-sdk-ubuntu-2204.patch" \
  "Livox-SDK Ubuntu 22.04"
pushd Livox-SDK/build >/dev/null 2>&1 || (mkdir -p Livox-SDK/build && pushd Livox-SDK/build >/dev/null)
cmake .. && make -j"$(nproc)" && sudo make install
popd >/dev/null

# --- 2. livox_ros2_driver (SDK1 ROS2 driver, supports Horizon) ----------------
if [ ! -d "livox_ros2_driver" ]; then
  echo ">> Cloning livox_ros2_driver (SDK1 -- Horizon-capable)"
  git clone https://github.com/Livox-SDK/livox_ros2_driver.git
fi

# --- 3. Point-LIO (ROS2) ------------------------------------------------------
if [ ! -d "point_lio" ]; then
  echo ">> Cloning Point-LIO (ROS2)"
  git clone https://github.com/dfloreaa/point_lio_ros2.git point_lio
fi
apply_vendor_patch \
  "${SRC}/point_lio" \
  "${SCRIPT_DIR}/vendor_patches/point-lio-sdk1-custommsg.patch" \
  "Point-LIO SDK1 CustomMsg"

# --- 4. foxglove_bridge (apt) -------------------------------------------------
echo ">> Installing build/tooling deps + foxglove_bridge + sensor_msgs_py (apt)"
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-foxglove-bridge \
  ros-humble-sensor-msgs-py

# --- 5. VDBFusion (pip) -------------------------------------------------------
echo ">> Installing VDBFusion (pip)"
python3 -m pip install --user vdbfusion || echo "   (vdbfusion pip install failed; see HANDOFF §6 for source build)"

# --- 6. rosdep + build --------------------------------------------------------
cd "${WS_ROOT}"
rosdep install --from-paths src --ignore-src -y
echo ">> colcon build"
colcon build --symlink-install

echo ""
echo "Done. Source the overlay:  source ${WS_ROOT}/install/setup.bash"
echo "Then:                      ros2 launch scanner_bringup scanner.launch.py"
