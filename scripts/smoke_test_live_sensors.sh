#!/usr/bin/env bash
#
# smoke_test_live_sensors.sh
# Quick live hardware check for the Horizon and optional D435i before a room scan.
set -euo pipefail

CAMERA_ENABLED="${1:-false}"

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
set -u

echo "Livox topics:"
ros2 topic list | grep '^/livox/' | sed -n '1,20p' || true
echo
echo "Livox lidar rate:"
timeout 5s ros2 topic hz /livox/lidar | sed -n '1,20p' || true
echo
echo "Livox IMU rate:"
timeout 5s ros2 topic hz /livox/imu | sed -n '1,20p' || true

if [[ "${CAMERA_ENABLED}" == "true" ]]; then
  echo
  echo "RealSense topics:"
  ros2 topic list | grep '^/camera/' | sed -n '1,40p' || true
  echo
  echo "RealSense color rate:"
  timeout 5s ros2 topic hz /camera/d435i/color/image_raw | sed -n '1,20p' || true
  echo
  echo "RealSense depth rate:"
  timeout 5s ros2 topic hz /camera/d435i/depth/image_rect_raw | sed -n '1,20p' || true
fi
