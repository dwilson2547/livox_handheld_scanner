#!/usr/bin/env bash
#
# calibrate_lidar_camera.sh
# Helper notes for the Horizon <-> D435i extrinsic, used only for colorization /
# visual cross-check (NOT for LIO -- the LIO IMU is the Horizon's BMI088).
#
# This is NOT needed for odometry or meshing. Run it only when you want colored
# meshes or to project the camera onto the cloud.
#
# Recommended tool: livox_camera_calib (HKU MaRS). With the short ~50-150mm
# baseline in this rig, calibration is sensitive to small rotation errors, so
# capture a scene with strong depth-discontinuity edges.
#
# TODO(copilot): wire this into a proper ros2 launch once the camera path is
# actually wanted. For now it's a documented manual procedure.
set -euo pipefail

cat <<'EOF'
Horizon <-> D435i extrinsic calibration (manual procedure)
----------------------------------------------------------
1. Record a short bag with both sensors static, viewing a scene with clear
   straight edges at multiple depths (a doorway / box corner works well):

     ros2 bag record -o calib_capture /livox/lidar /camera/color/image_raw \
       /camera/color/camera_info

2. Use livox_camera_calib (build separately, HKU MaRS repo) to solve the
   LiDAR->camera transform from the edge correspondences.

3. Drop the resulting 4x4 into a static_transform_publisher in
   scanner.launch.py if/when you enable the colorization path.

Reminder: the D435i's BMI055 IMU stays OUT of the LIO pipeline. Camera = color
and visual cross-check only.
EOF
