#!/usr/bin/env bash
# Wrapper for systemd — sources the ROS 2 environment that .bashrc normally provides,
# then launches the scanner control panel HTTP server.
set -e

source /opt/ros/humble/setup.bash
source /home/daniel/ros2_ws/install/setup.bash

exec ros2 launch scanner_bringup control_panel.launch.py
