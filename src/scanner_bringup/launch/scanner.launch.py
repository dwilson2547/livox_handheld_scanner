#!/usr/bin/env python3
"""
Top-level launch for the handheld Livox Horizon scanner.

Brings up, in order:
  1. Livox Horizon driver (livox_ros2_driver, SDK1, CustomMsg output)
     or a bag replay
  2. Point-LIO odometry
  3. (optional) RealSense D435i color/depth driver
  4. VDBFusion meshing node
  5. Coverage / health node
  6. Parallel rosbag recorder of raw topics (for offline refinement)
  7. (optional) Foxglove bridge

Launch args:
  use_bag    (bool, default false)  -- replay a bag instead of live driver
  bag_path   (str)                  -- path to bag dir when use_bag:=true
  record     (bool, default true)   -- record raw topics during a live session
  foxglove   (bool, default true)   -- start foxglove_bridge
  rviz       (bool, default false)  -- start rviz2 with the bundled config
  enable_camera (bool, default false)  -- start the optional D435i driver
  point_lio_config (path)              -- override Point-LIO parameter file
  meshing_config   (path)              -- override meshing parameter file
"""
from datetime import datetime
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_yaml_dict(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return data or {}


def _launch_setup(context, *args, **kwargs):
    bringup_share = FindPackageShare("scanner_bringup")
    bringup_share_path = Path(get_package_share_directory("scanner_bringup"))

    use_bag = LaunchConfiguration("use_bag")
    bag_path = LaunchConfiguration("bag_path")
    enable_camera = LaunchConfiguration("enable_camera")
    foxglove = LaunchConfiguration("foxglove")
    point_lio_config = LaunchConfiguration("point_lio_config")
    meshing_config = LaunchConfiguration("meshing_config")
    rviz = LaunchConfiguration("rviz")
    use_bag_enabled = context.launch_configurations["use_bag"] == "true"
    record_enabled = (
        context.launch_configurations["record"] == "true" and not use_bag_enabled
    )
    session_output = f"sessions/session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    actions = []

    # --- 1a. Live Livox Horizon driver (SDK1) -------------------------------
    # NOTE(copilot): livox_ros2_driver launch/param files are vendored by the
    # setup script. The driver MUST be configured to publish CustomMsg
    # (xfer_format=1) so per-point timestamps survive for deskewing.
    # See docs/HANDOFF.md §2 and §4.
    driver = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_driver",
        output="screen",
        condition=IfCondition("false" if use_bag_enabled else "true"),
        parameters=[
            {
                "user_config_path": PathJoinSubstitution(
                    [bringup_share, "config", "horizon.json"]
                ),
                "xfer_format": 1,  # 1 = Livox CustomMsg (keeps point timestamps)
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,  # Hz
                "output_data_type": 0,
                "frame_id": "livox_frame",
            },
        ],
    )
    actions.append(driver)

    # --- 1b. Bag replay (alternative to live driver) ------------------------
    bag_replay = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path, "--clock"],
        output="screen",
        condition=IfCondition(use_bag),
    )
    actions.append(bag_replay)

    # --- 2. Point-LIO -------------------------------------------------------
    # NOTE(copilot): point_lio package is vendored. The shipped config is
    # patched for the Horizon (SDK1 CustomMsg + BMI088 extrinsic). See
    # config/point_lio_horizon.yaml.
    point_lio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="laserMapping",
        output="screen",
        parameters=[
            point_lio_config,
            {"use_sim_time": use_bag},
        ],
    )
    actions.append(point_lio)

    # --- 2b. Optional RealSense D435i --------------------------------------
    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="d435i",
        output="screen",
        condition=IfCondition(enable_camera),
        parameters=[
            _load_yaml_dict(str(bringup_share_path / "config" / "realsense.yaml")),
            {"camera_name": "d435i", "camera_namespace": "camera"},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )
    actions.append(realsense)

    # --- 3. Meshing (VDBFusion) --------------------------------------------
    meshing = Node(
        package="scanner_meshing",
        executable="vdb_meshing_node",
        name="vdb_meshing",
        output="screen",
        parameters=[
            meshing_config,
            {"use_sim_time": use_bag},
        ],
    )
    actions.append(meshing)

    # --- 4. Coverage / health ----------------------------------------------
    coverage = Node(
        package="scanner_coverage",
        executable="coverage_node",
        name="coverage",
        output="screen",
        parameters=[
            PathJoinSubstitution([bringup_share, "config", "coverage.yaml"]),
            {"use_sim_time": use_bag},
        ],
    )
    actions.append(coverage)

    # --- 5. Parallel raw recorder (live sessions only) ----------------------
    # Records exactly the topics needed to re-run LIO + refinement offline.
    recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "-o", session_output,
            "/livox/lidar",
            "/livox/imu",
            "/tf", "/tf_static",
        ],
        output="screen",
        condition=IfCondition("true" if record_enabled else "false"),
    )
    actions.append(recorder)

    # --- 6. Foxglove bridge -------------------------------------------------
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        condition=IfCondition(foxglove),
        parameters=[{"port": 8765}],
    )
    actions.append(foxglove_bridge)

    # --- rviz (optional, dev) ----------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        arguments=[
            "-d",
            PathJoinSubstitution([bringup_share, "rviz", "scanner.rviz"]),
        ],
    )
    actions.append(rviz_node)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_bag",
                default_value="false",
                description="Replay a rosbag instead of starting the live Livox driver.",
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="Path to the bag directory when use_bag:=true.",
            ),
            DeclareLaunchArgument(
                "record",
                default_value="true",
                description="Record raw Livox and TF topics during a live session.",
            ),
            DeclareLaunchArgument(
                "foxglove",
                default_value="true",
                description="Start foxglove_bridge on port 8765.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz with the bundled scanner config.",
            ),
            DeclareLaunchArgument(
                "enable_camera",
                default_value="false",
                description="Start the optional RealSense D435i color/depth driver.",
            ),
            DeclareLaunchArgument(
                "point_lio_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("scanner_bringup"), "config", "point_lio_horizon.yaml"]
                ),
                description="Point-LIO parameter file to use.",
            ),
            DeclareLaunchArgument(
                "meshing_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("scanner_bringup"), "config", "meshing.yaml"]
                ),
                description="Meshing parameter file to use.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
