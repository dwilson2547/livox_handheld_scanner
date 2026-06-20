#!/usr/bin/env python3
"""
Top-level launch for the handheld Livox Horizon scanner.

Brings up, in order:
  1. Livox Horizon driver (livox_ros2_driver, SDK1, CustomMsg output)  -- or a bag replay
  2. Point-LIO odometry
  3. VDBFusion meshing node
  4. Coverage / health node
  5. Parallel rosbag recorder of raw topics (for offline refinement)
  6. (optional) Foxglove bridge

Launch args:
  use_bag    (bool, default false)  -- replay a bag instead of live driver
  bag_path   (str)                  -- path to bag dir when use_bag:=true
  record     (bool, default true)   -- record raw topics during a live session
  foxglove   (bool, default true)   -- start foxglove_bridge
  rviz       (bool, default false)  -- start rviz2 with the bundled config
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    bringup_share = FindPackageShare("scanner_bringup")

    use_bag = LaunchConfiguration("use_bag")
    bag_path = LaunchConfiguration("bag_path")
    record = LaunchConfiguration("record")
    foxglove = LaunchConfiguration("foxglove")
    rviz = LaunchConfiguration("rviz")

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
        condition=IfCondition(
            # only when NOT replaying a bag
            ["false" if context.launch_configurations["use_bag"] == "true" else "true"]
        ),
        parameters=[
            PathJoinSubstitution([bringup_share, "config", "horizon.json"]),
            {
                "xfer_format": 1,        # 1 = Livox CustomMsg (keeps per-point timestamps)
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,    # Hz
                "output_data_type": 0,
                "frame_id": "livox_frame",
            },
        ],
    )
    actions.append(driver)

    # --- 1b. Bag replay (alternative to live driver) ------------------------
    bag_replay = ExecuteProcess(
        cmd=["ros2", "bag", "play", "--clock", bag_path],
        output="screen",
        condition=IfCondition(use_bag),
    )
    actions.append(bag_replay)

    # --- 2. Point-LIO -------------------------------------------------------
    # NOTE(copilot): point_lio package is vendored. The shipped config is
    # patched for the Horizon (SDK1 CustomMsg + BMI088 extrinsic). See
    # config/point_lio_horizon.yaml.
    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("point_lio"), "launch", "point_lio.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution(
                [bringup_share, "config", "point_lio_horizon.yaml"]
            ),
            "use_sim_time": use_bag,  # bag replay uses /clock
        }.items(),
    )
    actions.append(point_lio)

    # --- 3. Meshing (VDBFusion) --------------------------------------------
    meshing = Node(
        package="scanner_meshing",
        executable="vdb_meshing_node",
        name="vdb_meshing",
        output="screen",
        parameters=[
            PathJoinSubstitution([bringup_share, "config", "meshing.yaml"]),
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
            "-o", "sessions/session",        # NOTE(copilot): timestamp this dir
            "/livox/lidar",
            "/livox/imu",
            "/tf", "/tf_static",
        ],
        output="screen",
        condition=IfCondition(record),
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
            DeclareLaunchArgument("use_bag", default_value="false"),
            DeclareLaunchArgument("bag_path", default_value=""),
            DeclareLaunchArgument("record", default_value="true"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
