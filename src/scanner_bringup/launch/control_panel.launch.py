#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_SESSIONS_DIR = os.path.expanduser("~/documents/workspace/sessions")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "host",
                default_value="0.0.0.0",
                description="HTTP bind address for the scanner control UI.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8090",
                description="HTTP port for the scanner control UI.",
            ),
            DeclareLaunchArgument(
                "sessions_dir",
                default_value=_DEFAULT_SESSIONS_DIR,
                description="Absolute path to the sessions directory.",
            ),
            Node(
                package="scanner_control",
                executable="scanner_control_server",
                name="scanner_control_server",
                output="screen",
                arguments=[
                    "--bind", LaunchConfiguration("host"),
                    "--port", LaunchConfiguration("port"),
                    "--sessions-dir", LaunchConfiguration("sessions_dir"),
                ],
            ),
        ]
    )
