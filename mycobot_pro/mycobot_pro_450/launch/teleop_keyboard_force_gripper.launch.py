# -*- coding: utf-8 -*-
"""Compatibility entry that forces gripper:=f100 into teleop_keyboard.launch.py."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("urdf", default_value="legacy"),
            DeclareLaunchArgument("model", default_value=""),
            DeclareLaunchArgument("rvizconfig", default_value=""),
            DeclareLaunchArgument("ip", default_value="192.168.0.232"),
            DeclareLaunchArgument("port", default_value="4500"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/teleop_keyboard.launch.py"]
                ),
                launch_arguments={
                    "urdf": LaunchConfiguration("urdf"),
                    "gripper": "f100",
                    "model": LaunchConfiguration("model"),
                    "rvizconfig": LaunchConfiguration("rvizconfig"),
                    "ip": LaunchConfiguration("ip"),
                    "port": LaunchConfiguration("port"),
                }.items(),
            ),
        ]
    )
