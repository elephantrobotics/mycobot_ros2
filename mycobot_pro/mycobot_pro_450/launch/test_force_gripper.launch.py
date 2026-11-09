# -*- coding: utf-8 -*-
"""Compatibility display entry that forces gripper:=f100."""

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
            DeclareLaunchArgument("gui", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/test.launch.py"]
                ),
                launch_arguments={
                    "urdf": LaunchConfiguration("urdf"),
                    "gripper": "f100",
                    "model": LaunchConfiguration("model"),
                    "rvizconfig": LaunchConfiguration("rvizconfig"),
                    "gui": LaunchConfiguration("gui"),
                }.items(),
            ),
        ]
    )
