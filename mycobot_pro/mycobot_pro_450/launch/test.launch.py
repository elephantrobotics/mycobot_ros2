# -*- coding: utf-8 -*-
"""RViz display for Pro 450 with legacy/v2 URDF profile selection."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mycobot_pro_450.pro450_launch_common import (
    joint5_offset_actions,
    resolve_profile_from_context,
    resolve_rviz_config,
    robot_state_publisher_node,
)
from mycobot_pro_450.pro450_profiles import declare_profile_launch_arguments


def _setup(context, *args, **kwargs):
    urdf_profile, gripper_profile, urdf_path, joint5_offset_deg = (
        resolve_profile_from_context(context)
    )
    rvizconfig = resolve_rviz_config(context, urdf_profile, gripper_profile)

    actions = []
    actions.extend(joint5_offset_actions(joint5_offset_deg))
    actions.append(robot_state_publisher_node(urdf_path))
    actions.append(
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(LaunchConfiguration("gui")),
        )
    )
    actions.append(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("gui")),
        )
    )
    actions.append(
        Node(
            name="rviz2",
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rvizconfig],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        declare_profile_launch_arguments()
        + [
            DeclareLaunchArgument(
                "rvizconfig",
                default_value="",
                description="Optional RViz2 config override",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Use joint_state_publisher_gui when true",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
