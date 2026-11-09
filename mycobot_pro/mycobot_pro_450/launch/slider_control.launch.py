# -*- coding: utf-8 -*-
"""Slider control launch with Pro 450 URDF profile selection.

``gripper:=none`` uses the stock joint_state_publisher_gui + slider_control.
``gripper:=f100`` switches to the safe GUI (no Randomize) and
slider_control_force_gripper for real arm+gripper control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mycobot_pro_450.pro450_launch_common import (
    joint5_offset_actions,
    resolve_profile_from_context,
    resolve_rviz_config,
    robot_description_parameter,
    robot_state_publisher_node,
)
from mycobot_pro_450.pro450_profiles import declare_profile_launch_arguments


def _setup(context, *args, **kwargs):
    urdf_profile, gripper_profile, urdf_path, joint5_offset_deg = (
        resolve_profile_from_context(context)
    )
    rvizconfig = resolve_rviz_config(context, urdf_profile, gripper_profile)
    use_force_gripper = gripper_profile == "f100"

    actions = []
    actions.extend(joint5_offset_actions(joint5_offset_deg))
    actions.append(robot_state_publisher_node(urdf_path))

    if use_force_gripper:
        actions.append(
            Node(
                package="mycobot_pro_450",
                executable="safe_joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=IfCondition(LaunchConfiguration("gui")),
                parameters=[
                    {"robot_description": robot_description_parameter(urdf_path)},
                    {"use_gui": True},
                ],
                output="screen",
            )
        )
        control_executable = "slider_control_force_gripper"
        control_name = "slider_control_force_gripper"
    else:
        actions.append(
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(LaunchConfiguration("gui")),
            )
        )
        control_executable = "slider_control"
        control_name = "slider_control"

    actions.append(
        Node(
            name="rviz2",
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rvizconfig],
        )
    )
    actions.append(
        Node(
            package="mycobot_pro_450",
            executable=control_executable,
            name=control_name,
            parameters=[
                {"ip": LaunchConfiguration("ip")},
                {"port": LaunchConfiguration("port")},
                {"joint5_offset_deg": float(joint5_offset_deg)},
            ],
            output="screen",
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
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "ip",
                default_value="192.168.0.232",
                description="IP address used by the device",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="4500",
                description="Port number used by the device",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
