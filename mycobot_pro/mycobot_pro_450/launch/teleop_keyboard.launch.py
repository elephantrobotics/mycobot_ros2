# -*- coding: utf-8 -*-
"""Keyboard teleop display/service launch with Pro 450 URDF profile selection.

This launch starts robot_state_publisher, listen_real_service, and RViz.
Run the matching keyboard client in another terminal:

Always run: ros2 run mycobot_pro_450 teleop_keyboard
g/h keys appear only when this launch used gripper:=f100.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
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
            package="mycobot_pro_450",
            executable="listen_real_service",
            name="listen_real_service",
            output="screen",
            parameters=[
                {"ip": LaunchConfiguration("ip")},
                {"port": LaunchConfiguration("port")},
                {"joint5_offset_deg": float(joint5_offset_deg)},
                {"gripper": gripper_profile},
            ],
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
    actions.append(
        LogInfo(
            msg=[
                "Pro 450 keyboard teleop ready (urdf=",
                urdf_profile,
                ", gripper=",
                gripper_profile,
                "). In another terminal run: "
                "ros2 run mycobot_pro_450 teleop_keyboard"
                "  (g/h gripper keys enabled only when gripper:=f100)",
            ]
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
