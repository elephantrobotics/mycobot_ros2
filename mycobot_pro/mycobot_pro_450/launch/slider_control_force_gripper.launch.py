# -*- coding: utf-8 -*-
"""Force-gripper slider launch; keeps specialized executable, supports urdf:=legacy|v2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mycobot_pro_450.pro450_launch_common import (
    joint5_offset_actions,
    resolve_rviz_config,
    robot_state_publisher_node,
)
from mycobot_pro_450.pro450_profiles import (
    get_joint5_offset_deg,
    resolve_urdf_path,
)


def _setup(context, *args, **kwargs):
    urdf_profile = LaunchConfiguration("urdf").perform(context)
    model = LaunchConfiguration("model").perform(context)
    gripper_profile = "f100"
    try:
        urdf_path = resolve_urdf_path(urdf_profile, gripper_profile, model)
        joint5_offset_deg = get_joint5_offset_deg(urdf_profile)
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc

    rvizconfig = resolve_rviz_config(context, urdf_profile, gripper_profile)

    actions = []
    actions.extend(joint5_offset_actions(joint5_offset_deg))
    actions.append(robot_state_publisher_node(urdf_path))
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
    actions.append(
        Node(
            package="mycobot_pro_450",
            executable="slider_control_force_gripper",
            name="slider_control_force_gripper",
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
        [
            DeclareLaunchArgument(
                "urdf",
                default_value="legacy",
                description="Pro 450 structure profile: legacy or v2",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="",
                description="Optional URDF path override",
            ),
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
