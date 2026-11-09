# -*- coding: utf-8 -*-
"""Shared OpaqueFunction helpers for Pro 450 ROS2 launches."""

from __future__ import annotations

from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

from mycobot_pro_450.pro450_profiles import (
    default_rviz_config,
    get_joint5_offset_deg,
    resolve_urdf_path,
)


def resolve_profile_from_context(context):
    """Read launch configs and resolve URDF path + J5 offset.

    Raises:
        RuntimeError: when urdf/gripper profiles are invalid.
    """
    urdf_profile = LaunchConfiguration("urdf").perform(context)
    gripper_profile = LaunchConfiguration("gripper").perform(context)
    model = LaunchConfiguration("model").perform(context)

    try:
        urdf_path = resolve_urdf_path(urdf_profile, gripper_profile, model)
        # Offset follows the urdf profile arg even when model overrides the file,
        # matching ROS1 pro450_description.launch behavior.
        joint5_offset_deg = get_joint5_offset_deg(urdf_profile)
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc

    return urdf_profile, gripper_profile, urdf_path, joint5_offset_deg


def robot_description_parameter(urdf_path: str) -> ParameterValue:
    return ParameterValue(Command(["xacro ", urdf_path]), value_type=str)


def joint5_offset_actions(joint5_offset_deg: float):
    """Apply launch-tree SetParameter for scheme C (ROS1-aligned logical name)."""
    return [
        SetParameter(name="joint5_offset_deg", value=float(joint5_offset_deg)),
    ]


def resolve_rviz_config(context, urdf_profile: str, gripper_profile: str) -> str:
    rvizconfig = LaunchConfiguration("rvizconfig").perform(context).strip()
    if rvizconfig:
        return rvizconfig
    return default_rviz_config(urdf_profile, gripper_profile)


def robot_state_publisher_node(urdf_path: str) -> Node:
    # Do not attach joint5_offset_deg here: RSP rejects undeclared parameters.
    # Offset is injected via SetParameter / consumer nodes (scheme C).
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_parameter(urdf_path),
            }
        ],
    )
