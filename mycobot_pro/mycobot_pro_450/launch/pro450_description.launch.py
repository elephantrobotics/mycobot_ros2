# -*- coding: utf-8 -*-
"""Shared Pro 450 description launch (ROS2 counterpart of ROS1 pro450_description.launch).

Validates ``urdf`` / ``gripper`` profiles, resolves the URDF path, and injects
``joint5_offset_deg`` into the launch parameter scope (scheme C).

This file is the common include/entry for profile selection. Display and
control launches typically call the same helpers via OpaqueFunction so they
can pass the resolved URDF into ``robot_state_publisher``.
"""

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from mycobot_pro_450.pro450_launch_common import (
    joint5_offset_actions,
    resolve_profile_from_context,
)
from mycobot_pro_450.pro450_profiles import declare_profile_launch_arguments


def _setup(context, *args, **kwargs):
    urdf_profile, gripper_profile, urdf_path, joint5_offset_deg = (
        resolve_profile_from_context(context)
    )
    model = LaunchConfiguration("model").perform(context).strip()
    source = model if model else "{} + {}".format(urdf_profile, gripper_profile)

    actions = []
    actions.extend(joint5_offset_actions(joint5_offset_deg))
    actions.append(
        LogInfo(
            msg=[
                "Pro 450 description profile: ",
                source,
                " | URDF: ",
                urdf_path,
                " | joint5_offset_deg=",
                str(joint5_offset_deg),
            ]
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        declare_profile_launch_arguments()
        + [OpaqueFunction(function=_setup)]
    )
