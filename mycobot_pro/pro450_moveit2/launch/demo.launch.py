# -*- coding: utf-8 -*-
"""Unified Pro 450 MoveIt2 Fake demo with URDF/gripper profile selection.

Supported:
  ros2 launch pro450_moveit2 demo.launch.py
  ros2 launch pro450_moveit2 demo.launch.py urdf:=v2
  ros2 launch pro450_moveit2 demo.launch.py urdf:=v2 gripper:=f100

Defaults remain urdf:=legacy gripper:=none. legacy/f100 is rejected.
Reference packages (pro450_v2_*_moveit2_config) are kept and still usable alone.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_spawn_controllers_launch,
    generate_static_virtual_joint_tfs_launch,
    generate_warehouse_db_launch,
)

# launch/ is installed to share/pro450_moveit2/launch
_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from moveit_profile_utils import (  # noqa: E402
    build_moveit_configs,
    ros2_controllers_path,
)


def _launch_setup(context, *args, **kwargs):
    urdf_profile = context.launch_configurations.get("urdf", "legacy")
    gripper_profile = context.launch_configurations.get("gripper", "none")

    try:
        moveit_config = build_moveit_configs(urdf_profile, gripper_profile)
        controllers_yaml = ros2_controllers_path(urdf_profile, gripper_profile)
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc

    actions = [
        Node(
            package="pro450_moveit2_control",
            executable="moveit_profile",
            name="pro450_moveit_profile",
            output="screen",
            parameters=[
                {
                    "urdf": urdf_profile,
                    "gripper": gripper_profile,
                    "joint5_offset_deg": 90.0 if urdf_profile == "v2" else 0.0,
                }
            ],
        )
    ]
    actions.extend(generate_static_virtual_joint_tfs_launch(moveit_config).entities)
    actions.extend(generate_rsp_launch(moveit_config).entities)
    actions.extend(generate_move_group_launch(moveit_config).entities)

    actions.append(
        GroupAction(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            actions=list(generate_moveit_rviz_launch(moveit_config).entities),
        )
    )
    actions.append(
        GroupAction(
            condition=IfCondition(LaunchConfiguration("db")),
            actions=list(generate_warehouse_db_launch(moveit_config).entities),
        )
    )

    actions.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                controllers_yaml,
            ],
            output="screen",
        )
    )
    actions.extend(generate_spawn_controllers_launch(moveit_config).entities)
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf",
                default_value="legacy",
                description="URDF profile: legacy | v2",
            ),
            DeclareLaunchArgument(
                "gripper",
                default_value="none",
                description="Gripper profile: none | f100 (f100 requires urdf:=v2)",
            ),
            DeclareBooleanLaunchArg(
                "db",
                default_value=False,
                description="By default, we do not start a database (it can be large)",
            ),
            DeclareBooleanLaunchArg(
                "debug",
                default_value=False,
                description="By default, we are not in debug mode",
            ),
            DeclareBooleanLaunchArg("use_rviz", default_value=True),
            OpaqueFunction(function=_launch_setup),
        ]
    )
