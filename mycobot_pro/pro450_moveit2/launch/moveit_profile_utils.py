# -*- coding: utf-8 -*-
"""Pro 450 MoveIt2 profile selection helpers for launch files."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Tuple

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

PACKAGE_NAME = "pro450_moveit2"
SUPPORTED_MOVEIT_PROFILES = ("legacy/none", "v2/none", "v2/f100")

# robot_name matches config/{robot_name}.urdf.xacro and .srdf basenames.
_PROFILE_FILES: Dict[Tuple[str, str], Dict[str, str]] = {
    ("legacy", "none"): {
        "robot_name": "firefighter",
        "joint_limits": "config/joint_limits.yaml",
        "moveit_controllers": "config/moveit_controllers.yaml",
        "ros2_controllers": "config/ros2_controllers.yaml",
        "initial_positions": "config/initial_positions.yaml",
    },
    ("v2", "none"): {
        "robot_name": "pro450_v2",
        "joint_limits": "config/joint_limits_v2.yaml",
        "moveit_controllers": "config/moveit_controllers_v2.yaml",
        "ros2_controllers": "config/ros2_controllers_v2.yaml",
        "initial_positions": "config/initial_positions_v2.yaml",
    },
    ("v2", "f100"): {
        "robot_name": "pro450_v2_force_gripper",
        "joint_limits": "config/joint_limits_v2_f100.yaml",
        "moveit_controllers": "config/moveit_controllers_v2_f100.yaml",
        "ros2_controllers": "config/ros2_controllers_v2_f100.yaml",
        "initial_positions": "config/initial_positions_v2_f100.yaml",
    },
}


def resolve_moveit_profile(urdf_profile: str, gripper_profile: str) -> Dict[str, str]:
    """Return file map for a supported MoveIt profile, or raise ValueError."""
    key = (urdf_profile, gripper_profile)
    if key not in _PROFILE_FILES:
        raise ValueError(
            "Unsupported MoveIt profile: urdf={} gripper={}. "
            "Supported values: {}".format(
                urdf_profile, gripper_profile, ", ".join(SUPPORTED_MOVEIT_PROFILES)
            )
        )
    return dict(_PROFILE_FILES[key])


def build_moveit_configs(urdf_profile: str, gripper_profile: str):
    """Build MoveItConfigs for the selected Pro 450 profile.

    Pass URDF/SRDF paths explicitly. ``.setup_assistant`` pins firefighter.srdf,
    so a bare ``robot_description_semantic()`` would ignore the selected profile.
    """
    profile = resolve_moveit_profile(urdf_profile, gripper_profile)
    share = Path(get_package_share_directory(PACKAGE_NAME))
    initial_positions = str(share / profile["initial_positions"])
    robot_name = profile["robot_name"]

    return (
        MoveItConfigsBuilder(robot_name, package_name=PACKAGE_NAME)
        .robot_description(
            file_path=f"config/{robot_name}.urdf.xacro",
            mappings={"initial_positions_file": initial_positions},
        )
        .robot_description_semantic(file_path=f"config/{robot_name}.srdf")
        .joint_limits(file_path=profile["joint_limits"])
        .trajectory_execution(file_path=profile["moveit_controllers"])
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )


def ros2_controllers_path(urdf_profile: str, gripper_profile: str) -> str:
    """Absolute path to the ros2_controllers yaml for this profile."""
    profile = resolve_moveit_profile(urdf_profile, gripper_profile)
    share = Path(get_package_share_directory(PACKAGE_NAME))
    return str(share / profile["ros2_controllers"])
