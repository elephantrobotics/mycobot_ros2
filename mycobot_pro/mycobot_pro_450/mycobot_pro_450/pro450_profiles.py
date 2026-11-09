# -*- coding: utf-8 -*-
"""Pro 450 URDF / gripper profile helpers for ROS2 launch files.

Mirrors the ROS1 ``pro450_description.launch`` selection rules:
``urdf:=legacy|v2``, ``gripper:=none|f100``, optional ``model`` override.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

SUPPORTED_URDF_PROFILES = ("legacy", "v2")
SUPPORTED_GRIPPER_PROFILES = ("none", "f100")

_URDF_RELATIVE_PATHS = {
    ("legacy", "none"): os.path.join(
        "urdf", "mycobot_pro_450", "mycobot_pro_450.urdf"
    ),
    ("legacy", "f100"): os.path.join(
        "urdf", "mycobot_pro_450", "mycobot_pro_450_force_gripper.urdf"
    ),
    ("v2", "none"): os.path.join(
        "urdf", "mycobot_pro_450", "new_structure", "mycobot_pro450_v2.urdf"
    ),
    ("v2", "f100"): os.path.join(
        "urdf",
        "mycobot_pro_450",
        "new_structure",
        "mycobot_pro450_v2_force_gripper.urdf",
    ),
}


def validate_urdf_profile(urdf_profile: str) -> None:
    if urdf_profile not in SUPPORTED_URDF_PROFILES:
        raise ValueError(
            "Unsupported urdf profile: {}. Supported values: legacy, v2".format(
                urdf_profile
            )
        )


def validate_gripper_profile(gripper_profile: str) -> None:
    if gripper_profile not in SUPPORTED_GRIPPER_PROFILES:
        raise ValueError(
            "Unsupported gripper profile: {}. Supported values: none, f100".format(
                gripper_profile
            )
        )


def validate_profiles(urdf_profile: str, gripper_profile: str) -> None:
    validate_urdf_profile(urdf_profile)
    validate_gripper_profile(gripper_profile)


def get_joint5_offset_deg(urdf_profile: str) -> float:
    """Return ROS logical J5 offset relative to firmware coordinates."""
    validate_urdf_profile(urdf_profile)
    return 90.0 if urdf_profile == "v2" else 0.0


def resolve_urdf_path(
    urdf_profile: str, gripper_profile: str, model: str = ""
) -> str:
    """Resolve the URDF file path for the selected profile.

    When ``model`` is non-empty it takes precedence, matching ROS1 behavior.
    """
    model = (model or "").strip()
    if model:
        return model

    validate_profiles(urdf_profile, gripper_profile)
    description_share = get_package_share_directory("mycobot_description")
    return os.path.join(
        description_share, _URDF_RELATIVE_PATHS[(urdf_profile, gripper_profile)]
    )


def default_rviz_config(urdf_profile: str = "legacy", gripper_profile: str = "none") -> str:
    """Return the default RViz2 config for Pro 450 display launches.

    v2 + f100 uses ``base_link`` as the root link, so it needs a dedicated
    RViz config (same rule as ROS1). All other profiles use Fixed Frame ``base``.
    """
    config_name = "mycobot_pro_450.rviz"
    if urdf_profile == "v2" and gripper_profile == "f100":
        config_name = "pro450_new_structure_gripper.rviz"
    return os.path.join(
        get_package_share_directory("mycobot_pro_450"),
        "config",
        config_name,
    )


def declare_profile_launch_arguments():
    """Launch arguments shared by Pro 450 description consumers."""
    return [
        DeclareLaunchArgument(
            "urdf",
            default_value="legacy",
            description="Pro 450 structure profile: legacy or v2",
        ),
        DeclareLaunchArgument(
            "gripper",
            default_value="none",
            description="End-effector profile: none or f100",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="",
            description=(
                "Optional URDF path override; when set it takes precedence "
                "over urdf/gripper profile selection"
            ),
        ),
    ]
