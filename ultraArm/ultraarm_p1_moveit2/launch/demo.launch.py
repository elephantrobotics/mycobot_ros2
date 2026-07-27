import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Default MoveIt2 demo + mimic-expanded Goal/Path preview (parallel-link shells).

    Native MotionPlanning Goal/Path robot visuals are disabled in moveit_preview.rviz;
    MimicPreview RobotModel (frame_prefix=mimic_preview/) shows the correct linkage.
    Vanilla MoveIt (no preview) is available as demo_old.launch.py.
    """
    moveit_config = MoveItConfigsBuilder(
        "firefighter", package_name="ultraarm_p1_moveit2"
    ).to_moveit_configs()
    pkg_share = get_package_share_directory("ultraarm_p1_moveit2")
    preview_rviz = os.path.join(pkg_share, "config", "moveit_preview.rviz")

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "demo_old.launch.py")
                ),
                launch_arguments={"use_rviz": "false"}.items(),
            ),
            Node(
                package="ultraarm_p1_moveit2_control",
                executable="mimic_preview_node",
                name="mimic_preview_node",
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="mimic_preview_rsp",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    {"frame_prefix": "mimic_preview/"},
                ],
                remappings=[("joint_states", "/mimic_preview/joint_states")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="mimic_preview_world_tf",
                arguments=["0", "0", "0", "0", "0", "0", "world", "mimic_preview/world"],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "moveit_rviz.launch.py")
                ),
                launch_arguments={"rviz_config": preview_rviz}.items(),
            ),
        ]
    )
