import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    res = []

    model_launch_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/mypalletizer_260_pi/mypalletizer_260_pi.urdf"
        ),
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mypalletizer_260"),
            "config/mypalletizer.rviz"
        ),
    )
    res.append(rvizconfig_launch_arg)

    gui_launch_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
    )
    res.append(gui_launch_arg)

    num_launch_arg = DeclareLaunchArgument(
        "num",
        default_value="0",
    )
    res.append(num_launch_arg)
    
    # 发布数据
    follow_display_node = Node(
        name="follow_display",
        package="mypalletizer_260",
        executable="follow_display",
    )
    res.append(follow_display_node)

    mypalletizer_260_node = Node(
        name="opencv_camera",
        package="mypalletizer_260",
        executable="opencv_camera",
        arguments=[LaunchConfiguration("num")]
    )
    res.append(mypalletizer_260_node)

    mypalletizer_260_node = Node(
        name="detect_marker",
        package="mypalletizer_260",
        executable="detect_marker"
    )
    res.append(mypalletizer_260_node)

    mypalletizer_260_node = Node(
        name="following_marker",
        package="mypalletizer_260",
        executable="following_marker"
    )
    res.append(mypalletizer_260_node)

    return LaunchDescription(res)
