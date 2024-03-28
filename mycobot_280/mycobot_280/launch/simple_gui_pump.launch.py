import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    res = []

    port_launch_arg = DeclareLaunchArgument(
        name="port",
        default_value="/dev/ttyUSB0"
    )
    res.append(port_launch_arg)

    baud_launch_arg = DeclareLaunchArgument(
        name="baud",
        default_value="115200"
    )
    res.append(baud_launch_arg)

    model_launch_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/mycobot_280_m5/mycobot_280_m5_with_pump.urdf"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mycobot_280"),
            "config/mycobot.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    gui_launch_arg = DeclareLaunchArgument(
        name="gui",
        default_value="false"
    )
    res.append(gui_launch_arg)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
    res.append(robot_state_publisher_node)

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)

    follow_display_node = Node(
        package="mycobot_280",
        executable="follow_display",
        name="follow_display",
        output="screen"
    )
    res.append(follow_display_node)

    mycobot_280_node = Node(
        name="simple_gui",
        package="mycobot_280",
        executable="simple_gui",
    )
    res.append(mycobot_280_node)

    return LaunchDescription(res)
