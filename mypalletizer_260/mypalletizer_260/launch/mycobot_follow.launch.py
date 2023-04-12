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
        "model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/260_urdf/mypal_260.urdf"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mypalletizer_260"),
            "config/mypalletizer.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description}],
        arguments=[LaunchConfiguration("model")]
    )
    res.append(robot_state_publisher_node)

    follow_display_node = Node(
        package="mypalletizer_260",
        executable="follow_display",
        name="follow_display",
        output="screen"
    )
    res.append(follow_display_node)

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)

    return LaunchDescription(res)
