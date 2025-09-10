import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    res = []

    ip_launch_arg = DeclareLaunchArgument(
        name="ip",
        default_value="192.168.0.232",
        description='IP address used by the device'
    )
    res.append(ip_launch_arg)

    port_launch_arg = DeclareLaunchArgument(
        name="port",
        default_value="4500",
        description='Port number used by the device'
    )
    res.append(port_launch_arg)

    model_launch_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/mycobot_pro_450/mycobot_pro_450.urdf"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mycobot_pro_450"),
            "config/mycobot_pro_450.rviz"
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

    listen_real_service_node = Node(
        package="mycobot_pro_450",
        executable="listen_real_service",
        name="listen_real_service",
        output="screen",
        parameters=[{
        "ip": LaunchConfiguration("ip"),
        "port": LaunchConfiguration("port")
        }]
    )
    res.append(listen_real_service_node)

    mycobot_pro_450_node = Node(
        name="simple_gui",
        package="mycobot_pro_450",
        executable="simple_gui",
        parameters=[
            {'ip': LaunchConfiguration('ip')},
            {'port': LaunchConfiguration('port')}
        ],
        output="screen"
    )
    res.append(mycobot_pro_450_node)

    return LaunchDescription(res)
