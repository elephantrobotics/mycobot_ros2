import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    res = []

    model_launch_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/mycobot_pro_630/mycobot_pro_630.urdf"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mycobot_630"),
            "config/mycobot_630.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    gui_launch_arg = DeclareLaunchArgument(
        "gui",
        default_value="true"
    )
    res.append(gui_launch_arg)
    
    serial_ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='192.168.1.159',
        description='ip to use'
    )
    res.append(serial_ip_arg)
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='5001',
        description='port to use'
    )
    res.append(port_arg)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
    res.append(robot_state_publisher_node)

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    res.append(joint_state_publisher_gui_node)

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)
    
    slider_control_node = Node(
        package="mycobot_630",
        executable="slider_control",
        parameters=[
            {'ip': LaunchConfiguration('ip')},
            {'port': LaunchConfiguration('port')}
        ],
        name="slider_control",
        output="screen"
    )
    res.append(slider_control_node)

    return LaunchDescription(res)
