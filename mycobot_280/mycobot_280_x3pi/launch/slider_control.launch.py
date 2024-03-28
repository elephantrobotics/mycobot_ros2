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
            "urdf/mycobot_280_x3pi/mycobot_280_x3pi.urdf"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mycobot_280_x3pi"),
            "config/mycobot_x3pi.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    gui_launch_arg = DeclareLaunchArgument(
        "gui",
        default_value="true"
    )
    res.append(gui_launch_arg)
    
    serial_port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyS3',
        description='Serial port to use'
    )
    res.append(serial_port_arg)
    baud_rate_arg = DeclareLaunchArgument(
        'baud',
        default_value='1000000',
        description='Baud rate to use'
    )
    res.append(baud_rate_arg)

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
        package="mycobot_280_x3pi",
        executable="slider_control",
        # parameters=[
        #     {'port': LaunchConfiguration('port')},
        #     {'baud': LaunchConfiguration('baud')}
        # ],
        name="slider_control",
        output="screen"
    )
    res.append(slider_control_node)

    return LaunchDescription(res)
