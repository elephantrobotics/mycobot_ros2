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
            "urdf/mypalletizer_260_m5/mypalletizer_260_m5.urdf"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mypalletizer_260"),
            "config/mypalletizer.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    num_launch_arg = DeclareLaunchArgument(
        name="num",
        default_value="0",
    )
    res.append(num_launch_arg)

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                LaunchConfiguration('model')
            ]
        ),
        value_type=str
    )

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        arguments=[LaunchConfiguration("model")]
    )
    res.append(robot_state_publisher_node)
    
    # 发布数据
    follow_display_node = Node(
        name="follow_display",
        package="mecharm",
        executable="follow_display",
    )
    res.append(follow_display_node)

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        # output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )
    res.append(rviz_node)
    
    real_listener_node = Node(
        name="listen_real_of_topic",
        package="mecharm",
        executable="listen_real_of_topic"
    )
    res.append(real_listener_node)

    # opencv_camera_node = Node(
    #     name="opencv_camera",
    #     package="mycobot_280",
    #     executable="opencv_camera",
    #     output="screen",
    #     arguments=[LaunchConfiguration("num")]
    # )
    # res.append(opencv_camera_node)
    
    # detect_marker_node = Node(
    #     name="detect_marker",
    #     package="mycobot_280",
    #     executable="detect_marker",
    # )
    # res.append(detect_marker_node)
    
    
    

    return LaunchDescription(res)
