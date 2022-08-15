import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    res = []

    port_launch_arg = DeclareLaunchArgument(
        name="port",
        default_value="/dev/ttyAMA0"
        # default_value="192.168.123.62"
        
    )
    res.append(port_launch_arg)

    baud_launch_arg = DeclareLaunchArgument(
        name="baud",
        default_value="1000000"
        # default_value="9000"
        
    )
    res.append(baud_launch_arg)

    model_launch_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/260_pi/mypal_260_pi.urdf"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mypalletizer_260_pi"),
            "config/mypalletizer_pi.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    gui_launch_arg = DeclareLaunchArgument(
        name="gui",
        # default_value="false"
        default_value="true"
        
    )
    res.append(gui_launch_arg)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
        # package='robot_state_publisher',
        # executable='robot_state_publisher',
        # name="robot_state_publisher",
        # output="screen",
        # parameters=[{'robot_description': robot_description}],
        # arguments=[LaunchConfiguration("model")]
    )
    res.append(robot_state_publisher_node)

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', LaunchConfiguration("rvizconfig")]
    )
    res.append(rviz_node)

    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )
    # res.append(joint_state_publisher_gui_node)
    simple_gui_node = Node(
        package="mypalletizer_260_pi",
        namespace="simple_gui_node",
        name="simple_gui",
        executable="simple_gui",
        output="screen"
    )
    res.append(simple_gui_node)
    
    follow_display_node = Node(
        package="mypalletizer_260_pi",
        executable="follow_display",
        name="follow_display",
        output="screen"
    )
    res.append(follow_display_node)

    # service_node = Node(
    #     package="mypalletizer_communication",
    #     namespace="service_node",
    #     name="service",
    #     executable="mypalletizer_services",
    #     output="screen",
    #     parameters=[{
    #         "port": LaunchConfiguration("port"),
    #         "baud": LaunchConfiguration("baud"),
    #     }]
    # )
    # res.append(service_node)
    
    
    
    return LaunchDescription(res)
