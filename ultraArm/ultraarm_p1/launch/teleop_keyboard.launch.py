import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    res = []

    # Enable ANSI colors under ros2 launch (WARN=yellow, ERROR=red).
    res.append(SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"))

    model_launch_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/ultraArm_p1/ultraArm_p1.urdf"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("ultraarm_p1"),
            "config/ultraarm_p1.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)
    
    port_launch_arg = DeclareLaunchArgument(
        name="port",
        default_value="/dev/ttyUSB0",
        description='Port used by the device'
    )
    res.append(port_launch_arg)

    baud_launch_arg = DeclareLaunchArgument(
        name="baud",
        default_value="1000000",
        description='baud number used by the device'
    )
    res.append(baud_launch_arg)

    publish_rate_launch_arg = DeclareLaunchArgument(
        name="publish_rate",
        default_value="10.0",
        description='JointState publish rate (Hz) for RViz model follow'
    )
    res.append(publish_rate_launch_arg)
    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
        arguments=[LaunchConfiguration("model")]
    )
    res.append(robot_state_publisher_node)

    listen_real_service_node = Node(
        package="ultraarm_p1",
        executable="listen_real_service",
        name="listen_real_service",
        output="screen",
        parameters=[{
        "port": LaunchConfiguration("port"),
        "baud": ParameterValue(LaunchConfiguration("baud"), value_type=int),
        "publish_rate": ParameterValue(LaunchConfiguration("publish_rate"), value_type=float),
        }]
    )
    res.append(listen_real_service_node)
    
    rviz_node = Node(
    name="rviz2",
    package="rviz2",
    executable="rviz2",
    output="screen",
    arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)

    return LaunchDescription(res)
