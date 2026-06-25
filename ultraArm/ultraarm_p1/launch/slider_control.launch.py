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
    
    speed_launch_arg = DeclareLaunchArgument(
        name="speed",
        default_value="25",
        description='Robot motion speed'
    )
    res.append(speed_launch_arg)

    command_rate_launch_arg = DeclareLaunchArgument(
        name="command_rate",
        default_value="5.0",
        description='Max rate for sending real robot commands'
    )
    res.append(command_rate_launch_arg)

    queue_limit_launch_arg = DeclareLaunchArgument(
        name="queue_limit",
        default_value="0",
        description='Queue limit used when stop-before-send is disabled'
    )
    res.append(queue_limit_launch_arg)

    min_angle_delta_launch_arg = DeclareLaunchArgument(
        name="min_angle_delta",
        default_value="0.2",
        description='Minimum angle delta before sending a new target'
    )
    res.append(min_angle_delta_launch_arg)

    use_stop_before_send_launch_arg = DeclareLaunchArgument(
        name="use_stop_before_send",
        default_value="true",
        description='Clear firmware queue with stop() when the queue is too deep'
    )
    res.append(use_stop_before_send_launch_arg)

    stop_queue_threshold_launch_arg = DeclareLaunchArgument(
        name="stop_queue_threshold",
        default_value="10",
        description='Call stop() before sending when queue size is above this value'
    )
    res.append(stop_queue_threshold_launch_arg)

    stop_settle_time_launch_arg = DeclareLaunchArgument(
        name="stop_settle_time",
        default_value="0.02",
        description='Delay after stop() before sending the latest target'
    )
    res.append(stop_settle_time_launch_arg)

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

    gui_launch_arg = DeclareLaunchArgument(
        "gui",
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
    )
    res.append(robot_state_publisher_node)

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        # remappings=[
        #     ('/joint_states', '/joint_states_raw')
        # ]
    )
    res.append(joint_state_publisher_gui_node)

    joint_coupling_node = Node(
        package='ultraarm_p1', 
        executable='joint_coupling_node',
        output='screen'
    )
    # res.append(joint_coupling_node)
    
    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)
    
    slider_control_node = Node(
        package="ultraarm_p1",
        executable="slider_control",
        name="slider_control",
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int)},
            {'speed': ParameterValue(LaunchConfiguration('speed'), value_type=int)},
            {'command_rate': ParameterValue(LaunchConfiguration('command_rate'), value_type=float)},
            {'queue_limit': ParameterValue(LaunchConfiguration('queue_limit'), value_type=int)},
            {'min_angle_delta': ParameterValue(LaunchConfiguration('min_angle_delta'), value_type=float)},
            {'use_stop_before_send': ParameterValue(LaunchConfiguration('use_stop_before_send'), value_type=bool)},
            {'stop_queue_threshold': ParameterValue(LaunchConfiguration('stop_queue_threshold'), value_type=int)},
            {'stop_settle_time': ParameterValue(LaunchConfiguration('stop_settle_time'), value_type=float)}
        ],
        output="screen"
    )
    res.append(slider_control_node)

    return LaunchDescription(res)
