"""Independent slider/random-target function; requires the public environment."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    session = Node(
        package='mycobotpro630_gazeboros2', executable='pro630_control_session.py',
        name='pro630_slider_session', output='screen', parameters=[{
            'feature': 'slider', 'environment': LaunchConfiguration('environment'),
            'show_force_execute': ParameterValue(LaunchConfiguration('show_force_execute'), value_type=bool),
            'startup_timeout_sec': ParameterValue(LaunchConfiguration('startup_timeout_sec'), value_type=float),
            'use_sim_time': False}])
    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value='simulation', choices=['simulation', 'real']),
        DeclareLaunchArgument('startup_timeout_sec', default_value='600.0'),
        DeclareLaunchArgument('show_force_execute', default_value='false', choices=['false', 'true'],
                             description='Show the simulation-only Force Execute test button.'),
        RegisterEventHandler(OnProcessExit(target_action=session,
            on_exit=[EmitEvent(event=Shutdown(reason='Slider function session ended'))])),
        session])
