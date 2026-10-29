"""Independent keyboard function; no Gazebo, MoveIt or slider process is spawned."""
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
        name='pro630_keyboard_session', output='screen', parameters=[{
            'feature': 'keyboard', 'environment': 'real',
            'startup_timeout_sec': ParameterValue(LaunchConfiguration('startup_timeout_sec'), value_type=float),
            'use_sim_time': False}])
    return LaunchDescription([
        DeclareLaunchArgument('startup_timeout_sec', default_value='600.0'),
        RegisterEventHandler(OnProcessExit(target_action=session,
            on_exit=[EmitEvent(event=Shutdown(reason='Keyboard function session ended'))])),
        session])
