"""Real drag following; start alongside slider.launch.py environment:=real."""
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    session = Node(
        package='mycobotpro630_gazeboros2', executable='pro630_control_session.py',
        name='pro630_drag_session', output='screen', parameters=[{
            'feature': 'drag', 'environment': 'real',
            'startup_timeout_sec': 600.0, 'use_sim_time': False}])
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=session,
            on_exit=[EmitEvent(event=Shutdown(reason='Drag following session ended'))])),
        session])
