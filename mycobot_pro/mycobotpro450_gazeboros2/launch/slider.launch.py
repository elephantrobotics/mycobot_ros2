import os

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


CONTROLLER_MANAGER_TIMEOUT = "30"


def controller_spawner(controller_name):
    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawner_{controller_name}",
        output="screen",
        arguments=[
            controller_name,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            CONTROLLER_MANAGER_TIMEOUT,
        ],
    )


def continue_after_success(completed_name, next_action):
    """Start next_action only when the completed spawner exited successfully."""

    def on_exit(event, _context):
        if event.returncode == 0:
            return [next_action]

        reason = (
            f"Failed to activate {completed_name}: controller spawner exited "
            f"with code {event.returncode}."
        )
        return [
            LogInfo(msg=f"ERROR: {reason}"),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    return on_exit

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="mycobotpro450_gazeboros2").to_moveit_configs()

    # MoveIt/FCL gets accurate triangle meshes, while Gazebo/ODE gets a
    # low-poly convex representation. Feeding the exact dynamic meshes to ODE
    # causes false contact impulses, folded startup poses, and gzserver crashes.
    gazebo_robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("mycobotpro450_gazeboros2"),
                            "config",
                            "firefighter.urdf.xacro",
                        ]
                    ),
                    " collision_mesh_dir:=collision_gazebo",
                ]
            ),
            value_type=str,
        )
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {'use_sim_time': True}],
    )

    # This publisher exists only to feed spawn_entity. Its TF streams are
    # isolated so it cannot duplicate or corrupt the normal MoveIt TF tree.
    gazebo_description_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="gazebo_spawn",
        name="robot_description_publisher",
        output="screen",
        parameters=[
            gazebo_robot_description,
            {"use_sim_time": True, "frame_prefix": "gazebo_spawn_unused/"},
        ],
        remappings=[
            ("/joint_states", "/gazebo_spawn/joint_states_unused"),
            ("/tf", "/gazebo_spawn/tf_unused"),
            ("/tf_static", "/gazebo_spawn/tf_static_unused"),
        ],
    )

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/gazebo_spawn/robot_description',
            '-entity', 'mycobotpro450',
        ],
        output='screen'
    )

    # controller_manager can time out when several spawners call its services at
    # the same time during Gazebo startup.  Bring the controllers up in a strict
    # order and do not expose the command GUI until every controller is active.
    joint_state_spawner = controller_spawner("joint_state_broadcaster")
    arm_spawner = controller_spawner("arm_controller")
    gripper_spawner = controller_spawner("pro_gripper_controller")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    slider_gui = Node(
        package='mycobotpro450_gazeboros2',
        executable='pro450_slider_gui.py',
        name='pro450_slider_gui',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    start_arm_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=continue_after_success(
                "joint_state_broadcaster", arm_spawner
            ),
        )
    )
    start_gripper_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_spawner,
            on_exit=continue_after_success("arm_controller", gripper_spawner),
        )
    )
    start_gui_after_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_spawner,
            on_exit=continue_after_success(
                "pro_gripper_controller", slider_gui
            ),
        )
    )

    return LaunchDescription([
        rsp,
        gazebo_description_publisher,
        gazebo,
        spawn_entity,
        move_group,
        rviz,
        start_arm_after_joint_state,
        start_gripper_after_arm,
        start_gui_after_gripper,
        TimerAction(period=3.0, actions=[joint_state_spawner]),
    ])

