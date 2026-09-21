import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


CONTROLLER_MANAGER_TIMEOUT = "30"
DEFAULT_REAL_SNAPSHOT_FILE = "/tmp/mycobotpro450_real_initial_positions.yaml"


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


def continue_after_success(completed_name, next_actions):
    """Start next_actions only when the completed process exited successfully."""
    if not isinstance(next_actions, (list, tuple)):
        next_actions = [next_actions]

    def on_exit(event, _context):
        if event.returncode == 0:
            return list(next_actions)
        reason = (
            f"Failed to complete {completed_name}: process exited "
            f"with code {event.returncode}."
        )
        return [
            LogInfo(msg=f"ERROR: {reason}"),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    return on_exit


def build_simulation_stack(
    moveit_config,
    initial_positions_file,
    pause_gazebo,
    unpause_before_verify,
):
    """Build one isolated Pro450 Gazebo stack for the selected environment."""
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
                    " initial_positions_file:=",
                    initial_positions_file,
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
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )
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

    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"pause": pause_gazebo}.items(),
    )
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "/gazebo_spawn/robot_description",
            "-entity",
            "mycobotpro450",
        ],
        output="screen",
    )

    joint_state_spawner = controller_spawner("joint_state_broadcaster")
    arm_spawner = controller_spawner("arm_controller")
    gripper_spawner = controller_spawner("pro_gripper_controller")
    pose_verifier = Node(
        package="mycobotpro450_gazeboros2",
        executable="verify_initial_pose.py",
        name="pro450_initial_pose_verifier",
        output="screen",
        parameters=[
            {
                "expected_file": ParameterValue(
                    initial_positions_file, value_type=str
                ),
                "tolerance_rad": LaunchConfiguration("initial_pose_tolerance_rad"),
                "timeout_sec": LaunchConfiguration("initial_pose_timeout_sec"),
                "unpause_before_verify": unpause_before_verify,
            }
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
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
        package="mycobotpro450_gazeboros2",
        executable="pro450_slider_gui.py",
        name="pro450_slider_gui",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    start_arm_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=continue_after_success("joint_state_broadcaster", arm_spawner),
        )
    )
    start_gripper_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_spawner,
            on_exit=continue_after_success("arm_controller", gripper_spawner),
        )
    )
    start_verifier_after_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_spawner,
            on_exit=continue_after_success(
                "pro_gripper_controller", pose_verifier
            ),
        )
    )
    expose_tools_after_verification = RegisterEventHandler(
        OnProcessExit(
            target_action=pose_verifier,
            on_exit=continue_after_success(
                "Pro450 initial-pose verification",
                [move_group, rviz, slider_gui],
            ),
        )
    )

    return [
        rsp,
        gazebo_description_publisher,
        gazebo,
        spawn_entity,
        start_arm_after_joint_state,
        start_gripper_after_arm,
        start_verifier_after_gripper,
        expose_tools_after_verification,
        TimerAction(period=3.0, actions=[joint_state_spawner]),
    ]


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "firefighter", package_name="mycobotpro450_gazeboros2"
    ).to_moveit_configs()
    environment = LaunchConfiguration("environment")
    real_snapshot_file = LaunchConfiguration("real_snapshot_file")
    default_initial_positions = os.path.join(
        get_package_share_directory("mycobotpro450_gazeboros2"),
        "config",
        "initial_positions.yaml",
    )

    simulation_stack = build_simulation_stack(
        moveit_config, default_initial_positions, "false", False
    )
    real_stack = build_simulation_stack(
        moveit_config, real_snapshot_file, "true", True
    )

    pose_gate = Node(
        package="mycobotpro450_gazeboros2",
        executable="pro450_pose_gate.py",
        name="pro450_pose_gate",
        output="screen",
        parameters=[
            {
                "output_file": ParameterValue(real_snapshot_file, value_type=str),
                "timeout_sec": LaunchConfiguration("real_snapshot_timeout_sec"),
            }
        ],
        condition=IfCondition(PythonExpression(["'", environment, "' == 'real'"])),
    )
    start_real_stack_after_snapshot = RegisterEventHandler(
        OnProcessExit(
            target_action=pose_gate,
            on_exit=continue_after_success(
                "stable real-Pro450 pose acquisition", real_stack
            ),
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "environment",
                default_value="simulation",
                choices=["simulation", "real"],
                description="Pro450 environment: simulation or real-pose mirror.",
            ),
            DeclareLaunchArgument(
                "real_snapshot_file",
                default_value=DEFAULT_REAL_SNAPSHOT_FILE,
                description="Temporary Pro450 pose file generated from read-only feedback.",
            ),
            DeclareLaunchArgument(
                "real_snapshot_timeout_sec",
                default_value="600.0",
                description="Maximum wait for an external stable Pro450 snapshot.",
            ),
            DeclareLaunchArgument(
                "initial_pose_tolerance_rad",
                default_value="0.01",
                description="Maximum Gazebo-vs-snapshot startup joint error.",
            ),
            DeclareLaunchArgument(
                "initial_pose_timeout_sec",
                default_value="20.0",
                description="Maximum wait for matching Gazebo joint feedback.",
            ),
            GroupAction(
                actions=simulation_stack,
                condition=IfCondition(
                    PythonExpression(["'", environment, "' == 'simulation'"])
                ),
            ),
            pose_gate,
            start_real_stack_after_snapshot,
        ]
    )
