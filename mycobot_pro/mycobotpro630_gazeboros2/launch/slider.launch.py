"""Shared Pro630 environment: Gazebo, MoveIt and real-pose tracking."""
import os
import logging
import re
import uuid

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
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
DEFAULT_REAL_SNAPSHOT_FILE = "/tmp/mycobotpro630_real_initial_positions.yaml"


class GazeboMulticastLogFilter(logging.Filter):
    """Hide selected repeated multicast diagnostics; preserve other output."""
    message = re.compile(
        r"(?:Exception sending a multicast message:\s*"
        r"(?:No such device|没有那个设备|没有这个设备|Network is unreachable|网络不可达)\s*)+"
    )

    def filter(self, record):
        text = re.sub(r"^\[gz(?:server|client)-\d+\]\s*", "", record.getMessage())
        return self.message.fullmatch(text.strip()) is None


def filter_gazebo_multicast_logs(event, context):
    name = event.process_name
    if re.fullmatch(r"gz(?:server|client)-\d+", name):
        for stream in ("stdout", "stderr"):
            logger = logging.getLogger(f"{name}-{stream}")
            if not any(isinstance(f, GazeboMulticastLogFilter) for f in logger.filters):
                logger.addFilter(GazeboMulticastLogFilter())
    return []


CONTROLLERS = [
    "arm_controller",
    "joint_state_broadcaster",
    "pro_gripper_controller",
]


def use_current_planning_start(moveit_config):
    """Resolve RViz's cached query start before all existing request adapters."""
    adapter = "pro630_gazebo/UseCurrentStartState"
    for pipeline in moveit_config.planning_pipelines["planning_pipelines"]:
        config = moveit_config.planning_pipelines[pipeline]
        existing = config.get("request_adapters", "").split()
        config["request_adapters"] = " ".join(
            [adapter] + [name for name in existing if name != adapter])
    return moveit_config


def controller_spawner(*, inactive=False):
    arguments = [
        *CONTROLLERS,
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        CONTROLLER_MANAGER_TIMEOUT,
    ]
    if inactive:
        arguments.append("--inactive")
    else:
        arguments.append("--activate-as-group")
    return Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_pro630_controllers",
        output="screen",
        arguments=arguments,
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
    coordinated_start,
    control_mode,
    session_id="",
):
    """Build one isolated Pro630 Gazebo stack for the selected environment."""
    gazebo_robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("mycobotpro630_gazeboros2"),
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
        launch_arguments={"pause": pause_gazebo, "gui": LaunchConfiguration("gazebo_gui")}.items(),
    )
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "/gazebo_spawn/robot_description",
            "-entity",
            "mycobotpro630",
        ],
        output="screen",
    )

    controllers = controller_spawner(inactive=coordinated_start)
    controller_activator = Node(
        package="mycobotpro630_gazeboros2",
        executable="activate_gazebo_controllers.py",
        name="pro630_controller_activation_coordinator",
        output="screen",
        parameters=[{"controllers": CONTROLLERS}],
    )
    pose_verifier = Node(
        package="mycobotpro630_gazeboros2",
        executable="verify_initial_pose.py",
        name="pro630_initial_pose_verifier",
        output="screen",
        parameters=[
            {
                "expected_file": ParameterValue(
                    initial_positions_file, value_type=str
                ),
                "tolerance_rad": ParameterValue(LaunchConfiguration("initial_pose_tolerance_rad"), value_type=float),
                "timeout_sec": ParameterValue(LaunchConfiguration("initial_pose_timeout_sec"), value_type=float),
                "unpause_before_verify": False,
            }
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )
    environment_ready = Node(
        package="mycobotpro630_gazeboros2",
        executable="pro630_environment_state.py",
        name="pro630_environment_ready",
        arguments=["--announce-ready"],
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "environment": control_mode,
            "session_id": session_id,
        }],
    )

    if coordinated_start:
        after_controller_setup = RegisterEventHandler(
            OnProcessExit(
                target_action=controllers,
                on_exit=continue_after_success(
                    "inactive Pro630 controller configuration",
                    controller_activator,
                ),
            )
        )
        after_controller_activation = RegisterEventHandler(
            OnProcessExit(
                target_action=controller_activator,
                on_exit=continue_after_success(
                    "coordinated Pro630 controller activation",
                    pose_verifier,
                ),
            )
        )
        expose_tools = RegisterEventHandler(
            OnProcessExit(
                target_action=pose_verifier,
                on_exit=continue_after_success(
                    "Pro630 initial-pose verification",
                    [move_group, environment_ready],
                ),
            )
        )
        lifecycle_handlers = [
            after_controller_setup,
            after_controller_activation,
            expose_tools,
        ]
    else:
        # Normal simulation is intentionally not guarded by the real-pose
        # fail-closed verifier.  The single spawner waits for Gazebo's
        # controller manager and activates every controller in one update.
        expose_tools = RegisterEventHandler(
            OnProcessExit(
                target_action=controllers,
                on_exit=continue_after_success(
                    "grouped Pro630 controller activation",
                    [move_group, environment_ready],
                ),
            )
        )
        lifecycle_handlers = [expose_tools]

    return [
        RegisterEventHandler(OnProcessExit(target_action=move_group,
            on_exit=[EmitEvent(event=Shutdown(reason='MoveIt planning and collision service ended'))])),
        RegisterEventHandler(OnProcessStart(on_start=filter_gazebo_multicast_logs)),
        Node(package='mycobotpro630_gazeboros2', executable='pro630_collision_scene.py',
             output='screen', parameters=[{'use_sim_time': False}]),
        rsp,
        gazebo_description_publisher,
        gazebo,
        spawn_entity,
        *lifecycle_handlers,
        controllers,
    ]


def generate_launch_description():
    session_id = uuid.uuid4().hex
    moveit_config = use_current_planning_start(MoveItConfigsBuilder(
        "firefighter", package_name="mycobotpro630_gazeboros2"
    ).to_moveit_configs())
    environment = LaunchConfiguration("environment")
    real_snapshot_file = LaunchConfiguration("real_snapshot_file")
    default_initial_positions = os.path.join(
        get_package_share_directory("mycobotpro630_gazeboros2"),
        "config",
        "initial_positions.yaml",
    )

    simulation_stack = build_simulation_stack(
        moveit_config, default_initial_positions, "false", False,
        "simulation", session_id,
    )
    real_stack = build_simulation_stack(
        moveit_config, real_snapshot_file, "true", True,
        "real", session_id,
    )

    pose_gate = Node(
        package="mycobotpro630_gazeboros2",
        executable="pro630_pose_gate.py",
        name="pro630_pose_gate",
        output="screen",
        parameters=[
            {
                "output_file": ParameterValue(real_snapshot_file, value_type=str),
                "timeout_sec": ParameterValue(LaunchConfiguration("real_snapshot_timeout_sec"), value_type=float),
            }
        ],
        condition=IfCondition(PythonExpression(["'", environment, "' == 'real'"])),
    )
    robot_bridge = Node(
        package='mycobotpro630_gazeboros2', executable='pro630_robot_bridge.py',
        output='screen', parameters=[{'use_sim_time': False,
            'pro630_ip': ParameterValue(LaunchConfiguration('pro630_ip'), value_type=str),
            'pro630_port': ParameterValue(LaunchConfiguration('pro630_port'), value_type=int)}],
        condition=IfCondition(PythonExpression(["'", environment, "' == 'real'"])),
    )
    start_real_stack_after_snapshot = RegisterEventHandler(
        OnProcessExit(
            target_action=pose_gate,
            on_exit=continue_after_success(
                "stable real-Pro630 pose acquisition", real_stack
            ),
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "environment",
                default_value="simulation",
                choices=["simulation", "real"],
                description="Pro630 environment: simulation or real-pose mirror.",
            ),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("pro630_ip", default_value="192.168.1.191"),
            DeclareLaunchArgument("pro630_port", default_value="5001"),
            DeclareLaunchArgument(
                "real_snapshot_file",
                default_value=DEFAULT_REAL_SNAPSHOT_FILE,
                description="Temporary Pro630 pose file generated from read-only feedback.",
            ),
            DeclareLaunchArgument(
                "real_snapshot_timeout_sec",
                default_value="600.0",
                description="Maximum wait for an external stable Pro630 snapshot.",
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
            Node(
                package="mycobotpro630_gazeboros2",
                executable="pro630_environment_state.py",
                name="pro630_environment_state",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "environment": environment,
                    "session_id": session_id,
                }],
            ),
            pose_gate,
            RegisterEventHandler(OnProcessExit(target_action=robot_bridge,
                on_exit=[EmitEvent(event=Shutdown(reason='Public robot bridge ended'))])),
            robot_bridge,
            start_real_stack_after_snapshot,
        ]
    )
