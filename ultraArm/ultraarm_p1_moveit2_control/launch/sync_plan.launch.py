from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0", description="Serial port used by ultraArm P1"),
        DeclareLaunchArgument("baud", default_value="1000000", description="Serial baudrate"),
        DeclareLaunchArgument("speed", default_value="25", description="Robot motion speed"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states", description="JointState topic from MoveIt2"),
        DeclareLaunchArgument("min_angle_delta", default_value="0.2", description="Minimum angle delta before sending"),
        DeclareLaunchArgument("connect_robot", default_value="true", description="Connect and send commands to the real robot"),
        Node(
            package="ultraarm_p1_moveit2_control",
            executable="sync_plan",
            name="sync_plan",
            output="screen",
            parameters=[
                {"port": LaunchConfiguration("port")},
                {"baud": ParameterValue(LaunchConfiguration("baud"), value_type=int)},
                {"speed": ParameterValue(LaunchConfiguration("speed"), value_type=int)},
                {"joint_states_topic": LaunchConfiguration("joint_states_topic")},
                {"min_angle_delta": ParameterValue(LaunchConfiguration("min_angle_delta"), value_type=float)},
                {"connect_robot": ParameterValue(LaunchConfiguration("connect_robot"), value_type=bool)},
            ],
        ),
    ])