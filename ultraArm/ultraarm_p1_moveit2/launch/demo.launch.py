from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="ultraarm_p1_moveit2").to_moveit_configs()
    demo_ld = generate_demo_launch(moveit_config)
    # Prepend colorized logging for WARN/ERROR under ros2 launch.
    return LaunchDescription(
        [SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")] + list(demo_ld.entities)
    )
