from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    """Vanilla MoveIt2 demo (no MimicPreview). Used by demo.launch.py with use_rviz:=false."""
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="ultraarm_p1_moveit2").to_moveit_configs()
    demo_ld = generate_demo_launch(moveit_config)
    return LaunchDescription(
        [SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")] + list(demo_ld.entities)
    )
