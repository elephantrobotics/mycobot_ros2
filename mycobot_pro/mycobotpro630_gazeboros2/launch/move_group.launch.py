from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
import os


def generate_launch_description():
    # Humble's helper reads DISPLAY even when its debug terminal is disabled.
    os.environ.setdefault("DISPLAY", "")
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="mycobotpro630_gazeboros2").to_moveit_configs()
    return generate_move_group_launch(moveit_config)

