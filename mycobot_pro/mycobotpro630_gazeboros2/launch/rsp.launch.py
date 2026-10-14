import os
import sys
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, os.path.join(get_package_prefix("mycobotpro630_gazeboros2"), "lib", "mycobotpro630_gazeboros2"))
from pro630_configuration import public_profile, project_moveit
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    with_gripper = public_profile()
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="mycobotpro630_gazeboros2").robot_description(mappings={"with_gripper": "true" if with_gripper else "false"}).to_moveit_configs()
    project_moveit(moveit_config, with_gripper)
    return generate_rsp_launch(moveit_config)

