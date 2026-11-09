# -*- coding: utf-8 -*-
"""Hold the MoveIt demo URDF/gripper profile for a later sync_plan process."""

import rclpy
from rclpy.node import Node


class MoveItProfile(Node):
    def __init__(self):
        super().__init__("pro450_moveit_profile")
        self.declare_parameter("urdf", "legacy")
        self.declare_parameter("gripper", "none")
        self.declare_parameter("joint5_offset_deg", 0.0)
        urdf_profile = self.get_parameter("urdf").get_parameter_value().string_value
        gripper_profile = (
            self.get_parameter("gripper").get_parameter_value().string_value
        )
        offset = (
            self.get_parameter("joint5_offset_deg").get_parameter_value().double_value
        )
        self.get_logger().info(
            "MoveIt profile: urdf=%s gripper=%s joint5_offset_deg=%.1f"
            % (urdf_profile, gripper_profile, offset)
        )


def main(args=None):
    rclpy.init(args=args)
    node = MoveItProfile()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
