# -*- coding: utf-8 -*-
"""Publish real Pro450 joint states to RViz (firmware → logical)."""

import math
import time

import pymycobot
import rclpy
from packaging import version
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from mycobot_pro_450.joint5_support import Joint5ProfileGuard

MIN_REQUIRE_VERSION = "4.0.1"

current_verison = pymycobot.__version__
print("current pymycobot library version: {}".format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        "The version of pymycobot library must be greater than {} or higher. "
        "Current version is {}. Please upgrade the library version.".format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
print("pymycobot library version meets the requirements!")
from pymycobot import Pro450Client  # noqa: E402


class Talker(Node):
    """Publish joint states and end-effector marker while dragging the arm."""

    def __init__(self):
        super().__init__("follow_display")
        self.declare_parameter("ip", "192.168.0.232")
        self.declare_parameter("port", 4500)
        self.declare_parameter("joint5_offset_deg", 0.0)

        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        configured_offset = (
            self.get_parameter("joint5_offset_deg").get_parameter_value().double_value
        )

        self.get_logger().info("ip:%s, port:%d" % (ip, port))
        self.mycobot_450 = Pro450Client(ip, port)
        if self.mycobot_450.is_power_on != 1:
            self.mycobot_450.power_on()
        time.sleep(0.05)
        self.mycobot_450.set_free_move_mode(1)
        time.sleep(0.05)

        self.j5 = Joint5ProfileGuard(self.get_logger())
        self.j5.configure(self.mycobot_450, configured_offset)

        self.get_logger().info(
            "Please press the button at the end of the machine to drag the joint.\n"
            "请按下机器末端按钮进行关节拖拽运动"
        )

    def start(self):
        pub = self.create_publisher(JointState, "joint_states", 10)
        pub_marker = self.create_publisher(Marker, "visualization_marker", 10)
        rate = self.create_rate(30)

        joint_state_send = JointState()
        joint_state_send.header = Header()
        joint_state_send.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        joint_state_send.velocity = [0.0]
        joint_state_send.effort = []

        marker_ = Marker()
        marker_.header.frame_id = "base"
        marker_.ns = "my_namespace"

        self.get_logger().info("Publishing ...")
        while rclpy.ok():
            rclpy.spin_once(self)
            joint_state_send.header.stamp = self.get_clock().now().to_msg()
            try:
                angles = self.mycobot_450.get_angles()
                if isinstance(angles, list) and len(angles) >= 6:
                    logical = self.j5.convert_firmware_angles_to_logical(angles[:6])
                    if logical is None:
                        rate.sleep()
                        continue
                    joint_state_send.position = [
                        math.radians(value) for value in logical
                    ]
                    pub.publish(joint_state_send)
                else:
                    self.get_logger().warn(
                        "Failed to get valid angles: {}".format(angles)
                    )

                coords = self.mycobot_450.get_coords()
                if not isinstance(coords, list) or len(coords) == 0 or coords == -1:
                    self.get_logger().warn(
                        "Failed to get valid coordinates: {}".format(coords)
                    )
                    coords = [0, 0, 0, 0, 0, 0]

                marker_.header.stamp = self.get_clock().now().to_msg()
                marker_.type = marker_.SPHERE
                marker_.action = marker_.ADD
                marker_.scale.x = 0.04
                marker_.scale.y = 0.04
                marker_.scale.z = 0.04
                marker_.pose.position.x = coords[1] / 1000 * -1
                marker_.pose.position.y = coords[0] / 1000
                marker_.pose.position.z = coords[2] / 1000
                marker_.color.a = 1.0
                marker_.color.g = 1.0
                pub_marker.publish(marker_)
                rate.sleep()
            except Exception as exc:
                print(exc)


def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    talker.start()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
