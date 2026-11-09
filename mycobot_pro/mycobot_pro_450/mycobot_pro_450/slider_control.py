# -*- coding: utf-8 -*-
"""Subscribe to JointState and send Pro450 arm angles (logical → firmware)."""

import math
import time

import pymycobot
import rclpy
from packaging import version
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mycobot_pro_450.joint5_support import Joint5ProfileGuard

MIN_REQUIRE_VERSION = "4.0.1"

current_verison = pymycobot.__version__
print("current pymycobot library version: {}".format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        "The version of pymycobot library must be greater than {} or higher. "
        "The current version is {}. Please upgrade the library version.".format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
print("pymycobot library version meets the requirements!")
from pymycobot import Pro450Client  # noqa: E402


class Slider_Subscriber(Node):
    """ROS2 node that subscribes to joint states and sends commands to Pro450."""

    def __init__(self):
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
            JointState, "joint_states", self.listener_callback, 10
        )

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
        time.sleep(0.05)
        if self.mycobot_450.is_power_on != 1:
            self.mycobot_450.power_on()
        time.sleep(0.05)
        if self.mycobot_450.get_fresh_mode() != 1:
            self.mycobot_450.set_fresh_mode(1)
        time.sleep(0.05)
        self.mycobot_450.set_limit_switch(2, 0)

        self.j5 = Joint5ProfileGuard(self.get_logger())
        self.j5.configure(self.mycobot_450, configured_offset)
        self.last_sent_angles = None

    def listener_callback(self, msg: JointState):
        if len(msg.position) < 6:
            return

        logical = []
        for value in msg.position[:6]:
            logical.append(round(math.degrees(value), 2))

        firmware = self.j5.convert_logical_angles_to_firmware(logical)
        if firmware is None:
            return

        if self.last_sent_angles is not None and all(
            abs(current - previous) < 0.01
            for current, previous in zip(firmware, self.last_sent_angles)
        ):
            return

        self.last_sent_angles = list(firmware)
        self.get_logger().info("joint_angles: {}".format(firmware))
        self.mycobot_450.send_angles(firmware, 25)


def main(args=None):
    rclpy.init(args=args)
    node = Slider_Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
