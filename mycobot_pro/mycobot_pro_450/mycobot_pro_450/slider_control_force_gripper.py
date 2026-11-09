# -*- coding: utf-8 -*-
"""Slider control for Pro450 + F100 force gripper (logical → firmware)."""

import math
import threading
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
    """Subscribe to joint states and control arm + force gripper."""

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

        self.command_lock = threading.Lock()
        self.angles_queue = None
        self.pending_gripper_value = None
        self.last_sent_angles = None
        self.last_sent_gripper_value = None

        threading.Thread(target=self._send_loop, daemon=True).start()

    def listener_callback(self, msg: JointState):
        if len(msg.position) < 6:
            return

        logical = [
            round(math.degrees(value), 2) for value in msg.position[:6]
        ]
        firmware = self.j5.convert_logical_angles_to_firmware(logical)
        if firmware is None:
            return

        gripper_command = None
        if "gripper_controller" in msg.name:
            gripper_index = msg.name.index("gripper_controller")
            if gripper_index < len(msg.position):
                gripper_position = msg.position[gripper_index]
                gripper_command = int(
                    round(max(0.0, min(1.0, gripper_position)) * 100)
                )

        angles_unchanged = self.last_sent_angles is not None and all(
            abs(current - previous) < 0.01
            for current, previous in zip(firmware, self.last_sent_angles)
        )
        gripper_unchanged = (
            gripper_command is None
            or gripper_command == self.last_sent_gripper_value
        )
        if angles_unchanged and gripper_unchanged:
            return

        if not angles_unchanged:
            self.last_sent_angles = list(firmware)
        if gripper_command is not None:
            self.last_sent_gripper_value = gripper_command

        self.get_logger().info(
            "joint_list: {}, gripper_value: {}".format(
                firmware,
                gripper_command if gripper_command is not None else "unchanged",
            )
        )
        with self.command_lock:
            if not angles_unchanged:
                self.angles_queue = firmware
            if gripper_command is not None:
                self.pending_gripper_value = gripper_command

    def _send_loop(self):
        while True:
            time.sleep(0.01)
            with self.command_lock:
                angles_to_send = self.angles_queue
                gripper_to_send = self.pending_gripper_value
                self.angles_queue = None
                self.pending_gripper_value = None
            # Prioritize gripper Modbus so arm stream cannot starve it.
            if gripper_to_send is not None:
                self.mycobot_450.set_pro_gripper_angle(gripper_to_send)
            if angles_to_send is not None:
                self.mycobot_450.send_angles(angles_to_send, 25)


def main(args=None):
    rclpy.init(args=args)
    node = Slider_Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
