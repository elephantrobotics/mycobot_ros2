#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sync MoveIt /joint_states to a Pro 450.

Reads urdf/gripper from the running demo node ``pro450_moveit_profile`` when
present, otherwise from local parameters (default legacy/none). J5 uses
Joint5ProfileGuard. F100 maps gripper_controller 0..1 to 0..100.
"""

import math
import threading
import time

import pymycobot
import rclpy
from packaging import version
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mycobot_pro_450.joint5_support import Joint5ProfileGuard

MIN_REQUIRE_VERSION = "4.0.1"
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT = "gripper_controller"
PROFILE_NODE = "pro450_moveit_profile"
SUPPORTED = {("legacy", "none"), ("v2", "none"), ("v2", "f100")}

current_version = pymycobot.__version__
print("current pymycobot library version: {}".format(current_version))
if version.parse(current_version) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        "The version of pymycobot library must be greater than {} or higher. "
        "The current version is {}. Please upgrade the library version.".format(
            MIN_REQUIRE_VERSION, current_version
        )
    )
print("pymycobot library version meets the requirements!")
from pymycobot import Pro450Client  # noqa: E402


def _read_profile_parameters(node):
    """Return (urdf, gripper, offset) from the demo profile node, or None."""
    client = node.create_client(
        GetParameters, "/{}/get_parameters".format(PROFILE_NODE)
    )
    if not client.wait_for_service(timeout_sec=2.0):
        return None
    request = GetParameters.Request()
    request.names = ["urdf", "gripper", "joint5_offset_deg"]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done() or future.result() is None:
        return None
    values = future.result().values
    if len(values) < 3:
        return None
    urdf_profile = values[0].string_value or "legacy"
    gripper_profile = values[1].string_value or "none"
    offset = float(values[2].double_value)
    return urdf_profile, gripper_profile, offset


class SliderSubscriber(Node):
    def __init__(self):
        super().__init__("control_sync_plan")
        self.declare_parameter("ip", "192.168.0.232")
        self.declare_parameter("port", 4500)
        self.declare_parameter("urdf", "legacy")
        self.declare_parameter("gripper", "none")

        remote = _read_profile_parameters(self)
        if remote is not None:
            urdf_profile, gripper_profile, configured_offset = remote
            self.get_logger().info(
                "Using MoveIt profile from /{}: urdf={} gripper={} offset={:.1f}".format(
                    PROFILE_NODE, urdf_profile, gripper_profile, configured_offset
                )
            )
        else:
            urdf_profile = self.get_parameter("urdf").get_parameter_value().string_value
            gripper_profile = (
                self.get_parameter("gripper").get_parameter_value().string_value
            )
            urdf_profile = (urdf_profile or "legacy").strip().lower()
            gripper_profile = (gripper_profile or "none").strip().lower()
            configured_offset = 90.0 if urdf_profile == "v2" else 0.0
            self.get_logger().warn(
                "No /{} profile node; using local urdf={} gripper={} offset={:.1f}. "
                "Start demo.launch.py first to avoid passing parameters.".format(
                    PROFILE_NODE, urdf_profile, gripper_profile, configured_offset
                )
            )

        urdf_profile = urdf_profile.strip().lower()
        gripper_profile = gripper_profile.strip().lower()
        if (urdf_profile, gripper_profile) not in SUPPORTED:
            raise RuntimeError(
                "Unsupported MoveIt profile: urdf={} gripper={}. "
                "Supported values: legacy/none, v2/none, v2/f100".format(
                    urdf_profile, gripper_profile
                )
            )

        self.enable_force_gripper = gripper_profile == "f100"
        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value

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

        self.create_subscription(JointState, "joint_states", self.listener_callback, 10)
        threading.Thread(target=self._send_loop, daemon=True).start()

    def listener_callback(self, msg: JointState):
        name_to_position = {
            name: msg.position[index]
            for index, name in enumerate(msg.name)
            if index < len(msg.position)
        }
        if any(joint not in name_to_position for joint in ARM_JOINTS):
            return

        logical = [
            round(math.degrees(name_to_position[joint]), 2) for joint in ARM_JOINTS
        ]
        firmware = self.j5.convert_logical_angles_to_firmware(logical)
        if firmware is None:
            return

        gripper_command = None
        if self.enable_force_gripper and GRIPPER_JOINT in name_to_position:
            gripper_position = name_to_position[GRIPPER_JOINT]
            gripper_command = int(round(max(0.0, min(1.0, gripper_position)) * 100))

        angles_unchanged = self.last_sent_angles is not None and all(
            abs(current - previous) < 0.01
            for current, previous in zip(firmware, self.last_sent_angles)
        )
        gripper_unchanged = (
            gripper_command is None or gripper_command == self.last_sent_gripper_value
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
            if gripper_command is not None and not gripper_unchanged:
                self.pending_gripper_value = gripper_command

    def _send_loop(self):
        while rclpy.ok():
            time.sleep(0.01)
            with self.command_lock:
                angles_to_send = self.angles_queue
                gripper_to_send = self.pending_gripper_value
                self.angles_queue = None
                self.pending_gripper_value = None
            if gripper_to_send is not None:
                self.mycobot_450.set_pro_gripper_angle(gripper_to_send)
            if angles_to_send is not None:
                self.mycobot_450.send_angles(angles_to_send, 25)


def main(args=None):
    rclpy.init(args=args)
    node = SliderSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
