#!/usr/bin/env python3

"""MoveIt2 JointState bridge for the real ultraArm P1.

This node mirrors the ROS1 moveit sync_plan.py behavior:
subscribe to JointState, convert radians to ultraArm degrees, reject unsafe
J2/J3 combinations, and send valid targets to the robot.
"""

import math

import pymycobot
import rclpy
from packaging import version
from rclpy.node import Node
from sensor_msgs.msg import JointState


MIN_REQUIRE_VERSION = "4.0.5"
J2_RANGE = (-18.0, 85.0)
J3_RANGE = (-1.0, 110.0)
ZERO_EPS_DEG = 0.1
INVALID_WARN_DELTA_DEG = 0.5
DEFAULT_SEND_ANGLE_DELTA_DEG = 0.2


def snap_zero(angle_deg):
    return 0.0 if abs(angle_deg) < ZERO_EPS_DEG else angle_deg


def valid_region(j2_deg, j3_deg):
    """Return whether the J2/J3 combination is mechanically reachable."""

    if not (J2_RANGE[0] <= j2_deg <= J2_RANGE[1] and J3_RANGE[0] <= j3_deg <= J3_RANGE[1]):
        return False

    if -18 <= j2_deg < 0:
        # J2 < 0: reject J3 >= 42° (same hard cut as joint_coupling_node).
        if j3_deg >= 42.0:
            return False
        cond1 = math.cos(math.radians(-j2_deg + j3_deg)) - math.sin(math.radians(45 + j2_deg)) <= 7 / 30
        cond2 = abs(math.cos(math.radians(-j2_deg + j3_deg))) >= 15.4 / 30
        return cond1 and cond2

    if 0 <= j2_deg <= 50.87:
        return math.cos(math.radians(j2_deg - j3_deg)) >= 15.4 / 30

    if 50.87 < j2_deg < 76.72:
        return True

    if 76.72 <= j2_deg <= 85:
        return abs(math.cos(math.radians(j2_deg - j3_deg))) >= 6.89 / 30

    return False


class SyncPlan(Node):
    def __init__(self):
        super().__init__("sync_plan")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 1000000)
        self.declare_parameter("speed", 25)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("min_angle_delta", DEFAULT_SEND_ANGLE_DELTA_DEG)
        self.declare_parameter("connect_robot", True)

        self.speed = int(self.get_parameter("speed").value)
        self.min_angle_delta = float(self.get_parameter("min_angle_delta").value)
        self.connect_robot = bool(self.get_parameter("connect_robot").value)
        self.last_invalid_pair = None
        self.last_sent_angles = None
        self.was_invalid = False
        self.ua = None

        self.check_pymycobot_version()
        if self.connect_robot:
            self.connect_ultraarm()
        else:
            self.get_logger().warn("connect_robot is false; valid targets will be logged but not sent")

        topic = self.get_parameter("joint_states_topic").value
        self.sub = self.create_subscription(JointState, topic, self.callback, 10)
        self.get_logger().info("sync_plan started, listening on %s" % topic)

    def check_pymycobot_version(self):
        current_version = pymycobot.__version__
        self.get_logger().info("Current pymycobot library version: %s" % current_version)
        if version.parse(current_version) < version.parse(MIN_REQUIRE_VERSION):
            raise RuntimeError(
                "The pymycobot library must be %s or higher. Current version is %s."
                % (MIN_REQUIRE_VERSION, current_version)
            )
        self.get_logger().info("pymycobot library version meets the requirements")

    def connect_ultraarm(self):
        from pymycobot import UltraArmP1

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        self.get_logger().info("Connecting ultraArm P1 on %s, baud %d" % (port, baud))
        self.ua = UltraArmP1(port, baud)
        self.ua.set_joint_enable(0)

    def joint_angle_deg(self, msg, joint_name):
        try:
            index = msg.name.index(joint_name)
        except ValueError:
            raise KeyError(joint_name)
        return round(snap_zero(math.degrees(msg.position[index])), 2)

    def warn_invalid_pair(self, joint2, joint3):
        should_warn = not self.was_invalid
        if self.last_invalid_pair is not None:
            should_warn = should_warn or abs(joint2 - self.last_invalid_pair[0]) >= INVALID_WARN_DELTA_DEG
            should_warn = should_warn or abs(joint3 - self.last_invalid_pair[1]) >= INVALID_WARN_DELTA_DEG
        else:
            should_warn = True

        if should_warn:
            self.get_logger().warn(
                "Rejected unsafe J2/J3 combination from MoveIt2: J2=%.2f deg, J3=%.2f deg"
                % (joint2, joint3)
            )
            self.last_invalid_pair = (joint2, joint3)
        self.was_invalid = True

    def angles_changed(self, angles):
        if self.last_sent_angles is None:
            return True
        return any(abs(current - previous) >= self.min_angle_delta for current, previous in zip(angles, self.last_sent_angles))

    def callback(self, msg):
        try:
            joint1 = self.joint_angle_deg(msg, "J1")
            joint2 = self.joint_angle_deg(msg, "J2")
            joint3 = self.joint_angle_deg(msg, "J3")
            joint4 = self.joint_angle_deg(msg, "J4")
        except (KeyError, IndexError):
            self.get_logger().warn("JointState missing one of J1/J2/J3/J4; message ignored")
            return

        if not valid_region(joint2, joint3):
            self.warn_invalid_pair(joint2, joint3)
            return

        self.was_invalid = False
        self.last_invalid_pair = None
        angles = [round(angle, 2) for angle in [joint1, joint2, joint3 + 90.0, joint4]]

        if not self.angles_changed(angles):
            return

        self.last_sent_angles = list(angles)
        self.get_logger().info("send angles: %s" % angles)
        if self.connect_robot:
            self.ua.set_angles(angles, self.speed, _async=False)


def main(args=None):
    rclpy.init(args=args)
    node = SyncPlan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
