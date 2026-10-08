#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MyCobot Pro 450 Follow Display (ROS 2)
Reads real robot joint angles and syncs them to Gazebo.
"""

import math
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from pymycobot import Pro450Client

PRO450_IP = "192.168.0.232"
PRO450_PORT = 4500
GRIPPER_ID = 14
SYNC_RATE = 20.0  # Hz

GRIPPER_MIN_ANGLE = 0
GRIPPER_MAX_ANGLE = 100
GAZEBO_MIN_POSITION = 0
GAZEBO_MAX_POSITION = 57.3

ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT_NAMES = ["gripper_controller"]

ANGLE_SCALE = [1.0, 0.85, 1.0, 1.35, 0.95, 1.0]
ANGLE_OFFSET = [0, 0, 0, 0, 0, 0]


class FollowDisplay(Node):
    def __init__(self):
        from rclpy.parameter import Parameter
        super().__init__('follow_display_gazebo',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub_arm = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.pub_gripper = self.create_publisher(
            JointTrajectory, '/pro_gripper_controller/joint_trajectory', 10)

        self.mc = None
        self.last_valid_angles = None
        self.last_mapped_gripper = None

        if self.initialize_pro450():
            self.timer = self.create_timer(1.0 / SYNC_RATE, self.sync_to_gazebo)
            self.get_logger().info("Ready for follow control.")

    def initialize_pro450(self):
        try:
            self.get_logger().info(
                "Connecting to Pro450 @ {}:{}...".format(PRO450_IP, PRO450_PORT))
            self.mc = Pro450Client(PRO450_IP, PRO450_PORT)
            time.sleep(1.0)
            self.mc.power_on()
            time.sleep(1.0)
            self.mc.set_servo_calibration(6)
            time.sleep(0.5)
            # init gripper
            self.mc.get_pro_gripper(1, GRIPPER_ID)
            # release servos to allow manual movement
            self.mc.release_all_servos()
            self.mc.set_pro_gripper_enabled(0, GRIPPER_ID)
            return True
        except Exception as e:
            self.get_logger().error("Pro450 init failed: {}".format(e))
            return False

    def is_valid_angles(self, angles):
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            return False
        return all(isinstance(a, (int, float)) and -180 <= a <= 180 for a in angles)

    def is_valid_gripper_angle(self, angle):
        if not isinstance(angle, (int, float)):
            return False
        return 0 <= angle <= 100

    def sync_to_gazebo(self):
        # Arm
        angles = None
        for _ in range(3):
            try:
                angles = self.mc.get_angles()
                if self.is_valid_angles(angles):
                    self.last_valid_angles = angles[:]
                    break
            except Exception:
                pass
            time.sleep(0.02)

        if not self.is_valid_angles(angles):
            if self.last_valid_angles is not None:
                angles = self.last_valid_angles
            else:
                return

        # Gripper
        gripper_angle = None
        valid_gripper = False
        for _ in range(3):
            try:
                raw_angle = self.mc.get_pro_gripper_angle(GRIPPER_ID)
                if self.is_valid_gripper_angle(raw_angle):
                    gripper_angle = raw_angle
                    valid_gripper = True
                    break
            except Exception:
                pass
            time.sleep(0.02)

        compensated = [angles[i] * ANGLE_SCALE[i] + ANGLE_OFFSET[i] for i in range(6)]

        arm_traj = JointTrajectory()
        arm_traj.header.stamp = self.get_clock().now().to_msg()
        arm_traj.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(a) for a in compensated]
        pt.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s
        arm_traj.points.append(pt)
        self.pub_arm.publish(arm_traj)

        if valid_gripper:
            mapped_gripper = ((gripper_angle - GRIPPER_MIN_ANGLE) /
                              (GRIPPER_MAX_ANGLE - GRIPPER_MIN_ANGLE)) * \
                             (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION) + GAZEBO_MIN_POSITION
            self.last_mapped_gripper = max(
                GAZEBO_MIN_POSITION, min(GAZEBO_MAX_POSITION, mapped_gripper))

        mapped = self.last_mapped_gripper if self.last_mapped_gripper else GAZEBO_MIN_POSITION

        g_traj = JointTrajectory()
        g_traj.header.stamp = self.get_clock().now().to_msg()
        g_traj.joint_names = GRIPPER_JOINT_NAMES
        gp = JointTrajectoryPoint()
        gp.positions = [math.radians(mapped)]
        gp.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s
        g_traj.points.append(gp)
        self.pub_gripper.publish(g_traj)


def main(args=None):
    rclpy.init(args=args)
    node = FollowDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.mc:
            try:
                node.mc.release_all_servos()
            except Exception:
                pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
