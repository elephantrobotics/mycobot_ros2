#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MyCobot Pro 630 Follow Display (ROS 2)
Reads real robot joint angles and syncs them to Gazebo.
"""

import math
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from pymycobot import ElephantRobot

PRO630_IP = "192.168.1.191"
PRO630_PORT = 5001
GRIPPER_ID = 14
SYNC_RATE = 10.0  # Hz (lower rate for stable TCP communication with Pro 630)

GRIPPER_MIN_ANGLE = 0
GRIPPER_MAX_ANGLE = 100
GAZEBO_MIN_POSITION = 0
GAZEBO_MAX_POSITION = 57.3

ARM_JOINT_NAMES = ["joint1_to_base", "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5"]
GRIPPER_JOINT_NAMES = ["gripper_controller"]

ANGLE_SCALE = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
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

        if self.initialize_pro630():
            self.timer = self.create_timer(1.0 / SYNC_RATE, self.sync_to_gazebo)
            self.get_logger().info("Ready for follow control.")

    def initialize_pro630(self):
        try:
            self.get_logger().info(f"Connecting to Pro630 @ {PRO630_IP}:{PRO630_PORT}...")
            self.mc = ElephantRobot(PRO630_IP, PRO630_PORT)
            self.mc.start_client()
            time.sleep(1.0)
            
            try:
                self.mc.set_servo_calibration(6)
                time.sleep(0.5)
                self.mc.get_pro_gripper(1, GRIPPER_ID)
                self.mc.release_all_servos()
                self.mc.set_pro_gripper_enabled(0, GRIPPER_ID)
            except Exception:
                pass
            return True
        except Exception as e:
            self.get_logger().error(f"Pro630 init failed: {e}")
            return False

    def is_valid_angles(self, angles):
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            return False
        return all(isinstance(a, (int, float)) and -360 <= a <= 360 for a in angles)

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

        # Apply joint compensation from Real Robot to Gazebo (J2+90, J4+90)
        compensated = [angles[i] * ANGLE_SCALE[i] + ANGLE_OFFSET[i] for i in range(6)]
        if len(compensated) > 3:
            compensated[1] += 90.0
            compensated[3] += 90.0

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
