#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MyCobot Pro 630 Teleop Keyboard Controller (ROS 2)
Controls Gazebo simulation and optionally real Pro630 arm.
"""

import math
import time
import sys
import select
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Try importing real robot client (optional)
try:
    from pymycobot import ElephantRobot
    HAS_PRO630 = True
except ImportError:
    HAS_PRO630 = False

PRO630_IP = "192.168.1.191"
PRO630_PORT = 5001

ARM_JOINTS = ["joint1_to_base", "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5"]
GRIPPER_JOINT = "gripper_controller"

JOINT_LIMITS = [
    (-360, 360), (-360, 360), (-360, 360),
    (-360, 360), (-360, 360), (-360, 360)
]

GRIPPER_ID = 14
GRIPPER_MIN_ANGLE = 0
GRIPPER_MAX_ANGLE = 100
GAZEBO_MIN_POSITION = 0
GAZEBO_MAX_POSITION = 57.3

ANGLE_STEP = 5.0
FAST_STEP = 15.0
ROBOT_SPEED = 50


class TeleopKeyboard(Node):
    def __init__(self):
        from rclpy.parameter import Parameter
        super().__init__('teleop_keyboard_gazebo',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub_arm = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.pub_gripper = self.create_publisher(
            JointTrajectory, '/pro_gripper_controller/joint_trajectory', 10)

        self.mc = None
        self.current_angles = [0, 0, 0, 0, 0, 0]
        self.current_gripper_angle = 50

        self.latest_arm_command = None
        self.latest_gripper_command = None
        self.command_lock = threading.Lock()
        self.command_event = threading.Event()
        self.executor_running = True

        self.initialize_pro630()

        self.executor_thread = threading.Thread(
            target=self.command_executor_thread, daemon=True)
        self.executor_thread.start()

        self.sync_arm(self.current_angles)
        self.sync_gripper(self.current_gripper_angle)

    def initialize_pro630(self):
        if not HAS_PRO630:
            self.get_logger().warn("pymycobot not found, Gazebo-only mode")
            return False
        try:
            self.get_logger().info(
                "Connecting to Pro630 @ {}:{}...".format(PRO630_IP, PRO630_PORT))
            self.mc = ElephantRobot(PRO630_IP, PRO630_PORT)
            self.mc.start_client()
            time.sleep(1.0)
            
            try:
                self.mc.set_servo_calibration(6)
                time.sleep(0.5)

                angles = self.mc.get_angles()
                if angles and len(angles) == 6:
                    # Apply reverse compensation: real robot -> Gazebo (J2+90, J4+90)
                    angles[1] += 90.0
                    angles[3] += 90.0
                    self.current_angles = angles[:]

                try:
                    grip_angle = self.mc.get_pro_gripper_angle(GRIPPER_ID)
                    if grip_angle is not None and 0 <= grip_angle <= 100:
                        self.current_gripper_angle = grip_angle
                except Exception:
                    pass

                self.get_logger().info("Pro630 connected OK")
            except Exception:
                pass
            return True
        except Exception as e:
            self.get_logger().error("Pro630 init failed: {}".format(e))
            self.mc = None
            return False

    def clamp_angles(self, angles):
        clamped = []
        for angle, (min_limit, max_limit) in zip(angles, JOINT_LIMITS):
            clamped.append(max(min_limit, min(max_limit, angle)))
        return clamped

    def publish_arm_to_gazebo(self, angles):
        try:
            arm_traj = JointTrajectory()
            arm_traj.header.stamp = self.get_clock().now().to_msg()
            arm_traj.joint_names = ARM_JOINTS
            pt = JointTrajectoryPoint()
            pt.positions = [math.radians(a) for a in angles]
            pt.time_from_start = Duration(sec=0, nanosec=500000000)
            arm_traj.points.append(pt)
            self.pub_arm.publish(arm_traj)
        except Exception:
            pass

    def publish_gripper_to_gazebo(self, gripper_angle):
        try:
            mapped = ((gripper_angle - GRIPPER_MIN_ANGLE) /
                      (GRIPPER_MAX_ANGLE - GRIPPER_MIN_ANGLE)) * \
                     (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION) + GAZEBO_MIN_POSITION
            mapped = max(GAZEBO_MIN_POSITION, min(GAZEBO_MAX_POSITION, mapped))

            gripper_traj = JointTrajectory()
            gripper_traj.header.stamp = self.get_clock().now().to_msg()
            gripper_traj.joint_names = [GRIPPER_JOINT]
            gp = JointTrajectoryPoint()
            gp.positions = [math.radians(mapped)]
            gp.time_from_start = Duration(sec=0, nanosec=500000000)
            gripper_traj.points.append(gp)
            self.pub_gripper.publish(gripper_traj)
        except Exception:
            pass

    def sync_arm(self, angles):
        with self.command_lock:
            self.latest_arm_command = angles[:]
            self.command_event.set()

    def sync_gripper(self, angle):
        with self.command_lock:
            self.latest_gripper_command = angle
            self.command_event.set()

    def command_executor_thread(self):
        while self.executor_running and rclpy.ok():
            if not self.command_event.wait(timeout=0.05):
                continue

            with self.command_lock:
                arm_cmd = self.latest_arm_command
                gripper_cmd = self.latest_gripper_command
                self.latest_arm_command = None
                self.latest_gripper_command = None
                self.command_event.clear()

            if arm_cmd is not None:
                self.publish_arm_to_gazebo(arm_cmd)
                if self.mc is not None:
                    try:
                        # Apply joint compensation for Pro 630 (J2-90, J4-90)
                        angles_to_send = arm_cmd[:]
                        if len(angles_to_send) > 3:
                            angles_to_send[1] -= 90.0
                            angles_to_send[3] -= 90.0
                        
                        # Use 630 specific write_angles (speed mapping: ROBOT_SPEED is 0-100, write_angles uses higher range usually)
                        self.mc.write_angles(angles_to_send, ROBOT_SPEED * 10)
                    except Exception:
                        pass

            if gripper_cmd is not None:
                self.publish_gripper_to_gazebo(gripper_cmd)
                if self.mc is not None:
                    try:
                        ga = max(0, min(100, int(gripper_cmd)))
                        self.mc.force_set_angle(GRIPPER_ID, ga)
                    except Exception:
                        pass

    def read_current_from_robot(self):
        if self.mc is None:
            print("[X] Robot not connected")
            return
        try:
            angles = self.mc.get_angles()
            if angles and len(angles) == 6:
                # Apply reverse compensation: real robot -> Gazebo (J2+90, J4+90)
                angles[1] += 90.0
                angles[3] += 90.0
                self.current_angles = angles[:]
                print("[>] Robot angles (compensated for Gazebo): {} deg".format(
                    [round(a, 1) for a in self.current_angles]))
            try:
                ga = self.mc.get_pro_gripper_angle(GRIPPER_ID)
                if ga is not None and 0 <= ga <= 100:
                    self.current_gripper_angle = ga
                    print("[>] Robot gripper: {} deg".format(
                        self.current_gripper_angle))
            except Exception:
                pass
            self.publish_arm_to_gazebo(self.current_angles)
            self.publish_gripper_to_gazebo(self.current_gripper_angle)
            print("[OK] Synced to Gazebo")
        except Exception:
            pass


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.prev = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.prev)


def get_key_non_blocking():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    print("=== MyCobot Pro 630 Teleop Keyboard (ROS 2) ===")
    print("w/s, e/d, r/f, t/g, y/h, u/j to control joints.")
    print("W/S, E/D, R/F, T/G, Y/H, U/J for fast control.")
    print("o/p for gripper fully open/close. [/] for step +-10.")
    print("1 to reset, 2 to show current, 3 to read from real robot, q to quit.")

    def key_loop():
        with RawTerminal():
            while rclpy.ok():
                key = get_key_non_blocking()
                if key is None:
                    time.sleep(0.01)
                    continue
                if key == 'q':
                    print("\nQuitting...")
                    node.executor_running = False
                    rclpy.shutdown()
                    break

                if key == '1':
                    node.current_angles = [0, 0, 0, 0, 0, 0]
                    node.sync_arm(node.current_angles)
                    print("[Reset] All joints to 0")
                    continue
                if key == '2':
                    print("\n[>] Angles: {}, Gripper: {}".format(
                        node.current_angles, node.current_gripper_angle))
                    continue
                if key == '3':
                    node.read_current_from_robot()
                    continue
                if key == 'o':
                    node.current_gripper_angle = 100
                    node.sync_gripper(node.current_gripper_angle)
                    continue
                elif key == 'p':
                    node.current_gripper_angle = 0
                    node.sync_gripper(node.current_gripper_angle)
                    continue
                elif key == '[':
                    node.current_gripper_angle = min(
                        100, node.current_gripper_angle + 10)
                    node.sync_gripper(node.current_gripper_angle)
                    continue
                elif key == ']':
                    node.current_gripper_angle = max(
                        0, node.current_gripper_angle - 10)
                    node.sync_gripper(node.current_gripper_angle)
                    continue

                normal_mapping = {
                    'w': (0, +ANGLE_STEP), 's': (0, -ANGLE_STEP),
                    'e': (1, +ANGLE_STEP), 'd': (1, -ANGLE_STEP),
                    'r': (2, +ANGLE_STEP), 'f': (2, -ANGLE_STEP),
                    't': (3, +ANGLE_STEP), 'g': (3, -ANGLE_STEP),
                    'y': (4, +ANGLE_STEP), 'h': (4, -ANGLE_STEP),
                    'u': (5, +ANGLE_STEP), 'j': (5, -ANGLE_STEP),
                }
                fast_mapping = {
                    'W': (0, +FAST_STEP), 'S': (0, -FAST_STEP),
                    'E': (1, +FAST_STEP), 'D': (1, -FAST_STEP),
                    'R': (2, +FAST_STEP), 'F': (2, -FAST_STEP),
                    'T': (3, +FAST_STEP), 'G': (3, -FAST_STEP),
                    'Y': (4, +FAST_STEP), 'H': (4, -FAST_STEP),
                    'U': (5, +FAST_STEP), 'J': (5, -FAST_STEP),
                }

                if key in normal_mapping:
                    idx, step = normal_mapping[key]
                    node.current_angles[idx] += step
                    node.current_angles = node.clamp_angles(
                        node.current_angles)
                    node.sync_arm(node.current_angles)
                    print("[>] Joint {}: {:.1f} deg".format(
                        idx + 1, node.current_angles[idx]))
                    continue
                if key in fast_mapping:
                    idx, step = fast_mapping[key]
                    node.current_angles[idx] += step
                    node.current_angles = node.clamp_angles(
                        node.current_angles)
                    node.sync_arm(node.current_angles)
                    print("[>>] Joint {}: {:.1f} deg".format(
                        idx + 1, node.current_angles[idx]))
                    continue

    t = threading.Thread(target=key_loop, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.executor_running = False
        if node.mc:
            try:
                node.mc.release_all_servos()
            except Exception:
                pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
