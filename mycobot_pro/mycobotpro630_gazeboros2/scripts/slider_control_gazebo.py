#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import threading
import queue
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from pymycobot import ElephantRobot

PRO630_IP = "192.168.1.191"
PRO630_PORT = 5001

JOINT_LIMITS = [
    (-360, 360), (-360, 360), (-360, 360),
    (-360, 360), (-360, 360), (-360, 360)
]
GRIPPER_LIMITS = (0, 57.3)
GRIPPER_ID = 14                 

GAZEBO_MIN_POSITION = 0 
GAZEBO_MAX_POSITION = 57.3
PRO630_GRIPPER_MIN = 0      
PRO630_GRIPPER_MAX = 100    

ANGLE_THRESHOLD = 3.0           
GRIPPER_THRESHOLD = 5.0         
MAX_COMMAND_RATE = 10.0         

ARM_JOINTS = ["joint1_to_base", "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5"]
GRIPPER_JOINT = "gripper_controller"

class RobotCommand:
    def __init__(self, cmd_type, data):
        self.type = cmd_type
        self.data = data

def map_gripper_angle_to_pro630(gazebo_angle):
    gazebo_angle = max(GAZEBO_MIN_POSITION, min(GAZEBO_MAX_POSITION, gazebo_angle))
    mapped_angle = ((gazebo_angle - GAZEBO_MIN_POSITION) / (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION)) * (PRO630_GRIPPER_MAX - PRO630_GRIPPER_MIN) + PRO630_GRIPPER_MIN
    return int(round(max(PRO630_GRIPPER_MIN, min(PRO630_GRIPPER_MAX, mapped_angle))))

def estimate_end_effector_height(j2, j3, j4):
    j2_rad = math.radians(j2)
    j3_rad = math.radians(j3)
    j4_rad = math.radians(j4)
    
    L1 = 0.048    
    L2 = 0.18     
    L3 = 0.1735   
    L4 = 0.08     
    L5 = 0.17  
    angle2 = j2_rad
    angle3 = angle2 + j3_rad
    angle4 = angle3 + j4_rad

    height = 0.155 + L1 
    height += L2 * math.cos(angle2) 
    height += L3 * math.cos(angle3) 
    height += L4 * math.cos(angle4) 
    height += L5 * math.cos(angle4) 
    return height

class SliderControl(Node):
    def __init__(self, mode):
        from rclpy.parameter import Parameter
        if mode == 1:
            super().__init__('slider_control_gazebo', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        else:
            super().__init__('slider_control_gazebo')
        self.mode = mode
        
        self.mc = None
        self.last_angles = None
        self.last_gripper_angle = None
        self.last_command_time = 0
        self.command_queue = queue.Queue(maxsize=5)
        self.is_stopped = False
        self.stop_lock = threading.Lock()
        self.current_end_effector_coords = None
        self.coords_lock = threading.Lock()
        
        self.pub_arm = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 1)
        self.pub_gripper = self.create_publisher(JointTrajectory, '/pro_gripper_controller/joint_trajectory', 1)

        if self.mode == 1:
            self.get_logger().info("Mode 1: Gazebo Sim Only")

        elif self.mode == 2:
            self.get_logger().info("Mode 2: Real Robot + Gazebo Sync")
            if not self.initialize_pro630():
                self.get_logger().error("Init failed")
                return
            
            threading.Thread(target=self.command_executor, daemon=True).start()
            threading.Thread(target=self.monitor_height, daemon=True).start()

        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1)
        self.create_subscription(Point, '/pro630/end_effector_coords', self.coords_cb, 1)

    def initialize_pro630(self):
        try:
            self.mc = ElephantRobot(PRO630_IP, PRO630_PORT)
            self.mc.start_client()
            time.sleep(1.0)
            
            try:
                self.mc.set_servo_calibration(6)
                time.sleep(0.5)
                self.get_logger().info(f"Connected, angles: {self.mc.get_angles()}")
                self.mc.get_pro_gripper(1, GRIPPER_ID)
                self.mc.release_all_servos()
            except Exception:
                pass
            time.sleep(0.5)
            return True
        except Exception as e:
            self.get_logger().error(str(e))
            return False

    def coords_cb(self, msg):
        with self.coords_lock:
            self.current_end_effector_coords = msg

    def joint_states_cb(self, msg):
        arm_deg = [0.0] * 6
        grip_deg = 0.0
        name_to_deg = {name: math.degrees(pos) for name, pos in zip(msg.name, msg.position)}
        
        for i, joint_name in enumerate(ARM_JOINTS):
            if joint_name in name_to_deg:
                arm_deg[i] = round(name_to_deg[joint_name], 1)
        if GRIPPER_JOINT in name_to_deg:
            grip_deg = round(name_to_deg[GRIPPER_JOINT], 1)

        for i, (angle, (min_limit, max_limit)) in enumerate(zip(arm_deg, JOINT_LIMITS)):
            arm_deg[i] = max(min_limit, min(max_limit, angle))
        grip_deg = max(GRIPPER_LIMITS[0], min(GRIPPER_LIMITS[1], grip_deg))

        current_time = time.time()
        if current_time - self.last_command_time < 1.0 / MAX_COMMAND_RATE:
            return
            
        angle_diff = float('inf')
        if self.last_angles:
            angle_diff = sum(abs(a - b) for a, b in zip(arm_deg, self.last_angles))
        gripper_diff = abs(grip_deg - self.last_gripper_angle) if self.last_gripper_angle is not None else float('inf')
        
        if angle_diff < ANGLE_THRESHOLD and gripper_diff < GRIPPER_THRESHOLD:
            return

        # Always publish to Gazebo to keep simulator in sync with command source (like sliders)
        # This prevents contention on /joint_states from multiple sources
        try:
            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = ARM_JOINTS
            pt = JointTrajectoryPoint()
            pt.positions = [math.radians(d) for d in arm_deg]
            pt.time_from_start = Duration(sec=0, nanosec=200000000)
            traj.points = [pt]
            self.pub_arm.publish(traj)
            
            traj_g = JointTrajectory()
            traj_g.header.stamp = self.get_clock().now().to_msg()
            traj_g.joint_names = [GRIPPER_JOINT]
            ptg = JointTrajectoryPoint()
            ptg.positions = [math.radians(grip_deg)]
            ptg.time_from_start = Duration(sec=0, nanosec=200000000)
            traj_g.points = [ptg]
            self.pub_gripper.publish(traj_g)
            
            self.last_angles = arm_deg.copy()
            self.last_gripper_angle = grip_deg
            self.last_command_time = time.time()
        except Exception as e:
            self.get_logger().debug(f"Gazebo publish error: {e}")

        if self.mode == 2:
            try:
                self.command_queue.put_nowait(RobotCommand('angles', arm_deg))
                if gripper_diff >= GRIPPER_THRESHOLD:
                    self.command_queue.put_nowait(RobotCommand('gripper', grip_deg))
            except queue.Full:
                try:
                    self.command_queue.get_nowait()
                    self.command_queue.put_nowait(RobotCommand('angles', arm_deg))
                except: pass

    def command_executor(self):
        while rclpy.ok():
            try:
                try: first_cmd = self.command_queue.get(timeout=0.1)
                except queue.Empty: continue
                
                latest_angles_cmd = first_cmd if first_cmd.type == 'angles' else None
                latest_gripper_cmd = first_cmd if first_cmd.type == 'gripper' else None
                
                while True:
                    try:
                        cmd = self.command_queue.get_nowait()
                        if cmd.type == 'angles': latest_angles_cmd = cmd
                        elif cmd.type == 'gripper': latest_gripper_cmd = cmd
                    except queue.Empty: break

                if not self.mc: continue

                if latest_angles_cmd:
                    j2, j3, j4 = latest_angles_cmd.data[1], latest_angles_cmd.data[2], latest_angles_cmd.data[3]
                    target_height_m = estimate_end_effector_height(j2, j3, j4)
                    target_height = target_height_m * 1000
                    MIN_SAFE_HEIGHT = 170

                    current_height = None
                    with self.coords_lock:
                        if self.current_end_effector_coords:
                            current_height = self.current_end_effector_coords.z

                    if target_height < MIN_SAFE_HEIGHT:
                        continue 
                    else:
                        if current_height is not None and current_height < MIN_SAFE_HEIGHT:
                            with self.stop_lock:
                                self.is_stopped = False

                    try:
                        # Apply joint compensation for Pro 630 (J2-90, J4-90)
                        angles_to_send = latest_angles_cmd.data[:]
                        if len(angles_to_send) > 3:
                            angles_to_send[1] -= 90.0
                            angles_to_send[3] -= 90.0
                        
                        self.mc.write_angles(angles_to_send, 1000)
                        self.last_angles = latest_angles_cmd.data.copy()
                    except: pass
                
                if latest_gripper_cmd:
                    ga = latest_gripper_cmd.data
                    ma = map_gripper_angle_to_pro630(ga)
                    try: self.mc.force_set_angle(GRIPPER_ID, ma)
                    except: pass
                    self.last_gripper_angle = ga
                
                self.last_command_time = time.time()
            except: pass

    def monitor_height(self):
        MIN_SAFE_HEIGHT = 170
        RECOVERY_HEIGHT = 170
        
        while rclpy.ok():
            try:
                with self.coords_lock:
                    if self.current_end_effector_coords:
                        end_height = self.current_end_effector_coords.z
                        if end_height < MIN_SAFE_HEIGHT:
                            with self.stop_lock:
                                self.is_stopped = True
                            try: self.mc.stop()
                            except: pass
                        elif end_height >= RECOVERY_HEIGHT:
                            with self.stop_lock:
                                self.is_stopped = False
            except: pass
            time.sleep(0.05)


def main(args=None):
    print("Select Mode:")
    print("1: Gazebo Sim")
    print("2: Real Robot")
    mode_str = input("Enter 1 or 2 (default 2): ").strip()
    mode = 1 if mode_str == "1" else 2

    rclpy.init(args=args)
    node = SliderControl(mode)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.mc:
            try: node.mc.release_all_servos()
            except: pass
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
