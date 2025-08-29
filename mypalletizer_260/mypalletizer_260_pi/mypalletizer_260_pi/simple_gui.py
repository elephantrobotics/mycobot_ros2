#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import fcntl
import os
import tkinter as tk
import time
import queue
import threading
from tkinter import messagebox
import pymycobot
from packaging import version
import rclpy
from rclpy.node import Node

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))

if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'Current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyPalletizer260
    from pymycobot.robot_info import RobotLimit


def acquire(lock_file):
    """Acquire a file lock to prevent concurrent access."""
    try:
        file_descriptor = os.open(
            lock_file, os.O_RDWR | os.O_CREAT | os.O_TRUNC)
    except OSError as erro_info:
        print(f"Failed to open lock file {lock_file}: {erro_info}")
        return None
    timeout = 50.0
    start_time = current_time = time.time()
    while current_time < start_time + timeout:
        try:
            fcntl.flock(file_descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return file_descriptor
        except:
            time.sleep(1)
            current_time = time.time()
    os.close(file_descriptor)
    return None


def release(fd):
    """Release a previously acquired file lock."""
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)
    except:
        pass


class WindowNode(Node):
    """ROS2 node for controlling MyCobot via a simple GUI window.

    This node integrates a Tkinter GUI with a MyCobot robotic arm.
    Users can input joint angles or coordinates, control the gripper,
    and see live updates of the arm's state. Robot commands are queued
    and executed in a background thread for thread-safe operation.

    Attributes:
        mc: Instance of the robotic arm controller (e.g., MyPalletizer260).
        win: Tkinter window handle.
        speed: Current speed for motion commands.
        speed_d: Tkinter StringVar for the speed entry.
        record_coords: Latest coordinates for display.
        res_angles: Latest joint angles for display.
        cmd_queue: Queue for sending commands to the robot.
        angles_min: Min joint angle limits.
        angles_max: Max joint angle limits.
        coords_min: Min coordinate limits.
        coords_max: Max coordinate limits.
    """

    def __init__(self, handle):
        """Initialize the WindowNode with a Tkinter window and robot connection.

        Args:
            handle: Tkinter window handle to attach GUI elements.
        """
        super().__init__('simple_gui')

        # Declare ROS2 parameters
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 1000000)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        print("port:%s, baud:%d" % (port, baud))
        self.mc = MyPalletizer260(port, baud)
        time.sleep(0.05)

        # Tkinter window setup
        self.win = handle
        self.win.resizable(0, 0)
        self.speed = 50
        self.speed_d = tk.StringVar()
        self.speed_d.set(str(self.speed))

        # Robotic arm data
        self.record_coords = [[0, 0, 0, 0], self.speed]
        self.res_angles = [[0, 0, 0, 0], self.speed]

        # Command queue and background worker
        self.cmd_queue = queue.Queue()
        threading.Thread(target=self.worker, daemon=True).start()

        # Initialize robot state
        self.get_date()

        # Screen dimensions
        self.ws = self.win.winfo_screenwidth()
        self.hs = self.win.winfo_screenheight()

        # Calculate window position
        x = (self.ws / 2) - 190
        y = (self.hs / 2) - 250
        self.win.geometry("440x350+{}+{}".format(int(x), int(y)))

        # Initialize GUI layout
        self.set_layout()
        self.need_input()
        self.show_init()

        # Add buttons
        tk.Button(self.frmLT, text="Set joints", width=8, command=self.get_joint_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )
        tk.Button(self.frmRT, text="Set coords", width=8, command=self.get_coord_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )
        tk.Button(self.frmLB, text="Open gripper", command=self.gripper_open, width=8).grid(
            row=1, column=0, sticky="w", padx=3, pady=20
        )
        tk.Button(self.frmLB, text="Close gripper", command=self.gripper_close, width=8).grid(
            row=1, column=1, sticky="w", padx=3, pady=2
        )

        # Periodic GUI update
        self.update_gui()

        # Robot model and limits
        self.robot_name = "MyPalletizer260"
        self.angles_min = RobotLimit.robot_limit[self.robot_name]["angles_min"]
        self.angles_max = RobotLimit.robot_limit[self.robot_name]["angles_max"]
        self.coords_min = RobotLimit.robot_limit[self.robot_name]["coords_min"]
        self.coords_max = RobotLimit.robot_limit[self.robot_name]["coords_max"]

    def worker(self):
        """Background thread that executes queued robot commands."""
        while rclpy.ok():
            try:
                cmd = self.cmd_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                if cmd[0] == "angles":
                    _, values, speed = cmd
                    lock = acquire("/tmp/mycobot_lock")
                    self.mc.send_angles(values, speed)
                    release(lock)

                elif cmd[0] == "coords":
                    _, values, speed = cmd
                    lock = acquire("/tmp/mycobot_lock")
                    self.mc.send_coords(values, speed)
                    release(lock)

                elif cmd[0] == "gripper":
                    _, state = cmd
                    lock = acquire("/tmp/mycobot_lock")
                    self.mc.set_gripper_state(state, 80, 1)
                    release(lock)

            except Exception as e:
                self.get_logger().warn(f"worker error: {e}")

    def gripper_open(self):
        """Queue command to open the gripper."""
        self.cmd_queue.put(("gripper", 0))

    def gripper_close(self):
        """Queue command to close the gripper."""
        self.cmd_queue.put(("gripper", 1))

    def show_error(self, msg):
        """Safely show an error message box in the main thread.

        Args:
            msg (str): The error message to display.
        """
        self.win.after(0, lambda: messagebox.showerror(
            "Error", msg, parent=self.win))

    def get_coord_input(self):
        """Read coordinates from GUI, validate, and queue command to the robot."""
        try:
            c_value = [float(i.get()) for i in self.all_c]
        except ValueError:
            self.show_error("Please enter a number for the coordinates")
            return

        for idx, val in enumerate(c_value):
            if not (self.coords_min[idx] <= val <= self.coords_max[idx]):
                self.show_error(
                    f"Coordinate {['X','Y','Z','RX'][idx]} input value is out of range "
                    f"{self.coords_min[idx]}~{self.coords_max[idx]}"
                )
                return

        try:
            speed = int(float(self.get_speed.get())
                        ) if self.get_speed.get() else self.speed
            if not (1 <= speed <= 100):
                self.show_error("Speed input value must be between 1 and 100")
                return
        except ValueError:
            self.show_error("Speed must be a number")
            return

        self.speed = speed
        self.cmd_queue.put(("coords", c_value, self.speed))

    def get_joint_input(self):
        """Read joint angles from GUI, validate, and queue command to the robot."""
        try:
            j_value = [float(i.get()) for i in self.all_j]
        except ValueError:
            self.get_logger().info('radians')
            self.show_error("Please enter a number for the joint angle")
            return

        for idx, val in enumerate(j_value):
            if not (self.angles_min[idx] <= val <= self.angles_max[idx]):
                self.show_error(
                    f"Joint {idx+1} input value is out of range "
                    f"{self.angles_min[idx]}~{self.angles_max[idx]}"
                )
                return

        try:
            speed = int(float(self.get_speed.get())
                        ) if self.get_speed.get() else self.speed
            if not (1 <= speed <= 100):
                self.show_error("Speed input value must be between 1 and 100")
                return
        except ValueError:
            self.show_error("Speed must be a number")
            return

        self.speed = speed
        self.cmd_queue.put(("angles", j_value, self.speed))

    def get_date(self):
        """Retrieve current coordinates and joint angles from the robotic arm.

        Queries the robotic arm up to 2 seconds for coordinates and angles,
        acquiring a lock for safe access. The results are stored internally
        for display or further processing.
        """
        t = time.time()
        while time.time() - t < 2:
            if self.mc:
                lock = acquire("/tmp/mycobot_lock")
                self.res = self.mc.get_coords()
                release(lock)
            if self.res != []:
                break
            time.sleep(0.1)

        t = time.time()
        while time.time() - t < 2:
            if self.mc:
                lock = acquire("/tmp/mycobot_lock")
                self.angles = self.mc.get_angles()
                release(lock)
            if self.angles != []:
                break
            time.sleep(0.1)

        self.record_coords[0] = self.res
        self.res_angles[0] = self.angles

    def set_layout(self):
        """Set the interface layout"""
        self.frmLT = tk.Frame(width=200, height=200)
        self.frmLC = tk.Frame(width=200, height=200)
        self.frmLB = tk.Frame(width=200, height=200)
        self.frmRT = tk.Frame(width=200, height=200)
        self.frmLT.grid(row=0, column=0, padx=1, pady=3)
        self.frmLC.grid(row=1, column=0, padx=1, pady=3)
        self.frmLB.grid(row=1, column=1, padx=2, pady=3)
        self.frmRT.grid(row=0, column=1, padx=2, pady=3)

    def need_input(self):
        """Display the input angle coordinate data of the robot arm"""
        tk.Label(self.frmLT, text="Joint 1 ").grid(row=0)
        tk.Label(self.frmLT, text="Joint 2 ").grid(row=1)
        tk.Label(self.frmLT, text="Joint 3 ").grid(row=2)
        tk.Label(self.frmLT, text="Joint 4 ").grid(row=3)

        tk.Label(self.frmRT, text="x").grid(row=0)
        tk.Label(self.frmRT, text="y").grid(row=1)
        tk.Label(self.frmRT, text="z").grid(row=2)
        tk.Label(self.frmRT, text="rx").grid(row=3)

        # Set the default value of the input box
        self.j1_default = tk.StringVar(value=str(self.res_angles[0][0]))
        self.j2_default = tk.StringVar(value=str(self.res_angles[0][1]))
        self.j3_default = tk.StringVar(value=str(self.res_angles[0][2]))
        self.j4_default = tk.StringVar(value=str(self.res_angles[0][3]))

        self.x_default = tk.StringVar(value=str(self.record_coords[0][0]))
        self.y_default = tk.StringVar(value=str(self.record_coords[0][1]))
        self.z_default = tk.StringVar(value=str(self.record_coords[0][2]))
        self.rx_default = tk.StringVar(value=str(self.record_coords[0][3]))

        # joint Input Box
        self.J_1 = tk.Entry(self.frmLT, textvariable=self.j1_default)
        self.J_2 = tk.Entry(self.frmLT, textvariable=self.j2_default)
        self.J_3 = tk.Entry(self.frmLT, textvariable=self.j3_default)
        self.J_4 = tk.Entry(self.frmLT, textvariable=self.j4_default)
        self.J_1.grid(row=0, column=1, pady=3)
        self.J_2.grid(row=1, column=1, pady=3)
        self.J_3.grid(row=2, column=1, pady=3)
        self.J_4.grid(row=3, column=1, pady=3)

        # coord Input Box
        self.x = tk.Entry(self.frmRT, textvariable=self.x_default)
        self.y = tk.Entry(self.frmRT, textvariable=self.y_default)
        self.z = tk.Entry(self.frmRT, textvariable=self.z_default)
        self.rx = tk.Entry(self.frmRT, textvariable=self.rx_default)
        self.x.grid(row=0, column=1, pady=3)
        self.y.grid(row=1, column=1, pady=3)
        self.z.grid(row=2, column=1, pady=3)
        self.rx.grid(row=3, column=1, pady=3)

        # All input boxes are used to get the input data
        self.all_j = [self.J_1, self.J_2, self.J_3, self.J_4]
        self.all_c = [self.x, self.y, self.z, self.rx]

        # Speed ​​input box
        tk.Label(self.frmLB, text="speed").grid(row=0, column=0)
        self.get_speed = tk.Entry(
            self.frmLB, textvariable=self.speed_d, width=10)
        self.get_speed.grid(row=0, column=1)

    def safe_get_angle(self, angle_list, index, default="-1°"):
        """Safely get an angle from a nested list.

        This method attempts to retrieve an angle value from `angle_list[0][index]`.
        If the list is empty, the index is out of range, or an error occurs,
        it returns a default value.

        Args:
            angle_list (list[list[float]]): Nested list of angles.
            index (int): Index of the angle to retrieve.
            default (str, optional): Default value to return if retrieval fails. Defaults to "-1°".

        Returns:
            str: The angle as a string with a degree symbol, or the default value.
        """
        try:
            if isinstance(angle_list[0], list) and len(angle_list[0]) > index:
                return "{}°".format(round(angle_list[0][index], 2))
        except Exception as e:
            self.get_logger().warn(f"safe_get_angle error: {e}")
        return default

    def safe_get_coord(self, coords_list, index, default="0.0"):
        """Safely get a coordinate from a nested list.

        This method attempts to retrieve a coordinate value from `coords_list[0][index]`.
        If the list is empty, the index is out of range, the value is -1, or an error occurs,
        it returns a default value.

        Args:
            coords_list (list[list[float]]): Nested list of coordinates.
            index (int): Index of the coordinate to retrieve.
            default (str, optional): Default value to return if retrieval fails. Defaults to "0.0".

        Returns:
            str: The coordinate as a string, or the default value.
        """
        try:
            if isinstance(coords_list[0], list) and len(coords_list[0]) > index:
                value = coords_list[0][index]
                if value != -1:
                    return str(round(value, 2))
        except Exception as e:
            self.get_logger().warn(f"safe_get_coord error: {e}")
        return default

    def show_init(self):
        """Display the robot arm angle coordinate data"""
        # display
        tk.Label(self.frmLC, text="Joint 1 ").grid(row=0)
        tk.Label(self.frmLC, text="Joint 2 ").grid(row=1)
        tk.Label(self.frmLC, text="Joint 3 ").grid(row=2)
        tk.Label(self.frmLC, text="Joint 4 ").grid(row=3)

        # Get data display
        self.cont_1 = tk.StringVar(self.frmLC)
        self.cont_1.set(self.safe_get_angle(self.res_angles, 0))
        self.cont_2 = tk.StringVar(self.frmLC)
        self.cont_2.set(self.safe_get_angle(self.res_angles, 1))
        self.cont_3 = tk.StringVar(self.frmLC)
        self.cont_3.set(self.safe_get_angle(self.res_angles, 2))
        self.cont_4 = tk.StringVar(self.frmLC)
        self.cont_4.set(self.safe_get_angle(self.res_angles, 3))

        self.show_j1 = tk.Label(self.frmLC, textvariable=self.cont_1, width=7,
                                height=1, bg="white").grid(row=0, column=1, padx=0, pady=5)

        self.show_j2 = tk.Label(self.frmLC, textvariable=self.cont_2, width=7,
                                height=1, bg="white",).grid(row=1, column=1, padx=0, pady=5)
        self.show_j3 = tk.Label(self.frmLC, textvariable=self.cont_3, width=7,
                                height=1, bg="white").grid(row=2, column=1, padx=0, pady=5)
        self.show_j4 = tk.Label(self.frmLC, textvariable=self.cont_4, width=7,
                                height=1, bg="white").grid(row=3, column=1, padx=0, pady=5)

        # coord display
        tk.Label(self.frmLC, text="  x ").grid(row=0, column=3)
        tk.Label(self.frmLC, text="  y ").grid(row=1, column=3)
        tk.Label(self.frmLC, text="  z ").grid(row=2, column=3)
        tk.Label(self.frmLC, text="  rx ").grid(row=3, column=3)

        self.coord_x = tk.StringVar()
        self.coord_x.set(self.safe_get_coord(self.record_coords, 0))
        self.coord_y = tk.StringVar()
        self.coord_y.set(self.safe_get_coord(self.record_coords, 1))
        self.coord_z = tk.StringVar()
        self.coord_z.set(self.safe_get_coord(self.record_coords, 2))
        self.coord_rx = tk.StringVar()
        self.coord_rx.set(self.safe_get_coord(self.record_coords, 3))

        self.show_x = tk.Label(self.frmLC, textvariable=self.coord_x, width=7,
                               height=1, bg="white").grid(row=0, column=4, padx=5, pady=5)
        self.show_y = tk.Label(self.frmLC, textvariable=self.coord_y, width=7,
                               height=1, bg="white").grid(row=1, column=4, padx=5, pady=5)
        self.show_z = tk.Label(self.frmLC, textvariable=self.coord_z, width=7,
                               height=1, bg="white").grid(row=2, column=4, padx=5, pady=5)
        self.show_rx = tk.Label(self.frmLC, textvariable=self.coord_rx, width=7,
                                height=1, bg="white").grid(row=3, column=4, padx=5, pady=5)
        # mm Unit Display
        self.unit = tk.StringVar(value="mm")
        for i in range(4):
            tk.Label(self.frmLC, textvariable=self.unit).grid(row=i, column=5)

    def update_gui(self):
        """Periodically refresh the GUI and update joint angles and coordinates.

        This method queries the current joint angles and coordinates from the
        robot, updates the Tkinter variables for display, and schedules itself
        to run again after 200 ms.
        """
        try:
            lock = acquire("/tmp/mycobot_lock")
            angles = self.mc.get_angles()
            coords = self.mc.get_coords()
            release(lock)

            if angles:
                self.res_angles = [angles]
                self.cont_1.set(self.safe_get_angle(self.res_angles, 0))
                self.cont_2.set(self.safe_get_angle(self.res_angles, 1))
                self.cont_3.set(self.safe_get_angle(self.res_angles, 2))
                self.cont_4.set(self.safe_get_angle(self.res_angles, 3))

            if coords:
                self.record_coords = [coords]
                self.coord_x.set(self.safe_get_coord(self.record_coords, 0))
                self.coord_y.set(self.safe_get_coord(self.record_coords, 1))
                self.coord_z.set(self.safe_get_coord(self.record_coords, 2))
                self.coord_rx.set(self.safe_get_coord(self.record_coords, 3))

        except Exception as e:
            self.get_logger().warn(f"update_gui error: {e}")

        # Schedule next update in 200 ms
        self.win.after(200, self.update_gui)


def main(args=None):
    """Entry point for the MyCobot ROS2 GUI node.

    Initializes ROS2, creates the Tkinter main window, instantiates
    the WindowNode to control the robot, and starts the Tkinter event loop.
    """
    # Initialize ROS2
    rclpy.init(args=args)

    # Create Tkinter main window
    window = tk.Tk()
    window.title("MyCobot ROS GUI")

    # Instantiate the WindowNode with the Tkinter handle
    node = WindowNode(window)

    # Start the Tkinter event loop
    window.mainloop()


if __name__ == "__main__":
    main()
