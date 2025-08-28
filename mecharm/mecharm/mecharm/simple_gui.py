#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import tkinter as tk
import time
import os
import fcntl
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
    from pymycobot import MechArm270


def acquire(lock_file):
    """Acquire a file lock to prevent concurrent access.

    Args:
        lock_file (str): Path to the lock file.

    Returns:
        int | None: File descriptor if lock acquired, None if failed.
    """
    try:
        file_descriptor = os.open(lock_file, os.O_RDWR | os.O_CREAT | os.O_TRUNC)
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
    """Release a previously acquired file lock.

    Args:
        fd (int): File descriptor of the lock file.
    """
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)
    except:
        pass



class WindowNode(Node):
    """ROS2 node for controlling MyCobot via a simple GUI window.

    This class initializes the robot connection, sets up Tkinter GUI layout,
    and provides buttons to control joints, coordinates, gripper, and suction pump.
    """

    def __init__(self, handle):
        """Initialize the WindowNode.

        Args:
            handle (tk.Tk): The root Tkinter window handle.
        """
        super().__init__('simple_gui')

        # Declare ROS2 parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        print("port:%s, baud:%d" % (port, baud))
        self.mc = MechArm270(port, baud)
        time.sleep(0.05)

        # Ensure robot is in fresh mode
        if self.mc:
            lock = acquire("/tmp/mycobot_lock")
            if self.mc.get_fresh_mode() != 1:
                self.mc.set_fresh_mode(1)
            release(lock)
        time.sleep(0.05)

        # Tkinter window setup
        self.win = handle
        self.win.resizable(0, 0)  # Fixed window size

        self.model = 1
        self.speed = 50

        # Default speed variable
        self.speed_d = tk.StringVar()
        self.speed_d.set(str(self.speed))

        # Robotic arm data
        self.record_coords = [
            [0, 0, 0, 0, 0, 0],
            self.speed,
            self.model
        ]
        self.res_angles = [
            [0, 0, 0, 0, 0, 0],
            self.speed,
            self.model
        ]
        self.get_date()  # Initialize data from the robot

        # Screen dimensions
        self.ws = self.win.winfo_screenwidth()
        self.hs = self.win.winfo_screenheight()

        # Calculate window position
        x = (self.ws / 2) - 190
        y = (self.hs / 2) - 250
        self.win.geometry("440x440+{}+{}".format(int(x), int(y)))

        # GUI layout and widgets
        self.set_layout()
        self.need_input()
        self.show_init()

        # Buttons for joint settings
        tk.Button(self.frmLT, text="设置", width=5, command=self.get_joint_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )

        # Buttons for coordinate settings
        tk.Button(self.frmRT, text="设置", width=5, command=self.get_coord_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )

        # Gripper control buttons
        tk.Button(self.frmLB, text="夹爪(开)", command=self.gripper_open, width=5).grid(
            row=1, column=0, sticky="w", padx=3, pady=20
        )
        tk.Button(self.frmLB, text="夹爪(关)", command=self.gripper_close, width=5).grid(
            row=1, column=1, sticky="w", padx=3, pady=2
        )

        # Suction pump control buttons
        tk.Button(self.frmLB, text=" 吸泵(开)", command=self.pump_open, width=5).grid(
            row=2, column=0, sticky="w", padx=3, pady=20
        )
        tk.Button(self.frmLB, text="吸泵(关)", command=self.pump_close, width=5).grid(
            row=2, column=1, sticky="w", padx=3, pady=2
        )

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
        # Input prompt
        tk.Label(self.frmLT, text="Joint 1 ").grid(row=0)
        tk.Label(self.frmLT, text="Joint 2 ").grid(row=1)
        tk.Label(self.frmLT, text="Joint 3 ").grid(row=2)
        tk.Label(self.frmLT, text="Joint 4 ").grid(row=3)
        tk.Label(self.frmLT, text="Joint 5 ").grid(row=4)
        tk.Label(self.frmLT, text="Joint 6 ").grid(row=5)

        tk.Label(self.frmRT, text=" x ").grid(row=0)
        tk.Label(self.frmRT, text=" y ").grid(row=1)
        tk.Label(self.frmRT, text=" z ").grid(row=2)
        tk.Label(self.frmRT, text=" rx ").grid(row=3)
        tk.Label(self.frmRT, text=" ry ").grid(row=4)
        tk.Label(self.frmRT, text=" rz ").grid(row=5)

        # Set the default value of the input box
        self.j1_default = tk.StringVar()
        self.j1_default.set(self.res_angles[0][0])
        self.j2_default = tk.StringVar()
        self.j2_default.set(self.res_angles[0][1])
        self.j3_default = tk.StringVar()
        self.j3_default.set(self.res_angles[0][2])
        self.j4_default = tk.StringVar()
        self.j4_default.set(self.res_angles[0][3])
        self.j5_default = tk.StringVar()
        self.j5_default.set(self.res_angles[0][4])
        self.j6_default = tk.StringVar()
        self.j6_default.set(self.res_angles[0][5])

        self.x_default = tk.StringVar()
        self.x_default.set(self.record_coords[0][0])
        self.y_default = tk.StringVar()
        self.y_default.set(self.record_coords[0][1])
        self.z_default = tk.StringVar()
        self.z_default.set(self.record_coords[0][2])
        self.rx_default = tk.StringVar()
        self.rx_default.set(self.record_coords[0][3])
        self.ry_default = tk.StringVar()
        self.ry_default.set(self.record_coords[0][4])
        self.rz_default = tk.StringVar()
        self.rz_default.set(self.record_coords[0][5])

        # joint Input Box
        self.J_1 = tk.Entry(self.frmLT, textvariable=self.j1_default)
        self.J_1.grid(row=0, column=1, pady=3)
        self.J_2 = tk.Entry(self.frmLT, textvariable=self.j2_default)
        self.J_2.grid(row=1, column=1, pady=3)
        self.J_3 = tk.Entry(self.frmLT, textvariable=self.j3_default)
        self.J_3.grid(row=2, column=1, pady=3)
        self.J_4 = tk.Entry(self.frmLT, textvariable=self.j4_default)
        self.J_4.grid(row=3, column=1, pady=3)
        self.J_5 = tk.Entry(self.frmLT, textvariable=self.j5_default)
        self.J_5.grid(row=4, column=1, pady=3)
        self.J_6 = tk.Entry(self.frmLT, textvariable=self.j6_default)
        self.J_6.grid(row=5, column=1, pady=3)

        # coord Input Box
        self.x = tk.Entry(self.frmRT, textvariable=self.x_default)
        self.x.grid(row=0, column=1, pady=3, padx=0)
        self.y = tk.Entry(self.frmRT, textvariable=self.y_default)
        self.y.grid(row=1, column=1, pady=3)
        self.z = tk.Entry(self.frmRT, textvariable=self.z_default)
        self.z.grid(row=2, column=1, pady=3)
        self.rx = tk.Entry(self.frmRT, textvariable=self.rx_default)
        self.rx.grid(row=3, column=1, pady=3)
        self.ry = tk.Entry(self.frmRT, textvariable=self.ry_default)
        self.ry.grid(row=4, column=1, pady=3)
        self.rz = tk.Entry(self.frmRT, textvariable=self.rz_default)
        self.rz.grid(row=5, column=1, pady=3)

        # All input boxes are used to get the input data
        self.all_j = [self.J_1, self.J_2,
                      self.J_3, self.J_4, self.J_5, self.J_6]
        self.all_c = [self.x, self.y, self.z, self.rx, self.ry, self.rz]

        # Speed input box
        tk.Label(
            self.frmLB,
            text="speed",
        ).grid(row=0, column=0)
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
            if angle_list and len(angle_list[0]) > index:
                return "{}°".format(angle_list[0][index])
        except Exception as e:
            self.get_logger().warn("safe_get_angle error: {}".format(e))
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
            if coords_list and len(coords_list) > 0:
                value = coords_list[0][index]
                if value != -1:
                    return str(value)
        except Exception as e:
            self.get_logger().warn("safe_get_coord error: {}".format(e))
        return default

    def show_init(self):
        """Display the robot arm angle coordinate data"""
        # display
        tk.Label(self.frmLC, text="Joint 1 ").grid(row=0)
        tk.Label(self.frmLC, text="Joint 2 ").grid(row=1)
        tk.Label(self.frmLC, text="Joint 3 ").grid(row=2)
        tk.Label(self.frmLC, text="Joint 4 ").grid(row=3)
        tk.Label(self.frmLC, text="Joint 5 ").grid(row=4)
        tk.Label(self.frmLC, text="Joint 6 ").grid(row=5)

        # Get data display
        self.cont_1 = tk.StringVar(self.frmLC)
        self.cont_1.set(self.safe_get_angle(self.res_angles, 0))
        self.cont_2 = tk.StringVar(self.frmLC)
        self.cont_2.set(self.safe_get_angle(self.res_angles, 1))
        self.cont_3 = tk.StringVar(self.frmLC)
        self.cont_3.set(self.safe_get_angle(self.res_angles, 2))
        self.cont_4 = tk.StringVar(self.frmLC)
        self.cont_4.set(self.safe_get_angle(self.res_angles, 3))
        self.cont_5 = tk.StringVar(self.frmLC)
        self.cont_5.set(self.safe_get_angle(self.res_angles, 4))
        self.cont_6 = tk.StringVar(self.frmLC)
        self.cont_6.set(self.safe_get_angle(self.res_angles, 5))
        self.cont_all = [
            self.cont_1,
            self.cont_2,
            self.cont_3,
            self.cont_4,
            self.cont_5,
            self.cont_6,
            self.speed,
            self.model,
        ]

        self.show_j1 = tk.Label(
            self.frmLC,
            textvariable=self.cont_1,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=0, column=1, padx=0, pady=5)

        self.show_j2 = tk.Label(
            self.frmLC,
            textvariable=self.cont_2,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=1, column=1, padx=0, pady=5)
        self.show_j3 = tk.Label(
            self.frmLC,
            textvariable=self.cont_3,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=2, column=1, padx=0, pady=5)
        self.show_j4 = tk.Label(
            self.frmLC,
            textvariable=self.cont_4,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=3, column=1, padx=0, pady=5)
        self.show_j5 = tk.Label(
            self.frmLC,
            textvariable=self.cont_5,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=4, column=1, padx=0, pady=5)
        self.show_j6 = tk.Label(
            self.frmLC,
            textvariable=self.cont_6,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=5, column=1, padx=5, pady=5)

        self.all_jo = [
            self.show_j1,
            self.show_j2,
            self.show_j3,
            self.show_j4,
            self.show_j5,
            self.show_j6,
        ]

        # display
        tk.Label(self.frmLC, text="  x ").grid(row=0, column=3)
        tk.Label(self.frmLC, text="  y ").grid(row=1, column=3)  # second row
        tk.Label(self.frmLC, text="  z ").grid(row=2, column=3)
        tk.Label(self.frmLC, text="  rx ").grid(row=3, column=3)
        tk.Label(self.frmLC, text="  ry ").grid(row=4, column=3)
        tk.Label(self.frmLC, text="  rz ").grid(row=5, column=3)
        self.coord_x = tk.StringVar()
        self.coord_x.set(self.safe_get_coord(self.record_coords, 0))
        self.coord_y = tk.StringVar()
        self.coord_y.set(self.safe_get_coord(self.record_coords, 1))
        self.coord_z = tk.StringVar()
        self.coord_z.set(self.safe_get_coord(self.record_coords, 2))
        self.coord_rx = tk.StringVar()
        self.coord_rx.set(self.safe_get_coord(self.record_coords, 3))
        self.coord_ry = tk.StringVar()
        self.coord_ry.set(self.safe_get_coord(self.record_coords, 4))
        self.coord_rz = tk.StringVar()
        self.coord_rz.set(self.safe_get_coord(self.record_coords, 5))

        self.coord_all = [
            self.coord_x,
            self.coord_y,
            self.coord_z,
            self.coord_rx,
            self.coord_ry,
            self.coord_rz,
            self.speed,
            self.model,
        ]

        self.show_x = tk.Label(
            self.frmLC,
            textvariable=self.coord_x,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=0, column=4, padx=5, pady=5)
        self.show_y = tk.Label(
            self.frmLC,
            textvariable=self.coord_y,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=1, column=4, padx=5, pady=5)
        self.show_z = tk.Label(
            self.frmLC,
            textvariable=self.coord_z,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=2, column=4, padx=5, pady=5)
        self.show_rx = tk.Label(
            self.frmLC,
            textvariable=self.coord_rx,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=3, column=4, padx=5, pady=5)
        self.show_ry = tk.Label(
            self.frmLC,
            textvariable=self.coord_ry,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=4, column=4, padx=5, pady=5)
        self.show_rz = tk.Label(
            self.frmLC,
            textvariable=self.coord_rz,
            font=("Arial", 9),
            width=7,
            height=1,
            bg="white",
        ).grid(row=5, column=4, padx=5, pady=5)

        # mm Unit Display
        self.unit = tk.StringVar()
        self.unit.set("mm")
        for i in range(6):
            tk.Label(self.frmLC, textvariable=self.unit, font=("Arial", 9)).grid(
                row=i, column=5
            )

    def gripper_open(self):
        """Open the robotic arm gripper.

        Acquires a lock to ensure exclusive access to the robotic arm and
        sends the command to open the gripper.

        Note:
            If an exception occurs, it is silently ignored.
        """
        try:
            if self.mc:
                lock = acquire("/tmp/mycobot_lock")
                self.mc.set_gripper_state(0, 80, 1)
                release(lock)
        except Exception:
            pass


    def gripper_close(self):
        """Close the robotic arm gripper.

        Acquires a lock to ensure exclusive access to the robotic arm and
        sends the command to close the gripper.

        Note:
            If an exception occurs, it is silently ignored.
        """
        try:
            if self.mc:
                lock = acquire("/tmp/mycobot_lock")
                self.mc.set_gripper_state(1, 80, 1)
                release(lock)
        except Exception:
            pass

    def pump_open(self):
        """Turn on the suction pump.

        Acquires a lock to ensure exclusive access and sets the pump output to ON.
        A small delay is added to ensure proper command execution.

        Note:
            If an exception occurs, it is silently ignored.
        """
        try:
            if self.mc:
                lock = acquire("/tmp/mycobot_lock")
                self.mc.set_basic_output(5, 0)
                release(lock)
                time.sleep(0.05)
        except Exception:
            # Probably because the method has no return value, the service throws an unhandled error
            pass

    def pump_close(self):
        """Turn off the suction pump.

        Acquires a lock to ensure exclusive access and sets the pump output to OFF.
        A small delay is added to ensure proper command execution.

        Note:
            If an exception occurs, it is silently ignored.
        """
        try:
            if self.mc:
                lock = acquire("/tmp/mycobot_lock")
                self.mc.set_basic_output(5, 1)
                time.sleep(0.05)
                self.mc.set_basic_output(2, 0)
                time.sleep(0.05)
                self.mc.set_basic_output(2, 1)
                time.sleep(0.05)
                release(lock)
        except Exception:
            pass

    def get_coord_input(self):
        """Read coordinate input from the GUI and send it to the robotic arm.

        The coordinates are retrieved from the GUI input fields, the speed
        is updated if specified, and the coordinates are sent to the robotic arm.
        The GUI is updated to reflect the new coordinates.
        """
        c_value = [float(i.get()) for i in self.all_c]
        self.speed = int(float(self.get_speed.get())) if self.get_speed.get() else self.speed

        try:
            if self.mc:
                lock = acquire("/tmp/mycobot_lock")
                self.mc.send_coords(c_value, self.speed, self.model)
                release(lock)
        except Exception:
            pass

        self.show_j_date(c_value, "coord")


    def get_joint_input(self):
        """Read joint angles input from the GUI and send it to the robotic arm.

        The joint angles are retrieved from the GUI input fields, the speed
        is updated if specified, and the angles are sent to the robotic arm.
        The GUI is updated to reflect the new angles.
        """
        j_value = [float(i.get()) for i in self.all_j]
        self.speed = int(float(self.get_speed.get())) if self.get_speed.get() else self.speed

        res = [j_value, self.speed]
        try:
            if self.mc:
                lock = acquire("/tmp/mycobot_lock")
                self.mc.send_angles(*res)
                release(lock)
        except Exception:
            pass

        self.show_j_date(j_value)


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


    def show_j_date(self, date, way=""):
        """Update the GUI with current joint or coordinate data.

        Args:
            date (list[float]): List of joint angles or coordinates.
            way (str, optional): "coord" to indicate coordinates, otherwise joint angles.
        """
        if way == "coord":
            for i, j in zip(date, self.coord_all):
                j.set(str(i))
        else:
            for i, j in zip(date, self.cont_all):
                j.set(str(i) + "°")


    def run(self):
        """Run the GUI main loop.

        Continuously updates the Tkinter window and handles GUI events.
        The loop exits if the window is destroyed.
        """
        while True:
            try:
                self.win.update()
                time.sleep(0.001)
            except tk.TclError as e:
                if "application has been destroyed" in str(e):
                    break
                else:
                    raise



def main(args=None):
    """Run the MyCobot ROS GUI application.

    Initializes ROS2, creates the Tkinter window and the WindowNode,
    and starts the GUI main loop.

    Args:
        args (list, optional): Command-line arguments passed to ROS2.
            Defaults to None.
    """
    rclpy.init(args=args)
    window = tk.Tk()
    window.title("mycobot ros GUI")
    node = WindowNode(window)

    try:
        node.run()
    except KeyboardInterrupt:
        # Allow graceful exit on Ctrl+C
        pass
    # Note: The destroy_node() and shutdown() calls are commented out in
    # the original code. They can be added if proper ROS2 shutdown is needed.
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == "__main__":
    main()

