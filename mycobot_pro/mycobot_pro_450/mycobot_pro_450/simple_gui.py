#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import queue
import sys
import threading
import tkinter as tk
import time
from tkinter import messagebox
import rclpy
from rclpy.node import Node
from pymycobot.robot_info import RobotLimit
from mycobot_pro450_interfaces.srv import SetAngles, SetCoords, GripperStatus, GetCoords, GetAngles


class WindowNode(Node):
    """ROS2 node with Tkinter GUI interface for controlling a MyCobot robotic arm.

    This node provides services to send joint angles, coordinates, and gripper
    commands to the robot, as well as to query the current robot state. It also
    integrates a Tkinter GUI for user input and displays robot data in real time.
    """

    def __init__(self, handle):
        """Initialize the WindowNode, ROS2 service clients, and GUI.

        Args:
            handle (tk.Tk): Tkinter window instance to attach the GUI.
        """
        super().__init__('simple_gui')

        # ROS2 client request
        self.set_angles_client = self.create_client(SetAngles, '/set_angles')
        self.set_coords_client = self.create_client(SetCoords, '/set_coords')
        self.set_gripper_client = self.create_client(
            GripperStatus, '/set_gripper')
        self.get_coords_client = self.create_client(GetCoords, '/get_coords')
        self.get_angles_client = self.create_client(GetAngles, '/get_angles')
        self.set_force_gripper_client = self.create_client(
            GripperStatus, '/set_force_gripper')

        # Waiting for the service to go online
        while not self.set_angles_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Service not available, waiting again...')

        # Tkinter window setup
        self.win = handle
        self.win.resizable(0, 0)  # Fixed window size

        self.speed = 50

        # Default speed variable
        self.speed_d = tk.StringVar()
        self.speed_d.set(str(self.speed))

        # Robotic arm data
        self.record_coords = [
            [0, 0, 0, 0, 0, 0],
            self.speed
        ]
        self.res_angles = [
            [0, 0, 0, 0, 0, 0],
            self.speed
        ]
        # Command queue and background worker
        self.cmd_queue = queue.Queue()
        threading.Thread(target=self.worker, daemon=True).start()

        self.get_date()  # Initialize data from the robot

        # Screen dimensions
        self.ws = self.win.winfo_screenwidth()
        self.hs = self.win.winfo_screenheight()

        # Calculate window position
        x = (self.ws / 2) - 190
        y = (self.hs / 2) - 250
        self.win.geometry("470x440+{}+{}".format(int(x), int(y)))

        # GUI layout and widgets
        self.set_layout()
        self.need_input()
        self.show_init()

        # Buttons for joint settings
        tk.Button(self.frmLT, text="Set Joints", width=10, command=self.get_joint_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )

        # Buttons for coordinate settings
        tk.Button(self.frmRT, text="Set Coords", width=10, command=self.get_coord_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )

        # Gripper control buttons
        tk.Button(self.frmLB, text="Gripper Open", command=self.gripper_open, width=10).grid(
            row=1, column=0, sticky="w", padx=3, pady=20
        )
        tk.Button(self.frmLB, text="Gripper Close", command=self.gripper_close, width=10).grid(
            row=1, column=1, sticky="w", padx=3, pady=2
        )

        # Suction pump control buttons
        # tk.Button(self.frmLB, text=" 吸泵(开)", command=self.pump_open, width=5).grid(
        #     row=2, column=0, sticky="w", padx=3, pady=20
        # )
        # tk.Button(self.frmLB, text="吸泵(关)", command=self.pump_close, width=5).grid(
        #     row=2, column=1, sticky="w", padx=3, pady=2
        # )

        # Periodic GUI update
        self.update_gui()

        # Robot model and limits
        self.robot_name = "Pro450Client"
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
                    self.speed = speed
                    self.send_angles(values)

                elif cmd[0] == "coords":
                    _, values, speed = cmd
                    self.record_coords[0] = values
                    self.record_coords[1] = speed
                    self.send_coords()

                elif cmd[0] == "gripper":
                    _, state = cmd
                    self.set_force_gripper(state)

            except Exception as e:
                self.get_logger().warn(f"worker error: {e}")

    def wait_for_future(self, future, timeout=3.0):
        """Wait for a future to complete with timeout (non-blocking ROS executor)."""
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            # Only process the callback once, so the GUI will not be blocked
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.result()

    def get_initial_coords(self):
        """Fetch current coordinates from the robot.

        Returns:
            list: [[x, y, z, rx, ry, rz], speed]
        """
        request = GetCoords.Request()
        future = self.get_coords_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        result = self.wait_for_future(future)
        if result is not None:
            return [[result.x, result.y, result.z,
                     result.rx, result.ry, result.rz], self.speed]
        else:
            self.get_logger().error('Failed to get coordinates')
            return [[-1, -1, -1, -1, -1, -1], self.speed]

    def get_initial_angles(self):
        """Fetch current joint angles from the robot.

        Returns:
            list: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        """
        request = GetAngles.Request()
        future = self.get_angles_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        result = self.wait_for_future(future)
        if result is not None:
            return [result.joint_1, result.joint_2, result.joint_3,
                    result.joint_4, result.joint_5, result.joint_6]
        else:
            self.get_logger().error("Failed to get angles")
            return [-1, -1, -1, -1, -1, -1]

    def send_coords(self):
        """Send coordinates to the robot, ensuring they are within limits."""
        coords = self.record_coords[0]
        request = SetCoords.Request()
        request.x, request.y, request.z, request.rx, request.ry, request.rz = coords
        request.speed = self.record_coords[1]

        future = self.set_coords_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to set coordinates')

    def send_angles(self, angles):
        """Send joint angles to the robot.

        Args:
            angles (list): List of 6 joint angle values.
        """
        request = SetAngles.Request()
        (request.joint_1, request.joint_2, request.joint_3,
         request.joint_4, request.joint_5, request.joint_6) = angles
        request.speed = self.speed

        future = self.set_angles_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to set angles')

    def set_force_gripper(self, status):
        """Force control gripper open/close.

        Args:
            status (bool): True for open, False for close.
        """
        request = GripperStatus.Request()
        request.status = status
        future = self.set_force_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to control force gripper')

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
        """Display input fields for joint angles and robot coordinates."""

        # Joint labels and input variables
        joint_names = ["Joint 1", "Joint 2",
                       "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
        self.all_j = []
        self.joint_vars = []

        for i, name in enumerate(joint_names):
            tk.Label(self.frmLT, text=name).grid(row=i)
            var = tk.StringVar()
            var.set(self.res_angles[0][i])
            self.joint_vars.append(var)
            entry = tk.Entry(self.frmLT, textvariable=var)
            entry.grid(row=i, column=1, pady=3)
            self.all_j.append(entry)

        # Coordinate labels and input variables
        coord_names = ["x", "y", "z", "rx", "ry", "rz"]
        self.all_c = []
        self.coord_vars = []

        for i, name in enumerate(coord_names):
            tk.Label(self.frmRT, text=f" {name} ").grid(row=i)
            var = tk.StringVar()
            var.set(self.record_coords[0][i])
            self.coord_vars.append(var)
            entry = tk.Entry(self.frmRT, textvariable=var)
            entry.grid(row=i, column=1, pady=3, padx=0)
            self.all_c.append(entry)

        # Speed input
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
            if angle_list and len(angle_list[0]) > index:
                return "{}°".format(round(angle_list[0][index], 2))
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
            # self.get_logger().info("safe_get_coord: {}".format(coords_list))
            if coords_list and len(coords_list) > 0:
                value = coords_list[0][index]
                if value != -1:
                    return str(round(value, 2))
        except Exception as e:
            self.get_logger().warn("safe_get_coord error: {}".format(e))
        return default

    def show_init(self):
        """Display the robot arm joint angles and coordinate data in the GUI."""

        # Joint labels
        joint_names = ["Joint 1", "Joint 2",
                       "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
        self.cont_all = []
        self.all_jo = []

        for i, name in enumerate(joint_names):
            tk.Label(self.frmLC, text=name).grid(row=i)
            var = tk.StringVar(self.frmLC)
            var.set(self.safe_get_angle(self.res_angles, i))
            self.cont_all.append(var)
            lbl = tk.Label(
                self.frmLC,
                textvariable=var,
                font=("Arial", 9),
                width=7,
                height=1,
                bg="white"
            )
            lbl.grid(row=i, column=1, padx=5, pady=5)
            self.all_jo.append(lbl)

        # Add speed to joint variables
        self.cont_all.append(self.speed)

        # Coordinate labels
        coord_names = ["x", "y", "z", "rx", "ry", "rz"]
        self.coord_all = []
        coord_vars = []

        for i, name in enumerate(coord_names):
            tk.Label(self.frmLC, text=f"  {name} ").grid(row=i, column=3)
            var = tk.StringVar(self.frmLC)
            var.set(self.safe_get_coord(self.record_coords, i))
            self.coord_all.append(var)
            lbl = tk.Label(
                self.frmLC,
                textvariable=var,
                font=("Arial", 9),
                width=7,
                height=1,
                bg="white"
            )
            lbl.grid(row=i, column=4, padx=5, pady=5)
            coord_vars.append(lbl)

        # Add speed to coordinate variables
        self.coord_all.append(self.speed)

        # Unit display (mm)
        unit_var = tk.StringVar(value="mm")
        for i in range(6):
            tk.Label(self.frmLC, textvariable=unit_var,
                     font=("Arial", 9)).grid(row=i, column=5)

    def gripper_open(self):
        """Open the robotic arm's gripper.

        Attempts to open the gripper by setting the force gripper state to True.
        Any exceptions during the operation are silently ignored.
        """
        try:
            self.set_force_gripper(True)
        except Exception:
            pass

    def gripper_close(self):
        """Close the robotic arm's gripper.

        Attempts to close the gripper by setting the force gripper state to False.
        Any exceptions during the operation are silently ignored.
        """
        try:
            self.set_force_gripper(False)
        except Exception:
            pass

    def show_error(self, msg):
        """Safely show an error message box in the main thread.

        Args:
            msg (str): The error message to display.
        """
        self.win.after(0, lambda: messagebox.showerror(
            "Error", msg, parent=self.win))

    def get_coord_input(self):
        """Read coordinates input from the GUI and send them to the robotic arm.

        Retrieves the coordinate values from the GUI input fields, validates
        them against predefined min/max limits, and reads the speed input.
        Sends the coordinates and speed as a command to the robot through
        a queue for execution.

        Displays error messages via `show_error` if input is invalid.

        Raises:
            ValueError: If the coordinate or speed inputs cannot be converted
            to numbers.
        """
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
            speed_str = self.get_speed.get()
            if not speed_str:
                self.show_error("Please enter a speed value (1-100)")
                return

            speed = int(float(speed_str))
            if not (1 <= speed <= 100):
                self.show_error("Speed input value must be between 1 and 100")
                return
        except ValueError:
            self.show_error("Speed must be a number")
            return

        self.speed = speed
        self.cmd_queue.put(("coords", c_value, self.speed))

    def get_joint_input(self):
        """Read joint angles input from the GUI and send them to the robotic arm.

        Retrieves the joint angles from the GUI input fields, validates them
        against predefined min/max limits, and reads the speed input.
        Sends the joint angles and speed as a command to the robot through
        a queue for execution.

        Displays error messages via `show_error` if input is invalid.

        Raises:
            ValueError: If the joint angle or speed inputs cannot be converted
            to numbers.
        """
        try:
            j_value = [float(i.get()) for i in self.all_j]
        except ValueError:
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
            speed_str = self.get_speed.get()
            if not speed_str:
                self.show_error("Please enter a speed value (1-100)")
                return

            speed = int(float(speed_str))
            if not (1 <= speed <= 100):
                self.show_error("Speed input value must be between 1 and 100")
                return
        except ValueError:
            self.show_error("Speed must be a number")
            return

        self.speed = speed
        self.cmd_queue.put(("angles", j_value, self.speed))

    def get_date(self):
        """Retrieve the current coordinates and joint angles from the robotic arm.

        Queries the robot for its current coordinates and joint angles for up
        to 2 seconds each. Uses service client methods rather than direct robot
        access. The retrieved values are rounded to 2 decimal places and stored
        internally for GUI display or further processing.

        Updates:
            self.res: Current coordinates as a list of floats.
            self.angles: Current joint angles as a list of floats.
            self.record_coords[0]: Updated coordinate record.
            self.res_angles[0]: Updated joint angle record.
        """
        # Get coordinates
        t_start = time.time()
        while time.time() - t_start < 2:
            self.res = self.get_initial_coords()[0]
            if self.res != [-1, -1, -1, -1, -1, -1]:
                self.res = [round(val, 2) for val in self.res]
                break
            time.sleep(0.1)

        # Get joint angles
        t_start = time.time()
        while time.time() - t_start < 2:
            self.angles = self.get_initial_angles()
            if self.angles != [-1, -1, -1, -1, -1, -1]:
                self.angles = [round(val, 2) for val in self.angles]
                break
            time.sleep(0.1)

        # Update internal records
        self.record_coords[0] = self.res
        self.res_angles[0] = self.angles

    def update_gui(self):
        """Periodically refresh the GUI and update joint angles and coordinates.

        This method queries the current joint angles and coordinates from the
        robot, updates the Tkinter variables for display, and schedules itself
        to run again after 300 ms.
        """
        try:
            angles = self.get_initial_angles()
            coords = self.get_initial_coords()
            if angles:
                self.res_angles = [angles]
                for i, var in enumerate(self.cont_all[:6]):
                    var.set(self.safe_get_angle(self.res_angles, i))

            if coords:
                self.record_coords = coords
                for i, var in enumerate(self.coord_all[:6]):
                    var.set(self.safe_get_coord(self.record_coords, i))
        except Exception as e:
            self.get_logger().warn(f"update_gui error: {e}")

        # Schedule next update in 300 ms
        self.win.after(300, self.update_gui)


def main(args=None):
    """Initialize the ROS2 node and launch the Tkinter GUI for MyCobot.

    This function initializes the rclpy client library, creates the main
    Tkinter window, initializes the WindowNode to handle ROS2 communication
    and GUI interactions, and starts the Tkinter main loop. The GUI can be
    safely interrupted using Ctrl+C (KeyboardInterrupt).

    Args:
        args (list[str], optional): Command line arguments to pass to rclpy.
            Defaults to None.
    """
    rclpy.init(args=args)
    window = tk.Tk()
    window.title("mycobot ros GUI")
    node = WindowNode(window)

    try:
        window.mainloop()
    except KeyboardInterrupt:
        # Allow graceful exit on Ctrl+C
        print("Exiting...")
        rclpy.shutdown()  # Shutdown ROS2 client library
        sys.exit(0)       # Exit the program


if __name__ == "__main__":
    main()
