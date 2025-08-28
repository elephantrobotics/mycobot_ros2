from __future__ import print_function
import sys
import termios
import tty
import traceback
import time
import rclpy
from rclpy.node import Node
from mecharm_interfaces.srv import SetAngles, SetCoords, GripperStatus, GetCoords, GetAngles, PumpStatus


MSG = """\
Mycobot Teleop Keyboard Controller
---------------------------
Movimg options(control coordinations [x,y,z,rx,ry,rz]):
              w(x+)

    a(y-)     s(x-)     d(y+)

    z(z-) x(z+)

u(rx+)   i(ry+)   o(rz+)
j(rx-)   k(ry-)   l(rz-)

+/- : Increase/decrease movement step size

Gripper control:
    g - open
    h - close

Pump control:
    b - open
    m - close

Other:
    1 - Go to init pose
    2 - Go to home pose
    3 - Resave home pose
    q - Quit
"""

COORD_LIMITS = {
    'x': (-272, 272),
    'y': (-272, 272),
    'z': (-86, 408.9),
    'rx': (-180, 180),
    'ry': (-180, 180),
    'rz': (-180, 180)
}


def vels(speed, turn):
    """Return current speed and percent change information.

    Args:
        speed (int): Movement speed value.
        turn (int): Percentage change for movement step size.

    Returns:
        str: Formatted string with current speed and change percent.
    """
    return "currently:\tspeed: %s\tchange percent: %s  " % (speed, turn)


class Raw(object):
    """Context manager for raw terminal input mode."""

    def __init__(self, stream):
        """Initialize Raw input handler.

        Args:
            stream (file): Input stream (usually sys.stdin).
        """
        self.stream = stream
        self.fd = self.stream.fileno()

    def __enter__(self):
        """Enable raw terminal input mode."""
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)

    def __exit__(self, type, value, traceback):
        """Restore original terminal settings."""
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)


class TeleopKeyboardNode(Node):
    """ROS2 node for controlling MyCobot via keyboard inputs."""

    def __init__(self):
        """Initialize TeleopKeyboardNode and create service clients."""
        super().__init__('teleop_keyboard_client')

        # client request
        self.set_angles_client = self.create_client(SetAngles, '/set_angles')
        self.set_coords_client = self.create_client(SetCoords, '/set_coords')
        self.set_gripper_client = self.create_client(
            GripperStatus, '/set_gripper')
        self.get_coords_client = self.create_client(GetCoords, '/get_coords')
        self.get_angles_client = self.create_client(GetAngles, '/get_angles')
        self.set_pump_client = self.create_client(
            PumpStatus, '/set_pump_status')

        # Waiting for the service to go online
        while not self.set_angles_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.speed = 50
        self.model = 1  # Sport Mode
        self.change_percent = 5  # Percentage of change

        self.change_angle = 180 * self.change_percent / 100
        self.change_len = 250 * self.change_percent / 100

        self.init_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.home_pose = [0.0, 0.0, 0.0, 0.0, 90.0, 0.0]

        self.record_coords = self.get_initial_coords()

    def get_initial_coords(self):
        """Fetch current coordinates from the robot.

        Returns:
            list: [[x, y, z, rx, ry, rz], speed, model]
        """
        request = GetCoords.Request()
        future = self.get_coords_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return [[future.result().x, future.result().y, future.result().z,
                     future.result().rx, future.result().ry, future.result().rz], self.speed, self.model]
        else:
            self.get_logger().error('Failed to get coordinates')
            return [[-1, -1, -1, -1, -1, -1], self.speed, self.model]

    def get_initial_angles(self):
        """Fetch current joint angles from the robot.

        Returns:
            list: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        """
        request = GetAngles.Request()
        future = self.get_angles_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return [future.result().joint_1, future.result().joint_2, future.result().joint_3,
                    future.result().joint_4, future.result().joint_5, future.result().joint_6]
        else:
            self.get_logger().error("Failed to get angles")
            return [-1, -1, -1, -1, -1, -1]

    def print_status(self):
        """Print the current coordinates to console."""
        coords = self.record_coords[0]
        print(
            "\r current coords: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]" % tuple(coords))

    def send_coords(self):
        """Send coordinates to the robot, ensuring they are within limits."""
        coords = self.record_coords[0]
        for i, axis in enumerate(['x', 'y', 'z', 'rx', 'ry', 'rz']):
            min_limit, max_limit = COORD_LIMITS[axis]
            if coords[i] < min_limit or coords[i] > max_limit:
                self.get_logger().warn(
                    f"{axis} value {coords[i]} exceeds the limit range [{min_limit}, {max_limit}], unable to send coordinates")
                return
        request = SetCoords.Request()
        request.x, request.y, request.z, request.rx, request.ry, request.rz = coords
        request.speed = self.record_coords[1]
        request.model = self.record_coords[2]

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

    def set_gripper(self, status):
        """Control gripper open/close.

        Args:
            status (bool): True for open, False for close.
        """
        request = GripperStatus.Request()
        request.status = status
        future = self.set_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to control gripper')

    def set_pump_status(self, status, pin1, pin2):
        """Control the suction pump.

        Args:
            status (bool): Pump on/off.
            pin1 (int): Control pin 1.
            pin2 (int): Control pin 2.
        """
        request = PumpStatus.Request()
        request.status = status
        request.pin1 = pin1
        request.pin2 = pin2

        future = self.set_pump_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().flag:
                self.get_logger().info(
                    f"Pump status set to {status}, pin1: {pin1}, pin2: {pin2}")
            else:
                self.get_logger().error("Pump service returned failure.")
        else:
            self.get_logger().error('Failed to call pump service.')

    def keyboard_listener(self):
        """Listen for keyboard input and execute corresponding robot actions."""
        print(MSG)
        print(vels(self.speed, self.change_percent))
        while rclpy.ok():
            try:
                with Raw(sys.stdin):
                    key = sys.stdin.read(1)
                if key == "q":
                    break
                elif key in ["w", "W"]:
                    self.record_coords[0][0] += self.change_len
                    self.send_coords()
                elif key in ["s", "S"]:
                    self.record_coords[0][0] -= self.change_len
                    self.send_coords()
                elif key in ["a", "A"]:
                    self.record_coords[0][1] -= self.change_len
                    self.send_coords()
                elif key in ["d", "D"]:
                    self.record_coords[0][1] += self.change_len
                    self.send_coords()
                elif key in ["z", "Z"]:
                    self.record_coords[0][2] -= self.change_len
                    self.send_coords()
                elif key in ["x", "X"]:
                    self.record_coords[0][2] += self.change_len
                    self.send_coords()
                elif key in ["u", "U"]:
                    self.record_coords[0][3] += self.change_angle
                    self.send_coords()
                elif key in ["j", "J"]:
                    self.record_coords[0][3] -= self.change_angle
                    self.send_coords()
                elif key in ["i", "I"]:
                    self.record_coords[0][4] += self.change_angle
                    self.send_coords()
                elif key in ["k", "K"]:
                    self.record_coords[0][4] -= self.change_angle
                    self.send_coords()
                elif key in ["o", "O"]:
                    self.record_coords[0][5] += self.change_angle
                    self.send_coords()
                elif key in ["l", "L"]:
                    self.record_coords[0][5] -= self.change_angle
                    self.send_coords()
                elif key in ["g", "G"]:
                    self.set_gripper(True)  # open
                elif key in ["h", "H"]:
                    self.set_gripper(False)  # close
                elif key in ["b", "B"]:
                    self.set_pump_status(True, 1, 2)
                elif key in ["m", "M"]:
                    self.set_pump_status(False, 1, 2)
                elif key == "1":
                    self.send_angles(self.init_pose)
                    time.sleep(2)
                    self.record_coords = self.get_initial_coords()
                elif key == "2":
                    self.send_angles(self.home_pose)
                    time.sleep(2)
                    self.record_coords = self.get_initial_coords()
                elif key == "3":
                    self.home_pose = self.get_initial_angles()
                    print(f"New home pose saved: {self.home_pose}")
                elif key == '+':
                    self.change_percent = min(self.change_percent + 1, 20)
                    self.change_angle = 180 * self.change_percent / 100
                    self.change_len = 250 * self.change_percent / 100
                    print("Increase change_percent to %d%%, move step: %.1f mm" % (
                        self.change_percent, self.change_len))
                elif key == '-':
                    self.change_percent = max(self.change_percent - 1, 1)
                    self.change_angle = 180 * self.change_percent / 100
                    self.change_len = 250 * self.change_percent / 100
                    print("Decrease change_percent to %d%%, move step: %.1f mm" % (
                        self.change_percent, self.change_len))
                else:
                    continue

            except Exception as e:
                e = traceback.format_exc()
                self.get_logger().error(f"Error in key processing: {e}")
                continue

            time.sleep(0.02)


def main(args=None):
    """Main entry point to start the TeleopKeyboardNode."""
    rclpy.init(args=args)
    teleop_keyboard = TeleopKeyboardNode()
    teleop_keyboard.keyboard_listener()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
