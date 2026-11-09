from __future__ import print_function
import sys
import termios
import tty
import traceback
import time
import rclpy
from rclpy.node import Node
from mycobot_pro450_interfaces.srv import SetAngles, SetCoords, GripperStatus, GetCoords, GetAngles

from mycobot_pro_450.joint5_support import (
    load_robot_limit_profile,
    resolve_gripper_profile_for_client_node,
    resolve_joint5_offset_for_client_node,
)

MSG_BASE = """\
Mycobot Teleop Keyboard Controller
---------------------------
Movimg options(control coordinations [x,y,z,rx,ry,rz]):
              w(x+)

    a(y-)     s(x-)     d(y+)

    z(z-) x(z+)

u(rx+)   i(ry+)   o(rz+)
j(rx-)   k(ry-)   l(rz-)

+/- : Increase/decrease movement step size
"""

MSG_GRIPPER = """\
Force Gripper control:
    g - open
    h - close
"""

MSG_OTHER = """\
Other:
    1 - Go to init pose
    2 - Go to home pose
    3 - Resave home pose
    q - Quit
"""

COORD_AXES = ('x', 'y', 'z', 'rx', 'ry', 'rz')


def build_help_message(enable_force_gripper):
    """Build help text; hide g/h when gripper profile is none."""
    if enable_force_gripper:
        return MSG_BASE + "\n" + MSG_GRIPPER + "\n" + MSG_OTHER
    return MSG_BASE + "\n" + MSG_OTHER


def get_coordinate_limits(joint5_offset_deg):
    """Load Cartesian limits from pymycobot's Pro450 profile."""
    _, limits = load_robot_limit_profile(joint5_offset_deg)
    return dict(
        zip(COORD_AXES, zip(limits["coords_min"], limits["coords_max"]))
    )


def vels(speed, turn):
    """Return current speed and percent change information."""
    return "currently:\tspeed: %s\tchange percent: %s  " % (speed, turn)


class Raw(object):
    """Context manager for raw terminal input mode."""

    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()

    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)


class TeleopKeyboardNode(Node):
    """ROS2 node for controlling MyCobot via keyboard inputs."""

    def __init__(self, gripper_local_default="none"):
        """Initialize TeleopKeyboardNode and create service clients.

        Args:
            gripper_local_default: Fallback gripper profile when driver param
                is unavailable. Force-gripper entry uses ``f100``.
        """
        super().__init__('teleop_keyboard_client')

        self.set_angles_client = self.create_client(SetAngles, '/set_angles')
        self.set_coords_client = self.create_client(SetCoords, '/set_coords')
        self.get_coords_client = self.create_client(GetCoords, '/get_coords')
        self.get_angles_client = self.create_client(GetAngles, '/get_angles')
        self.set_force_gripper_client = self.create_client(
            GripperStatus, '/set_force_gripper')

        while not self.set_angles_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.joint5_offset_deg = resolve_joint5_offset_for_client_node(self)
        self.gripper_profile = resolve_gripper_profile_for_client_node(
            self, local_default=gripper_local_default
        )
        self.enable_force_gripper = self.gripper_profile == "f100"
        self.coord_limits = get_coordinate_limits(self.joint5_offset_deg)
        self.get_logger().info(
            "Keyboard ready: joint5_offset_deg={}, gripper={}, g/h={}".format(
                self.joint5_offset_deg,
                self.gripper_profile,
                "enabled" if self.enable_force_gripper else "hidden",
            )
        )

        self.speed = 50
        self.change_percent = 5
        self.change_angle = 180 * self.change_percent / 100
        self.change_len = 250 * self.change_percent / 100

        self.init_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Firmware-coordinate home pose; J5 is always 0°.
        self.home_pose = [0.0, -10.0, -123.0, 45.0, 0.0, 0.0]

        self.record_coords = self.get_initial_coords()
        self.ready_for_coords = False
        self._warned_gripper_disabled = False

    def get_initial_coords(self):
        request = GetCoords.Request()
        future = self.get_coords_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return [[future.result().x, future.result().y, future.result().z,
                     future.result().rx, future.result().ry, future.result().rz],
                    self.speed]
        self.get_logger().error('Failed to get coordinates')
        return [[-1, -1, -1, -1, -1, -1], self.speed]

    def get_initial_angles(self):
        request = GetAngles.Request()
        future = self.get_angles_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return [future.result().joint_1, future.result().joint_2,
                    future.result().joint_3, future.result().joint_4,
                    future.result().joint_5, future.result().joint_6]
        self.get_logger().error("Failed to get angles")
        return [-1, -1, -1, -1, -1, -1]

    def send_coords(self):
        coords = self.record_coords[0]
        for idx, axis in enumerate(COORD_AXES):
            min_limit, max_limit = self.coord_limits[axis]
            if coords[idx] < min_limit or coords[idx] > max_limit:
                self.get_logger().warn(
                    "%s value %s exceeds the limit range [%s, %s], unable to send coordinates"
                    % (axis, coords[idx], min_limit, max_limit)
                )
                return

        request = SetCoords.Request()
        request.x, request.y, request.z, request.rx, request.ry, request.rz = coords
        request.speed = self.record_coords[1]
        future = self.set_coords_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to set coordinates')

    def send_angles(self, angles):
        request = SetAngles.Request()
        (request.joint_1, request.joint_2, request.joint_3,
         request.joint_4, request.joint_5, request.joint_6) = angles
        request.speed = self.speed
        future = self.set_angles_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to set angles')

    def set_force_gripper(self, status):
        request = GripperStatus.Request()
        request.status = status
        future = self.set_force_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to control force gripper')

    def keyboard_listener(self):
        print(build_help_message(self.enable_force_gripper))
        print(vels(self.speed, self.change_percent))
        while rclpy.ok():
            try:
                with Raw(sys.stdin):
                    key = sys.stdin.read(1)
                if key == "q":
                    break
                elif key == "1":
                    self.send_angles(self.init_pose)
                    time.sleep(2)
                    self.record_coords = self.get_initial_coords()
                    self.ready_for_coords = False
                    self.get_logger().warn(
                        "Returned to zero pose. Press '2' to enable coordinate control.\n"
                    )
                elif key == "2":
                    self.send_angles(self.home_pose)
                    time.sleep(2)
                    self.record_coords = self.get_initial_coords()
                    self.ready_for_coords = True
                    self.get_logger().info(
                        "Home pose reached. Coordinate control enabled.\n"
                    )
                elif key == "3":
                    self.home_pose = self.get_initial_angles()
                    print(f"New home pose saved: {self.home_pose}")
                elif key in ["w", "W", "s", "S", "a", "A", "d", "D", "z", "Z", "x", "X",
                             "u", "U", "j", "J", "i", "I", "k", "K", "o", "O", "l", "L"]:
                    if not self.ready_for_coords:
                        self.get_logger().warn(
                            "Coordinate control disabled. Please press '2' to enable."
                        )
                        continue

                    if key in ["w", "W"]:
                        self.record_coords[0][0] += self.change_len
                    elif key in ["s", "S"]:
                        self.record_coords[0][0] -= self.change_len
                    elif key in ["a", "A"]:
                        self.record_coords[0][1] -= self.change_len
                    elif key in ["d", "D"]:
                        self.record_coords[0][1] += self.change_len
                    elif key in ["z", "Z"]:
                        self.record_coords[0][2] -= self.change_len
                    elif key in ["x", "X"]:
                        self.record_coords[0][2] += self.change_len
                    elif key in ["u", "U"]:
                        self.record_coords[0][3] += self.change_angle
                    elif key in ["j", "J"]:
                        self.record_coords[0][3] -= self.change_angle
                    elif key in ["i", "I"]:
                        self.record_coords[0][4] += self.change_angle
                    elif key in ["k", "K"]:
                        self.record_coords[0][4] -= self.change_angle
                    elif key in ["o", "O"]:
                        self.record_coords[0][5] += self.change_angle
                    elif key in ["l", "L"]:
                        self.record_coords[0][5] -= self.change_angle

                    self.send_coords()

                elif key in ["g", "G", "h", "H"]:
                    if not self.enable_force_gripper:
                        if not self._warned_gripper_disabled:
                            self.get_logger().warn(
                                "Force gripper keys g/h are disabled "
                                "(launch gripper:=none). "
                                "Use gripper:=f100 to enable."
                            )
                            self._warned_gripper_disabled = True
                        continue
                    if key in ["g", "G"]:
                        self.set_force_gripper(True)
                    else:
                        self.set_force_gripper(False)

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


def main(args=None, gripper_local_default="none"):
    """Main entry point to start the TeleopKeyboardNode."""
    rclpy.init(args=args)
    teleop_keyboard = TeleopKeyboardNode(
        gripper_local_default=gripper_local_default
    )
    teleop_keyboard.keyboard_listener()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
