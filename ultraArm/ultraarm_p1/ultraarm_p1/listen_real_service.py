import math
import time
import threading
import traceback

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from ultraarm_p1_interfaces.srv import SetAngles, SetCoords, GetCoords, GetAngles
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '4.0.5'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'The current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison
        )
    )
print('pymycobot library version meets the requirements!')
from pymycobot import UltraArmP1
from pymycobot.robot_info import RobotLimit

ROBOT_LIMIT = RobotLimit.robot_limit.get("UltraArmP1", {})
JOINT_LIMITS = list(zip(
    ROBOT_LIMIT.get("angles_min", [-165, -18, 89, -179]),
    ROBOT_LIMIT.get("angles_max", [165, 85, 200, 179]),
))


def valid_angles(angles):
    """Return True if all joint angles are inside the expected P1 range."""
    return all(low <= angle <= high for angle, (low, high) in zip(angles, JOINT_LIMITS))


class MyCobotDriver(Node):
    """ROS2 node for controlling the ultraArm P1.

    Provides publishers for joint states and services for controlling
    joints and coordinates.
    """

    def __init__(self):
        """Initialize MyCobotDriver node, publishers, and services."""
        super().__init__('mycobot_driver_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 1000000)
        # Match ROS1 listen_real default (~10 Hz) so RViz follows without multi-second lag.
        self.declare_parameter('publish_rate', 10.0)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        publish_rate = max(float(self.get_parameter("publish_rate").value), 0.1)

        self.get_logger().info("port:%s, baud:%d, publish_rate:%.1f Hz" % (port, baud, publish_rate))
        self.ua = UltraArmP1(port, baud)
        self.ua.set_joint_enable(0)
        self.latest_angles = [0.0, 0.0, 90.0, 0.0]
        self.latest_coords = [0.0, 0.0, 0.0, 0.0]
        # Protect short serial IO only (no process-wide file lock).
        self._serial_lock = threading.Lock()
        self._cb_group = ReentrantCallbackGroup()
        self._last_invalid_warn_time = 0.0

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_joint_states,
            callback_group=self._cb_group,
        )

        # Service servers
        self.srv_angles = self.create_service(
            SetAngles, 'set_angles', self.set_angles_callback, callback_group=self._cb_group
        )
        self.srv_coords = self.create_service(
            SetCoords, 'set_coords', self.set_coords_callback, callback_group=self._cb_group
        )
        self.srv_get_coords = self.create_service(
            GetCoords, 'get_coords', self.get_coords_callback, callback_group=self._cb_group
        )
        self.srv_get_angles = self.create_service(
            GetAngles, 'get_angles', self.get_angles_callback, callback_group=self._cb_group
        )

    def publish_joint_states(self):
        """Publish current joint states to the `joint_states` topic."""
        try:
            with self._serial_lock:
                angles = self.ua.get_angles_info()
            if not angles or not isinstance(angles, list) or len(angles) != 4 or not valid_angles(angles):
                now = time.time()
                if now - self._last_invalid_warn_time >= 2.0:
                    self.get_logger().warn("Skip invalid joint angles for RViz: %s" % angles)
                    self._last_invalid_warn_time = now
                return
            self.latest_angles = list(angles)
            display_angles = list(angles)
            display_angles[2] -= 90
            positions = [math.radians(a) for a in display_angles]

            joint_names = ["J1", "J2", "J3", "J4"]
            if len(positions) < len(joint_names):
                positions += [0.0] * (len(joint_names) - len(positions))

            js = JointState()
            js.header = Header()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = joint_names[:len(positions)]
            js.position = positions[:len(joint_names)]
            self.pub.publish(js)
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"Joint state publish error: {e}")

    def set_angles_callback(self, request, response):
        """Set joint angles via ROS2 service."""
        try:
            angles = [round(j, 2) for j in [request.joint_1, request.joint_2, request.joint_3, request.joint_4]]
            speed = request.speed
            self.get_logger().info(f"command start angles: {angles}")
            # pymycobot: _async=False = open-loop return immediately;
            # _async=True waits for serial "end" and blocks joint_states publishing.
            with self._serial_lock:
                self.ua.set_angles(angles, speed, _async=False)
            self.get_logger().info(f"command finish angles: {angles}")
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"SetJointAngles service error: {e}")
            response.flag = False
        return response

    def set_coords_callback(self, request, response):
        """Set end-effector coordinates via ROS2 service."""
        try:
            coords = [round(j, 2) for j in [request.x, request.y, request.z, request.rx]]
            self.get_logger().info(f"command start coords: {coords}")
            with self._serial_lock:
                self.ua.set_coords(coords, request.speed, _async=False)
            self.get_logger().info(f"command finish coords: {coords}")
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"Set coords failed: {e}")
            response.flag = False
        return response

    def get_coords_callback(self, request, response):
        """Get current end-effector coordinates."""
        coords = None
        try:
            for _ in range(3):
                with self._serial_lock:
                    data = self.ua.get_coords_info()

                if (
                    isinstance(data, list)
                    and len(data) == 4
                    and all(c != -1 for c in data)
                ):
                    coords = data
                    break

                time.sleep(0.05)

            if coords is not None:
                self.latest_coords = list(coords)
                response.x, response.y, response.z, response.rx = coords
            else:
                self.get_logger().warn(
                    "Failed to get coordinates after 3 retries; return latest valid coords: %s"
                    % self.latest_coords
                )
                response.x, response.y, response.z, response.rx = self.latest_coords

        except Exception:
            self.get_logger().error(f"GetCoords service error: {traceback.format_exc()}")

        return response

    def get_angles_callback(self, request, response):
        """Get current joint angles."""
        angles = None
        try:
            for _ in range(3):
                with self._serial_lock:
                    data = self.ua.get_angles_info()

                if (
                    isinstance(data, list)
                    and len(data) == 4
                    and all(a != -1 for a in data)
                ):
                    angles = data
                    break

                time.sleep(0.05)

            if angles is not None:
                self.latest_angles = list(angles)
                response.joint_1, response.joint_2, response.joint_3, response.joint_4 = angles
            else:
                self.get_logger().warn(
                    "Failed to get angles after 3 retries; return latest valid angles: %s"
                    % self.latest_angles
                )
                response.joint_1, response.joint_2, response.joint_3, response.joint_4 = self.latest_angles

        except Exception:
            self.get_logger().error(f"GetAngles service error: {traceback.format_exc()}")

        return response


def main(args=None):
    """Main entry point for running the MyCobotDriver node."""
    rclpy.init(args=args)
    node = MyCobotDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
