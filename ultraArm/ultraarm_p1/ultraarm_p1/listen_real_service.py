import math
import time
import os
import fcntl
import rclpy
import traceback
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from ultraarm_p1_interfaces.srv import SetAngles, SetCoords, GetCoords, GripperStatus, GetAngles, GetGripperValue
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
            time.sleep(0.001)
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


class MyCobotDriver(Node):
    """ROS2 node for controlling the MyCobotPro450 robot arm.

    Provides publishers for joint states and services for controlling
    joints, coordinates, grippers, and pumps.
    """

    def __init__(self):
        """Initialize MyCobotDriver node, publishers, and services."""
        super().__init__('mycobot_driver_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 1000000)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.ua = UltraArmP1(port, baud)
        self.ua.set_joint_enable(0)
        
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1, self.publish_joint_states)

        # Service servers
        self.srv_angles = self.create_service(SetAngles, 'set_angles', self.set_angles_callback)
        self.srv_coords = self.create_service(SetCoords, 'set_coords', self.set_coords_callback)
        self.srv_get_coords = self.create_service(GetCoords, 'get_coords', self.get_coords_callback)
        self.srv_get_angles = self.create_service(GetAngles, 'get_angles', self.get_angles_callback)

    def publish_joint_states(self):
        """Publish current joint states to the `joint_states` topic."""
        lock = None
        try:
            lock = acquire('/tmp/mycobot_lock')
            angles = self.ua.get_angles_info()
            release(lock)
            # self.get_logger().info(f"Raw angles from MyCobot: {angles}")
            if not angles or not isinstance(angles, list) or len(angles) != 4:
                self.get_logger().warn("Failed to get valid joint angles, fallback to [-1] * 4.")
                return
            angles[2] -= 90
            positions = [math.radians(a) for a in angles]
                
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
        finally:
            if lock is not None:
                release(lock)

    def set_angles_callback(self, request, response):
        """Set joint angles via ROS2 service.

        Args:
            request (SetAngles.Request): The request object containing target joint
                angles (joint_1 to joint_4) and movement speed.
            response (SetAngles.Response): The response object that will be updated
                with the operation result.

        Returns:
            SetAngles.Response: The response with a boolean `flag` indicating
            whether the operation succeeded.
        """
        lock = None
        try:
            lock = acquire('/tmp/mycobot_lock')
            if lock is None:
                self.get_logger().error("Failed to acquire serial lock")
                response.flag = False
                return response
            angles = [round(j, 2) for j in [request.joint_1, request.joint_2, request.joint_3, request.joint_4]]
            speed = request.speed
            self.get_logger().info(f"command start angles: {angles}")
            self.ua.set_angles(angles, speed, _async=False)
            release(lock)
            self.get_logger().info(f"command finish angles: {angles}")
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"SetJointAngles service error: {e}")
            response.flag = False
        finally:
            if lock is not None:
                release(lock)
        return response

    def set_coords_callback(self, request, response):
        """Set end-effector coordinates via ROS2 service.

        Args:
            request (SetCoords.Request): The request object containing target coordinates
                (x, y, z, rx), motion speed.
            response (SetCoords.Response): The response object that will be updated
                with the operation result.

        Returns:
            SetCoords.Response: The response with a boolean `flag` indicating
            whether the operation succeeded.
        """
        lock = None
        try:
            lock = acquire('/tmp/mycobot_lock')
            if lock is None:
                self.get_logger().error("Failed to acquire serial lock")
                response.flag = False
                return response
            coords = [round(j, 2) for j in [request.x, request.y, request.z, request.rx]]
            self.get_logger().info(f"command start coords: {coords}")
            self.ua.set_coords(coords, request.speed, _async=False)
            release(lock)
            self.get_logger().info(f"command finish coords: {coords}")
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"Set coords failed: {e}")
            response.flag = False
        finally:
            if lock is not None:
                release(lock)
        return response

    def get_coords_callback(self, request, response):
        """Get current end-effector coordinates.

        Args:
            request (GetCoords.Request): The request object (unused in this service).
            response (GetCoords.Response): The response object that will be updated
                with the current robot end-effector coordinates.

        Returns:
            GetCoords.Response: The response containing the current coordinates
            (x, y, z, rx).
        """
        lock = None
        coords = None
        try:
            lock = acquire('/tmp/mycobot_lock')
            if lock is None:
                self.get_logger().error("GetCoords failed: failed to acquire serial lock")
                return response

            for _ in range(3):
                data = self.ua.get_coords_info()

                if (
                    isinstance(data, list)
                    and len(data) == 4
                    and all(c != -1 for c in data)
                ):
                    coords = data
                    break

                # self.get_logger().warn(f"Invalid coords read: {data}")
                time.sleep(0.05)

            if coords is not None:
                response.x, response.y, response.z, response.rx = coords
            else:
                self.get_logger().error("Failed to get coordinates after 3 retries.")

        except Exception:
            self.get_logger().error(f"GetCoords service error: {traceback.format_exc()}")

        finally:
            if lock is not None:
                release(lock)

        return response

    def get_angles_callback(self, request, response):
        """Get current joint angles.

        Args:
            request (GetAngles.Request): The request object (unused in this service).
            response (GetAngles.Response): The response object that will be updated
                with the current joint angles.

        Returns:
            GetAngles.Response: The response containing four joint angles.
        """
        lock = None
        angles = None
        try:
            lock = acquire('/tmp/mycobot_lock')
            if lock is None:
                self.get_logger().error("GetAngles failed: failed to acquire serial lock")
                return response

            for _ in range(3):
                data = self.ua.get_angles_info()

                if (
                    isinstance(data, list)
                    and len(data) == 4
                    and all(a != -1 for a in data)
                ):
                    angles = data
                    break

                # self.get_logger().warn(f"Invalid angles read: {data}")
                time.sleep(0.05)

            if angles is not None:
                response.joint_1, response.joint_2, response.joint_3, response.joint_4 = angles
            else:
                self.get_logger().error("Failed to get angles after 3 retries.")

        except Exception:
            self.get_logger().error(f"GetAngles service error: {traceback.format_exc()}")

        finally:
            if lock is not None:
                release(lock)

        return response


def main(args=None):
    """Main entry point for running the MyCobotDriver node."""
    rclpy.init(args=args)
    node = MyCobotDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
