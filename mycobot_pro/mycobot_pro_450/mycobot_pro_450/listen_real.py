import fcntl
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import pymycobot
from packaging import version

# Minimum required pymycobot version
MIN_REQUIRE_VERSION = '3.9.9'

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
    from pymycobot import Pro450Client


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


class Talker(Node):
    """ROS2 node that publishes real-time joint angles from MyCobotPro450."""

    def __init__(self):
        """Initialize the Talker node and connect to the MyCobotPro450 robot."""
        super().__init__("real_listener")

        self.declare_parameter('ip', '192.168.0.232')
        self.declare_parameter('port', 4500)

        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value

        self.get_logger().info("ip:%s, port:%d" % (ip, port))
        self.mycobot_450 = Pro450Client(ip, port)

    def start(self):
        """Start publishing joint states at 30 Hz.

        Publishes:
            JointState messages to the 'joint_states' topic with
            current angles in radians for all six joints.
        """
        pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )
        rate = self.create_rate(30)  # 30 Hz

        # Initialize joint state message
        joint_state_send = JointState()
        joint_state_send.header = Header()
        joint_state_send.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        joint_state_send.velocity = [0.0]
        joint_state_send.effort = []

        while rclpy.ok():
            rclpy.spin_once(self)

            # Get real angles from MyCobotPro450
            if self.mycobot_450:
                lock = acquire("/tmp/mycobot_lock")
                res = self.mycobot_450.get_angles()
                release(lock)
                self.get_logger().info(f"Invalid angles received: {res}")

            try:
                # Skip invalid readings
                if not res or not isinstance(res, list) or len(res) != 6:
                    res = [-1] * 6
                    self.get_logger().warn(f"Invalid angles received: {res}")
                    continue

                # Convert angles to radians
                radians_list = [
                    res[0] * (math.pi / 180),
                    res[1] * (math.pi / 180),
                    res[2] * (math.pi / 180),
                    res[3] * (math.pi / 180),
                    res[4] * (math.pi / 180),
                    res[5] * (math.pi / 180),
                ]

                # Publish joint states
                joint_state_send.header.stamp = self.get_clock().now().to_msg()
                joint_state_send.position = radians_list
                pub.publish(joint_state_send)
                rate.sleep()
            except Exception as e:
                print(e)


def main(args=None):
    """Main function to run the Talker node.

    Args:
        args (list, optional): Command-line arguments for ROS2. Defaults to None.
    """
    rclpy.init(args=args)

    talker = Talker()
    talker.start()
    rclpy.spin(talker)

    talker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
