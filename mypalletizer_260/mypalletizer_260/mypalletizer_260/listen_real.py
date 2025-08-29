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


def acquire(lock_file):
    """Acquire a file lock to prevent concurrent access.

    Args:
        lock_file (str): Path to the lock file.

    Returns:
        int | None: File descriptor if lock acquired, None if failed.
    """
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
    def __init__(self):
        super().__init__("real_listener")

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = MyPalletizer260(port, str(baud))

    def start(self):
        pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )
        rate = self.create_rate(30)  # 30hz

        # pub joint state
        joint_state_send = JointState()
        joint_state_send.header = Header()

        joint_state_send.name = [
            "joint1_to_base",
            "joint2_to_joint1",
            "joint3_to_joint2",
            # "joint4_to_joint3",
            "joint5_to_joint4",
        ]

        joint_state_send.velocity = [0.0, ]
        joint_state_send.effort = []

        while rclpy.ok():

            rclpy.spin_once(self)
            # get real angles from server.
            if self.mc:
                lock = acquire("/tmp/mycobot_lock")
                res = self.mc.get_angles()
                release(lock)
            try:
                if res[0] == res[1] == res[2] == 0.0:
                    continue
                radians_list = [
                    res[0] * (math.pi / 180),
                    res[1] * (math.pi / 180),
                    res[2] * (math.pi / 180),
                    res[3] * (math.pi / 180),
                ]
                # self.get_logger().info("res: {}".format(radians_list))

                # publish angles.
                joint_state_send.header.stamp = self.get_clock().now().to_msg()
                joint_state_send.position = radians_list
                pub.publish(joint_state_send)
                rate.sleep()
            except Exception as e:
                print(e)


def main(args=None):
    rclpy.init(args=args)

    talker = Talker()
    talker.start()
    rclpy.spin(talker)

    talker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
