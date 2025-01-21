import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import pymycobot
from packaging import version
import os
import time
import fcntl

# min low version require
# MAX_REQUIRE_VERSION = '3.7.0'
MIN_REQUIRE_VERSION = '3.6.6'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be less than {} . The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot280 import MyCobot280

# Avoid serial port conflicts and need to be locked
def acquire(lock_file):
    open_mode = os.O_RDWR | os.O_CREAT | os.O_TRUNC
    try:
        fd = os.open(lock_file, open_mode)
    except OSError as e:
        print(f"Failed to open lock file {lock_file}: {e}")
        return None

    pid = os.getpid()
    lock_file_fd = None
    
    timeout = 15.0
    start_time = current_time = time.time()
    while current_time < start_time + timeout:
        try:
            # The LOCK_EX means that only one process can hold the lock
            # The LOCK_NB means that the fcntl.flock() is not blocking
            # and we are able to implement termination of while loop,
            # when timeout is reached.
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (IOError, OSError):
            time.sleep(1)
            pass
        else:
            lock_file_fd = fd
            # print(f"Lock acquired by PID: {pid}")
            break

        # print('pid waiting for lock:%d'% pid)
        current_time = time.time()
    if lock_file_fd is None:
        print(f"Failed to acquire lock after {timeout} seconds")
        os.close(fd)
    return lock_file_fd


def release(lock_file_fd):
    # Do not remove the lockfile:
    try:
        fcntl.flock(lock_file_fd, fcntl.LOCK_UN)
        os.close(lock_file_fd)
        # print("Lock released successfully")
    except OSError as e:
        print(f"Failed to release lock: {e}")

class Talker(Node):
    def __init__(self):
        super().__init__("real_listener")
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
   
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = MyCobot280(port, str(baud))

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
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
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
                    res[4] * (math.pi / 180),
                    res[5] * (math.pi / 180),
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
