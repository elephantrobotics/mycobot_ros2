import rclpy
import time
import os
import fcntl
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import math
import pymycobot
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '3.8.0'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280RDKX5

# Avoid serial port conflicts and need to be locked
def acquire(lock_file):
    open_mode = os.O_RDWR | os.O_CREAT | os.O_TRUNC
    fd = os.open(lock_file, open_mode)

    pid = os.getpid()
    lock_file_fd = None
    
    timeout = 50.0
    start_time = current_time = time.time()
    while current_time < start_time + timeout:
        try:
            # The LOCK_EX means that only one process can hold the lock
            # The LOCK_NB means that the fcntl.flock() is not blocking
            # and we are able to implement termination of while loop,
            # when timeout is reached.
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (IOError, OSError):
            pass
        else:
            lock_file_fd = fd
            break

        # print('pid waiting for lock:%d'% pid)
        time.sleep(1.0)
        current_time = time.time()
    if lock_file_fd is None:
        os.close(fd)
    return lock_file_fd


def release(lock_file_fd):
    # Do not remove the lockfile:
    fcntl.flock(lock_file_fd, fcntl.LOCK_UN)
    os.close(lock_file_fd)
    return None


class Talker(Node):
    def __init__(self):
        super().__init__("follow_display")
        self.declare_parameter('port', '/dev/ttyS1')
        self.declare_parameter('baud', 1000000)
   
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = MyCobot280RDKX5(port,str(baud))
        if self.mc:
            lock = acquire("/tmp/mycobot_lock")
            self.mc.release_all_servos()
            release(lock)

    def start(self):
        pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )
        pub_marker = self.create_publisher(
            msg_type=Marker,
            topic="visualization_marker",
            qos_profile=10
        )
        rate = self.create_rate(30)

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
        joint_state_send.velocity = [0.0,]
        joint_state_send.effort = []

        marker_ = Marker()
        marker_.header.frame_id = "/joint1"
        marker_.ns = "my_namespace"

        while rclpy.ok():
            rclpy.spin_once(self)
            joint_state_send.header.stamp = self.get_clock().now().to_msg()
            
            try:
                if self.mc:
                    lock = acquire("/tmp/mycobot_lock")
                    angles = self.mc.get_angles()
                    release(lock)
                data_list = []
                radians = [round(math.radians(angle), 3) for angle in angles]
                for _, value in enumerate(radians):
                    data_list.append(value)
            
                # self.get_logger().info('angles: {}'.format([round(math.degrees(angle), 2) for angle in data_list]))
                joint_state_send.position = data_list

                pub.publish(joint_state_send)
                
                if self.mc:
                    lock = acquire("/tmp/mycobot_lock")
                    coords = self.mc.get_coords()
                    release(lock)
                # coords = []

                # marker
                marker_.header.stamp = self.get_clock().now().to_msg()
                marker_.type = marker_.SPHERE
                marker_.action = marker_.ADD
                marker_.scale.x = 0.04
                marker_.scale.y = 0.04
                marker_.scale.z = 0.04

                # marker position initial
                # self.get_logger().info('{}'.format(coords))
                
                if not coords:
                    coords = [0, 0, 0, 0, 0, 0]
                    # self.get_logger().info("error [101]: can not get coord values")
                if self.mc:
                    lock = acquire("/tmp/mycobot_lock")
                    marker_.pose.position.x = coords[1] / 1000 * -1
                    marker_.pose.position.y = coords[0] / 1000
                    marker_.pose.position.z = coords[2] / 1000
                    release(lock)

                marker_.color.a = 1.0
                marker_.color.g = 1.0
                pub_marker.publish(marker_)

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

