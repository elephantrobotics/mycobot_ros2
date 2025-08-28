import math
import time
import os
import fcntl
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from mycobot_interfaces.srv import SetAngles, SetCoords, GetCoords, GripperStatus, GetAngles
import pymycobot
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280


def acquire(lock_file):
    try:
        fd = os.open(lock_file, os.O_RDWR | os.O_CREAT | os.O_TRUNC)
    except OSError as e:
        return None
    timeout = 50.0
    start_time = current_time = time.time()
    while current_time < start_time + timeout:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return fd
        except:
            time.sleep(1)
            current_time = time.time()
    os.close(fd)
    return None


def release(fd):
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)
    except:
        pass


class MyCobotDriver(Node):
    def __init__(self):
        super().__init__('mycobot_driver_node')
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 1000000)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.mc = MyCobot280(port, str(baud))
        if self.mc.get_fresh_mode() !=1:
            self.mc.set_fresh_mode(1)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        self.srv_angles = self.create_service(SetAngles, 'set_angles', self.set_angles_callback)  # 设置角度服务
        self.srv_coords = self.create_service(SetCoords, 'set_coords', self.set_coords_callback)  # 设置坐标服务
        self.srv_get_coords = self.create_service(GetCoords, 'get_coords', self.get_coords_callback)  # 获取坐标服务
        self.srv_get_angles = self.create_service(GetAngles, 'get_angles', self.get_angles_callback)  # 获取坐标服务
        self.srv_gripper = self.create_service(GripperStatus, 'set_gripper', self.set_gripper_callback)  # 夹爪控制服务

    def publish_joint_states(self):
        try:
            lock = acquire('/tmp/mycobot_lock')
            angles = self.mc.get_angles()
            release(lock)
            if not angles or angles[0:3] == [0.0, 0.0, 0.0] or len(angles) != 6:
                return
            js = JointState()
            js.header = Header()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [
                "joint2_to_joint1", 
                "joint3_to_joint2", 
                "joint4_to_joint3",
                "joint5_to_joint4", 
                "joint6_to_joint5", 
                "joint6output_to_joint6"
            ]
            js.position = [math.radians(a) for a in angles]
            self.pub.publish(js)
        except Exception as e:
            self.get_logger().error(f"Joint state publish error: {e}")

    def set_angles_callback(self, request, response):
        try:
            lock = acquire('/tmp/mycobot_lock')

            # 角度列表
            angles = [
                request.joint_1,
                request.joint_2,
                request.joint_3,
                request.joint_4,
                request.joint_5,
                request.joint_6,
            ]
            speed = request.speed

            # 发送角度
            self.mc.send_angles(angles, speed)
            release(lock)

            response.flag = True  # srv返回是bool flag
        except Exception as e:
            release(lock)
            self.get_logger().error(f"SetJointAngles service error: {e}")
            response.flag = False
        return response

    def set_coords_callback(self, request, response):
        try:
            lock = acquire('/tmp/mycobot_lock')
            coords = [request.x, request.y, request.z, request.rx, request.ry, request.rz]
            self.mc.send_coords(coords, request.speed, request.model)
            release(lock)
            response.flag = True
        except Exception as e:
            self.get_logger().error(f"Set coords failed: {e}")
            response.flag = False
        return response

    def get_coords_callback(self, request, response):
        try:
            lock = acquire('/tmp/mycobot_lock')
            coords = self.mc.get_coords()
            release(lock)
            if not coords or len(coords) != 6:
                return
            if coords:
                response.x = coords[0]
                response.y = coords[1]
                response.z = coords[2]
                response.rx = coords[3]
                response.ry = coords[4]
                response.rz = coords[5]
            else:
                self.get_logger().error("Failed to get coordinates.")
        except Exception as e:
            self.get_logger().error(f"GetCoords service error: {e}")
        return response

    def get_angles_callback(self, request, response):
        try:
            lock = acquire('/tmp/mycobot_lock')
            angles = self.mc.get_angles()
            release(lock)

            if angles:
                response.joint_1 = angles[0]
                response.joint_2 = angles[1]
                response.joint_3 = angles[2]
                response.joint_4 = angles[3]
                response.joint_5 = angles[4]
                response.joint_6 = angles[5]
            else:
                self.get_logger().error("Failed to get angles.")
        except Exception as e:
            self.get_logger().error(f"GetAngles service error: {e}")
        return response

    def set_gripper_callback(self, request, response):
        try:
            lock = acquire('/tmp/mycobot_lock')
            # 设置固定的速度
            speed = 80

            # 根据状态控制夹爪
            if request.status:
                self.mc.set_gripper_state(0, speed, 1)  # 打开夹爪
            else:
                self.mc.set_gripper_state(1, speed, 1)  # 关闭夹爪
            release(lock)
            response.flag = True
        except Exception as e:
            self.get_logger().error(f"SetGripper service error: {e}")
            response.flag = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MyCobotDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
