from __future__ import print_function
import sys
import rclpy
from rclpy.node import Node
import termios
import tty
import time
from mycobot_interfaces.srv import SetAngles, SetCoords, GripperStatus, GetCoords, GetAngles


msg = """\
Mycobot Teleop Keyboard Controller
---------------------------
Movimg options (control coordinations [x,y,z,rx,ry,rz]):
              w(x+)

    a(y-)     s(x-)     d(y+)

    z(z-) x(z+)

u(rx+)   i(ry+)   o(rz+)
j(rx-)   k(ry-)   l(rz-)

+/- : Increase/decrease movement step size

Gripper control:
    g - open
    h - close

Other:
    1 - Go to init pose
    2 - Go to home pose
    q - Quit
"""

COORD_LIMITS = {
    'x': (-350, 350),
    'y': (-350, 350),
    'z': (-70, 523.9),
    'rx': (-180, 180),
    'ry': (-180, 180),
    'rz': (-180, 180)
}


def vels(speed, turn):
    return "currently:\tspeed: %s\tchange percent: %s  " % (speed, turn)


class Raw(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()

    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_client')

        # 客户端请求
        self.set_angles_client = self.create_client(SetAngles, '/set_angles')
        self.set_coords_client = self.create_client(SetCoords, '/set_coords')
        self.set_gripper_client = self.create_client(GripperStatus, '/set_gripper')
        self.get_coords_client = self.create_client(GetCoords, '/get_coords')
        self.get_angles_client = self.create_client(GetAngles, '/get_angles')

        # 等待服务上线
        while not self.set_angles_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.speed = 50  # 移动速度
        self.model = 1  # 运动模式
        self.change_percent = 5  # 改变的百分比

        self.change_angle = 180 * self.change_percent / 100
        self.change_len = 250 * self.change_percent / 100

        self.init_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.home_pose = [0.0, 8.0, -127.0, 40.0, 0.0, 0.0]

        self.record_coords = self.get_initial_coords()

    def get_initial_coords(self):
        # 获取坐标服务
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
        request = GetAngles.Request()
        future = self.get_angles_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return [future.result().joint_1, future.result().joint_2, future.result().joint_3, future.result().joint_4,
                     future.result().joint_5, future.result().joint_6]
        else:
            self.get_logger().error("Failed to get angles")
            return [-1, -1, -1, -1, -1, -1]

    def print_status(self):
        coords = self.record_coords[0]
        print("\r current coords: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]" % tuple(coords))

    def send_coords(self):
        coords = self.record_coords[0]
        # 检查坐标是否超出限位
        for i, axis in enumerate(['x', 'y', 'z', 'rx', 'ry', 'rz']):
            min_limit, max_limit = COORD_LIMITS[axis]
            if coords[i] < min_limit or coords[i] > max_limit:
                self.get_logger().warn(f"{axis} value {coords[i]} exceeds the limit range [{min_limit}, {max_limit}], unable to send coordinates")
                return  # 如果有任何坐标超出限位，直接返回，不发送请求
        request = SetCoords.Request()
        request.x = coords[0]
        request.y = coords[1]
        request.z = coords[2]
        request.rx = coords[3]
        request.ry = coords[4]
        request.rz = coords[5]
        request.speed = self.record_coords[1]
        request.model = self.record_coords[2]

        future = self.set_coords_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            pass
            # self.get_logger().info(f"Coords set: {request}")
        else:
            self.get_logger().error('Failed to set coordinates')

    def send_angles(self, angles):
        request = SetAngles.Request()
        request.joint_1 = angles[0]
        request.joint_2 = angles[1]
        request.joint_3 = angles[2]
        request.joint_4 = angles[3]
        request.joint_5 = angles[4]
        request.joint_6 = angles[5]
        request.speed = self.speed

        future = self.set_angles_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            pass
            # self.get_logger().info(f"Angles set: {request}")
        else:
            self.get_logger().error('Failed to set angles')

    def set_gripper(self, status):
        request = GripperStatus.Request()
        request.status = status

        future = self.set_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            pass
            # self.get_logger().info(f"Gripper status set: {status}")
        else:
            self.get_logger().error('Failed to control gripper')

    def keyboard_listener(self):
        print(msg)
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
                elif key == "1":
                    self.send_angles(self.init_pose)
                    time.sleep(2)
                    self.record_coords = self.get_initial_coords()
                elif key == "2":
                    self.send_angles(self.home_pose)
                    time.sleep(2)
                    self.record_coords = self.get_initial_coords()
                elif key == "3":
                    self.home_pose = self.get_initial_angles()  # 保存当前姿态为新的home姿态
                    print(f"New home pose saved: {self.home_pose}")
                elif key == '+':
                    self.change_percent = min(self.change_percent + 1, 20)
                    self.change_angle = 180 * self.change_percent / 100
                    self.change_len = 250 * self.change_percent / 100
                    print("Increase change_percent to %d%%, move step: %.1f mm" % (self.change_percent, self.change_len))
                elif key == '-':
                    self.change_percent = max(self.change_percent - 1, 1)
                    self.change_angle = 180 * self.change_percent / 100
                    self.change_len = 250 * self.change_percent / 100
                    print("Decrease change_percent to %d%%, move step: %.1f mm" % (self.change_percent, self.change_len))

                else:
                    continue

                self.print_status()
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Error in key processing: {e}")
                continue
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    teleop_keyboard = TeleopKeyboard()
    teleop_keyboard.keyboard_listener()
    rclpy.shutdown()


if __name__ == '__main__':
    main()