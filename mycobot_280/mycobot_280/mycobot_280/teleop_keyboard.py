from __future__ import print_function
import sys
import termios
import tty
import time
import os
import pymycobot
from packaging import version
import rclpy
from rclpy.node import Node
# min low version require
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280

msg = """\
Mycobot Teleop Keyboard Controller
---------------------------
Movimg options(control coordinations [x,y,z,rx,ry,rz]):
              w(x+)

    a(y-)     s(x-)     d(y+)

    z(z-) x(z+)

u(rx+)   i(ry+)   o(rz+)
j(rx-)   k(ry-)   l(rz-)

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

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('mycobot_teleop_keyboard')

        # 声明参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        # 获取参数
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.get_logger().info(f'使用端口: {port}, 波特率: {baud}')

        # 初始化机械臂
        self.mc = MyCobot280(port, baud)
        time.sleep(0.05)
        self.mc.set_fresh_mode(1)
        time.sleep(0.05)
        self.teleop_keyboard()
    
    def teleop_keyboard(self):
        # robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        # robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        # if robot_m5:
        #     port = robot_m5
        # else:
        #     port = robot_wio

        model = 1
        speed = 50
        change_percent = 5

        change_angle = 180 * change_percent / 100
        change_len = 250 * change_percent / 100

        init_pose = [[0, 0, 0, 0, 0, 0], speed]
        home_pose = [[0, 8, -127, 40, 0, 0], speed]

        self.mc.send_angles(*init_pose)

        while True:
            res = self.mc.get_coords()
            if res:
                break
            time.sleep(0.1)
        
        record_coords = [res, speed, model]

        try:
            print(msg)
            print(vels(speed, change_percent))
            while 1:
                try:
                    print("\r current coords: %s" % record_coords)
                    with Raw(sys.stdin):
                        key = sys.stdin.read(1)
                    if key == "q":
                        self.mc.release_all_servos()
                        break
                    elif key in ["w", "W"]:
                        record_coords[0][0] += change_len
                        self.mc.send_coords(*record_coords)
                    elif key in ["s", "S"]:
                        record_coords[0][0] -= change_len
                        self.mc.send_coords(*record_coords)
                    elif key in ["a", "A"]:
                        record_coords[0][1] -= change_len
                        self.mc.send_coords(*record_coords)
                    elif key in ["d", "D"]:
                        record_coords[0][1] += change_len
                        self.mc.send_coords(*record_coords)
                    elif key in ["z", "Z"]:
                        record_coords[0][2] -= change_len
                        self.mc.send_coords(*record_coords)
                    elif key in ["x", "X"]:
                        record_coords[0][2] += change_len
                        self.mc.send_coords(*record_coords)
                    elif key in ["u", "U"]:
                        record_coords[0][3] += change_angle
                        self.mc.send_coords(*record_coords)
                    elif key in ["j", "J"]:
                        record_coords[0][3] -= change_angle
                        self.mc.send_coords(*record_coords)
                    elif key in ["i", "I"]:
                        record_coords[0][4] += change_angle
                        self.mc.send_coords(*record_coords)
                    elif key in ["k", "K"]:
                        record_coords[0][4] -= change_angle
                        self.mc.send_coords(*record_coords)
                    elif key in ["o", "O"]:
                        record_coords[0][5] += change_angle
                        self.mc.send_coords(*record_coords)
                    elif key in ["l", "L"]:
                        record_coords[0][5] -= change_angle
                        self.mc.send_coords(*record_coords)
                    elif key in ["g", "G"]:
                        self.mc.set_gripper_state(0, 80)
                    elif key in ["h", "H"]:
                        self.mc.set_gripper_state(1, 80)
                    elif key in ["b", "B"]:
                        self.mc.set_basic_output(2, 0)
                        self.mc.set_basic_output(5, 0)
                    elif key in ["m", "M"]:
                        self.mc.set_basic_output(2, 1)
                        self.mc.set_basic_output(5, 1)
                    elif key == "1":
                        self.mc.send_angles(*init_pose)
                    elif key in "2":
                        self.mc.send_angles(*home_pose)
                    elif key in "3":
                        rep = self.mc.get_angles()
                        home_pose[0] =rep
                    else:
                        continue

                except Exception as e:
                    # print(e)
                    continue

                time.sleep(1)

        except Exception as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
