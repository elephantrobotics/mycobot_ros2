from __future__ import print_function
from pymycobot.mycobot import MyCobot
# from pymycobot.mycobotsocket import MyCobotSocket
import sys
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3


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



class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def vels(self, speed, turn):
        return Twist(linear=Vector3(x=speed), angular=Vector3(z=turn))

    def teleop_keyboard(self):
        mc = MyCobot("/dev/ttyS3", 1000000)

        model = 1
        speed = 30
        change_percent = 2

        change_angle = 180 * change_percent / 100
        change_len = 250 * change_percent / 100

        init_pose = [[0, 0, 0, 0, 0, 0], speed]
        home_pose = [[0, 8, -127, 40, 0, 0], speed]

        mc.send_angles(*init_pose)

        while True:
            res = mc.get_coords()
            if res:
                break
            time.sleep(0.1)

        record_coords = [res, speed, model]

        try:
            print(msg)
            print(self.vels(speed, change_percent))
            while rclpy.ok():
                try:
                    print("\r current coords: %s" % record_coords)
                    with Raw(sys.stdin):
                        key = sys.stdin.read(1)
                    if key == "q":
                        mc.release_all_servos()
                        break
                    elif key in ["w", "W"]:
                        record_coords[0][0] += change_len
                        mc.send_coords(*record_coords)
                    elif key in ["s", "S"]:
                        record_coords[0][0] -= change_len
                        mc.send_coords(*record_coords)
                    elif key in ["a", "A"]:
                        record_coords[0][1] -= change_len
                        mc.send_coords(*record_coords)
                    elif key in ["d", "D"]:
                        record_coords[0][1] += change_len
                        mc.send_coords(*record_coords)
                    elif key in ["z", "Z"]:
                        record_coords[0][2] -= change_len
                        mc.send_coords(*record_coords)
                    elif key in ["x", "X"]:
                        record_coords[0][2] += change_len
                        mc.send_coords(*record_coords)
                    elif key in ["u", "U"]:
                        record_coords[0][3] += change_angle
                        mc.send_coords(*record_coords)
                    elif key in ["j", "J"]:
                        record_coords[0][3] -= change_angle
                        mc.send_coords(*record_coords)
                    elif key in ["i", "I"]:
                        record_coords[0][4] += change_angle
                        mc.send_coords(*record_coords)
                    elif key in ["k", "K"]:
                        record_coords[0][4] -= change_angle
                        mc.send_coords(*record_coords)
                    elif key in ["o", "O"]:
                        record_coords[0][5] += change_angle
                        mc.send_coords(*record_coords)
                    elif key in ["l", "L"]:
                        record_coords[0][5] -= change_angle
                        mc.send_coords(*record_coords)
                    elif key in ["g", "G"]:
                        mc.set_gripper_state(0, 30)
                    elif key in ["h", "H"]:
                        mc.set_gripper_state(1, 30)
                    elif key == "1":
                        mc.send_angles(*init_pose)
                        record_coords = [res, speed, model]
                    elif key in "2":
                        mc.send_angles(*home_pose)
                        record_coords = [res, speed, model]
                    elif key in "3":
                        rep = mc.get_angles()
                        home_pose[0] =rep
                    else:
                        continue
                except Exception as e:
                    # print(e)
                    continue
                time.sleep(1/30) # 控制频率为30Hz
                self.pub.publish(self.vels(speed, change_percent))
        except Exception as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)
    teleop_keyboard = TeleopKeyboard()
    teleop_keyboard.teleop_keyboard()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
