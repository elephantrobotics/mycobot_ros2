from __future__ import print_function
import sys
import termios
import tty
import time
import pymycobot
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '3.6.0'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot320 import MyCobot320

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


def teleop_keyboard():
    mc = MyCobot320("/dev/ttyAMA0", 115200)

    model = 0
    speed = 30
    change_percent = 3

    change_angle = 180 * change_percent / 100
    change_len = 250 * change_percent / 100

    init_pose = [[0, 0, 0, 0, 0, 0], speed]
    home_pose = [[0, 8, -127, 40, 90, 0], speed]

    mc.send_angles(*init_pose)

    while True:
        res = mc.get_coords()
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
                    mc.switch_gripper(True)
                elif key in ["h", "H"]:
                    mc.switch_gripper(False)
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

            time.sleep(1)

    except Exception as e:
        print(e)


def main():
    teleop_keyboard()


if __name__ == "__main__":
    main()
