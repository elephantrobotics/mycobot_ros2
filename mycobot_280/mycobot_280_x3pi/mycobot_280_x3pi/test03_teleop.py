import asyncio
import argparse
import sys
import termios
import tty
import time

from pymycobot.mycobot import MyCobot


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


class TeleopKeyboard:
    def __init__(self, port, baudrate):
        self.mc = MyCobot(port, baudrate)

        self.model = 0
        self.speed = 30
        self.change_percent = 2

        self.change_angle = 180 * self.change_percent / 100
        self.change_len = 250 * self.change_percent / 100

        self.init_pose = [[0, 0, 0, 0, 0, 0], self.speed]
        self.home_pose = [[0, 8, -127, 40, 0, 0], self.speed]


        self.record_coords = self.get_initial_coords()

    def get_initial_coords(self):
        while True:
            res = self.mc.get_coords()
            if res:
                break
            time.sleep(0.1)

        return [res, self.speed, self.model]

    def print_status(self):
        print("\r current coords: %s" % self.record_coords)

    def move_to(self, delta_x):
        self.record_coords[0][0] += delta_x
        self.mc.send_coords(*self.record_coords)

    async def keyboard_listener(self):
        with termios.tcgetattr(sys.stdin):
            tty.setcbreak(sys.stdin)
            while True:
                key = await asyncio.to_thread(sys.stdin.read, 1)
                if key == "q":
                    self.mc.release_all_servos()
                    break
                elif key in ["w", "W"]:
                    record_coords[0][0] += self.change_len
                    self.mc.send_coords(*record_coords)
                elif key in ["s", "S"]:
                    record_coords[0][0] -= self.change_len
                    self.mc.send_coords(*record_coords)
                elif key in ["a", "A"]:
                    record_coords[0][1] -= self.change_len
                    self.mc.send_coords(*record_coords)
                elif key in ["d", "D"]:
                    record_coords[0][1] += self.change_len
                    self.mc.send_coords(*record_coords)
                elif key in ["z", "Z"]:
                    record_coords[0][2] -= self.change_len
                    self.mc.send_coords(*record_coords)
                elif key in ["x", "X"]:
                    record_coords[0][2] += self.change_len
                    self.mc.send_coords(*record_coords)
                elif key in ["u", "U"]:
                    record_coords[0][3] += self.change_angle
                    self.mc.send_coords(*record_coords)
                elif key in ["j", "J"]:
                    record_coords[0][3] -= self.change_angle
                    self.mc.send_coords(*record_coords)
                elif key in ["i", "I"]:
                    record_coords[0][4] += self.change_angle
                    self.mc.send_coords(*record_coords)
                elif key in ["k", "K"]:
                    record_coords[0][4] -= self.change_angle
                    self.mc.send_coords(*record_coords)
                elif key in ["o", "O"]:
                    record_coords[0][5] += self.change_angle
                    self.mc.send_coords(*record_coords)
                elif key in ["l", "L"]:
                    record_coords[0][5] -= self.change_angle
                    self.mc.send_coords(*record_coords)
                elif key in ["g", "G"]:
                    self.mc.set_gripper_state(0, 30)
                elif key in ["h", "H"]:
                    self.mc.set_gripper_state(1, 30)
                elif key == "1":
                    self.mc.send_angles(*self.init_pose)
                    record_coords = [self.res, self.speed, self.model]
                elif key in "2":
                    self.mc.send_angles(*self.home_pose)
                    record_coords = [self.res, self.speed, self.model]
                elif key in "3":
                    rep = self.mc.get_angles()
                    self.home_pose[0] =rep
                else:
                    continue

                self.print_status()
                await asyncio.sleep(0.1)

    async def run(self):
        print("Use W/S to move the robot along X-axis, Q to quit")
        print(msg)
        print("currently:\tspeed: %s\tchange percent: %s  " % (self.speed, self.change_percent))
        while True:
            try:
                await self.keyboard_listener()
            except Exception as e:
                print(e)
                continue


def parse_args():
    parser = argparse.ArgumentParser(description="Teleoperate a MyCobot robot using the keyboard")
    parser.add_argument("--port", type=str, default="/dev/ttyS3", help="the serial port used to communicate with the robot")
    parser.add_argument("--baudrate", type=int, default=1000000, help="the baudrate used to communicate with the robot")
    return parser.parse_args()


def main():
    args = parse_args()
    teleop_keyboard = TeleopKeyboard(args.port, args.baudrate)
    asyncio.run(teleop_keyboard.run())


if __name__ == '__main__':
    main()
