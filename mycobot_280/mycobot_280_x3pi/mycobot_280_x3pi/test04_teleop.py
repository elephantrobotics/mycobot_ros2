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


class TeleopKeyboard:
    def __init__(self, port, baudrate):
        self.mc = MyCobot(port, baudrate)

        self.model = 0
        self.speed = 40
        self.change_percent = 5

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
        print("\r current coords2: %s" % self.record_coords)

    def keyboard_listener(self):
        print(msg)
        print(vels(self.speed, self.change_percent))
        while True:
            try:
                print("\r current coords: %s" % self.record_coords)
                with Raw(sys.stdin):
                    key = sys.stdin.read(1)
                if key == "q":
                    self.mc.release_all_servos()
                    break
                elif key in ["w", "W"]:
                    self.record_coords[0][0] += self.change_len
                    print('x+x+_coords:', self.record_coords)
                    self.mc.send_coords(*self.record_coords)
                    print('x+x+_end')
                elif key in ["s", "S"]:
                    self.record_coords[0][0] -= self.change_len
                    print('x-x-_coords:', self.record_coords)
                    self.mc.send_coords(*self.record_coords)
                    print('x-x-_end')
                elif key in ["a", "A"]:
                    self.record_coords[0][1] -= self.change_len
                    print('y-y-_coords:', self.record_coords)
                    self.mc.send_coords(*self.record_coords)
                    print('y-y-_end')
                elif key in ["d", "D"]:
                    self.record_coords[0][1] += self.change_len
                    self.mc.send_coords(*self.record_coords)
                elif key in ["z", "Z"]:
                    self.record_coords[0][2] -= self.change_len
                    self.mc.send_coords(*self.record_coords)
                elif key in ["x", "X"]:
                    self.record_coords[0][2] += self.change_len
                    self.mc.send_coords(*self.record_coords)
                elif key in ["u", "U"]:
                    self.record_coords[0][3] += self.change_angle
                    self.mc.send_coords(*self.record_coords)
                elif key in ["j", "J"]:
                    self.record_coords[0][3] -= self.change_angle
                    self.mc.send_coords(*self.record_coords)
                elif key in ["i", "I"]:
                    self.record_coords[0][4] += self.change_angle
                    self.mc.send_coords(*self.record_coords)
                elif key in ["k", "K"]:
                    self.record_coords[0][4] -= self.change_angle
                    self.mc.send_coords(*self.record_coords)
                elif key in ["o", "O"]:
                    self.record_coords[0][5] += self.change_angle
                    self.mc.send_coords(*self.record_coords)
                elif key in ["l", "L"]:
                    self.record_coords[0][5] -= self.change_angle
                    self.mc.send_coords(*self.record_coords)
                elif key in ["g", "G"]:
                    self.mc.set_gripper_state(0, 30)
                elif key in ["h", "H"]:
                    self.mc.set_gripper_state(1, 30)
                elif key == "1":
                    self.mc.send_angles(*self.init_pose)
                    print('111_pre_coords:', self.record_coords)
                    self.record_coords = self.get_initial_coords()
                    print('111_end_coords:', self.record_coords)
                elif key in "2":
                    self.mc.send_angles(*self.home_pose)
                    print('222_pre_coords:', self.record_coords)
                    self.record_coords = self.get_initial_coords()
                    print('222_end_coords:', self.record_coords)
                elif key in "3":
                    rep = self.mc.get_angles()
                    self.home_pose[0] =rep
                else:
                    continue
                
                self.print_status()
                time.sleep(0.1)
            except Exception as e:
                # print(e)
                continue
            time.sleep(1)
            
def main():
    teleop_keyboard = TeleopKeyboard('/dev/ttyS3', 1000000)
    teleop_keyboard.keyboard_listener()
    
    
if __name__ == '__main__':
    main()