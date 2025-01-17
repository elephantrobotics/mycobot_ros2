import sys
import os
import fcntl
import termios
import tty
import time

import pymycobot
from packaging import version

# min low version require
MAX_REQUIRE_VERSION = '3.5.3'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be less than {} . The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot import MyCobot


# Avoid serial port conflicts and need to be locked
def acquire(lock_file):
    open_mode = os.O_RDWR | os.O_CREAT | os.O_TRUNC
    fd = os.open(lock_file, open_mode)

    pid = os.getpid()
    lock_file_fd = None
    
    timeout = 30.0
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
    def __init__(self):
        self.mc = MyCobot('/dev/ttyS3', 1000000)
        time.sleep(0.05)
        self.mc.set_fresh_mode(1)
        time.sleep(0.05)

        self.model = 0
        self.speed = 30
        self.change_percent = 5

        self.change_angle = 180 * self.change_percent / 100
        self.change_len = 250 * self.change_percent / 100

        self.init_pose = [[0, 0, 0, 0, 0, 0], self.speed]
        self.home_pose = [[0, 8, -127, 40, 0, 0], self.speed]


        self.record_coords = self.get_initial_coords()

    def get_initial_coords(self):
        while True:
            if self.mc:
                lock = acquire("/tmp/mycobot_lock")
                res = self.mc.get_coords()
                release(lock)
                if res:
                    break
                time.sleep(0.1)

        return [res, self.speed, self.model]

    def print_status(self):
        print("\r current coords: %s" % self.record_coords)

    def keyboard_listener(self):
        print(msg)
        print(vels(self.speed, self.change_percent))
        while True:
            try:
                # print("\r current coords: %s" % self.record_coords)
                with Raw(sys.stdin):
                    key = sys.stdin.read(1)
                if key == "q":
                    break
                elif key in ["w", "W"]:
                    self.record_coords[0][0] += self.change_len
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["s", "S"]:
                    self.record_coords[0][0] -= self.change_len
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["a", "A"]:
                    self.record_coords[0][1] -= self.change_len
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["d", "D"]:
                    self.record_coords[0][1] += self.change_len
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["z", "Z"]:
                    self.record_coords[0][2] -= self.change_len
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["x", "X"]:
                    self.record_coords[0][2] += self.change_len
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["u", "U"]:
                    self.record_coords[0][3] += self.change_angle
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["j", "J"]:
                    self.record_coords[0][3] -= self.change_angle
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["i", "I"]:
                    self.record_coords[0][4] += self.change_angle
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["k", "K"]:
                    self.record_coords[0][4] -= self.change_angle
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["o", "O"]:
                    self.record_coords[0][5] += self.change_angle
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["l", "L"]:
                    self.record_coords[0][5] -= self.change_angle
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_coords(*self.record_coords)
                        release(lock)
                elif key in ["g", "G"]:
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.set_gripper_state(0, 30)
                        release(lock)
                elif key in ["h", "H"]:
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.set_gripper_state(1, 30)
                        release(lock)
                elif key == "1":
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_angles(*self.init_pose)
                        release(lock)
                    time.sleep(3)
                    self.record_coords = self.get_initial_coords()
                elif key in "2":
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        self.mc.send_angles(*self.home_pose)
                        release(lock)
                    time.sleep(3)
                    self.record_coords = self.get_initial_coords()
                elif key in "3":
                    if self.mc:
                        lock = acquire("/tmp/mycobot_lock")
                        rep = self.mc.get_angles()
                        release(lock)
                    self.home_pose[0] =rep
                else:
                    continue
                
                self.print_status()
                time.sleep(0.1)
            except Exception as e:
                print(e)
                continue
            time.sleep(1)
            
            
def main():
    teleop_keyboard = TeleopKeyboard()
    teleop_keyboard.keyboard_listener()
    
    
if __name__ == '__main__':
    main()