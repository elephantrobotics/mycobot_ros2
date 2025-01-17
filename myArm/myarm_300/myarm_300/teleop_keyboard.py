import fcntl
import sys
import termios
import tty
import time
import os

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
    from pymycobot.myarm import MyArm


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
MyArm Teleop Keyboard Controller
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
  
    mc = MyArm("/dev/ttyAMA0", 115200)

    model = 0
    speed = 80
    change_percent = 10

    change_angle = 180 * change_percent / 100
    change_len = 250 * change_percent / 100

    init_pose = [[0, 0, 0, 0, 0, 0, 0], speed]
    home_pose = [[0, 0, 0, -90, 0, -90, 0], speed]
    if mc:
        lock = acquire("/tmp/myarm_lock")
        mc.set_free_mode(1)
        release(lock)
        time.sleep(0.05)
        
    if mc:
        lock = acquire("/tmp/myarm_lock")
        mc.send_angles(*home_pose)
        release(lock)
        time.sleep(2)

    while True:
        if mc:
            lock = acquire("/tmp/myarm_lock")
            res = mc.get_coords()
            release(lock)
        if res:
            break
        time.sleep(0.1)

    record_coords = [res, speed, model]
    print(record_coords)

    try:
        print(msg)
        print(vels(speed, change_percent))
        while 1:
            try:
                print("\r current coords: %s" % record_coords)
                with Raw(sys.stdin):
                    key = sys.stdin.read(1)
                if key == "q":
                    # mc.release_all_servos()
                    break
                elif key in ["w", "W"]:
                    record_coords[0][0] += change_len
                    print('start X+', change_len)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["s", "S"]:
                    record_coords[0][0] -= change_len
                    print('start X-', change_len)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["a", "A"]:
                    record_coords[0][1] -= change_len
                    print('start Y-', change_len)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["d", "D"]:
                    record_coords[0][1] += change_len
                    print('start Y+', change_len)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["z", "Z"]:
                    record_coords[0][2] -= change_len
                    print('start Z-', change_len)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["x", "X"]:
                    record_coords[0][2] += change_len
                    print('start Z+', change_len)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["u", "U"]:
                    record_coords[0][3] += change_angle
                    print('start RX+', change_angle)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["j", "J"]:
                    record_coords[0][3] -= change_angle
                    print('start RX-', change_angle)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["i", "I"]:
                    record_coords[0][4] += change_angle
                    print('start RY+', change_angle)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["k", "K"]:
                    record_coords[0][4] -= change_angle
                    print('start RY-', change_angle)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["o", "O"]:
                    record_coords[0][5] += change_angle
                    print('start RZ+', change_angle)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["l", "L"]:
                    record_coords[0][5] -= change_angle
                    print('start RZ-', change_angle)
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_coords(*record_coords)
                        release(lock)
                elif key in ["g", "G"]:
                    print('open gripper')
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.set_gripper_state(0, 95)
                        release(lock)
                elif key in ["h", "H"]:
                    print('close gripper')
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.set_gripper_state(1, 95)
                        release(lock)
                elif key == "1":
                    print('go init point')
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_angles(*init_pose)
                        release(lock)
                elif key in "2":
                    print('go home pose')
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        mc.send_angles(*home_pose)
                        release(lock)
                elif key in "3":
                    if mc:
                        lock = acquire("/tmp/myarm_lock")
                        rep = mc.get_angles()
                        release(lock)
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
