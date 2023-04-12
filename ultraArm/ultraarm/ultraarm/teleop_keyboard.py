from pymycobot.ultraArm import ultraArm
import rospy
import sys
import termios
import tty
import time


# Terminal output prompt information. 终端输出提示信息
msg = """\
ultraArm Teleop Keyboard Controller
---------------------------
Movimg options(control coordinations [x,y,z]):
              w(x+)

    a(y-)     s(x-)     d(y+)

        z(z-)       x(z+)

   

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
    ua = ultraArm("/dev/ttyUSB0", 115200)
    ua.go_zero()
    
    model = 0
    speed = 30
    change_percent = 2

    change_angle = 180 * change_percent / 100
    change_len = 250 * change_percent / 100

    init_pose = [0, 0,0, speed]
    home_pose = [0,30, 30,speed]

    ua.set_angles(*init_pose)

    while True:
        res = ua.get_coords_info()
        if res.x > 1:
            break
        time.sleep(0.1)

    record_coords = [res, speed]

    print(record_coords)

    try:
        print(msg)
        print(vels(speed, change_percent))
        # Keyboard keys call different motion functions. 键盘按键调用不同的运动功能
        while 1:
            try:
                # print("\r current coords: %s" % record_coords, end="")
                with Raw(sys.stdin):
                    key = sys.stdin.read(1)
                if key == "q":
                    break
                elif key in ["w", "W"]:
                    record_coords[0] += change_len
                    ua.set_coords(*record_coords)
                elif key in ["s", "S"]:
                    record_coords[0] -= change_len
                    ua.set_coords(*record_coords)
                elif key in ["a", "A"]:
                    record_coords[1] -= change_len
                    ua.set_coords(*record_coords)
                elif key in ["d", "D"]:
                    record_coords[1] += change_len
                    ua.set_coords(*record_coords)
                elif key in ["z", "Z"]:
                    record_coords[2] -= change_len
                    ua.set_coords(*record_coords)
                elif key in ["x", "X"]:
                    record_coords[2] += change_len
                    ua.set_coords(*record_coords)
                elif key in ["g", "G"]:
                    ua.switch_gripper(True)
                elif key in ["h", "H"]:
                    ua.switch_gripper(False)
                elif key == "1":
                    rsp = ua.set_angles(*init_pose)
                    record_coords = [res.x, res.y, res.z, speed]
                elif key in "2":
                    rsp = ua.set_angles(*home_pose)
                    record_coords = [res.x, res.y, res.z, speed]
                elif key in "3":
                    rep = ua.get_angles_info()
                    home_pose[0] = rep
                   
                else:
                    continue

            except Exception as e:
                # print(e)
                continue

    except Exception as e:
        print(e)


def main():
    teleop_keyboard()


if __name__ == "__main__":
    main()
