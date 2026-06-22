from pymycobot import UltraArmP1
import time

ua = UltraArmP1('/dev/ttyUSB0', 1000000, debug=1)
ua.go_home()
for i in range(1, 2):
    angles = ua.get_angles_info()
# time.sleep(0.1)
# coords = ua.get_coords_info()
    if angles == -1:
        print('failed count:', i)
    print(angles)