from pymycobot.mycobot import MyCobot
import time

def test():

    mc=MyCobot('/dev/ttyUSB0',115200)
    mc.send_angles([0,0,0,0,0,0],30)
    time.sleep(1)
    # while True:
    #     mc.release_all_servos()
    #     print(mc.get_coords())
    while True:
        change_coords=[[175.8, -100.1, 43.4, 177.43, 16.68, 162.09], 10, 0]
        mc.send_coords(*change_coords)
        time.sleep(1.5)
        print(mc.get_coords())

        change_coords=[[175.8, -120.1, 43.4, 177.43, 16.68, 162.09], 10, 0]
        mc.send_coords(*change_coords)
        time.sleep(1.5)
        print(mc.get_coords())

        change_coords=[[175.8, -95.1, 43.4, 177.43, 16.68, 162.09], 10, 0]
        mc.send_coords(*change_coords)
        time.sleep(1.5)
        print(mc.get_coords())

        change_coords=[[180.8, -60.099999999999994, 43.4, 177.43, 16.68, 162.09], 10, 0]
        mc.send_coords(*change_coords)
        time.sleep(1.5)
        print(mc.get_coords())

        change_coords=[[185.8, 0.099999999999994, 43.4, 177.43, 16.68, 162.09], 10, 0]
        mc.send_coords(*change_coords)
        time.sleep(1.5)
        print(mc.get_coords())

test()