import sys
from pymycobot.elephantrobot import ElephantRobot

mc = ElephantRobot('192.168.1.159', 5001, debug=True)

mc.start_client()

print(mc.get_angles())