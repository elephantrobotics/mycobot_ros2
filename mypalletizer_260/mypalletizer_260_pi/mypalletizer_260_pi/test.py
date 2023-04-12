from pymycobot import MyCobotSocket

mc = MyCobotSocket("192.168.123.62", 9000)
mc.connect() 

print(mc.get_angles())
print(mc.get_radians())
print(mc.get_coords())
