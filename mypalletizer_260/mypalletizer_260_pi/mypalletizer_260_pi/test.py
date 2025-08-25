import pymycobot
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyPalletizerSocket

mc = MyPalletizerSocket("192.168.123.62", 9000)
mc.connect() 

print(mc.get_angles())
print(mc.get_radians())
print(mc.get_coords())
