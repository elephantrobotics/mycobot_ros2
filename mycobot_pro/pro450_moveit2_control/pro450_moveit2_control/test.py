import time
from pymycobot import Pro450Client

pro450 = Pro450Client('192.168.0.232', debug=1)

time.sleep(0.5)

print('angles:', pro450.get_angles())