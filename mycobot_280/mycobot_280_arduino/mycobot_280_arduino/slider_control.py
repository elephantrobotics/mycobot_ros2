import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import os
import time
import math
import pymycobot
from packaging import version

# min low version require
# MAX_REQUIRE_VERSION = '3.7.0'
MIN_REQUIRE_VERSION = '3.6.6'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be less than {} . The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280
    

class Slider_Subscriber(Node):
    def __init__(self):
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        self.subscription
        
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        if self.robot_m5:
            port = self.robot_m5
        else:
            port = self.robot_wio
        self.get_logger().info("port:%s, baud:%d" % (port, 115200))
        self.mc = MyCobot280(port, 115200)
        time.sleep(0.05)
        self.mc.set_fresh_mode(1)
        time.sleep(0.05)
        # self.declare_parameter('port', '/dev/ttyUSB0')
        # self.declare_parameter('baud', 115200)
        # port = self.get_parameter('port').get_parameter_value().string_value
        # baud = self.get_parameter('baud').get_parameter_value().integer_value
        # self.get_logger().info("port:%s, baud:%d" % (port, baud))
        # self.mc = MyCobot(port, baud)

    def listener_callback(self, msg):

        data_list = []
        for _, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
            
        print('data_list: {}'.format(data_list))
        self.mc.send_angles(data_list, 25)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
