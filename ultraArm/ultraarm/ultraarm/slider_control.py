import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node
import math
import pymycobot
from packaging import version
# min low version require
MAX_REQUIRE_VERSION = '3.9.1'
current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    from pymycobot.ultraArmP340 import ultraArmP340
    class_name = 'new'
else:
    from pymycobot.ultraArm import ultraArm
    class_name = 'old'
    print("Note: This class is no longer maintained since v3.6.0, please refer to the project documentation: https://github.com/elephantrobotics/pymycobot/blob/main/README.md")


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
        if class_name == 'old':
            self.ua = ultraArm("/dev/ttyUSB0", 115200)
        else:
            self.ua = ultraArmP340('/dev/ttyUSB0', 115200)
        self.ua.go_zero()


    def listener_callback(self, msg):
        print(msg.position)
        data_list = []
        for _, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
            
        print('data_list: {}'.format(data_list))
        self.ua.set_angles(data_list, 25)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()