import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node
from pymycobot.ultraArm import ultraArm
import math


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

        self.ua = ultraArm("/dev/ttyUSB0", 115200)
        self.ua.go_zero()


    def listener_callback(self, msg):
        print(msg.position)
        data_list = []
        for _, value in enumerate(msg.position):
            data_list.append(round(value, 3))
            
        print('data_list:',data_list)
        self.ua.set_radians(data_list, 80)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()