import time
import math
import rclpy
from pymycobot.mycobot import MyCobot
from rclpy.node import Node
from sensor_msgs.msg import JointState


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

        self.mc = MyCobot("/dev/ttyAMA0", 115200)
        time.sleep(0.05)
        self.mc.set_free_mode(1)
        time.sleep(0.05)
        
    def listener_callback(self, msg):
        data_list = []
        for _, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)

        print('joint angles: {}'.format(data_list))
        self.mc.send_angles(data_list, 25)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
