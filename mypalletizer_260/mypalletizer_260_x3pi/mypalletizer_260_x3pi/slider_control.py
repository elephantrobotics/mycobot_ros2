import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# from pymycobot import MyCobotSocket
from pymycobot.mypalletizer import MyPalletizer
import math,sys

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

        self.mc = MyPalletizer("/dev/ttyS3", 1000000)    

    def listener_callback(self, msg):
        
        global old_list   
        old_list = []
        # print(msg.position)
        
        data_list = []
        for _, value in enumerate(msg.position):
            value = value * 180 / math.pi
            data_list.append(value)
        # print ("angles:", data_list)
        
        self.mc.send_angles(data_list, 25)


def main(args=None):
   
    rclpy.init(args=args)    
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber) 
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
