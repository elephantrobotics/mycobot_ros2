import rclpy
from pymycobot.mybuddy import MyBuddy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
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

        self.mb = MyBuddy("/dev/ttyACM0", 115200)

    def listener_callback(self, msg):
        print(msg.position)
        data_list = []
        for _, value in enumerate(msg.position):
            data_list.append(value)

        print(data_list)
        
        data_list1 = data_list[:6]
        data_list2 = data_list[6:-1]
        data_list3 = data_list[-1:]
        
        print("left_arm: %s" % data_list1)
        print("right_arm: %s" % data_list2)
        print("waist: %s" % data_list3)

        self.mb.send_radians(1,data_list1, 50)
        time.sleep(0.02)
        self.mb.send_radians(2,data_list2, 50)
        time.sleep(0.02)

        # print(data_list3[0]* (180 / math.pi))
        print("\n")

        self.mb.send_angle(3, 1, data_list3[0]* (180 / math.pi), 10)
        # mb.send_radians(3,data_list3, 50)
        time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
