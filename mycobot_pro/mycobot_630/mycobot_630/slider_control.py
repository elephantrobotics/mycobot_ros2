import rclpy
from pymycobot.elephantrobot import ElephantRobot
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import sys


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
     
        self.declare_parameter('ip', '192.168.1.159')
        self.declare_parameter('port', 5001)
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        self.get_logger().info("ip:%s, port:%d" % (ip, port))
        self.mc = ElephantRobot(ip, port)
        # START CLIENT,启动客户端
        res = self.mc.start_client()
        if res != "":
            sys.exit(1)

        self.mc.set_speed(90)

    def listener_callback(self, msg):
        # print(msg.position)
        data_list = []
        for _, value in enumerate(msg.position):
            angles = round(math.degrees(value), 2)
            data_list.append(angles)
        
        print('current angles:', data_list)
        self.mc.write_angles(data_list, 800)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
