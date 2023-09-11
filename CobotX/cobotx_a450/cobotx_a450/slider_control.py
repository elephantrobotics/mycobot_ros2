import rclpy
from pymycobot.cobotx import CobotX
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
     
        self.declare_parameter('port', '/dev/ttyAMA1')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = CobotX(port, baud)

    def listener_callback(self, msg):
        # print(msg.position)
        data_list = []
        for _, value in enumerate(msg.position):
            angles = round(math.degrees(value), 2)
            data_list.append(angles)
        
        print('current angles:', data_list)
        # self.mc.send_radians(data_list, 80)
        self.mc.send_angles(data_list, 80)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
