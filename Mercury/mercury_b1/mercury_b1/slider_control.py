import rclpy
from pymycobot.mercury import Mercury
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


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
     
        self.declare_parameter('port1', '/dev/ttyTHS0')
        self.declare_parameter('port2', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port1 = self.get_parameter('port1').get_parameter_value().string_value
        port2 = self.get_parameter('port2').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.get_logger().info("left arm:%s, baud:%d" % (port1, baud))
        self.get_logger().info("right arm:%s, baud:%d" % (port2, baud))
        # left arm
        self.l = Mercury(port1, baud)
        # right arm
        self.r = Mercury(port2, baud)
        time.sleep(0.05)
        # self.l.set_fresh_mode(1)
        # self.r.set_fresh_mode(1)
        # time.sleep(0.05)
        
    def listener_callback(self, msg):

        data_list = []
        for index, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
            
        print('data_list: {}'.format(data_list))
        
        left_arm = data_list[:7]
        right_arm = data_list[7:-3]
        middle_arm = data_list[-3:]
        
        print('left_angles: {}'.format(left_arm))
        print('right_angles: {}'.format(right_arm))
        print('middle_arm: {}'.format(middle_arm))
        
        self.l.send_angles(left_arm, 25)
        self.r.send_angles(right_arm, 25)
        self.r.send_angle(11, middle_arm[0], 25)
        self.r.send_angle(12, middle_arm[1], 25)
        self.r.send_angle(13, middle_arm[2], 25)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
