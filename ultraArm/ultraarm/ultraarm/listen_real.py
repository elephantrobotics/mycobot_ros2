import time
import math

import rclpy
from pymycobot.ultraArm import ultraArm
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class Talker(Node):
    def __init__(self):
        super().__init__("real_listener")
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
   
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.ua = ultraArm(port, str(baud))
        self.ua.go_zero()
        
        
    def start(self):
        pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )
        rate = self.create_rate(30)  # 30hz

        # pub joint state
        joint_state_send = JointState()
        joint_state_send.header = Header()

        joint_state_send.name = [
            "joint1_to_base",
            "joint2_to_joint1",
            "joint3_to_joint2",
        ]
        joint_state_send.velocity = [0.0,]
        joint_state_send.effort = []
        
        while rclpy.ok():
            rclpy.spin_once(self)
            # get real angles from server
            res = self.ua.get_angles_info()
            try:
                if res[0] == res[1] == res[2] ==0.0:
                    continue
                radians_list = [
                    round(res[0] * (math.pi / 180), 2),
                    round(res[1] * (math.pi / 180), 2),
                    round(res[2] * (math.pi / 180), 2),
                ]
                self.get_logger().info("res: {}".format(radians_list))
    
                # publish angles.
                joint_state_send.header.stamp = self.get_clock().now().to_msg()
                joint_state_send.position = radians_list
                pub.publish(joint_state_send)
                rate.sleep()
            except Exception as e:
                print(e)

def main(args=None):
    rclpy.init(args=args)
    
    talker = Talker()
    talker.start()
    rclpy.spin(talker)
    
    talker.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()