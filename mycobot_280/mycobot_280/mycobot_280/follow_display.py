import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import os
import math
import pymycobot
from packaging import version

# min low version require
MAX_REQUIRE_VERSION = '3.5.3'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) > version.parse(MAX_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be less than {} . The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot import MyCobot


class Talker(Node):
    def __init__(self):
        super().__init__("follow_display")
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
   
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        if self.robot_m5:
            port = self.robot_m5
        else:
            port = self.robot_wio

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = MyCobot(port, str(baud))
        self.mc.release_all_servos()

    def start(self):
        pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )
        pub_marker = self.create_publisher(
            msg_type=Marker,
            topic="visualization_marker",
            qos_profile=10
        )
        rate = self.create_rate(30)

        # pub joint state
        joint_state_send = JointState()
        joint_state_send.header = Header()

        joint_state_send.name = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ] 
        joint_state_send.velocity = [0.0,]
        joint_state_send.effort = []

        marker_ = Marker()
        marker_.header.frame_id = "/joint1"
        marker_.ns = "my_namespace"

        while rclpy.ok():
            rclpy.spin_once(self)
            joint_state_send.header.stamp = self.get_clock().now().to_msg()
            try:
                angles = self.mc.get_radians()
                data_list = []
                for _, value in enumerate(angles):
                    data_list.append(value)

                

                #self.get_logger().info('angles: {}'.format([round(math.degrees(angle), 2) for angle in data_list]))
                joint_state_send.position = data_list

                pub.publish(joint_state_send)

                coords = self.mc.get_coords()

                # marker
                marker_.header.stamp = self.get_clock().now().to_msg()
                marker_.type = marker_.SPHERE
                marker_.action = marker_.ADD
                marker_.scale.x = 0.04
                marker_.scale.y = 0.04
                marker_.scale.z = 0.04

                # marker position initial
                # self.get_logger().info('{}'.format(coords))
                
                if not coords:
                    coords = [0, 0, 0, 0, 0, 0]
                    # self.get_logger().info("error [101]: can not get coord values")

                marker_.pose.position.x = coords[1] / 1000 * -1
                marker_.pose.position.y = coords[0] / 1000
                marker_.pose.position.z = coords[2] / 1000

                marker_.color.a = 1.0
                marker_.color.g = 1.0
                pub_marker.publish(marker_)

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

