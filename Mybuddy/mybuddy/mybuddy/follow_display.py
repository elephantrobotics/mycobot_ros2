import math
import time
import rclpy
from pymycobot.mybuddy import MyBuddy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


class Talker(Node):
    def __init__(self):
        super().__init__("follow_display")
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
   
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mb = MyBuddy(port, str(baud))
        self.mb.release_all_servos()

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
            "joint1_L",
            "joint2_L",
            "joint3_L",
            "joint4_L",
            "joint5_L",
            "joint6_L",
            "joint1_R",
            "joint2_R",
            "joint3_R",
            "joint4_R",
            "joint5_R",
            "joint6_R",
            "base_link1",
            
        ] 
        joint_state_send.velocity = [0.0,]
        joint_state_send.effort = []

        marker_ = Marker()
        marker_.header.frame_id = "/base_link1"
        marker_.ns = "my_namespace"

        while rclpy.ok():
            rclpy.spin_once(self)
            joint_state_send.header.stamp = self.get_clock().now().to_msg()

            left_radians = self.mb.get_radians(1)
            
            right_radians = self.mb.get_radians(2)
            
            wangles = self.mb.get_angles(3)[0]*(math.pi/180)
            waist_radian =[]
            waist_radian.append(wangles)

            print('left:',left_radians,'right:',right_radians,'w:',waist_radian)
        
            # =======all_radians=======
            all_radians = left_radians + right_radians + waist_radian
            data_list = []
            for _, value in enumerate(all_radians):
                data_list.append(value)

            

            # self.get_logger().info('radians: {}'.format(data_list))
            joint_state_send.position = data_list

            print("all_radians: %s" % data_list)

            pub.publish(joint_state_send)

            # =======left_coords=======
            left_coords = self.mb.get_coords(1)

            # =======right_coords=======
            right_coords = self.mb.get_coords(2)

            waist_coords = self.mb.get_angles(3)

            # marker
            marker_.header.stamp = self.get_clock().now().to_msg()
            marker_.type = marker_.SPHERE
            marker_.action = marker_.ADD
            marker_.scale.x = 0.04
            marker_.scale.y = 0.04
            marker_.scale.z = 0.04

            # marker position initial
            # self.get_logger().info('{}'.format(coords))
            
            # if not coords:
            #     coords = [0, 0, 0, 0, 0, 0]
                # self.get_logger().info("error [101]: can not get coord values")

            marker_.pose.position.x = left_coords[1] / 1000 * -1
            marker_.pose.position.y = left_coords[0] / 1000
            marker_.pose.position.z = left_coords[2] / 1000
            
            time.sleep(0.02)
            
            marker_.pose.position.x = right_coords[1] / 1000 * -1
            marker_.pose.position.y = right_coords[0] / 1000
            marker_.pose.position.z = right_coords[2] / 1000
            
            time.sleep(0.02)
            
            marker_.pose.position.x = waist_coords[0] / 1000 * -1

            marker_.color.a = 1.0
            marker_.color.g = 1.0
            pub_marker.publish(marker_)

            rate.sleep()

        
def main(args=None):
    rclpy.init(args=args)
    
    talker = Talker()
    talker.start()
    rclpy.spin(talker)
    
    talker.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()

