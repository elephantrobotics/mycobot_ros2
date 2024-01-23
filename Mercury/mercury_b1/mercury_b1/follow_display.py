import rclpy
import time
import traceback
import math
from pymycobot.mercury import Mercury
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


class Talker(Node):
    def __init__(self):
        super().__init__("follow_display")
        self.declare_parameter('port1', '/dev/ttyTHS0')
        self.declare_parameter('port2', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
   
        port1 = self.get_parameter("port1").get_parameter_value().string_value
        port2 = self.get_parameter("port2").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info("left arm:%s, right arm:%s, baud:%d" % (port1, port2, baud))
        self.l = Mercury(port1, str(baud))
        self.r = Mercury(port2, str(baud))
        
        self.l.release_all_servos()
        time.sleep(0.05)
        self.r.release_all_servos()
        time.sleep(0.05)
        print("Rlease all servos over.\n")

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
        "joint7_L",
        "joint1_R",
        "joint2_R",
        "joint3_R",
        "joint4_R",
        "joint5_R",
        "joint6_R",
        "joint7_R",
        "eye",
        "head",
        "body",
    ] 
        joint_state_send.velocity = [0.0,]
        joint_state_send.effort = []

        marker_ = Marker()
        marker_.header.frame_id = "/base"
        marker_.ns = "my_namespace"

        while rclpy.ok():
            rclpy.spin_once(self)
            joint_state_send.header.stamp = self.get_clock().now().to_msg()
            try:
                left_angles = self.l.get_angles()
                right_angles = self.r.get_angles()
                eye_angle = self.r.get_angle(11)
                head_angle = self.r.get_angle(12)
                body_angle = self.r.get_angle(13)
                
                print('left_angles: {}'.format(left_angles))
                print('right_angles: {}'.format(right_angles))
                print('camera_angle: {}'.format(eye_angle))
                print('head_angle: {}'.format(head_angle))
                print('body_angle: {}'.format(body_angle))
                
                all_angles = left_angles + right_angles + [eye_angle] + [head_angle] + [body_angle]
                data_list = []
                for _, value in enumerate(all_angles):
                    radians = math.radians(value)
                    data_list.append(radians)

                # self.get_logger().info('radians: {}'.format(data_list))
                joint_state_send.position = data_list

                pub.publish(joint_state_send)
                
                left_coords = self.l.get_coords()
                right_coords = self.r.get_coords()
                eye_coords = [self.r.get_angle(11)]
                head_coords = [self.r.get_angle(12)]
                body_coords = [self.r.get_angle(13)]
                
                # marker
                marker_.header.stamp = self.get_clock().now().to_msg()
                marker_.type = marker_.SPHERE
                marker_.action = marker_.ADD
                marker_.scale.x = 0.04
                marker_.scale.y = 0.04
                marker_.scale.z = 0.04

                # marker position initial    
                if not left_coords:
                    left_coords = [0, 0, 0, 0, 0, 0, 0]
                    self.get_logger().info("error [101]: can not get coord values")

                marker_.pose.position.x = left_coords[1] / 1000 * -1
                marker_.pose.position.y = left_coords[0] / 1000
                marker_.pose.position.z = left_coords[2] / 1000

                marker_.pose.position.x = right_coords[1] / 1000 * -1
                marker_.pose.position.y = right_coords[0] / 1000
                marker_.pose.position.z = right_coords[2] / 1000

                marker_.pose.position.x = eye_coords[0] / 1000 * -1
                marker_.pose.position.x = head_coords[0] / 1000 * -1
                marker_.pose.position.x = body_coords[0] / 1000 * -1
                
                marker_.color.a = 1.0
                marker_.color.g = 1.0
                pub_marker.publish(marker_)

                rate.sleep()
            except Exception as e:
                e = traceback.format_exc()
                print(str(e))
        
def main(args=None):
    rclpy.init(args=args)
    
    talker = Talker()
    talker.start()
    rclpy.spin(talker)
    
    talker.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()

