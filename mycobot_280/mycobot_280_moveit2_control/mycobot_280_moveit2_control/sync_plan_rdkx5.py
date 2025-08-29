import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math
import pymycobot
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '3.8.0'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280RDKX5


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

        self.declare_parameter('port', '/dev/ttyS1')
        self.declare_parameter('baud', 1000000)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = MyCobot280RDKX5(port, baud)
        time.sleep(0.05)
        if self.mc.get_fresh_mode() == 0:
            self.mc.set_fresh_mode(1)
            time.sleep(0.05)

        # RViz中目标顺序
        self.rviz_order = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6'
        ]

    def listener_callback(self, msg):
        # 创建字典将关节名称与位置值关联
        joint_state_dict = {name: msg.position[i] for i, name in enumerate(msg.name)}
        # 根据 RViz 顺序重新排列关节角度
        data_list = []
        for joint in self.rviz_order:
            # 获取弧度并转为角度
            if joint in joint_state_dict:
                radians_to_angles = round(math.degrees(joint_state_dict[joint]), 3)
                data_list.append(radians_to_angles)
 
        print('data_list: {}'.format(data_list))
        self.mc.send_angles(data_list, 80)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
