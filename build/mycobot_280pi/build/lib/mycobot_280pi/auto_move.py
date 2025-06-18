import rclpy
from rclpy.node import Node
import time
from pymycobot import MyCobot280
import math

class MoveToPosition(Node):
    def __init__(self):
        super().__init__('move_to_position')
        self.mc = MyCobot280("/dev/ttyAMA0", 1000000)
        time.sleep(0.1)
        self.mc.set_fresh_mode(1)
        time.sleep(0.1)

        
        initial_pos = [0, 0, 0, 0, 0, 0]
        self.mc.send_angles([math.degrees(x) for x in initial_pos], 50)
        time.sleep(3)

        # degrés -> radians
        target_pos = [-0.00, -0.78, 0.78, 1.59, -1.59, -0.00]
        target_deg = [round(math.degrees(angle), 2) for angle in target_pos]

        self.mc.send_angles(target_deg, 50)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPosition()
    rclpy.spin_once(node, timeout_sec=5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
