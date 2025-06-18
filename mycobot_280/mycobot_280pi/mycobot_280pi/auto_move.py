import rclpy
from rclpy.node import Node
import time
import math

from pymycobot import MyCobot280

class AutoMover(Node):
    def __init__(self):
        super().__init__('auto_mover')
        self.mc = MyCobot280('/dev/ttyAMA0', 1000000)
        time.sleep(0.1)
        self.mc.set_fresh_mode(1)
        time.sleep(0.1)

        self.run_sequence()

    def run_sequence(self):
        self.get_logger().info('🚀 Starting 2-position movement sequence...')

        # Liste des positions en RADIANS → conversion en DEGRÉS pour send_angles
        pos_init = [0, 0, 0, 0, 0, 0]
        pos_target_radians = [-0.00, -0.78, 0.78, 1.59, -1.59, -0.00]
        pos_target_degrees = [round(math.degrees(x), 2) for x in pos_target_radians]

        positions = [pos_init, pos_target_degrees]

        for i, pos in enumerate(positions):
            self.get_logger().info(f'📍 Moving to position {i}: {pos}')
            self.mc.send_angles(pos, 30)
            time.sleep(3)

        self.get_logger().info('✅ Sequence complete.')

def main(args=None):
    rclpy.init(args=args)
    node = AutoMover()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
