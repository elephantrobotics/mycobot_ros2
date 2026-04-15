#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from pymycobot import Pro450Client

PRO450_IP = "192.168.0.232"
PRO450_PORT = 4500
BROADCAST_RATE = 50.0

class CoordsBroadcaster(Node):
    def __init__(self):
        super().__init__('coords_broadcaster')
        self.pub_coords = self.create_publisher(Point, '/pro450/end_effector_coords', 10)
        self.mc = None
        self.initialize_pro450()
        self.timer = self.create_timer(1.0 / BROADCAST_RATE, self.broadcast_coords)
        self.get_logger().info('Coords Broadcaster started.')

    def initialize_pro450(self):
        try:
            self.get_logger().info(f"Connecting to Pro450 @ {PRO450_IP}:{PRO450_PORT}...")
            self.mc = Pro450Client(PRO450_IP, PRO450_PORT)
            time.sleep(1.0)
            current_angles = self.mc.get_angles()
            self.get_logger().info(f"Pro450 connected! Current angles: {current_angles}")
        except Exception as e:
            self.get_logger().error(f"Pro450 init failed: {e}")

    def broadcast_coords(self):
        if self.mc is None:
            return
        try:
            coords = self.mc.get_coords()
            if coords is not None and len(coords) >= 3:
                point = Point()
                point.x = float(coords[0])
                point.y = float(coords[1])
                point.z = float(coords[2])
                self.pub_coords.publish(point)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = CoordsBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

