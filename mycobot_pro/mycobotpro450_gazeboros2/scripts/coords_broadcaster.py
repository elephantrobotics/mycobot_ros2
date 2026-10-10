#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from pro450_sdk_adapter import Pro450Client

DEFAULT_PRO450_IP = "192.168.0.232"
DEFAULT_PRO450_PORT = 4500
DEFAULT_BROADCAST_RATE = 10.0

class CoordsBroadcaster(Node):
    def __init__(self):
        super().__init__('coords_broadcaster')
        self.declare_parameter('pro450_ip', DEFAULT_PRO450_IP)
        self.declare_parameter('pro450_port', DEFAULT_PRO450_PORT)
        self.declare_parameter('broadcast_rate', DEFAULT_BROADCAST_RATE)
        self.pro450_ip = self.get_parameter('pro450_ip').value
        self.pro450_port = int(self.get_parameter('pro450_port').value)
        self.broadcast_rate = max(1.0, float(self.get_parameter('broadcast_rate').value))
        self.pub_coords = self.create_publisher(Point, '/pro450/end_effector_coords', 10)
        self.mc = None
        self.initialize_pro450()
        self.timer = self.create_timer(1.0 / self.broadcast_rate, self.broadcast_coords)
        self.get_logger().info('Coords Broadcaster started.')

    def initialize_pro450(self):
        try:
            self.get_logger().info(
                f"Connecting to Pro450 @ {self.pro450_ip}:{self.pro450_port}..."
            )
            self.mc = Pro450Client(self.pro450_ip, self.pro450_port)
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
        except Exception as exc:
            self.get_logger().warning(
                f"Unable to read Pro450 coordinates: {exc}",
                throttle_duration_sec=5.0,
            )

def main(args=None):
    rclpy.init(args=args)
    node = CoordsBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.mc is not None:
            node.mc.close()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

