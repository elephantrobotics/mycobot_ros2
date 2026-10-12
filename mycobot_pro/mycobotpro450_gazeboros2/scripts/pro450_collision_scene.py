#!/usr/bin/env python3
"""Public owner of the existing single MoveIt floor and collision matrix setup."""
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from slider_control_gazebo import SliderControl


class CollisionScene(Node):
    _ensure_acm_name = staticmethod(SliderControl._ensure_acm_name)
    _wait_for_future = SliderControl._wait_for_future

    def __init__(self):
        super().__init__('pro450_collision_scene')
        self._stop_requested = False
        self.floor_frame, self.floor_size_m = 'world', 20.
        self.floor_thickness_m, self.floor_clearance_m = .10, 0.
        self.get_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.pub = self.create_publisher(Bool, '/pro450/collision_scene_ready',
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ready = self.working = False
        self.create_timer(.5, self.tick)

    def tick(self):
        self.pub.publish(Bool(data=self.ready))
        if (self.ready or self.working or not self.get_scene_client.service_is_ready()
                or not self.apply_scene_client.service_is_ready()):
            return
        self.working = True
        threading.Thread(target=self.install_scene, daemon=True).start()

    def install_scene(self):
        try:
            self.ready, reason = SliderControl._ensure_floor_collision_scene(self)
            if not self.ready:
                self.get_logger().warning(reason)
        finally:
            self.working = False


def main():
    rclpy.init()
    node = CollisionScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_requested = True
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
