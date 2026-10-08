#!/usr/bin/env python3
"""Activate configured Pro450 controllers on Gazebo's first physics update."""

import time

import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from std_srvs.srv import Empty


DEFAULT_CONTROLLERS = [
    "arm_controller",
    "joint_state_broadcaster",
    "pro_gripper_controller",
]


class ControllerActivationCoordinator(Node):
    def __init__(self):
        super().__init__("pro450_controller_activation_coordinator")
        self.declare_parameter("controllers", DEFAULT_CONTROLLERS)
        self.controllers = list(self.get_parameter("controllers").value)
        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.unpause_client = self.create_client(Empty, "/unpause_physics")
        self.pause_client = self.create_client(Empty, "/pause_physics")

    def _wait_for_future(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def _restore_pause(self):
        if not self.pause_client.wait_for_service(timeout_sec=1.0):
            return
        future = self.pause_client.call_async(Empty.Request())
        self._wait_for_future(future, 2.0)

    def activate(self):
        if not self.switch_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("Controller switch service is unavailable.")
            return False
        if not self.unpause_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Gazebo unpause service is unavailable.")
            return False

        request = SwitchController.Request()
        request.activate_controllers = self.controllers
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 10

        # Queue the atomic controller switch while physics is still paused.
        # The service completes only when controller_manager processes its next
        # update, so unpausing below makes activation occur on the first update
        # instead of leaving the model uncontrolled for several seconds.
        switch_future = self.switch_client.call_async(request)
        time.sleep(0.05)
        unpause_future = self.unpause_client.call_async(Empty.Request())

        if not self._wait_for_future(unpause_future, 5.0):
            self.get_logger().error("Timed out while unpausing Gazebo.")
            self._restore_pause()
            return False
        if unpause_future.result() is None:
            self.get_logger().error("Gazebo rejected the unpause request.")
            self._restore_pause()
            return False
        if not self._wait_for_future(switch_future, 12.0):
            self.get_logger().error("Timed out activating Pro450 controllers.")
            self._restore_pause()
            return False
        result = switch_future.result()
        if result is None or not result.ok:
            self.get_logger().error("Controller manager rejected grouped activation.")
            self._restore_pause()
            return False

        self.get_logger().info(
            "Gazebo unpaused with all Pro450 controllers activated as a group."
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ControllerActivationCoordinator()
    return_code = 0
    try:
        if not node.activate():
            return_code = 1
    except KeyboardInterrupt:
        return_code = 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()
