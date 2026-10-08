"""Manual Gazebo-only acceptance check for the Pro450 hold controller.

Run only after ``slider.launch.py environment:=simulation`` is ready. This
script never creates a Pro450Client and never selects real mode.
"""

import os
import sys
import threading
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from teleop_keyboard_gazebo import TeleopKeyboard  # noqa: E402


def feedback(node, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = node._fresh_feedback()
        if value is not None:
            return value[0]
        time.sleep(0.05)
    raise RuntimeError("Gazebo joint feedback did not become available")


def hold_and_release(node, direction):
    result = node.press_hold(2, direction)
    if not result.startswith("accepted"):
        raise RuntimeError(f"J3 press was not accepted: {result}")
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline:
        node.hold_tick()
        time.sleep(0.05)
    node.release_hold()
    deadline = time.monotonic() + 5.0
    while not node.hold_idle() and time.monotonic() < deadline:
        node.hold_tick()
        time.sleep(0.05)
    if not node.hold_idle():
        raise RuntimeError("J3 failed to settle after key release")
    time.sleep(0.25)
    return feedback(node)


def main():
    rclpy.init(args=["--ros-args", "-p", "mode:=simulation"])
    node = TeleopKeyboard()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        if node.mode != "simulation":
            raise RuntimeError("Refusing to run outside simulation mode")
        if node.get_node_names().count("teleop_keyboard_gazebo") != 1:
            raise RuntimeError("Another keyboard controller is running")
        if "slider_control_gazebo" in node.get_node_names():
            raise RuntimeError("Slider control bridge is also running")
        node.set_hold_speed_gear(1)
        start = feedback(node)
        positive = hold_and_release(node, 1)
        if positive[2] - start[2] < 0.003:
            raise RuntimeError(f"J3 positive direction failed: {start[2]} -> {positive[2]}")
        if abs(positive[3] - start[3]) > 0.003:
            raise RuntimeError(f"J4 drifted during J3+: {start[3]} -> {positive[3]}")
        time.sleep(0.75)
        settled = feedback(node)
        if abs(settled[2] - positive[2]) > 0.002:
            raise RuntimeError("J3 continued moving after key release")
        negative = hold_and_release(node, -1)
        if negative[2] - settled[2] > -0.003:
            raise RuntimeError(f"J3 negative direction failed: {settled[2]} -> {negative[2]}")
        if abs(negative[3] - settled[3]) > 0.003:
            raise RuntimeError(f"J4 drifted during J3-: {settled[3]} -> {negative[3]}")
        print("PASS: J3 +/- signs, release stop, and J4 isolation in Gazebo")
    finally:
        if not node.hold_idle():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
        spinner.join(timeout=1.0)


if __name__ == "__main__":
    main()
