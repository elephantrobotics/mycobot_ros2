"""Gazebo-only gripper hold acceptance; never connects to the real robot."""

import os
import sys
import threading
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from teleop_keyboard_gazebo import TeleopKeyboard  # noqa: E402
from verify_pro450_keyboard_sim import feedback  # noqa: E402


def hold_gripper(node, direction, seconds):
    result = node.press_hold(6, direction)
    if not result.startswith("accepted"):
        raise RuntimeError(f"Gripper press rejected: {result}")
    peak_speed = 0.0
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        node.hold_tick()
        observed = node._fresh_feedback()
        if observed is not None:
            peak_speed = max(peak_speed, abs(observed[1][6]))
        time.sleep(0.05)
    node.release_hold()
    deadline = time.monotonic() + 6.0
    while not node.hold_idle() and time.monotonic() < deadline:
        node.hold_tick()
        time.sleep(0.05)
    if not node.hold_idle():
        raise RuntimeError("Gripper did not settle after release")
    time.sleep(0.25)
    stopped = feedback(node)
    time.sleep(0.75)
    settled = feedback(node)
    if abs(settled[6] - stopped[6]) > 0.003:
        raise RuntimeError("Gripper continued moving after final hold")
    return settled, peak_speed


def main():
    rclpy.init(args=["--ros-args", "-p", "mode:=simulation"])
    node = TeleopKeyboard()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        if node.mode != "simulation" or node.mc is not None:
            raise RuntimeError("Refusing to test outside Gazebo-only mode")
        time.sleep(1.0)
        if node.get_node_names().count("teleop_keyboard_gazebo") != 1:
            raise RuntimeError("Another keyboard controller is running")
        if "slider_control_gazebo" in node.get_node_names():
            raise RuntimeError("Slider bridge is also running")
        initial = feedback(node)
        peak_by_gear = {}
        for gear, seconds in ((2, 3.0), (5, 2.0)):
            node.set_hold_speed_gear(gear)
            start = feedback(node)
            direction = -1 if start[6] >= 0.5 else 1
            first, first_peak = hold_gripper(node, direction, seconds)
            second, second_peak = hold_gripper(node, -direction, seconds)
            if direction * (first[6] - start[6]) < 0.10:
                raise RuntimeError(f"Gear {gear} first direction failed: {start[6]} -> {first[6]}")
            if -direction * (second[6] - first[6]) < 0.10:
                raise RuntimeError(f"Gear {gear} opposite direction failed: {first[6]} -> {second[6]}")
            for pose in (first, second):
                if max(abs(pose[index] - initial[index]) for index in range(6)) > 0.01:
                    raise RuntimeError("Arm joints drifted while only gripper was commanded")
                if not 0.0 <= pose[6] <= 1.0:
                    raise RuntimeError("Gripper exceeded its URDF position bounds")
            peak_by_gear[gear] = max(first_peak, second_peak)
            print(f"Gear {gear}: {start[6]:.6f} -> {first[6]:.6f} -> "
                  f"{second[6]:.6f} rad; measured peak {peak_by_gear[gear]:.3f} rad/s")
        if peak_by_gear[2] <= 0.05:
            raise RuntimeError("Default gear did not exceed the old 0.05 rad/s ceiling")
        if peak_by_gear[5] <= peak_by_gear[2] * 1.2:
            raise RuntimeError("High gear did not increase measured gripper speed")
        print("PASS: faster default/high gripper gears, +/- directions, release stop, "
              "URDF bounds, and stationary arm")
    finally:
        if not node.hold_idle():
            node.stop()
        if node._hold_validation_worker is not None:
            node._hold_validation_worker.join(timeout=2.0)
        rclpy.try_shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
