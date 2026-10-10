"""Real-mode ROS/Gazebo integration using a fake SDK, never a robot socket.

Requires the simulation launch and no other keyboard/slider bridge. Hardware
SDK import is replaced before importing the controller; all motion is virtual.
"""
import math
import os
from pathlib import Path
import sys
import threading
import time
import types

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


class FakePro450:
    def __init__(self, *_args, **_kwargs):
        self.pose = [0.0] * 6 + [0.0]
        self.goal = None
        self.speed = 0.0
        self.stamp = time.monotonic()
        self.calls = []

    def update(self):
        now = time.monotonic()
        elapsed, self.stamp = now - self.stamp, now
        if self.goal is not None:
            axis, target = self.goal
            delta = target - self.pose[axis]
            self.pose[axis] += max(-self.speed * elapsed, min(self.speed * elapsed, delta))

    def is_power_on(self):
        return 1

    def get_error_information(self):
        return 0

    def is_moving(self):
        self.update()
        return int(self.goal is not None and
                   abs(self.pose[self.goal[0]] - self.goal[1]) > 1e-5)

    def get_angles(self):
        self.update()
        return [math.degrees(p) for p in self.pose[:6]]

    def get_pro_gripper_angle(self, gripper_id=14):
        return self.pose[6] * 100

    def get_fresh_mode(self):
        return 0

    def set_fresh_mode(self, mode):
        self.calls.append(('fresh', mode))
        return 1

    def jog_angle(self, joint_id, direction, speed, _async=True):
        self.update()
        self.calls.append(('jog', joint_id, direction, speed))
        sign = 1 if direction == 1 else -1
        self.goal = (joint_id - 1, self.pose[joint_id - 1] + sign * 100)
        self.speed = math.radians(1.5 * speed)
        return 1

    def send_angle(self, joint_id, angle, speed, _async=False):
        self.update()
        self.calls.append(('angle', joint_id, angle, speed))
        self.goal, self.speed = (joint_id - 1, math.radians(angle)), math.radians(1.5 * speed)
        return 1

    def stop(self, deceleration=0, _async=False):
        self.update()
        self.calls.append(('stop', deceleration))
        self.goal = None
        return 1

    def set_angle_callback(self, callback):
        # This fixture updates on reads; the production socket callback is tested separately.
        self.angle_callback = callback

    def close(self):
        pass


# Neither import nor instantiate pymycobot. Missing methods fail rather than
# falling through to any physical transport.
fake_module = types.ModuleType('pro450_sdk_adapter')
fake_module.Pro450Client = FakePro450
sys.modules['pro450_sdk_adapter'] = fake_module

import rclpy
from teleop_keyboard_gazebo import TeleopKeyboard


def tick_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        node.hold_tick()
        time.sleep(.05)


def main():
    rclpy.init(args=['--ros-args', '-p', 'mode:=real', '-p', 'real_hold_enabled:=true',
                    '-p', 'real_gripper_hold_enabled:=false'])
    node = TeleopKeyboard()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        assert type(node.mc) is FakePro450
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if node._fresh_feedback() is not None and node._mirror_positions is not None:
                break
            time.sleep(.1)
        time.sleep(.5)
        assert not node.mc.calls, 'startup issued a motion write'
        assert node.press_hold(2, 1).startswith('rejected'), 'startup was unlocked'
        node.set_hold_speed_gear(1)
        result = node.arm_real()
        assert result.startswith('armed'), result
        assert not node.mc.calls, 'arming issued a motion write'
        original_validator = node._state_is_valid
        node._state_is_valid = lambda *_: (False, [], 'collision (fake rejection test)')
        assert node.press_hold(2, 1).startswith('accepted')
        tick_for(node, .8)
        node.release_hold()
        tick_for(node, .5)
        assert not node.mc.calls, 'rejected collision corridor issued motion'
        node._state_is_valid = original_validator
        start = node._fresh_feedback()[0]
        for direction in (1, -1):
            before = node._fresh_feedback()[0][2]
            result = node.press_hold(2, direction)
            assert result.startswith('accepted'), result
            tick_for(node, 3)
            node.release_hold()
            tick_for(node, 1.5)
            after = node._fresh_feedback()[0][2]
            assert direction * (after - before) > .003, (direction, before, after)
            assert node.hold_idle(), 'release did not settle'
        goals = [call for call in node.mc.calls if call[0] == 'angle']
        assert goals and all(call[1] == 3 and math.isfinite(call[2]) and 1 <= call[3] <= 4
                             for call in goals), goals
        assert not any(call[0] == 'jog' for call in node.mc.calls), node.mc.calls
        assert all(abs(node._fresh_feedback()[0][i] - start[i]) < .001
                   for i in range(7) if i != 2), 'other hardware joint changed'
        assert ('stop', 0) in node.mc.calls, node.mc.calls
        result = node.press_hold(6, 1)
        assert result.startswith('rejected'), 'uncalibrated real gripper was enabled'
        node.stop()
        assert not node.real_transport.armed
        print('PASS: FAKE SDK real-mode startup read-only, explicit arm, J3 +/- hold, '
              'single-axis speed mapping, release STOP, isolation and gripper lock')
    finally:
        node.real_transport.close()
        node._real_thread.join(2)
        node.destroy_node()
        rclpy.try_shutdown()
        spinner.join(1)


if __name__ == '__main__':
    main()
