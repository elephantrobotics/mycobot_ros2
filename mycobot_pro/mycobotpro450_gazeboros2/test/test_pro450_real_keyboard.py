"""Hardware transport acceptance with a fake SDK: never opens a socket."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from pro450_real_keyboard import RealKeyboardTransport, sdk_speed_for_rad, SDK_RAD_PER_SPEED


class FakeSDK:
    def __init__(self):
        self.calls = []
        self.pose = [0.0] * 6 + [0.5]

    def is_power_on(self):
        return 1

    def is_moving(self):
        return 0

    def get_error_information(self):
        return 0

    def send_angle(self, axis, angle, speed, _async=False):
        self.calls.append(('angle', axis, angle, speed, _async))
        return 1

    def stop(self, deceleration=0, _async=False):
        self.calls.append(('stop', deceleration))
        return 1

    def set_pro_gripper_speed(self, speed):
        self.calls.append(('gripper_speed', speed))
        return 1

    def set_pro_gripper_angle(self, angle):
        self.calls.append(('gripper_angle', angle))
        return 1


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.sdk = FakeSDK()
        self.now = 1.0
        self.transport = RealKeyboardTransport(
            self.sdk, lambda: list(self.sdk.pose), lambda _: None,
            [(-3, 3)] * 6 + [(0, 1)], clock=lambda: self.now)
        for _ in range(4):
            self.transport.cycle()
            self.now += 0.1

    def command(self, axis=2):
        self.assertTrue(self.transport.arm())
        self.assertTrue(self.transport.submit(axis, self.sdk.pose[axis] + 0.2,
            math.radians(12), self.sdk.pose, self.sdk.pose[axis] + 0.5))
        self.transport.cycle()

    def test_read_only_startup_and_close_request_have_no_writes(self):
        self.assertEqual(self.sdk.calls, [])
        self.transport.stop(emergency=True, lock=True)
        self.transport.cycle()
        self.assertEqual(self.sdk.calls, [])

    def test_locked_submit_is_rejected(self):
        self.assertFalse(self.transport.submit(2, .2, .2, self.sdk.pose, .5))
        self.transport.cycle()
        self.assertEqual(self.sdk.calls, [])

    def test_single_axis_speed_mapping_and_finite_endpoint(self):
        self.command()
        call = self.sdk.calls[0]
        self.assertEqual(call[0:2], ('angle', 3))
        self.assertEqual(call[3:], (8, True))
        self.assertLessEqual(math.radians(call[2]), .08)

    def test_release_uses_decelerating_stop_and_invalidates_command(self):
        self.command()
        self.transport.stop()
        self.transport.cycle()
        self.assertEqual(self.sdk.calls[-1], ('stop', 1))
        self.now += .3
        self.transport.cycle()
        self.assertEqual(len(self.sdk.calls), 2)

    def test_emergency_stop_and_lock(self):
        self.command()
        self.transport.stop(emergency=True, lock=True)
        self.transport.cycle()
        self.assertEqual(self.sdk.calls[-1], ('stop', 0))
        self.assertFalse(self.transport.armed)

    def test_other_joint_drift_rejects_goal(self):
        self.assertTrue(self.transport.arm())
        origin = list(self.sdk.pose)
        self.sdk.pose[1] = .02
        self.transport.submit(2, .2, .2, origin, .5)
        with self.assertRaisesRegex(RuntimeError, 'uncommanded'):
            self.transport.cycle()
        self.assertEqual(self.sdk.calls, [])

    def test_stale_gui_command_requests_stop(self):
        self.command()
        self.now += .6
        self.transport.cycle()
        self.assertEqual(self.sdk.calls[-1], ('stop', 0))
        self.assertFalse(self.transport.armed)

    def test_gripper_speed_only_after_explicit_command(self):
        self.command(6)
        self.assertEqual(self.sdk.calls[0], ('gripper_speed', 30))
        self.assertEqual(self.sdk.calls[1][0], 'gripper_angle')

    def test_urdf_speed_clamp_never_rounds_up(self):
        self.assertEqual(sdk_speed_for_rad(10), 38)
        self.assertLessEqual(38 * SDK_RAD_PER_SPEED, 1)
        for gear in range(1, 6):
            self.assertEqual(sdk_speed_for_rad(math.radians(6 * gear)), 4 * gear)

    def test_warmup_cannot_arm(self):
        transport = RealKeyboardTransport(self.sdk, lambda: self.sdk.pose,
            lambda _: None, self.transport.limits, clock=lambda: self.now)
        transport.cycle()
        self.assertFalse(transport.arm())


if __name__ == '__main__':
    unittest.main()
