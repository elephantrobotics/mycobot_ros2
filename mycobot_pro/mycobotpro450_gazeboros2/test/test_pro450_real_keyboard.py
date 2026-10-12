"""Hardware transport acceptance with a fake SDK: never opens a socket."""
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from pro450_real_keyboard import (RealKeyboardTransport, RealPoseReader, ArmReadError,
                                 sdk_speed_for_rad, SDK_RAD_PER_SPEED)


class ArmReadFailureTests(unittest.TestCase):
    def execute_sequence(self, outcomes):
        transport = RealKeyboardTransport(Mock(), lambda: None, Mock(), [(-3., 3.)] * 7)
        transport.armed = True
        index = [0]
        states = []
        def cycle():
            outcome = outcomes[index[0]]
            states.append((transport.armed, transport.error))
            if outcome is not None:
                raise outcome
        transport.cycle = cycle
        transport.closed = SimpleNamespace(
            is_set=lambda: index[0] >= len(outcomes),
            wait=lambda _: index.__setitem__(0, index[0] + 1))
        transport.run()
        return transport, states

    def test_four_angle_failures_do_not_latch_fifth_does(self):
        transport, states = self.execute_sequence([ArmReadError('timeout')] * 5)
        self.assertTrue(all(armed and error is None for armed, error in states))
        self.assertFalse(transport.armed)
        self.assertIn('Five consecutive', transport.error)

    def test_success_resets_counter_between_failure_groups(self):
        transport, states = self.execute_sequence(
            [ArmReadError('timeout')] * 4 + [None] + [ArmReadError('bad payload')] * 4)
        self.assertTrue(transport.armed)
        self.assertIsNone(transport.error)
        self.assertEqual(transport._arm_read_failures, 4)

    def test_other_faults_still_latch_on_first_exception(self):
        transport, _ = self.execute_sequence([RuntimeError('invalid hardware moving status')])
        self.assertFalse(transport.armed)
        self.assertEqual(transport.error, 'invalid hardware moving status')

    def test_sdk_exception_and_invalid_payload_are_angle_read_failures(self):
        sdk = Mock()
        reader = RealPoseReader(sdk, [(-3., 3.)] * 7)
        sdk.get_angles.side_effect = TimeoutError('no reply')
        with self.assertRaises(ArmReadError):
            reader()
        sdk.get_angles.side_effect = None
        for payload in (-1, [0.] * 5, [float('nan')] * 6, [200.] * 6, [True] * 6):
            sdk.get_angles.return_value = payload
            with self.assertRaises(ArmReadError):
                reader()

    def test_valid_angles_missing_timestamp_remain_immediate_other_fault(self):
        sdk = Mock(get_angles=Mock(return_value=[0.] * 6), sample_time=Mock(return_value=None))
        reader = RealPoseReader(sdk, [(-3., 3.)] * 7)
        with self.assertRaises(RuntimeError) as caught:
            reader()
        self.assertNotIsInstance(caught.exception, ArmReadError)


class FakeSDK:
    def __init__(self):
        self.calls = []
        self.pose = [0.0] * 6 + [0.5]
        self.angle_results = [1]

    def is_power_on(self):
        return 1

    def is_moving(self):
        return 0

    def get_error_information(self):
        return 0

    def get_fresh_mode(self):
        return 0

    def set_fresh_mode(self, mode):
        self.calls.append(('fresh', mode))
        return 1

    def jog_angle(self, joint_id, direction, speed, _async=True):
        self.calls.append(('jog', joint_id, direction, speed, _async))
        return 1

    def send_angle(self, axis, angle, speed, _async=False):
        self.calls.append(('angle', axis, angle, speed, _async))
        return self.angle_results.pop(0) if self.angle_results else 1

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

    def test_zero_travel_scanned_goal_holds_current_pose_without_jog(self):
        self.assertTrue(self.transport.arm())
        origin = list(self.sdk.pose)
        self.assertTrue(self.transport.submit(2, origin[2], math.radians(6),
                                              origin, origin[2], position_goal=True))
        self.transport.cycle()
        self.assertEqual(len(self.sdk.calls), 1)
        goal = self.sdk.calls[0]
        self.assertEqual(goal[0:2], ('angle', 3))
        self.assertEqual(goal[2], math.degrees(origin[2]))
        self.assertFalse(any(call[0] == 'jog' for call in self.sdk.calls))
        self.transport.stop(emergency=True)
        self.transport.cycle()
        self.assertEqual(self.sdk.calls[-1], ('stop', 0))

    def test_arm_hold_jogs_once_at_gear_speed(self):
        self.command()
        self.assertEqual(self.sdk.calls, [('jog', 3, 1, 8, True)])
        self.now += .3
        self.transport.submit(2, self.sdk.pose[2] + 0.2, math.radians(12),
                              self.sdk.pose, self.sdk.pose[2] + 0.5)
        self.transport.cycle()
        self.assertEqual(len(self.sdk.calls), 1)

    def test_jog_stops_inside_checked_boundary(self):
        self.command()
        for step in range(1, 6):
            self.sdk.pose[2] = 0.09 * step
            self.now += .25
            origin = list(self.sdk.pose)
            origin[2] = 0.0
            self.transport.submit(2, 0.5, math.radians(12), origin, 0.5)
            self.transport.cycle()
        self.transport.cycle()
        self.assertEqual(self.sdk.calls[-1], ('stop', 1))

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

    def test_other_joint_drift_does_not_lock(self):
        self.assertTrue(self.transport.arm())
        origin = list(self.sdk.pose)
        self.sdk.pose[1] = .02
        self.transport.submit(2, .2, .2, origin, .5)
        self.transport.cycle()
        self.assertEqual(self.sdk.calls[0][0], 'jog')
        self.assertTrue(self.transport.armed)

    def test_stale_gui_command_requests_stop(self):
        self.command()
        self.now += .6
        self.transport.cycle()
        self.assertEqual(self.sdk.calls[-1], ('stop', 0))
        self.assertFalse(self.transport.armed)

    def test_position_goal_replaces_jog_with_one_send_angle(self):
        self.command()
        self.now += 0.3
        self.assertTrue(self.transport.submit(
            2, 0.40, math.radians(12), self.sdk.pose, 0.40, position_goal=True))
        self.transport.cycle()
        self.assertEqual([call[0] for call in self.sdk.calls], ['jog', 'stop', 'angle'])
        self.assertEqual(self.sdk.calls[-1], ('angle', 3, round(math.degrees(0.40), 2), 8, True))
        self.now += 0.3
        self.transport.submit(2, 0.40, math.radians(12), self.sdk.pose, 0.40, position_goal=True)
        self.transport.cycle()
        self.assertEqual(len([call for call in self.sdk.calls if call[0] == 'angle']), 1)

    def test_measurement_past_the_stop_angle_stops_again(self):
        self.command()
        self.now += 0.3
        self.transport.submit(2, 0.40, math.radians(12), self.sdk.pose, 0.40, position_goal=True)
        self.transport.cycle()
        self.sdk.pose[2] = 0.45
        self.now += 0.3
        self.transport.submit(2, 0.40, math.radians(12), [0.0] * 6 + [0.5], 0.40, position_goal=True)
        self.transport.cycle()
        self.assertEqual(self.sdk.calls[-1], ('stop', 1))
        self.assertEqual(len([call for call in self.sdk.calls if call[0] == 'angle']), 1)

    def test_rejected_send_angle_stops_the_jog_and_retries(self):
        self.command()
        self.sdk.angle_results = [0, 1]
        self.now += 0.3
        self.transport.submit(2, 0.40, math.radians(12), self.sdk.pose, 0.40, position_goal=True)
        self.transport.cycle()
        kinds = [call[0] for call in self.sdk.calls]
        self.assertEqual(kinds, ['jog', 'stop', 'angle', 'angle'])
        self.assertEqual(self.sdk.calls[1], ('stop', 1))

    def test_gripper_opens_once_at_gear_speed(self):
        self.command(6)
        self.assertEqual(self.sdk.calls, [
            ('gripper_speed', 8), ('gripper_angle', 100)])
        self.now += 0.3
        self.transport.submit(6, self.sdk.pose[6] + 0.2, math.radians(12),
                              self.sdk.pose, self.sdk.pose[6] + 0.5)
        self.transport.cycle()
        self.assertEqual(len(self.sdk.calls), 2)

    def test_gripper_closes_to_zero(self):
        self.assertTrue(self.transport.arm())
        opening = self.sdk.pose[6]
        self.transport.submit(6, opening - 0.2, math.radians(6),
                              self.sdk.pose, opening - 0.5)
        self.transport.cycle()
        self.assertEqual(self.sdk.calls, [
            ('gripper_speed', 4), ('gripper_angle', 0)])

    def test_gripper_gear_change_updates_speed_only(self):
        self.command(6)
        self.now += 0.3
        opening = self.sdk.pose[6]
        self.transport.submit(6, opening + 0.2, math.radians(24),
                              self.sdk.pose, opening + 0.5)
        self.transport.cycle()
        self.assertEqual(self.sdk.calls, [
            ('gripper_speed', 8), ('gripper_angle', 100), ('gripper_speed', 16)])

    def test_gripper_release_restores_measured_angle_hold_without_any_stop(self):
        self.command(6)
        self.sdk.pose[6] = .42
        self.transport.stop(emergency=True)
        self.transport.cycle()
        self.assertEqual(self.sdk.calls, [
            ('gripper_speed', 8), ('gripper_angle', 100), ('gripper_angle', 42)])
        self.assertFalse(self.transport.motion_sent)

    def test_gripper_release_forces_new_read_before_position_hold(self):
        from unittest.mock import Mock
        self.command(6)
        events = []
        reader = Mock(spec=['force_gripper_read'], side_effect=lambda: events.append('read') or list(self.sdk.pose))
        reader.force_gripper_read.side_effect = lambda: events.append('force')
        self.transport.read_pose = reader
        original = self.sdk.set_pro_gripper_angle
        self.sdk.set_pro_gripper_angle = lambda angle: events.append(('hold', angle)) or original(angle)
        self.sdk.pose[6] = .42
        self.transport.stop(emergency=True)
        self.transport.cycle()
        self.assertEqual(events[:3], ['force', 'read', ('hold', 42)])

    def test_checked_position_goal_starts_with_send_angle_without_jog_or_stop(self):
        self.assertTrue(self.transport.arm())
        self.transport.submit(2, .40, math.radians(12), self.sdk.pose, .40,
                              position_goal=True)
        self.transport.cycle()
        self.assertEqual(self.sdk.calls, [('angle', 3, round(math.degrees(.40), 2), 8, True)])

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
