"""Motor command boundaries, startup ownership and latest feedback, without hardware."""
import ast
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from pro450_drag_support import DragState, set_arm_enabled, set_drag_enabled
from pro450_session_support import control_commands


class DragTests(unittest.TestCase):
    def state(self):
        sdk = Mock(set_motor_enabled=Mock(return_value=1), set_pro_gripper_enabled=Mock(return_value=1))
        now = [10.]
        state = DragState(sdk, clock=lambda: now[0])
        state.observe([0.] * 6, now[0], 100.)
        return sdk, state, now

    def test_all_six_motors_only_and_toggle_not_repeated_while_pending(self):
        sdk, state, _ = self.state()
        self.assertTrue(state.request_toggle())
        self.assertFalse(state.request_toggle())
        state.apply_request()
        sdk.set_motor_enabled.assert_called_once_with(254, 0)
        self.assertFalse(state.enabled)
        self.assertTrue(state.request_toggle())
        state.apply_request()
        self.assertTrue(state.enabled)
        self.assertEqual(sdk.method_calls, [('set_motor_enabled', (254, 0), {}),
                                         ('set_pro_gripper_enabled', (0,), {}),
                                         ('set_motor_enabled', (254, 1), {}),
                                         ('set_pro_gripper_enabled', (1,), {})])

    def test_ambiguous_ack_is_unknown_and_next_explicit_request_enables(self):
        sdk, state, _ = self.state()
        sdk.set_motor_enabled.side_effect = [-1, 1]
        state.request_toggle(); state.apply_request()
        self.assertIsNone(state.enabled)
        state.request_toggle(); state.apply_request()
        sdk.set_motor_enabled.assert_called_with(254, 1)
        self.assertTrue(state.enabled)

    def test_stale_feedback_blocks_toggle_and_old_frames_never_replace_new(self):
        _, state, now = self.state()
        for index in range(1, 100):
            now[0] += .01
            self.assertTrue(state.observe([index] * 6, now[0], 100. + index))
        self.assertAlmostEqual(state.latest[2][0], math.radians(99))
        self.assertFalse(state.observe([0.] * 6, 10., 100.))
        self.assertFalse(state.observe([float('nan')] * 6, now[0] + .01, 300.))
        now[0] += .6
        self.assertFalse(state.request_toggle())

    def test_delayed_switch_does_not_write_after_feedback_expires(self):
        sdk, state, now = self.state()
        state.request_toggle()
        now[0] += .6
        state.apply_request()
        sdk.set_motor_enabled.assert_not_called()
        self.assertTrue(state.enabled)
        self.assertFalse(state.pending)

    def test_gripper_failure_marks_combined_state_unknown_and_next_m_enables_both(self):
        sdk, state, _ = self.state()
        sdk.set_pro_gripper_enabled.side_effect = [-1, 1]
        state.request_toggle(); state.apply_request()
        self.assertIsNone(state.enabled)
        self.assertIn('combined state unknown', state.message)
        state.request_toggle(); state.apply_request()
        self.assertTrue(state.enabled)
        sdk.set_motor_enabled.assert_called_with(254, 1)
        sdk.set_pro_gripper_enabled.assert_called_with(1)
        sdk.set_pro_gripper_angle.assert_not_called()
        sdk.set_pro_gripper_speed.assert_not_called()

    def test_startup_combined_enable_sends_no_position_or_speed(self):
        sdk, _, _ = self.state()
        set_drag_enabled(sdk, True)
        self.assertEqual(sdk.method_calls, [('set_motor_enabled', (254, 1), {}),
                                         ('set_pro_gripper_enabled', (1,), {})])

    def test_real_only_process_allowlist(self):
        commands = control_commands('drag', 'real', '/installed', '/python')
        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0][2].endswith('pro450_drag_follow.py'))
        with self.assertRaises(ValueError):
            control_commands('drag', 'simulation', '/installed', '/python')
        source = (ROOT / 'launch/drag_follow_gazebo.launch.py').read_text(encoding='utf-8')
        self.assertNotIn('DeclareLaunchArgument', source)
        self.assertIn("'environment': 'real'", source)

    def test_startup_enable_is_owned_by_drag_program_not_session(self):
        session = (ROOT / 'scripts/pro450_control_session.py').read_text(encoding='utf-8')
        self.assertNotIn('set_arm_enabled', session)
        self.assertNotIn('Pro450Client', session)
        source = (ROOT / 'scripts/pro450_drag_follow.py').read_text(encoding='utf-8')
        self.assertEqual(source.count('set_drag_enabled(self.sdk, True)'), 1)
        self.assertIn('PublicPro450Client', source)

    def test_drag_program_never_sends_real_motion_or_gripper_position_commands(self):
        tree = ast.parse((ROOT / 'scripts/pro450_drag_follow.py').read_text(encoding='utf-8'))
        calls = {n.func.attr for n in ast.walk(tree)
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
        self.assertFalse(calls & {'send_angles', 'send_angle', 'jog_angle', 'power_on',
            'set_pro_gripper_angle', 'set_pro_gripper_speed'})


if __name__ == '__main__':
    unittest.main()
