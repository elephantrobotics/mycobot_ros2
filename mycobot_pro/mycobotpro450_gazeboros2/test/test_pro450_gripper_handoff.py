"""Regressions for the measured arrival -> old arm-cache gripper reversal."""
import ast
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

scripts = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
from pro450_gripper_profile import GripperMotionEstimate


def extract(filename, classname, names, namespace):
    tree = ast.parse((scripts / filename).read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == classname)
    body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=body, type_ignores=[]), filename, 'exec'), namespace)
    return namespace


class GripperHandoffTests(unittest.TestCase):
    def setUp(self):
        self.now = [10.0]
        self.model = GripperMotionEstimate(clock=lambda: self.now[0])

    def test_measured_closing_arrival_keeps_goal_after_estimate_finishes(self):
        # er trace: old arm buffer .48, estimated goal .29, real readback .29.
        self.model.observe(.48, 10)
        self.now[0] = 11
        self.model.start(.48, .29, .29)
        self.now[0] = 12
        self.assertEqual(self.model.render(), (.29, 0, 1))
        self.model.observe(.29, 12)
        self.assertIsNone(self.model.sample())
        for t in (12, 12.02, 12.1, 15):
            self.now[0] = t
            self.assertEqual(self.model.render(), (.29, 0, 0))

    def test_quantized_opening_arrival_corrects_continuously_to_real_readback(self):
        self.model.observe(.29, 10)
        self.now[0] = 11
        self.model.start(.29, .49, .29)
        self.now[0] = 12
        self.assertEqual(self.model.render(), (.49, 0, 1))
        self.model.observe(.48, 12)
        self.assertIsNone(self.model.sample())
        self.assertEqual(self.model.render(), (.49, 0, 2))
        outputs = []
        for i in range(16):
            self.now[0] = 12 + i*.02
            outputs.append(self.model.render())
        positions = [p[0] for p in outputs]
        self.assertTrue(all(.48 <= p <= .49 for p in positions))
        self.assertTrue(all(a >= b for a, b in zip(positions, positions[1:])))
        self.assertLessEqual(max(abs(a-b)/.02 for a, b in zip(positions, positions[1:])), .1)
        self.assertTrue(all(p[1] == 0 for p in outputs))
        self.assertEqual(outputs[-1], (.48, 0, 0))

    def test_stale_or_duplicate_cache_cannot_restore_pre_motion_opening(self):
        self.model.observe(.48, 10)
        self.now[0] = 11
        self.model.start(.48, .29, .29)
        self.now[0] = 12
        self.model.observe(.29, 12)
        self.model.observe(.48, 10)
        self.model.observe(.48, 12)
        self.assertEqual(self.model.render(), (.29, 0, 0))

    def test_sparse_read_during_motion_cannot_interrupt_calibrated_ramp(self):
        self.model.observe(.3, 10)
        self.now[0] = 11
        self.model.start(.3, .5, .2)
        self.now[0] = 11.5
        self.model.observe(.34, 11.5)
        self.assertAlmostEqual(self.model.render()[0], .4)
        self.assertEqual(self.model.render()[1:], (.2, 1))
        self.now[0] = 12.1
        self.model.observe(.4, 12.1)  # Real arrival still unconfirmed.
        self.assertEqual(self.model.render(), (.5, 0, 1))

    def test_same_new_readback_does_not_keep_restarting_correction(self):
        self.model.observe(.49, 10)
        self.now[0] = 11
        self.model.observe(.48, 11)
        self.now[0] = 11.1
        self.model.observe(.48, 11.1)
        self.now[0] = 11.21
        self.assertEqual(self.model.render(), (.48, 0, 0))

    def test_new_measurement_retargets_from_current_output_without_jump(self):
        self.model.observe(.5, 10)
        self.now[0] = 11
        self.model.observe(.48, 11)
        self.now[0] = 11.1
        before = self.model.render()[0]
        self.model.observe(.49, 11.1)
        self.assertAlmostEqual(self.model.render()[0], before)
        self.now[0] = 11.6
        self.assertEqual(self.model.render(), (.49, 0, 0))

    def test_new_command_during_correction_starts_at_current_display_position(self):
        self.model.observe(.49, 10)
        self.now[0] = 11
        self.model.observe(.48, 11)
        self.now[0] = 11.1
        before = self.model.render()[0]
        self.model.start(.48, .29, .29)
        self.assertEqual(self.model.render()[0], before)
        self.assertEqual(self.model.sample()[0], .48)  # Actual start remains measured.
        self.now[0] = 12.1
        self.model.observe(.29, 12.1)
        self.assertEqual(self.model.render(), (.29, 0, 0))

    def test_stop_freezes_output_then_accepts_new_stopped_position(self):
        self.model.start(.6, .4, .1)
        self.now[0] = 11
        self.model.hold()
        self.now[0] = 12
        self.assertAlmostEqual(self.model.render()[0], .5)
        self.assertEqual(self.model.render()[1], 0)
        self.model.observe(.45, 12)
        self.assertAlmostEqual(self.model.render()[0], .5)
        self.now[0] = 13
        self.assertEqual(self.model.render(), (.45, 0, 0))

    def test_invalid_readbacks_cannot_seed_or_modify_output(self):
        for value, stamp in ((-1, 10), (1.1, 10), (math.nan, 10), (.5, math.nan), (True, 10)):
            self.model.observe(value, stamp)
        self.assertIsNone(self.model.render())
        self.model.observe(.29, 10)
        self.model.observe(-1, 11)
        self.assertEqual(self.model.render(), (.29, 0, 0))

    def test_bridge_overrides_old_cached_opening_and_false_velocity_after_arrival(self):
        ns = extract('slider_control_gazebo.py', 'SliderControl', {'_mirror_loop'},
                     dict(time=time, rclpy=SimpleNamespace(ok=lambda: True),
                          Float64MultiArray=SimpleNamespace, publish_age=Mock()))
        for value in (.29, .49):
            with self.subTest(value=value):
                model = GripperMotionEstimate()
                model.observe(value, time.monotonic())
                stop = Mock()
                stop.is_set.side_effect = [False, True]
                obj = SimpleNamespace(_mirror_stop=stop, _pose_reader=None,
                    _real_mirror=SimpleNamespace(render=lambda: ([0.]*6+[.48], [0.]*6+[-1.969744])),
                    _gripper_model=model, gripper_estimate_pub=Mock(),
                    _publish_trajectory=Mock(), _publish_gripper_trajectory=Mock(), get_logger=Mock())
                ns['_mirror_loop'](obj)
                pose, duration, velocity = obj._publish_trajectory.call_args[0]
                self.assertEqual(pose[6], value)
                self.assertEqual(velocity[6], 0)
                self.assertEqual(duration, .04)

    def test_bridge_does_not_publish_arm_history_without_initial_gripper_output(self):
        ns = extract('slider_control_gazebo.py', 'SliderControl', {'_mirror_loop'},
                     dict(time=time, rclpy=SimpleNamespace(ok=lambda: True),
                          Float64MultiArray=SimpleNamespace, publish_age=Mock()))
        stop = Mock()
        stop.is_set.side_effect = [False, True]
        obj = SimpleNamespace(_mirror_stop=stop, _pose_reader=None,
            _real_mirror=SimpleNamespace(render=lambda: ([0.]*7, [0.]*7)),
            _gripper_model=self.model, gripper_estimate_pub=Mock(),
            _publish_trajectory=Mock(), _publish_gripper_trajectory=Mock(), get_logger=Mock())
        ns['_mirror_loop'](obj)
        obj._publish_trajectory.assert_not_called()
        obj._publish_gripper_trajectory.assert_not_called()

    def test_gui_distinguishes_correction_from_speed_based_estimate(self):
        ns = extract('pro450_slider_gui.py', 'SliderGuiNode', {'_estimate_cb', 'gripper_estimate'},
                     dict(time=time, math=math))
        node = SimpleNamespace(_lock=threading.Lock())
        ns['_estimate_cb'](node, SimpleNamespace(data=[.49, 0, 2]))
        self.assertEqual(ns['gripper_estimate'](node), (.49, 0, 2))
        ns['_estimate_cb'](node, SimpleNamespace(data=[.48, 0, 0]))
        self.assertIsNone(ns['gripper_estimate'](node))


if __name__ == '__main__':
    unittest.main()
