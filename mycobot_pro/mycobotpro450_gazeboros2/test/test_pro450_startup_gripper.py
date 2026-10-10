"""Startup retry deadline tests; no ROS imports or physical connection."""
import ast
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


def read_method():
    path = Path(__file__).resolve().parents[1] / 'scripts/slider_control_gazebo.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SliderControl')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_read_startup_gripper')
    ns = dict(math=math, threading=threading, time=time)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), ns)
    return ns['_read_startup_gripper']


class StartupGripperTests(unittest.TestCase):
    def obj(self, replies):
        sdk = Mock()
        sdk.get_pro_gripper_angle.side_effect = replies
        return SimpleNamespace(mc=sdk, get_logger=Mock(return_value=Mock()))

    def test_minus_one_retries_until_real_angle(self):
        obj = self.obj([-1, -1, 44])
        self.assertEqual(read_method()(obj, timeout=1), 44)
        self.assertEqual(obj.mc.get_pro_gripper_angle.call_count, 3)
        obj.mc.close.assert_not_called()
        self.assertEqual([c[0] for c in obj.mc.method_calls], ['get_pro_gripper_angle'] * 3)

    def test_invalid_values_and_exception_cannot_seed_initial_pose(self):
        obj = self.obj([None, True, float('nan'), 101, TimeoutError('no reply'), 0])
        self.assertEqual(read_method()(obj, timeout=2), 0)
        self.assertEqual(obj.mc.get_pro_gripper_angle.call_count, 6)

    def test_valid_closed_and_open_angles_are_accepted(self):
        for value in (0, 100):
            obj = self.obj([value])
            self.assertEqual(read_method()(obj), value)
            obj.mc.get_pro_gripper_angle.assert_called_once()

    def test_persistent_failure_times_out_and_stops_retrying(self):
        obj = self.obj(None)
        obj.mc.get_pro_gripper_angle.side_effect = None
        obj.mc.get_pro_gripper_angle.return_value = -1
        with self.assertRaisesRegex(RuntimeError, 'timed out after 0.2s'):
            read_method()(obj, timeout=.2)
        obj.mc.close.assert_called_once()
        count = obj.mc.get_pro_gripper_angle.call_count
        time.sleep(.15)
        self.assertEqual(obj.mc.get_pro_gripper_angle.call_count, count)

    def test_blocked_query_is_interrupted_at_deadline(self):
        obj = self.obj(None)
        release = threading.Event()
        obj.mc.get_pro_gripper_angle.side_effect = lambda: release.wait(2) or -1
        obj.mc.close.side_effect = release.set
        began = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            read_method()(obj, timeout=.1)
        self.assertLess(time.monotonic() - began, .5)
        obj.mc.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
