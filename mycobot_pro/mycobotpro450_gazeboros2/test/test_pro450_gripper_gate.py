"""Negative SDK responses must never become valid startup poses. No ROS/SDK."""
import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

scripts = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
from pro450_real_keyboard import valid_gripper_position, RealKeyboardTransport


def load_methods(filename, class_name, names, namespace):
    tree = ast.parse((scripts / filename).read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=methods, type_ignores=[]), filename, 'exec'), namespace)
    return namespace


ns = load_methods('teleop_keyboard_gazebo.py', 'TeleopKeyboard',
                  {'_read_real_pose', '_connect_read_only'},
                  dict(math=math, time=SimpleNamespace(sleep=lambda _: None),
                       valid_gripper_position=valid_gripper_position,
                       DEFAULT_PRO450_IP='unused', DEFAULT_PRO450_PORT=4500,
                       JOINT_LIMITS_RAD=[(-3, 3)] * 6 + [(0, 1)]))


class ReadOnlyFakeSdk:
    def __init__(self, gripper):
        self.gripper = iter(gripper)

    def is_power_on(self):
        return 1

    def get_error_information(self):
        return 0

    def is_moving(self):
        return 0

    def get_angles(self):
        return [0] * 6

    def get_pro_gripper_angle(self):
        return next(self.gripper)

    # No hardware-writing methods: any unexpected write fails the test.


class Reader:
    _read_real_pose = ns['_read_real_pose']
    _connect_read_only = ns['_connect_read_only']

    def __init__(self):
        self._publish_snapshot = Mock()
        self.logger = Mock()

    def get_logger(self):
        return self.logger


gate_ns = load_methods('pro450_pose_gate.py', 'Pro450PoseGate', {'_snapshot_cb'},
                       dict(math=math, COMMAND_JOINTS=['joint' + str(i) for i in range(6)] +
                            ['gripper_controller'], JOINT_LIMITS_RAD=[(-3, 3)] * 6 + [(0, 1)]))


class Gate:
    _snapshot_cb = gate_ns['_snapshot_cb']

    def __init__(self):
        self.success, self.failure_reason = False, ''
        self.max_snapshot_age_sec = 2.5
        self.output_file = 'unused'
        self._write_yaml_atomically = Mock()
        self.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**10))
        self.get_logger = Mock(return_value=Mock())


class GripperGateTests(unittest.TestCase):
    def test_negative_sdk_is_error_before_conversion(self):
        for value in (-1, -0.01, -100):
            with self.subTest(value=value), self.assertRaisesRegex(RuntimeError, 'read failed'):
                valid_gripper_position(value)

    def test_invalid_sdk_response_is_rejected(self):
        for value in (True, None, '0', math.nan, math.inf, 101):
            with self.subTest(value=value), self.assertRaises(RuntimeError):
                valid_gripper_position(value)

    def test_zero_is_valid_not_a_failure(self):
        self.assertEqual(valid_gripper_position(0), 0)
        self.assertEqual(valid_gripper_position(100), 1)

    def test_any_bad_startup_sample_blocks_publication(self):
        for index in range(5):
            with self.subTest(index=index):
                reader = Reader()
                values = [0] * 5
                values[index] = -1
                sdk = ReadOnlyFakeSdk(values)
                fake_module = SimpleNamespace(Pro450Client=lambda *_: sdk)
                with patch.dict(sys.modules, pymycobot=fake_module):
                    with self.assertRaisesRegex(RuntimeError, 'gripper read failed'):
                        reader._connect_read_only()
                reader._publish_snapshot.assert_not_called()

    def test_valid_all_joint_startup_can_publish(self):
        reader = Reader()
        sdk = ReadOnlyFakeSdk([50] * 5)
        with patch.dict(sys.modules, pymycobot=SimpleNamespace(Pro450Client=lambda *_: sdk)):
            reader._connect_read_only()
        reader._publish_snapshot.assert_called_once_with([0] * 6 + [.5])

    def test_negative_snapshot_closes_launch_gate_without_writing_file(self):
        gate = Gate()
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
                              name=gate_ns['COMMAND_JOINTS'], position=[0] * 6 + [-.01])
        gate._snapshot_cb(msg)
        self.assertFalse(gate.success)
        self.assertIn('gripper read failed', gate.failure_reason)
        gate._write_yaml_atomically.assert_not_called()

    def test_runtime_read_failure_locks_without_initial_motion_writes(self):
        sdk = ReadOnlyFakeSdk([-1])
        reader = Reader()
        reader.mc = sdk
        errors = []
        transport = RealKeyboardTransport(sdk, reader._read_real_pose, Mock(),
                                         ns['JOINT_LIMITS_RAD'], report_error=errors.append)
        transport.closed.wait = lambda _: transport.closed.set()
        transport.run()
        self.assertFalse(transport.arm())
        self.assertIsNone(transport.feedback())
        self.assertIn('gripper read failed', transport.error)
        self.assertEqual(len(errors), 1)
        transport.publish_pose.assert_not_called()


if __name__ == '__main__':
    unittest.main()
