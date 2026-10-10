"""ROS-free checks of measured publication and Actual sample-age handling."""
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
from pro450_real_keyboard import RealPoseReader
from pro450_real_mirror import RealPoseBuffer


def message():
    return SimpleNamespace(header=SimpleNamespace(stamp=None))


namespace = dict(time=time, Time=SimpleNamespace, JointState=message, Float64MultiArray=SimpleNamespace)
tree = ast.parse((scripts / 'pro450_feedback.py').read_text())
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)],
                        type_ignores=[]), 'pro450_feedback.py', 'exec'), namespace)


class FeedbackTimeTests(unittest.TestCase):
    def setUp(self):
        self.reader = RealPoseReader(object(), [(-3, 3)] * 6 + [(0, 1)])
        self.reader.seed_gripper(.3)
        self.node = SimpleNamespace(_sample_publish_lock=threading.Lock(), _real_mirror=RealPoseBuffer(),
                                    real_joint_pub=Mock(), real_age_pub=Mock())
        self.joints = ['joint' + str(i) for i in range(1, 7)] + ['gripper_controller']

    def test_received_time_survives_publication_and_duplicates_are_not_new_samples(self):
        mono, wall = time.monotonic() - .2, time.time() - .2
        self.reader.observe_arm([10] * 6, mono, wall)
        namespace['publish_feedback'](self.node, [0] * 7, self.reader, self.joints)
        msg = self.node.real_joint_pub.publish.call_args[0][0]
        self.assertAlmostEqual(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9, wall, places=6)
        self.assertEqual(self.node._real_mirror.latest()[0], mono)
        self.assertAlmostEqual(msg.position[0], math.radians(10))
        namespace['publish_feedback'](self.node, [0] * 7, self.reader, self.joints)
        self.node.real_joint_pub.publish.assert_called_once()

    def test_stale_pose_does_not_get_new_publication_timestamp(self):
        self.reader.observe_arm([10] * 6, time.monotonic() - 5, time.time() - 5)
        namespace['publish_feedback'](self.node, [0] * 7, self.reader, self.joints)
        self.node.real_joint_pub.publish.assert_not_called()
        self.assertIsNone(self.node._real_mirror.latest())

    def test_new_arm_is_published_before_slow_gripper_query_finishes(self):
        publisher = namespace['attach_feedback']
        callback = []
        self.node.mc = SimpleNamespace(set_angle_callback=callback.append)
        self.node._on_real_sample = lambda pose: namespace['publish_feedback'](
            self.node, pose, self.reader, self.joints)
        publisher(self.node, self.reader)
        old_gripper_stamp = self.reader.gripper_time
        callback[0](SimpleNamespace(angles=(20,) * 6, monotonic=time.monotonic(), wall_time=time.time()))
        msg = self.node.real_joint_pub.publish.call_args[0][0]
        self.assertAlmostEqual(msg.position[0], math.radians(20))
        self.assertEqual(msg.position[6], .3)
        self.assertEqual(self.reader.gripper_time, old_gripper_stamp)

    def test_gui_uses_source_stamp_rather_than_callback_time(self):
        tree = ast.parse((scripts / 'pro450_slider_gui.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SliderGuiNode')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_joint_cb')
        ns = dict(time=time, math=math, COMMAND_JOINTS=self.joints)
        exec(compile(ast.Module(body=[method], type_ignores=[]), 'pro450_slider_gui.py', 'exec'), ns)
        node = SimpleNamespace(environment='real', _lock=threading.Lock())
        stamp = time.time() - 5
        msg = SimpleNamespace(name=self.joints, position=[0] * 7, velocity=[0] * 7,
                              header=SimpleNamespace(stamp=SimpleNamespace(sec=int(stamp),
                                nanosec=int((stamp - int(stamp)) * 1e9))))
        ns['_joint_cb'](node, msg)
        self.assertGreater(time.monotonic() - node._actual_time, 4.9)


if __name__ == '__main__':
    unittest.main()
