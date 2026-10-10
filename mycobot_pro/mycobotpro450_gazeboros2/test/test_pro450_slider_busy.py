"""Exercise command admission and GUI buttons without ROS, Qt or hardware."""
import ast
import math
from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace, MethodType
import unittest
from unittest.mock import Mock


scripts = Path(__file__).resolve().parents[1] / 'scripts'


def load_methods(file, cls, names):
    tree = ast.parse((scripts / file).read_text(encoding='utf-8'))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
    ns = dict(math=math, time=time, queue=queue, JointState=Mock, Bool=SimpleNamespace,
              COMMAND_JOINTS=['joint1'] * 7, RAD_DISPLAY_DECIMALS=4)
    exec(compile(ast.Module(body=[n for n in node.body if isinstance(n, ast.FunctionDef)
                                and n.name in names], type_ignores=[]), file, 'exec'), ns)
    return ns


class SliderBusyTests(unittest.TestCase):
    def gui_node(self):
        ns = load_methods('pro450_slider_gui.py', 'SliderGuiNode',
                          {'execute', '_busy_cb', '_status_cb', 'command_busy'})
        obj = SimpleNamespace(_lock=threading.Lock(), _command_pending=False,
                              _command_busy=False, _status='', target_pub=Mock(),
                              force_target_pub=Mock(), get_clock=Mock())
        for name in ('execute', '_busy_cb', '_status_cb', 'command_busy'):
            setattr(obj, name, MethodType(ns[name], obj))
        return obj

    def test_submission_blocks_duplicate_before_backend_ack(self):
        node = self.gui_node()
        self.assertTrue(node.execute([0] * 7, 20))
        self.assertTrue(node.command_busy())
        self.assertFalse(node.execute([0] * 7, 20))
        node.target_pub.publish.assert_called_once()

    def test_old_idle_heartbeat_does_not_unlock_pending_submission(self):
        node = self.gui_node()
        node.execute([0] * 7, 20)
        node._busy_cb(SimpleNamespace(data=False))
        self.assertTrue(node.command_busy())
        node._busy_cb(SimpleNamespace(data=True))
        node._busy_cb(SimpleNamespace(data=False))
        self.assertFalse(node.command_busy())

    def test_rejection_before_validation_releases_pending(self):
        node = self.gui_node()
        node.execute([0] * 7, 20)
        node._status_cb(SimpleNamespace(data='Rejected: keyboard controller is running.'))
        self.assertFalse(node.command_busy())

    def test_terminal_status_does_not_unlock_during_backend_cleanup(self):
        node = self.gui_node()
        node.execute([0] * 7, 20)
        node._busy_cb(SimpleNamespace(data=True))
        node._status_cb(SimpleNamespace(data='Real robot command failed: timeout'))
        self.assertTrue(node.command_busy())
        self.assertFalse(node.execute([0] * 7, 20, force_collision=True))
        node._busy_cb(SimpleNamespace(data=False))
        self.assertFalse(node.command_busy())

    def test_failed_publish_does_not_leave_gui_busy(self):
        node = self.gui_node()
        node.target_pub.publish.side_effect = RuntimeError('publisher failed')
        with self.assertRaises(RuntimeError):
            node.execute([0] * 7, 20)
        self.assertFalse(node.command_busy())

    def test_submit_disables_buttons_immediately(self):
        ns = load_methods('pro450_slider_gui.py', 'Pro450SliderWindow', {'_execute'})
        node = self.gui_node()
        node.snapshot = lambda: (None, 0, False, 'Ready')
        obj = SimpleNamespace(node=node, _target_positions_rad=lambda: [0] * 7,
                              speed_box=Mock(value=Mock(return_value=20)),
                              execute_button=Mock(), force_execute_button=Mock(),
                              status_label=Mock())
        ns['_execute'](obj)
        obj.execute_button.setEnabled.assert_called_once_with(False)
        obj.force_execute_button.setEnabled.assert_called_once_with(False)

    def test_fresh_feedback_cannot_reenable_buttons_while_busy(self):
        ns = load_methods('pro450_slider_gui.py', 'Pro450SliderWindow', {'_refresh'})
        node = self.gui_node()
        node.environment = 'simulation'
        node.snapshot = lambda: ([0] * 7, time.monotonic(), True, 'Validating...')
        obj = SimpleNamespace(node=node, target_initialized=True, actual_labels=[Mock() for _ in range(7)],
                              execute_button=Mock(), force_execute_button=Mock(), status_label=Mock(),
                              last_status=None)
        node._busy_cb(SimpleNamespace(data=True))
        ns['_refresh'](obj)
        obj.execute_button.setEnabled.assert_called_with(False)
        obj.force_execute_button.setEnabled.assert_called_with(False)
        node._busy_cb(SimpleNamespace(data=False))
        ns['_refresh'](obj)
        obj.execute_button.setEnabled.assert_called_with(True)

    def test_backend_queue_advertises_busy_before_enqueue(self):
        ns = load_methods('slider_control_gazebo.py', 'SliderControl',
                          {'_queue_real_motion', '_publish_command_busy'})
        obj = SimpleNamespace(_state_lock=threading.RLock(), _command_active=False,
                              _stop_event=threading.Event(), busy_pub=Mock(),
                              command_queue=Mock(), _publish_status=Mock())
        obj._publish_command_busy = MethodType(ns['_publish_command_busy'], obj)
        def enqueue(_):
            self.assertTrue(obj.busy_pub.publish.call_args[0][0].data)
        obj.command_queue.put_nowait.side_effect = enqueue
        ns['_queue_real_motion'](obj, [0] * 7, .2)
        self.assertTrue(obj._command_active)

    def test_simulation_rejection_releases_busy_in_finally(self):
        ns = load_methods('slider_control_gazebo.py', 'SliderControl',
                          {'_validate_and_execute', '_publish_command_busy'})
        obj = SimpleNamespace(_state_lock=threading.RLock(), _command_active=True,
                              _publish_status=Mock(), _path_is_valid=lambda *a: (False, 'collision'),
                              busy_pub=Mock())
        obj._publish_command_busy = MethodType(ns['_publish_command_busy'], obj)
        ns['_validate_and_execute'](obj, [0] * 7, [0] * 7, .2, False)
        self.assertFalse(obj._command_active)
        self.assertFalse(obj.busy_pub.publish.call_args[0][0].data)


if __name__ == '__main__':
    unittest.main()
