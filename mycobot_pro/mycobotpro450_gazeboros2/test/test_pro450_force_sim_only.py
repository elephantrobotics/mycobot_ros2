"""Force commands never publish from real GUI or enter the real SDK queue."""
import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

scripts = Path(__file__).resolve().parents[1] / 'scripts'


def method(filename, classname, name, namespace=None):
    tree = ast.parse((scripts / filename).read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == classname)
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    ns = namespace or {}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), filename, 'exec'), ns)
    return ns[name]


class ForceSimOnlyTests(unittest.TestCase):
    def test_real_window_click_only_shows_mode_notice_even_while_busy(self):
        dialog = Mock()
        click = method('pro450_slider_gui.py', 'Pro450SliderWindow', '_force_execute',
                       dict(QMessageBox=dialog))
        node = SimpleNamespace(environment='real', command_busy=Mock(return_value=True), execute=Mock())
        window = SimpleNamespace(node=node, _target_positions_rad=Mock())
        click(window)
        dialog.warning.assert_called_once()
        self.assertEqual(dialog.warning.call_args[0][1], 'Force Execute Unavailable')
        dialog.information.assert_not_called()
        node.execute.assert_not_called()
        node.command_busy.assert_not_called()
        window._target_positions_rad.assert_not_called()

    def test_real_node_direct_force_submission_cannot_publish_or_mark_pending(self):
        execute = method('pro450_slider_gui.py', 'SliderGuiNode', 'execute')
        node = SimpleNamespace(environment='real', _lock=threading.Lock(),
            _command_pending=False, _command_busy=False,
            target_pub=Mock(), force_target_pub=Mock(), get_clock=Mock())
        self.assertFalse(execute(node, [0]*7, 50, force_collision=True))
        node.target_pub.publish.assert_not_called()
        node.force_target_pub.publish.assert_not_called()
        node.get_clock.assert_not_called()
        self.assertFalse(node._command_pending)

    def test_real_backend_rejects_external_force_topic_before_any_motion_path(self):
        callback = method('slider_control_gazebo.py', 'SliderControl', '_force_target_cb')
        for enabled in (True, False):
            with self.subTest(real_motion_enabled=enabled):
                node = SimpleNamespace(mode=2, _real_motion_enabled=enabled, mc=Mock(),
                    _publish_status=Mock(), _handle_target=Mock(), _queue_real_motion=Mock())
                callback(node, object())
                node._publish_status.assert_called_once()
                node._handle_target.assert_not_called()
                node._queue_real_motion.assert_not_called()
                self.assertEqual(node.mc.method_calls, [])

    def test_simulation_backend_routes_only_to_force_simulation_handler(self):
        callback = method('slider_control_gazebo.py', 'SliderControl', '_force_target_cb')
        node = SimpleNamespace(mode=1, mc=None, _handle_target=Mock(), _publish_status=Mock())
        msg = object()
        callback(node, msg)
        node._handle_target.assert_called_once_with(msg, force_collision=True)

    def test_simulation_click_keeps_confirmation_and_ten_percent_speed_cap(self):
        dialog = SimpleNamespace(information=Mock(), warning=Mock(return_value=1), Yes=1, Cancel=2)
        click = method('pro450_slider_gui.py', 'Pro450SliderWindow', '_force_execute',
                       dict(QMessageBox=dialog, FORCE_EXECUTE_MAX_SPEED_PERCENT=10))
        node = SimpleNamespace(environment='simulation', command_busy=lambda: False,
            execute=Mock(return_value=True), snapshot=lambda: (None, 0, False, 'Ready'))
        window = SimpleNamespace(node=node, _target_positions_rad=lambda: [0]*7,
            speed_box=Mock(value=Mock(return_value=50)), execute_button=Mock(),
            force_execute_button=Mock(), status_label=Mock())
        click(window)
        dialog.information.assert_not_called()
        dialog.warning.assert_called_once()
        node.execute.assert_called_once_with([0]*7, 10, force_collision=True)

    def test_real_mode_force_notice_remains_clickable_without_fresh_feedback(self):
        refresh = method('pro450_slider_gui.py', 'Pro450SliderWindow', '_refresh',
                         dict(math=math, time=SimpleNamespace(monotonic=lambda: 10)))
        node = SimpleNamespace(environment='real', snapshot=lambda: (None, 0, False, 'Ready'),
                               command_busy=lambda: True)
        window = SimpleNamespace(node=node, target_initialized=True, actual_labels=[],
            execute_button=Mock(), force_execute_button=Mock(), status_label=Mock())
        refresh(window)
        window.execute_button.setEnabled.assert_called_with(False)
        window.force_execute_button.setEnabled.assert_called_with(True)


if __name__ == '__main__':
    unittest.main()
