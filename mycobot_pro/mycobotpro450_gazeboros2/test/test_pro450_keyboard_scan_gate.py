"""Real keyboard commands wait for a complete checked target; no hardware I/O."""
import ast
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


def method(name, namespace=None):
    path = Path(__file__).resolve().parents[1] / 'scripts/teleop_keyboard_gazebo.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TeleopKeyboard')
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    ns = dict(time=time, HOLD_SPEED_GEARS=(.1, .2, .3, .4, .5))
    ns.update(namespace or {})
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), ns)
    return ns[name]


def node():
    n = SimpleNamespace(mode='real', _hold_lock=threading.Lock(), _hold_axis=2,
        _hold_direction=1, _hold_pressed=True, _hold_validated_limit=.15,
        _hold_velocity=0, _hold_last_tick=time.monotonic(), _hold_validation_pending=False,
        _hold_final_target=None, _hold_goal_sent=False, _hold_validation_origin=[0]*7,
        _hold_speed_gear=2, _real_braking=False, real_transport=Mock(error=None),
        get_node_names=lambda: [], _fresh_feedback=lambda: ([0]*7, [0]*7),
        _hold_speed_limit=lambda _: .2, _publish_hold_waypoint=Mock(),
        _request_hold_clearance=Mock(), _request_collision_stop=Mock(),
        _hold_pending=None, _hold_generation=0, _armed=True, _stop_requested=False,
        real_gripper_hold_enabled=True)
    return n


class ScanGateTests(unittest.TestCase):
    def test_tick_waits_without_submitting_any_motion_until_scan_finishes(self):
        n = node()
        method('hold_tick')(n)
        n.real_transport.submit.assert_not_called()
        n._publish_hold_waypoint.assert_not_called()
        n._request_hold_clearance.assert_not_called()
        n._hold_final_target = .4
        method('hold_tick')(n)
        n._publish_hold_waypoint.assert_called_once()

    def test_real_waypoint_cannot_fall_back_to_jog_during_scan(self):
        n = node()
        publish = method('_publish_hold_waypoint')
        publish(n, [0]*7, 2, .1, .05)
        n.real_transport.submit.assert_not_called()
        n._hold_final_target = .4
        publish(n, [0]*7, 2, .1, .05)
        n.real_transport.submit.assert_called_once_with(2, .4, .2, [0]*7, .4, position_goal=True)

    def test_real_key_down_requests_only_full_scan_without_short_jog_clearance(self):
        n = node()
        n._hold_axis = None
        self.assertTrue(method('press_hold')(n, 2, 1).startswith('accepted'))
        n._request_hold_clearance.assert_not_called()
        n._request_collision_stop.assert_called_once_with([0]*7)
        n.real_transport.submit.assert_not_called()

    def test_key_release_during_scan_cancels_submission(self):
        n = node()
        method('release_hold')(n)
        n.real_transport.stop.assert_called_once_with(emergency=True)
        n._hold_final_target = .4  # Even a late result cannot send after key-up.
        method('hold_tick')(n)
        n.real_transport.submit.assert_not_called()
        n._publish_hold_waypoint.assert_not_called()

    def test_scan_failure_cannot_fall_back_to_unchecked_motion(self):
        n = node()
        n._ensure_floor_collision_scene = lambda: (False, 'service unavailable')
        n.get_logger = Mock()
        method('_scan_collision_stop', dict(
            HOLD_SCAN_GRID=.01, HOLD_SCAN_FINE=.001, HOLD_STOP_MARGIN=.01))(
                n, [0]*7, 2, 0, 0, .4)
        self.assertFalse(n._hold_pressed)
        self.assertIsNone(n._hold_final_target)
        method('hold_tick')(n)
        n.real_transport.submit.assert_not_called()
        n._publish_hold_waypoint.assert_not_called()


if __name__ == '__main__':
    unittest.main()
