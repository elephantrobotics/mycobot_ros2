"""Exercise the actual clearance methods without importing ROS or a robot SDK."""

import ast
import math
from pathlib import Path
import threading
import time
import unittest


source = Path(__file__).resolve().parents[1] / "scripts" / "teleop_keyboard_gazebo.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
node_class = next(node for node in tree.body
                  if isinstance(node, ast.ClassDef) and node.name == "TeleopKeyboard")
methods = [node for node in node_class.body if isinstance(node, ast.FunctionDef)
           and node.name in ("_validate_hold_clearance", "_request_hold_clearance",
                             "_hold_speed_limit", "_execute_step")]
namespace = {"math": math, "threading": threading, "time": time,
             "SDK_RAD_PER_SPEED": math.pi / 120.0}
# Evaluate only the numeric settings used by these methods, never module imports.
needed = {"HOLD_SPEED_GEARS", "HOLD_ARM_URDF_VELOCITY_LIMIT",
          "HOLD_GRIPPER_SPEED_GEARS", "HOLD_GRIPPER_URDF_VELOCITY_LIMIT",
          "HOLD_MAX_ACCELERATION",
          "HOLD_COLLISION_MARGIN", "HOLD_VALIDATION_ADVANCE", "REAL_GRIPPER_SPEED",
          "REAL_SPEED", "MAX_DURATION", "MIRROR_TOLERANCE", "SETTLE_TOLERANCE"}
for node in tree.body:
    if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id in needed for target in node.targets):
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), namespace)
namespace["JOINT_LIMITS_RAD"] = [(-3.0, 3.0)] * 6 + [(0.0, 1.0)]
exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), namespace)


class ControllerStub:
    _validate_hold_clearance = namespace["_validate_hold_clearance"]
    _request_hold_clearance = namespace["_request_hold_clearance"]
    _hold_speed_limit = namespace["_hold_speed_limit"]

    def __init__(self, limit):
        self._hold_lock = threading.Lock()
        self._hold_generation = 1
        self._hold_axis = 6
        self._hold_direction = 1 if limit == 1.0 else -1
        self._hold_speed_gear = 2
        self._hold_validated_limit = limit
        self._hold_pressed = True
        self._hold_validation_pending = False
        self.warnings = []
        self._path_is_valid = lambda current, target: (True, "valid")

    def get_logger(self):
        return self

    def warning(self, message):
        self.warnings.append(message)


class HoldClearanceTests(unittest.TestCase):
    def test_gripper_gears_are_independent_and_below_urdf_limit(self):
        controller = ControllerStub(1.0)
        for gear, expected in enumerate((0.20, 0.30, 0.40, 0.50, 0.60), 1):
            controller._hold_speed_gear = gear
            self.assertAlmostEqual(controller._hold_speed_limit(6), expected)
            self.assertLess(controller._hold_speed_limit(6), 1.0)
            self.assertAlmostEqual(controller._hold_speed_limit(0), math.radians(6.0 * gear))

    def test_valid_equal_endpoint_keeps_both_directions_enabled(self):
        for boundary, position in ((1.0, 0.7892), (0.0, 0.2108)):
            with self.subTest(boundary=boundary):
                controller = ControllerStub(boundary)
                controller._validate_hold_clearance(
                    [0.0] * 6 + [position], 6, controller._hold_direction, 1, boundary)
                self.assertTrue(controller._hold_pressed)
                self.assertEqual(controller.warnings, [])

    def test_checked_urdf_boundary_does_not_start_another_worker(self):
        for boundary, position in ((1.0, 0.7892), (0.0, 0.2108)):
            controller = ControllerStub(boundary)
            controller._request_hold_clearance([0.0] * 6 + [position])
            self.assertFalse(controller._hold_validation_pending)
            self.assertFalse(hasattr(controller, "_hold_validation_worker"))
            self.assertTrue(controller._hold_pressed)

    def test_shorter_valid_corridor_replaces_previous_limit(self):
        controller = ControllerStub(1.0)
        controller._validate_hold_clearance([0.0] * 6 + [0.5], 6, 1, 1, 0.75)
        self.assertEqual(controller._hold_validated_limit, 0.75)
        self.assertTrue(controller._hold_pressed)

    def test_service_error_still_brakes(self):
        controller = ControllerStub(1.0)
        controller._path_is_valid = lambda current, target: (False, "service unavailable")
        controller._validate_hold_clearance([0.0] * 6 + [0.7892], 6, 1, 1, 1.0)
        self.assertFalse(controller._hold_pressed)
        self.assertIn("service unavailable", controller.warnings[0])


class FakeGripperSdk:
    def __init__(self, speed_result=1):
        self.calls = []
        self.speed_result = speed_result

    def is_moving(self):
        return 0

    def set_pro_gripper_speed(self, speed):
        self.calls.append(("speed", speed))
        return self.speed_result

    def set_pro_gripper_angle(self, angle):
        self.calls.append(("angle", angle))
        return 1


class StepStub(ControllerStub):
    _execute_step = namespace["_execute_step"]

    def __init__(self, armed, speed_result=1):
        super().__init__(1.0)
        self.mode = "real"
        self._armed = armed
        self._stop_requested = False
        self._active = True
        self._state_lock = threading.Lock()
        self._robot_lock = threading.Lock()
        self.mc = FakeGripperSdk(speed_result)
        self.pose = [0.0] * 6 + [0.5]
        self._fresh_feedback = lambda: (list(self.pose), [0.0] * 7)
        self._read_real_pose = lambda: list(self.pose)
        self._duration = lambda current, target: 0.0

    def _publish_trajectory(self, target, duration, gripper_only):
        self.pose = list(target)

    def info(self, message):
        pass

    def error(self, message):
        self.warnings.append(message)


class RealGripperExecutionTests(unittest.TestCase):
    def test_locked_real_controls_do_not_write_speed_or_angle(self):
        controller = StepStub(armed=False)
        controller._execute_step(controller.pose, [0.0] * 6 + [0.6], True)
        self.assertEqual(controller.mc.calls, [])

    def test_legacy_real_step_is_disabled_even_when_armed(self):
        controller = StepStub(armed=True)
        controller._execute_step(controller.pose, [0.0] * 6 + [0.6], True)
        self.assertEqual(controller.mc.calls, [])
        self.assertFalse(controller._active)

    def test_legacy_gripper_step_cannot_bypass_transport(self):
        controller = StepStub(armed=True, speed_result=0)
        controller._execute_step(controller.pose, [0.0] * 6 + [0.6], True)
        self.assertEqual(controller.mc.calls, [])


if __name__ == "__main__":
    unittest.main()
