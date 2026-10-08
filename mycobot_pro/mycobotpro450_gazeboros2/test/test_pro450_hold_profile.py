"""ROS-independent checks for Pro450 simulation hold/braking math."""

import os
import sys
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from pro450_hold_profile import PositionVelocityEstimator, hold_setpoint  # noqa: E402


class HoldProfileTests(unittest.TestCase):
    def test_high_gripper_gear_can_start_near_both_joint_boundaries(self):
        for position, direction, boundary in ((0.9, 1, 1.0), (0.1, -1, 0.0)):
            with self.subTest(direction=direction):
                endpoint, velocity, duration = hold_setpoint(
                    position, 0.0, 0.0, direction, True, boundary,
                    0.05, 0.60, 1.40)
                self.assertGreater(direction * (endpoint - position), 0.0)
                self.assertGreater(direction * velocity, 0.0)
                self.assertLessEqual(abs(velocity), 0.60)
                self.assertLessEqual(abs(endpoint - position) / duration, 1.0)
                self.assertTrue(0.0 <= endpoint <= 1.0)

    def test_stationary_position_has_zero_speed(self):
        estimator = PositionVelocityEstimator()
        result = None
        for index in range(12):
            result = estimator.update(index * 0.02, [0.0, -0.003616616])
        self.assertIsNotNone(result)
        self.assertEqual(result, [0.0, 0.0])

    def test_position_change_estimates_speed(self):
        estimator = PositionVelocityEstimator()
        result = None
        for index in range(12):
            stamp = index * 0.02
            result = estimator.update(stamp, [0.1 * stamp])
        self.assertAlmostEqual(result[0], 0.1, places=6)

    def test_clock_reset_discards_old_velocity(self):
        estimator = PositionVelocityEstimator()
        estimator.update(10.0, [1.0])
        self.assertIsNone(estimator.update(0.0, [0.0]))

    def test_no_collision_clearance_means_no_motion(self):
        position, velocity, _ = hold_setpoint(
            0.0, 0.0, 0.0, 1, True, 0.0, 0.05, 0.1, 0.2)
        self.assertEqual(position, 0.0)
        self.assertEqual(velocity, 0.0)

    def test_press_ramps_up_within_validated_corridor(self):
        position, velocity, _ = hold_setpoint(
            0.0, 0.0, 0.0, 1, True, 0.20, 0.05, 0.1, 0.2)
        self.assertGreater(velocity, 0.0)
        self.assertLessEqual(velocity, 0.010000001)
        self.assertGreater(position, 0.0)
        self.assertLess(position, 0.20)

    def test_release_decelerates_without_reversing(self):
        position, velocity, _ = hold_setpoint(
            0.05, 0.1, 0.1, 1, False, 0.20, 0.05, 0.1, 0.2)
        self.assertGreaterEqual(velocity, 0.0)
        self.assertLess(velocity, 0.1)
        self.assertGreater(position, 0.05)

    def test_brakes_before_clearance_is_exhausted(self):
        position, velocity, _ = hold_setpoint(
            0.13, 0.08, 0.08, 1, True, 0.20, 0.05, 0.1, 0.2)
        self.assertLess(velocity, 0.08)
        self.assertLessEqual(position, 0.165)

    def test_negative_direction(self):
        position, velocity, _ = hold_setpoint(
            0.0, 0.0, 0.0, -1, True, -0.20, 0.05, 0.1, 0.2)
        self.assertLess(position, 0.0)
        self.assertLess(velocity, 0.0)

    def test_repeated_setpoints_release_and_stay_within_clearance(self):
        position = actual_velocity = command_velocity = 0.0
        for index in range(100):
            _, command_velocity, duration = hold_setpoint(
                position, actual_velocity, command_velocity, 1,
                index < 30, 0.20, 0.05, 0.1, 0.2)
            # A conservative first-order stand-in for controller response.
            actual_velocity += (command_velocity - actual_velocity) * 0.05 / duration
            position += actual_velocity * 0.05
            self.assertLessEqual(position, 0.165001)
        self.assertLess(abs(actual_velocity), 0.005)

    def test_fastest_gear_has_braking_room_and_respects_urdf_speed(self):
        speed = math.radians(30.0)
        acceleration = 1.40
        clearance = 0.55
        margin = 0.01
        urdf_velocity_limit = 1.0
        self.assertLess(speed, urdf_velocity_limit)
        braking_distance = speed * speed / (2.0 * acceleration)
        reaction_distance = speed * 0.25
        self.assertLess(braking_distance + reaction_distance + margin, clearance)
        endpoint, command_velocity, duration = hold_setpoint(
            0.0, 0.0, 0.0, 1, True, clearance, 0.05,
            speed, acceleration, collision_margin=margin)
        self.assertGreater(endpoint, 0.0)
        self.assertLessEqual(command_velocity, speed)
        self.assertLessEqual(endpoint / duration, urdf_velocity_limit)


if __name__ == "__main__":
    unittest.main()
