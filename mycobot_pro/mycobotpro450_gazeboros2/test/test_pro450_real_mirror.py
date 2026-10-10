"""ROS-independent checks for delayed real-pose rendering."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from pro450_real_mirror import (  # noqa: E402
    INITIAL_PERIOD_S, MAX_PERIOD_S, MIN_PERIOD_S, RENDER_MARGIN_S, RealPoseBuffer,
)


def pose_at(value):
    return [value] * 7


class RealPoseBufferTests(unittest.TestCase):
    def test_interpolation_uses_measured_slope(self):
        clock = {"now": 0.0}
        buffer = RealPoseBuffer(clock=lambda: clock["now"])
        buffer.push(pose_at(0.0), when=0.0)
        buffer.push(pose_at(1.0), when=0.2)
        period = 0.7 * INITIAL_PERIOD_S + 0.3 * 0.2
        rendered = buffer.render(now=0.2 + period + RENDER_MARGIN_S - 0.1)
        self.assertIsNotNone(rendered)
        pose, velocity = rendered
        self.assertAlmostEqual(pose[0], 0.5, places=6)
        self.assertAlmostEqual(velocity[0], 5.0, places=6)

    def test_render_never_passes_the_newest_sample(self):
        buffer = RealPoseBuffer(clock=lambda: 0.0)
        buffer.push(pose_at(0.2), when=0.0)
        buffer.push(pose_at(0.8), when=0.2)
        pose, velocity = buffer.render(now=0.6)
        self.assertEqual(pose, pose_at(0.8))
        self.assertEqual(velocity, [0.0] * 7)

    def test_stale_measurement_is_not_rendered_as_live(self):
        buffer = RealPoseBuffer(clock=lambda: 0.0)
        buffer.push(pose_at(0.8), when=0.2)
        self.assertIsNone(buffer.render(now=5.0))

    def test_period_estimate_converges_and_stays_bounded(self):
        buffer = RealPoseBuffer(clock=lambda: 0.0)
        for index in range(1, 40):
            buffer.push(pose_at(0.0), when=index * 0.2)
        pose, _velocity = buffer.render(now=40 * 0.2)
        self.assertEqual(pose, pose_at(0.0))
        fast = RealPoseBuffer(clock=lambda: 0.0)
        for index in range(1, 80):
            fast.push(pose_at(float(index)), when=index * 0.02)
        newest = 79.0
        rendered, _velocity = fast.render(now=79 * 0.02)
        minimum_lag = (MIN_PERIOD_S + RENDER_MARGIN_S) / 0.02
        self.assertLessEqual(rendered[0], newest - minimum_lag + 0.2)
        self.assertGreater(rendered[0], newest - (MAX_PERIOD_S + RENDER_MARGIN_S) / 0.02)

    def test_a_long_gap_jumps_to_the_new_sample(self):
        buffer = RealPoseBuffer(clock=lambda: 0.0)
        buffer.push(pose_at(0.0), when=0.0)
        buffer.push(pose_at(3.0), when=1.0)
        pose, velocity = buffer.render(now=0.5 + INITIAL_PERIOD_S + RENDER_MARGIN_S)
        self.assertEqual(pose, pose_at(3.0))
        self.assertEqual(velocity, [0.0] * 7)

    def test_non_finite_pose_is_rejected(self):
        buffer = RealPoseBuffer(clock=lambda: 0.0)
        with self.assertRaises(ValueError):
            buffer.push([math.nan] + [0.0] * 6, when=0.0)
        with self.assertRaises(ValueError):
            buffer.push([0.0] * 6, when=0.0)
        self.assertIsNone(buffer.latest())


if __name__ == "__main__":
    unittest.main()
