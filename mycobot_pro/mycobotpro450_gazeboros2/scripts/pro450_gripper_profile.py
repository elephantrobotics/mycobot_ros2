"""Interpolation of measured gripper opening-units/second, without SDK edits."""
import json
import math
import threading
import time
from pathlib import Path


def measured_gripper_rate(speed, direction, profile=None):
    if profile is None:
        profile = json.loads(Path(__file__).with_name('pro450_gripper_speed.json').read_text())
    rates = {int(s): float(v) / 100.0 for s, v in profile[direction]}
    return rates[int(speed)]


class GripperMotionEstimate:
    """Continuous model from one measured start; never represents sensor data."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.motion = None

    def start(self, start, goal, rate):
        if not all(math.isfinite(v) for v in (start, goal, rate)) or rate <= 0:
            raise ValueError('invalid estimated gripper motion')
        with self.lock:
            self.motion = (float(start), float(goal), float(rate), self.clock())

    def sample(self):
        with self.lock:
            if self.motion is None:
                return None
            start, goal, rate, began = self.motion
            travel = max(0.0, self.clock() - began) * rate
            direction = 1 if goal >= start else -1
            done = travel >= abs(goal-start)
            value = goal if done else start + direction * travel
            return value, 0.0 if done else direction * rate, done

    def observe(self, value, measured):
        with self.lock:
            if self.motion is not None:
                start, goal, rate, began = self.motion
                # Confirm with a post-arrival measurement only. Sparse readings
                # during motion must not restart or jump the simulated ramp.
                if measured <= began + abs(goal-start)/rate or abs(value-goal) > 0.02:
                    return
            self.motion = None

    def hold(self):
        with self.lock:
            sample = self.sample()
            if sample is not None:
                self.motion = (sample[0], sample[0], 1.0, self.clock())


def gripper_speed_for_duration(start, goal, duration, profile=None):
    if not all(math.isfinite(v) for v in (start, goal, duration)) or duration <= 0:
        raise ValueError('invalid gripper motion')
    delta = abs(goal - start)
    if delta < 0.5:
        return None
    if profile is None:
        profile = json.loads(Path(__file__).with_name('pro450_gripper_speed.json').read_text())
    direction = 'opening' if goal > start else 'closing'
    points = sorted(profile[direction], key=lambda p: p[0])
    desired = delta / duration
    # Only measured settings are used. Avoid extrapolating a short local
    # calibration to untested higher speeds or claiming percent == velocity.
    candidates = [p for p in points if p[1] <= desired]
    selected = max(candidates, key=lambda p: p[1]) if candidates else points[0]
    return int(selected[0])
