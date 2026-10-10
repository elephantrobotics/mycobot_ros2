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
    """Independent display output; its estimates never replace sensor data.

    sample() retains the physical motion's completion semantics. render() also
    retains the last output after arrival, so arm history cannot take over the
    gripper with an older cached value. Source flags: measured=0, estimated=1,
    correcting to a new measurement=2.
    """
    CORRECTION_RATE = 0.1
    CORRECTION_MIN_TIME = 0.2

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.motion = None
        self._display_motion = None
        self._output = None
        self._correction = None
        self._last_measurement = None
        self._holding = False

    @staticmethod
    def _sample_motion(motion, now):
        start, goal, rate, began = motion
        travel = max(0.0, now - began) * rate
        direction = 1 if goal >= start else -1
        done = travel >= abs(goal-start)
        return (goal if done else start + direction * travel,
                0.0 if done else direction * rate, done)

    def start(self, start, goal, rate):
        if not all(math.isfinite(v) for v in (start, goal, rate)) or rate <= 0:
            raise ValueError('invalid estimated gripper motion')
        with self.lock:
            now = self.clock()
            current = self._render(now)
            display_start = float(start) if current is None else current[0]
            self.motion = (float(start), float(goal), float(rate), now)
            self._display_motion = (display_start, float(goal), float(rate), now)
            self._correction = None
            self._holding = False

    def sample(self):
        with self.lock:
            if self.motion is None:
                return None
            return self._sample_motion(self.motion, self.clock())

    def _render(self, now):
        if self._display_motion is not None:
            value, velocity, _ = self._sample_motion(self._display_motion, now)
            self._output = value
            return value, velocity, 1.0
        if self._correction is not None:
            start, goal, began, duration = self._correction
            u = min(1.0, max(0.0, (now-began)/duration))
            self._output = start + (goal-start) * u*u*(3.0-2.0*u)
            if u < 1.0:
                # Each short trajectory ends at rest. Do not feed the arm
                # history's cached-value slope into the gripper controller.
                return self._output, 0.0, 2.0
            self._correction = None
        return None if self._output is None else (self._output, 0.0, 0.0)

    def render(self):
        with self.lock:
            return self._render(self.clock())

    def observe(self, value, measured):
        if (not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                    and math.isfinite(v) for v in (value, measured))
                or not 0.0 <= value <= 1.0):
            return
        with self.lock:
            if self._last_measurement is not None and measured <= self._last_measurement:
                return
            self._last_measurement = measured
            if self.motion is not None:
                start, goal, rate, began = self.motion
                # Confirm with a post-arrival measurement only. Sparse readings
                # during motion must not restart or jump the simulated ramp.
                if (measured <= began + abs(goal-start)/rate
                        or (not self._holding and abs(value-goal) > 0.02)):
                    return
            now = self.clock()
            current = self._render(now)
            self.motion = None
            self._display_motion = None
            self._holding = False
            if current is None:
                self._output = float(value)
                return
            # Repeated readings of the same destination must not postpone
            # an in-progress correction indefinitely.
            if self._correction is not None and value == self._correction[1]:
                return
            delta = abs(value-current[0])
            if delta <= 1e-12:
                self._output = float(value)
                self._correction = None
            else:
                # Smoothstep's peak slope is 1.5: bound the display correction
                # to CORRECTION_RATE, even for a newly retargeted measurement.
                duration = max(self.CORRECTION_MIN_TIME, 1.5*delta/self.CORRECTION_RATE)
                self._correction = (current[0], float(value), now, duration)

    def hold(self):
        with self.lock:
            now = self.clock()
            current = self._render(now)
            if current is not None:
                self.motion = (current[0], current[0], 1.0, now)
                self._display_motion = self.motion
                self._correction = None
                self._holding = True


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
