"""Pure motion-profile math for Pro450 press/hold/release control.

This module deliberately has no ROS or hardware dependency, so the same
profile can be tested before a future real-robot transport is considered.
"""

import math
from collections import deque


class PositionVelocityEstimator:
    """Estimate joint speed from position history, not simulator velocity fields."""

    def __init__(self, window=0.12):
        self.window = window
        self.samples = deque()

    def update(self, stamp, positions):
        values = tuple(float(value) for value in positions)
        if not math.isfinite(stamp) or not all(math.isfinite(v) for v in values):
            self.samples.clear()
            return None
        if self.samples and stamp <= self.samples[-1][0]:
            self.samples.clear()
            return None
        self.samples.append((stamp, values))
        cutoff = stamp - self.window
        while len(self.samples) > 2 and self.samples[1][0] <= cutoff:
            self.samples.popleft()
        elapsed = stamp - self.samples[0][0]
        if elapsed < self.window * 0.65:
            return None
        previous = self.samples[0][1]
        return [(value - old) / elapsed for value, old in zip(values, previous)]


def approach(value, target, maximum_change):
    return value + max(-maximum_change, min(maximum_change, target - value))


def hold_setpoint(position, actual_velocity, command_velocity, direction,
                  pressed, validated_limit, dt, speed_limit, acceleration,
                  horizon=0.25, collision_margin=0.01):
    """Return (position, velocity, duration) for a bounded JTC waypoint.

    ``validated_limit`` is the last collision-checked position in ``direction``.
    Reserve the maximum-speed braking distance before that point. All returned
    waypoint positions stay inside this conservative boundary.
    """
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    if not all(math.isfinite(x) for x in (
            position, actual_velocity, command_velocity, validated_limit,
            dt, speed_limit, acceleration, horizon, collision_margin)):
        raise ValueError("non-finite hold profile input")
    if dt <= 0 or speed_limit <= 0 or acceleration <= 0 or horizon <= 0:
        raise ValueError("hold profile rates must be positive")

    dt = min(dt, 0.10)
    # A high selected gear must not make a short, valid corridor unusable.
    # Reduce the allowed speed near its end, reserving at most half of the
    # available distance for braking so motion can still start from rest.
    available = max(0.0, direction * (validated_limit - position) - collision_margin)
    speed_limit = min(speed_limit, math.sqrt(acceleration * available))
    braking_reserve = speed_limit * speed_limit / (2.0 * acceleration)
    safe_limit = validated_limit - direction * (braking_reserve + collision_margin)
    if direction * (safe_limit - position) < 0:
        safe_limit = position
    remaining = direction * (safe_limit - position)
    approaching_speed = max(0.0, direction * actual_velocity,
                            direction * command_velocity)
    distance_to_brake = (approaching_speed * horizon +
                         approaching_speed * approaching_speed / (2.0 * acceleration))
    desired = (direction * speed_limit
               if pressed and remaining > distance_to_brake else 0.0)
    velocity = approach(command_velocity, desired, acceleration * dt)

    # Avoid requesting motion deeper into an exhausted clearance corridor.
    if remaining <= 0 and direction * velocity > 0:
        velocity = 0.0

    # When releasing, use enough time to lower the measured velocity without
    # asking the trajectory controller for a sharper deceleration than requested.
    duration = max(horizon, abs(actual_velocity - velocity) / acceleration)
    endpoint = position + 0.5 * (actual_velocity + velocity) * duration
    if direction * (endpoint - safe_limit) > 0:
        endpoint = safe_limit
        velocity = 0.0
    return endpoint, velocity, duration
