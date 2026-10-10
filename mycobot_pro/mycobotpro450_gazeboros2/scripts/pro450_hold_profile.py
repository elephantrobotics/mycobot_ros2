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


def absolute_samples(start, goal, step):
    """Absolute grid points from just beyond ``start`` through ``goal``."""
    if step <= 0:
        raise ValueError("grid step must be positive")
    direction = 1 if goal >= start else -1
    if direction * (goal - start) <= 1e-12:
        return
    if direction > 0:
        point = (math.floor(start / step) + 1) * step
        while point < goal - 1e-12:
            yield point
            point += step
    else:
        point = (math.ceil(start / step) - 1) * step
        while point > goal + 1e-12:
            yield point
            point -= step
    yield goal


def collision_stop_angle(start, goal, margin, grid, fine, blocked):
    """Stop angle for one joint, independent of when the scan is started.

    ``blocked(angle)`` is true when that angle is in collision. Samples lie on
    a fixed absolute grid. The first blocked sample brackets the boundary, and
    bisection refines it. A clear path returns ``goal``. No travel room returns
    ``start``.
    """
    if grid <= 0 or fine <= 0 or margin < 0:
        raise ValueError("invalid collision scan settings")
    if not all(math.isfinite(value) for value in (start, goal, margin, grid, fine)):
        raise ValueError("non-finite collision scan input")
    direction = 1 if goal >= start else -1
    if direction * (goal - start) <= 1e-9:
        return start
    if blocked(start):
        return start
    # Ten grid steps per probe keeps a full-range scan inside a short press.
    # The bracket is still absolute, so the refined boundary does not depend
    # on the exact start angle.
    last_free = start
    hit = None
    for sample in absolute_samples(start, goal, grid * 10):
        if blocked(sample):
            hit = sample
            break
        last_free = sample
    if hit is None:
        return goal
    while abs(hit - last_free) > fine:
        mid = (hit + last_free) / 2.0
        if blocked(mid):
            hit = mid
        else:
            last_free = mid
    target = hit - direction * margin
    if direction * (target - start) < 0:
        target = start
    if direction * (target - goal) > 0:
        target = goal
    return target


def trapezoid_samples(start, goal, speed, acceleration, period=0.05):
    """Sample one trapezoid from ``start`` to ``goal``.

    Returns ``(times, positions)`` in seconds from the start of the move.
    ``speed`` and ``acceleration`` are positive magnitudes.
    """
    if speed <= 0 or acceleration <= 0 or period <= 0:
        raise ValueError("trapezoid rates must be positive")
    if not all(math.isfinite(value) for value in (start, goal, speed, acceleration, period)):
        raise ValueError("non-finite trapezoid input")
    distance = abs(goal - start)
    direction = 1 if goal >= start else -1
    if distance <= 1e-9:
        return [0.0], [start]
    accel_time = speed / acceleration
    accel_distance = 0.5 * acceleration * accel_time * accel_time
    if 2.0 * accel_distance >= distance:
        accel_time = math.sqrt(distance / acceleration)
        cruise_time = 0.0
        peak = acceleration * accel_time
    else:
        cruise_time = (distance - 2.0 * accel_distance) / speed
        peak = speed

    def traveled(elapsed):
        if elapsed <= accel_time:
            return 0.5 * acceleration * elapsed * elapsed
        if elapsed <= accel_time + cruise_time:
            return (0.5 * acceleration * accel_time * accel_time +
                    peak * (elapsed - accel_time))
        decel = elapsed - accel_time - cruise_time
        return (0.5 * acceleration * accel_time * accel_time +
                peak * cruise_time +
                peak * decel - 0.5 * acceleration * decel * decel)

    total = 2.0 * accel_time + cruise_time
    times = []
    positions = []
    elapsed = 0.0
    while elapsed < total - 1e-9:
        times.append(elapsed)
        positions.append(start + direction * traveled(elapsed))
        elapsed += period
    times.append(total)
    positions.append(goal)
    return times, positions


def smoothstep(fraction):
    """Zero-velocity cubic used by a one-point joint trajectory."""
    fraction = min(1.0, max(0.0, fraction))
    return fraction * fraction * (3.0 - 2.0 * fraction)


def inverse_smoothstep(progress):
    """Time fraction at which ``smoothstep`` reaches ``progress``."""
    progress = min(1.0, max(0.0, progress))
    if progress <= 0.0 or progress >= 1.0:
        return progress
    low, high = 0.0, 1.0
    for _ in range(50):
        mid = 0.5 * (low + high)
        if smoothstep(mid) < progress:
            low = mid
        else:
            high = mid
    return 0.5 * (low + high)


def shared_progress_samples(start, goal, duration, step):
    """Samples of one straight joint-space line that share a single progress.

    Positions stay on the collision-checked line. Their timestamps follow the
    same smoothstep the simulation trajectory controller uses, so every joint,
    including the gripper, arrives together.
    """
    if duration <= 0 or step <= 0:
        raise ValueError("duration and step must be positive")
    if len(start) != len(goal) or not start:
        raise ValueError("sample endpoints differ in length")
    if not all(math.isfinite(value) for value in (*start, *goal, duration, step)):
        raise ValueError("non-finite shared-progress input")
    deltas = [goal_value - start_value for start_value, goal_value in zip(start, goal)]
    max_delta = max(abs(value) for value in deltas)
    if max_delta <= 1e-9:
        return [(duration, list(goal))]
    count = max(1, math.ceil(max_delta / step - 1e-12))
    samples = []
    for index in range(1, count + 1):
        progress = index / count
        position = [
            start_value + delta * progress
            for start_value, delta in zip(start, deltas)
        ]
        samples.append((duration * inverse_smoothstep(progress), position))
    samples[-1] = (duration, list(goal))
    return samples
