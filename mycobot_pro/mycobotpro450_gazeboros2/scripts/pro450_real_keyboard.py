"""Bounded, single-axis hardware transport. No ROS and no startup writes.

Position-command replacement and firmware braking require hardware acceptance;
this is not a certified streaming servo or a hardware emergency stop.
"""
import math
import threading
import time

from pro450_hold_profile import PositionVelocityEstimator

SDK_RAD_PER_SPEED = math.pi / 120.0  # User calibration: 100 -> 150 deg/s.


def valid_gripper_position(raw):
    """Check SDK read status before mapping 0..100 to the URDF coordinate."""
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise RuntimeError(f"Pro450 gripper read failed: invalid SDK response {raw!r}")
    if raw < 0:
        raise RuntimeError(
            f"Pro450 gripper read failed: SDK returned {raw!r}; "
            "Gazebo startup and real motion are blocked")
    if not math.isfinite(raw) or raw > 100:
        raise RuntimeError(f"Pro450 gripper read failed: invalid SDK position {raw!r}")
    return float(raw) / 100.0


def sdk_speed_for_rad(velocity, urdf_limit=1.0):
    if not math.isfinite(velocity) or not math.isfinite(urdf_limit) or urdf_limit <= 0:
        raise ValueError("invalid speed")
    # Round down, never exceed either the requested or the URDF speed.
    return max(0, min(100, int((min(abs(velocity), urdf_limit) + 1e-12) /
                             SDK_RAD_PER_SPEED)))


class RealPoseReader:
    """Single-owner reads with independent timestamps; never writes to SDK.

    Arm: each owner cycle. Gripper: 2 Hz. Failure: at most two spaced retries,
    then a 1 s cooldown. A failed/stale gripper blocks the combined pose even
    if a previous cached value exists. Cache timestamps are never refreshed
    without a successful physical query.
    """
    def __init__(self, sdk, limits, clock=time.monotonic, report=None):
        self.sdk, self.limits, self.clock = sdk, limits, clock
        self.report = report or (lambda _: None)
        self.arm_time = self.gripper_time = None
        self.arm = None
        self.gripper = None
        self.gripper_valid = False
        self.next_gripper = 0.0
        self.gripper_period = 0.5
        self.gripper_max_age = 0.8
        self.retry_count = 0
        self.read_count = self.failure_count = self.success_count = 0
        self.last_raw = None
        self.last_error = ''

    def fresh(self):
        now = self.clock()
        return (self.arm_time is not None and self.gripper_time is not None
                and self.gripper_valid and now - self.arm_time <= 0.5
                and now - self.gripper_time <= self.gripper_max_age)

    def force_gripper_read(self):
        self.next_gripper = self.clock()

    def __call__(self):
        raw = self.sdk.get_angles()
        if not isinstance(raw, (list, tuple)) or len(raw) != 6:
            raise RuntimeError(f"Invalid Pro450 arm response: {raw!r}")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in raw):
            raise RuntimeError(f"Invalid Pro450 arm response: {raw!r}")
        arm = [math.radians(v) for v in raw]
        if not all(math.isfinite(v) and lo <= v <= hi
                   for v, (lo, hi) in zip(arm, self.limits[:6])):
            raise RuntimeError(f"Pro450 arm feedback violates joint limits: {raw!r}")
        self.arm, self.arm_time = arm, self.clock()
        if self.clock() >= self.next_gripper:
            started = self.clock()
            self.read_count += 1
            self.last_raw = None
            try:
                self.last_raw = self.sdk.get_pro_gripper_angle(gripper_id=14)
                if self.last_raw == -1 and self.gripper is not None:
                    # SDK -1 must not lock motion: keep the last valid opening.
                    self.gripper_time = self.clock()
                    self.gripper_valid = True
                    self.retry_count = 0
                    self.next_gripper = self.clock() + self.gripper_period
                    self.report(
                        f"gripper read returned -1; reusing last valid value "
                        f"{self.gripper:.3f}, read_elapsed={self.clock() - started:.3f}s")
                    return self._combined()
                value = valid_gripper_position(self.last_raw)
            except Exception as exc:
                self.gripper_valid = False
                self.failure_count += 1
                self.retry_count += 1
                attempt = self.retry_count
                delay = 0.2 if attempt <= 2 else 1.0
                self.next_gripper = self.clock() + delay
                if attempt >= 3:
                    self.retry_count = 0
                self.last_error = (
                    f"{exc}; raw={self.last_raw!r}, read_elapsed="
                    f"{self.clock() - started:.3f}s, failed_attempt={attempt}/3, "
                    f"total_failures={self.failure_count}, next_read_in={delay:.1f}s")
                self.report(self.last_error)
                raise RuntimeError(self.last_error) from exc
            self.gripper, self.gripper_time = value, self.clock()
            self.gripper_valid = True
            self.retry_count = 0
            self.success_count += 1
            self.next_gripper = self.clock() + self.gripper_period
            self.last_error = ''
            self.report(
                f"gripper read valid: raw={self.last_raw!r}, "
                f"read_elapsed={self.clock() - started:.3f}s, reads={self.read_count}, "
                f"total_failures={self.failure_count}")
        return self._combined()

    def seed_gripper(self, value):
        self.gripper, self.gripper_time = float(value), self.clock()
        self.gripper_valid = True
        self.next_gripper = self.clock() + self.gripper_period

    def _combined(self):
        if not self.fresh():
            raise RuntimeError(self.last_error or "Pro450 arm/gripper feedback stale")
        return list(self.arm) + [self.gripper]


class RealKeyboardTransport:
    """One SDK owner; latest command only, finite goals, read-only by default."""

    def __init__(self, sdk, read_pose, publish_pose, limits, clock=time.monotonic,
                 report_error=None):
        self.sdk, self.read_pose, self.publish_pose = sdk, read_pose, publish_pose
        self.limits, self.clock = limits, clock
        self.report_error = report_error or (lambda _: None)
        self.lock = threading.Lock()
        self.pose = None
        self.velocity = None
        self.received = 0.0
        self.error = None
        self.armed = False
        self.command = None
        self.stop_pending = None
        self.motion_sent = False
        self.motion_axis = None
        self.generation = 0
        self.closed = threading.Event()
        self.estimator = PositionVelocityEstimator(window=0.25)
        self.last_send = 0.0
        self.last_signature = None
        self.command_time = 0.0
        self.moving = True
        self.last_health = 0.0
        self.recovery_ready = False
        self._recovery_samples = 0
        self._recovery_origin = None
        self._last_recovery_read = -1

    def feedback(self):
        with self.lock:
            if ((self.error and not self.recovery_ready) or self.pose is None or
                    self.clock() - self.received > 0.5 or
                    (hasattr(self.read_pose, 'fresh') and not self.read_pose.fresh())):
                return None
            return list(self.pose), list(self.velocity)

    def arm(self):
        with self.lock:
            if ((self.error and not self.recovery_ready) or self.pose is None or
                    self.clock() - self.received > 0.5 or
                    (hasattr(self.read_pose, 'fresh') and not self.read_pose.fresh())
                    or any(abs(v) > 0.01 for v in self.velocity)
                    or self.moving or self.motion_sent or self.stop_pending is not None):
                return False
            # Only this explicit operator action clears the motion fault latch.
            self.error = None
            self.recovery_ready = False
            self.armed = True
            return True

    def submit(self, axis, endpoint, velocity, origin, validated_limit):
        with self.lock:
            if not self.armed or self.error or self.stop_pending is not None:
                return False
            self.command = (self.generation, axis, endpoint, abs(velocity),
                            list(origin), validated_limit)
            self.command_time = self.clock()
            return True

    def stop(self, emergency=False, lock=False):
        with self.lock:
            self.generation += 1
            self.command = None
            self.stop_pending = bool(emergency) or bool(self.stop_pending)
            if lock:
                self.armed = False

    def cycle(self):
        """Only the owner thread calls this; reads and writes cannot overlap."""
        with self.lock:
            if (self.motion_sent and self.command is not None and
                    self.clock() - self.command_time > 0.5):
                self.armed = False
                self.command = None
                self.stop_pending = True
            stop = self.stop_pending
            self.stop_pending = None
        if stop is not None and self.motion_sent:
            if self.motion_axis == 6:
                # No independent gripper STOP exists in the inspected SDK.
                # Replace the finite target by measured opening; not an E-stop.
                if hasattr(self.read_pose, 'force_gripper_read'):
                    self.read_pose.force_gripper_read()
                pose = self.read_pose()
                self.sdk.set_pro_gripper_angle(round(pose[6] * 100))
            else:
                self.sdk.stop(deceleration=0 if stop else 1, _async=True)
            self.motion_sent = False
            self.last_signature = None
        pose = self.read_pose()
        sample_time = getattr(self.read_pose, 'arm_time', None) or self.clock()
        now = self.clock()
        if now - self.last_health >= 1.0:
            if self.sdk.is_power_on() != 1 or self.sdk.get_error_information() not in (0, None):
                raise RuntimeError("hardware power/error check failed")
            self.last_health = now
        moving = self.sdk.is_moving()
        if moving not in (0, 1):
            raise RuntimeError("invalid hardware moving status")
        if len(pose) != 7 or not all(math.isfinite(p) and lo <= p <= hi
                                    for p, (lo, hi) in zip(pose, self.limits)):
            raise RuntimeError("invalid hardware feedback")
        velocities = self.estimator.update(sample_time, pose)
        now = self.clock()
        with self.lock:
            # Do not expose an unmeasured zero velocity during estimator warmup.
            if velocities is not None:
                self.pose, self.velocity, self.received = pose, velocities, sample_time
                self.moving = bool(moving)
                if self.error:
                    read_id = getattr(self.read_pose, 'success_count', sample_time)
                    if read_id != self._last_recovery_read:
                        self._last_recovery_read = read_id
                        stable = (not moving and not self.motion_sent and
                                  all(abs(v) <= 0.01 for v in velocities))
                        if self._recovery_origin is not None:
                            stable = stable and all(
                                abs(a - b) <= (math.radians(0.2) if j < 6 else 0.005)
                                for j, (a, b) in enumerate(zip(pose, self._recovery_origin)))
                        if stable:
                            if self._recovery_origin is None:
                                self._recovery_origin = list(pose)
                            self._recovery_samples += 1
                        else:
                            self._recovery_samples = 0
                            self._recovery_origin = None
                        self.recovery_ready = self._recovery_samples >= 3
            # State is committed before callbacks use it for launch gating.
            command = self.command
        if velocities is not None:
            self.publish_pose(pose)
        if command is None or velocities is None or now - self.last_send < 0.20:
            return
        generation, axis, endpoint, velocity, origin, boundary = command
        if any(abs(v) > 1.05 for v in velocities):
            raise RuntimeError("measured speed exceeds URDF limit")
        if any(abs(pose[j] - origin[j]) > 0.005 for j in range(7) if j != axis):
            raise RuntimeError("uncommanded joint changed; collision corridor invalid")
        direction = 1 if boundary > origin[axis] else -1
        # Additional communication/feedback travel reserve beyond the shared
        # mathematical profile. Firmware stop distance still needs calibration.
        reserve = velocity * 0.5 + velocity * velocity / (2 * 1.4) + 0.01
        safe_boundary = boundary - direction * reserve
        if direction * (safe_boundary - endpoint) < 0:
            endpoint = safe_boundary
        if (direction * (endpoint - pose[axis]) <= 0 or
                direction * (boundary - endpoint) < 0.01):
            return
        lo, hi = self.limits[axis]
        if not lo <= endpoint <= hi:
            raise RuntimeError("target outside URDF")
        # Bound every command even if feedback/GUI subsequently stops arriving.
        endpoint = pose[axis] + max(-0.08, min(0.08, endpoint - pose[axis]))
        speed = sdk_speed_for_rad(velocity)
        if axis != 6 and speed == 0:
            return
        signature = (axis, round(endpoint, 4), speed)
        with self.lock:
            if (generation != self.generation or not self.armed or self.error
                    or self.stop_pending is not None):
                return
            if signature == self.last_signature:
                return
            # Mark before calling: even an ambiguous timeout requires a STOP.
            self.motion_sent, self.motion_axis = True, axis
        if axis == 6:
            if self.sdk.set_pro_gripper_speed(30) != 1:
                raise RuntimeError("gripper speed setting rejected")
            with self.lock:
                if generation != self.generation or not self.armed:
                    return
            result = self.sdk.set_pro_gripper_angle(round(endpoint * 100))
        else:
            result = self.sdk.send_angle(axis + 1, math.degrees(endpoint), speed,
                                         _async=True)
        if isinstance(result, str) or result == 0:
            raise RuntimeError(f"SDK rejected motion: {result}")
        self.last_send, self.last_signature = now, signature

    def _latch_fault(self, exc):
        with self.lock:
            changed = self.error != str(exc)
            self.error, self.armed, self.command = str(exc), False, None
            self.generation += 1
            self.recovery_ready = False
            self._recovery_samples = 0
            self._recovery_origin = None
            # A pre-fault cached sample cannot count as successful recovery.
            self._last_recovery_read = getattr(self.read_pose, 'success_count', -1)
        if changed:
            self.report_error(f"Real feedback/control failed; motion locked: {exc}")

    def run(self):
        while not self.closed.is_set():
            try:
                self.cycle()
            except Exception as exc:
                self._latch_fault(exc)
                # A failed read/send may have left a finite goal active.
                if self.motion_sent:
                    try:
                        if self.motion_axis != 6:
                            self.sdk.stop(deceleration=0, _async=True)
                        else:
                            if hasattr(self.read_pose, 'force_gripper_read'):
                                self.read_pose.force_gripper_read()
                            self.sdk.set_pro_gripper_angle(round(self.read_pose()[6] * 100))
                        self.motion_sent = False
                    except Exception:
                        pass  # Keep latch; operator must use hardware E-stop.
            self.closed.wait(0.10)

    def close(self):
        self.stop(emergency=True, lock=True)
        # Owner must service STOP before being asked to exit.
        deadline = self.clock() + 2.0
        while (self.stop_pending is not None or self.motion_sent) and self.clock() < deadline:
            time.sleep(0.02)
        self.closed.set()
