#!/usr/bin/env python3
"""Measure Pro450 hold speed and acceleration. Does not move unless --execute.

Run on the robot computer after sourcing the workspace. The arm must be clear
of obstacles. Each sample is a short out-and-back move. Paste the printed
HOLD_TRACKED_ACCELERATION value back into teleop_keyboard_gazebo.py.
"""
import argparse
import math
import time


def _samples(read, duration, period):
    points = []
    end = time.monotonic() + duration
    while time.monotonic() < end:
        stamp = time.monotonic()
        points.append((stamp, read()))
        remaining = period - (time.monotonic() - stamp)
        if remaining > 0:
            time.sleep(remaining)
    return points


def _profile(samples):
    """Return peak speed and a robust acceleration from position samples."""
    speeds = []
    for (t0, p0), (t1, p1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        speeds.append((t1, (p1 - p0) / dt))
    if len(speeds) < 4:
        raise RuntimeError("not enough samples")
    peak = max(abs(speed) for _stamp, speed in speeds)
    accels = []
    for (t0, v0), (t1, v1) in zip(speeds, speeds[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        accels.append(abs(v1 - v0) / dt)
    accels.sort()
    # Ignore the noisiest quarter of the finite differences.
    kept = accels[:max(1, int(len(accels) * 0.75))]
    return peak, sum(kept) / len(kept)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true",
                        help="Actually move the robot. Without this, the script exits.")
    parser.add_argument("--ip", default="192.168.0.232")
    parser.add_argument("--port", type=int, default=4500)
    parser.add_argument("--joint", type=int, default=4, help="Joint id 1-6")
    parser.add_argument("--delta-deg", type=float, default=8.0)
    parser.add_argument("--gripper", action="store_true",
                        help="Also measure one gripper open/close of 20 units")
    parser.add_argument("--period", type=float, default=0.02)
    args = parser.parse_args()
    if not args.execute:
        raise SystemExit("Refusing to move. Re-run with --execute when the arm is clear.")
    if not 1 <= args.joint <= 6:
        raise SystemExit("joint must be 1..6")

    from pymycobot import Pro450Client
    robot = Pro450Client(args.ip, args.port)
    if robot.is_power_on() != 1:
        raise SystemExit("Pro450 is not powered")
    if robot.get_fresh_mode() != 0:
        robot.set_fresh_mode(0)

    speeds = (4, 8, 12, 16, 20)
    print("speed_sdk peak_rad_s accel_rad_s2")
    arm_accels = []
    for speed in speeds:
        angles = robot.get_angles()
        start = float(angles[args.joint - 1])
        goal = start + args.delta_deg
        robot.send_angle(args.joint, goal, speed, _async=True)
        samples = _samples(
            lambda: math.radians(float(robot.get_angles()[args.joint - 1])),
            duration=max(1.5, abs(args.delta_deg) / 5.0),
            period=args.period,
        )
        robot.send_angle(args.joint, start, speed, _async=True)
        time.sleep(max(1.5, abs(args.delta_deg) / 5.0))
        peak, accel = _profile(samples)
        arm_accels.append(accel)
        print(f"{speed} {peak:.4f} {accel:.4f}")
    suggested = sorted(arm_accels)[len(arm_accels) // 2]
    print(f"HOLD_TRACKED_ACCELERATION = {suggested:.2f}")

    if args.gripper:
        opening = robot.get_pro_gripper_angle(gripper_id=14)
        if not isinstance(opening, (int, float)) or opening < 0:
            raise SystemExit(f"gripper read failed: {opening!r}")
        goal = min(100, int(opening) + 20)
        robot.set_pro_gripper_speed(8)
        robot.set_pro_gripper_angle(goal)
        samples = _samples(
            lambda: float(robot.get_pro_gripper_angle(gripper_id=14)) / 100.0,
            duration=2.0,
            period=args.period,
        )
        robot.set_pro_gripper_angle(int(opening))
        peak, accel = _profile(samples)
        print(f"gripper peak_rad_s {peak:.4f} accel_rad_s2 {accel:.4f}")


if __name__ == "__main__":
    main()
