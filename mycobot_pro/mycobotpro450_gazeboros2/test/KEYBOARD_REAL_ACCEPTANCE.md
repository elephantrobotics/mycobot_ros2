# Pro450 keyboard acceptance

Scope: keyboard only. Random-target/slider real execution is not changed.
No automatic power-on, reset, servo release or gripper initialization.
No hardware tests are performed by the automated scripts.

## Simulation / hardware-call substitute

Use the same ROS_DOMAIN_ID and RMW_IMPLEMENTATION in every terminal.
Stop other keyboard nodes and slider_control_gazebo before testing.

```bash
cd ~/xzh
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=55 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch mycobotpro450_gazeboros2 slider.launch.py environment:=simulation
```

In another terminal with the same environment:

```bash
python3 -m unittest discover -s src/mycobot_ros2/mycobot_pro/mycobotpro450_gazeboros2/test -p 'test_pro450*.py'
python3 src/mycobot_ros2/mycobot_pro/mycobotpro450_gazeboros2/test/verify_pro450_keyboard_sim.py
python3 src/mycobot_ros2/mycobot_pro/mycobotpro450_gazeboros2/test/verify_pro450_gripper_hold_sim.py
QT_QPA_PLATFORM=offscreen python3 src/mycobot_ros2/mycobot_pro/mycobotpro450_gazeboros2/test/verify_pro450_keyboard_ui.py
python3 src/mycobot_ros2/mycobot_pro/mycobotpro450_gazeboros2/test/verify_pro450_keyboard_real_fake.py
```

The last test replaces pymycobot before controller import. It cannot construct
a physical robot client. Its fake actuator is not evidence about real dynamics.

## Real read-only acceptance

The physical robot must already be powered and stationary, with valid gripper
feedback. Provide physical clearance and an accessible hardware E-stop.
Do not run other hardware clients, slider_control_gazebo, or RViz Execute.

Terminal A (same setup as above):

```bash
ros2 run mycobotpro450_gazeboros2 teleop_keyboard_gazebo.py --ros-args -p mode:=real
```

Terminal B:

```bash
ros2 launch mycobotpro450_gazeboros2 slider.launch.py environment:=real
```

Expected: initial pose matches; startup, window open, M and motion keys cannot
move hardware because real_hold_enabled defaults to false.

## Experimental low-speed arm acceptance (operator only)

After read-only acceptance, restart only the keyboard process with:

```bash
ros2 run mycobotpro450_gazeboros2 teleop_keyboard_gazebo.py --ros-args -p mode:=real -p real_hold_enabled:=true
```

Keep the environment launch running. Wait for stationary, fresh synchronized
feedback. In the keyboard window, choose gear 1, then M. Neither operation
should move hardware. Briefly hold a safe single joint key and release. Verify
direction, decelerating stop, no other-joint movement, no vibration. Repeat in
the opposite direction, then check N/Space locking, focus loss and exit.
Do not increase speed before this passes. Real gripper hold remains disabled.

Gears 1-5 use SDK arm settings 4/8/12/16/20, corresponding nominally to
6/12/18/24/30 deg/s under the user-provided 100 -> 150 deg/s calibration.
Actual controller ramps, replace-in-motion behavior and stop distance must be
measured. SDK speed is integer-quantized; the startup/ramp below SDK minimum
speed is suppressed, not rounded up. URDF arm limit remains 1 rad/s.

## Remaining limits

- Runtime reads are serialized by one owner. Arm is read each cycle; gripper
  nominally at 2 Hz (0.5 s after a successful read). Two 0.2 s-spaced retries
  follow a failure, then 1 s cooldown. No retry issues any SDK write.
- Gripper cache expires after 0.8 s and arm feedback after 0.5 s. Cache reuse
  retains its original timestamp. A failed read invalidates the cache for
  control immediately; old values are never passed off as new measurements.
- Feedback state is committed before snapshot/mirror publication. Snapshot
  header time represents the oldest constituent read, not publication time.
- Recovery requires three new physical gripper reads with stationary, stable
  pose; repeated cache use cannot count. Read recovery is not motion recovery:
  the latch stays until an explicit M action and synchronization checks pass.
- Debug logs show raw SDK gripper return, query duration and failure counters.
  They do not include the underlying Modbus packet: a -1 alone cannot identify
  timeout vs SDK packet verification failure. Enable with --log-level debug.
- Startup still requires five valid fresh arm/gripper samples, now spaced by
  0.5 s. Invalid startup feedback does not publish a snapshot or permit motion.

- This uses bounded single-axis send_angle targets, not a documented streaming
  servo. Firmware may replan replacement targets; no hardware smoothness claim.
- Real release uses stop(deceleration=1); firmware deceleration is not confirmed
  equal to the simulation's 1.4 rad/s^2 profile.
- Communication reserve assumes up to 0.5 s travel plus mathematical braking.
  That assumption needs measurement. Ordinary TCP is not a safety-rated channel.
- Target span is capped at 0.08 rad. Latest command replaces pending commands;
  stale UI commands lock and request stop. An RPC blocking can delay STOP.
- Gripper speed setting 30 has no calibrated rad/s mapping or independent stop
  API. Experimental enabling exists, but is not part of this acceptance. A
  measured-position replacement target is not a guaranteed gripper E-stop.
- All STOPs retain servo torque. Use the hardware E-stop for physical danger.
- Gazebo mirrors measured poses through its existing controllers; it is not
  a certified exact physical mirror and can lag hardware. Hardware feedback,
  not Gazebo pose, determines collision checks and standstill.
