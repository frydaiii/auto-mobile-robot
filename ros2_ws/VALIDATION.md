# Stage 1 execution evidence

## Hardware simulation extension — native Ubuntu, 2026-11-17

ROS 2 Lyrical / Gazebo Jetty, isolated `ROS_DOMAIN_ID=88` and
`GZ_PARTITION=amr_hw_validation`, headless GUI/RViz disabled. These are software
simulation checks, not hardware measurements. This section supersedes the older
360-ray scan and test-count statements below.

- All three packages built, including the new C++ `amr_bumper_state` plugin.
- 25 pytest tests passed; measured Python statement/branch coverage was 95.35%,
  above the configured 90% floor. The C++ plugin is outside Python coverage;
  its contact/clear behavior was exercised in the headless run.
- Fresh `/scan` (383 rays, minimum 0.15 m), `/map`, `/imu/data_raw`, both range
  topics, encoder counts and all three bumper states were received.
- Combined forward/turn request of 0.15 m/s and 0.6 rad/s changed measured wheel
  encoder counts; maximum observed wheel speed was 59.999999995 RPM.
- Raw IMU orientation was marked unavailable; stationary Z acceleration was
  approximately 9.79 m/s² in a sampled noisy reading.
- Added a static box at (-1.35, -2), with the robot reset to (-2, -2).
  Front ToF readings were approximately 0.45436 m on both sides before approach.
  A straight approach made contact with the physical bumper geometry. Despite
  continuing forward commands, `/cmd_vel_safe` became zero and measured forward
  speed fell below 0.02 m/s.
- Conversion tests caught and fixed sdformat's collision-name rewriting.
  Native Contact sends only nonempty events; the custom plugin supplies 50 Hz
  snapshots, including clear states, and latches impacts between snapshots.
- Unit and real ROS tests cover command loss and bumper feedback loss with
  simulation time paused, contact release requiring a new command, named-wheel
  encoder quantization, malformed feedback, sim-time reversal and Range
  infinity conventions. They do not prove realistic optical or motor dynamics.

Shutdown still reproduces the upstream bridge SIGINT crash (signal 11) and
Gazebo's delayed shutdown requiring SIGTERM. Diagnostic processes were stopped;
no real hardware was commanded. Rendering extensions/root-inertia warnings also
remain. SLAM emits a minimum-range comparison warning due to float32 rounding
of 0.15 m in LaserScan versus the double parameter (printed as 0.2 versus 0.1
with one decimal); it clamps to the received sensor limit.

Tested 2026-11-17 in WSL Ubuntu 26.04.1 LTS x86-64 with ROS 2 Lyrical
and Gazebo Sim 10 (Jetty). This is simulation validation only.

## Passed

- RViz motion investigation: the original total centre of mass was 0.196 mm
  ahead of the wheel axle, outside the rear-caster support triangle. Gazebo
  measured a forward pitch of 18.5 degrees, tilting the scan toward the floor.
  With `chassis_com_x=-0.040`, the robot stays level (pitch approximately
  -0.000087 degrees). In a repeated straight drive, odometry advanced 1.27485 m
  and the `map -> base_link` transform used by RViz advanced 1.27485 m, with
  negligible lateral drift. Previously map-frame X advanced only 0.446 m for
  1.274 m of odometry and SLAM introduced spurious lateral/heading correction.
  A default-model mass-centre regression test failed before the fix and passed
  afterward. Suite now contains 13 passing tests; Python coverage remains 100%.
  This corrects the physical model and does not establish full-loop accuracy.

- Expanded automated suite: 12 tests passed on the same Ubuntu/ROS target.
  Coverage.py measured 100% of 73 executable statements and all 10 branches
  across `command_gate.py`, `command_gate_node.py`, and `mapping.launch.py`.
  `.coveragerc` enforces a 90% minimum. Packaging metadata, assets and upstream
  dependencies are not counted. Tests exercise real ROS messages and the steady
  watchdog with simulation time paused; only the entrypoint's indefinite spin
  is replaced to trigger return, keyboard interruption and external shutdown.

- `colcon build --symlink-install`: all three packages built.
- `colcon test --python-testing pytest` and `colcon test-result --verbose`:
  six tests, zero errors/failures/skips. Explicit pytest selection is needed
  because this Ubuntu setuptools no longer accepts `tests_require`.
- `mapping.launch.py gui:=false rviz:=false`: robot spawned, SLAM activated,
  map and all configured bridge interfaces published.
- Scan contained 360 ranges in `lidar_link`; wheel joint feedback named both
  drive joints; TF lookup from `map` to `lidar_link` succeeded.
- A 0.1 m/s forward command for four seconds produced +0.398 m odometry X.
- A +0.3 rad/s rotation increased quaternion Z by 0.432 over three seconds.
- After command loss, gated velocity became zero and measured linear velocity
  fell below 0.01 m/s after one second. Unit tests separately cover the exact
  0.5-second expiry boundary, limits, invalid input, startup and recovery.
- Nav2 saved YAML + PGM: 124 × 122 cells at 0.05 m resolution, origin
  `[-1.206, -1.024, 0]`. A separate map server reloaded it; every loaded cell
  matched the saved image under its YAML thresholds, including 322 occupied
  cells. This checks saved-map preservation, not loop-closure accuracy.
- A second headless launch spawned successfully and registered the LiDAR.

## Unresolved or pending

- Clean shutdown failed in the installed stack: `ros_gz_bridge` exited with
  signal 11 during SIGINT on both launches. One Gazebo shutdown required
  SIGTERM escalation. A duplicate shutdown exception from the upstream launch
  wrapper was also observed; the application no longer requests that wrapper's
  extra on-exit shutdown. Recheck clean shutdown on native Ubuntu and after
  ROS/Gazebo updates. No simulation processes were intentionally left running.
- Full closed-loop driving around obstacles and overlapping wall alignment
  within 0.15 m remain pending; no map accuracy acceptance is claimed.
- Interactive Gazebo GUI, RViz visual appearance and keyboard operation remain
  pending. Commands were supplied programmatically for the headless tests.
- Long-duration scan rate/freshness, independent simulator ground-truth
  comparison, and exhaustive duplicate-TF ownership inspection remain pending.

The WSL renderer emitted EGL warnings and successfully fell back to software
rendering. URDF parsing reports KDL's root-inertia warning; Gazebo retains the
inertial for physics. SDFormat warns that `gz_frame_id` is an extension while
preserving it; the resulting ROS scan frame was verified as `lidar_link`.
