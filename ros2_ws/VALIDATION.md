# Stage 1 execution evidence

Tested 2026-11-17 in WSL Ubuntu 26.04.1 LTS x86-64 with ROS 2 Lyrical
and Gazebo Sim 10 (Jetty). This is simulation validation only.

## Passed

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
