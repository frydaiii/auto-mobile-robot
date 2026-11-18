# Simulated Hardware Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement task by task.

**Goal:** Extend the existing simulation with limited drive, encoders, IMU, ToF and bumpers.

**Architecture:** Native Gazebo sensors feed ROS bridges. Python adapters expose
raw hardware-style messages; the command gate enforces wheel and contact limits.

**Tech Stack:** ROS 2 Lyrical, Gazebo Jetty, Xacro, Python/rclpy, pytest.

**Spec:** ../specs/2026-11-17-sim-hardware-design.md

## Global Constraints

- Keep LD14 settings and existing mapping/TF ownership.
- Preserve steady-clock command expiry, including paused simulation.
- Use configurable mounting defaults approved in conversation.
- Label unmeasured parameters as assumptions; no real hardware control.

## Review Focus

- Combined/reverse commands must bound each wheel and preserve curvature.
- Missing/stale contact feedback and contact release must not revive old commands.
- Bad joint arrays or timestamps must not produce false encoder ticks.
- ToF no-return and too-close returns must follow Range infinity conventions.
- MPU6050 output must not advertise ground-truth orientation.

### Task 1: Drive and contact limits

Files: command_gate.py, command_gate_node.py, test_command_gate*.py.
Interfaces: CommandGate receives wheel geometry/RPM and bumper requirement;
receive_bumper(name, pressed, now) invalidates motion on contact; output(now) bounds wheels.

- [x] Add tests for combined/reverse wheel limits, stale contacts and release.
- [x] Run tests and observe failures; implement proportional scaling and contact watchdog.
- [x] Verify pure and ROS watchdog tests.

### Task 2: Sensor model and adapters

Files: amr.urdf.xacro, indoor.sdf, bridge.yaml, new hardware_feedback.py,
hardware_feedback_node.py, hardware.yaml, launch, setup.py, package manifests.
Interfaces: /imu/data_raw (Imu), /range/{left,right} (Range),
/encoder_ticks (Int64MultiArray; left/right counts), /bumper/{left,center,right} (Bool).
Bridge inputs live under /sim; /joint_states retains its existing ownership.

- [x] Add failing encoder/range conversion and ROS adapter tests.
- [x] Implement adapters and configurable native IMU, ToF and contact sensors.
- [x] Add launch/configuration/dependencies and test expanded sensor connectivity.
- [x] Build and run the full test suite in isolated ROS domain 87.

### Task 3: Runtime validation and documentation

Files: README.md, ros2_ws/VALIDATION.md.

- [x] Run isolated headless mapping; verify fresh scan/map/IMU/range/ticks/contact data.
- [x] Exercise simulated wheel motion/contact stop; shut down diagnostics.
- [x] Document defaults, topics, assumptions and observed verification limits.
- [x] Review diff, run final tests and leave changes available in this checkout.

## Execution record

Work in this checkout on feat/sim-hardware-sensors to retain the earlier LD14
changes. No commits, merge or publish requested. Implement inline; request one
independent final review. Encoder telemetry uses Int64MultiArray with a dimension
label identifying left/right wheels; JointState retains its standard radian units.
Runtime finding: Gazebo Contact publishes nonempty events only. A small native
snapshot plugin publishes current contact/clear state at 50 Hz, latches transient
impacts and withholds feedback when required physics components are missing.

Verification: 25 pytest tests passed, 95.35% Python coverage; two runner tests passed.
The headless smoke check measured a 60 RPM ceiling, live sensor outputs and
contact-triggered zero command despite continued drive requests. Independent
review found no Important/Critical defects. Native contact event topics are
separate from periodic snapshots; receipt-time freshness assumption documented.
