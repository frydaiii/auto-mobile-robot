# Simulated drive and feedback hardware

Approved in conversation: add GA25-370 60 RPM wheel limits and configurable
encoder counts, an MPU6050-style accelerometer/gyroscope, two VL53L0X-style
distance sensors, and three front bumper contact zones. Default placement is
front-left/front-right ToF, left/center/right front bumpers and a central IMU.

Use Gazebo physics and native sensors with ROS adapters. Synthetic readings
independent of the world would not test obstacle/contact behavior; detailed
electrical and firmware emulation is outside this phase.

The existing 0.15 m/s, 0.6 rad/s command limits and steady-clock watchdog remain.
Scale the combined command to keep both 65 mm wheels at or below 60 RPM.
Bumper contact blocks motion; release requires a new command, and stale bumper
feedback also blocks motion. Encoder ticks derive from measured joint motion,
not commands, with configurable counts per wheel revolution (1440 provisional).
The IMU reports raw acceleration/gyro, not simulated ground-truth orientation.
ToF outputs ROS Range from a planar 25-degree fan, up to 2 m. It approximates a
VL53L0X's three-dimensional optical field; ideal geometry is not target-dependent
optical simulation. Sensor rates, mounting, noise/bias and encoder resolution
are simulation assumptions, documented and configurable.

Native contact zones approximate switch actuation; bumper flex, switch bounce,
motor torque/current and realistic load-dependent motor curves require hardware
measurements and are outside scope. No battery, docking, firmware or real motion.

Verification: pure limit/quantization/invalid-input tests, real ROS node tests,
expanded Xacro/SDF and bridge/launch checks, and an isolated headless run showing
fresh feedback and contact-triggered stop. Stop diagnostic runs after checking.
