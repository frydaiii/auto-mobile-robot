# Auto Mobile Robot

A low-cost indoor mobile robot built with parts available in Vietnam, focused on **automatic charger docking** and **failure detection and recovery**. Mapping supports those goals. Hardware preparation is the current workstream; software simulation is a separate workstream.

**Status:** component selection and hardware qualification. This README records the proposed build and supplied supplier quotes; it does not claim that the hardware, firmware, docking, or charging system has been validated.

## Goals

## Stage 1 simulation — manual driving and mapping

The repository includes the Codex skill
[run-amr-project](.agents/skills/run-amr-project/SKILL.md). Open this checkout in
Codex and request `$run-amr-project` to launch it. The skill travels with the
repository under `.agents/skills`; no personal skill installation is required.
On another machine, install the dependencies below and build first. The runner
resolves the workspace from its own location and supports Ubuntu or Windows/WSL:

```bash
# From the repository root, in Ubuntu (two terminals):
bash .agents/skills/run-amr-project/scripts/run.sh launch
bash .agents/skills/run-amr-project/scripts/run.sh keyboard
# Optional headless launch:
bash .agents/skills/run-amr-project/scripts/run.sh launch gui:=false rviz:=false
```

Runner portability checks: `python3 -m unittest discover -s tests` from the root.

The `ros2_ws` colcon workspace targets **Ubuntu 26.04 x86-64, ROS 2 Lyrical,
Gazebo Jetty, SLAM Toolbox and RViz**. See the official
[ROS installation instructions](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html)
and [Gazebo/ROS pairing](https://gazebosim.org/docs/jetty/ros_installation/).
Install ROS first, then:

```bash
sudo apt update
sudo apt install ros-lyrical-desktop ros-lyrical-ros-gz ros-lyrical-slam-toolbox \
  ros-lyrical-nav2-map-server ros-lyrical-xacro ros-lyrical-teleop-twist-keyboard \
  python3-colcon-common-extensions python3-rosdep python3-pytest
sudo rosdep init  # once per machine; skip if already initialized
rosdep update
source /opt/ros/lyrical/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro lyrical
colcon build --symlink-install
colcon test --python-testing pytest
colcon test-result --verbose
source install/setup.bash
ros2 launch amr_bringup mapping.launch.py
```

Run the coverage suite from `ros2_ws` after building and sourcing both setup
files (no Gazebo instance is needed):

```bash
sudo apt install python3-coverage
ROS_DOMAIN_ID=87 python3 -m coverage run -m pytest src/amr_bringup/test && \
  python3 -m coverage report && python3 -m coverage html
```

The report enforces a 90% minimum with branch measurement enabled. It covers
all application Python modules and the mapping launch file; packaging metadata,
tests, XML/YAML/RViz assets and upstream ROS/Gazebo code are outside the code
coverage denominator. Open `ros2_ws/htmlcov/index.html` for the detailed report.
Use an unused ROS domain ID to isolate the real ROS messaging tests from other
robot sessions. The suite checks command limits/expiry, ROS forwarding with a
paused simulation clock, malformed commands, entrypoint cleanup, and GUI/headless
launch construction. Code coverage does not measure simulated map accuracy or
prove clean shutdown of upstream processes.

Disable either window with `gui:=false` or `rviz:=false`. With both disabled,
Gazebo still renders the GPU LiDAR using headless rendering; an EGL-capable
GPU or Mesa software renderer is required. All world geometry is bundled;
no Fuel model downloads are needed. Stop the launch with Ctrl+C before relaunching.

In a second terminal, source both setup files, then drive:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p use_sim_time:=true -p stamped:=false -p speed:=0.15 -p turn:=0.6 \
  -p repeat_rate:=10.0 -p key_timeout:=0.2
```

Use `i` forward, `,` reverse, `j`/`l` rotate and `k` stop. The command gate
clamps `/cmd_vel` (`Twist`) to ±0.15 m/s and ±0.6 rad/s, forwarding only
`/cmd_vel_safe` to Gazebo. Combined commands are then scaled together to keep
each wheel at or below the motor's nominal 60 RPM, preserving turn curvature.
At the default wheel size, a 0.15 m/s + 0.6 rad/s request becomes approximately
0.1418 m/s + 0.5672 rad/s. Fresh, clear feedback from all three bumpers is
required: contact or feedback older than 0.5 seconds blocks motion. Releasing
a bumper requires a new command. Keyboard repeat counts as a new command;
release the driving key after a collision. The gate publishes zero after 0.5 seconds without a fresh
command, checked every 20 ms using a steady clock even when simulation is paused.
This is a simulation command-loss guard, not hardware emergency stopping.

The robot starts at world (-2, -2), heading +X. Follow the clear perimeter
counterclockwise through approximately (2, -2), (2, 2), (-2, 2), and back to
(-2, -2); pause at corners and revisit obstacles from multiple angles. Compare
overlapping wall edges in RViz: the acceptance tolerance is 0.15 m (three map
cells). These are validation instructions, not a claim of measured map quality.

Save after the map has updated:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/indoor --fmt pgm --ros-args \
  -p use_sim_time:=true -p save_map_timeout:=10.0
```

This creates `maps/indoor.yaml` and `maps/indoor.pgm` for Stage 2. To inspect the
saved occupancy map separately, stop mapping and run:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$PWD/maps/indoor.yaml" \
  -p use_sim_time:=false
# Another sourced terminal:
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

Compare `/map` metadata (resolution 0.05, dimensions and origin) and occupied
cells before/after reload; map saving thresholds can quantize uncertain cells.
Navigation is outside this stage.

Packages: `amr_description` owns Xacro geometry, `amr_simulation` owns the 6 × 6 m
world and bridges, and `amr_bringup` owns launch, command gating, SLAM and RViz.
Xacro arguments expose chassis dimensions/mass, wheel geometry/mass/friction,
caster geometry/mass and LiDAR position/mass. Defaults assume 65 mm wheels,
with `chassis_com_x=-0.040` m placing the chassis mass behind the drive axle
so that the robot rests on both wheels and the rear caster without tipping.
The mass centre must remain inside that support triangle when changing masses.
Other defaults are
220 mm wheel separation and a 280 × 220 mm chassis. The caster is approximated
by a low-friction sphere. These physical properties are unmeasured assumptions.
The simulated LiDAR follows the **LDROBOT LD14 (not LD14P)** nominal scan
specifications: 6 Hz, 360° coverage, and 0.15–8 m range, from the
[LD14 datasheet v1.5, section 2.3](https://make.net.za/wp-content/datasheets/LDROBOT%20LD14%20Datasheet%20v1.5%20-%20Translated.pdf).
Its 383 uniformly spaced rays per revolution approximate the specified
2,300 measurements/s (383 × 6 = 2,298/s), with no duplicate endpoint.
SLAM uses the same minimum and maximum ranges. Gazebo's 0.01 m range
resolution is a simulation setting, not the LD14's measurement accuracy.
Scans are idealized and instantaneous: distance-dependent error, target
reflectivity, sunlight interference and rolling-scan distortion are not modeled.
LiDAR mounting, shape and mass remain provisional simulation assumptions.

### Simulated drive and feedback hardware

The simulation now includes motor speed limits and hardware-style feedback.
Defaults live in [`hardware.yaml`](ros2_ws/src/amr_bringup/config/hardware.yaml).
Mounting positions, update rates, IMU noise/bias and encoder resolution are
configurable. To use a separate configuration file:

```bash
bash .agents/skills/run-amr-project/scripts/run.sh launch hardware_config:=/absolute/path/hardware.yaml
```

| Hardware | ROS output | Default simulation behavior |
| --- | --- | --- |
| GA25-370 drive pair | `/joint_states`, `/odom` | 60 RPM wheel speed ceiling; measured joint rotation feeds odometry |
| A/B encoders | `/encoder_ticks` (`std_msgs/Int64MultiArray`) | Signed cumulative left/right ticks, rounded from measured joint angles; 1,440 quadrature edges per wheel revolution is an **unverified placeholder** |
| MPU6050 GY-521 | `/imu/data_raw` (`sensor_msgs/Imu`) | Central mount, 100 Hz accelerometer and gyro with configurable Gaussian noise and static bias |
| Two VL53L0X | `/range/left`, `/range/right` (`sensor_msgs/Range`) | Front left/right, angled outward 15°, 20 Hz; 25° planar fan with nine rays, provisional 0.03 m minimum and nominal 2 m maximum |
| Three SS-5GL bumper zones | `/bumper/left`, `/bumper/center`, `/bumper/right` (`std_msgs/Bool`) | Left/center/right along the front, physical contact zones with 50 Hz contact/clear snapshots |

Encoder array order is left then right, also named in its dimension label.
Counts start at zero on the first valid joint sample and reset when simulation
time moves backward or the feedback node restarts. Tick messages do not carry
timestamps; `/joint_states` retains stamped positions/velocities in radians.
The MPU6050-style output marks orientation unavailable (`orientation_covariance[0]
= -1`); simulator ground-truth orientation is not exposed as a measurement.
Acceleration includes the gravity-related stationary reading (approximately
+9.81 m/s² on Z when upright). Raw bridged sensor inputs use `/sim/...` topics.

The ToF model selects the nearest return in a horizontal fan. It approximates
the real sensor's three-dimensional field of view and can miss objects between
the rays. No return is `+inf`; a below-minimum return is `-inf`. The
[ST VL53L0X datasheet](https://www.st.com/resource/en/datasheet/vl53l0x.pdf)
specifies a 25° field of view and ranging up to 2 m under suitable conditions;
the simulated rate and near limit are selected defaults, not guarantees of
the physical breakout's performance.

Gazebo's native contact system supplies collision data. The bundled
`amr_bumper_state` plugin also publishes clear states and latches brief impacts
until the next snapshot. It withholds snapshots if required contact components
are missing. Bumper feedback freshness is measured on ROS receipt using a steady
clock; it assumes local transport is timely, not replayed. Paused simulation
stops snapshots, so driving remains blocked until feedback resumes.

Motor torque, acceleration, electrical response and encoder pulse timing are
not modeled; the drive still uses Gazebo's ideal velocity controller. IMU values
are not calibrated MPU6050 noise specifications, and ADC quantization, sensor
saturation, temperature drift, ToF reflectivity/sunlight effects, switch bounce
and flexible bumper mechanics are not modeled. These limits support software
testing and do not qualify the actual hardware.

All ROS simulation nodes use simulation time. `/clock`, `/scan`, `/odom`,
`/joint_states` and `/map` are bridged/published using standard message types.
Gazebo DiffDrive integrates wheel joint motion for odometry; simulator pose
must only be used for independent validation. TF ownership is exclusive:
SLAM publishes `map → odom`, DiffDrive publishes `odom → base_link`, and
robot-state publishing supplies wheel and `base_link → lidar_link` transforms.

Check live data with `ros2 topic hz /scan`, `ros2 topic echo /joint_states --once`,
`ros2 topic echo /odom --once`, and `ros2 run tf2_ros tf2_echo map lidar_link`.
Use `ros2 topic info /odom -v` and `/tf` to inspect publishers; `/tf` legitimately
has multiple publishers for different edges. Forward must increase odom X;
positive rotation must increase yaw. Kill keyboard teleop while moving and
verify `/cmd_vel_safe` becomes zero and wheel motion stops. Repeat after a
clean shutdown and with `gui:=false`.

If scans are missing, inspect Gazebo rendering errors and try Mesa with
`LIBGL_ALWAYS_SOFTWARE=1`; headless mode still needs rendering libraries. If
RViz scans are absent, use Best Effort reliability; map and robot-description
topics use Transient Local durability. If transforms are missing, check clock,
scan frame `lidar_link`, joint feedback and that only one launch is running.
If packages are missing, rerun rosdep and source `install/setup.bash` in every
terminal. Detailed execution evidence and pending checks are recorded in
[`ros2_ws/VALIDATION.md`](ros2_ws/VALIDATION.md).

## Hardware project goals

- Detect a low battery, reach the charging station, align with it, and confirm charging.
- Detect motion failures, collisions, and communication loss; stop safely and attempt bounded recovery when appropriate.
- Use a two-wheel differential-drive platform with a caster, wheel encoders, and inexpensive sensors.
- Keep the prototype affordable and practical to source in Vietnam.

The first milestone is a dependable drive and power platform with useful fault telemetry. Docking depends on that foundation. A detailed map alone does not prove that the robot can dock or recover reliably.

## Current configuration

| Subsystem | Current choice | Qualification still required |
| --- | --- | --- |
| Controller | ESP32-S3, already owned | Exact development-board pinout and power arrangement |
| Higher-level autonomy | Laptop for initial development | Communication protocol and loss-of-link behavior |
| Drive | Two GA25-370 brushed DC motors, 12V, approximately 60RPM, A/B encoders | Actual stall current, encoder signals, cables, and mechanical fit |
| Motor driver | One MKE-M18, two channels, I²C | Combined motor load, thermal behavior, and command-timeout stopping |
| Mechanics | Two 65mm drive wheels, shaft adapters, GA25 brackets, 25mm caster | Chassis layout, mounting heights, bumper travel, and fabrication quotes |
| Battery | Provisional protected 3S 3000mAh Li-ion pack | Current capacity, connectors, protection behavior, and charger compatibility |
| Logic power | XL4015 buck, adjusted to 5V | Output stability and ESP32 board power connection |
| Feedback | Two INA240A1 modules, INA226, MPU6050, two VL53L0X sensors, three bumper switches | Shunts, signal levels, calibration, addresses, and mounting |
| Charging station | Undecided | Dock guidance, contacts, alignment, charge detection, and electrical integration |
| Mapping | LDROBOT LD14 360° scanning LiDAR | Confirm USB adapter, mounting position, and ROS 2 scan data |

**Recorded selection changes:** MKE-M18 replaces XY-160D at BOM ID 10. MPU6050 GY-521 replaces ICM-42688 at BOM ID 17; the BOM takes precedence over the older IMU reference in the brief.

## Hardware by stage

### Stage 1 — Drive manually and draw a map

Use the ESP32-S3, two 12V 60RPM encoder motors, MKE-M18, chassis, wheels, caster, protected power system, and laptop. Add one LDROBOT LD14 with its four-pin cable and compatible USB adapter. The GY-521 is optional for the first map. The [motor selected for this build](https://www.thegioiic.com/ga25-370-dong-co-giam-toc-co-encoder-12vdc-60rpm-truc-4mm) is the GA25-370 with a 4mm shaft. BOM ID 2 retains the earlier alternative supplier quote for budgeting.

Adapt the [Linorobot2 base firmware](https://github.com/linorobot/linorobot2_hardware) to send timed wheel commands through the MKE-M18. Verify encoder direction, counts per wheel revolution, and stopping on lost commands. Run the [LD14 ROS 2 driver](https://github.com/ldrobotSensorTeam/ldlidar_ros2) and SLAM Toolbox on the laptop. Drive a loop and check that the walls align on the saved map.

### Stage 2 — Navigate on the saved map

Reuse the Stage 1 hardware. Arrange power and connectivity for untethered operation. Use localization and Nav2 to reach a marked approach point in front of the future dock from several starting positions. Check the final heading as well as position.

### Stage 3 — Dock and confirm charging

Add mating contacts, mechanical guides, and a charger matched to the battery. Use battery telemetry to trigger the return. The two TOF200Cs (ID 28) are optional aids for comparing left and right gaps during the final approach. Test lateral centering separately. Verify charging with a charger status signal or the INA226 (ID 14) installed in the charging or battery path. Test both a successful charge and a missed contact.

### Stage 4 — Detect failures and recover

| Fault | Feedback and expected response |
| --- | --- |
| Lost drive commands | Interrupt updates with the wheels raised. Verify both motor outputs turn off within the required time, including while the shared bus remains stuck. |
| Wheel commanded but stationary | Use the existing encoders to detect the fault, stop both wheels, and report the affected side. Optional INA240A1 modules and shunts (IDs 15–16) add individual motor-current measurements. |
| Collision | Use the bumper and switches (IDs 26–27) to detect contact and stop. |
| Missing LiDAR scans | Stop autonomous motion until fresh scans arrive and localization is valid. |
| Failed charging contact | Back out and realign. Bound retries, then stop and report the fault. |

The buzzer and LED (IDs 29–30) are optional local fault indicators.

## System responsibilities

The intended split is:

- **ESP32-S3:** motor commands, encoder acquisition, sensor acquisition, local stop behavior, and fault reporting.
- **Laptop:** initial navigation, docking decisions, recovery coordination, and logging.
- **Power hardware:** battery protection, fusing, motor-power disconnection, and regulated logic power.

This is a target architecture, not an implemented interface. Motor stopping on lost commands must be demonstrated independently of laptop navigation. The driver's own behavior when ESP32 communication fails also needs verification.

## Hardware bill of materials

Prices below are the **last supplied quotes**, in VND per unit or set, excluding delivery. Links are supplier references, not confirmation of current stock, specifications, or price. `QUOTE` means a fabrication drawing and supplier quote are still required.

The maintained spreadsheet format is TSV with these columns, in this order: `Id`, `Component`, `Name`, `Quantity`, `URL`, `Price`, `Note`. Keep IDs stable when changing a component. Multiple URLs in one cell are separated by semicolons.

### Stage 1 — Drive manually and draw a map

Includes the charger for manual battery charging and the optional IMU.

| Id | Component | Name | Quantity | URL | Price | Note |
| --- | --- | --- | ---: | --- | ---: | --- |
| 1 | Controller | ESP32-S3 | 1 | [Reference](https://www.espressif.com/en/products/socs/esp32-s3) | 0 | Already owned; reference URL, not a purchase listing. |
| 2 | Motor | JGA25-370 12V 60RPM with A/B encoder | 2 | [Đức Huy](https://dientuduchuy.com/products/jga25-370-12v-dong-co-dc-giam-toc-12vdc-60rpm-co-encoder-hoi-tiep-toc-do-vi-tri) | 163000 | Candidate SKU K9_55. Exact stall current unverified; confirm included encoder cables. |
| 3 | Motor bracket | 25mm metal motor mounting bracket | 2 | [Hshop](https://hshop.vn/Ga-bat-kim-loai-cho-dong-co-duong-kinh-25mm) | 20000 | Check mounting-hole alignment with motor and chassis. |
| 4 | Drive wheel | 65mm rubber wheel with 12mm hex mounting | 2 | [Hshop](https://hshop.vn/Banh-xe-robot-lop-cao-su-65mm-khop-luc-giac-12mm) | 45000 | Requires shaft adapters in ID 5. |
| 5 | Shaft adapter | 4mm motor shaft to 12mm hex adapter | 2 | [Hshop](https://hshop.vn/khop-noi-luc-giac-truc-dong-co-4mm-kich-thuoc-12x18mm-hexagon-motor-coupling) | 20000 | Confirm motor shaft diameter and usable shaft length. |
| 6 | Caster | 25mm PP swivel caster | 1 | [Hshop](https://hshop.vn/banh-xe-xoay-da-huong-bang-nhua-pp-25mm-caster-wheel) | 15000 | Mounting height must keep chassis level with drive wheels. |
| 7 | Chassis | Custom 250–300mm chassis plate | 1 | [Fabrication service](https://imaker.vn/dich-vu-in-3d-cat-laser-mica) | QUOTE | Drawing, material thickness, and quote pending. |
| 8 | Mounting hardware | M3 brass standoff assortment including screws and nuts | 1 | [Supplier](https://laptrinhnhunga-z.com/san-pham/lap-trinh-nhung-a-zg6-combo-hop-tru-cot-dong-hinh-luc-giac-m3-nhieu-loai/) | 135000 | One assortment box; choose suitable screw lengths during assembly. |
| 9 | Battery straps | 25cm battery strap | 2 | [Supplier](https://maybayrc.vn/day-buoc-pin-20cm-25cm?variantid=66672369) | 3760 | Per strap; select single 25cm variant and confirm fit. |
| 10 | Motor driver | MKE-M18 BD62130 I2C Motor Driver Module | 1 | [Hshop](https://hshop.vn/mach-dieu-khien-dong-co-dc-mke-m18-bd62130-i2c-motor-driver-module) | 165000 | Replaces XY-160D. Two channels; motor supply 8–13V; listed as 3.3V-compatible I²C. Advertised 3A/channel; chip recommended DC maximum is 2.4A, subject to cooling. Test motor current and timeout stopping. |
| 11 | Battery | MTM 3S 3000mAh protected Li-ion pack | 1 | [Supplier](https://maybommini.vn/?product=khoi-pin-sac-li-ion-12v-3-0ah-10a-3s-mtm-jack-nguon-dc-5-5x21mm) | 220000 | Provisional; advertised 5A continuous/10A peak. Confirm combined demand. Supplied DC connector needs XT60 adaptation at extra cost. |
| 12 | Charger | 3S Li-ion 12.6V 2A CC/CV charger | 1 | [Supplier](https://dientutuonglai.com/adapter-sac-pin-3s-126v-2a.html) | 70000 | Confirm charge-current compatibility, connector dimensions, and polarity with final battery. |
| 13 | Logic power supply | XL4015 adjustable buck converter | 1 | [Hshop](https://hshop.vn/mach-giamop-dc-xl4015-5a-cu-chinh-dung) | 45000 | Set and measure output at 5V before connecting electronics. |
| 17 | IMU | MPU6050 IMU 6DOF GY-521 | 1 | [Hshop](https://hshop.vn/cam-bien-6-dof-bac-tu-do-gy-521-mpu6050) | 85000 | Replaces ICM-42688 for prototype. I²C; 3-axis gyro and accelerometer. Calibrate stationary gyro bias and combine with encoders. Keep I²C signals at 3.3V. |
| 18 | Battery connectors | AMASS XT60H male/female pair with protective caps | 2 | [Supplier](https://www.linhkientot.vn/products/cap-giac-noi-dien-xt60h-duc-cai-jack-noi-nguon-dien-pin) | 22000 | Per complete male/female pair; quantity 2 means two pairs. |
| 19 | Main battery fuse holder | 16AWG inline standard automotive fuse holder | 1 | [Supplier](https://dientutuyetnga.com/products/16awg-de-cau-chi-o-to-chong-nuoc-standard-fuses) | 15000 | Holder only; fuse rating and price pending. |
| 20 | Motor fuse holders | 16AWG inline standard automotive fuse holder | 2 | [Supplier](https://dientutuyetnga.com/products/16awg-de-cau-chi-o-to-chong-nuoc-standard-fuses) | 15000 | Per holder; one per motor branch. Fuses excluded; finalize ratings and placement with wiring design. |
| 21 | Motor power switch | 12V 20A advertised rocker switch | 1 | [Supplier](https://lamchucongnghe.com/cong-tac-bap-benh-chong-nuoc-2-vi-tri-co-led-12v-20a) | 14900 | Candidate; verify actual DC contact rating before using for motor-power disconnection. |
| 22 | Power wiring | 18AWG silicone wire; red and black | 1 | [Red](https://dientutuyetnga.com/products/day-dien-chiu-nhiet-silicon-sieu-mem-18awg-do); [Black](https://dientutuyetnga.com/products/day-dien-chiu-nhiet-silicon-sieu-mem-18awg-den) | 30000 | Set combines one red sale unit and one black sale unit; confirm supplied lengths. |
| 23 | Signal wiring | 40-piece female-female 20cm jumper pack | 1 | [Hshop](https://hshop.vn/Day-Cam-Breadboard-cai-cai-20cm-cap-det-40-soi-F-F-Jumper-Wire) | 30000 | For signals; motor connectors/harnesses cost extra if absent from motors. |
| 24 | Assembly supplies | PCB, terminals, heat-shrink, and cable ties | 1 | [PCB](https://dientutuyetnga.com/products/board-dong-duc-lo-pcb-5x7cm); [Terminals](https://nshopvn.com/product/domino-kf301-2p-5-08/); [Heat-shrink](https://dientutuyetnga.com/products/hop-530-ong-gen-co-nhiet); [Cable ties](https://mangnhuatudien.com/day-thit-day-rut/) | 90000 | Combined price: one 5×7cm PCB, five KF301-2P terminals, one 530-piece heat-shrink box, and 100 cable ties; separate purchases. |
| 25 | USB data cable | UGREEN 60518 USB-C to USB-C 1m | 1 | [Hshop](https://hshop.vn/cap-usb-c-to-usb-c-dai-1m-ho-tro-pd-60w-cao-cap-60518-chinh-hang-ugreen) | 115000 | Confirm laptop and ESP32 ports; skip if a compatible data cable is available. |
| 31 | Mapping LiDAR | LDROBOT LD14 | 1 | [Shopee](https://shopee.vn/product/18248080/45116934321) | 390000 | Lowest advertised variant. Confirm cable, USB adapter, availability, and delivered price. |

Priced items in this stage total **1,997,420₫**.

### Stage 2 — Navigate on the saved map

Reuse the Stage 1 hardware. No additional BOM items are allocated to navigation.

### Stage 3 — Dock and confirm charging

Reuse the Stage 1 battery and charger. Dock contacts, guides, and station fabrication still need quotes. The TOF200Cs are optional alignment aids.

| Id | Component | Name | Quantity | URL | Price | Note |
| --- | --- | --- | ---: | --- | ---: | --- |
| 14 | Battery monitor | CJMCU-226 INA226 | 1 | [Supplier](https://dientutuyetnga.com/products/cjmcu-226-cam-bien-dong-ina226) | 63000 | Check shunt resistance and total-current range; replacement/calibration may be needed. Use an I²C address different from MKE-M18 default 0x40. |
| 28 | Distance sensors | VL53L0X TOF200C breakout | 2 | [Hshop](https://hshop.vn/cam-bien-khoang-cach-tof-laser-radar-vl53l0x) | 115000 | Confirm accessible XSHUT pins so sensors can receive distinct I²C addresses. |

Priced items in this stage total **293,000₫**.

### Stage 4 — Detect failures and recover

The motor-current sensors and shunts are optional diagnostics. The buzzer and LED are optional indicators.

| Id | Component | Name | Quantity | URL | Price | Note |
| --- | --- | --- | ---: | --- | ---: | --- |
| 15 | Motor current sensor | INA240A1 module | 2 | [Supplier](https://www.thegioiic.com/ina240a1-mach-cam-bien-dong-20v-v) | 111720 | Previously out of stock. One per motor; supplied 0.1Ω shunt needs modification. Configure bidirectional reference and ESP32 ADC scaling. |
| 16 | Shunt resistors | 0.01Ω 2512 3W 1%; pack of 5 | 1 | [Supplier](https://linhkien.cxtvn.com/8825-dien-tro-001r-2512-3w-1-5c.html) | 5000 | Five resistors per pack; confirm footprints and final measurement ranges before fitting. |
| 26 | Bumper switches | Omron SS-5GL lever microswitch | 3 | [Supplier](https://vattudientu.vn/ss5gl-cong-tac-hanh-trinh-5a-omron-co-tay-gat-thanglimit-switch-p39849821.html) | 14000 | Per switch; requires bumper actuation and mounts. |
| 27 | Bumper structure | Custom movable bumper and switch mounts | 1 | [Fabrication service](https://imaker.vn/dich-vu-in-3d-cat-laser-mica) | QUOTE | Design and quote pending; switches priced separately in ID 26. |
| 29 | Buzzer | MKE-M03 active buzzer module | 1 | [Hshop](https://hshop.vn/mach-coi-bao-mke-m03-buzzer-module) | 25000 | Listed as 5V supply with 3.3V-compatible control; connection cable included. |
| 30 | Status indicator | MKE-M01 LED module | 1 | [Hshop](https://hshop.vn/mach-led-don-mke-m01-1-led-10mm-rgybw-module) | 20000 | Select colour; listed as 5V supply with 3.3V-compatible control. |

Priced items in this stage total **315,440₫**.

### Budget

**Priced-item subtotal: 2,605,860₫**, calculated as quantity × quoted unit/set price. The owned ESP32-S3 contributes 0₫.

This is not the finished robot cost. It excludes chassis and bumper fabrication, actual fuses, delivery, connector adaptations, missing harnesses, dock hardware, any extra LD14 cable or USB adapter cost, test equipment, and the laptop. Reusing the USB cable reduces the subtotal by 115,000₫.

## Qualification blockers

### Motor and driver compatibility

The exact stall current of the selected TheGioiIC 60RPM motor is unknown. The supplied references are not measurements of that motor:

- [Open Impulse's JGA25-370 selector](https://www.openimpulse.com/blog/jga25-370-gearmotor-selector/) was reported to list 1.2A stall current for its own 12V variants.
- [Hshop's 280RPM motor listing](https://hshop.vn/dong-co-ga25-370-dc-servo-geared-motor-12v-280rpm) was reported to list 1.8A maximum loaded current, without explicitly identifying it as stall current.

Neither value qualifies the selected motor. The MKE-M18's advertised 3A/channel must also be distinguished from the reported BD62130 recommended 2.4A maximum DC, with thermal constraints. Obtain the actual module documentation and qualify both channels together before accepting the pairing.

Record supply voltage, no-load current, startup current, representative loaded current, temperature, and encoder response. Obtain stall-current evidence from the supplier or a controlled, current-limited test procedure; do not use an unrestricted battery stall as the qualification method.

### Power and protection

- The provisional 3S pack reaches 12.6V when full, within the stated MKE-M18 8–13V supply range. That comparison alone does not qualify the complete power system.
- Check combined motor and electronics demand against the pack's advertised 5A continuous/10A peak ratings.
- Finalize fuse values, wire routing, connector polarity, and motor-power disconnection after characterizing the actual load.
- Confirm charger current, battery charge limits, connector dimensions, and polarity before charging.
- Resolve how USB power and the external 5V rail connect to the exact ESP32-S3 board before using both.
- Define low-battery thresholds and verify them under load; the selected pack and protection behavior remain provisional.

### Measurement and bus integration

- Confirm INA240 module availability, shunt footprint, reference configuration, output range, and ESP32 ADC compatibility before buying or modifying modules. The 0.01Ω shunts are candidates, not a validated measurement design.
- Check INA226 shunt resistance, calibration, and total-current range.
- Resolve the stated MKE-M18 default I²C address of `0x40` against the INA226 configuration.
- Confirm both VL53L0X boards expose XSHUT and define their startup/address-assignment sequence.
- Verify breakout pull-ups and signal voltage levels across the shared I²C bus. Check motor encoder voltage levels and pinout before connecting to ESP32 inputs.

### Mechanical integration

Before fabrication, measure the actual motor shaft, adapter engagement, wheel/bracket clearances, and caster mounting height. The chassis and bumper drawings must include battery retention, wiring access, switch mounting, and bumper travel. Dock interface dimensions remain undecided and must be considered before freezing the chassis.

## Hardware preparation sequence

1. **Qualify the drive pair.** Obtain the actual motors and MKE-M18 documentation; characterize current, encoder behavior, both-channel operation, and command-loss stopping.
2. **Finalize the power design.** Select compatible battery/charger connections, fuses, disconnect hardware, and logic-power wiring using measured demand.
3. **Qualify the sensors.** Resolve current-sensor availability and ranges, voltage levels, I²C addresses, and calibration requirements.
4. **Complete mechanical drawings.** Measure real parts, design the chassis and movable bumper, then obtain fabrication quotes.
5. **Assemble and validate the mobile base.** Begin with wheels raised; progress to supervised low-speed floor tests after stop behavior is verified.
6. **Choose the dock approach.** Specify guidance, contacts, alignment tolerance, charge detection, and electrical integration before attempting autonomous charging.

Software simulation can proceed separately. Its motor, sensor, battery, and docking assumptions must remain explicit until hardware measurements are available.

## Validation checklist

These are planned checks, not completed results. Numerical limits and pass criteria still need to be set from component documentation and measurements.

- [ ] Confirm actual motor model, shaft dimensions, encoder pinout, signal levels, and cable contents.
- [ ] Characterize both motors and demonstrate acceptable driver current and temperature under representative combined load.
- [ ] Demonstrate stopping when laptop commands cease and when ESP32-to-driver communication is interrupted.
- [ ] Verify power polarity, regulated 5V output, fuse selection, and physical motor-power disconnection.
- [ ] Validate battery and motor current readings against a reference measurement across the intended range.
- [ ] Confirm stable encoder, IMU, distance-sensor, and bumper readings while motors operate.
- [ ] Demonstrate fault reporting and a safe stopped state before implementing automatic recovery.
- [ ] Define recovery limits, retry counts, and stop conditions; verify that retries cannot continue indefinitely.
- [ ] Complete chassis and bumper fit checks before fabrication approval.
- [ ] Define a repeatable docking test with starting position, alignment error, battery state, and actual charging confirmation recorded.

## Open decisions

| Decision | Evidence or work needed |
| --- | --- |
| Final motor/driver pairing | Actual motor current and combined-load qualification |
| Final battery, charger, connectors, and fuses | Load measurements and completed power wiring design |
| Motor-current measurement circuit | Available modules, confirmed shunts, reference voltage, and ADC range |
| Chassis and bumper | Measured parts, drawings, material selection, and quotes |
| Dock hardware and charging integration | Guidance method, contact design, alignment strategy, and charge confirmation |
| LD14 integration | Verify the cable and USB adapter, scan orientation, and saved-map alignment |
| Firmware and simulation stack | Interfaces and implementation choices; not specified in this hardware brief |

No build, flashing, or simulation commands are defined by the supplied brief. Add reproducible setup instructions when those implementations exist.
