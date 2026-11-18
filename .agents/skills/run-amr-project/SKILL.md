---
name: run-amr-project
description: Use when asked to run, restart, stop, or check this auto-mobile-robot repository's ROS 2 simulation on Ubuntu or Windows with WSL.
---

# Run AMR project

Resolve the repository from this skill's location (`.agents/skills/run-amr-project`)
or the current checkout; never assume a username, drive, distro name or old PID.
The target is Ubuntu 26.04 x86-64, ROS 2 Lyrical and Gazebo Jetty.
Read the repository README for installation/build instructions if setup is missing.
This workflow controls simulation; it does not authorize real hardware motion.

## Start or reuse

Inspect current processes first (`ps -eo pid,args` on Ubuntu). Reuse an existing
mapping launch from this checkout instead of starting duplicate simulators,
bridges or teleop publishers. Start only missing components.

The helper sources ROS and runs `colcon build --symlink-install` before every
launch. This creates the workspace install on first run and registers files
added after a pull or branch switch, such as `config/hardware.yaml`. It sources
the rebuilt workspace before starting ROS and stops if the build fails.

On native Ubuntu, from the repository root:

```bash
bash .agents/skills/run-amr-project/scripts/run.sh launch
```

Keep the launch session/PID available for logs and shutdown. The helper resolves
the workspace relative to itself, so it also works from another current directory.
Optional environment overrides: `AMR_WORKSPACE` (workspace directory),
`AMR_ROS_SETUP` (ROS setup file); default ROS is `/opt/ros/lyrical/setup.bash`.
It sets `QT_QPA_PLATFORM=xcb` unless explicitly overridden: the WSL default
previously failed with `Invalid parentWindowHandle`. For requested headless use,
append `gui:=false rviz:=false`; LiDAR still requires EGL/software rendering.

## Windows / WSL

Discover installed distributions with `wsl --list --quiet` and select the one
with this ROS environment; do not assume its name. Translate the absolute script
path into that distro with `wsl -d <distro> -- wslpath -a -u <Windows-script-path>`.
Use the resulting Linux path, passing it as one quoted argument:

```text
wsl -d <distro> -- bash <Linux-script-path> launch
```

For spaces in paths, prefer passing separate process arguments; if a shell parses
them, use that shell's quoting rather than JSON stringification. For GUI mode,
WSLg or another working display server is needed. Report rendering failures;
do not silently switch a requested GUI run to headless.

## Keyboard control

After the launch build completes, open a separate visible terminal and execute
the same helper with `keyboard`. It uses the existing workspace install.
On Windows, use `Start-Process wsl.exe` with `-WindowStyle Normal` and arguments
`-d <distro> -- bash "<Linux-script-path>" keyboard` correctly quoted for the
actual path. `wt.exe new-tab` previously returned without keeping teleop alive.
On native Ubuntu, use an available terminal emulator or provide the command
when a visible terminal cannot be opened. Do not run teleop without a TTY.

Verify `pgrep -af teleop_twist_keyboard` before claiming the console is ready.
Tell the user its actual window title (`AMR keyboard driving` or `wsl.exe`) and
to focus it: `i` forward, `,` reverse, `j`/`l` rotate, `k` stop.
Starting the project does not require sending robot movement commands.

## Health and diagnostics

After sourcing ROS, verify robot creation and SLAM activation in logs, plus
fresh `/scan` and `/map` using bounded `timeout 15 ros2 topic echo ... --once`.
Process existence alone does not prove a working display. Inspect GUI state
when needed; report map shader warnings rather than assuming they are harmless.
If RViz alone crashes, restart only RViz with the X11 backend, this checkout's
`ros2_ws/src/amr_bringup/config/mapping.rviz`, and `use_sim_time:=true`.

If Gazebo moves but RViz appears stuck, compare `/odom` and `map -> base_link`
motion and inspect simulator pitch. The model once tipped, aiming LiDAR at the
floor and confusing SLAM; `chassis_com_x=-0.040` corrected it. The mass-centre
regression test is `ros2_ws/src/amr_bringup/test/test_robot_stability.py`.
Treat that as a diagnostic clue, not an assumed cause of every display issue.

## Stop

Stop teleop first, then this checkout's mapping launch and separately started
RViz, using Ctrl+C/SIGINT and freshly verified PIDs. Recheck owned children after
a short wait; escalate only surviving project processes if necessary. Do not
kill unrelated ROS sessions or use `wsl --shutdown`. The installed bridge has
crashed during SIGINT and Gazebo has needed SIGTERM; disclose failures if they
recur. For "stop then check", stop the original run before reproducing and leave
diagnostic runs stopped unless the user requests otherwise.
