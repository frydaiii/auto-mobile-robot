#!/usr/bin/env bash
set -e
mode="${1:-launch}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
  launch|keyboard) ;;
  *) echo 'Usage: bash run.sh [launch|keyboard] [ROS arguments...]' >&2; exit 2 ;;
esac
amr_repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
amr_run_workspace="${AMR_WORKSPACE:-$amr_repo_root/ros2_ws}"
amr_ros_setup="${AMR_ROS_SETUP:-/opt/ros/lyrical/setup.bash}"
if [[ ! -f "$amr_ros_setup" || ! -f "$amr_run_workspace/install/setup.bash" ]]; then
  echo "ROS or workspace setup missing. Install ROS and build $amr_run_workspace first (see README)." >&2
  exit 1
fi
source "$amr_ros_setup"
cd -- "$amr_run_workspace"
source install/setup.bash
if [[ "$mode" == launch ]]; then
  # WSL RViz needs X11; allow an explicit platform override.
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
  exec ros2 launch amr_bringup mapping.launch.py "$@"
else
  printf '\033]0;AMR keyboard driving\007'
  exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -p use_sim_time:=true -p stamped:=false -p speed:=0.15 -p turn:=0.6 \
    -p repeat_rate:=10.0 -p key_timeout:=0.2 "$@"
fi
