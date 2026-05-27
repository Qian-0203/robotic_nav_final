#!/bin/bash
set -e

: "${ROS_DISTRO:=humble}"
cd /workspaces
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install
source install/setup.bash
ros2 launch mission_task_pkg task_bringup.launch.py "$@"
