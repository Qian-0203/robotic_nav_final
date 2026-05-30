#!/bin/bash
set -e

: "${ROS_DISTRO:=humble}"
cd /workspaces
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install
source install/setup.bash
ros2 launch yolo_pkg yolo_and_arucode.launch.py "$@"
