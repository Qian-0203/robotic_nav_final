#!/bin/bash
set -e

: "${ROS_DISTRO:=humble}"
cd /workspaces
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# The CUDA image may contain stale colcon outputs under /workspaces. Rebuilding
# with --symlink-install over those artifacts can fail with "File exists".
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
ros2 launch yolo_pkg yolo_and_arucode.launch.py "$@"
