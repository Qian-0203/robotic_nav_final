#!/bin/bash
set -euo pipefail

: "${ROS_DISTRO:=humble}"
: "${PROS_CAR_CONTAINER_NAME:=pros_car}"

TASK_IDS=("$@")
if [ "${#TASK_IDS[@]}" -eq 0 ]; then
    TASK_IDS=(task1_bear task2_bridge_no_init task3_knob_no_init)
fi

run_sequence() {
    cd /workspaces
    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    source install/setup.bash
    set -u
    ros2 run mission_task_pkg mission_sequence_client "$@"
}

if [ -d /workspaces ] && command -v ros2 >/dev/null 2>&1; then
    run_sequence "${TASK_IDS[@]}"
    exit 0
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$PROS_CAR_CONTAINER_NAME"; then
    echo "Container '$PROS_CAR_CONTAINER_NAME' is not running."
    echo "Start it first, for example:"
    echo "  cd $(dirname "$0")"
    echo "  ./car_control.sh --task stage:=mission enable_unity_arm_bridge:=true enable_car_c_writer:=false"
    exit 1
fi

docker exec -it "$PROS_CAR_CONTAINER_NAME" bash -lc '
    set -euo pipefail
    cd /workspaces
    set +u
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
    source install/setup.bash
    set -u
    ros2 run mission_task_pkg mission_sequence_client "$@"
' bash "${TASK_IDS[@]}"
