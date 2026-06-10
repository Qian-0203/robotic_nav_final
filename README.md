# Robotic Navigation Final Project

This repository is the current PROS ROS 2 workspace for the final project. It
contains Docker-based launch stacks for SLAM, localization, Nav2 navigation,
Unity simulation, real robot sensors, car and arm control, mission
orchestration, and YOLO/ArUco perception.

## Current Architecture

The runtime is split into three cooperating workspaces:

| Area | Path | Responsibility |
| --- | --- | --- |
| Infrastructure | `workspace/pros/pros_app` | Docker Compose wrappers for lidar, cameras, SLAM, localization, Nav2, rosbridge/Foxglove, Unity bridges, GPS, IMU, and map storage. |
| Control and mission | `workspace/pros/pros_car` | ROS 2 packages for car control, arm control, custom actions, mission behavior trees, robot description, and serial writers. |
| Perception | `workspace/pros/ros2_yolo_integration` | YOLO detection, bridge segmentation, object depth offsets, ArUco depth detection, and camera geometry utilities. |

The shared Docker network is `compose_my_bridge_network`. Create it once before
starting stacks if Docker has not already created it:

```bash
docker network create --driver bridge compose_my_bridge_network
```

For the physical robot versus Unity split, see
`docs/real_vs_unity_map.md`. For the node/topic graph, see
`docs/node_topic_relation_diagram.md`.

## Quick Start: Unity Mission Stack

Start Unity localization and navigation:

```bash
cd workspace/pros/pros_app
./localization_unity.sh
```

Start car, arm, and mission orchestration:

```bash
cd workspace/pros/pros_car
./car_control.sh --task stage:=mission enable_unity_arm_bridge:=true enable_car_c_writer:=false
```

Start YOLO perception in mission mode:

```bash
cd workspace/pros/ros2_yolo_integration
./yolo_activate_cu128.sh --task yolo_mode:=1 enable_arucode:=false
```

Send a mission from inside the `pros_car` container:

```bash
docker exec -it pros_car bash
cd /workspaces
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source install/setup.bash
ros2 run mission_task_pkg mission_task_client task1_bear
```

## Quick Start: Real Robot Mission Stack

Pick the sensor stack that matches the robot hardware:

```bash
cd workspace/pros/pros_app
./localization.sh                # RPLidar
# ./localization_ydlidar.sh      # YDLidar
# ./localization_oradarlidar.sh  # ORadar
```

Start a camera stack for YOLO if needed:

```bash
cd workspace/pros/pros_app
./camera_gemini.sh
# ./camera_astra.sh
# ./camera_dabai.sh
```

Start the mission stack with the real wheel serial writer enabled:

```bash
cd workspace/pros/pros_car
./car_control.sh --task stage:=mission enable_unity_arm_bridge:=false enable_car_c_writer:=true
```

Then start YOLO with the same command used for Unity.

## Mission Tasks

Mission behavior is configured in
`workspace/pros/pros_car/src/mission_task_pkg/config/mission_trees.yaml`.
Waypoints and timing live in
`workspace/pros/pros_car/src/mission_task_pkg/config/mission_waypoints.yaml`.

Supported task IDs:

| Task | Summary |
| --- | --- |
| `task1_bear` | Prepare start pose, approach and align to the bear, pick it up, return to start, and place it. |
| `task2_bridge` | Detect and align to the bridge, cross it, pick up the bear, descend, return, and place it. |
| `task3_knob` | Navigate through knob staging waypoints, align to the knob, run the arm unlock sequence, and drive through the doorway. |

Detailed workflow notes are in `docs/final_project_workflow.md`. Architecture
notes and current technical risks are in `docs/project_architecture_review.md`.

## Development Notes

Both `task_bringup.sh` and `yolo_bringup.sh` build the mounted ROS workspace
inside their containers before launching. For fast package-only rebuilds, enter
the container and use `colcon build --symlink-install --packages-select ...` or
`--packages-up-to ...` as appropriate.

This repository still targets ROS 2 Humble at runtime. Migration notes for ROS 2
Jazzy are tracked in `docs/ros2-jazzy-migration.md`.
