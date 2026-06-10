# Robotic Navigation Final Project

This repository collects the PROS robotic navigation workspace for the final
project. It includes ROS 2 packages and Docker Compose launch files for car
control, SLAM, localization, navigation, camera/lidar support, Unity simulation,
and YOLO integration.

## Branches

- `main`: stable project workspace based on the original ROS 2 Humble setup.
- `ros2-jazzy-migration`: migration branch for ROS 2 Jazzy Docker images,
  dependency cleanup, and compose image overrides.

## Project Layout

- `workspace/pros/pros_app`: navigation app, SLAM/localization launch scripts,
  Docker Compose files, camera/lidar utilities, and Unity support.
- `workspace/pros/pros_car`: car control ROS 2 workspace and robot control
  packages.
- `workspace/pros/ros2_yolo_integration`: YOLO ROS 2 integration workspace.
- `docs`: migration notes and project documentation when available.

If you are trying to tell which files are for the physical robot and which are
for the Unity simulation, start with `docs/real_vs_unity_map.md`.

## Quick Start

Start the mission control stack with:

```bash
cd workspace/pros/pros_car
./car_control.sh --task stage:=mission  # car + arm + mission server
```

Run YOLO perception in its workspace:

```bash
cd workspace/pros/ros2_yolo_integration
./yolo_activate_cu128.sh --task yolo_mode:=1 enable_arucode:=false
```

Detailed workflow notes are in `docs/final_project_workflow.md`.

## Mission Tasks

The mission server supports:

- `task1_bear`: navigate to the bear, align, pick it up, return, and place it.
- `task2_bridge`: align to the bridge, cross it, pick up the bear, return, and
  place it.
- `task3_knob`: navigate through `knob_stage_1`, `knob_stage_2`, and
  `knob_stage_3`, align to the knob, run the arm
  `knob_pre_grasp -> knob_grasp -> knob_unlock` flow, then drive forward
  through the doorway.

Run Task 3 from the `pros_car` container after sourcing the workspace:

```bash
ros2 run mission_task_pkg mission_task_client task3_knob
```

Task 3 tuning is YAML-driven:

- Waypoints and `door_enter_sec` live in
  `workspace/pros/pros_car/src/mission_task_pkg/config/mission_waypoints.yaml`.
- The knob visual-servo depth override and fast forward `enter_after_knob`
  action live in
  `workspace/pros/pros_car/src/mission_task_pkg/config/mission_trees.yaml`.
- Arm poses live in
  `workspace/pros/pros_car/src/arm_control_pkg/config/arm_config.yaml`.
