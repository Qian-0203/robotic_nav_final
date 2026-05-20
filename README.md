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

## Quick Start

For the Humble workspace on `main`, start with:

```bash
cd workspace/pros/pros_app
./control.sh
```

For Jazzy migration work, switch to the migration branch and read the branch
README:

```bash
git checkout ros2-jazzy-migration
```

Detailed launch notes are in `workspace/pros/pros_app/README.md`.
