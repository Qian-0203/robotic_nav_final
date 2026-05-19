# ROS 2 Jazzy Migration

This branch prepares the PROS workspace for a two-phase ROS 2 Humble to Jazzy migration.

## Phase 1: Dependency Cleanup

Keep the current Humble runtime images, but make package dependencies explicit so future Jazzy images can be resolved with `rosdep`.

Completed in this branch:

- Added missing `ament_python` build tool declarations for Python ROS packages.
- Added ROS message, action, navigation, tf2, `cv_bridge`, and `ament_index_python` dependencies used by package imports.
- Added `workspace/pros/pros_car/requirements.txt`, which the existing `pros_car/Dockerfile` already copies.
- Replaced the hard-coded `/opt/ros/humble` source in `pros_car/car_control_for_docker_compose.sh` with `${ROS_DISTRO:-humble}`.

Recommended validation before Phase 2:

```bash
cd workspace/pros/pros_car
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro humble
colcon build

cd ../ros2_yolo_integration
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro humble
colcon build
```

## Phase 2: Jazzy Image Variants

Create separate Jazzy images instead of mutating the existing Humble images in place.

Required changes:

- Use Ubuntu 24.04/Noble based images for Jazzy binary packages.
- Replace `ROS_DISTRO=humble` with `ROS_DISTRO=jazzy` in Jazzy-only Dockerfiles.
- Replace `ros-humble-*` apt packages with `ros-jazzy-*`.
- Source `/opt/ros/jazzy/setup.bash`.
- Use the `jazzy` branch of source-built ROS packages such as `vision_opencv`, or prefer released `ros-jazzy-cv-bridge` packages when compatible.
- Replace Humble-specific Foxglove and project base images with Jazzy-compatible images.

Jetson note:

The current YOLO integration image is based on JetPack 6, which uses an Ubuntu 22.04 root filesystem. ROS 2 Jazzy deb packages target Ubuntu 24.04/Noble. For Jetson deployment, use a JetPack 7/Ubuntu 24.04 compatible base image or plan for a source-build based Jazzy image.
