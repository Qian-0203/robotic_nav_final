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

Started in this branch:

- Added Ubuntu 24.04/Noble based Jazzy image variants:
  - `workspace/pros/pros_car/Dockerfile.jazzy`
  - `workspace/pros/pros_app/docker/Dockerfile.jazzy`
  - `workspace/pros/ros2_yolo_integration/Dockerfile.jazzy`
- Added Jazzy compose entry points:
  - `workspace/pros/pros_car/docker-compose_car_control.jazzy.yml`
  - `workspace/pros/ros2_yolo_integration/docker-compose.jazzy.yml`
- Parameterized PROS app compose image names so `.env` can switch services from Humble images to Jazzy images without copying every compose file.
- Added `workspace/pros/pros_app/docker/compose/jazzy.env.example` with the image overrides.
- Fixed interface generation in `custome_interfaces` so both service files are generated.
- Fixed `ros_communication_pkg` to import `GetLatestData` from `custome_interfaces`.

Initial validation commands:

```bash
cd workspace/pros/pros_car
docker compose -f docker-compose_car_control.jazzy.yml build
docker compose -f docker-compose_car_control.jazzy.yml run --rm car bash -lc "colcon build"

cd ../ros2_yolo_integration
docker compose -f docker-compose.jazzy.yml build
docker compose -f docker-compose.jazzy.yml run --rm yolo bash -lc "colcon build"

cd ../pros_app/docker
docker build -f Dockerfile.jazzy -t pros_app:jazzy .
cd compose
# Merge the image override values from jazzy.env.example into the local .env.
docker compose -f docker-compose_slam.yml config
```

Remaining work:

- Select or build Jazzy-compatible images for hardware-specific drivers that are not included in the generic `pros_app:jazzy` image, especially YDLIDAR, ORADAR, RPLIDAR, camera, GPS, IMU, and Unity lidar transformer packages.
- Replace the fallback Humble Foxglove bridge tag in `jazzy.env.example` after a Jazzy-compatible bridge image is selected.
- Decide whether Jetson YOLO deployment should move to a JetPack 7/Ubuntu 24.04 base image or stay as a source-build exception.

Jetson note:

The current YOLO integration image is based on JetPack 6, which uses an Ubuntu 22.04 root filesystem. ROS 2 Jazzy deb packages target Ubuntu 24.04/Noble. For Jetson deployment, use a JetPack 7/Ubuntu 24.04 compatible base image or plan for a source-build based Jazzy image.
