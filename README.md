# robotic_nav_final

ROS 2 robotic navigation workspace with Jazzy container variants for the car control, navigation app, and YOLO integration packages.

## Prerequisites

- Docker with the Compose plugin.
- Network access the first time you build the images.
- NVIDIA Container Toolkit only if you later switch to GPU-specific images. The default Jazzy YOLO image in this branch uses CPU PyTorch for portable validation.

Run all commands from the repository root unless a command changes directory.

## Build Jazzy Images

### Car Control

```bash
cd workspace/pros/pros_car
docker build -f Dockerfile.jazzy -t pros_car:jazzy .
```

This creates `pros_car:jazzy`. The image includes the runtime dependencies needed by
`ros2 run pros_car_py robot_control`, including:

- ROS/Jazzy packages: `cv_bridge`, `nav2_msgs`, `tf2_ros`, and `tf2_geometry_msgs`
- Apt Python packages: `python3-scipy`, `python3-serial`, and `python3-yaml`
- Pip packages from `requirements.txt`: `orjson`, `pydantic`, `pybullet`, and `urwid`

### PROS App

```bash
cd workspace/pros/pros_app/docker
docker build -f Dockerfile.jazzy -t pros_app:jazzy .
```

This creates `pros_app:jazzy`, used by the navigation, SLAM, localization, rosbridge, and related compose files through environment overrides.

### YOLO Integration

```bash
cd workspace/pros/ros2_yolo_integration
docker compose -f docker-compose.jazzy.yml build
```

This creates `ros2_yolo_integration:jazzy`.

## Launch Containers

### Car Control Container

```bash
cd workspace/pros/pros_car
./car_control.sh
```

The launcher opens an interactive shell in `/workspaces`, builds the mounted ROS
workspace, and sources `install/setup.bash`. After the prompt appears, run:

```bash
ros2 run pros_car_py robot_control
```

Normal launches skip dependency installation because dependencies are expected to be
baked into `pros_car:jazzy`. If dependencies changed and you want to refresh them
inside a temporary container, run:

```bash
INSTALL_DEPENDENCIES=true ./car_control.sh
```

For faster repeated launches after the workspace was already built:

```bash
BUILD_WORKSPACE=false ./car_control.sh
```

### YOLO Container

```bash
cd workspace/pros/ros2_yolo_integration
docker compose -f docker-compose.jazzy.yml run --rm yolo bash
```

Useful validation command:

```bash
docker compose -f docker-compose.jazzy.yml run --rm yolo bash -lc "colcon build"
```

### PROS App Containers

The app compose files live in `workspace/pros/pros_app/docker/compose`. They default to the original project images, but can use the Jazzy image through `jazzy.env.example`.

Example: launch SLAM with the Jazzy image override:

```bash
cd workspace/pros/pros_app/docker/compose
docker compose --env-file jazzy.env.example -f docker-compose_slam.yml up
```

Example: check the launch file without starting the full system:

```bash
docker compose --env-file jazzy.env.example -f docker-compose_slam.yml run --rm slam \
  ros2 launch /workspace/demo/slam.xml --show-args
```

Other app compose files can be launched the same way:

```bash
docker compose --env-file jazzy.env.example -f docker-compose_localization.yml up
docker compose --env-file jazzy.env.example -f docker-compose_navigation.yml up
docker compose --env-file jazzy.env.example -f docker-compose_rosbridge_server.yml up
```

## Stop Containers

Run `down` with the same compose file used for launch:

```bash
docker compose -f docker-compose_car_control.jazzy.yml down
docker compose -f docker-compose.jazzy.yml down
docker compose --env-file jazzy.env.example -f docker-compose_slam.yml down
```

## Notes

- `workspace/pros/pros_app/docker/compose/jazzy.env.example` maps compose services to `pros_app:jazzy`.
- PROS app compose files reuse the external `compose_my_bridge_network`; create it with `docker network create --driver bridge compose_my_bridge_network` if rosbridge fails before binding `9090`.
- `workspace/pros/pros_car/car_control.sh` uses `pros_car:jazzy` by default. Override it with `PROS_CAR_IMAGE=<image> ./car_control.sh`.
- Hardware-specific drivers still need final Jazzy-compatible images or packages, especially lidar, camera, GPS, IMU, and Unity bridge components.
- More migration details are in `docs/ros2-jazzy-migration.md`.
