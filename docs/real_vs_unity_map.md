# Real Car vs Unity Car Map

This repo is not split into two completely separate projects. Most of the ROS
nodes are shared. The difference is mostly in which launch scripts, compose
files, and hardware writer nodes you start.

## Short Answer

- Use `workspace/pros/pros_app/*_unity.sh` for the Unity car.
- Use `workspace/pros/pros_app/slam*.sh`, `localization*.sh`, camera, lidar,
  GPS, and IMU scripts without `_unity` for the real car.
- Use `workspace/pros/pros_car` for both Unity and the real car. Its launch
  arguments decide whether it publishes to Unity topics or to real serial
  devices.
- Use `workspace/pros/ros2_yolo_integration` for both Unity and the real car,
  as long as the expected camera topics are available.

## Main Folders

| Path | Target | What it contains |
| --- | --- | --- |
| `workspace/pros/pros_app` | Real + Unity | Docker Compose launch layer for SLAM, localization, Nav2, cameras, lidar, rosbridge/Foxglove, and Unity bridges. |
| `workspace/pros/pros_car` | Real + Unity | ROS 2 packages for car control, arm control, mission tasks, actions, robot descriptions, and serial writers. |
| `workspace/pros/ros2_yolo_integration` | Real + Unity | YOLO, depth, and ArUco perception. It is target-agnostic; it only needs camera/depth topics. |
| `workspace/yolo_training` | Neither runtime car | YOLO training container, not part of car bringup. |
| `docs` | Documentation | Workflow, architecture, migration, and this split note. |

## Unity Car Files

These files are for Unity simulation or Unity topic adaptation.

### Unity `pros_app` Launch Scripts

| File | Purpose |
| --- | --- |
| `workspace/pros/pros_app/slam_unity.sh` | Starts Unity robot bringup and SLAM. |
| `workspace/pros/pros_app/localization_unity.sh` | Starts Unity robot bringup, localization, and navigation. This is the main Unity navigation stack. |
| `workspace/pros/pros_app/camera_calibration_unity.sh` | Converts Unity compressed image topics to raw camera topics before camera calibration. |
| `workspace/pros/pros_app/slam_unity.bat` | Windows wrapper for Unity SLAM. |
| `workspace/pros/pros_app/localization_unity.bat` | Windows wrapper for Unity localization. |
| `workspace/pros/pros_app/control_old.sh` | Old Unity-focused menu. |

### Unity `pros_app` Compose Files

| File | Purpose |
| --- | --- |
| `docker-compose_robot_unity.yml` | Starts `robot_bringup_unity.xml`. |
| `docker-compose_slam_unity.yml` | Adds Unity lidar transformer and SLAM. |
| `docker-compose_localization_unity.yml` | Adds Unity lidar transformer and Unity localization. |
| `docker-compose_navigation_unity.yml` | Starts Unity Nav2 navigation. |
| `docker-compose_rplidar_unity.yml` | Older Unity lidar/scan matcher path. |
| `docker-compose_lidar_transformer.yml` | Standalone Unity lidar transformer. |
| `docker-compose_image_trans_to_raw.yml` | Unity camera compressed-to-raw adapter. |

### Unity Demo/Launch Files

| File | Purpose |
| --- | --- |
| `robot_bringup_unity.xml` | Publishes the Unity robot description and scan matcher. |
| `localization_unity.xml` | AMCL/localization for Unity. |
| `navigation_unity.xml` | Nav2 navigation for Unity. |
| `slam_unity.xml` | SLAM launch variant for Unity. |
| `rplidar_unity.xml` | Unity scan matcher path. |
| `v6_unity.urdf` | Unity-specific robot URDF. |

## Real Car Files

These files talk to physical sensors or real serial devices.

### Real `pros_app` Launch Scripts

| File | Purpose |
| --- | --- |
| `workspace/pros/pros_app/slam.sh` | Real car SLAM with RPLidar. |
| `workspace/pros/pros_app/slam_ydlidar.sh` | Real car SLAM with YDLidar. |
| `workspace/pros/pros_app/slam_oradarlidar.sh` | Real car SLAM with ORadar lidar. |
| `workspace/pros/pros_app/localization.sh` | Real car localization/navigation with RPLidar. |
| `workspace/pros/pros_app/localization_ydlidar.sh` | Real car localization/navigation with YDLidar. |
| `workspace/pros/pros_app/localization_oradarlidar.sh` | Real car localization/navigation with ORadar lidar. |
| `workspace/pros/pros_app/camera_astra.sh` | Physical Astra camera. |
| `workspace/pros/pros_app/camera_dabai.sh` | Physical Orbbec Dabai camera. |
| `workspace/pros/pros_app/camera_gemini.sh` | Physical Orbbec Gemini camera. |
| `workspace/pros/pros_app/camera_calibration.sh` | Physical camera calibration. |
| `workspace/pros/pros_app/gps.sh` | Physical GPS on `/dev/usb_gps`. |
| `workspace/pros/pros_app/imu.sh` | Physical IMU on `/dev/imu_usb`. |
| `workspace/pros/pros_app/store_map.sh` | Stores a SLAM map; use after real or Unity SLAM is running. |

### Real `pros_app` Compose Files

| File | Hardware or stack |
| --- | --- |
| `docker-compose_rplidar.yml` | Real RPLidar on `/dev/usb_lidar`. |
| `docker-compose_ydlidar.yml` | Real YDLidar on `/dev/usb_ydlidar`. |
| `docker-compose_oradarlidar.yml` | Real ORadar lidar on `/dev/oradar`. |
| `docker-compose_camera_astra.yml` | Real Astra camera on `/dev/video0` and `/dev/video1`. |
| `docker-compose_camera_dabai.yml` | Real Dabai USB camera. |
| `docker-compose_camera_gemini.yml` | Real Gemini USB camera. |
| `docker-compose_gps.yml` | Real GPS on `/dev/usb_gps`. |
| `docker-compose_imu.yml` | Real IMU on `/dev/imu_usb`. |
| `docker-compose_slam.yml` | Generic real SLAM launch. |
| `docker-compose_slam_oradarlidar.yml` | ORadar-specific real SLAM launch. |
| `docker-compose_localization.yml` | Generic real localization launch. |
| `docker-compose_navigation.yml` | Generic real Nav2 launch. |

### Real Serial Device Names

The real car expects these udev names:

| Device | Expected path |
| --- | --- |
| Front wheel ESP32 | `/dev/usb_front_wheel` |
| Rear wheel ESP32 | `/dev/usb_rear_wheel` |
| Robot arm ESP32 | `/dev/usb_robot_arm` |
| RPLidar | `/dev/usb_lidar` |
| YDLidar | `/dev/usb_ydlidar` |
| ORadar lidar | `/dev/oradar` |
| GPS | `/dev/usb_gps` |
| IMU | `/dev/imu_usb` |

## Shared Car Control Workspace

`workspace/pros/pros_car` is shared, but these files reveal the target split:

| File | Target | Notes |
| --- | --- | --- |
| `src/mission_task_pkg/launch/task_bringup.launch.py` | Real + Unity | Main mission bringup. Defaults are Unity-like: `enable_unity_arm_bridge:=true`, `enable_car_c_writer:=false`. |
| `car_control.sh` | Real + Unity | Runs the car container. It passes real USB wheel/arm devices into Docker only if they exist on the host. |
| `task_bringup.sh` | Real + Unity | Builds the workspace and launches `task_bringup.launch.py`. |
| `src/car_control_pkg` | Real + Unity | Car control, manual control, and navigation action server. Publishes wheel command topics; does not itself open serial ports. |
| `src/arm_control_pkg` | Real + Unity | Arm control and action server. Also contains `unity_arm_republish.py`. |
| `src/mission_task_pkg` | Real + Unity | Mission behavior trees and mission action server. |
| `src/pros_car_py/pros_car_py/carC_serial_writer.py` | Real car | Subscribes to wheel command topics and writes to `/dev/usb_rear_wheel` and `/dev/usb_front_wheel`. |
| `src/pros_car_py/pros_car_py/carC_serial_reader.py` | Real car | Reads wheel controller serial data. |
| `src/pros_car_py/pros_car_py/arm_writer.py` | Real car | Writes `/robot_arm` commands to `/dev/usb_robot_arm`. |
| `src/pros_car_py/pros_car_py/arm_reader.py` | Real car | Reads real arm serial data. |
| `src/arm_control_pkg/arm_control_pkg/unity_arm_republish.py` | Unity | Relays `/robot_arm_tmp` to `/robot_arm` for Unity. |
| `src/arm_control_pkg/launch/arm.launch.py` | Mixed | Starts both the real arm writer and Unity arm republisher, so check this before using it on hardware. |

## Which Commands To Use

### Unity Localization/Navigation/Mission

```bash
cd workspace/pros/pros_app
./localization_unity.sh
```

```bash
cd workspace/pros/pros_car
./car_control.sh --task stage:=mission enable_unity_arm_bridge:=true enable_car_c_writer:=false
```

```bash
cd workspace/pros/ros2_yolo_integration
./yolo_activate_cu128.sh --task yolo_mode:=1 enable_arucode:=false
```

### Real Car Localization/Navigation/Mission

Pick the lidar script that matches the hardware:

```bash
cd workspace/pros/pros_app
./localization.sh              # RPLidar
# ./localization_ydlidar.sh    # YDLidar
# ./localization_oradarlidar.sh # ORadar
```

Start any physical camera stack needed by YOLO:

```bash
cd workspace/pros/pros_app
./camera_gemini.sh
# or ./camera_astra.sh
# or ./camera_dabai.sh
```

Start the car mission stack with the real wheel serial writer enabled:

```bash
cd workspace/pros/pros_car
./car_control.sh --task stage:=mission enable_unity_arm_bridge:=false enable_car_c_writer:=true
```

If the real arm also needs serial output, start `arm_writer` or use an arm
launch path that includes it. The current `task_bringup.launch.py` starts the
arm controller, but it does not start `pros_car_py arm_writer`.

## Ambiguous Or Shared Files

These are not clearly real-only or Unity-only:

| File or folder | Why |
| --- | --- |
| `docker-compose_store_map.yml` and `store_map.sh` | Stores whichever map is being produced by the active SLAM stack. |
| `docker-compose_rosbridge_server.yml` | Viewer bridge, usable for both. |
| `docker-compose_ros_topic_bridge.yml` | Topic bridging, usable for both depending on config. |
| `docker-compose_camera_calibration.yml` | Calibration tool. Real camera by default; Unity wrapper adds image conversion first. |
| `workspace/pros/ros2_yolo_integration` | Perception is target-agnostic, but the camera source can be real or Unity. |
| `workspace/pros/pros_app/control.sh` | Menu contains both real and Unity scripts. |

## Rules Of Thumb

- Filename contains `_unity`: Unity simulation path.
- Compose file mounts `/dev/...`: real hardware path.
- Node opens `serial.Serial` or imports `Serial`: real hardware path.
- `enable_car_c_writer:=true`: real wheel ESP32 output is active.
- `enable_unity_arm_bridge:=true`: Unity arm topic relay is active.
- `pros_car` mission/control logic is shared unless it starts one of the real
  serial writers or the Unity republisher.
