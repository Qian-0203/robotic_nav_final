# Final Project Workflow

This runbook describes the current mission workflow for the PROS navigation
project. The active runtime is ROS 2 Humble in Docker containers.

## Runtime Pieces

| Stack | Path | Role |
| --- | --- | --- |
| `pros_app` | `workspace/pros/pros_app` | Starts Unity or real sensor infrastructure, SLAM/localization, Nav2, rosbridge/Foxglove, and support bridges. |
| `pros_car` | `workspace/pros/pros_car` | Starts car control, arm control, optional serial writers, and the mission action server. |
| `ros2_yolo_integration` | `workspace/pros/ros2_yolo_integration` | Starts YOLO detection, bridge segmentation, object offsets, and optional ArUco detection. |

All runtime containers should use the external Docker network
`compose_my_bridge_network`:

```bash
docker network create --driver bridge compose_my_bridge_network
```

## Unity Mission Workflow

Start localization and navigation for Unity:

```bash
cd workspace/pros/pros_app
./localization_unity.sh
```

This starts the Unity robot bringup plus Unity localization/navigation compose
files. Do not start a second localization stack at the same time.

Start the mission/control stack:

```bash
cd workspace/pros/pros_car
./car_control.sh --task stage:=mission enable_unity_arm_bridge:=true enable_car_c_writer:=false
```

Start YOLO in mission mode:

```bash
cd workspace/pros/ros2_yolo_integration
./yolo_activate_cu128.sh --task yolo_mode:=1 enable_arucode:=false
```

`yolo_mode:=1` publishes bounding boxes and `/yolo/object/offset`. Use
`yolo_mode:=4` when bridge segmentation is needed for bridge TF/marker output.

## Real Robot Mission Workflow

Start the real localization/navigation stack for the installed lidar:

```bash
cd workspace/pros/pros_app
./localization.sh                # RPLidar
# ./localization_ydlidar.sh      # YDLidar
# ./localization_oradarlidar.sh  # ORadar
```

Start the physical camera used by YOLO:

```bash
cd workspace/pros/pros_app
./camera_gemini.sh
# ./camera_astra.sh
# ./camera_dabai.sh
```

Start the mission stack with the real wheel writer enabled:

```bash
cd workspace/pros/pros_car
./car_control.sh --task stage:=mission enable_unity_arm_bridge:=false enable_car_c_writer:=true
```

The current mission launch starts `car_control_node`, `arm_control_node`, and
`mission_task_node`. It does not start the real arm serial writer; start the arm
writer separately if the physical arm needs serial output.

## Send A Mission

Enter the running car container and source the workspace:

```bash
docker exec -it pros_car bash
cd /workspaces
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source install/setup.bash
```

Send one mission:

```bash
ros2 run mission_task_pkg mission_task_client task1_bear
```

Supported task IDs:

| Task | Behavior |
| --- | --- |
| `task1_bear` | Prepare start pose, dynamically approach the bear, visual-servo for pickup, run `pickup_bear`, return to `start`, and run `place_bear`. |
| `task2_bridge` | Detect bridge, pre-align, handle bridge yaw, visual-servo the bridge and bear, climb, pick up bear, descend, return, and place. |
| `task3_knob` | Navigate through `knob_stage_1`, `knob_stage_2`, and `knob_stage_3`, align to the knob, run `grasp_knob`, and drive through the door. |

## Launch Arguments

`mission_task_pkg task_bringup.launch.py`:

| Argument | Default | Meaning |
| --- | --- | --- |
| `stage` | `mission` | Only `mission` is currently supported. |
| `enable_unity_arm_bridge` | `true` | Starts `unity_arm_republish_node`. Disable for real hardware unless needed. |
| `enable_car_c_writer` | `false` | Starts the real wheel serial writer. Enable for the physical robot. |
| `enable_cmd_vel_wheel_control` | `false` | Allows `car_control_node` to convert `/cmd_vel` directly to wheel commands. Keep disabled during mission visual servo unless explicitly testing it. |
| `waypoints_file` | empty | Optional override for `mission_waypoints.yaml`. |
| `mission_trees_file` | empty | Optional override for `mission_trees.yaml`. |

`yolo_pkg yolo_and_arucode.launch.py`:

| Argument | Default | Meaning |
| --- | --- | --- |
| `enable_yolo` | `true` | Starts `yolo_detection_node`. |
| `enable_arucode` | `false` | Starts `arucode_node`. |
| `yolo_mode` | `1` | `1` boxes and offsets, `2` screenshots, `3` 5 FPS screenshots, `4` segmentation, `interactive` terminal prompt. |

## Mission Data Flow

1. `mission_task_client` sends a task ID to `mission_task_server`.
2. `mission_task_node` loads the task sequence from `mission_trees.yaml`.
3. The mission publishes `/target_label` so YOLO focuses on the needed object.
4. YOLO publishes JSON object offsets on `/yolo/object/offset`.
5. The mission publishes `/initialpose` and `/goal_pose` as needed.
6. The mission calls `nav_action_server` for coarse navigation.
7. The mission publishes direct wheel commands for fine visual servo.
8. The mission calls `arm_action_server` for pickup, place, and knob actions.

## Configuration Files

| File | Purpose |
| --- | --- |
| `workspace/pros/pros_car/src/mission_task_pkg/config/mission_trees.yaml` | Mission task behavior trees and per-step overrides. |
| `workspace/pros/pros_car/src/mission_task_pkg/config/mission_waypoints.yaml` | Initial pose, waypoints, dynamic approach settings, and visual-servo timing. |
| `workspace/pros/pros_car/src/arm_control_pkg/config/arm_config.yaml` | Arm poses and arm automation settings. |
| `workspace/pros/ros2_yolo_integration/src/yolo_pkg/config/yolo_params.yaml` | Camera topics, YOLO model package, model paths, labels, and device setting. |

Current Task 3 defaults:

| Setting | Value |
| --- | --- |
| `knob_stage_1` | `x: 3.0`, `y: 0.0`, `yaw: 0.0`, `frame_id: map` |
| `knob_stage_2` | `x: 3.0`, `y: 2`, `yaw: 0.0`, `frame_id: map` |
| `knob_stage_3` | `x: 3.5`, `y: 1.5`, `yaw: 0.0`, `frame_id: map` |
| knob visual-servo depth | `0.35 m` |
| door entry drive | `FORWARD` for `20.0 s` |

## Manual Navigation Smoke Test

Use this inside the car container when checking Nav2 and car control without a
mission client:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  frame_id: map
pose:
  pose:
    orientation:
      w: 1.0
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
"

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: map
pose:
  position:
    x: 1.0
    y: 0.0
    z: 0.0
  orientation:
    w: 1.0
"

ros2 action send_goal /nav_action_server action_interface/action/NavGoal "{mode: Manual_Nav}"
```

## Rebuild Shortcuts

The bringup scripts build the full mounted workspace at startup. For focused
development, enter the container and rebuild only what changed:

```bash
cd /workspaces
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
colcon build --symlink-install --packages-select mission_task_pkg
source install/setup.bash
```

If a dependency changed, use:

```bash
colcon build --symlink-install --packages-up-to mission_task_pkg
```
