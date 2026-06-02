# YOLO Servo Mission Workflow

This project is split into two ROS 2 workspaces:

- `workspace/pros/pros_car`: car control, arm control, action interfaces, and mission orchestration.
- `workspace/pros/ros2_yolo_integration`: YOLO and ArUco perception nodes.

Both workspaces run on the shared Docker network `compose_my_bridge_network`.

## Startup Order

1. Mission control stack
   - Starts `car_control_pkg/car_control_node`.
   - Provides `nav_action_server`.
   - Starts `arm_control_pkg/arm_control_node`.
   - Provides `arm_action_server`.
   - Starts `unity_arm_republish_node` by default.
   - Starts `mission_task_pkg/mission_task_node`.
   - The mission node coordinates navigation, visual servoing, and arm actions.
   - Supported tasks: `task1_bear`, `task2_bridge`, `task3_knob`.

2. Visual detection
   - Starts `yolo_pkg/yolo_detection_node`.
   - Mode `1` publishes object offsets on `/yolo/object/offset` for visual servoing.
   - The node waits for RGB and depth topics instead of prompting for terminal input.

## Container Commands

Unity localization and navigation:

```bash
cd workspace/pros/pros_app
./localization_unity.sh
```

This script also starts rosbridge on port `9090` and Foxglove bridge on port
`8765`. Do not start `rosbridge_server.sh` at the same time.

Mission control stack:

```bash
cd workspace/pros/pros_car
./car_control.sh --task stage:=mission
```

YOLO perception:

```bash
cd workspace/pros/ros2_yolo_integration
./yolo_activate.sh --task yolo_mode:=1 enable_arucode:=false
```

Send a mission task after the mission stack is running:

```bash
ros2 run mission_task_pkg mission_task_client task1_bear
```

Manual navigation smoke test:

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
    x: 0.8
    y: 0.0
    z: 0.0
  orientation:
    w: 1.0
"

ros2 action send_goal /nav_action_server action_interface/action/NavGoal "{mode: Manual_Nav}"
```

## Launch Arguments

`mission_task_pkg task_bringup.launch.py`:

- `stage:=mission`: car plus arm plus mission action server. This is the only
  supported stage.
- `enable_unity_arm_bridge:=true|false`: controls `unity_arm_republish_node`.
- `waypoints_file:=/path/to/mission_waypoints.yaml`: overrides mission waypoints.

`yolo_pkg yolo_and_arucode.launch.py`:

- `enable_yolo:=true|false`: starts YOLO detection.
- `enable_arucode:=true|false`: starts ArUco detection.
- `yolo_mode:=1`: bounding boxes plus `/yolo/object/offset`.
- `yolo_mode:=2`: bounding boxes plus screenshots.
- `yolo_mode:=3`: 5 FPS screenshots.
- `yolo_mode:=4`: segmentation.
- `yolo_mode:=interactive`: restores the old terminal menu.

## Mission Data Flow

- `/target_label`: mission node tells YOLO which object to prioritize.
- `/yolo/object/offset`: YOLO publishes detected object offsets as JSON.
- `/goal_pose`: mission node publishes navigation goals.
- `/move_base_simple/goal`: accepted as a Foxglove/RViz-compatible alias for
  `/goal_pose`.
- `nav_action_server`: car navigation action server.
- `arm_action_server`: arm action server.
- `mission_task_server`: high-level task action server.

`Manual_Nav` computes a Nav2 path from `/goal_pose` through
`/compute_path_to_pose` if no `/plan` has arrived yet.

## Test Results

Container smoke tests completed:

- `pros_car`: `colcon build --symlink-install` completed for 9 packages.
- `ros2_yolo_integration`: `colcon build --symlink-install` completed for 4 packages.
- `mission_task_pkg task_bringup.launch.py stage:=mission` started the car,
  arm, Unity arm bridge, and mission nodes.
- `yolo_pkg yolo_and_arucode.launch.py yolo_mode:=1 enable_arucode:=false` started `yolo_detection_node` and waited for RGB/depth topics.
- Manual navigation test computed a Nav2 path with 18 poses and published
  `[10.0, 10.0]` on both car wheel command topics.

The remaining warnings are setuptools deprecation warnings from the base ROS 2 Python build path.
