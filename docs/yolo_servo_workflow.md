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
   - Each task uses a shared two-stage target approach:
     1. YOLO detects the target and the mission node estimates the target pose
        in `map`.
     2. The mission node publishes a coarse stand-off `/goal_pose`, runs
        `Manual_Nav`, then switches to slow YOLO visual servo for fine
        alignment.

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

```bash
docker exec -it pros_car bash   

```

Send a mission task after the mission stack is running:

```bash
cd /workspaces
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source install/setup.bash
ros2 run mission_task_pkg mission_task_client task1_bear

```
```bash
ros2 topic echo /car_C_front_wheel
ros2 topic echo /car_C_rear_wheel
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
    x: 1.0
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
- `/yolo/object/offset`: YOLO publishes detected object offsets as JSON with
  `offset_flu = [forward, left, up]`.
- `/goal_pose`: mission node publishes configured fallback waypoints, captured
  return poses, or dynamically estimated stand-off goals in front of detected
  targets.
- `/move_base_simple/goal`: accepted as a Foxglove/RViz-compatible alias for
  `/goal_pose`.
- `nav_action_server`: car navigation action server.
- `arm_action_server`: arm action server.
- `mission_task_server`: high-level task action server.

`Manual_Nav` clears its cached path and computes a fresh Nav2 path from the
latest `/goal_pose` through `/compute_path_to_pose`.

## YOLO-Guided Approach

The mission node treats map navigation as the coarse approach and visual servo
as fine alignment:

1. Publish `/target_label` for the task target.
2. Wait for a valid YOLO detection.
3. Estimate target map pose from `/amcl_pose` plus YOLO `offset_flu`.
4. Publish a stand-off `/goal_pose` facing the detected target.
5. Run `Manual_Nav`.
6. Use slow direct wheel commands for final YOLO alignment.

Default dynamic stand-off distances:

- `bear`: `0.65 m`
- `knob`: `0.65 m`
- `bridge`: `1.00 m`

If dynamic detection, pose estimation, or navigation fails, the mission falls
back to the configured waypoint for that task.

Current fine visual-servo tuning:

- forward wheel command: `100`
- rotation wheel command: `250`
- lateral tolerance: `0.03 m`
- depth tolerance: `0.04 m`
- command period: `0.20 s`
- rotation settle time: `0.35 s`

These slow commands are intentionally separate from coarse navigation. They are
only used while aligning to the YOLO target after `Manual_Nav` has already
reached the stand-off goal.

## Debug Notes

The wheel topics can have multiple publishers during mission bringup:

- `car_control_node`
- `manual_control_node`
- `mission_task_node`
- `arm_commute_node`

If wheel speed appears to flicker between motion and zero, first check active
publishers and the mission log:

```bash
ros2 topic info -v /car_C_front_wheel
ros2 topic info -v /car_C_rear_wheel
ros2 topic echo /yolo/object/offset
```

During visual servo, changing YOLO lateral/depth offsets can also make the
mission alternate between forward, clockwise, counterclockwise, and stop
commands. The coarse `/goal_pose` should place the robot close enough that this
stage only performs small final corrections.

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
