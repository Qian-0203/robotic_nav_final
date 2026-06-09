# YOLO Servo Mission Workflow

This document explains how to run the YOLO-guided mission stack and what the
mission node does during a task.

## Big Picture

The system has three parts:

- `pros_app`: localization, navigation, rosbridge, Foxglove, and Unity support.
- `pros_car`: car control, arm control, action servers, and mission logic.
- `ros2_yolo_integration`: YOLO perception and object depth offsets.

The important runtime loop is:

1. YOLO publishes object offsets on `/yolo/object/offset`.
2. `mission_task_node` selects a task from `mission_trees.yaml`.
3. The mission publishes `/target_label` so YOLO focuses on the needed object.
4. The mission publishes `/goal_pose` and asks `nav_action_server` to run
   `Manual_Nav` for coarse navigation.
5. The mission uses slow direct wheel commands for final visual alignment.
6. The mission asks `arm_action_server` to run pickup, place, or grasp actions.

All containers should be on the shared Docker network
`compose_my_bridge_network`.

## Start The Stack

Start localization and navigation first:

```bash
cd workspace/pros/pros_app
./localization_unity.sh
```

This also starts rosbridge on port `9090` and Foxglove bridge on port `8765`.
Do not start `rosbridge_server.sh` at the same time.

Start the car, arm, and mission stack:

```bash
cd workspace/pros/pros_car
./car_control.sh --task stage:=mission
```

Start YOLO perception:

```bash
cd workspace/pros/ros2_yolo_integration
./yolo_activate_cu128.sh --task yolo_mode:=1 enable_arucode:=false
```

`yolo_mode:=1` is the mode used by missions because it publishes
`/yolo/object/offset`.

## Send A Mission

Enter the car container:

```bash
docker exec -it pros_car bash
```

Source ROS and send one task:

```bash
cd /workspaces
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source install/setup.bash

ros2 run mission_task_pkg mission_task_client task1_bear
```

Supported task IDs:

- `task1_bear`
- `task2_bridge`
- `task3_knob`

## What Each Task Does

Tasks are defined in
`workspace/pros/pros_car/src/mission_task_pkg/config/mission_trees.yaml`.
The Python node does not hard-code the full task sequence anymore. It only runs
small reusable primitives.

`task1_bear`:

1. Publish the configured initial pose.
2. Approach the bear with YOLO-guided navigation.
3. Visual-servo until the bear is aligned for pickup.
4. Run arm mode `pickup_bear`.
5. Navigate back to waypoint `start`.
6. Run arm mode `place_bear`.

`task2_bridge`:

1. Publish the configured initial pose.
2. Set YOLO target label to `bridge`.
3. Wait for bridge detection.
4. Visual-servo to the bridge center.
5. Align bridge orientation from the segmentation mask angle.
6. Drive forward for `bridge_climb_sec`.
7. Set YOLO target label to `bear`.
8. Pick up the bear on top of the bridge.
9. Drive backward for `bridge_descend_sec`.
10. Navigate back to waypoint `start`.
11. Run arm mode `place_bear`.

`task3_knob`:

1. Publish the configured initial pose.
2. Navigate to `knob_stage_1`.
3. Navigate to `knob_stage_2`.
4. Set YOLO target label to `knob`.
5. Visual-servo until the knob is aligned for grasp.
6. Run arm mode `grasp_knob`.
7. Drive forward for `door_enter_sec`.

## How YOLO-Guided Approach Works

For an `ApproachDetectedTarget` step, the mission node uses this flow:

1. Publish `/target_label`.
2. Wait for a matching YOLO detection on `/yolo/object/offset`.
3. Read the current robot pose from `/amcl_pose`.
4. Convert the YOLO `offset_flu = [forward, left, up]` into a target position
   in the `map` frame.
5. Publish a stand-off `/goal_pose`.
6. Send `Manual_Nav` to `nav_action_server`.
7. If dynamic approach fails, use the configured fallback waypoint or search.

The configured stand-off distances are:

| Target | Stand-off |
| --- | --- |
| `bear` | `0.40 m` |
| `knob` | `0.50 m` |
| `bridge` | `1.00 m` |

For `bear`, short-hop approach is enabled. Instead of sending one long dynamic
goal, the mission can navigate in smaller steps and refresh the detection
between hops. If no bear detection with confidence at least `0.85` is found
within `5s`, the mission stops rotation and falls back to waypoint
`bear_approach` at `(0.25, 0.25)`.

## Visual Servo

Visual servo is the final alignment stage after navigation. It publishes
direct wheel speeds to:

- `car_C_front_wheel`
- `car_C_rear_wheel`

If the bear is still not detected during this stage, the chassis rotates
counterclockwise while looking for it.

Current tuning:

| Setting | Value |
| --- | --- |
| Forward command | `100` |
| Rotation command | `230` |
| Target depth | `0.39 m` |
| Lateral tolerance | `0.03 m` |
| Depth tolerance | `0.01 m` |
| Command period | `0.20 s` |
| Rotation settle time | `0.35 s` |
| Aligned settle time | `2.0 s` |
| Bear lateral recovery backward time | `1.0 s` |
| Bear lateral recovery max attempts | `5` |

The servo logic is simple:

- Target is left of center: rotate counterclockwise.
- Target is right of center: rotate clockwise.
- Target is centered but too far away: drive forward slowly.
- Bear is at target depth but outside lateral tolerance: drive backward briefly,
  then retry alignment.
- Target is centered and close enough: stop.
- Target is missing: rotate slowly until it is found or timeout expires.

## Main Topics And Actions

| Interface | Role |
| --- | --- |
| `mission_task_server` | Receives high-level task goals. |
| `nav_action_server` | Runs `Manual_Nav` after `/goal_pose` is published. |
| `arm_action_server` | Runs arm modes such as `pickup_bear`. |
| `/target_label` | Tells YOLO which object to prioritize. |
| `/yolo/object/offset` | YOLO object offsets as JSON. |
| `/amcl_pose` | Current localized robot pose. |
| `/goal_pose` | Navigation target for Nav2 and car control. |
| `car_C_front_wheel` | Front wheel speed command. |
| `car_C_rear_wheel` | Rear wheel speed command. |

## Useful Launch Arguments

`mission_task_pkg task_bringup.launch.py`:

| Argument | Meaning |
| --- | --- |
| `stage:=mission` | Starts car, arm, and mission nodes. |
| `enable_unity_arm_bridge:=true|false` | Starts or disables Unity arm republish. |
| `enable_car_c_writer:=true|false` | Starts or disables the real wheel serial writer. |
| `waypoints_file:=/path/to/file.yaml` | Overrides `mission_waypoints.yaml`. |
| `mission_trees_file:=/path/to/file.yaml` | Overrides `mission_trees.yaml`. |

`yolo_pkg yolo_and_arucode.launch.py`:

| Argument | Meaning |
| --- | --- |
| `enable_yolo:=true|false` | Starts YOLO detection. |
| `enable_arucode:=true|false` | Starts ArUco detection. |
| `yolo_mode:=1` | Bounding boxes plus `/yolo/object/offset`. |
| `yolo_mode:=2` | Bounding boxes plus screenshots. |
| `yolo_mode:=3` | 5 FPS screenshots. |
| `yolo_mode:=4` | Segmentation. |
| `yolo_mode:=interactive` | Restores the old terminal menu. |

## Manual Navigation Smoke Test

Use this when you want to check navigation without YOLO or the mission client:

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

## Debug Checklist

Check whether YOLO is publishing detections:

```bash
ros2 topic echo /yolo/object/offset
```

Check what YOLO target the mission requested:

```bash
ros2 topic echo /target_label
```

Check wheel publishers if wheel speed flickers or the robot stops unexpectedly:

```bash
ros2 topic info -v /car_C_front_wheel
ros2 topic info -v /car_C_rear_wheel
```

During mission bringup, these nodes may publish wheel commands:

- `car_control_node`
- `mission_task_node`
- `arm_control_node`

The mission node only publishes wheel commands during visual servo, timed drive,
or stop commands. Coarse movement should go through `/goal_pose` plus
`nav_action_server`.

Check the current navigation goal:

```bash
ros2 topic echo /goal_pose
```

Check the robot pose used for dynamic target estimation:

```bash
ros2 topic echo /amcl_pose
```
