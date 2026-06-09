import math


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi)."""
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(quat):
    """Return yaw from a ROS quaternion-like object or tuple."""
    if hasattr(quat, "x"):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
    else:
        x, y, z, w = quat
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
   Default logging verbosity is set to INFO
[INFO] [car_control_node-1]: process started with pid [1573]
[INFO] [arm_control_node-2]: process started with pid [1575]
[INFO] [unity_arm_republish_node-3]: process started with pid [1577]
[INFO] [mission_task_node-4]: process started with pid [1579]
[mission_task_node-4] Traceback (most recent call last):
[mission_task_node-4]   File "/workspaces/install/mission_task_pkg/lib/mission_task_pkg/mission_task_node", line 33, in <module>
[mission_task_node-4]     sys.exit(load_entry_point('mission-task-pkg', 'console_scripts', 'mission_task_node')())
[mission_task_node-4]   File "/workspaces/install/mission_task_pkg/lib/mission_task_pkg/mission_task_node", line 25, in importlib_load_entry_point
[mission_task_node-4]     return next(matches).load()
[mission_task_node-4]   File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 171, in load
[mission_task_node-4]     module = import_module(match.group('module'))
[mission_task_node-4]   File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
[mission_task_node-4]     return _bootstrap._gcd_import(name[level:], package, level)
[mission_task_node-4]   File "<frozen importlib._bootstrap>", line 1050, in _gcd_import
[mission_task_node-4]   File "<frozen importlib._bootstrap>", line 1027, in _find_and_load
[mission_task_node-4]   File "<frozen importlib._bootstrap>", line 1006, in _find_and_load_unlocked
[mission_task_node-4]   File "<frozen importlib._bootstrap>", line 688, in _load_unlocked
[mission_task_node-4]   File "<frozen importlib._bootstrap_external>", line 879, in exec_module
[mission_task_node-4]   File "<frozen importlib._bootstrap_external>", line 1017, in get_code
[mission_task_node-4]   File "<frozen importlib._bootstrap_external>", line 947, in source_to_code
[mission_task_node-4]   File "<frozen importlib._bootstrap>", line 241, in _call_with_frames_removed
[mission_task_node-4]   File "/workspaces/build/mission_task_pkg/mission_task_pkg/mission_task_node.py", line 621
[mission_task_node-4]     action = "FORWARD_SLOW"target_depth_m
[mission_task_node-4]                            ^^^^^^^^^^^^^^
[mission_task_node-4] SyntaxError: invalid syntax
[ERROR] [mission_task_node-4]: process has died [pid 1579, exit code 1, cmd '/workspaces/install/mission_task_pkg/lib/mission_task_pkg/mission_task_node --ros-args -r __node:=mission_task_node --params-file /tmp/launch_params_e97oszs7'].
[unity_arm_republish_node-3] [INFO] [1781016102.295140581] [unity_arm_republish_node]: Relaying JointTrajectoryPoint: /robot_arm_tmp  ->  /robot_arm
[arm_control_node-2] pybullet build time: Nov 28 2023 23:45:17
  return math.atan2(siny_cosp, cosy_cosp)


def snap_angle_to_axis(angle, axes=None):
    """Snap a heading to the nearest allowed map axis."""
    if axes is None:
        axes = (0.0, math.pi / 2.0, -math.pi / 2.0, math.pi)
    normalized = normalize_angle(angle)
    return min(axes, key=lambda axis: abs(normalize_angle(normalized - axis)))


def estimate_axis_aligned_bridge_heading(robot_yaw, image_angle_rad, axes=None):
    """Convert image-frame bridge angle to map heading and snap it to an axis."""
    raw_heading = normalize_angle(robot_yaw + image_angle_rad)
    return snap_angle_to_axis(raw_heading, axes=axes)


def bridge_axis_alignment_error(robot_yaw, image_angle_rad, axes=None):
    """Return signed robot yaw error to the snapped bridge axis."""
    snapped_heading = estimate_axis_aligned_bridge_heading(
        robot_yaw,
        image_angle_rad,
        axes=axes,
    )
    return normalize_angle(snapped_heading - robot_yaw), snapped_heading


def select_detection(detections, label, min_confidence=0.0):
    """Pick the nearest valid detection for a target label."""
    best = None
    for detection in detections or []:
        if not isinstance(detection, dict) or detection.get("label") != label:
            continue
        offset = detection.get("offset_flu")
        if not isinstance(offset, list) or len(offset) != 3:
            continue
        try:
            offset_flu = [float(v) for v in offset]
            confidence = float(detection.get("confidence", 0.0))
        except (TypeError, ValueError):
            continue
        if offset_flu[0] <= 0.0 or confidence < min_confidence:
            continue
        if best is None:
            best = {"confidence": confidence, "offset_flu": offset_flu}
            continue
        if offset_flu[0] < best["offset_flu"][0]:
            best = {"confidence": confidence, "offset_flu": offset_flu}
        elif offset_flu[0] == best["offset_flu"][0] and confidence > best["confidence"]:
            best = {"confidence": confidence, "offset_flu": offset_flu}
    return best


def estimate_target_map_pose(robot_x, robot_y, robot_yaw, offset_flu):
    """Project a robot-frame FLU offset into map-frame target coordinates."""
    forward, left, _height = offset_flu
    target_x = robot_x + forward * math.cos(robot_yaw) - left * math.sin(robot_yaw)
    target_y = robot_y + forward * math.sin(robot_yaw) + left * math.cos(robot_yaw)
    return target_x, target_y


def compute_standoff_goal(robot_x, robot_y, target_x, target_y, standoff_m):
    """Return a map goal pose in front of the target, facing the target."""
    dx = target_x - robot_x
    dy = target_y - robot_y
    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        yaw = 0.0
        goal_x = robot_x
        goal_y = robot_y
    else:
        yaw = math.atan2(dy, dx)
        goal_x = target_x - standoff_m * math.cos(yaw)
        goal_y = target_y - standoff_m * math.sin(yaw)
    return goal_x, goal_y, yaw


def compute_dynamic_goal(robot_x, robot_y, robot_quat, offset_flu, standoff_m):
    """Estimate target pose and dynamic stand-off goal from robot pose and YOLO offset."""
    robot_yaw = yaw_from_quaternion(robot_quat)
    target_x, target_y = estimate_target_map_pose(
        robot_x, robot_y, robot_yaw, offset_flu
    )
    goal_x, goal_y, goal_yaw = compute_standoff_goal(
        robot_x, robot_y, target_x, target_y, standoff_m
    )
    return {
        "target_x": target_x,
        "target_y": target_y,
        "goal_x": goal_x,
        "goal_y": goal_y,
        "goal_yaw": goal_yaw,
    }
