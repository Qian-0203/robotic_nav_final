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
    return math.atan2(siny_cosp, cosy_cosp)


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
