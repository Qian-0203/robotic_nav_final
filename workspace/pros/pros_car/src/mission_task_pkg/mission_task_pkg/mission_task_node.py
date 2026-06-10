import json
import math
import os
import threading
import time

import rclpy
import yaml
from action_interface.action import ArmGoal, MissionTask, NavGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from mission_task_pkg.behavior_tree import BehaviorTreeRunner
from mission_task_pkg.mission_geometry import (
    compute_dynamic_goal,
    select_detection,
    yaw_from_quaternion,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


ACTION_MAPPINGS = {
    "FORWARD": [300.0, 300.0, 300.0, 300.0],
    "COUNTERCLOCKWISE_ROTATION": [-300.0, 300.0, -300.0, 300.0],
    "CLOCKWISE_ROTATION": [300.0, -300.0, 300.0, -300.0],
    "FORWARD_SLOW": [100.0, 100.0, 100.0, 100.0],
    "BACKWARD_SLOW": [-100.0, -100.0, -100.0, -100.0],
    "COUNTERCLOCKWISE_ROTATION_SLOW": [-230.0, 230.0, -230.0, 230.0],
    "CLOCKWISE_ROTATION_SLOW": [230.0, -230.0, 230.0, -230.0],
    "STOP": [0.0, 0.0, 0.0, 0.0],
}

INITIAL_POSE_COVARIANCE = [
    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0685,
]


def _is_nearer_offset(candidate, current):
    """Return True when candidate has a smaller valid forward depth."""
    if current is None:
        return True
    return candidate[0] > 0.0 and candidate[0] < current[0]


class MissionTaskNode(Node):
    def __init__(self):
        super().__init__("mission_task_node")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("mission_trees_file", "")
        self.config = self._load_config()
        self.mission_trees = self._load_mission_trees()
        self.bt_runner = BehaviorTreeRunner(self.mission_trees, self._run_bt_primitive)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.target_label_pub = self.create_publisher(String, "/target_label", 10)
        self.control_signal_pub = self.create_publisher(
            String, "car_control_signal", 10
        )
        self.object_coordinates = {}
        self.object_detections = []
        self._last_control_action = None
        self._last_control_log_time = 0.0
        self._last_control_log_action = None
        self._last_bridge_edge_log_time = 0.0
        self._last_bridge_tf_yaw = None
        self.latest_amcl_pose = None
        self.create_subscription(
            String, "/yolo/object/offset", self._object_offset_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, 10
        )

        self._settle_after_node_start()

        self.nav_client = ActionClient(self, NavGoal, "nav_action_server")
        self.arm_client = ActionClient(self, ArmGoal, "arm_action_server")
        self.action_server = ActionServer(
            self,
            MissionTask,
            "mission_task_server",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Mission task server initialized")

    def _load_config(self):
        param_path = self.get_parameter("waypoints_file").value
        if param_path:
            config_path = param_path
        else:
            try:
                share_dir = get_package_share_directory("mission_task_pkg")
                config_path = os.path.join(share_dir, "config", "mission_waypoints.yaml")
            except Exception:
                here = os.path.dirname(os.path.dirname(__file__))
                config_path = os.path.join(here, "config", "mission_waypoints.yaml")

        with open(config_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        self.get_logger().info(f"Loaded mission config: {config_path}")
        return config

    def _load_mission_trees(self):
        param_path = self.get_parameter("mission_trees_file").value
        if param_path:
            config_path = param_path
        else:
            try:
                share_dir = get_package_share_directory("mission_task_pkg")
                config_path = os.path.join(share_dir, "config", "mission_trees.yaml")
            except Exception:
                here = os.path.dirname(os.path.dirname(__file__))
                config_path = os.path.join(here, "config", "mission_trees.yaml")

        with open(config_path, "r", encoding="utf-8") as f:
            trees = yaml.safe_load(f) or {}

        self.get_logger().info(f"Loaded mission behavior trees: {config_path}")
        return trees

    def goal_callback(self, goal_request):
        if goal_request.task_id not in self.bt_runner.task_ids():
            self.get_logger().error(f"Unknown task_id: {goal_request.task_id}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self._publish_control("STOP")
        self.get_logger().info("Mission cancel requested")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        task_id = goal_handle.request.task_id
        result = MissionTask.Result()
        try:
            self.bt_runner.run(task_id, goal_handle)
        except MissionCanceled as exc:
            goal_handle.canceled()
            result.success = False
            result.message = str(exc)
            return result
        except Exception as exc:
            self._publish_control("STOP")
            goal_handle.abort()
            result.success = False
            result.message = f"{task_id} failed: {exc}"
            return result

        self._publish_control("STOP")
        goal_handle.succeed()
        result.success = True
        result.message = f"{task_id} completed"
        return result

    def _run_bt_primitive(self, node_type, node, goal_handle):
        if node_type == "PrepareMissionStartPose":
            self._prepare_mission_start_pose()
            return
        if node_type == "SetTargetLabel":
            self._set_target_label(node["label"])
            return
        if node_type == "WaitDetection":
            label = node["label"]
            dynamic_cfg = self.config.get("dynamic_approach", {})
            phase = node.get("phase", f"wait_{label}")
            self._publish_phase(goal_handle, phase)
            self._settle_before_detection(goal_handle, label, dynamic_cfg)
            self._wait_for_detection(goal_handle, label, dynamic_cfg)
            return
        if node_type == "NavigateWaypoint":
            self._navigate(goal_handle, node["waypoint"])
            return
        if node_type == "ApproachDetectedTarget":
            self._approach_detected_target(
                goal_handle,
                node["label"],
                node.get("phase", f"approach_detected_{node['label']}"),
                node["fallback_waypoint"],
            )
            return
        if node_type == "VisualServo":
            self._visual_servo(
                goal_handle,
                node["label"],
                node.get("phase", f"align_{node['label']}"),
                target_depth=node.get("target_depth", node.get("target_depth_m")),
                lateral_tolerance=node.get("lateral_tolerance_m"),
                timeout_sec=node.get("timeout_sec"),
                allow_missing=bool(node.get("allow_missing", False)),
            )
            return
        if node_type == "ApproachBridgePreAlign":
            self._approach_bridge_pre_align(
                goal_handle,
                node.get("label", "bridge"),
                node.get("phase", "approach_bridge_y_offset"),
                node.get("offset_m"),
            )
            return
        if node_type == "ConditionalBridgeYawTurn":
            self._conditional_bridge_yaw_turn(
                goal_handle,
                node.get("phase", "bridge_yaw_180_ccw_turn"),
                node.get("action", "COUNTERCLOCKWISE_ROTATION_SLOW"),
                node.get("config_key", "bridge_yaw_180_turn_sec"),
            )
            return
        if node_type == "TimedDrive":
            self._timed_drive(
                goal_handle,
                node["phase"],
                node["action"],
                node["config_key"],
            )
            return
        if node_type == "ArmGoal":
            self._send_arm_goal(goal_handle, node["mode"])
            return
        if node_type == "ReturnToStart":
            self._return_to_start(goal_handle)
            return
        raise RuntimeError(f"Unknown behavior tree node type: {node_type}")

    def _approach_detected_target(self, goal_handle, label, phase, fallback_waypoint):
        self._check_cancel(goal_handle)
        self._set_target_label(label)
        dynamic_cfg = self.config.get("dynamic_approach", {})
        if not bool(dynamic_cfg.get("enabled", True)):
            self._fallback_approach(goal_handle, label, fallback_waypoint, "disabled")
            return

        self._publish_phase(goal_handle, phase)
        try:
            if self._use_short_hop_approach(label, dynamic_cfg):
                self._run_short_hop_approach(goal_handle, label, phase, dynamic_cfg)
            else:
                self._settle_before_detection(goal_handle, label, dynamic_cfg)
                detection = self._wait_for_detection(goal_handle, label, dynamic_cfg)
                goal_pose = self._build_dynamic_goal_pose(label, detection, dynamic_cfg)
                self._publish_goal_pose(goal_pose)
                self._send_nav_goal(goal_handle, "Manual_Nav")
        except MissionCanceled:
            raise
        except Exception as exc:
            self._publish_control("STOP")
            self.get_logger().warn(
                f"Dynamic approach to {label} failed: {exc}; using fallback"
            )
            self._fallback_approach(goal_handle, label, fallback_waypoint, str(exc))

    def _fallback_approach(self, goal_handle, label, fallback_waypoint, reason):
        dynamic_cfg = self.config.get("dynamic_approach", {})
        if not bool(dynamic_cfg.get("fallback_to_waypoint", True)):
            raise RuntimeError(f"Dynamic approach to {label} failed: {reason}")
        if self._use_waypoints():
            self._navigate(goal_handle, fallback_waypoint)
        else:
            self._search_for_target(goal_handle, label, f"find_{label}")

    def _wait_for_detection(self, goal_handle, label, dynamic_cfg):
        timeout = float(dynamic_cfg.get("detection_timeout_sec", 45.0))
        min_confidence = self._dynamic_label_float(
            dynamic_cfg.get("min_confidence", 0.0),
            label,
            0.0,
        )
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            self._check_cancel(goal_handle)
            if label == "bridge":
                detection = self._best_full_detection(label, min_confidence)
                if detection is not None:
                    touching_edges = self._mask_touching_edges(detection)
                    if not touching_edges:
                        self.get_logger().info(
                            f"Selected {label} detection: "
                            f"confidence={float(detection.get('confidence', 0.0)):.3f} "
                            f"offset_flu={detection.get('offset_flu')}"
                        )
                        return {
                            "confidence": float(detection.get("confidence", 0.0)),
                            "offset_flu": [float(v) for v in detection["offset_flu"]],
                        }
                    action = self._bridge_edge_recovery_action(
                        detection,
                        touching_edges,
                    )
                    self._log_bridge_edge_recovery(touching_edges, action)
                    self._publish_control(action)
                    time.sleep(0.1)
                    continue

            detection = select_detection(
                self.object_detections,
                label,
                min_confidence=min_confidence,
            )
            if detection is not None:
                self.get_logger().info(
                    f"Selected {label} detection: "
                    f"confidence={detection['confidence']:.3f} "
                    f"offset_flu={detection['offset_flu']}"
                )
                return detection
            self._publish_control("COUNTERCLOCKWISE_ROTATION_SLOW")
            time.sleep(0.1)
        self._publish_control("STOP")
        raise RuntimeError(
            f"Timed out waiting for {label} confidence >= {min_confidence:.2f}"
        )

    def _run_short_hop_approach(
        self,
        goal_handle,
        label,
        phase,
        dynamic_cfg,
        full_goal_builder=None,
    ):
        hop_cfg = self._short_hop_config(label, dynamic_cfg)
        max_hops = int(hop_cfg.get("max_hops", 3))
        step_distance = float(hop_cfg.get("step_distance_m", 0.6))
        goal_tolerance = float(hop_cfg.get("goal_tolerance_m", 0.15))
        detection_timeout = float(
            hop_cfg.get(
                "detection_refresh_timeout_sec",
                dynamic_cfg.get("detection_timeout_sec", 45.0),
            )
        )

        if max_hops <= 0:
            raise RuntimeError("short-hop dynamic approach requires max_hops > 0")
        if step_distance <= 0.0:
            raise RuntimeError("short-hop dynamic approach requires step_distance_m > 0")

        refresh_cfg = dict(dynamic_cfg)
        refresh_cfg["detection_timeout_sec"] = detection_timeout

        for hop_idx in range(1, max_hops + 1):
            self._publish_phase(goal_handle, f"{phase}:hop_{hop_idx}")
            self._settle_before_detection(goal_handle, label, dynamic_cfg)
            detection = self._wait_for_detection(goal_handle, label, refresh_cfg)
            if full_goal_builder is None:
                full_goal_pose = self._build_dynamic_goal_pose(
                    label,
                    detection,
                    dynamic_cfg,
                )
            else:
                full_goal_pose = full_goal_builder()
            hop_goal_pose, is_final_goal = self._build_short_hop_goal_pose(
                full_goal_pose,
                step_distance,
                goal_tolerance,
            )
            self._publish_goal_pose(hop_goal_pose)
            self.get_logger().info(
                f"Short-hop {label} approach {hop_idx}/{max_hops}: "
                f"goal=({hop_goal_pose.pose.position.x:.2f}, "
                f"{hop_goal_pose.pose.position.y:.2f}) "
                f"final={is_final_goal}"
            )
            self._send_nav_goal(goal_handle, "Manual_Nav")
            if is_final_goal:
                return

        raise RuntimeError(f"Short-hop dynamic approach to {label} exceeded {max_hops} hops")

    def _settle_before_detection(self, goal_handle, label, dynamic_cfg):
        settle_sec = self._dynamic_label_float(
            dynamic_cfg.get("amcl_settle_before_detection_sec", 0.0),
            label,
            0.0,
        )
        if settle_sec <= 0.0:
            return

        self._publish_control("STOP")
        self._publish_phase(goal_handle, f"settle_amcl_before_detection:{label}")
        self.get_logger().info(
            f"Waiting {settle_sec:.1f}s before detecting {label}"
        )
        start = time.monotonic()
        while time.monotonic() - start < settle_sec:
            self._check_cancel(goal_handle)
            time.sleep(0.1)

    def _build_dynamic_goal_pose(self, label, detection, dynamic_cfg):
        if self.latest_amcl_pose is None:
            raise RuntimeError("missing /amcl_pose")

        pose = self.latest_amcl_pose.pose.pose
        standoff = self._dynamic_label_float(
            dynamic_cfg.get("standoff_m", {}),
            label,
            0.9,
        )
        goal = compute_dynamic_goal(
            pose.position.x,
            pose.position.y,
            pose.orientation,
            detection["offset_flu"],
            standoff,
        )

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.latest_amcl_pose.header.frame_id or "map"
        msg.pose.position.x = goal["goal_x"]
        msg.pose.position.y = goal["goal_y"]
        msg.pose.position.z = 0.0
        msg.pose.orientation = self._quaternion_from_yaw(goal["goal_yaw"])
        self.get_logger().info(
            f"Dynamic {label} target=({goal['target_x']:.2f}, "
            f"{goal['target_y']:.2f}) goal=({goal['goal_x']:.2f}, "
            f"{goal['goal_y']:.2f}) yaw={goal['goal_yaw']:.2f}"
        )
        return msg

    def _build_short_hop_goal_pose(self, full_goal_pose, step_distance, goal_tolerance):
        if self.latest_amcl_pose is None:
            raise RuntimeError("missing /amcl_pose")

        robot_pose = self.latest_amcl_pose.pose.pose
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        goal_x = full_goal_pose.pose.position.x
        goal_y = full_goal_pose.pose.position.y
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = math.hypot(dx, dy)
        is_final_goal = distance <= step_distance + goal_tolerance

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = full_goal_pose.header.frame_id
        msg.pose.position.z = 0.0
        msg.pose.orientation = full_goal_pose.pose.orientation

        if is_final_goal or distance <= 1e-6:
            msg.pose.position.x = goal_x
            msg.pose.position.y = goal_y
            return msg, True

        scale = step_distance / distance
        msg.pose.position.x = robot_x + dx * scale
        msg.pose.position.y = robot_y + dy * scale
        return msg, False

    def _dynamic_label_float(self, value, label, default):
        if isinstance(value, dict):
            return float(value.get(label, value.get("default", default)))
        return float(value if value is not None else default)

    def _use_short_hop_approach(self, label, dynamic_cfg):
        hop_cfg = self._short_hop_config(label, dynamic_cfg)
        return bool(hop_cfg.get("enabled", False))

    def _short_hop_config(self, label, dynamic_cfg):
        hop_cfg = dynamic_cfg.get("short_hop", {})
        if not isinstance(hop_cfg, dict):
            return {}
        merged = dict(hop_cfg.get("default", {}))
        label_cfg = hop_cfg.get(label, {})
        if isinstance(label_cfg, dict):
            merged.update(label_cfg)
        for key, value in hop_cfg.items():
            if key not in {"default", label} and key not in merged:
                merged[key] = value
        return merged

    def _navigate(self, goal_handle, waypoint_name):
        self._check_cancel(goal_handle)
        self._publish_phase(goal_handle, f"navigate:{waypoint_name}")
        self._publish_waypoint(waypoint_name)
        time.sleep(1.0)
        self._send_nav_goal(goal_handle, "Manual_Nav")

    def _return_to_start(self, goal_handle):
        self._publish_phase(goal_handle, "return_to_start")
        self._navigate(goal_handle, "start")

    def _publish_goal_pose(self, msg):
        self._publish_repeated(self.goal_pose_pub, msg)

    def _publish_waypoint(self, waypoint_name):
        waypoint = self.config["waypoints"][waypoint_name]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = waypoint.get("frame_id", "map")
        msg.pose.position.x = float(waypoint["x"])
        msg.pose.position.y = float(waypoint["y"])
        msg.pose.position.z = float(waypoint.get("z", 0.0))
        msg.pose.orientation = self._quaternion_from_yaw(float(waypoint.get("yaw", 0.0)))
        self._publish_repeated(self.goal_pose_pub, msg)
        self.get_logger().info(
            f"Published waypoint {waypoint_name}: "
            f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )

    def _prepare_mission_start_pose(self):
        initial_pose_cfg = self.config.get("initial_pose", {})
        if not bool(initial_pose_cfg.get("publish", True)):
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = initial_pose_cfg.get("frame_id", "map")
        msg.pose.pose.position.x = float(initial_pose_cfg.get("x", 0.0))
        msg.pose.pose.position.y = float(initial_pose_cfg.get("y", 0.0))
        msg.pose.pose.position.z = float(initial_pose_cfg.get("z", 0.0))
        msg.pose.pose.orientation = self._quaternion_from_yaw(
            float(initial_pose_cfg.get("yaw", 0.0))
        )
        msg.pose.covariance = list(INITIAL_POSE_COVARIANCE)
        msg.pose.covariance[35] = float(initial_pose_cfg.get("yaw_covariance", 0.0685))

        self._publish_repeated(self.initial_pose_pub, msg)
        time.sleep(float(initial_pose_cfg.get("settle_sec", 3.0)))
        self.get_logger().info(
            "Published initial pose: "
            f"({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})"
        )

    def _settle_after_node_start(self):
        mission_cfg = self.config.get("mission", {})
        settle_sec = float(mission_cfg.get("startup_settle_sec", 3.0))
        if settle_sec <= 0.0:
            return

        self.get_logger().info(
            f"Waiting {settle_sec:.1f}s for localization to settle after startup"
        )
        time.sleep(settle_sec)

    def _set_target_label(self, label):
        msg = String()
        msg.data = label
        self._publish_repeated(self.target_label_pub, msg)
        self.get_logger().info(f"Set YOLO target label: {label}")

    def _visual_servo(
        self,
        goal_handle,
        label,
        phase,
        target_depth=None,
        lateral_tolerance=None,
        timeout_sec=None,
        allow_missing=False,
    ):
        servo_cfg = self.config.get("visual_servo", {})
        target_depth = float(target_depth or servo_cfg.get("target_depth_m", 0.45))
        lateral_tol = float(
            lateral_tolerance
            if lateral_tolerance is not None
            else servo_cfg.get("lateral_tolerance_m", 0.01)
        )
        depth_tol = float(servo_cfg.get("depth_tolerance_m", 0.08))
        timeout = float(timeout_sec or servo_cfg.get("timeout_sec", 30.0))
        command_period = float(servo_cfg.get("command_period_sec", 0.2))
        rotation_settle = float(servo_cfg.get("rotation_settle_sec", command_period))
        aligned_settle = float(servo_cfg.get("aligned_settle_sec", 3.0))
        bear_recovery_depth = float(servo_cfg.get("bear_recovery_depth_m", 1.5))
        bear_lateral_recovery_sec = float(
            servo_cfg.get("bear_lateral_recovery_backward_sec", 1.0)
        )
        bear_lateral_recovery_max_attempts = int(
            servo_cfg.get(
                "bear_lateral_recovery_max_attempts",
                servo_cfg.get("bear_recovery_max_attempts", 5),
            )
        )
        bear_min_confidence = float(servo_cfg.get("bear_candidate_min_confidence", 0.0))

        self._publish_phase(goal_handle, phase)
        start = time.monotonic()
        saw_target = False
        bear_lateral_recovery_attempts = 0
        previous_lateral = None
        previous_action = None

        while time.monotonic() - start < timeout:
            self._check_cancel(goal_handle)
            detection = (
                select_detection(
                    self.object_detections,
                    label,
                    min_confidence=bear_min_confidence,
                )
                if label == "bear"
                else None
            )
            coords_source = "offset_flu"
            if label == "bridge" and phase == "align_bridge_center":
                detection = self._best_full_detection(label)
                coords = self._bridge_mask_center_offset(detection)
                coords_source = "mask_center_offset_flu"
            else:
                coords = (
                    detection["offset_flu"]
                    if detection is not None
                    else self.object_coordinates.get(label)
                )
            if coords and len(coords) == 3:
                saw_target = True
                depth, lateral, _height = coords
                if label == "bear":
                    confidence = (
                        detection["confidence"] if detection is not None else None
                    )
                    confidence_text = (
                        f"{confidence:.3f}" if confidence is not None else "unknown"
                    )
                    self.get_logger().info(
                        f"Target bear alignment: depth={depth:.3f} m, "
                        f"lateral={lateral:.3f} m, "
                        f"confidence={confidence_text}"
                    )
                    if bear_recovery_depth > 0.0 and depth > bear_recovery_depth:
                        self._publish_control("STOP")
                        raise VisualServoRecoveryNeeded(
                            label,
                            depth,
                            bear_recovery_depth,
                        )
                    if (
                        depth <= target_depth + depth_tol
                        and abs(lateral) > lateral_tol
                        and bear_lateral_recovery_sec > 0.0
                        and bear_lateral_recovery_attempts
                        < bear_lateral_recovery_max_attempts
                    ):
                        bear_lateral_recovery_attempts += 1
                        self.get_logger().warn(
                            "Bear reached target depth but lateral is outside "
                            f"tolerance; backing up for "
                            f"{bear_lateral_recovery_sec:.1f}s "
                            f"(attempt {bear_lateral_recovery_attempts}/"
                            f"{bear_lateral_recovery_max_attempts})"
                        )
                        recovery_start = time.monotonic()
                        while (
                            time.monotonic() - recovery_start
                            < bear_lateral_recovery_sec
                        ):
                            self._check_cancel(goal_handle)
                            self._publish_control("BACKWARD_SLOW")
                            time.sleep(min(0.1, bear_lateral_recovery_sec))
                        self._publish_control("STOP")
                        time.sleep(command_period)
                        continue
                if abs(lateral) <= lateral_tol and depth <= target_depth + depth_tol:
                    self._log_visual_servo_state(
                        label,
                        phase,
                        depth,
                        lateral,
                        target_depth,
                        lateral_tol,
                        previous_lateral,
                        previous_action,
                        "STOP",
                        coords_source,
                    )
                    self._publish_control("STOP")
                    if aligned_settle > 0.0:
                        self.get_logger().info(
                            f"{label} aligned; waiting {aligned_settle:.1f}s to settle"
                        )
                        time.sleep(aligned_settle)
                    return
                if lateral > lateral_tol:
                    action = "COUNTERCLOCKWISE_ROTATION_SLOW"
                elif lateral < -lateral_tol:
                    action = "CLOCKWISE_ROTATION_SLOW"
                else:
                    action = "FORWARD_SLOW"
                self._log_visual_servo_state(
                    label,
                    phase,
                    depth,
                    lateral,
                    target_depth,
                    lateral_tol,
                    previous_lateral,
                    previous_action,
                    action,
                    coords_source,
                )
                previous_lateral = lateral
                previous_action = action
            else:
                action = "COUNTERCLOCKWISE_ROTATION_SLOW"
            self._publish_control(action)
            if action in {
                "CLOCKWISE_ROTATION_SLOW",
                "COUNTERCLOCKWISE_ROTATION_SLOW",
            }:
                time.sleep(rotation_settle)
            else:
                time.sleep(command_period)

        self._publish_control("STOP")
        if allow_missing and not saw_target:
            self.get_logger().warn(f"{label} was not detected; continuing by waypoint")
            return
        raise RuntimeError(f"Timed out while servoing to {label}")

    def _search_for_target(self, goal_handle, label, phase):
        servo_cfg = self.config.get("visual_servo", {})
        timeout = float(servo_cfg.get("search_timeout_sec", 45.0))
        self._publish_phase(goal_handle, phase)
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            self._check_cancel(goal_handle)
            coords = self.object_coordinates.get(label)
            if coords and len(coords) == 3:
                self._publish_control("STOP")
                return
            self._publish_control("COUNTERCLOCKWISE_ROTATION_SLOW")
            time.sleep(0.1)
        self._publish_control("STOP")
        raise RuntimeError(f"Timed out while searching for {label}")

    def _approach_bridge_pre_align(self, goal_handle, label, phase, offset_m=None):
        self._check_cancel(goal_handle)
        self._publish_phase(goal_handle, phase)
        if self.latest_amcl_pose is None:
            raise RuntimeError("missing /amcl_pose")

        if offset_m is None:
            offset_m = self.config.get("visual_servo", {}).get(
                "bridge_pre_align_y_offset_m", 0.5
            )
        offset_m = float(offset_m)
        dynamic_cfg = self.config.get("dynamic_approach", {})

        if offset_m != 0.0 and self._use_short_hop_approach(label, dynamic_cfg):
            self._run_short_hop_approach(
                goal_handle,
                label,
                phase,
                dynamic_cfg,
                full_goal_builder=lambda: self._build_bridge_pre_align_goal_pose(
                    self._require_full_bridge_detection(label),
                    offset_m,
                ),
            )
            return

        detection = self._best_full_detection(label)
        if detection is None:
            raise RuntimeError(f"missing full {label} detection")

        goal_pose = self._build_bridge_pre_align_goal_pose(
            detection,
            offset_m,
        )
        self._publish_goal_pose(goal_pose)
        self._send_nav_goal(goal_handle, "Manual_Nav")

    def _require_full_bridge_detection(self, label):
        detection = self._best_full_detection(label)
        if detection is None:
            raise RuntimeError(f"missing full {label} detection")
        touching_edges = self._mask_touching_edges(detection)
        if touching_edges:
            raise RuntimeError(
                f"{label} mask still reaches camera edge: {touching_edges}"
            )
        return detection

    def _build_bridge_pre_align_goal_pose(
        self,
        detection,
        offset_m,
    ):
        bridge_offset = self._bridge_detection_offset(detection)
        if bridge_offset is None:
            raise RuntimeError("bridge detection missing bridge TF origin offset")

        pose = self.latest_amcl_pose.pose.pose
        robot_yaw = yaw_from_quaternion(pose.orientation)
        bridge_tf_yaw = self._snapped_bridge_world_yaw(detection, robot_yaw)
        self._last_bridge_tf_yaw = bridge_tf_yaw
        bridge_x, bridge_y = self._offset_flu_to_map_xy(
            pose.position.x,
            pose.position.y,
            robot_yaw,
            bridge_offset,
        )
        offset_x, offset_y, goal_yaw = self._bridge_pre_align_offset_and_yaw(
            bridge_tf_yaw,
            offset_m,
        )
        axis_name = "x" if self._is_bridge_yaw_180(bridge_tf_yaw) else "y"
        frame_id = self.latest_amcl_pose.header.frame_id or "map"
        goal_x = bridge_x + offset_x
        goal_y = bridge_y + offset_y
        self.get_logger().info(
            "Bridge pre-align determined: "
            f"bridge_offset_flu={bridge_offset} "
            f"pre_align_offset_map=({offset_x:.3f}, {offset_y:.3f}) "
            f"snapped_world_axis={axis_name} "
            f"snapped_world_yaw={bridge_tf_yaw:.3f} "
            f"bridge_tf_waypoint_offset={offset_m:.3f} "
            f"goal_map=({goal_x:.3f}, {goal_y:.3f}) "
            f"goal_yaw={goal_yaw:.3f}"
        )

        return self._make_pose_stamped(goal_x, goal_y, goal_yaw, frame_id)

    @staticmethod
    def _bridge_pre_align_offset_and_yaw(bridge_tf_yaw, offset_m):
        if MissionTaskNode._is_bridge_yaw_180(bridge_tf_yaw):
            return 0.0, -offset_m, math.pi / 2.0
        return -offset_m, 0.0, -math.pi / 2.0

    @staticmethod
    def _is_bridge_yaw_180(bridge_tf_yaw):
        return abs(abs(bridge_tf_yaw) - math.pi) < 1e-3

    def _make_pose_stamped(self, x, y, yaw, frame_id):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation = self._quaternion_from_yaw(yaw)
        return msg

    @staticmethod
    def _snapped_bridge_world_yaw(detection, robot_yaw):
        bridge_model = detection.get("bridge_model")
        if not isinstance(bridge_model, dict):
            return math.pi

        corners = bridge_model.get("ground_edge_corners_flu")
        if not isinstance(corners, list) or len(corners) != 2:
            return math.pi
        try:
            start = [float(value) for value in corners[0]]
            end = [float(value) for value in corners[1]]
        except (TypeError, ValueError):
            return math.pi
        if len(start) != 3 or len(end) != 3:
            return math.pi

        edge_flu = [
            end[0] - start[0],
            end[1] - start[1],
            0.0,
        ]
        edge_x, edge_y = MissionTaskNode._vector_flu_to_map_xy(robot_yaw, edge_flu)
        if abs(edge_x) < 1e-6 and abs(edge_y) < 1e-6:
            return math.pi
        return math.pi if abs(edge_x) >= abs(edge_y) else math.pi / 2.0

    @staticmethod
    def _bridge_detection_offset(detection):
        bridge_model = detection.get("bridge_model")
        if isinstance(bridge_model, dict):
            offset = bridge_model.get("bridge_tf_origin_flu")
            if isinstance(offset, list) and len(offset) == 3:
                return [float(value) for value in offset]
            offset = bridge_model.get("ground_contact_offset_flu")
            if isinstance(offset, list) and len(offset) == 3:
                return [float(value) for value in offset]
        offset = detection.get("offset_flu")
        if isinstance(offset, list) and len(offset) == 3:
            return [float(value) for value in offset]
        return None

    @staticmethod
    def _offset_flu_to_map_xy(robot_x, robot_y, robot_yaw, offset_flu):
        forward, left, _up = offset_flu
        return (
            robot_x + forward * math.cos(robot_yaw) - left * math.sin(robot_yaw),
            robot_y + forward * math.sin(robot_yaw) + left * math.cos(robot_yaw),
        )

    @staticmethod
    def _vector_flu_to_map_xy(robot_yaw, vector_flu):
        forward, left, _up = vector_flu
        return (
            forward * math.cos(robot_yaw) - left * math.sin(robot_yaw),
            forward * math.sin(robot_yaw) + left * math.cos(robot_yaw),
        )

    def _timed_drive(self, goal_handle, phase, action, config_key):
        servo_cfg = self.config.get("visual_servo", {})
        duration = float(servo_cfg.get(config_key, 2.0))
        self._publish_phase(goal_handle, phase)
        start = time.monotonic()
        while time.monotonic() - start < duration:
            self._check_cancel(goal_handle)
            self._publish_control(action)
            time.sleep(0.1)
        self._publish_control("STOP")

    def _conditional_bridge_yaw_turn(self, goal_handle, phase, action, config_key):
        self._publish_phase(goal_handle, phase)
        if self._last_bridge_tf_yaw is None:
            self.get_logger().warn("No bridge yaw cached; skipping conditional turn")
            return
        if not self._is_bridge_yaw_180(self._last_bridge_tf_yaw):
            self.get_logger().info("Bridge yaw is 90 deg; skipping CCW pre-center turn")
            return
        self.get_logger().info("Bridge yaw is 180 deg; turning CCW before center align")
        self._timed_drive(goal_handle, phase, action, config_key)

    def _best_full_detection(self, label, min_confidence=0.0):
        best = None
        for detection in self.object_detections or []:
            if not isinstance(detection, dict) or detection.get("label") != label:
                continue
            offset = detection.get("offset_flu")
            if not isinstance(offset, list) or len(offset) != 3:
                continue
            try:
                confidence = float(detection.get("confidence", 0.0))
                depth = float(offset[0])
            except (TypeError, ValueError):
                continue
            if depth <= 0.0 or confidence < min_confidence:
                continue
            if best is None:
                best = detection
                continue
            best_confidence = float(best.get("confidence", 0.0))
            best_depth = float(best.get("offset_flu", [float("inf")])[0])
            if confidence > best_confidence or (
                confidence == best_confidence and depth < best_depth
            ):
                best = detection
        return best

    @staticmethod
    def _bridge_mask_center_offset(detection):
        if not isinstance(detection, dict):
            return None
        offset = detection.get("mask_center_offset_flu")
        if not isinstance(offset, list) or len(offset) != 3:
            return None
        try:
            return [float(value) for value in offset]
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _mask_touching_edges(detection):
        edges = detection.get("mask_touching_edges")
        if isinstance(edges, list):
            return [str(edge) for edge in edges if edge]

        bounds = detection.get("mask_bounds_px")
        size = detection.get("mask_image_size_px")
        if not (
            isinstance(bounds, list)
            and len(bounds) == 4
            and isinstance(size, list)
            and len(size) == 2
        ):
            return []

        min_x, min_y, max_x, max_y = [float(value) for value in bounds]
        width, height = [float(value) for value in size]
        touching = []
        if min_x <= 0.0:
            touching.append("left")
        if max_x >= width - 1.0:
            touching.append("right")
        if min_y <= 0.0:
            touching.append("top")
        if max_y >= height - 1.0:
            touching.append("bottom")
        return touching

    @staticmethod
    def _bridge_edge_recovery_action(detection, touching_edges):
        if "left" in touching_edges:
            return "COUNTERCLOCKWISE_ROTATION_SLOW"
        if "right" in touching_edges:
            return "CLOCKWISE_ROTATION_SLOW"

        offset = detection.get("mask_center_offset_flu") or detection.get("offset_flu")
        if isinstance(offset, list) and len(offset) >= 2:
            try:
                lateral = float(offset[1])
            except (TypeError, ValueError):
                lateral = 0.0
            if lateral > 0.0:
                return "COUNTERCLOCKWISE_ROTATION_SLOW"
            if lateral < 0.0:
                return "CLOCKWISE_ROTATION_SLOW"
        return "COUNTERCLOCKWISE_ROTATION_SLOW"

    def _send_nav_goal(self, goal_handle, mode):
        goal = NavGoal.Goal()
        goal.mode = mode
        self._send_action_goal(goal_handle, self.nav_client, goal, "navigation")

    def _send_arm_goal(self, goal_handle, mode):
        self._publish_phase(goal_handle, f"arm:{mode}")
        goal = ArmGoal.Goal()
        goal.mode = mode
        self._send_action_goal(goal_handle, self.arm_client, goal, f"arm:{mode}")

    def _send_action_goal(self, goal_handle, client, goal, name):
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"{name} action server is not available")

        done = threading.Event()
        state = {"accepted": False, "success": False, "message": ""}

        def goal_response_callback(future):
            action_goal_handle = future.result()
            if not action_goal_handle.accepted:
                state["message"] = f"{name} goal rejected"
                done.set()
                return
            state["accepted"] = True
            result_future = action_goal_handle.get_result_async()
            result_future.add_done_callback(result_callback)

        def result_callback(future):
            goal_result = future.result()
            state["success"] = bool(goal_result.result.success)
            state["message"] = goal_result.result.message
            done.set()

        client.send_goal_async(goal).add_done_callback(goal_response_callback)

        while not done.wait(timeout=0.1):
            self._check_cancel(goal_handle)

        if not state["accepted"] or not state["success"]:
            raise RuntimeError(state["message"] or f"{name} failed")

    def _publish_control(self, action):
        if action == "STOP" and self._last_control_action == "STOP":
            return
        vel = ACTION_MAPPINGS[action]
        msg = String()
        msg.data = f"Mission_Control:{action}"
        self.control_signal_pub.publish(msg)
        self._last_control_action = action
        self._log_control_publish(action, vel[0:2], vel[2:4])

    @staticmethod
    def _publish_repeated(publisher, msg, count=3, interval=0.1):
        for _ in range(count):
            publisher.publish(msg)
            time.sleep(interval)

    def _log_control_publish(self, action, front_vel, rear_vel):
        now = time.monotonic()
        if (
            action == self._last_control_log_action
            and now - self._last_control_log_time < 1.0
        ):
            return
        self._last_control_log_action = action
        self._last_control_log_time = now
        self.get_logger().info(
            "Chassis command "
            f"{action}: front={list(front_vel)} rear={list(rear_vel)} "
            f"control_signal_subs={self.control_signal_pub.get_subscription_count()} "
            f"front_publishers={self.count_publishers('car_C_front_wheel')} "
            f"rear_publishers={self.count_publishers('car_C_rear_wheel')}"
        )

    def _log_visual_servo_state(
        self,
        label,
        phase,
        depth,
        lateral,
        target_depth,
        lateral_tol,
        previous_lateral,
        previous_action,
        action,
        coords_source="offset_flu",
    ):
        if phase != "align_bridge_center":
            return

        trend = "first_sample"
        delta_abs = 0.0
        if previous_lateral is not None:
            delta_abs = abs(lateral) - abs(previous_lateral)
            if delta_abs < -1e-4:
                trend = "decreasing"
            elif delta_abs > 1e-4:
                trend = "increasing"
            else:
                trend = "unchanged"
        previous_lateral_text = (
            f"{previous_lateral:.3f}m"
            if previous_lateral is not None
            else "none"
        )

        self.get_logger().info(
            "Visual servo diagnostic "
            f"phase={phase} label={label} depth={depth:.3f}m "
            f"target_depth={target_depth:.3f}m lateral={lateral:.3f}m "
            f"abs_lateral={abs(lateral):.3f}m lateral_tol={lateral_tol:.3f}m "
            f"coords_source={coords_source} "
            f"prev_lateral={previous_lateral_text}"
            f" prev_action={previous_action or 'none'} next_action={action} "
            f"delta_abs_lateral={delta_abs:+.3f}m trend={trend}"
        )

    def _log_bridge_edge_recovery(self, touching_edges, action):
        now = time.monotonic()
        if now - self._last_bridge_edge_log_time < 1.0:
            return
        self._last_bridge_edge_log_time = now
        self.get_logger().info(
            "Bridge mask reaches camera edge "
            f"{touching_edges}; rotating with {action}"
        )

    def _object_offset_callback(self, msg):
        try:
            object_list = json.loads(msg.data)
            self.object_detections = object_list if isinstance(object_list, list) else []
            coordinates = {}
            for item in self.object_detections:
                label = item.get("label")
                offset = item.get("offset_flu")
                if label and isinstance(offset, list) and len(offset) == 3:
                    float_offset = [float(v) for v in offset]
                    if _is_nearer_offset(float_offset, coordinates.get(label)):
                        coordinates[label] = float_offset
            self.object_coordinates = coordinates
        except Exception as exc:
            self.get_logger().warn(f"Could not parse YOLO object offsets: {exc}")

    def _amcl_callback(self, msg):
        self.latest_amcl_pose = msg

    def _publish_phase(self, goal_handle, phase):
        feedback = MissionTask.Feedback()
        feedback.phase = phase
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"Mission phase: {phase}")

    def _check_cancel(self, goal_handle):
        if goal_handle.is_cancel_requested:
            self._publish_control("STOP")
            raise MissionCanceled("Mission canceled")

    @staticmethod
    def _quaternion_from_yaw(yaw):
        quat = Quaternion()
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

    def _use_waypoints(self):
        return bool(self.config.get("use_waypoints", False))


class MissionCanceled(Exception):
    pass


class VisualServoRecoveryNeeded(Exception):
    def __init__(self, label, depth, threshold):
        super().__init__(
            f"{label} depth {depth:.3f} m exceeds recovery threshold "
            f"{threshold:.3f} m"
        )
        self.label = label
        self.depth = depth
        self.threshold = threshold


def main(args=None):
    rclpy.init(args=args)
    node = MissionTaskNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
