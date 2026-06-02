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
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


ACTION_MAPPINGS = {
    "FORWARD_SLOW": [150.0, 150.0, 150.0, 150.0],
    "COUNTERCLOCKWISE_ROTATION_SLOW": [-250.0, 250.0, -250.0, 250.0],
    "CLOCKWISE_ROTATION_SLOW": [250.0, -250.0, 250.0, -250.0],
    "STOP": [0.0, 0.0, 0.0, 0.0],
}


def _is_nearer_offset(candidate, current):
    """Return True when candidate has a smaller valid forward depth."""
    if current is None:
        return True
    return candidate[0] > 0.0 and candidate[0] < current[0]


class MissionTaskNode(Node):
    def __init__(self):
        super().__init__("mission_task_node")
        self.declare_parameter("waypoints_file", "")
        self.config = self._load_config()

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.target_label_pub = self.create_publisher(String, "/target_label", 10)
        self.front_wheel_pub = self.create_publisher(
            Float32MultiArray, "car_C_front_wheel", 10
        )
        self.rear_wheel_pub = self.create_publisher(
            Float32MultiArray, "car_C_rear_wheel", 10
        )

        self.object_coordinates = {}
        self.latest_amcl_pose = None
        self.create_subscription(
            String, "/yolo/object/offset", self._object_offset_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, 10
        )

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

    def goal_callback(self, goal_request):
        if goal_request.task_id not in {"task1_bear", "task2_bridge", "task3_knob"}:
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
            if task_id == "task1_bear":
                self._run_task1_bear(goal_handle)
            elif task_id == "task2_bridge":
                self._run_task2_bridge(goal_handle)
            elif task_id == "task3_knob":
                self._run_task3_knob(goal_handle)
            else:
                raise RuntimeError(f"Unknown task_id: {task_id}")
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

    def _run_task1_bear(self, goal_handle):
        self._prepare_mission_start_pose()
        start_pose = self._capture_start_pose()
        self._go_to_target(goal_handle, "bear", "bear_approach", "find_bear")
        self._set_target_label("bear")
        self._visual_servo(goal_handle, "bear", "align_bear_for_pickup")
        self._send_arm_goal(goal_handle, "pickup_bear")
        self._return_to_start(goal_handle, start_pose)
        self._send_arm_goal(goal_handle, "place_bear")

    def _run_task2_bridge(self, goal_handle):
        self._prepare_mission_start_pose()
        start_pose = self._capture_start_pose()
        self._go_to_target(goal_handle, "bridge", "bridge_entry", "find_bridge")
        self._set_target_label("bridge")
        self._visual_servo(
            goal_handle,
            "bridge",
            "align_bridge",
            target_depth=0.8,
            timeout_sec=20.0,
        )
        self._timed_drive(goal_handle, "cross_bridge", "FORWARD_SLOW", "bridge_cross_sec")
        self._return_to_start(goal_handle, start_pose)

    def _run_task3_knob(self, goal_handle):
        self._prepare_mission_start_pose()
        self._go_to_target(goal_handle, "knob", "knob_approach", "find_knob")
        self._set_target_label("knob")
        self._visual_servo(goal_handle, "knob", "align_knob_for_opening")
        self._send_arm_goal(goal_handle, "open_knob")
        if self._use_waypoints():
            self._navigate(goal_handle, "door_inside")
        else:
            self._timed_drive(goal_handle, "enter_door", "FORWARD_SLOW", "door_enter_sec")

    def _go_to_target(self, goal_handle, label, waypoint_name, phase):
        if self._use_waypoints():
            self._navigate(goal_handle, waypoint_name)
        else:
            self._set_target_label(label)
            self._search_for_target(goal_handle, label, phase)

    def _navigate(self, goal_handle, waypoint_name):
        self._check_cancel(goal_handle)
        self._publish_phase(goal_handle, f"navigate:{waypoint_name}")
        self._publish_waypoint(waypoint_name)
        time.sleep(1.0)
        self._send_nav_goal(goal_handle, "Manual_Nav")

    def _return_to_start(self, goal_handle, start_pose):
        self._publish_phase(goal_handle, "return_to_start")
        if start_pose is None:
            self.get_logger().warn("No /amcl_pose captured; using configured start waypoint")
            self._navigate(goal_handle, "start")
            return
        for _ in range(3):
            self.goal_pose_pub.publish(start_pose)
            time.sleep(0.1)
        time.sleep(1.0)
        self._send_nav_goal(goal_handle, "Manual_Nav")

    def _capture_start_pose(self):
        deadline = time.monotonic() + 5.0
        while self.latest_amcl_pose is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if self.latest_amcl_pose is None:
            return None
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.latest_amcl_pose.header.frame_id or "map"
        msg.pose = self.latest_amcl_pose.pose.pose
        return msg

    def _publish_waypoint(self, waypoint_name):
        waypoint = self.config["waypoints"][waypoint_name]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = waypoint.get("frame_id", "map")
        msg.pose.position.x = float(waypoint["x"])
        msg.pose.position.y = float(waypoint["y"])
        msg.pose.position.z = float(waypoint.get("z", 0.0))
        msg.pose.orientation = self._quaternion_from_yaw(float(waypoint.get("yaw", 0.0)))
        for _ in range(3):
            self.goal_pose_pub.publish(msg)
            time.sleep(0.1)
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
        msg.pose.covariance = [
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(initial_pose_cfg.get("yaw_covariance", 0.0685)),
        ]

        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.1)
        time.sleep(float(initial_pose_cfg.get("settle_sec", 0.5)))
        self.get_logger().info(
            "Published initial pose: "
            f"({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})"
        )

    def _set_target_label(self, label):
        msg = String()
        msg.data = label
        for _ in range(3):
            self.target_label_pub.publish(msg)
            time.sleep(0.1)
        self.get_logger().info(f"Set YOLO target label: {label}")

    def _visual_servo(
        self,
        goal_handle,
        label,
        phase,
        target_depth=None,
        timeout_sec=None,
        allow_missing=False,
    ):
        servo_cfg = self.config.get("visual_servo", {})
        target_depth = float(target_depth or servo_cfg.get("target_depth_m", 0.45))
        lateral_tol = float(servo_cfg.get("lateral_tolerance_m", 0.01))
        depth_tol = float(servo_cfg.get("depth_tolerance_m", 0.08))
        timeout = float(timeout_sec or servo_cfg.get("timeout_sec", 30.0))

        self._publish_phase(goal_handle, phase)
        start = time.monotonic()
        saw_target = False

        while time.monotonic() - start < timeout:
            self._check_cancel(goal_handle)
            coords = self.object_coordinates.get(label)
            if coords and len(coords) == 3:
                saw_target = True
                depth, lateral, _height = coords
                if abs(lateral) <= lateral_tol and depth <= target_depth + depth_tol:
                    self._publish_control("STOP")
                    return
                if lateral > lateral_tol:
                    self._publish_control("COUNTERCLOCKWISE_ROTATION_SLOW")
                elif lateral < -lateral_tol:
                    self._publish_control("CLOCKWISE_ROTATION_SLOW")
                else:
                    self._publish_control("FORWARD_SLOW")
            else:
                self._publish_control("CLOCKWISE_ROTATION_SLOW")
            time.sleep(0.1)

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
            self._publish_control("CLOCKWISE_ROTATION_SLOW")
            time.sleep(0.1)
        self._publish_control("STOP")
        raise RuntimeError(f"Timed out while searching for {label}")

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
        vel = ACTION_MAPPINGS[action]
        front_msg = Float32MultiArray()
        rear_msg = Float32MultiArray()
        front_msg.data = vel[0:2]
        rear_msg.data = vel[2:4]
        self.front_wheel_pub.publish(front_msg)
        self.rear_wheel_pub.publish(rear_msg)

    def _object_offset_callback(self, msg):
        try:
            object_list = json.loads(msg.data)
            coordinates = {}
            for item in object_list:
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
