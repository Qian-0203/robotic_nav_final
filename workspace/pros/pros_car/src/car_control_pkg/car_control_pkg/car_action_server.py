import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_interface.action import NavGoal
from car_control_pkg.car_nav_controller import NavigationController
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
import threading


class NavigationActionServer(Node):
    def __init__(self, car_control_node):
        super().__init__("navigation_action_server_node")
        self._action_server = ActionServer(
            self,
            NavGoal,
            "nav_action_server",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.car_control_node = car_control_node
        self.nav_controller = NavigationController(self.car_control_node)
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose"
        )
        self.get_logger().info("Navigation Action Server initialized")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        requested_mode = goal_request.mode
        if requested_mode == "Manual_Nav":
            car_position, _ = self.car_control_node.get_car_position_and_orientation()
            goal_pose = self.car_control_node.get_goal_pose()
            missing = []
            if not car_position:
                missing.append("/amcl_pose")
            if not goal_pose:
                missing.append("/goal_pose")
            if missing:
                self.get_logger().error(
                    "Cannot start navigation: Missing data: " + ", ".join(missing)
                )
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Enter the cancel callback")
        self.car_control_node.publish_control("STOP")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Navigation action callback"""
        result = NavGoal.Result()
        mode = goal_handle.request.mode
        print("mode : ", mode)
        rate = self.create_rate(10)
        self.nav_controller.reset_index()
        self.car_control_node.set_custom_nav_active(True)
        try:
            if mode == "Manual_Nav":
                self.car_control_node.latest_global_plan = None
                plan_result = self._ensure_global_plan(goal_handle)
                if isinstance(plan_result, NavGoal.Result):
                    goal_handle.abort()
                    return plan_result

            while rclpy.ok():
                # First give executor time to process callbacks
                rate.sleep()
                car_auto_method = self._select_car_auto_method(mode)
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Navigation canceled by user")
                    self.car_control_node.publish_control("STOP")
                    result = NavGoal.Result(success=False, message="Navigation canceled")
                    goal_handle.canceled()
                    break

                nav_result = car_auto_method()
                if isinstance(nav_result, NavGoal.Result):
                    if nav_result.success:
                        self.get_logger().info(
                            f"Navigation completed: {nav_result.message}"
                        )
                        goal_handle.succeed()
                    else:
                        self.get_logger().error(f"Navigation failed: {nav_result.message}")
                        goal_handle.abort()
                    # Exit the loop and return the result once a final state is reached.
                    result = nav_result
                    break

                # Publish feedback if navigation is ongoing
                feedback_msg = NavGoal.Feedback()
                feedback_msg.distance_to_goal = float(0.0)
                goal_handle.publish_feedback(feedback_msg)
        finally:
            self.car_control_node.set_custom_nav_active(False)

        return result

    def _ensure_global_plan(self, goal_handle):
        if self.car_control_node.get_path_points(include_orientation=True):
            return None

        car_position, car_orientation = (
            self.car_control_node.get_car_position_and_orientation()
        )
        goal_pose_msg = self.car_control_node.latest_goal_pose
        if not car_position or goal_pose_msg is None:
            return NavGoal.Result(
                success=False,
                message="Cannot compute path: missing /amcl_pose or /goal_pose",
            )

        if not self.compute_path_client.wait_for_server(timeout_sec=5.0):
            return NavGoal.Result(
                success=False,
                message="Nav2 compute_path_to_pose action server is not available",
            )

        start_pose = PoseStamped()
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.header.frame_id = goal_pose_msg.header.frame_id or "map"
        start_pose.pose.position = car_position
        start_pose.pose.orientation = car_orientation

        compute_goal = ComputePathToPose.Goal()
        compute_goal.start = start_pose
        compute_goal.goal = goal_pose_msg
        compute_goal.use_start = True
        compute_goal.planner_id = ""

        done = threading.Event()
        state = {"accepted": False, "path": None, "message": ""}

        def goal_response_callback(future):
            action_goal_handle = future.result()
            if not action_goal_handle.accepted:
                state["message"] = "Nav2 compute path goal rejected"
                done.set()
                return
            state["accepted"] = True
            action_goal_handle.get_result_async().add_done_callback(result_callback)

        def result_callback(future):
            try:
                state["path"] = future.result().result.path
            except Exception as exc:
                state["message"] = f"Nav2 compute path failed: {exc}"
            done.set()

        self.compute_path_client.send_goal_async(compute_goal).add_done_callback(
            goal_response_callback
        )

        while not done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                self.car_control_node.publish_control("STOP")
                return NavGoal.Result(success=False, message="Navigation canceled")

        path = state["path"]
        if not state["accepted"] or path is None or not path.poses:
            return NavGoal.Result(
                success=False,
                message=state["message"] or "Nav2 returned an empty path",
            )

        self.car_control_node.latest_global_plan = path
        self.get_logger().info(f"Computed Nav2 path with {len(path.poses)} poses")
        return None

    def _select_car_auto_method(self, mode: str):
        """
        根據模式選擇對應的 arm_auto_controller 方法或創建一個可調用對象。
        """
        if mode == "Manual_Nav":
            return self.nav_controller.manual_nav
        elif mode == "Customize_Nav":
            return self.nav_controller.customize_nav
        else:
            self.get_logger().error(f"Unknown mode requested: {mode}")  # Log error here
            def unsupported_mode():
                return NavGoal.Result(
                    success=False,
                    message=f"Unknown navigation mode: {mode}",
                )

            return unsupported_mode
