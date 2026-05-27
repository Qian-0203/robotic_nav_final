import sys

import rclpy
from action_interface.action import MissionTask
from rclpy.action import ActionClient
from rclpy.node import Node


class MissionTaskClient(Node):
    def __init__(self):
        super().__init__("mission_task_client")
        self.client = ActionClient(self, MissionTask, "mission_task_server")

    def send_task(self, task_id):
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("mission_task_server is not available")

        goal = MissionTask.Goal()
        goal.task_id = task_id
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f"Task rejected: {task_id}")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result

    def feedback_cb(self, feedback):
        self.get_logger().info(f"phase={feedback.feedback.phase}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionTaskClient()
    task_id = sys.argv[1] if len(sys.argv) > 1 else "task1_bear"
    try:
        result = node.send_task(task_id)
        node.get_logger().info(f"success={result.success} message={result.message}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
