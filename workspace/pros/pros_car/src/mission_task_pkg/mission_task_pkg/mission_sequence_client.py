import sys
import time

import rclpy
from action_interface.action import MissionTask
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_TASK_IDS = ["task1_bear", "task2_bridge_no_init", "task3_knob_no_init"]
SEQUENCE_MODES = {
    "mode1": DEFAULT_TASK_IDS,
    "mode2": ["task2_bridge_direct", "task3_knob_no_init"],
}
HEADING_ADJUST_ACTION = "COUNTERCLOCKWISE_ROTATION"
HEADING_ADJUST_SEC = 8
CONTROL_PERIOD_SEC = 0.2


class MissionSequenceClient(Node):
    def __init__(self):
        super().__init__("mission_sequence_client")
        self.client = ActionClient(self, MissionTask, "mission_task_server")
        self.control_signal_pub = self.create_publisher(
            String, "car_control_signal", 10
        )

    def run_sequence(self, task_ids, skip_first_heading_adjust=False):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("mission_task_server is not available")

        for index, task_id in enumerate(task_ids, start=1):
            if self._needs_heading_adjust(task_id, index, skip_first_heading_adjust):
                self._adjust_heading_before_task(index, task_id)
            self.get_logger().info(
                f"Starting mission {index}/{len(task_ids)}: {task_id}"
            )
            result = self._send_task(task_id)
            self.get_logger().info(
                f"Finished {task_id}: success={result.success} message={result.message}"
            )
            if not result.success:
                raise RuntimeError(f"{task_id} failed: {result.message}")

    def _send_task(self, task_id):
        goal = MissionTask.Goal()
        goal.task_id = task_id
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if goal_handle is None:
            raise RuntimeError(f"No goal response for task: {task_id}")
        if not goal_handle.accepted:
            raise RuntimeError(f"Task rejected: {task_id}")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result

    def _adjust_heading_before_task(self, index, task_id):
        self.get_logger().info(
            f"Adjusting heading before mission {index} ({task_id}): "
            f"{HEADING_ADJUST_ACTION} for {HEADING_ADJUST_SEC:.1f}s"
        )
        try:
            deadline = time.monotonic() + HEADING_ADJUST_SEC
            while time.monotonic() < deadline:
                self._publish_control(HEADING_ADJUST_ACTION)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(CONTROL_PERIOD_SEC)
        finally:
            self._publish_control("STOP")
            time.sleep(CONTROL_PERIOD_SEC)

    @staticmethod
    def _needs_heading_adjust(task_id, index, skip_first_heading_adjust):
        if skip_first_heading_adjust and index == 1:
            return False
        return task_id.startswith(("task2", "task3"))

    def _publish_control(self, action):
        msg = String()
        msg.data = f"Mission_Control:{action}"
        self.control_signal_pub.publish(msg)

    def feedback_cb(self, feedback):
        self.get_logger().info(f"phase={feedback.feedback.phase}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionSequenceClient()
    cli_args = sys.argv[1:]
    skip_first_heading_adjust = False
    if not cli_args:
        task_ids = DEFAULT_TASK_IDS
    elif len(cli_args) == 1 and cli_args[0] in SEQUENCE_MODES:
        task_ids = SEQUENCE_MODES[cli_args[0]]
        skip_first_heading_adjust = cli_args[0] == "mode2"
    else:
        task_ids = cli_args
    try:
        node.run_sequence(task_ids, skip_first_heading_adjust)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
