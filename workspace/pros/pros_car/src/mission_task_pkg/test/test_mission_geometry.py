import math
import unittest

from mission_task_pkg.mission_geometry import (
    compute_dynamic_goal,
    estimate_target_map_pose,
    select_detection,
    yaw_from_quaternion,
)


class MissionGeometryTest(unittest.TestCase):
    def test_yaw_from_quaternion_half_turn(self):
        yaw = yaw_from_quaternion(
            (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
        )

        self.assertAlmostEqual(yaw, math.pi / 2.0)

    def test_estimate_target_map_pose_uses_forward_and_left_offsets(self):
        target_x, target_y = estimate_target_map_pose(
            robot_x=1.0,
            robot_y=2.0,
            robot_yaw=math.pi / 2.0,
            offset_flu=[2.0, 0.5, 0.0],
        )

        self.assertAlmostEqual(target_x, 0.5)
        self.assertAlmostEqual(target_y, 4.0)

    def test_compute_dynamic_goal_faces_target_at_standoff_distance(self):
        goal = compute_dynamic_goal(
            robot_x=0.0,
            robot_y=0.0,
            robot_quat=(0.0, 0.0, 0.0, 1.0),
            offset_flu=[2.0, 0.0, 0.0],
            standoff_m=0.9,
        )

        self.assertAlmostEqual(goal["target_x"], 2.0)
        self.assertAlmostEqual(goal["target_y"], 0.0)
        self.assertAlmostEqual(goal["goal_x"], 1.1)
        self.assertAlmostEqual(goal["goal_y"], 0.0)
        self.assertAlmostEqual(goal["goal_yaw"], 0.0)

    def test_select_detection_filters_invalid_and_uses_confidence_then_depth(self):
        detections = [
            {"label": "bear", "confidence": 0.2, "offset_flu": [1.0, 0.0, 0.0]},
            {"label": "bear", "confidence": 0.95, "offset_flu": [-1.0, 0.0, 0.0]},
            {"label": "knob", "confidence": 0.99, "offset_flu": [1.0, 0.0, 0.0]},
            {"label": "bear", "confidence": 0.8, "offset_flu": [2.0, 0.0, 0.0]},
            {"label": "bear", "confidence": 0.8, "offset_flu": [1.5, 0.0, 0.0]},
        ]

        selected = select_detection(detections, "bear", min_confidence=0.5)

        self.assertEqual(selected, {"confidence": 0.8, "offset_flu": [1.5, 0.0, 0.0]})

    def test_select_detection_returns_none_when_confidence_is_too_low(self):
        selected = select_detection(
            [{"label": "bear", "confidence": 0.89, "offset_flu": [1.0, 0.0, 0.0]}],
            "bear",
            min_confidence=0.9,
        )

        self.assertIsNone(selected)


if __name__ == "__main__":
    unittest.main()
