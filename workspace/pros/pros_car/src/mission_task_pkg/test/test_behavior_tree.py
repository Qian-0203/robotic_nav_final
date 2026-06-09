import os
import unittest

import yaml

from mission_task_pkg.behavior_tree import BehaviorTreeError, BehaviorTreeRunner


class BehaviorTreeRunnerTest(unittest.TestCase):
    def test_sequence_executes_primitives_in_order(self):
        calls = []
        tree = {
            "task": {
                "type": "Sequence",
                "children": [
                    {"type": "SetTargetLabel", "label": "bridge"},
                    {"type": "TimedDrive", "phase": "drive"},
                ],
            }
        }

        runner = BehaviorTreeRunner(
            tree,
            lambda node_type, node, _goal_handle: calls.append(
                (node_type, node.get("label"), node.get("phase"))
            ),
        )

        runner.run("task", None)

        self.assertEqual(
            calls,
            [
                ("SetTargetLabel", "bridge", None),
                ("TimedDrive", None, "drive"),
            ],
        )

    def test_unknown_task_raises(self):
        runner = BehaviorTreeRunner({}, lambda *_args: None)

        with self.assertRaises(BehaviorTreeError):
            runner.run("missing", None)

    def test_task2_tree_contains_bridge_then_bear_flow(self):
        trees = _load_mission_trees()
        runner = BehaviorTreeRunner(trees, lambda *_args: None)

        types = runner.flatten_types("task2_bridge")
        phases = [
            child.get("phase")
            for child in trees["task2_bridge"]["children"]
            if child.get("phase")
        ]

        self.assertIn("ApproachBridgePreAlign", types)
        self.assertIn("TimedDrive", types)
        self.assertIn("ReturnToStart", types)
        self.assertLess(types.index("WaitDetection"), types.index("ApproachBridgePreAlign"))
        self.assertLess(
            phases.index("approach_bridge_y_offset"),
            phases.index("approach_bridge_tf_point"),
        )
        self.assertLess(
            phases.index("approach_bridge_tf_point"),
            phases.index("align_bridge_center"),
        )
        self.assertLess(types.index("VisualServo"), types.index("ArmGoal"))

    def test_task3_tree_uses_fixed_waypoints_before_knob_servo(self):
        trees = _load_mission_trees()
        children = trees["task3_knob"]["children"]

        self.assertEqual(children[1]["type"], "NavigateWaypoint")
        self.assertEqual(children[1]["waypoint"], "knob_stage_1")
        self.assertEqual(children[2]["type"], "NavigateWaypoint")
        self.assertEqual(children[2]["waypoint"], "knob_stage_2")
        self.assertEqual(children[4]["type"], "VisualServo")
        self.assertEqual(children[5]["mode"], "grasp_knob")

    def test_task1_bear_fallback_and_detection_timeout(self):
        config = _load_mission_waypoints()

        bear_waypoint = config["waypoints"]["bear_approach"]
        self.assertEqual(bear_waypoint["x"], 0.25)
        self.assertEqual(bear_waypoint["y"], 0.25)

        bear_short_hop = config["dynamic_approach"]["short_hop"]["bear"]
        self.assertEqual(bear_short_hop["detection_refresh_timeout_sec"], 5.0)

        bridge_settle = config["dynamic_approach"]["amcl_settle_before_detection_sec"]
        self.assertEqual(bridge_settle["bridge"], 1.0)


def _load_mission_trees():
    here = os.path.dirname(__file__)
    config_path = os.path.join(here, "..", "config", "mission_trees.yaml")
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _load_mission_waypoints():
    here = os.path.dirname(__file__)
    config_path = os.path.join(here, "..", "config", "mission_waypoints.yaml")
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


if __name__ == "__main__":
    unittest.main()
