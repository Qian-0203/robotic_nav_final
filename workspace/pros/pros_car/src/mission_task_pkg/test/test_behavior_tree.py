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
            phases.index("bridge_yaw_180_ccw_turn"),
        )
        self.assertLess(
            phases.index("bridge_yaw_180_ccw_turn"),
            phases.index("align_bridge_center"),
        )
        self.assertLess(types.index("VisualServo"), types.index("ArmGoal"))
        bridge_center_servo = next(
            child
            for child in trees["task2_bridge"]["children"]
            if child.get("phase") == "align_bridge_center"
        )
        self.assertEqual(bridge_center_servo["type"], "VisualServo")
        self.assertEqual(bridge_center_servo["label"], "bridge")
        self.assertIn("lateral_tolerance_m", bridge_center_servo)
        self.assertIn("target_depth_m", bridge_center_servo)

        bear_center_servo = next(
            child
            for child in trees["task2_bridge"]["children"]
            if child.get("phase") == "align_bear_center_before_climb"
        )
        self.assertEqual(bear_center_servo["type"], "VisualServo")
        self.assertEqual(bear_center_servo["label"], "bear")
        self.assertIn("target_depth_m", bear_center_servo)
        self.assertIn("bear_candidate_min_confidence", bear_center_servo)
        self.assertIn("control_speed", bear_center_servo)

    def test_task3_tree_uses_fixed_waypoints_before_knob_servo(self):
        trees = _load_mission_trees()
        children = trees["task3_knob"]["children"]

        self.assertEqual(children[1]["type"], "NavigateWaypoint")
        self.assertEqual(children[1]["waypoint"], "knob_stage_1")
        self.assertEqual(children[2]["type"], "NavigateWaypoint")
        self.assertEqual(children[2]["waypoint"], "knob_stage_2")
        self.assertEqual(children[3]["type"], "NavigateWaypoint")
        self.assertEqual(children[3]["waypoint"], "knob_stage_3")
        self.assertEqual(children[5]["type"], "VisualServo")
        self.assertEqual(children[5]["phase"], "align_knob_for_grasp")
        self.assertEqual(children[5]["target_depth_m"], 0.35)
        self.assertLess(
            _first_child_index(children, "VisualServo"),
            _first_child_index(children, "ArmGoal"),
        )
        self.assertEqual(children[6]["mode"], "grasp_knob")

    def test_task1_bear_fallback_and_detection_timeout(self):
        config = _load_mission_waypoints()

        bear_waypoint = config["waypoints"]["bear_approach"]
        self.assertIn("x", bear_waypoint)
        self.assertIn("y", bear_waypoint)
        self.assertIn("yaw", bear_waypoint)

        bear_short_hop = config["dynamic_approach"]["short_hop"]["bear"]
        self.assertIn("detection_refresh_timeout_sec", bear_short_hop)

        bridge_settle = config["dynamic_approach"]["amcl_settle_before_detection_sec"]
        self.assertIn("bridge", bridge_settle)

    def test_mission_tree_references_match_waypoint_yaml(self):
        trees = _load_mission_trees()
        config = _load_mission_waypoints()
        waypoints = config["waypoints"]
        servo_cfg = config["visual_servo"]

        for node in _walk_tree_nodes(trees):
            if node.get("type") in {"NavigateWaypoint", "ApproachDetectedTarget"}:
                waypoint = node.get("waypoint", node.get("fallback_waypoint"))
                self.assertIn(waypoint, waypoints)
            if node.get("type") in {"TimedDrive", "ConditionalBridgeYawTurn"}:
                self.assertIn(node["config_key"], servo_cfg)
                if "config_key_90" in node:
                    self.assertIn(node["config_key_90"], servo_cfg)


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


def _walk_tree_nodes(trees):
    for root in trees.values():
        yield from _walk_node(root)


def _first_child_index(children, child_type):
    return next(
        index for index, child in enumerate(children) if child.get("type") == child_type
    )


def _walk_node(node):
    yield node
    child = node.get("child")
    if child:
        yield from _walk_node(child)
    for child in node.get("children", []):
        yield from _walk_node(child)


if __name__ == "__main__":
    unittest.main()
