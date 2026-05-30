from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def stage_condition(stage, allowed_stages):
    return IfCondition(PythonExpression(["'", stage, "' in ", repr(allowed_stages)]))


def stage_and_flag_condition(stage, allowed_stages, flag):
    return IfCondition(
        PythonExpression(
            [
                "'",
                stage,
                "' in ",
                repr(allowed_stages),
                " and '",
                flag,
                "'.lower() == 'true'",
            ]
        )
    )


def generate_launch_description():
    stage = LaunchConfiguration("stage")
    enable_unity_arm_bridge = LaunchConfiguration("enable_unity_arm_bridge")
    waypoints_file = LaunchConfiguration("waypoints_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "stage",
                default_value="mission",
                choices=["car", "arm", "control", "mission"],
                description=(
                    "Bringup stage: car only, arm only, control stack, or full "
                    "mission orchestration."
                ),
            ),
            DeclareLaunchArgument(
                "enable_unity_arm_bridge",
                default_value="true",
                description="Start the Unity arm republisher with arm stages.",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value="",
                description="Optional path to mission_waypoints.yaml",
            ),
            Node(
                package="car_control_pkg",
                executable="car_control_node",
                name="car_control_node",
                output="screen",
                condition=stage_condition(stage, ["car", "control", "mission"]),
            ),
            Node(
                package="arm_control_pkg",
                executable="arm_control_node",
                name="arm_control_node",
                output="screen",
                condition=stage_condition(stage, ["arm", "control", "mission"]),
            ),
            Node(
                package="arm_control_pkg",
                executable="unity_arm_republish_node",
                name="unity_arm_republish_node",
                output="screen",
                condition=stage_and_flag_condition(
                    stage, ["arm", "control", "mission"], enable_unity_arm_bridge
                ),
            ),
            Node(
                package="mission_task_pkg",
                executable="mission_task_node",
                name="mission_task_node",
                output="screen",
                parameters=[{"waypoints_file": waypoints_file}],
                condition=stage_condition(stage, ["mission"]),
            ),
        ]
    )
