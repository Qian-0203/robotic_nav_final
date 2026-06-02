from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_unity_arm_bridge = LaunchConfiguration("enable_unity_arm_bridge")
    waypoints_file = LaunchConfiguration("waypoints_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "stage",
                default_value="mission",
                choices=["mission"],
                description="Mission bringup stage. Partial car/arm stages are disabled.",
            ),
            DeclareLaunchArgument(
                "enable_unity_arm_bridge",
                default_value="true",
                description="Start the Unity arm republisher.",
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
            ),
            Node(
                package="arm_control_pkg",
                executable="arm_control_node",
                name="arm_control_node",
                output="screen",
            ),
            Node(
                package="arm_control_pkg",
                executable="unity_arm_republish_node",
                name="unity_arm_republish_node",
                output="screen",
                condition=IfCondition(enable_unity_arm_bridge),
            ),
            Node(
                package="mission_task_pkg",
                executable="mission_task_node",
                name="mission_task_node",
                output="screen",
                parameters=[{"waypoints_file": waypoints_file}],
            ),
        ]
    )
