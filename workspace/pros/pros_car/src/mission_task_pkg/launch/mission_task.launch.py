from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoints_file = LaunchConfiguration("waypoints_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoints_file",
                default_value="",
                description="Optional path to mission_waypoints.yaml",
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
