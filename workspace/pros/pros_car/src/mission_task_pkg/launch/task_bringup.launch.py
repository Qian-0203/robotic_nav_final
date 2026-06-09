from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_unity_arm_bridge = LaunchConfiguration("enable_unity_arm_bridge")
    enable_car_c_writer = LaunchConfiguration("enable_car_c_writer")
    enable_cmd_vel_wheel_control = LaunchConfiguration("enable_cmd_vel_wheel_control")
    waypoints_file = LaunchConfiguration("waypoints_file")
    mission_trees_file = LaunchConfiguration("mission_trees_file")

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
                "enable_car_c_writer",
                default_value="false",
                description="Start the wheel serial writer for the real car.",
            ),
            DeclareLaunchArgument(
                "enable_cmd_vel_wheel_control",
                default_value="false",
                description="Allow car_control_node to convert /cmd_vel into wheel commands.",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value="",
                description="Optional path to mission_waypoints.yaml",
            ),
            DeclareLaunchArgument(
                "mission_trees_file",
                default_value="",
                description="Optional path to mission_trees.yaml",
            ),
            Node(
                package="car_control_pkg",
                executable="car_control_node",
                output="screen",
                parameters=[
                    {
                        "enable_cmd_vel_wheel_control": enable_cmd_vel_wheel_control,
                    }
                ],
            ),
            Node(
                package="pros_car_py",
                executable="carC_writer",
                name="car_c_control_subscriber",
                output="screen",
                condition=IfCondition(enable_car_c_writer),
            ),
            Node(
                package="arm_control_pkg",
                executable="arm_control_node",
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
                parameters=[
                    {
                        "waypoints_file": waypoints_file,
                        "mission_trees_file": mission_trees_file,
                    }
                ],
            ),
        ]
    )
