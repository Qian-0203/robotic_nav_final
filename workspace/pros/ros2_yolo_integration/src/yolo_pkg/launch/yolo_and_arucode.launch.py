from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_arucode = LaunchConfiguration("enable_arucode")
    enable_yolo = LaunchConfiguration("enable_yolo")
    yolo_mode = LaunchConfiguration("yolo_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_arucode",
                default_value="false",
                description="Start the ArUco detector node.",
            ),
            DeclareLaunchArgument(
                "enable_yolo",
                default_value="true",
                description="Start the YOLO detector node.",
            ),
            DeclareLaunchArgument(
                "yolo_mode",
                default_value="1",
                description=(
                    "YOLO mode: 1 boxes/offsets, 2 screenshot, 3 5 FPS screenshots, "
                    "4 segmentation, interactive for terminal prompt."
                ),
            ),
            Node(
                package="arucode_pkg",
                executable="arucode_node",
                name="arucode_node",
                output="screen",
                condition=IfCondition(enable_arucode),
            ),
            Node(
                package="yolo_pkg",
                executable="yolo_detection_node",
                name="yolo_detection_node",
                output="screen",
                parameters=[{"mode": ParameterValue(yolo_mode, value_type=str)}],
                condition=IfCondition(enable_yolo),
            ),
        ]
    )
