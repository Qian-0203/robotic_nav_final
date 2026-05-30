#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="car_control_pkg",
                executable="car_control_node",
                name="car_control_node",
                output="screen",
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
