#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class UnityScanRelay(Node):
    def __init__(self):
        super().__init__("unity_scan_relay")
        self.declare_parameter("input_topic", "/scan_tmp")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("output_frame", "")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.output_frame = self.get_parameter("output_frame").value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(LaserScan, output_topic, qos)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.relay_scan, qos
        )
        self.get_logger().info(f"Relaying {input_topic} to {output_topic}")

    def relay_scan(self, msg):
        if self.output_frame:
            msg.header.frame_id = self.output_frame
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = UnityScanRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
