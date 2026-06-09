import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class WheelCommandMonitor(Node):
    def __init__(self):
        super().__init__("wheel_command_monitor")
        self.latest_front = None
        self.latest_rear = None
        self.latest_cmd_vel = None
        self.front_updates = 0
        self.rear_updates = 0
        self.cmd_vel_updates = 0

        self.create_subscription(
            Float32MultiArray,
            "/car_C_front_wheel",
            self.front_callback,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/car_C_rear_wheel",
            self.rear_callback,
            10,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )
        self.create_timer(2.0, self.report_status)
        self.get_logger().info(
            "Monitoring /cmd_vel, /car_C_front_wheel, and /car_C_rear_wheel"
        )

    def front_callback(self, msg):
        self.front_updates += 1
        self.latest_front = list(msg.data)
        self.get_logger().info(f"front command: {self.latest_front}")

    def rear_callback(self, msg):
        self.rear_updates += 1
        self.latest_rear = list(msg.data)
        self.get_logger().info(f"rear command: {self.latest_rear}")

    def cmd_vel_callback(self, msg):
        self.cmd_vel_updates += 1
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        expected_left = linear_x - 0.25 * angular_z
        expected_right = linear_x + 0.25 * angular_z
        self.latest_cmd_vel = {
            "linear_x": linear_x,
            "angular_z": angular_z,
            "expected_wheel_pair": [expected_left, expected_right],
        }
        self.get_logger().info(
            "cmd_vel: "
            f"linear_x={linear_x}, "
            f"angular_z={angular_z}, "
            f"expected_wheel_pair={[expected_left, expected_right]}"
        )

    def report_status(self):
        cmd_vel_publishers = self.count_publishers("/cmd_vel")
        front_publishers = self.count_publishers("/car_C_front_wheel")
        rear_publishers = self.count_publishers("/car_C_rear_wheel")
        self.get_logger().info(
            "status: "
            f"cmd_vel_publishers={cmd_vel_publishers}, "
            f"front_publishers={front_publishers}, "
            f"rear_publishers={rear_publishers}, "
            f"cmd_vel_updates={self.cmd_vel_updates}, "
            f"front_updates={self.front_updates}, "
            f"rear_updates={self.rear_updates}, "
            f"latest_cmd_vel={self.latest_cmd_vel}, "
            f"latest_front={self.latest_front}, "
            f"latest_rear={self.latest_rear}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WheelCommandMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
