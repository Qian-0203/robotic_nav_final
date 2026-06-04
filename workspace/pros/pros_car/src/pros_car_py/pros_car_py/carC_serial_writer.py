from pros_car_py.env import SERIAL_DEV_DEFAULT, SERIAL_DEV_FORWARD_DEFAULT
from pros_car_py.car_models import *
import rclpy
from rclpy.node import Node
import orjson
from std_msgs.msg import Float32MultiArray
from serial import Serial


class CarCControlSubscriber(Node):
    def __init__(self):
        super().__init__("car_c_control_subscriber")
        self.subscription = self.create_subscription(
            Float32MultiArray,
            DeviceDataTypeEnum.car_C_rear_wheel,  # topic name
            self.listener_callback,
            10,
        )
        serial_port = self.declare_parameter("serial_port", SERIAL_DEV_DEFAULT).value

        self.subscription  # prevent unused variable warning
        self._serial = Serial(serial_port, 115200, timeout=0)
        # ------------------------------------------------------------------
        self.subscription_forward = self.create_subscription(
            Float32MultiArray,
            DeviceDataTypeEnum.car_C_front_wheel,  # topic name
            self.listener_callback_forward,
            10,
        )
        serial_port_forward = self.declare_parameter(
            "serial_port_forward", SERIAL_DEV_FORWARD_DEFAULT
        ).value

        self.subscription_forward  # prevent unused variable warning
        self._serial_forward = Serial(serial_port_forward, 115200, timeout=0)

    # -------------------------------------------------------------------
    def listener_callback(self, msg):
        self.process_control_data(msg.data)

    def listener_callback_forward(self, msg):
        self.process_control_data_forward(msg.data)

    def process_control_data(self, target_vel):
        control_signal = CarCControl(target_vel=list(target_vel))

        self._serial.write(
            orjson.dumps(dict(control_signal), option=orjson.OPT_APPEND_NEWLINE)
        )
        self.get_logger().info(f"Received {control_signal}")

    def process_control_data_forward(self, target_vel):
        control_signal_forward = CarCControl(target_vel=list(target_vel))

        self._serial_forward.write(
            orjson.dumps(dict(control_signal_forward), option=orjson.OPT_APPEND_NEWLINE)
        )
        self.get_logger().info(f"Received {control_signal_forward}")


def main(args=None):
    rclpy.init(args=args)
    car_c_control_subscriber = CarCControlSubscriber()
    rclpy.spin(car_c_control_subscriber)
    car_c_control_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
