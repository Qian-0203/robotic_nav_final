import rclpy
import time
from rclpy.executors import MultiThreadedExecutor
from yolo_pkg.ros_communicator import RosCommunicator
from yolo_pkg.image_processor import ImageProcessor
from yolo_pkg.yolo_depth_extractor import YoloDepthExtractor
from yolo_pkg.yolo_bounding_box import YoloBoundingBox
from yolo_pkg.boundingbox_visaulizer import BoundingBoxVisualizer
from yolo_pkg.camera_geometry import CameraGeometry
import threading
from std_msgs.msg import String  # Import String message type
from yolo_pkg.load_params import LoadParams


def _init_ros_node(args=None):
    """
    Initialize the ROS 2 node with MultiThreadedExecutor for efficient handling of multiple subscribers.
    """
    rclpy.init(args=args)
    node = RosCommunicator()  # Initialize the ROS node
    executor = MultiThreadedExecutor()  # Use MultiThreadedExecutor
    executor.add_node(node)  # Add the node to the executor
    thread = threading.Thread(
        target=executor.spin
    )  # Start the executor in a separate thread
    thread.start()
    return node, executor, thread  # Return the node, executor, and thread


def menu():
    print("Select mode:")
    print("1: Draw bounding boxes without screenshot.")
    print("2: Draw bounding boxes with screenshot.")
    print("3: 5 fps screenshot.")
    print("4: segmentation.")
    print("Press Ctrl+C to exit.")

    user_input = input("Enter your choice (1/4): ")
    return user_input


def main(args=None):
    """
    Main function to initialize the node and run the bounding box visualizer.
    """
    load_params = LoadParams("yolo_pkg")
    ros_communicator, executor, ros_thread = _init_ros_node(args=args)
    ros_communicator.declare_parameter("mode", "1")
    image_processor = ImageProcessor(ros_communicator, load_params)
    yolo_boundingbox = YoloBoundingBox(image_processor, load_params)
    yolo_depth_extractor = YoloDepthExtractor(
        yolo_boundingbox, image_processor, ros_communicator
    )
    boundingbox_visualizer = BoundingBoxVisualizer(
        image_processor, yolo_boundingbox, ros_communicator
    )
    camera_geometry = CameraGeometry(yolo_depth_extractor)

    configured_mode = str(ros_communicator.get_parameter("mode").value)
    user_input = menu() if configured_mode == "interactive" else configured_mode
    ros_communicator.get_logger().info(f"YOLO detection mode: {user_input}")
    loop_rate = ros_communicator.create_rate(10)
    last_wait_log = 0.0

    try:
        while True:
            has_rgb = ros_communicator.get_latest_data("rgb_compress") is not None
            has_depth = (
                ros_communicator.get_latest_data("depth_image") is not None
                or ros_communicator.get_latest_data("depth_image_compress") is not None
            )
            needs_depth = user_input == "1"
            if not has_rgb or (needs_depth and not has_depth):
                now = time.monotonic()
                if now - last_wait_log > 5.0:
                    missing = []
                    if not has_rgb:
                        missing.append("rgb image")
                    if needs_depth and not has_depth:
                        missing.append("depth image")
                    ros_communicator.get_logger().warn(
                        "Waiting for " + " and ".join(missing)
                    )
                    last_wait_log = now
                loop_rate.sleep()
                continue

            if user_input == "1":
                offsets_3d = camera_geometry.calculate_offset_from_crosshair_2d()
                boundingbox_visualizer.draw_bounding_boxes(
                    draw_crosshair=True,
                    screenshot=False,
                    segmentation_status=False,
                    bounding_status=True,
                    offsets_3d_json=offsets_3d,
                )

                offset_msg = String()
                offset_msg.data = offsets_3d
                ros_communicator.publish_data("object_offset", offset_msg)

            elif user_input == "2":
                boundingbox_visualizer.draw_bounding_boxes(
                    draw_crosshair=True,
                    screenshot=True,
                    segmentation_status=False,
                    bounding_status=True,
                )
            elif user_input == "3":
                boundingbox_visualizer.draw_bounding_boxes(
                    draw_crosshair=True,
                    screenshot=False,
                    segmentation_status=False,
                    bounding_status=True,
                )

                # store 5fps unity camera picture
                boundingbox_visualizer.save_fps_screenshot()

            elif user_input == "4":
                boundingbox_visualizer.draw_bounding_boxes(
                    draw_crosshair=True,
                    screenshot=False,
                    segmentation_status=True,
                    bounding_status=False,
                )
            else:
                print("Invalid input.")

            # Example action for yolo_depth_extractor (can be removed if not needed)
            # depth_data = yolo_depth_extractor.get_yolo_object_depth()
            # print(f"Object Depth: {depth_data}")
            loop_rate.sleep()

    except KeyboardInterrupt:
        print("Shutting down gracefully...")
    finally:
        # Shut down the executor and ROS
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join()


if __name__ == "__main__":
    main()
