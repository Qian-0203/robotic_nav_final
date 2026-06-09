import rclpy
import time
import math
from rclpy.executors import MultiThreadedExecutor
from yolo_pkg.ros_communicator import RosCommunicator
from yolo_pkg.image_processor import ImageProcessor
from yolo_pkg.yolo_depth_extractor import YoloDepthExtractor
from yolo_pkg.yolo_bounding_box import YoloBoundingBox
from yolo_pkg.boundingbox_visaulizer import BoundingBoxVisualizer
from yolo_pkg.camera_geometry import CameraGeometry
import threading
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String  # Import String message type
from yolo_pkg.load_params import LoadParams
import json
from yolo_pkg.bridge_marker import build_bridge_marker_array


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
    ros_communicator.declare_parameter("bridge_marker_frame_id", "base_link")
    ros_communicator.declare_parameter("bridge_tf_child_frame_id", "bridge_detected")
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
            needs_depth = user_input in {"1", "4"}
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
                target_label = image_processor.get_yolo_target_label()
                use_segmentation = target_label in (None, "None", "bridge")
                if use_segmentation:
                    offsets_3d = (
                        camera_geometry.calculate_segmentation_offset_from_crosshair_2d()
                    )
                else:
                    offsets_3d = camera_geometry.calculate_offset_from_crosshair_2d()
                boundingbox_visualizer.draw_bounding_boxes(
                    draw_crosshair=True,
                    screenshot=False,
                    segmentation_status=use_segmentation,
                    bounding_status=not use_segmentation,
                    offsets_3d_json=offsets_3d,
                )
                offset_msg = String()
                offset_msg.data = offsets_3d
                ros_communicator.publish_data("object_offset", offset_msg)
                _publish_bridge_marker(ros_communicator, offsets_3d)
                _publish_bridge_tf(ros_communicator, offsets_3d)
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


def _publish_bridge_marker(ros_communicator, offsets_3d_json):
    try:
        offsets = json.loads(offsets_3d_json)
    except (TypeError, json.JSONDecodeError):
        return

    frame_id = str(ros_communicator.get_parameter("bridge_marker_frame_id").value)
    marker_array = build_bridge_marker_array(
        offsets,
        ros_communicator.get_clock().now().to_msg(),
        frame_id=frame_id,
    )
    ros_communicator.publish_data("bridge_marker", marker_array)


def _publish_bridge_tf(ros_communicator, offsets_3d_json):
    try:
        offsets = json.loads(offsets_3d_json)
    except (TypeError, json.JSONDecodeError):
        return

    detection = _select_bridge_detection(offsets)
    if detection is None:
        return

    offset = _bridge_detection_offset(detection)
    if offset is None:
        return

    transform = TransformStamped()
    transform.header.stamp = ros_communicator.get_clock().now().to_msg()
    transform.header.frame_id = str(
        ros_communicator.get_parameter("bridge_marker_frame_id").value
    )
    transform.child_frame_id = str(
        ros_communicator.get_parameter("bridge_tf_child_frame_id").value
    )
    transform.transform.translation.x = -float(offset[0])
    transform.transform.translation.y = -float(offset[1])
    transform.transform.translation.z = float(offset[2])

    yaw = float(detection.get("mask_angle_rad", 0.0))
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    ros_communicator.publish_transform(transform)


def _select_bridge_detection(offsets):
    best = None
    for detection in offsets or []:
        if not isinstance(detection, dict) or detection.get("label") != "bridge":
            continue
        offset = _bridge_detection_offset(detection)
        if offset is None:
            continue
        try:
            forward = float(offset[0])
            confidence = float(detection.get("confidence", 0.0))
        except (TypeError, ValueError):
            continue
        if forward <= 0.0:
            continue
        if best is None:
            best = detection
            continue
        best_offset = _bridge_detection_offset(best)
        best_forward = float(best_offset[0])
        best_confidence = float(best.get("confidence", 0.0))
        if forward < best_forward or (
            forward == best_forward and confidence > best_confidence
        ):
            best = detection
    return best


def _bridge_detection_offset(detection):
    bridge_model = detection.get("bridge_model")
    if isinstance(bridge_model, dict):
        offset = bridge_model.get("ground_contact_offset_flu")
        if isinstance(offset, list) and len(offset) == 3:
            return offset
    offset = detection.get("offset_flu")
    if isinstance(offset, list) and len(offset) == 3:
        return offset
    return None


if __name__ == "__main__":
    main()
