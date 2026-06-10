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
from rclpy.time import Time
from std_msgs.msg import String  # Import String message type
from tf2_ros import TransformException
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
    ros_communicator.declare_parameter("bridge_tf_parent_frame_id", "map")
    ros_communicator.declare_parameter("bridge_tf_source_frame_id", "base_link")
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
        _set_bridge_tf_active(ros_communicator, False)
        return

    detection = _select_bridge_detection(offsets)
    if detection is None:
        _set_bridge_tf_active(ros_communicator, False)
        return

    offset = _bridge_detection_offset(detection)
    if offset is None:
        _set_bridge_tf_active(ros_communicator, False)
        return

    parent_frame = str(
        ros_communicator.get_parameter("bridge_tf_parent_frame_id").value
    )
    source_frame = str(
        ros_communicator.get_parameter("bridge_tf_source_frame_id").value
    )
    try:
        source_to_parent = ros_communicator.lookup_transform(
            parent_frame,
            source_frame,
            Time(),
        )
    except TransformException as exc:
        _set_bridge_tf_active(ros_communicator, False)
        _log_bridge_tf_lookup_failure(
            ros_communicator,
            parent_frame,
            source_frame,
            exc,
        )
        return

    # Bridge TF uses the corrected detected offset convention: invert local X/Y,
    # keep Z, then express the result in the configured parent frame.
    bridge_offset = [-float(offset[0]), -float(offset[1]), float(offset[2])]
    source_yaw = _yaw_from_quaternion(source_to_parent.transform.rotation)
    bridge_x, bridge_y = _offset_flu_to_xy(
        source_to_parent.transform.translation.x,
        source_to_parent.transform.translation.y,
        source_yaw,
        bridge_offset,
    )

    transform = TransformStamped()
    transform.header.stamp = ros_communicator.get_clock().now().to_msg()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = str(
        ros_communicator.get_parameter("bridge_tf_child_frame_id").value
    )
    transform.transform.translation.x = bridge_x
    transform.transform.translation.y = bridge_y
    transform.transform.translation.z = (
        source_to_parent.transform.translation.z + bridge_offset[2]
    )

    yaw = _bridge_tf_yaw(detection, source_yaw)
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    ros_communicator.publish_transform(transform)
    _set_bridge_tf_active(ros_communicator, True)
    _log_bridge_tf(ros_communicator, transform, yaw, detection)


def _set_bridge_tf_active(ros_communicator, active):
    previous = getattr(_set_bridge_tf_active, "_active", None)
    if previous == active:
        return
    _set_bridge_tf_active._active = active
    child_frame = str(ros_communicator.get_parameter("bridge_tf_child_frame_id").value)
    if active:
        ros_communicator.get_logger().info(f"Publishing bridge TF: {child_frame}")
    else:
        ros_communicator.get_logger().info(
            f"Bridge not detected; stopped updating TF: {child_frame}"
        )


def _log_bridge_tf(ros_communicator, transform, yaw, detection):
    now = time.monotonic()
    last_log = getattr(_log_bridge_tf, "_last_log", 0.0)
    if now - last_log < 1.0:
        return
    _log_bridge_tf._last_log = now

    bridge_model = detection.get("bridge_model", {})
    ground_edge = bridge_model.get("ground_edge_corners_flu")
    ros_communicator.get_logger().info(
        "Bridge TF determined: "
        f"parent={transform.header.frame_id} child={transform.child_frame_id} "
        f"xyz=({transform.transform.translation.x:.3f}, "
        f"{transform.transform.translation.y:.3f}, "
        f"{transform.transform.translation.z:.3f}) "
        f"yaw={yaw:.3f} ground_edge_flu={ground_edge}"
    )


def _log_bridge_tf_lookup_failure(
    ros_communicator,
    parent_frame,
    source_frame,
    exc,
):
    now = time.monotonic()
    last_log = getattr(_log_bridge_tf_lookup_failure, "_last_log", 0.0)
    if now - last_log < 1.0:
        return
    _log_bridge_tf_lookup_failure._last_log = now
    ros_communicator.get_logger().warn(
        f"Cannot publish bridge TF in {parent_frame}: "
        f"missing transform {parent_frame} <- {source_frame}: {exc}"
    )


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


def _bridge_tf_yaw(detection, source_yaw):
    bridge_model = detection.get("bridge_model")
    if not isinstance(bridge_model, dict):
        return _normalize_angle(source_yaw)

    corners = bridge_model.get("ground_edge_corners_flu")
    if not isinstance(corners, list) or len(corners) != 2:
        return _normalize_angle(source_yaw)
    try:
        start = [float(value) for value in corners[0]]
        end = [float(value) for value in corners[1]]
    except (TypeError, ValueError):
        return _normalize_angle(source_yaw)
    if len(start) != 3 or len(end) != 3:
        return _normalize_angle(source_yaw)

    edge_flu = [
        end[0] - start[0],
        end[1] - start[1],
        0.0,
    ]
    edge_x, edge_y = _vector_flu_to_xy(source_yaw, edge_flu)
    if abs(edge_x) < 1e-6 and abs(edge_y) < 1e-6:
        return _normalize_angle(source_yaw)
    # Snap bridge yaw in the parent/map frame: 180 deg for X-like edges,
    # 90 deg for Y-like edges.
    return math.pi if abs(edge_x) >= abs(edge_y) else math.pi / 2.0


def _offset_flu_to_xy(origin_x, origin_y, yaw, offset_flu):
    forward, left, _up = [float(value) for value in offset_flu]
    return (
        origin_x + forward * math.cos(yaw) - left * math.sin(yaw),
        origin_y + forward * math.sin(yaw) + left * math.cos(yaw),
    )


def _vector_flu_to_xy(yaw, vector_flu):
    forward, left, _up = [float(value) for value in vector_flu]
    return (
        forward * math.cos(yaw) - left * math.sin(yaw),
        forward * math.sin(yaw) + left * math.cos(yaw),
    )


def _yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


if __name__ == "__main__":
    main()
