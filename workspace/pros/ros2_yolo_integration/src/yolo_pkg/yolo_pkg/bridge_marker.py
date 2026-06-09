import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def _point(x, y, z):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def _color(marker, red, green, blue, alpha):
    marker.color.r = float(red)
    marker.color.g = float(green)
    marker.color.b = float(blue)
    marker.color.a = float(alpha)


def _delete_all_marker(frame_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.action = Marker.DELETEALL
    return marker


def build_bridge_marker_array(
    detections,
    stamp,
    frame_id="base_link",
    min_confidence=0.0,
):
    markers = MarkerArray()
    markers.markers.append(_delete_all_marker(frame_id, stamp))

    bridge_detections = [
        item
        for item in detections or []
        if isinstance(item, dict)
        and item.get("label") == "bridge"
        and isinstance(item.get("bridge_model"), dict)
    ]
    if not bridge_detections:
        return markers

    marker_id = 1
    for detection in bridge_detections:
        confidence = float(detection.get("confidence", 0.0))
        if confidence < min_confidence:
            continue

        bridge_model = detection["bridge_model"]
        offset = bridge_model.get("ground_contact_offset_flu") or detection.get(
            "offset_flu"
        )
        if not isinstance(offset, list) or len(offset) != 3:
            continue

        forward, left, up = [float(value) for value in offset]
        if forward <= 0.0:
            continue

        depth = float(bridge_model.get("ground_contact_depth_m", forward))
        top_width = _pixel_width_to_m(
            bridge_model.get("top_width_px"),
            depth,
            detection,
            default_width=0.25,
        )
        bottom_width = _pixel_width_to_m(
            bridge_model.get("bottom_width_px"),
            depth,
            detection,
            default_width=max(top_width, 0.45),
        )
        height = _pixel_height_to_m(
            bridge_model.get("height_px"),
            depth,
            detection,
            default_height=0.25,
        )
        angle = float(detection.get("mask_angle_rad", 0.0))

        line_marker = _build_lateral_face_marker(
            marker_id,
            frame_id,
            stamp,
            forward,
            left,
            up,
            top_width,
            bottom_width,
            height,
            angle,
        )
        markers.markers.append(line_marker)
        marker_id += 1

        arrow_marker = _build_orientation_arrow(
            marker_id,
            frame_id,
            stamp,
            forward,
            left,
            up,
            bottom_width,
            angle,
        )
        markers.markers.append(arrow_marker)
        marker_id += 1

        text_marker = _build_text_marker(
            marker_id,
            frame_id,
            stamp,
            forward,
            left,
            up + height + 0.08,
            angle,
            confidence,
        )
        markers.markers.append(text_marker)
        marker_id += 1

    return markers


def _pixel_width_to_m(width_px, depth, detection, default_width):
    try:
        width_px = float(width_px)
        box = detection.get("box") or []
        box_width_px = max(float(box[2] - box[0]), 1.0)
    except (TypeError, ValueError, IndexError):
        return float(default_width)

    # Use a conservative fallback scale when camera intrinsics are not carried in JSON.
    # The marker is for visualization; navigation still uses the measured FLU offset.
    normalized_width = max(width_px / box_width_px, 0.1)
    return max(0.1, min(1.5, normalized_width * depth * 0.35))


def _pixel_height_to_m(height_px, depth, detection, default_height):
    try:
        height_px = float(height_px)
        box = detection.get("box") or []
        box_height_px = max(float(box[3] - box[1]), 1.0)
    except (TypeError, ValueError, IndexError):
        return float(default_height)

    normalized_height = max(height_px / box_height_px, 0.1)
    return max(0.1, min(1.2, normalized_height * depth * 0.3))


def _rotate_lateral_point(center_forward, center_left, local_forward, local_left, angle):
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    forward = center_forward + local_forward * cos_angle - local_left * sin_angle
    left = center_left + local_forward * sin_angle + local_left * cos_angle
    return forward, left


def _build_lateral_face_marker(
    marker_id,
    frame_id,
    stamp,
    forward,
    left,
    ground_z,
    top_width,
    bottom_width,
    height,
    angle,
):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "bridge_model"
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.035
    _color(marker, 0.0, 0.8, 1.0, 1.0)

    bottom_left = _rotate_lateral_point(forward, left, 0.0, bottom_width / 2.0, angle)
    bottom_right = _rotate_lateral_point(forward, left, 0.0, -bottom_width / 2.0, angle)
    top_right = _rotate_lateral_point(forward, left, 0.0, -top_width / 2.0, angle)
    top_left = _rotate_lateral_point(forward, left, 0.0, top_width / 2.0, angle)

    marker.points = [
        _point(bottom_left[0], bottom_left[1], ground_z),
        _point(bottom_right[0], bottom_right[1], ground_z),
        _point(top_right[0], top_right[1], ground_z + height),
        _point(top_left[0], top_left[1], ground_z + height),
        _point(bottom_left[0], bottom_left[1], ground_z),
    ]
    return marker


def _build_orientation_arrow(
    marker_id,
    frame_id,
    stamp,
    forward,
    left,
    ground_z,
    bottom_width,
    angle,
):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "bridge_orientation"
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.04
    marker.scale.y = 0.09
    marker.scale.z = 0.12
    _color(marker, 1.0, 0.75, 0.05, 1.0)

    arrow_length = max(0.35, min(1.2, bottom_width))
    start = _rotate_lateral_point(forward, left, -arrow_length / 2.0, 0.0, angle)
    end = _rotate_lateral_point(forward, left, arrow_length / 2.0, 0.0, angle)
    marker.points = [
        _point(start[0], start[1], ground_z + 0.05),
        _point(end[0], end[1], ground_z + 0.05),
    ]
    return marker


def _build_text_marker(marker_id, frame_id, stamp, forward, left, up, angle, confidence):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "bridge_label"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position = _point(forward, left, up)
    marker.scale.z = 0.16
    marker.text = f"bridge yaw approx {angle:.2f} rad conf {confidence:.2f}"
    _color(marker, 1.0, 1.0, 1.0, 1.0)
    return marker
