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

    marker_id = 1
    for detection in detections or []:
        if not _is_bridge_detection(detection, min_confidence):
            continue

        bridge_model = detection["bridge_model"]
        face_points = _marker_points(bridge_model.get("lateral_face_corners_flu"), 4)
        ground_points = _marker_points(bridge_model.get("ground_edge_corners_flu"), 2)
        if face_points is None or ground_points is None:
            continue

        markers.markers.append(
            _build_plane_marker(marker_id, frame_id, stamp, face_points)
        )
        marker_id += 1
        markers.markers.append(
            _build_outline_marker(marker_id, frame_id, stamp, face_points)
        )
        marker_id += 1
        markers.markers.append(
            _build_ground_edge_marker(marker_id, frame_id, stamp, ground_points)
        )
        marker_id += 1

    return markers


def _is_bridge_detection(detection, min_confidence):
    if not isinstance(detection, dict) or detection.get("label") != "bridge":
        return False
    if not isinstance(detection.get("bridge_model"), dict):
        return False
    try:
        confidence = float(detection.get("confidence", 0.0))
    except (TypeError, ValueError):
        return False
    return confidence >= min_confidence


def _marker_points(corners_flu, expected_count):
    if not isinstance(corners_flu, list) or len(corners_flu) != expected_count:
        return None

    points = []
    for corner in corners_flu:
        if not isinstance(corner, list) or len(corner) != 3:
            return None
        try:
            forward, left, up = [float(value) for value in corner]
        except (TypeError, ValueError):
            return None
        points.append(_point(-forward, -left, up))
    return points


def _base_marker(marker_id, frame_id, stamp, namespace, marker_type):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    return marker


def _build_plane_marker(marker_id, frame_id, stamp, face_points):
    marker = _base_marker(
        marker_id,
        frame_id,
        stamp,
        "bridge_model_plane",
        Marker.TRIANGLE_LIST,
    )
    _color(marker, 0.0, 0.8, 1.0, 0.35)
    top_left, top_right, bottom_right, bottom_left = face_points
    marker.points = [
        top_left,
        top_right,
        bottom_right,
        top_left,
        bottom_right,
        bottom_left,
    ]
    return marker


def _build_outline_marker(marker_id, frame_id, stamp, face_points):
    marker = _base_marker(
        marker_id,
        frame_id,
        stamp,
        "bridge_model",
        Marker.LINE_STRIP,
    )
    marker.scale.x = 0.035
    _color(marker, 0.0, 0.8, 1.0, 1.0)
    marker.points = face_points + [face_points[0]]
    return marker


def _build_ground_edge_marker(marker_id, frame_id, stamp, ground_points):
    marker = _base_marker(
        marker_id,
        frame_id,
        stamp,
        "bridge_ground_edge",
        Marker.LINE_STRIP,
    )
    marker.scale.x = 0.07
    _color(marker, 1.0, 0.15, 0.05, 1.0)
    marker.points = ground_points
    return marker
