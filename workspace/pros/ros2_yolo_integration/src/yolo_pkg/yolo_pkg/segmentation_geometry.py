import math

import numpy as np


def compute_mask_geometry(mask):
    points_yx = np.argwhere(mask > 0)
    area = int(points_yx.shape[0])
    if area == 0:
        return None

    height, width = mask.shape[:2]
    min_y = int(points_yx[:, 0].min())
    max_y = int(points_yx[:, 0].max())
    min_x = int(points_yx[:, 1].min())
    max_x = int(points_yx[:, 1].max())
    touching_edges = []
    if min_x <= 0:
        touching_edges.append("left")
    if max_x >= width - 1:
        touching_edges.append("right")
    if min_y <= 0:
        touching_edges.append("top")
    if max_y >= height - 1:
        touching_edges.append("bottom")

    center_y, center_x = points_yx.mean(axis=0)
    angle = 0.0
    if area >= 2:
        points_xy = np.column_stack((points_yx[:, 1], points_yx[:, 0])).astype(float)
        centered = points_xy - points_xy.mean(axis=0)
        covariance = np.cov(centered, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        principal = eigenvectors[:, int(np.argmax(eigenvalues))]
        angle = math.atan2(float(principal[1]), float(principal[0]))
        while angle > math.pi / 2.0:
            angle -= math.pi
        while angle < -math.pi / 2.0:
            angle += math.pi

    return {
        "mask_center_px": [round(float(center_x), 3), round(float(center_y), 3)],
        "mask_area_px": area,
        "mask_angle_rad": round(float(angle), 3),
        "mask_bounds_px": [min_x, min_y, max_x, max_y],
        "mask_image_size_px": [int(width), int(height)],
        "mask_touching_edges": touching_edges,
    }


def sample_mask_depth(depth_image, mask):
    if depth_image is None or not isinstance(depth_image, np.ndarray):
        return None
    if depth_image.shape[:2] != mask.shape[:2]:
        return None
    values = depth_image[mask > 0]
    valid = values[(values > 0) & (~np.isnan(values))]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def _round_point(point):
    return [round(float(point[0]), 3), round(float(point[1]), 3)]


def _mask_band(mask, points_yx, band_name):
    ys = points_yx[:, 0]
    height_px = int(ys.max() - ys.min() + 1)
    band_px = max(2, int(round(height_px * 0.08)))
    if band_name == "lower":
        threshold = float(np.percentile(ys, 90))
        return mask & (np.indices(mask.shape)[0] >= threshold - band_px)
    if band_name == "upper":
        threshold = float(np.percentile(ys, 10))
        return mask & (np.indices(mask.shape)[0] <= threshold + band_px)
    raise ValueError(f"unknown mask band: {band_name}")


def sample_mask_band_depth(depth_image, mask, points_yx, band_name):
    if depth_image is None or not isinstance(depth_image, np.ndarray):
        return None
    if depth_image.shape[:2] != mask.shape[:2]:
        return None
    band = _mask_band(mask, points_yx, band_name)
    values = depth_image[band > 0]
    valid = values[(values > 0) & (~np.isnan(values))]
    if valid.size == 0:
        return sample_mask_depth(depth_image, mask)
    return float(np.median(valid))


def sample_point_depth(depth_image, center_px, radius_px=2, fallback_depth=None):
    if depth_image is None or not isinstance(depth_image, np.ndarray):
        return fallback_depth
    center_x, center_y = center_px
    height, width = depth_image.shape[:2]
    x = int(round(center_x))
    y = int(round(center_y))
    radius_px = max(0, int(radius_px))
    x_min = max(0, x - radius_px)
    x_max = min(width, x + radius_px + 1)
    y_min = max(0, y - radius_px)
    y_max = min(height, y + radius_px + 1)
    if x_min >= x_max or y_min >= y_max:
        return fallback_depth

    values = depth_image[y_min:y_max, x_min:x_max]
    valid = values[(values > 0) & (~np.isnan(values))]
    if valid.size == 0:
        return fallback_depth
    return float(np.median(valid))


def estimate_mask_quadrilateral(points_yx):
    points_xy = np.column_stack((points_yx[:, 1], points_yx[:, 0])).astype(float)
    sums = points_xy[:, 0] + points_xy[:, 1]
    diffs = points_xy[:, 0] - points_xy[:, 1]

    top_left = points_xy[int(np.argmin(sums))]
    top_right = points_xy[int(np.argmax(diffs))]
    bottom_right = points_xy[int(np.argmax(sums))]
    bottom_left = points_xy[int(np.argmin(diffs))]
    return [
        top_left.tolist(),
        top_right.tolist(),
        bottom_right.tolist(),
        bottom_left.tolist(),
    ]


def _edge_with_lowest_z(corner_offsets):
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    return min(
        edges,
        key=lambda edge: _average_offset(
            corner_offsets[edge[0]],
            corner_offsets[edge[1]],
        )[2],
    )


def _bridge_tf_axes_from_ground_edge(edge_offsets):
    start, end = edge_offsets
    start_x, start_y = -float(start[0]), -float(start[1])
    end_x, end_y = -float(end[0]), -float(end[1])
    dx = end_x - start_x
    dy = end_y - start_y

    # Marker frame is x=-forward, y=-left, z=up. Default bridge TF +X points
    # to marker [1, 0, 0] (yaw 0). Switch to [0, 1, 0] only when the
    # lowest edge is more parallel to marker/world Y.
    if abs(dx) >= abs(dy):
        yaw = 0.0
        x_axis_marker = [1.0, 0.0, 0.0]
        y_axis_marker = [0.0, 1.0, 0.0]
        axis_name = "x"
    else:
        yaw = math.pi / 2.0
        x_axis_marker = [0.0, 1.0, 0.0]
        y_axis_marker = [-1.0, 0.0, 0.0]
        axis_name = "y"

    return {
        "axis_name": axis_name,
        "yaw_rad": yaw,
        "x_axis_flu": [
            -x_axis_marker[0],
            -x_axis_marker[1],
            x_axis_marker[2],
        ],
        "y_axis_flu": [
            -y_axis_marker[0],
            -y_axis_marker[1],
            y_axis_marker[2],
        ],
    }


def _average_offset(offset_a, offset_b):
    return [
        round((float(offset_a[0]) + float(offset_b[0])) / 2.0, 3),
        round((float(offset_a[1]) + float(offset_b[1])) / 2.0, 3),
        round((float(offset_a[2]) + float(offset_b[2])) / 2.0, 3),
    ]


def _average_point(point_a, point_b):
    return [
        float((point_a[0] + point_b[0]) / 2.0),
        float((point_a[1] + point_b[1]) / 2.0),
    ]


def calculate_offset_flu(center_px, depth, intrinsics):
    center_x, center_y = center_px
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])

    x_cam = (center_x - cx) * depth / fx
    y_cam = (center_y - cy) * depth / fy
    z_cam = depth
    return [round(float(z_cam), 3), round(float(-x_cam), 3), round(float(-y_cam), 3)]


def estimate_bridge_model(mask, depth_image, intrinsics):
    points_yx = np.argwhere(mask > 0)
    if points_yx.size == 0:
        return None

    corners_px = estimate_mask_quadrilateral(points_yx)
    ground_depth = sample_mask_band_depth(depth_image, mask, points_yx, "lower")
    if ground_depth is None or ground_depth <= 0.0:
        return None
    upper_depth = sample_mask_band_depth(depth_image, mask, points_yx, "upper")

    corner_fallback_depths = [
        upper_depth or ground_depth,
        upper_depth or ground_depth,
        ground_depth,
        ground_depth,
    ]
    corner_depths = [
        sample_point_depth(
            depth_image,
            corner_px,
            fallback_depth=fallback_depth,
        )
        for corner_px, fallback_depth in zip(corners_px, corner_fallback_depths)
    ]
    corner_offsets = [
        calculate_offset_flu(corner_px, depth, intrinsics)
        for corner_px, depth in zip(corners_px, corner_depths)
        if depth is not None and depth > 0.0
    ]
    if len(corner_offsets) != 4:
        return None

    ground_edge = _edge_with_lowest_z(corner_offsets)
    ground_contact_center_px = _average_point(
        corners_px[ground_edge[0]],
        corners_px[ground_edge[1]],
    )
    ground_contact_offset = _average_offset(
        corner_offsets[ground_edge[0]],
        corner_offsets[ground_edge[1]],
    )
    ground_edge_offsets = [
        corner_offsets[ground_edge[0]],
        corner_offsets[ground_edge[1]],
    ]
    bridge_tf = _bridge_tf_axes_from_ground_edge(ground_edge_offsets)
    ground_depth = (
        corner_depths[ground_edge[0]] + corner_depths[ground_edge[1]]
    ) / 2.0

    return {
        "type": "quadrilateral",
        "visible_face": "mask_outline",
        "lateral_face_corners_px": [_round_point(corner) for corner in corners_px],
        "lateral_face_corner_depths_m": [
            round(float(depth), 3) for depth in corner_depths
        ],
        "ground_edge_corners_px": [
            _round_point(corners_px[ground_edge[0]]),
            _round_point(corners_px[ground_edge[1]]),
        ],
        "ground_contact_center_px": _round_point(ground_contact_center_px),
        "ground_contact_depth_m": round(float(ground_depth), 3),
        "ground_contact_offset_flu": ground_contact_offset,
        "bridge_tf_origin_flu": ground_contact_offset,
        "bridge_tf_x_axis_flu": bridge_tf["x_axis_flu"],
        "bridge_tf_y_axis_flu": bridge_tf["y_axis_flu"],
        "bridge_tf_z_axis_flu": [0.0, 0.0, 1.0],
        "bridge_tf_axis_name": bridge_tf["axis_name"],
        "bridge_tf_yaw_rad": round(float(bridge_tf["yaw_rad"]), 3),
        "lateral_face_corners_flu": corner_offsets,
        "ground_edge_corners_flu": ground_edge_offsets,
    }


def build_segmentation_offset(segmentation_obj, depth_image, intrinsics):
    mask = segmentation_obj.get("mask")
    if mask is None:
        return None

    geometry = compute_mask_geometry(mask)
    if geometry is None:
        return None

    bridge_model = None
    if str(segmentation_obj.get("label", "")).lower() == "bridge":
        bridge_model = estimate_bridge_model(mask, depth_image, intrinsics)

    depth = sample_mask_depth(depth_image, mask)
    if depth is None or depth <= 0.0:
        return None

    offset_flu = calculate_offset_flu(
        geometry["mask_center_px"],
        depth,
        intrinsics,
    )
    if bridge_model is not None:
        offset_flu = bridge_model["ground_contact_offset_flu"]

    result = {
        "label": segmentation_obj["label"],
        "confidence": round(float(segmentation_obj.get("confidence", 0.0)), 3),
        "offset_flu": offset_flu,
        "box": [int(v) for v in segmentation_obj.get("box", [])],
    }
    result.update(geometry)
    if bridge_model is not None:
        result["mask_center_offset_flu"] = calculate_offset_flu(
            geometry["mask_center_px"],
            depth,
            intrinsics,
        )
        result["bridge_model"] = bridge_model
    return result


def build_segmentation_offsets(segmentation_objects, depth_image, intrinsics):
    offsets = []
    for obj in segmentation_objects or []:
        offset = build_segmentation_offset(obj, depth_image, intrinsics)
        if offset is not None:
            offsets.append(offset)
    return offsets
