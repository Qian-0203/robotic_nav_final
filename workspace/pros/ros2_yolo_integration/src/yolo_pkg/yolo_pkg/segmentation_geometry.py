import math

import numpy as np


def compute_mask_geometry(mask):
    points_yx = np.argwhere(mask > 0)
    area = int(points_yx.shape[0])
    if area == 0:
        return None

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

    ys = points_yx[:, 0]
    height_px = int(ys.max() - ys.min() + 1)
    band_px = max(2, int(round(height_px * 0.08)))

    top_y = float(np.percentile(ys, 10))
    bottom_y = float(np.percentile(ys, 90))
    top_points = points_yx[ys <= top_y + band_px]
    bottom_points = points_yx[ys >= bottom_y - band_px]
    if top_points.size == 0 or bottom_points.size == 0:
        return None

    top_left = [float(top_points[:, 1].min()), float(np.median(top_points[:, 0]))]
    top_right = [float(top_points[:, 1].max()), float(np.median(top_points[:, 0]))]
    bottom_left = [
        float(bottom_points[:, 1].min()),
        float(np.median(bottom_points[:, 0])),
    ]
    bottom_right = [
        float(bottom_points[:, 1].max()),
        float(np.median(bottom_points[:, 0])),
    ]
    ground_contact_center_px = [
        float(np.mean(bottom_points[:, 1])),
        float(np.median(bottom_points[:, 0])),
    ]

    ground_depth = sample_mask_band_depth(depth_image, mask, points_yx, "lower")
    if ground_depth is None or ground_depth <= 0.0:
        return None

    top_width_px = top_right[0] - top_left[0]
    bottom_width_px = bottom_right[0] - bottom_left[0]
    model = {
        "type": "trapezoidal_prism",
        "visible_face": "lateral",
        "lateral_face_corners_px": [
            _round_point(top_left),
            _round_point(top_right),
            _round_point(bottom_right),
            _round_point(bottom_left),
        ],
        "ground_contact_center_px": _round_point(ground_contact_center_px),
        "ground_contact_depth_m": round(float(ground_depth), 3),
        "ground_contact_offset_flu": calculate_offset_flu(
            ground_contact_center_px,
            ground_depth,
            intrinsics,
        ),
        "top_width_px": round(float(top_width_px), 3),
        "bottom_width_px": round(float(bottom_width_px), 3),
        "height_px": height_px,
    }
    return model


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
