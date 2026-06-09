import math
import unittest

import numpy as np

from yolo_pkg.segmentation_geometry import (
    build_segmentation_offset,
    compute_mask_geometry,
    estimate_bridge_model,
)


class SegmentationGeometryTest(unittest.TestCase):
    def test_compute_mask_geometry_center_area_and_angle(self):
        mask = np.zeros((20, 20), dtype=bool)
        for idx in range(5):
            mask[5 + idx, 8 + idx] = True
            mask[5 + idx, 9 + idx] = True

        geometry = compute_mask_geometry(mask)

        self.assertEqual(geometry["mask_area_px"], 10)
        self.assertAlmostEqual(geometry["mask_center_px"][0], 10.5)
        self.assertAlmostEqual(geometry["mask_center_px"][1], 7.0)
        self.assertAlmostEqual(
            abs(geometry["mask_angle_rad"]), math.pi / 4.0, places=1
        )

    def test_build_segmentation_offset_shape(self):
        mask = np.zeros((10, 10), dtype=bool)
        mask[4:6, 4:6] = True
        depth = np.ones((10, 10), dtype=float)
        obj = {
            "label": "bear",
            "confidence": 0.91,
            "box": (4, 4, 5, 5),
            "mask": mask,
        }

        result = build_segmentation_offset(
            obj,
            depth,
            {"fx": 1.0, "fy": 1.0, "cx": 4.5, "cy": 4.5},
        )

        self.assertEqual(result["label"], "bear")
        self.assertEqual(result["confidence"], 0.91)
        self.assertEqual(result["offset_flu"], [1.0, -0.0, -0.0])
        self.assertEqual(result["box"], [4, 4, 5, 5])
        self.assertEqual(result["mask_center_px"], [4.5, 4.5])
        self.assertEqual(result["mask_area_px"], 4)
        self.assertIn("mask_angle_rad", result)

    def test_bridge_offset_uses_ground_contact_center(self):
        mask = np.zeros((20, 30), dtype=bool)
        for y in range(6, 15):
            left = 14 - (y - 6) // 2
            right = 16 + (y - 6) // 2
            mask[y, left : right + 1] = True
        depth = np.ones((20, 30), dtype=float) * 2.0
        obj = {
            "label": "bridge",
            "confidence": 0.95,
            "box": (10, 6, 20, 15),
            "mask": mask,
        }

        result = build_segmentation_offset(
            obj,
            depth,
            {"fx": 10.0, "fy": 10.0, "cx": 15.0, "cy": 10.0},
        )

        self.assertEqual(result["label"], "bridge")
        self.assertIn("bridge_model", result)
        self.assertIn("mask_center_offset_flu", result)
        self.assertGreater(
            result["bridge_model"]["bottom_width_px"],
            result["bridge_model"]["top_width_px"],
        )
        self.assertEqual(
            result["offset_flu"],
            result["bridge_model"]["ground_contact_offset_flu"],
        )
        self.assertGreater(result["mask_center_offset_flu"][2], result["offset_flu"][2])

    def test_estimate_bridge_model_shape(self):
        mask = np.zeros((20, 30), dtype=bool)
        mask[5:8, 13:18] = True
        mask[8:15, 10:21] = True
        depth = np.ones((20, 30), dtype=float)

        model = estimate_bridge_model(
            mask,
            depth,
            {"fx": 1.0, "fy": 1.0, "cx": 15.0, "cy": 10.0},
        )

        self.assertEqual(model["type"], "trapezoidal_prism")
        self.assertEqual(model["visible_face"], "lateral")
        self.assertEqual(len(model["lateral_face_corners_px"]), 4)
        self.assertIn("ground_contact_center_px", model)
        self.assertIn("ground_contact_offset_flu", model)


if __name__ == "__main__":
    unittest.main()
