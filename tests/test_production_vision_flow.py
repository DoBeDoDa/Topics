import pathlib
import math
import shutil
import sys
import tempfile
import unittest


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[1]
PYTHON_DIRECTORY = REPOSITORY_ROOT / "python"
sys.path.insert(0, str(PYTHON_DIRECTORY))

from detection_filter import CaptureRejected, DetectionFilter
from rgb_base0_geometry import (
    CURRENT_CALIBRATION_PATH,
    DEFAULT_BRIDGE_PATH,
    CalibrationStartupError,
    RgbBase0Geometry,
)
from vision_payload import (
    MISSING_COORDINATE,
    build_projected_frame,
    box_center,
    format_wire_message,
    validate_coordinates,
)


class _BoxRow(list):
    def tolist(self):
        return list(self)


class FakeBox:
    def __init__(self, class_id, confidence, center_u, center_v):
        self.cls = [class_id]
        self.conf = [confidence]
        self.xyxy = [_BoxRow([
            center_u - 5.0,
            center_v - 5.0,
            center_u + 5.0,
            center_v + 5.0,
        ])]


class FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes


def filter_boxes(boxes):
    return DetectionFilter().filter([FakeResult(boxes)])


def six_holes():
    return [
        FakeBox(10, 0.90, 100, 100),
        FakeBox(10, 0.80, 300, 110),
        FakeBox(10, 0.70, 500, 120),
        FakeBox(10, 0.60, 110, 600),
        FakeBox(10, 0.50, 310, 590),
        FakeBox(10, 0.40, 510, 580),
    ]


class FakeProjector:
    target_z_mm = -211.26

    def __init__(self, failure_center=None, non_finite=False, reserved_sentinel=False):
        self.failure_center = failure_center
        self.non_finite = non_finite
        self.reserved_sentinel = reserved_sentinel
        self.calls = []

    def project(self, u, v):
        self.calls.append((u, v))
        if self.failure_center == (u, v):
            raise ValueError("synthetic ray failure")
        if self.non_finite:
            return math.nan, v, self.target_z_mm
        if self.reserved_sentinel:
            return MISSING_COORDINATE, v, self.target_z_mm
        return u, v, self.target_z_mm


class ProductionDetectionPolicyTests(unittest.TestCase):
    def test_fewer_than_six_holes_rejects_capture(self):
        holes = [FakeBox(10, 0.9, 100 + index * 100, 100) for index in range(5)]

        with self.assertRaises(CaptureRejected):
            filter_boxes(holes)

    def test_more_than_six_holes_filters_by_confidence_then_assigns_in_pixel_space(self):
        top_left = FakeBox(10, 0.90, 100, 100)
        top_middle = FakeBox(10, 0.80, 300, 110)
        top_right = FakeBox(10, 0.70, 500, 120)
        bottom_left = FakeBox(10, 0.60, 110, 600)
        bottom_middle = FakeBox(10, 0.50, 310, 590)
        bottom_right = FakeBox(10, 0.40, 510, 580)
        low_confidence_extra = FakeBox(10, 0.10, 700, 350)

        filtered = filter_boxes([
            low_confidence_extra,
            bottom_right,
            top_middle,
            bottom_left,
            top_right,
            bottom_middle,
            top_left,
        ])

        detections = filtered.detections
        self.assertEqual(filtered.raw_hole_count, 7)
        self.assertIs(detections[10], bottom_left)
        self.assertIs(detections[11], bottom_middle)
        self.assertIs(detections[12], bottom_right)
        self.assertIs(detections[13], top_right)
        self.assertIs(detections[14], top_middle)
        self.assertIs(detections[15], top_left)
        self.assertNotIn(low_confidence_extra, detections.values())

    def test_duplicate_ball_keeps_highest_confidence(self):
        lower = FakeBox(3, 0.60, 200, 300)
        higher = FakeBox(3, 0.95, 220, 320)
        holes = [
            FakeBox(10, 0.9 - index * 0.01, 100 + (index % 3) * 200, 100 + (index // 3) * 500)
            for index in range(6)
        ]

        filtered = filter_boxes([lower, *holes, higher])

        self.assertIs(filtered.detections[3], higher)
        self.assertEqual(filtered.duplicate_ball_drops, {3: 1})

    def test_exactly_six_holes_assigns_top_and_bottom_rows(self):
        filtered = filter_boxes(list(reversed(six_holes())))

        self.assertEqual(_center(filtered.detections[10]), (110.0, 600.0))
        self.assertEqual(_center(filtered.detections[11]), (310.0, 590.0))
        self.assertEqual(_center(filtered.detections[12]), (510.0, 580.0))
        self.assertEqual(_center(filtered.detections[13]), (500.0, 120.0))
        self.assertEqual(_center(filtered.detections[14]), (300.0, 110.0))
        self.assertEqual(_center(filtered.detections[15]), (100.0, 100.0))


def _center(box):
    return box_center(box)


class ProductionPayloadTests(unittest.TestCase):
    def _detections(self, balls=()):
        return filter_boxes([*balls, *six_holes()]).detections

    def test_one_frame_builds_exactly_32_finite_values_and_one_newline(self):
        cue = FakeBox(9, 0.95, 640, 360)
        ball_one = FakeBox(0, 0.90, 400, 300)
        projector = FakeProjector()

        projected = build_projected_frame(self._detections([cue, ball_one]), projector)

        self.assertEqual(len(projected.coordinates), 32)
        self.assertTrue(all(math.isfinite(value) for value in projected.coordinates))
        self.assertEqual(projected.coordinates[0:2], (400.0, 300.0))
        self.assertEqual(projected.coordinates[18:20], (640.0, 360.0))
        self.assertEqual(len(projector.calls), 8)
        self.assertEqual(projected.wire_message.count("\n"), 1)
        self.assertTrue(projected.wire_message.endswith("\n"))
        self.assertEqual(len(projected.wire_message.rstrip("\n").split(",")), 32)

    def test_missing_ball_keeps_its_fixed_sentinel_pair(self):
        ball_four = FakeBox(3, 0.90, 444, 333)
        first = build_projected_frame(self._detections([ball_four]), FakeProjector())
        second = build_projected_frame(self._detections([]), FakeProjector())

        self.assertEqual(first.coordinates[6:8], (444.0, 333.0))
        self.assertEqual(second.coordinates[6:8], (MISSING_COORDINATE, MISSING_COORDINATE))
        self.assertEqual(second.coordinates[8:10], (MISSING_COORDINATE, MISSING_COORDINATE))

    def test_balls_and_pockets_use_the_same_projector(self):
        projector = FakeProjector()
        ball = FakeBox(0, 0.9, 400, 300)

        build_projected_frame(self._detections([ball]), projector)

        self.assertEqual(len(projector.calls), 7)
        self.assertIn((400.0, 300.0), projector.calls)
        self.assertIn((110.0, 600.0), projector.calls)

    def test_geometry_failure_rejects_entire_payload(self):
        ball = FakeBox(0, 0.9, 400, 300)
        projector = FakeProjector(failure_center=(400.0, 300.0))

        with self.assertRaises(CaptureRejected):
            build_projected_frame(self._detections([ball]), projector)

    def test_non_finite_geometry_rejects_entire_payload(self):
        with self.assertRaises(CaptureRejected):
            build_projected_frame(self._detections([]), FakeProjector(non_finite=True))

    def test_detected_geometry_cannot_emit_reserved_missing_sentinel(self):
        with self.assertRaises(CaptureRejected):
            build_projected_frame(
                self._detections([]), FakeProjector(reserved_sentinel=True)
            )

    def test_payload_validation_rejects_wrong_count_nan_and_single_sentinel(self):
        valid = [MISSING_COORDINATE] * 20 + [1.0] * 12
        with self.assertRaises(CaptureRejected):
            validate_coordinates(valid[:-1])
        invalid_nan = list(valid)
        invalid_nan[0] = math.nan
        with self.assertRaises(CaptureRejected):
            validate_coordinates(invalid_nan)
        single_sentinel = list(valid)
        single_sentinel[0] = 1.0
        with self.assertRaises(CaptureRejected):
            validate_coordinates(single_sentinel)
        invalid_text = list(valid)
        invalid_text[0:2] = ["not-a-number", "not-a-number"]
        with self.assertRaises(CaptureRejected):
            validate_coordinates(invalid_text)

    def test_wire_formatter_never_adds_metadata(self):
        coordinates = [MISSING_COORDINATE] * 20 + [1.0] * 12
        message = format_wire_message(coordinates)

        self.assertEqual(message.count(","), 31)
        self.assertEqual(message.count("\n"), 1)
        self.assertEqual(len(message.rstrip("\n").split(",")), 32)


class CppGeometryBridgeTests(unittest.TestCase):
    def test_fixed_current_calibration_loads_and_projects(self):
        with RgbBase0Geometry() as geometry:
            point = geometry.project(642.505126953125, 352.9652099609375)

            self.assertEqual((geometry.width, geometry.height, geometry.fps), (1280, 720, 10))
            self.assertEqual(geometry.camera_serial_number, "AYKW545004C")
            self.assertAlmostEqual(geometry.target_z_mm, -211.26, places=9)
            self.assertTrue(all(math.isfinite(value) for value in point))
            self.assertAlmostEqual(point[2], geometry.target_z_mm, places=7)

    def test_missing_and_invalid_calibration_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            with self.assertRaises(CalibrationStartupError):
                RgbBase0Geometry(root / "missing.json", DEFAULT_BRIDGE_PATH)

            invalid = root / "invalid.json"
            invalid.write_text('{"schema_version":"1.3"}', encoding="utf-8")
            with self.assertRaises(CalibrationStartupError):
                RgbBase0Geometry(invalid, DEFAULT_BRIDGE_PATH)

    def test_calibration_is_loaded_once_not_reopened_per_projection(self):
        with tempfile.TemporaryDirectory() as directory:
            copied = pathlib.Path(directory) / "camera_calibration.json"
            shutil.copyfile(CURRENT_CALIBRATION_PATH, copied)
            with RgbBase0Geometry(copied, DEFAULT_BRIDGE_PATH) as geometry:
                copied.write_text("invalid after startup", encoding="utf-8")
                point = geometry.project(642.505126953125, 352.9652099609375)

            self.assertTrue(all(math.isfinite(value) for value in point))


if __name__ == "__main__":
    unittest.main()
