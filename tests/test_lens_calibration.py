import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_DIR = PROJECT_ROOT / "python"
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))

from lens_calibration import CalibrationConfig, calibrate_folder
from tests.calibration_fixtures import (
    CAMERA_POSES,
    write_consistent_chessboard_views,
)


class FolderCalibrationTests(unittest.TestCase):
    def test_folder_with_too_few_valid_images_is_rejected_without_latest_file(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            images_directory = root / "images"
            results_directory = root / "results"
            images_directory.mkdir()

            result = calibrate_folder(
                images_directory,
                results_directory,
                CalibrationConfig(min_valid_images=15),
            )

            self.assertFalse(result.accepted)
            self.assertEqual(result.reason, "insufficient_valid_images")
            self.assertEqual(result.valid_image_count, 0)
            self.assertFalse(
                (results_directory / "camera_intrinsics_latest.yml").exists()
            )

    def test_wrong_resolution_image_is_not_counted_as_valid(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            images_directory = root / "images"
            results_directory = root / "results"
            images_directory.mkdir()
            cv2.imwrite(
                str(images_directory / "wrong-size.png"),
                np.zeros((480, 640, 3), dtype=np.uint8),
            )

            result = calibrate_folder(
                images_directory,
                results_directory,
                CalibrationConfig(min_valid_images=1),
            )

            self.assertFalse(result.accepted)
            self.assertEqual(result.reason, "insufficient_valid_images")
            self.assertEqual(result.valid_image_count, 0)

    def test_valid_chessboard_views_produce_an_accepted_calibration(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            images_directory = root / "images"
            results_directory = root / "results"
            images_directory.mkdir()
            write_consistent_chessboard_views(images_directory)

            result = calibrate_folder(
                images_directory,
                results_directory,
                CalibrationConfig(
                    min_valid_images=len(CAMERA_POSES),
                    max_rms=1.0,
                ),
            )

            self.assertTrue(result.accepted)
            self.assertEqual(result.reason, "accepted")
            self.assertEqual(result.quality, "good")
            self.assertEqual(result.valid_image_count, len(CAMERA_POSES))
            self.assertLessEqual(result.rms_error, 1.0)
            self.assertTrue(
                (results_directory / "camera_intrinsics_latest.yml").exists()
            )
            self.assertTrue(
                (result.run_directory / "calibration_summary.json").exists()
            )
            self.assertEqual(
                len(list((result.run_directory / "corner_previews").glob("*.png"))),
                len(CAMERA_POSES),
            )
            self.assertEqual(
                len(
                    list(
                        (result.run_directory / "undistortion_previews").glob(
                            "*.png"
                        )
                    )
                ),
                len(CAMERA_POSES),
            )

    def test_geometric_outlier_is_excluded_before_final_calibration(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            images_directory = root / "images"
            results_directory = root / "results"
            images_directory.mkdir()
            valid_image_paths = write_consistent_chessboard_views(
                images_directory
            )
            outlier_source = cv2.imread(
                str(valid_image_paths[0]),
            )
            grid_y, grid_x = np.indices((720, 1280), dtype=np.float32)
            outlier = cv2.remap(
                outlier_source,
                grid_x,
                grid_y + 25 * np.sin(2 * np.pi * grid_x / 250),
                cv2.INTER_LINEAR,
                borderValue=200,
            )
            cv2.imwrite(str(images_directory / "outlier.png"), outlier)

            result = calibrate_folder(
                images_directory,
                results_directory,
                CalibrationConfig(
                    min_valid_images=len(CAMERA_POSES),
                    max_rms=1.0,
                ),
            )

            self.assertTrue(result.accepted)
            self.assertEqual(result.rejected_image_count, 1)
            self.assertLessEqual(result.rms_error, 1.0)

    def test_failed_run_preserves_latest_calibration_and_writes_diagnostics(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            images_directory = root / "images"
            results_directory = root / "results"
            images_directory.mkdir()
            results_directory.mkdir()
            latest_file = results_directory / "camera_intrinsics_latest.yml"
            latest_file.write_text("known-good-calibration", encoding="utf-8")

            result = calibrate_folder(
                images_directory,
                results_directory,
                CalibrationConfig(min_valid_images=15),
            )

            self.assertEqual(
                latest_file.read_text(encoding="utf-8"),
                "known-good-calibration",
            )
            self.assertTrue((result.run_directory / "calibration_report.csv").exists())

    def test_failed_atomic_promotion_preserves_previous_latest_file(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            images_directory = root / "images"
            results_directory = root / "results"
            images_directory.mkdir()
            results_directory.mkdir()
            write_consistent_chessboard_views(images_directory)
            latest_file = results_directory / "camera_intrinsics_latest.yml"
            latest_file.write_text("known-good-calibration", encoding="utf-8")

            with (
                patch(
                    "lens_calibration.os.replace",
                    side_effect=OSError("simulated promotion failure"),
                ),
                self.assertRaises(OSError),
            ):
                calibrate_folder(
                    images_directory,
                    results_directory,
                    CalibrationConfig(
                        min_valid_images=len(CAMERA_POSES),
                        max_rms=1.0,
                    ),
                )

            self.assertEqual(
                latest_file.read_text(encoding="utf-8"),
                "known-good-calibration",
            )


if __name__ == "__main__":
    unittest.main()
