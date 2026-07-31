import sys
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_DIR = PROJECT_ROOT / "python"
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))

from calibrate_intrinsics import run_calibration_workflow
from lens_calibration import CalibrationConfig
from tests.calibration_fixtures import (
    CAMERA_POSES,
    write_consistent_chessboard_views,
)


class CalibrationWorkflowTests(unittest.TestCase):
    def test_failed_calibration_does_not_open_camera(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            images_directory = root / "images"
            results_directory = root / "results"
            images_directory.mkdir()
            camera_opened = False

            def fail_if_camera_opens(*_args, **_kwargs):
                nonlocal camera_opened
                camera_opened = True
                raise AssertionError("Camera must not open after failed calibration.")

            result = run_calibration_workflow(
                images_directory=images_directory,
                results_directory=results_directory,
                config=CalibrationConfig(min_valid_images=15),
                live_test_runner=fail_if_camera_opens,
            )

            self.assertFalse(result.accepted)
            self.assertFalse(camera_opened)

    def test_accepted_calibration_automatically_opens_live_test(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            images_directory = root / "images"
            results_directory = root / "results"
            images_directory.mkdir()
            write_consistent_chessboard_views(images_directory)
            opened_calibration_path = None

            def record_live_test(calibration_path, *_args, **_kwargs):
                nonlocal opened_calibration_path
                opened_calibration_path = calibration_path
                return "quit"

            result = run_calibration_workflow(
                images_directory=images_directory,
                results_directory=results_directory,
                config=CalibrationConfig(
                    min_valid_images=len(CAMERA_POSES),
                    max_rms=1.0,
                ),
                live_test_runner=record_live_test,
            )

            self.assertTrue(result.accepted)
            self.assertEqual(result.live_test_status, "quit")
            self.assertEqual(
                opened_calibration_path,
                results_directory / "camera_intrinsics_latest.yml",
            )

    def test_live_frame_failure_does_not_invalidate_accepted_calibration(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            images_directory = root / "images"
            results_directory = root / "results"
            images_directory.mkdir()
            write_consistent_chessboard_views(images_directory)

            result = run_calibration_workflow(
                images_directory=images_directory,
                results_directory=results_directory,
                config=CalibrationConfig(
                    min_valid_images=len(CAMERA_POSES),
                    max_rms=1.0,
                ),
                live_test_runner=lambda *_args, **_kwargs: (
                    "frame_read_failed"
                ),
            )

            self.assertTrue(result.accepted)
            self.assertEqual(result.live_test_status, "frame_read_failed")


if __name__ == "__main__":
    unittest.main()
