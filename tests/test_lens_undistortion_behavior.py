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

from lens_undistortion import load_calibration, undistort_frame
from lens_undistortion_viewer import run_live_test
from tests.calibration_fixtures import write_zero_distortion_calibration


class FakeCapture:
    def __init__(self, frame: np.ndarray):
        self.frame = frame
        self.settings = {}
        self.released = False

    def isOpened(self) -> bool:
        return True

    def set(self, property_id: int, value: float) -> bool:
        self.settings[property_id] = value
        return True

    def read(self) -> tuple[bool, np.ndarray]:
        return True, self.frame.copy()

    def release(self) -> None:
        self.released = True


class LensUndistortionTests(unittest.TestCase):
    def test_zero_distortion_preserves_frame_size_and_pixels(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            calibration_path = Path(temporary_directory) / "intrinsics.yml"
            write_zero_distortion_calibration(calibration_path)
            frame = np.arange(720 * 1280 * 3, dtype=np.uint8).reshape(
                720,
                1280,
                3,
            )

            calibration = load_calibration(calibration_path)
            corrected = undistort_frame(frame, calibration)

            self.assertEqual(corrected.shape, frame.shape)
            np.testing.assert_array_equal(corrected, frame)

    def test_live_viewer_configures_and_releases_fixed_resolution_camera(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            calibration_path = root / "intrinsics.yml"
            write_zero_distortion_calibration(calibration_path)
            fake_capture = FakeCapture(
                np.zeros((720, 1280, 3), dtype=np.uint8)
            )

            with (
                patch(
                    "lens_undistortion_viewer.cv2.imshow"
                ),
                patch(
                    "lens_undistortion_viewer.cv2.waitKey",
                    return_value=ord("q"),
                ),
                patch(
                    "lens_undistortion_viewer.cv2.destroyAllWindows"
                ),
            ):
                exit_reason = run_live_test(
                    calibration_path,
                    root / "captures",
                    capture_factory=lambda *_: fake_capture,
                )

            self.assertEqual(exit_reason, "quit")
            self.assertEqual(
                fake_capture.settings[cv2.CAP_PROP_FRAME_WIDTH],
                1280,
            )
            self.assertEqual(
                fake_capture.settings[cv2.CAP_PROP_FRAME_HEIGHT],
                720,
            )
            self.assertTrue(fake_capture.released)

    def test_save_key_writes_original_corrected_and_comparison_images(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            calibration_path = root / "intrinsics.yml"
            captures_directory = root / "captures"
            write_zero_distortion_calibration(calibration_path)
            fake_capture = FakeCapture(
                np.zeros((720, 1280, 3), dtype=np.uint8)
            )

            with (
                patch("lens_undistortion_viewer.cv2.imshow"),
                patch(
                    "lens_undistortion_viewer.cv2.waitKey",
                    side_effect=[ord("s"), ord("q")],
                ),
                patch("lens_undistortion_viewer.cv2.destroyAllWindows"),
            ):
                run_live_test(
                    calibration_path,
                    captures_directory,
                    capture_factory=lambda *_: fake_capture,
                )

            saved_names = {
                path.name for path in captures_directory.glob("*.png")
            }
            self.assertEqual(len(saved_names), 3)
            self.assertTrue(
                any(name.endswith("_original.png") for name in saved_names)
            )
            self.assertTrue(
                any(
                    name.endswith("_undistorted.png")
                    for name in saved_names
                )
            )
            self.assertTrue(
                any(name.endswith("_comparison.png") for name in saved_names)
            )


if __name__ == "__main__":
    unittest.main()
