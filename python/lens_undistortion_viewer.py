"""Interactive side-by-side viewer for validating lens calibration."""

from datetime import datetime
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

from lens_undistortion import load_calibration, undistort_frame


CaptureFactory = Callable[..., object]


def _draw_reference_overlay(
    frame: np.ndarray,
    label: str,
    distortion_coefficients: np.ndarray,
) -> np.ndarray:
    overlay = frame.copy()
    height, width = overlay.shape[:2]
    color = (0, 255, 255)
    for fraction in (0.25, 0.5, 0.75):
        x = round(width * fraction)
        y = round(height * fraction)
        cv2.line(overlay, (x, 0), (x, height - 1), color, 1)
        cv2.line(overlay, (0, y), (width - 1, y), color, 1)
    cv2.putText(
        overlay,
        label,
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    coefficient_text = "D: " + ", ".join(
        f"{value:.5g}" for value in distortion_coefficients
    )
    cv2.putText(
        overlay,
        coefficient_text,
        (20, height - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return overlay


def run_live_test(
    calibration_path: Path,
    captures_directory: Path,
    camera_index: int = 0,
    capture_factory: CaptureFactory | None = None,
) -> str:
    """Open the fixed-resolution camera and compare raw/corrected frames."""

    calibration = load_calibration(Path(calibration_path))
    width, height = calibration.image_size
    capture_factory = capture_factory or cv2.VideoCapture
    capture = capture_factory(camera_index, cv2.CAP_DSHOW)
    try:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not capture.isOpened():
            raise RuntimeError(f"Unable to open camera index {camera_index}.")

        while True:
            success, frame = capture.read()
            if not success or frame is None:
                return "frame_read_failed"
            actual_height, actual_width = frame.shape[:2]
            if (actual_width, actual_height) != calibration.image_size:
                raise RuntimeError(
                    "Camera resolution does not match calibration: "
                    f"{actual_width}x{actual_height} != {width}x{height}."
                )

            corrected = undistort_frame(frame, calibration)
            raw_overlay = _draw_reference_overlay(
                frame,
                f"Original {width}x{height}",
                calibration.distortion_coefficients,
            )
            corrected_overlay = _draw_reference_overlay(
                corrected,
                f"Undistorted {width}x{height}",
                calibration.distortion_coefficients,
            )
            comparison = np.hstack((raw_overlay, corrected_overlay))
            display = cv2.resize(
                comparison,
                (width, height // 2),
                interpolation=cv2.INTER_AREA,
            )
            cv2.imshow("Lens Distortion Test", display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                return "quit"
            if key in (ord("s"), ord("S")):
                captures_directory = Path(captures_directory)
                captures_directory.mkdir(parents=True, exist_ok=True)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                cv2.imwrite(
                    str(captures_directory / f"{timestamp}_original.png"),
                    frame,
                )
                cv2.imwrite(
                    str(
                        captures_directory
                        / f"{timestamp}_undistorted.png"
                    ),
                    corrected,
                )
                cv2.imwrite(
                    str(captures_directory / f"{timestamp}_comparison.png"),
                    comparison,
                )
    finally:
        capture.release()
        cv2.destroyAllWindows()
