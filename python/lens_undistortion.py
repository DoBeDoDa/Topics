"""Load OpenCV lens parameters and correct full-frame camera images."""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class LensCalibration:
    """Parameters required to undistort one fixed-resolution camera stream."""

    image_size: tuple[int, int]
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray


def load_calibration(path: Path) -> LensCalibration:
    """Load and validate the five-parameter calibration file."""

    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise FileNotFoundError(f"Unable to open calibration file: {path}")
    try:
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
        camera_matrix = storage.getNode("camera_matrix").mat()
        distortion_coefficients = storage.getNode(
            "distortion_coefficients"
        ).mat()
    finally:
        storage.release()

    if width <= 0 or height <= 0:
        raise ValueError("Calibration file has an invalid image size.")
    if camera_matrix is None or camera_matrix.shape != (3, 3):
        raise ValueError("Calibration file has an invalid camera matrix.")
    if distortion_coefficients is None:
        raise ValueError("Calibration file has no distortion coefficients.")
    distortion_coefficients = np.asarray(
        distortion_coefficients,
        dtype=np.float64,
    ).reshape(-1)
    if distortion_coefficients.size != 5:
        raise ValueError(
            "Calibration file must contain exactly five distortion coefficients."
        )

    return LensCalibration(
        image_size=(width, height),
        camera_matrix=np.asarray(camera_matrix, dtype=np.float64),
        distortion_coefficients=distortion_coefficients,
    )


def undistort_frame(
    frame: np.ndarray,
    calibration: LensCalibration,
) -> np.ndarray:
    """Correct one image without cropping or changing its resolution."""

    height, width = frame.shape[:2]
    if (width, height) != calibration.image_size:
        raise ValueError(
            "Frame resolution does not match the calibration resolution: "
            f"{width}x{height} != "
            f"{calibration.image_size[0]}x{calibration.image_size[1]}."
        )
    return cv2.undistort(
        frame,
        calibration.camera_matrix,
        calibration.distortion_coefficients,
        None,
        calibration.camera_matrix,
    )
