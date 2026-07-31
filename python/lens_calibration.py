"""Core interfaces for folder-based camera lens calibration."""

import csv
import json
import os
import shutil
import uuid
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


SUPPORTED_IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png"}


@dataclass(frozen=True)
class CalibrationConfig:
    """Settings that control acceptance of a calibration run."""

    min_valid_images: int = 15
    image_size: tuple[int, int] = (1280, 720)
    pattern_size: tuple[int, int] = (9, 6)
    max_rms: float = 1.0
    max_filter_rounds: int = 3
    outlier_sigma: float = 3.0


@dataclass(frozen=True)
class CalibrationRunResult:
    """Observable result returned by a folder calibration run."""

    accepted: bool
    reason: str
    valid_image_count: int
    rms_error: float | None = None
    rejected_image_count: int = 0
    run_directory: Path | None = None
    quality: str = "failed"
    live_test_status: str = "not_started"


def _detect_corners(
    image: np.ndarray,
    pattern_size: tuple[int, int],
) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(gray, pattern_size)
    if found:
        return corners.astype(np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return None

    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def _object_points(pattern_size: tuple[int, int]) -> np.ndarray:
    columns, rows = pattern_size
    points = np.zeros((columns * rows, 3), np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    return points


def _write_calibration_file(
    path: Path,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    config: CalibrationConfig,
    rms_error: float,
) -> None:
    file_storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not file_storage.isOpened():
        raise OSError(f"Unable to write calibration file: {path}")
    try:
        file_storage.write("schema_version", 1)
        file_storage.write("image_width", config.image_size[0])
        file_storage.write("image_height", config.image_size[1])
        file_storage.write("pattern_columns", config.pattern_size[0])
        file_storage.write("pattern_rows", config.pattern_size[1])
        file_storage.write("rms_error", float(rms_error))
        file_storage.write("camera_matrix", camera_matrix)
        file_storage.write(
            "distortion_coefficients",
            distortion_coefficients.reshape(1, -1),
        )
    finally:
        file_storage.release()


def _validate_calibration_file(
    path: Path,
    expected_image_size: tuple[int, int],
) -> None:
    file_storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not file_storage.isOpened():
        raise OSError(f"Unable to reopen calibration file: {path}")
    try:
        width = int(file_storage.getNode("image_width").real())
        height = int(file_storage.getNode("image_height").real())
        camera_matrix = file_storage.getNode("camera_matrix").mat()
        distortion_coefficients = file_storage.getNode(
            "distortion_coefficients"
        ).mat()
    finally:
        file_storage.release()

    if (width, height) != expected_image_size:
        raise ValueError("Saved calibration has the wrong image size.")
    if camera_matrix is None or camera_matrix.shape != (3, 3):
        raise ValueError("Saved calibration has an invalid camera matrix.")
    if (
        distortion_coefficients is None
        or distortion_coefficients.size != 5
    ):
        raise ValueError(
            "Saved calibration does not contain five distortion coefficients."
        )
    if not np.all(np.isfinite(camera_matrix)) or not np.all(
        np.isfinite(distortion_coefficients)
    ):
        raise ValueError("Saved calibration contains non-finite values.")


def _promote_latest_calibration(
    source_path: Path,
    latest_path: Path,
    expected_image_size: tuple[int, int],
) -> None:
    temporary_path = latest_path.with_name(
        f".{latest_path.stem}.{uuid.uuid4().hex}.tmp.yml"
    )
    try:
        shutil.copyfile(source_path, temporary_path)
        _validate_calibration_file(temporary_path, expected_image_size)
        os.replace(temporary_path, latest_path)
    finally:
        temporary_path.unlink(missing_ok=True)


def _calibrate_views(
    image_points: list[np.ndarray],
    config: CalibrationConfig,
) -> tuple[
    float,
    np.ndarray,
    np.ndarray,
    list[np.ndarray],
    list[np.ndarray],
    list[float],
]:
    object_template = _object_points(config.pattern_size)
    object_points = [object_template.copy() for _ in image_points]
    (
        rms_error,
        camera_matrix,
        distortion_coefficients,
        rotation_vectors,
        translation_vectors,
    ) = cv2.calibrateCamera(
        object_points,
        image_points,
        config.image_size,
        None,
        None,
    )

    per_view_errors = []
    for object_view, image_view, rotation, translation in zip(
        object_points,
        image_points,
        rotation_vectors,
        translation_vectors,
    ):
        projected, _ = cv2.projectPoints(
            object_view,
            rotation,
            translation,
            camera_matrix,
            distortion_coefficients,
        )
        residuals = image_view.reshape(-1, 2) - projected.reshape(-1, 2)
        per_view_errors.append(
            float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
        )

    return (
        float(rms_error),
        camera_matrix,
        distortion_coefficients,
        rotation_vectors,
        translation_vectors,
        per_view_errors,
    )


def _write_diagnostic_report(
    run_directory: Path,
    image_diagnostics: dict[Path, dict[str, object]],
) -> None:
    report_path = run_directory / "calibration_report.csv"
    with report_path.open("w", newline="", encoding="utf-8-sig") as report_file:
        writer = csv.writer(report_file)
        writer.writerow(
            ["filename", "status", "reason", "reprojection_error_px"]
        )
        for image_path in sorted(image_diagnostics):
            diagnostic = image_diagnostics[image_path]
            writer.writerow(
                [
                    image_path.name,
                    diagnostic["status"],
                    diagnostic["reason"],
                    diagnostic.get("reprojection_error_px", ""),
                ]
            )


def _write_summary(
    run_directory: Path,
    *,
    accepted: bool,
    reason: str,
    quality: str,
    valid_image_count: int,
    rejected_image_count: int,
    config: CalibrationConfig,
    rms_error: float | None = None,
    camera_matrix: np.ndarray | None = None,
    distortion_coefficients: np.ndarray | None = None,
) -> None:
    summary = {
        "accepted": accepted,
        "reason": reason,
        "quality": quality,
        "valid_image_count": valid_image_count,
        "rejected_image_count": rejected_image_count,
        "rms_error_px": rms_error,
        "image_size": list(config.image_size),
        "pattern_inner_corners": list(config.pattern_size),
        "distortion_model": "opencv_standard_5_parameter",
        "camera_matrix": (
            camera_matrix.tolist() if camera_matrix is not None else None
        ),
        "distortion_coefficients": (
            np.asarray(distortion_coefficients).reshape(-1).tolist()
            if distortion_coefficients is not None
            else None
        ),
    }
    with (run_directory / "calibration_summary.json").open(
        "w",
        encoding="utf-8",
    ) as summary_file:
        json.dump(summary, summary_file, ensure_ascii=False, indent=2)


def calibrate_folder(
    images_directory: Path,
    results_directory: Path,
    config: CalibrationConfig | None = None,
) -> CalibrationRunResult:
    """Calibrate from images in one directory without deleting source files."""

    config = config or CalibrationConfig()
    images_directory = Path(images_directory)
    results_directory = Path(results_directory)
    results_directory.mkdir(parents=True, exist_ok=True)
    run_directory = results_directory / datetime.now().strftime(
        "run_%Y%m%d_%H%M%S_%f"
    )
    run_directory.mkdir()
    image_paths = [
        path
        for path in images_directory.iterdir()
        if path.is_file() and path.suffix.lower() in SUPPORTED_IMAGE_SUFFIXES
    ]

    candidate_images = []
    rejected_image_count = 0
    image_diagnostics = {
        image_path: {
            "status": "pending",
            "reason": "",
        }
        for image_path in image_paths
    }
    for image_path in image_paths:
        image = cv2.imread(str(image_path))
        if image is None:
            rejected_image_count += 1
            image_diagnostics[image_path] = {
                "status": "rejected",
                "reason": "unreadable_image",
            }
            continue
        height, width = image.shape[:2]
        if (width, height) != config.image_size:
            rejected_image_count += 1
            image_diagnostics[image_path] = {
                "status": "rejected",
                "reason": "wrong_resolution",
            }
            continue
        candidate_images.append((image_path, image))

    detected_views = []
    corner_previews_directory = run_directory / "corner_previews"
    for image_path, image in candidate_images:
        corners = _detect_corners(image, config.pattern_size)
        if corners is not None:
            detected_views.append((image_path, corners))
            image_diagnostics[image_path] = {
                "status": "detected",
                "reason": "",
            }
            corner_previews_directory.mkdir(exist_ok=True)
            annotated = image.copy()
            cv2.drawChessboardCorners(
                annotated,
                config.pattern_size,
                corners,
                True,
            )
            cv2.imwrite(
                str(
                    corner_previews_directory
                    / f"{image_path.stem}_corners.png"
                ),
                annotated,
            )
        else:
            rejected_image_count += 1
            image_diagnostics[image_path] = {
                "status": "rejected",
                "reason": "chessboard_not_found",
            }

    valid_image_count = len(detected_views)
    if valid_image_count < config.min_valid_images:
        _write_diagnostic_report(run_directory, image_diagnostics)
        _write_summary(
            run_directory,
            accepted=False,
            reason="insufficient_valid_images",
            quality="failed",
            valid_image_count=valid_image_count,
            rejected_image_count=rejected_image_count,
            config=config,
        )
        return CalibrationRunResult(
            accepted=False,
            reason="insufficient_valid_images",
            valid_image_count=valid_image_count,
            rejected_image_count=rejected_image_count,
            run_directory=run_directory,
        )

    active_views = detected_views
    for _ in range(config.max_filter_rounds):
        calibration = _calibrate_views(
            [corners for _, corners in active_views],
            config,
        )
        per_view_errors = np.asarray(calibration[-1], dtype=np.float64)
        median_error = float(np.median(per_view_errors))
        median_absolute_deviation = float(
            np.median(np.abs(per_view_errors - median_error))
        )
        robust_spread = 1.4826 * median_absolute_deviation
        outlier_threshold = median_error + max(
            config.outlier_sigma * robust_spread,
            0.1,
        )
        outlier_indexes = [
            index
            for index, error in enumerate(per_view_errors)
            if error > outlier_threshold
        ]
        removable_count = len(active_views) - config.min_valid_images
        if not outlier_indexes or removable_count <= 0:
            break
        outlier_indexes.sort(
            key=lambda index: per_view_errors[index],
            reverse=True,
        )
        removed_indexes = set(outlier_indexes[:removable_count])
        for removed_index in removed_indexes:
            removed_path = active_views[removed_index][0]
            image_diagnostics[removed_path] = {
                "status": "rejected",
                "reason": "statistical_outlier",
                "reprojection_error_px": float(
                    per_view_errors[removed_index]
                ),
            }
        rejected_image_count += len(removed_indexes)
        active_views = [
            view
            for index, view in enumerate(active_views)
            if index not in removed_indexes
        ]
    else:
        calibration = _calibrate_views(
            [corners for _, corners in active_views],
            config,
        )

    (
        rms_error,
        camera_matrix,
        distortion_coefficients,
        _,
        _,
        final_per_view_errors,
    ) = calibration
    valid_image_count = len(active_views)
    for (image_path, _), view_error in zip(
        active_views,
        final_per_view_errors,
    ):
        image_diagnostics[image_path] = {
            "status": "accepted_view",
            "reason": "",
            "reprojection_error_px": view_error,
        }
    _write_diagnostic_report(run_directory, image_diagnostics)

    images_by_path = dict(candidate_images)
    undistortion_previews_directory = (
        run_directory / "undistortion_previews"
    )
    undistortion_previews_directory.mkdir(exist_ok=True)
    for image_path, _ in active_views:
        original = images_by_path[image_path]
        corrected = cv2.undistort(
            original,
            camera_matrix,
            distortion_coefficients,
            None,
            camera_matrix,
        )
        comparison = np.hstack((original, corrected))
        cv2.imwrite(
            str(
                undistortion_previews_directory
                / f"{image_path.stem}_comparison.png"
            ),
            comparison,
        )

    run_calibration_path = run_directory / "camera_intrinsics.yml"
    _write_calibration_file(
        run_calibration_path,
        camera_matrix,
        distortion_coefficients,
        config,
        float(rms_error),
    )

    if rms_error > config.max_rms:
        _write_summary(
            run_directory,
            accepted=False,
            reason="rms_above_limit",
            quality="failed",
            valid_image_count=valid_image_count,
            rejected_image_count=rejected_image_count,
            config=config,
            rms_error=float(rms_error),
            camera_matrix=camera_matrix,
            distortion_coefficients=distortion_coefficients,
        )
        return CalibrationRunResult(
            accepted=False,
            reason="rms_above_limit",
            valid_image_count=valid_image_count,
            rms_error=float(rms_error),
            rejected_image_count=rejected_image_count,
            run_directory=run_directory,
            quality="failed",
        )

    quality = "good" if rms_error <= 0.5 else "warning"
    _promote_latest_calibration(
        run_calibration_path,
        results_directory / "camera_intrinsics_latest.yml",
        config.image_size,
    )
    _write_summary(
        run_directory,
        accepted=True,
        reason="accepted",
        quality=quality,
        valid_image_count=valid_image_count,
        rejected_image_count=rejected_image_count,
        config=config,
        rms_error=float(rms_error),
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion_coefficients,
    )
    return CalibrationRunResult(
        accepted=True,
        reason="accepted",
        valid_image_count=valid_image_count,
        rms_error=float(rms_error),
        rejected_image_count=rejected_image_count,
        run_directory=run_directory,
        quality=quality,
    )
