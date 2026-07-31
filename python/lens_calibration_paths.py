"""Shared project paths for lens calibration commands."""

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
LENS_CALIBRATION_DIRECTORY = PROJECT_ROOT / "calibration" / "lens"
IMAGES_DIRECTORY = LENS_CALIBRATION_DIRECTORY / "images"
RESULTS_DIRECTORY = LENS_CALIBRATION_DIRECTORY / "results"
LATEST_CALIBRATION = RESULTS_DIRECTORY / "camera_intrinsics_latest.yml"
LIVE_CAPTURES_DIRECTORY = RESULTS_DIRECTORY / "live_captures"
