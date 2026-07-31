"""Calibrate lens distortion from the project's fixed image folder."""

import argparse
from dataclasses import replace
from pathlib import Path
from typing import Callable

from lens_calibration_paths import IMAGES_DIRECTORY, RESULTS_DIRECTORY
from lens_calibration import (
    CalibrationConfig,
    CalibrationRunResult,
    calibrate_folder,
)
from lens_undistortion_viewer import run_live_test


def run_calibration_workflow(
    images_directory: Path = IMAGES_DIRECTORY,
    results_directory: Path = RESULTS_DIRECTORY,
    config: CalibrationConfig | None = None,
    live_test_runner: Callable[..., str] = run_live_test,
) -> CalibrationRunResult:
    """Run folder calibration and open live validation only after acceptance."""

    images_directory = Path(images_directory)
    results_directory = Path(results_directory)
    images_directory.mkdir(parents=True, exist_ok=True)
    result = calibrate_folder(
        images_directory,
        results_directory,
        config,
    )
    print(f"校正結果：{result.reason}")
    print(f"有效照片：{result.valid_image_count}")
    print(f"排除照片：{result.rejected_image_count}")
    print(f"品質分級：{result.quality}")
    if result.rms_error is not None:
        print(f"整體 RMS：{result.rms_error:.6f} px")
    print(f"診斷資料：{result.run_directory}")

    if not result.accepted:
        print("校正不合格：保留原有合格參數，不開啟鏡頭測試。")
        return result

    latest_calibration = (
        results_directory / "camera_intrinsics_latest.yml"
    )
    captures_directory = results_directory / "live_captures"
    print("校正合格，現在開啟即時去畸變測試。")
    try:
        live_test_status = live_test_runner(
            latest_calibration,
            captures_directory,
            camera_index=0,
        )
    except Exception as error:
        print(f"即時鏡頭測試失敗，但校正參數仍保留：{error}")
        return replace(result, live_test_status="error")

    if live_test_status == "frame_read_failed":
        print("即時鏡頭測試無法讀取畫面；校正參數仍然有效並已保留。")
    return replace(result, live_test_status=live_test_status)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="從固定資料夾計算相機內參與5參數鏡頭畸變。",
    )
    parser.add_argument(
        "--max-rms",
        type=float,
        default=1.0,
        help="合格的最大整體重投影誤差，預設 1.0 px。",
    )
    arguments = parser.parse_args()
    result = run_calibration_workflow(
        config=CalibrationConfig(max_rms=arguments.max_rms),
    )
    return 0 if result.accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
