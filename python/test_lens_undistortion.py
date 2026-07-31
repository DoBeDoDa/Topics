"""Open the camera and validate the latest accepted lens calibration."""

import argparse

from lens_calibration_paths import (
    LATEST_CALIBRATION,
    LIVE_CAPTURES_DIRECTORY,
)
from lens_undistortion_viewer import run_live_test


def main() -> int:
    parser = argparse.ArgumentParser(
        description="即時比較原始與去畸變後的相機畫面。",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="OpenCV 相機編號，預設為 0。",
    )
    arguments = parser.parse_args()

    try:
        exit_reason = run_live_test(
            LATEST_CALIBRATION,
            LIVE_CAPTURES_DIRECTORY,
            camera_index=arguments.camera_index,
        )
    except Exception as error:
        print(f"無法執行即時鏡頭測試：{error}")
        return 1

    if exit_reason == "frame_read_failed":
        print("相機已開啟，但無法讀取影像。")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
