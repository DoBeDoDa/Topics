from pathlib import Path

import cv2
import numpy as np


CAMERA_MATRIX = np.array(
    [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]]
)
BOARD_OUTER_CORNERS = np.float32(
    [[0, 0, 0], [250, 0, 0], [250, 175, 0], [0, 175, 0]]
)
CAMERA_POSES = [
    ([0, 0, 0], [-125, -87.5, 700]),
    ([0.15, -0.1, 0.03], [-220, -80, 720]),
    ([-0.12, 0.16, -0.04], [-20, -90, 750]),
    ([0.2, 0.05, 0.1], [-130, -150, 680]),
    ([-0.18, -0.08, -0.08], [-120, -20, 650]),
    ([0.08, 0.2, 0.15], [-160, -100, 600]),
]


def write_chessboard_image(
    path: Path,
    destination_corners: list[list[float]],
) -> None:
    square_size = 60
    square_columns = 10
    square_rows = 7
    board = np.full(
        (square_rows * square_size, square_columns * square_size),
        255,
        dtype=np.uint8,
    )
    for row in range(square_rows):
        for column in range(square_columns):
            if (row + column) % 2 == 0:
                board[
                    row * square_size : (row + 1) * square_size,
                    column * square_size : (column + 1) * square_size,
                ] = 0

    source_corners = np.float32(
        [
            [0, 0],
            [square_columns * square_size - 1, 0],
            [
                square_columns * square_size - 1,
                square_rows * square_size - 1,
            ],
            [0, square_rows * square_size - 1],
        ]
    )
    transform = cv2.getPerspectiveTransform(
        source_corners,
        np.float32(destination_corners),
    )
    image = cv2.warpPerspective(
        board,
        transform,
        (1280, 720),
        borderValue=200,
    )
    cv2.imwrite(str(path), image)


def write_consistent_chessboard_views(directory: Path) -> list[Path]:
    image_paths = []
    for index, (rotation, translation) in enumerate(CAMERA_POSES):
        projected_corners, _ = cv2.projectPoints(
            BOARD_OUTER_CORNERS,
            np.array(rotation, dtype=np.float64),
            np.array(translation, dtype=np.float64),
            CAMERA_MATRIX,
            np.zeros(5),
        )
        image_path = directory / f"view-{index}.png"
        write_chessboard_image(
            image_path,
            projected_corners.reshape(-1, 2).tolist(),
        )
        image_paths.append(image_path)
    return image_paths


def write_zero_distortion_calibration(path: Path) -> None:
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    storage.write("image_width", 1280)
    storage.write("image_height", 720)
    storage.write("camera_matrix", CAMERA_MATRIX)
    storage.write(
        "distortion_coefficients",
        np.zeros((1, 5), dtype=np.float64),
    )
    storage.release()
