import os

import cv2
import numpy as np

try:
    import torch
    import torch.nn as nn
    HAS_TORCH = True
except ImportError:
    torch = None
    nn = None
    HAS_TORCH = False


if HAS_TORCH:
    class CalibrationNet(nn.Module):
        def __init__(self):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(2, 64),
                nn.ReLU(),
                nn.Linear(64, 64),
                nn.ReLU(),
                nn.Linear(64, 2),
            )

        def forward(self, value):
            return self.net(value)
else:
    class CalibrationNet:
        pass


class HomographyCoordinateTransformer:
    """只負責將相機像素座標轉換為手臂平面座標。"""

    def __init__(self, camera_points, table_points, use_nn=False, nn_model_path=None):
        self.camera_points = np.asarray(camera_points, dtype=np.float32)
        self.table_points = np.asarray(table_points, dtype=np.float32)
        self.use_nn = bool(use_nn)

        if self.use_nn:
            self._load_neural_network(nn_model_path)
        else:
            self.matrix = self._create_homography()

    def _create_homography(self):
        if len(self.camera_points) != len(self.table_points):
            raise ValueError("相機校正點與手臂校正點數量不一致。")
        if len(self.camera_points) < 4:
            raise ValueError("Homography 至少需要 4 組校正點。")

        matrix, _ = cv2.findHomography(
            self.camera_points,
            self.table_points,
            cv2.LMEDS,
        )
        if matrix is None:
            raise ValueError("無法由目前校正點建立 Homography 矩陣。")
        return matrix

    def _load_neural_network(self, model_path):
        if not HAS_TORCH:
            raise RuntimeError("要求使用神經網路校正，但目前環境沒有 PyTorch。")
        if not model_path or not os.path.exists(model_path):
            raise FileNotFoundError(f"找不到神經網路校正模型: {model_path}")

        self.torch_device = torch.device("cpu")
        checkpoint = torch.load(
            model_path,
            map_location=self.torch_device,
            weights_only=False,
        )
        self.nn_model = CalibrationNet()
        self.nn_model.load_state_dict(checkpoint["model_state"])
        self.nn_model.eval()
        self.cam_mean = checkpoint["cam_mean"]
        self.cam_std = checkpoint["cam_std"]
        self.table_mean = checkpoint["table_mean"]
        self.table_std = checkpoint["table_std"]

    def transform(self, camera_x, camera_y):
        if self.use_nn:
            value = np.array([camera_x, camera_y], dtype=np.float32)
            scaled = (value - self.cam_mean) / self.cam_std
            tensor = torch.tensor(scaled, dtype=torch.float32).to(self.torch_device)
            with torch.no_grad():
                prediction = self.nn_model(tensor).cpu().numpy()
            result = prediction * self.table_std + self.table_mean
            return round(float(result[0]), 1), round(float(result[1]), 1)

        point = np.array([[[camera_x, camera_y]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(point, self.matrix)
        return (
            round(float(transformed[0][0][0]), 1),
            round(float(transformed[0][0][1]), 1),
        )

    def transform_detections(self, detections, output_class_count=16):
        coordinates = [-9999.0] * (output_class_count * 2)
        for class_id, box in detections.items():
            x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
            arm_x, arm_y = self.transform((x1 + x2) / 2, (y1 + y2) / 2)
            coordinates[class_id * 2] = arm_x
            coordinates[class_id * 2 + 1] = arm_y
        return coordinates
