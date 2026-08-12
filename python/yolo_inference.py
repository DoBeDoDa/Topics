"""載入 YOLO 模型並執行單張影像的物件偵測推論。"""

from ultralytics import YOLO


class YoloInference:
    """只負責載入 YOLO 模型並執行單幀推論。"""

    def __init__(self, model_path, confidence=0.3):
        self.model = YOLO(model_path)
        self.confidence = confidence
        expected_names = {
            0: "Ball_1", 1: "Ball_2", 2: "Ball_3", 3: "Ball_4",
            4: "Ball_5", 5: "Ball_6", 6: "Ball_7", 7: "Ball_8",
            8: "Ball_9", 9: "Ball_cue", 10: "hole",
        }
        if dict(self.model.names) != expected_names:
            raise RuntimeError(
                "YOLO class mapping is incompatible with the 32-value contract: "
                f"{self.model.names}"
            )

    def infer(self, frame):
        return self.model(frame, conf=self.confidence, verbose=False)
