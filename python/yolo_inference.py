from ultralytics import YOLO


class YoloInference:
    """只負責載入 YOLO 模型並執行單幀推論。"""

    def __init__(self, model_path, confidence=0.3):
        self.model = YOLO(model_path)
        self.confidence = confidence

    def infer(self, frame):
        return self.model(frame, conf=self.confidence, verbose=False)
