class DetectionFilter:
    """把 YOLO 原始 boxes 整理成 b1~b9、bw、p1~p6。"""

    BALL_CLASS_COUNT = 10
    GENERIC_POCKET_CLASS = 10
    OUTPUT_POCKET_CLASS_START = 10
    MAX_POCKETS = 6

    def filter(self, results):
        balls = {}
        pockets = []

        for box in results[0].boxes:
            class_id = int(box.cls[0])
            if class_id == self.GENERIC_POCKET_CLASS:
                pockets.append(box)
                continue

            if class_id >= self.BALL_CLASS_COUNT:
                continue

            previous = balls.get(class_id)
            if previous is None or box.conf[0] > previous.conf[0]:
                balls[class_id] = box

        filtered = dict(balls)
        pockets.sort(key=lambda box: float(box.xyxy[0][0]))
        for index, box in enumerate(pockets[:self.MAX_POCKETS]):
            filtered[self.OUTPUT_POCKET_CLASS_START + index] = box

        return filtered
