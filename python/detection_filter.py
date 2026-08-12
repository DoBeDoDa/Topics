"""Filter one raw-image YOLO result into fixed ball and pocket identities."""

from dataclasses import dataclass
import math


class CaptureRejected(RuntimeError):
    """The current image cannot produce a complete, safe vision payload."""


@dataclass(frozen=True)
class FilteredDetections:
    detections: dict
    raw_hole_count: int
    selected_holes: tuple
    duplicate_ball_drops: dict


class DetectionFilter:
    """Keep highest-confidence balls and assign exactly six generic holes."""

    BALL_CLASS_COUNT = 10
    GENERIC_POCKET_CLASS = 10
    MAX_POCKETS = 6

    @staticmethod
    def _confidence(box):
        confidence = float(box.conf[0])
        if not math.isfinite(confidence):
            raise CaptureRejected("YOLO detection confidence is non-finite")
        return confidence

    @staticmethod
    def _center(box):
        try:
            x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
        except (AttributeError, TypeError, ValueError) as error:
            raise CaptureRejected(f"Invalid YOLO hole bbox: {error}") from error
        center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        if not all(math.isfinite(value) for value in center):
            raise CaptureRejected("YOLO hole center is non-finite")
        return center

    def filter(self, results):
        if not results:
            raise CaptureRejected("YOLO returned no result object")

        balls = {}
        pockets = []
        duplicate_ball_drops = {}

        for box in results[0].boxes:
            class_id = int(box.cls[0])
            if class_id == self.GENERIC_POCKET_CLASS:
                self._confidence(box)
                self._center(box)
                pockets.append(box)
                continue
            if class_id < 0 or class_id >= self.BALL_CLASS_COUNT:
                continue

            self._confidence(box)
            previous = balls.get(class_id)
            if previous is None:
                balls[class_id] = box
                continue
            duplicate_ball_drops[class_id] = duplicate_ball_drops.get(class_id, 0) + 1
            if self._confidence(box) > self._confidence(previous):
                balls[class_id] = box

        raw_hole_count = len(pockets)
        if raw_hole_count < self.MAX_POCKETS:
            raise CaptureRejected(
                f"Detected {raw_hole_count} holes; exactly six are required"
            )

        selected = sorted(
            pockets,
            key=lambda box: (
                -self._confidence(box),
                self._center(box)[1],
                self._center(box)[0],
            ),
        )[:self.MAX_POCKETS]

        vertical = sorted(
            selected,
            key=lambda box: (self._center(box)[1], self._center(box)[0]),
        )
        top = sorted(vertical[:3], key=lambda box: self._center(box)[0])
        bottom = sorted(vertical[3:], key=lambda box: self._center(box)[0])

        filtered = dict(balls)
        # Image-space contract: bottom L->R=P1,P2,P3; top L->R=P6,P5,P4.
        filtered[10], filtered[11], filtered[12] = bottom
        filtered[15], filtered[14], filtered[13] = top

        return FilteredDetections(
            detections=filtered,
            raw_hole_count=raw_hole_count,
            selected_holes=tuple(selected),
            duplicate_ball_drops=duplicate_ball_drops,
        )
