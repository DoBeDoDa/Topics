"""Per-image ball/pocket detection filtering.

Two entry points share the same ball-dedup logic:
  - filter(): the original single-image, all-or-nothing contract (still
    requires >=6 raw holes in that one image and immediately assigns
    P1..P6). Kept unchanged for existing callers/tests.
  - extract_raw(): a new no-gate variant that returns whatever balls/holes
    one image happened to show, for CaptureWindowAccumulator to accumulate
    across many images before any P1..P6 assignment happens.
"""

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


@dataclass(frozen=True)
class RawImageDetections:
    """One image's ball/hole detections with no completeness requirement."""
    balls: dict  # class_id (0..9) -> box, deduped by confidence
    raw_holes: tuple  # any count, including zero
    duplicate_ball_drops: dict


def order_six_by_position(items, center_of):
    """Order exactly six items into [P1, P2, P3, P4, P5, P6].

    Image-space contract: bottom L->R=P1,P2,P3; top L->R=P6,P5,P4.
    center_of(item) must return that item's (x, y) pixel center.
    """
    vertical = sorted(items, key=lambda item: (center_of(item)[1], center_of(item)[0]))
    top = sorted(vertical[:3], key=lambda item: center_of(item)[0])
    bottom = sorted(vertical[3:], key=lambda item: center_of(item)[0])
    p6, p5, p4 = top
    p1, p2, p3 = bottom
    return [p1, p2, p3, p4, p5, p6]


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

    def _dedupe_balls_and_collect_holes(self, results):
        if not results:
            raise CaptureRejected("YOLO returned no result object")

        balls = {}
        holes = []
        duplicate_ball_drops = {}

        for box in results[0].boxes:
            class_id = int(box.cls[0])
            if class_id == self.GENERIC_POCKET_CLASS:
                self._confidence(box)
                self._center(box)
                holes.append(box)
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

        return balls, holes, duplicate_ball_drops

    def filter(self, results):
        balls, holes, duplicate_ball_drops = self._dedupe_balls_and_collect_holes(results)

        raw_hole_count = len(holes)
        if raw_hole_count < self.MAX_POCKETS:
            raise CaptureRejected(
                f"Detected {raw_hole_count} holes; exactly six are required"
            )

        selected = sorted(
            holes,
            key=lambda box: (
                -self._confidence(box),
                self._center(box)[1],
                self._center(box)[0],
            ),
        )[:self.MAX_POCKETS]

        ordered = order_six_by_position(selected, self._center)

        filtered = dict(balls)
        filtered[10], filtered[11], filtered[12], filtered[13], filtered[14], filtered[15] = ordered

        return FilteredDetections(
            detections=filtered,
            raw_hole_count=raw_hole_count,
            selected_holes=tuple(selected),
            duplicate_ball_drops=duplicate_ball_drops,
        )

    def extract_raw(self, results):
        """No-gate per-image extraction for CaptureWindowAccumulator.

        Unlike filter(), fewer than six holes (or zero) is not an error —
        the accumulator is responsible for deciding when enough images have
        accumulated to resolve six stable pocket locations.
        """
        balls, holes, duplicate_ball_drops = self._dedupe_balls_and_collect_holes(results)
        return RawImageDetections(
            balls=balls,
            raw_holes=tuple(holes),
            duplicate_ball_drops=duplicate_ball_drops,
        )
