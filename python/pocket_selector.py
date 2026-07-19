from dataclasses import dataclass, field

import numpy as np


@dataclass
class PocketCandidate:
    class_id: int
    angle_deg: float
    pixel_center: tuple


@dataclass
class PocketSelection:
    target_class_id: int = None
    candidates: list = field(default_factory=list)
    best: PocketCandidate = None


class PocketSelector:
    """只負責選擇目前最低號目標球及夾角最小的球袋。"""

    @staticmethod
    def _center(box):
        x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
        return int((x1 + x2) / 2), int((y1 + y2) / 2)

    def select(self, detections, coordinates):
        selection = PocketSelection()
        selection.target_class_id = next(
            (class_id for class_id in range(9) if class_id in detections),
            None,
        )

        if selection.target_class_id is None or 9 not in detections:
            return selection

        target_index = selection.target_class_id * 2
        cue = np.array(coordinates[18:20], dtype=float)
        target = np.array(coordinates[target_index:target_index + 2], dtype=float)
        cue_vector = target - cue

        for pocket_class in range(10, 16):
            if pocket_class not in detections:
                continue

            pocket_index = pocket_class * 2
            pocket = np.array(
                coordinates[pocket_index:pocket_index + 2],
                dtype=float,
            )
            pocket_vector = pocket - target
            cue_length = np.linalg.norm(cue_vector)
            pocket_length = np.linalg.norm(pocket_vector)
            if cue_length <= 0.1 or pocket_length <= 0.1:
                angle = 999.0
            else:
                cosine = np.dot(cue_vector, pocket_vector) / (
                    cue_length * pocket_length
                )
                angle = float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))

            selection.candidates.append(PocketCandidate(
                class_id=pocket_class,
                angle_deg=angle,
                pixel_center=self._center(detections[pocket_class]),
            ))

        if selection.candidates:
            selection.best = min(
                selection.candidates,
                key=lambda candidate: candidate.angle_deg,
            )
        return selection
