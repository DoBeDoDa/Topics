"""將球、球袋、路徑與座標資訊繪製到相機畫面。"""

import cv2


class VisionRenderer:
    """只負責偵測框、座標文字與候選擊球線的畫面呈現。"""

    LABEL_MAP = {
        0: "b1", 1: "b2", 2: "b3", 3: "b4", 4: "b5",
        5: "b6", 6: "b7", 7: "b8", 8: "b9", 9: "bw",
        10: "p1", 11: "p2", 12: "p3", 13: "p4", 14: "p5", 15: "p6",
    }

    COLOR_MAP = {
        0: (0, 255, 255), 1: (255, 0, 0), 2: (0, 0, 255),
        3: (0, 165, 255), 4: (128, 0, 128), 5: (0, 128, 0),
        6: (0, 0, 128), 7: (0, 0, 0), 8: (128, 128, 128),
        9: (255, 255, 255), 10: (255, 0, 255), 11: (255, 0, 255),
        12: (255, 0, 255), 13: (255, 0, 255), 14: (255, 0, 255),
        15: (255, 0, 255),
    }

    @staticmethod
    def _center(box):
        x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
        return int((x1 + x2) / 2), int((y1 + y2) / 2)

    def render(self, frame, detections, coordinates, pocket_selection):
        annotated = frame.copy()
        display_data = {}

        for class_id in range(16):
            name = self.LABEL_MAP[class_id]
            box = detections.get(class_id)
            if box is None:
                display_data[name] = "   --- ,    --- "
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            center = self._center(box)
            coordinate_index = class_id * 2
            arm_x = coordinates[coordinate_index]
            arm_y = coordinates[coordinate_index + 1]
            color = self.COLOR_MAP[class_id]

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.circle(annotated, center, 5, (0, 0, 255), -1)
            cv2.putText(
                annotated, name, (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
            )
            display_data[name] = f"{arm_x:>7.1f}, {arm_y:>7.1f}"

        self._render_shot_lines(annotated, detections, pocket_selection)
        return annotated, display_data

    def _render_shot_lines(self, frame, detections, selection):
        if selection.target_class_id is None or selection.best is None:
            return

        cue_center = self._center(detections[9])
        target_center = self._center(detections[selection.target_class_id])
        cv2.line(frame, cue_center, target_center, (255, 255, 255), 2, cv2.LINE_AA)

        for candidate in selection.candidates:
            cv2.line(
                frame, target_center, candidate.pixel_center,
                (140, 140, 140), 1, cv2.LINE_AA,
            )
            midpoint = (
                int((target_center[0] + candidate.pixel_center[0]) / 2),
                int((target_center[1] + candidate.pixel_center[1]) / 2),
            )
            cv2.putText(
                frame, f"{candidate.angle_deg:.1f} deg", midpoint,
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1,
            )

        best = selection.best
        cv2.line(
            frame, target_center, best.pixel_center,
            (0, 255, 0), 3, cv2.LINE_AA,
        )
        cv2.circle(frame, best.pixel_center, 15, (0, 255, 0), 2)
        cv2.putText(
            frame,
            f"TARGET p{best.class_id - 9} ({best.angle_deg:.1f} deg)",
            (best.pixel_center[0] - 50, best.pixel_center[1] - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
        )
