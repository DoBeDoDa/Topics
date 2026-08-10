#!/usr/bin/env python3
"""YOLO-only sidecar for the standalone RGB->Base0 validation tool.

This script performs no camera access, robot access, or geometry. It reads the ten
raw 1280x720 MJPG/JPEG frames saved by the C++ Orbbec v1.10.18 program.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
from ultralytics import YOLO


EXPECTED_CLASSES = {
    0: "Ball_1",
    1: "Ball_2",
    2: "Ball_3",
    3: "Ball_4",
    4: "Ball_5",
    5: "Ball_6",
    6: "Ball_7",
    7: "Ball_8",
    8: "Ball_9",
    9: "Ball_cue",
    10: "hole",
}
BALL_CLASS_IDS = set(range(10))
HOLE_CLASS_ID = 10


@dataclass
class Detection:
    frame_index: int
    frame_file: str
    class_id: int
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float
    u: float
    v: float
    decision: str
    reason: str


class TeeLogger:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = path.open("a", encoding="utf-8", newline="\n")

    def close(self) -> None:
        self._stream.close()

    def write(self, message: str) -> None:
        print(message, flush=True)
        self._stream.write(message + "\n")
        self._stream.flush()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--confidence", type=float, default=0.3)
    parser.add_argument("--expected-frames", type=int, default=10)
    parser.add_argument("--log-file", type=Path)
    return parser.parse_args()


def median(values: list[float]) -> float:
    return float(statistics.median(values))


def validate_frame(image, path: Path) -> None:
    if image is None:
        raise RuntimeError(f"OpenCV could not decode raw MJPG/JPEG frame: {path}")
    height, width = image.shape[:2]
    if (width, height) != (1280, 720):
        raise RuntimeError(f"Raw frame must remain exactly 1280x720; got {width}x{height}: {path}")


def main() -> int:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    annotated_dir = args.output / "annotated_frames"
    annotated_dir.mkdir(parents=True, exist_ok=True)
    logger = TeeLogger(args.log_file or (args.output / "yolo_terminal_log.txt"))
    try:
        if not (0.0 < args.confidence <= 1.0):
            raise RuntimeError("YOLO confidence must be in (0, 1]")
        frames = sorted(args.frames.glob("*.jpg"))
        if len(frames) != args.expected_frames:
            raise RuntimeError(
                f"Expected exactly {args.expected_frames} raw frames; found {len(frames)} in {args.frames}"
            )
        if not args.weights.is_file():
            raise RuntimeError(f"YOLO weights do not exist: {args.weights}")

        logger.write("[YOLO] Python handles detections only; it does not access Orbbec or HRSDK.")
        logger.write(f"[YOLO] weights={args.weights} confidence={args.confidence:.3f}")
        model = YOLO(str(args.weights))
        names = {int(key): str(value) for key, value in model.names.items()}
        if names != EXPECTED_CLASSES:
            raise RuntimeError(f"Unexpected YOLO class map. expected={EXPECTED_CLASSES}, actual={names}")

        all_detections: list[Detection] = []
        per_ball_observations: dict[int, list[Detection]] = {class_id: [] for class_id in BALL_CLASS_IDS}
        total_holes = 0

        for frame_index, frame_path in enumerate(frames):
            image = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
            validate_frame(image, frame_path)
            predictions = model.predict(source=image, conf=args.confidence, verbose=False)
            if len(predictions) != 1:
                raise RuntimeError(f"YOLO returned {len(predictions)} result objects for one frame")

            raw: list[Detection] = []
            boxes = predictions[0].boxes
            if boxes is not None:
                for box in boxes:
                    class_id = int(box.cls.item())
                    if class_id not in EXPECTED_CLASSES:
                        raise RuntimeError(f"YOLO returned unknown class id {class_id}")
                    confidence = float(box.conf.item())
                    x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
                    u = (x1 + x2) * 0.5
                    v = (y1 + y2) * 0.5
                    if not all(math.isfinite(value) for value in (confidence, x1, y1, x2, y2, u, v)):
                        raise RuntimeError(f"Non-finite YOLO output in {frame_path}")
                    if x2 <= x1 or y2 <= y1 or u < 0.0 or v < 0.0 or u >= 1280.0 or v >= 720.0:
                        raise RuntimeError(f"Invalid/out-of-bounds YOLO bbox in {frame_path}: {(x1, y1, x2, y2)}")
                    raw.append(
                        Detection(
                            frame_index,
                            frame_path.name,
                            class_id,
                            EXPECTED_CLASSES[class_id],
                            confidence,
                            x1,
                            y1,
                            x2,
                            y2,
                            u,
                            v,
                            "pending",
                            "",
                        )
                    )

            for class_id in BALL_CLASS_IDS:
                candidates = [detection for detection in raw if detection.class_id == class_id]
                candidates.sort(key=lambda detection: detection.confidence, reverse=True)
                for candidate_index, candidate in enumerate(candidates):
                    if candidate_index == 0:
                        candidate.decision = "kept_per_frame"
                        candidate.reason = "highest confidence for this ball class in this frame"
                        per_ball_observations[class_id].append(candidate)
                    else:
                        candidate.decision = "dropped_duplicate"
                        candidate.reason = "same ball class already kept at higher confidence"

            holes = [detection for detection in raw if detection.class_id == HOLE_CLASS_ID]
            holes.sort(key=lambda detection: detection.confidence, reverse=True)
            total_holes += len(holes)
            for hole in holes:
                hole.decision = "listed_only"
                hole.reason = "pocket Base0 calculation is intentionally deferred"

            all_detections.extend(raw)
            logger.write(f"[YOLO] frame={frame_index:02d} file={frame_path.name} raw_count={len(raw)}")
            for detection in sorted(raw, key=lambda item: (item.class_id, -item.confidence)):
                logger.write(
                    "  "
                    f"class={detection.class_name} id={detection.class_id} conf={detection.confidence:.6f} "
                    f"bbox=({detection.x1:.3f},{detection.y1:.3f},{detection.x2:.3f},{detection.y2:.3f}) "
                    f"uv=({detection.u:.3f},{detection.v:.3f}) decision={detection.decision} "
                    f"reason={detection.reason}"
                )

            for detection in raw:
                color = (0, 220, 0) if detection.decision == "kept_per_frame" else (0, 165, 255)
                if detection.decision == "dropped_duplicate":
                    color = (0, 0, 255)
                cv2.rectangle(
                    image,
                    (round(detection.x1), round(detection.y1)),
                    (round(detection.x2), round(detection.y2)),
                    color,
                    2,
                )
                cv2.putText(
                    image,
                    f"{detection.class_name} {detection.confidence:.3f} {detection.decision}",
                    (round(detection.x1), max(20, round(detection.y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                    cv2.LINE_AA,
                )
            annotated_path = annotated_dir / frame_path.name
            if not cv2.imwrite(str(annotated_path), image):
                raise RuntimeError(f"Failed to save annotated frame: {annotated_path}")

        stable_results: list[dict] = []
        logger.write("[YOLO] cross-frame stability analysis")
        for class_id in sorted(BALL_CLASS_IDS):
            observations = per_ball_observations[class_id]
            result = {
                "class_id": class_id,
                "class_name": EXPECTED_CLASSES[class_id],
                "observation_count": len(observations),
                "initial_median_u": None,
                "initial_median_v": None,
                "inlier_count": 0,
                "final_median_u": None,
                "final_median_v": None,
                "median_radial_distance_px": None,
                "status": "rejected",
                "reason": "",
                "observations": [],
            }
            if len(observations) < 8:
                result["reason"] = "fewer than 8/10 frames contain this class"
            else:
                initial_u = median([observation.u for observation in observations])
                initial_v = median([observation.v for observation in observations])
                result["initial_median_u"] = initial_u
                result["initial_median_v"] = initial_v
                inliers: list[Detection] = []
                for observation in observations:
                    radial = math.hypot(observation.u - initial_u, observation.v - initial_v)
                    kept = radial <= 5.0
                    result["observations"].append(
                        {
                            "frame_index": observation.frame_index,
                            "u": observation.u,
                            "v": observation.v,
                            "distance_from_initial_median_px": radial,
                            "inlier": kept,
                            "outlier_reason": "" if kept else "distance exceeds 5 px",
                        }
                    )
                    if kept:
                        inliers.append(observation)
                result["inlier_count"] = len(inliers)
                if len(inliers) < 8:
                    result["reason"] = "fewer than 8 observations remain after 5 px outlier rejection"
                else:
                    final_u = median([observation.u for observation in inliers])
                    final_v = median([observation.v for observation in inliers])
                    radial_distances = [math.hypot(observation.u - final_u, observation.v - final_v) for observation in inliers]
                    median_radial = median(radial_distances)
                    result["final_median_u"] = final_u
                    result["final_median_v"] = final_v
                    result["median_radial_distance_px"] = median_radial
                    if median_radial <= 2.0:
                        result["status"] = "accepted"
                        result["reason"] = "passed >=8/10, 5 px outlier, and <=2 px median radial checks"
                    else:
                        result["reason"] = "median radial distance exceeds 2 px"
            stable_results.append(result)
            logger.write(
                f"  class={result['class_name']} observations={result['observation_count']} "
                f"initial_median=({result['initial_median_u']},{result['initial_median_v']}) "
                f"inliers={result['inlier_count']} final_median=({result['final_median_u']},{result['final_median_v']}) "
                f"median_radial_px={result['median_radial_distance_px']} status={result['status']} "
                f"reason={result['reason']}"
            )
            for observation in result["observations"]:
                logger.write(
                    "    "
                    f"frame={observation['frame_index']:02d} uv=({observation['u']:.3f},{observation['v']:.3f}) "
                    f"distance_initial_px={observation['distance_from_initial_median_px']:.3f} "
                    f"inlier={observation['inlier']} reason={observation['outlier_reason']}"
                )

        hole_counts = []
        for frame_index in range(len(frames)):
            count = sum(
                1
                for detection in all_detections
                if detection.frame_index == frame_index and detection.class_id == HOLE_CLASS_ID
            )
            hole_counts.append(count)
            if count > 6:
                logger.write(f"[WARNING] frame={frame_index:02d} detected {count} holes; expected at most 6")
        logger.write(
            "[DEFERRED] Hole detections are listed only. After ball Base0 points are confirmed, remind the operator "
            "to implement and validate pocket-point calculation."
        )

        detections_document = {
            "schema_version": "1.0",
            "confidence_threshold": args.confidence,
            "frame_count": len(frames),
            "class_map": EXPECTED_CLASSES,
            "raw_detections": [asdict(detection) for detection in all_detections],
            "stable_ball_results": stable_results,
            "hole_counts_per_frame": hole_counts,
            "hole_calculation": "deferred_list_only",
        }
        with (args.output / "detections.json").open("w", encoding="utf-8", newline="\n") as stream:
            json.dump(detections_document, stream, ensure_ascii=False, indent=2)
            stream.write("\n")

        detection_fields = list(Detection.__dataclass_fields__)
        with (args.output / "detections.csv").open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=detection_fields)
            writer.writeheader()
            writer.writerows(asdict(detection) for detection in all_detections)

        with (args.output / "stable_ball_pixels.csv").open("w", encoding="utf-8-sig", newline="") as stream:
            fields = [
                "class_id",
                "class_name",
                "observation_count",
                "inlier_count",
                "final_median_u",
                "final_median_v",
                "median_radial_distance_px",
                "status",
                "reason",
            ]
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for result in stable_results:
                writer.writerow({field: result[field] for field in fields})

        logger.write(f"[YOLO] completed; raw detections={len(all_detections)} total hole detections={total_holes}")
        return 0
    except Exception as error:  # fail closed and preserve a useful terminal artifact
        logger.write(f"[YOLO ERROR] {type(error).__name__}: {error}")
        return 1
    finally:
        logger.close()


if __name__ == "__main__":
    sys.exit(main())
