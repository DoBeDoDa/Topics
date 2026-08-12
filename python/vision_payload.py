"""Build and validate the existing strict 32-value Base0 XY vision payload."""

from dataclasses import dataclass
import math

from detection_filter import CaptureRejected


MISSING_COORDINATE = -9999.0
FIELD_COUNT = 32
BALL_CLASS_COUNT = 10
POCKET_COUNT = 6


def detection_name(class_id):
    if 0 <= class_id <= 8:
        return f"Ball_{class_id + 1}"
    if class_id == 9:
        return "Ball_cue"
    if 10 <= class_id <= 15:
        return f"P{class_id - 9}"
    return f"class_{class_id}"


def box_center(box):
    try:
        x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
    except (AttributeError, TypeError, ValueError) as error:
        raise CaptureRejected(f"Invalid YOLO bbox: {error}") from error
    center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
    if not all(math.isfinite(value) for value in (*center, x1, y1, x2, y2)):
        raise CaptureRejected("YOLO bbox contains a non-finite value")
    if x2 < x1 or y2 < y1:
        raise CaptureRejected("YOLO bbox has reversed bounds")
    return center


@dataclass(frozen=True)
class ProjectedVisionFrame:
    coordinates: tuple
    pixel_centers: dict
    base0_points: dict
    missing_ball_names: tuple

    @property
    def wire_message(self):
        return format_wire_message(self.coordinates)


def validate_coordinates(coordinates):
    try:
        field_count = len(coordinates)
    except TypeError as error:
        raise CaptureRejected("Vision payload must be a sized sequence") from error
    if field_count != FIELD_COUNT:
        raise CaptureRejected(f"Vision payload must contain exactly {FIELD_COUNT} values")
    try:
        values = tuple(float(value) for value in coordinates)
    except (TypeError, ValueError, OverflowError) as error:
        raise CaptureRejected(f"Vision payload contains a non-numeric value: {error}") from error
    if not all(math.isfinite(value) for value in values):
        raise CaptureRejected("Vision payload contains NaN or Inf")

    for class_id in range(BALL_CLASS_COUNT):
        x_value = values[class_id * 2]
        y_value = values[class_id * 2 + 1]
        if (x_value == MISSING_COORDINATE) != (y_value == MISSING_COORDINATE):
            raise CaptureRejected(
                f"{detection_name(class_id)} contains a single-sided missing sentinel"
            )

    for pocket_index in range(POCKET_COUNT):
        field_index = 20 + pocket_index * 2
        if values[field_index] == MISSING_COORDINATE or values[field_index + 1] == MISSING_COORDINATE:
            raise CaptureRejected(f"P{pocket_index + 1} is missing")
    return values


def build_projected_frame(detections, projector):
    coordinates = [MISSING_COORDINATE] * FIELD_COUNT
    pixel_centers = {}
    base0_points = {}
    missing_ball_names = []

    for class_id in range(BALL_CLASS_COUNT + POCKET_COUNT):
        name = detection_name(class_id)
        box = detections.get(class_id)
        if box is None:
            if class_id < BALL_CLASS_COUNT:
                missing_ball_names.append(name)
                continue
            raise CaptureRejected(f"Required pocket {name} is missing")

        center = box_center(box)
        try:
            base0_x, base0_y, base0_z = map(float, projector.project(*center))
        except Exception as error:
            raise CaptureRejected(f"Geometry rejected {name}: {error}") from error
        if not all(math.isfinite(value) for value in (base0_x, base0_y, base0_z)):
            raise CaptureRejected(f"Geometry returned NaN or Inf for {name}")
        if base0_x == MISSING_COORDINATE or base0_y == MISSING_COORDINATE:
            raise CaptureRejected(
                f"Geometry returned the reserved missing sentinel for detected {name}"
            )

        field_index = class_id * 2
        coordinates[field_index] = base0_x
        coordinates[field_index + 1] = base0_y
        pixel_centers[name] = center
        base0_points[name] = (base0_x, base0_y, base0_z)

    validated = validate_coordinates(coordinates)
    return ProjectedVisionFrame(
        coordinates=validated,
        pixel_centers=pixel_centers,
        base0_points=base0_points,
        missing_ball_names=tuple(missing_ball_names),
    )


def format_wire_message(coordinates):
    values = validate_coordinates(coordinates)
    message = ",".join(str(value) for value in values) + "\n"
    if message.count("\n") != 1 or not message.endswith("\n"):
        raise CaptureRejected("Vision payload framing must contain exactly one trailing newline")
    return message
