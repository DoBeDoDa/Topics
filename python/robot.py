"""Production RGB vision -> Base0 XY -> existing 32-value TCP service."""

import math
import os
import socket
import sys
import time

import cv2

from detection_filter import CaptureRejected, DetectionFilter
from rgb_base0_geometry import (
    CURRENT_CALIBRATION_PATH,
    CalibrationStartupError,
    RgbBase0Geometry,
)
from vision_payload import build_projected_frame, detection_name, format_wire_message
from vision_renderer import VisionRenderer
from yolo_inference import YoloInference


CAMERA_INDEX = 0
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_MODEL_PATH = os.path.join(ROOT_DIR, "bin", "best.pt")


def _box_confidence(box):
    return float(box.conf[0])


def _box_center(box):
    x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


class BilliardDetector:
    """Own one YOLO inference and one startup-loaded C++ geometry handle."""

    def __init__(self, model_path=None, calibration_path=CURRENT_CALIBRATION_PATH):
        model_path = model_path or DEFAULT_MODEL_PATH
        self.geometry = RgbBase0Geometry(calibration_path=calibration_path)
        try:
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"YOLO model is missing: {model_path}")
            print("[STARTUP] Loading YOLO model...")
            self.inference = YoloInference(model_path)
        except Exception:
            self.geometry.close()
            raise

        self.filter = DetectionFilter()
        self.renderer = VisionRenderer()
        print(f"[CALIBRATION] fixed path: {self.geometry.calibration_path}")
        print(
            "[CALIBRATION] profile="
            f"{self.geometry.width}x{self.geometry.height}@{self.geometry.fps} "
            f"{self.geometry.profile_format} serial={self.geometry.camera_serial_number} "
            f"Z_target_mm={self.geometry.target_z_mm:.6f}"
        )
        print(
            "[CALIBRATION WARNING] OpenCV camera backend cannot verify the Gemini serial; "
            "runtime checks enforce image dimensions, FPS, and MJPG where exposed."
        )

    def detect(self, frame):
        if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 2:
            raise CaptureRejected("Captured RGB image is invalid")
        height, width = frame.shape[:2]
        print(f"[CAPTURE] calibration path={self.geometry.calibration_path}")
        print(f"[CAPTURE] image dimensions={width}x{height}")
        if width != self.geometry.width or height != self.geometry.height:
            raise CaptureRejected(
                f"Captured RGB image is {width}x{height}; calibration requires "
                f"{self.geometry.width}x{self.geometry.height}"
            )

        results = self.inference.infer(frame)
        filtered = self.filter.filter(results)
        detections = filtered.detections
        projected = build_projected_frame(detections, self.geometry)

        accepted = [
            f"{detection_name(class_id)}(conf={_box_confidence(box):.4f})"
            for class_id, box in sorted(detections.items())
        ]
        print(f"[YOLO] accepted detections: {accepted}")
        print(f"[YOLO] duplicate ball detections dropped: {filtered.duplicate_ball_drops}")
        print(f"[YOLO] raw hole count: {filtered.raw_hole_count}")
        print(
            "[YOLO] selected six holes: "
            f"{[(_box_confidence(box), _box_center(box)) for box in filtered.selected_holes]}"
        )
        pocket_pixels = {
            name: center for name, center in projected.pixel_centers.items() if name.startswith("P")
        }
        ball_pixels = {
            name: center for name, center in projected.pixel_centers.items() if not name.startswith("P")
        }
        print(f"[PIXEL] assigned P1-P6 centers: {pocket_pixels}")
        print(f"[PIXEL] final ball centers: {ball_pixels}")
        for name, point in projected.base0_points.items():
            print(f"[BASE0] {name}=({point[0]:.6f}, {point[1]:.6f}) mm")
        for name in projected.missing_ball_names:
            print(f"[MISSING] {name}=-9999.0,-9999.0")
        print(f"[PAYLOAD] {projected.wire_message.rstrip()}")

        annotated, display_data = self.renderer.render(
            frame, detections, projected.coordinates
        )
        return annotated, projected, display_data

    def close(self):
        self.geometry.close()


class USBCamera:
    """Open exactly the RGB profile declared by the fixed calibration."""

    def __init__(self, width, height, fps, profile_format, camera_index=CAMERA_INDEX):
        self.camera_index = camera_index
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.profile_format = str(profile_format)
        self.cap = None

    @staticmethod
    def _fourcc_text(value):
        integer = int(round(value))
        return "".join(chr((integer >> (8 * index)) & 0xFF) for index in range(4))

    def start(self):
        if self.cap is not None:
            return
        print(f"[CAMERA] Opening RGB camera index {self.camera_index}...")
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = None
            raise RuntimeError(f"Cannot open RGB camera index {self.camera_index}")

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.profile_format))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        actual_width = int(round(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        actual_height = int(round(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        actual_fps = float(self.cap.get(cv2.CAP_PROP_FPS))
        actual_format = self._fourcc_text(self.cap.get(cv2.CAP_PROP_FOURCC))
        if actual_width != self.width or actual_height != self.height:
            self.stop()
            raise RuntimeError(
                f"RGB camera profile mismatch: got {actual_width}x{actual_height}, "
                f"required {self.width}x{self.height}"
            )
        if not math.isfinite(actual_fps) or abs(actual_fps - self.fps) > 0.5:
            self.stop()
            raise RuntimeError(
                f"RGB camera FPS mismatch: got {actual_fps}, required {self.fps}"
            )
        if actual_format != self.profile_format:
            self.stop()
            raise RuntimeError(
                f"RGB camera format mismatch: got {actual_format!r}, required {self.profile_format!r}"
            )
        print(
            f"[CAMERA] verified profile={actual_width}x{actual_height}@{actual_fps:g} "
            f"{actual_format}"
        )

    def get_frame(self):
        if self.cap is None:
            return None
        success, frame = self.cap.read()
        return frame if success else None

    def stop(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class BilliardVisionServer:
    """The existing sole owner of the Python-to-C++ TCP socket."""

    def __init__(self, host="0.0.0.0", port=12345):
        self.host = host
        self.port = port
        self.server_socket = None
        self.connection = None
        self.address = None

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        print(f"[TCP] Waiting for the existing C++ client on port {self.port}...")
        self.connection, self.address = self.server_socket.accept()
        print(f"[TCP] C++ client connected: {self.address}")

    def send_coords(self, coordinates):
        if self.connection is None:
            return False
        try:
            message = format_wire_message(coordinates)
        except CaptureRejected as error:
            print(f"[CAPTURE REJECTED] payload not sent: {error}")
            return False
        try:
            self.connection.sendall(message.encode("utf-8"))
            print("[TCP] one validated 32-value payload sent")
            return True
        except socket.error as error:
            print(f"[TCP ERROR] C++ connection failed: {error}")
            return False

    def close(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None
        if self.server_socket is not None:
            self.server_socket.close()
            self.server_socket = None


class BilliardVisionApp:
    """Coordinate startup, one-image capture events, YOLO, and existing TCP."""

    def __init__(self, model_path=None, port=12345, calibration_path=CURRENT_CALIBRATION_PATH):
        self.detector = BilliardDetector(model_path, calibration_path=calibration_path)
        geometry = self.detector.geometry
        self.camera = USBCamera(
            geometry.width,
            geometry.height,
            geometry.fps,
            geometry.profile_format,
        )
        self.server = BilliardVisionServer(port=port)

    @staticmethod
    def print_dashboard(display_data):
        os.system("cls" if os.name == "nt" else "clear")
        print("=====================================================")
        print(" RGB vision -> Robot Base0 XY (mm)")
        print("=====================================================")
        for ball_id in range(1, 10):
            print(f" [Ball_{ball_id}] {display_data[f'b{ball_id}']}")
        print(f" [Ball_cue] {display_data['bw']}")
        for pocket_id in range(1, 7):
            print(f" [P{pocket_id}] {display_data[f'p{pocket_id}']}")
        print("=====================================================")
        print(" Press q or Esc to stop")

    def run(self):
        try:
            # Camera profile verification completes before the TCP service starts.
            self.camera.start()
            self.server.start()
            last_print_time = 0.0

            while True:
                frame = self.camera.get_frame()  # Exactly one raw RGB image per event.
                if frame is None:
                    print("[CAPTURE REJECTED] RGB image capture failed; payload not sent")
                    continue

                try:
                    annotated, projected, display_data = self.detector.detect(frame)  # YOLO once.
                except CaptureRejected as error:
                    print(f"[CAPTURE REJECTED] {error}; payload not sent")
                    cv2.imshow("Direct Arm Vision", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), ord("Q"), 27):
                        break
                    continue

                current_time = time.time()
                if current_time - last_print_time > 0.5:
                    self.print_dashboard(display_data)
                    last_print_time = current_time

                if not self.server.send_coords(projected.coordinates):
                    break

                cv2.imshow("Direct Arm Vision", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
        finally:
            self.camera.stop()
            self.server.close()
            self.detector.close()
            cv2.destroyAllWindows()
            print("[SYSTEM] Production vision service stopped")


def main():
    try:
        app = BilliardVisionApp()
    except (CalibrationStartupError, FileNotFoundError, RuntimeError) as error:
        print(f"[STARTUP ERROR] Production vision service did not start: {error}", file=sys.stderr)
        return 1
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
