"""Production RGB vision -> Base0 XY -> existing 32-value TCP service.

Raw images are accumulated until every tracked object resolves to a stable
state. Each resolved observation is emitted as one existing-format 32-value
Logical Frame. Freshness remains owned by the existing C++ local shot-cycle
gate, stale-buffer flush, and accumulation reset; no control handshake or
wire metadata is added.
"""

import math
import os
import socket
import sys

import cv2

from capture_accumulator import CaptureWindowAccumulator
from detection_filter import CaptureRejected, DetectionFilter
from rgb_base0_geometry import (
    CURRENT_CALIBRATION_PATH,
    CalibrationStartupError,
    RgbBase0Geometry,
)
from vision_payload import format_wire_message
from yolo_inference import YoloInference


CAMERA_INDEX = 0
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_MODEL_PATH = os.path.join(ROOT_DIR, "bin", "best.pt")

RAW_BALL_COLOR = (0, 255, 255)
RAW_CUE_COLOR = (255, 255, 255)
RAW_HOLE_COLOR = (255, 0, 255)


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

    def capture_raw(self, frame):
        """One raw RGB image -> one YOLO pass -> ungated per-image
        detections. Unlike the old single-shot pipeline, fewer than six
        holes or missing balls is not rejected here; that judgment moved to
        CaptureWindowAccumulator, which sees many images per Logical Frame.
        """
        if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 2:
            raise CaptureRejected("Captured RGB image is invalid")
        height, width = frame.shape[:2]
        if width != self.geometry.width or height != self.geometry.height:
            raise CaptureRejected(
                f"Captured RGB image is {width}x{height}; calibration requires "
                f"{self.geometry.width}x{self.geometry.height}"
            )

        results = self.inference.infer(frame)
        return self.filter.extract_raw(results)

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
    """The existing sole owner of the one-way Python-to-C++ TCP socket."""

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
        self._accept()

    def _accept(self):
        print(f"[TCP] Waiting for the existing C++ client on port {self.port}...")
        self.connection, self.address = self.server_socket.accept()
        print(f"[TCP] C++ client connected: {self.address}")

    def reaccept(self):
        """C++端receiveFrame() timeout後會自己關掉socket再重新connect；
        server_socket本身沒斷（一直listen(1)），但舊accept()只做一次，
        run()迴圈原本會一直對著舊connection送資料、永遠等不到C++的新
        connect()。這裡關掉舊connection、重新accept一次，讓C++重連能夠
        真的接上，而不是卡死等待。"""
        if self.connection is not None:
            self.connection.close()
            self.connection = None
        self._accept()

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
            print("[TCP] one validated Logical Frame sent")
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


def _draw_raw_preview(frame, raw):
    """Lightweight per-image preview: labels detected balls, and marks raw
    (still-unlabeled -- P1..P6 identity isn't known until the accumulator
    resolves six stable clusters) hole detections generically."""
    annotated = frame.copy()
    for class_id, box in raw.balls.items():
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        color = RAW_CUE_COLOR if class_id == 9 else RAW_BALL_COLOR
        label = "bw" if class_id == 9 else f"b{class_id + 1}"
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            annotated, label, (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
        )
    for box in raw.raw_holes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cv2.rectangle(annotated, (x1, y1), (x2, y2), RAW_HOLE_COLOR, 1)
    return annotated


def _print_dashboard(coordinates):
    os.system("cls" if os.name == "nt" else "clear")
    print("=====================================================")
    print(" RGB vision -> Robot Base0 XY (mm)")
    print("=====================================================")
    for ball_id in range(1, 10):
        x, y = coordinates[(ball_id - 1) * 2], coordinates[(ball_id - 1) * 2 + 1]
        print(f" [Ball_{ball_id}] {x:>9.2f}, {y:>9.2f}")
    print(f" [Ball_cue] {coordinates[18]:>9.2f}, {coordinates[19]:>9.2f}")
    for pocket_id in range(1, 7):
        field_index = 20 + (pocket_id - 1) * 2
        print(f" [P{pocket_id}] {coordinates[field_index]:>9.2f}, {coordinates[field_index + 1]:>9.2f}")
    print("=====================================================")
    print(" Press q or Esc to stop")


class BilliardVisionApp:
    """Coordinate startup, continuous stable observation, YOLO, and TCP."""

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

    def run(self):
        try:
            # Camera profile verification completes before the TCP service starts.
            self.camera.start()
            self.server.start()
            accumulator = CaptureWindowAccumulator()

            while True:
                frame = self.camera.get_frame()
                if frame is None:
                    print("[CAPTURE REJECTED] RGB image capture failed; skipping this image")
                    continue

                try:
                    raw = self.detector.capture_raw(frame)
                except CaptureRejected as error:
                    print(f"[CAPTURE REJECTED] {error}; skipping this image")
                    cv2.imshow("Direct Arm Vision", frame)
                    if self._quit_requested():
                        break
                    continue

                accumulator.feed(raw, self.detector.geometry)

                try:
                    coordinates = accumulator.resolved_coordinates(self.detector.geometry)
                except CaptureRejected as error:
                    print(f"[CAPTURE REJECTED] {error}; resetting this Logical Frame")
                    accumulator.reset()
                    coordinates = None

                if coordinates is not None:
                    _print_dashboard(coordinates)
                    if not self.server.send_coords(coordinates):
                        print("[TCP] C++端連線已斷，等待重新connect...")
                        self.server.reaccept()
                    accumulator.reset()
                elif accumulator.timed_out():
                    print(
                        "[WARN] Logical Frame did not resolve within the image "
                        "budget; resetting this Logical Frame's accumulator"
                    )
                    accumulator.reset()

                cv2.imshow("Direct Arm Vision", _draw_raw_preview(frame, raw))
                if self._quit_requested():
                    break
        finally:
            self.camera.stop()
            self.server.close()
            self.detector.close()
            cv2.destroyAllWindows()
            print("[SYSTEM] Production vision service stopped")

    @staticmethod
    def _quit_requested():
        key = cv2.waitKey(1) & 0xFF
        return key in (ord("q"), ord("Q"), 27)


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
