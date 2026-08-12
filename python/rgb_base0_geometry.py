"""Production wrapper around the authoritative C++ RGB-pixel to Base0 geometry."""

import ctypes
import math
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CURRENT_CALIBRATION_PATH = REPOSITORY_ROOT / "config" / "vision" / "camera_calibration.json"
DEFAULT_BRIDGE_PATH = (
    REPOSITORY_ROOT / "build" / "rgb_base0_calibration" / "rgb_base0_geometry_bridge.dll"
)


class CalibrationStartupError(RuntimeError):
    """The fixed current calibration or its C++ geometry bridge is unusable."""


class RgbBase0Geometry:
    """Load one fixed calibration once and project pixels through the C++ core."""

    _ERROR_CAPACITY = 2048
    _SERIAL_CAPACITY = 256

    def __init__(self, calibration_path=CURRENT_CALIBRATION_PATH, bridge_path=DEFAULT_BRIDGE_PATH):
        self.calibration_path = Path(calibration_path).resolve()
        self.bridge_path = Path(bridge_path).resolve()
        self._handle = ctypes.c_void_p()

        if not self.calibration_path.is_file():
            raise CalibrationStartupError(
                f"Current calibration file is missing: {self.calibration_path}"
            )
        if not self.bridge_path.is_file():
            raise CalibrationStartupError(
                "RGB-to-Base0 geometry bridge is missing; rebuild "
                f"rgb_base0_geometry_bridge: {self.bridge_path}"
            )

        try:
            self._library = ctypes.CDLL(str(self.bridge_path))
            self._configure_functions()
            error = ctypes.create_string_buffer(self._ERROR_CAPACITY)
            succeeded = self._library.rgb_base0_geometry_create(
                str(self.calibration_path), ctypes.byref(self._handle), error, len(error)
            )
            if not succeeded or not self._handle.value:
                raise CalibrationStartupError(self._decode_error(error))
            self._read_profile()
        except CalibrationStartupError:
            self.close()
            raise
        except (AttributeError, OSError, ValueError) as error:
            self.close()
            raise CalibrationStartupError(
                f"Cannot initialize RGB-to-Base0 geometry bridge {self.bridge_path}: {error}"
            ) from error

    def _configure_functions(self):
        library = self._library
        library.rgb_base0_geometry_create.argtypes = [
            ctypes.c_wchar_p, ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_char), ctypes.c_size_t,
        ]
        library.rgb_base0_geometry_create.restype = ctypes.c_int
        library.rgb_base0_geometry_destroy.argtypes = [ctypes.c_void_p]
        library.rgb_base0_geometry_destroy.restype = None
        library.rgb_base0_geometry_profile.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_char), ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_char), ctypes.c_size_t,
        ]
        library.rgb_base0_geometry_profile.restype = ctypes.c_int
        library.rgb_base0_geometry_project.argtypes = [
            ctypes.c_void_p, ctypes.c_double, ctypes.c_double,
            ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_char), ctypes.c_size_t,
        ]
        library.rgb_base0_geometry_project.restype = ctypes.c_int

    @staticmethod
    def _decode_error(buffer):
        message = buffer.value.decode("utf-8", errors="replace").strip()
        return message or "C++ RGB-to-Base0 geometry operation failed without diagnostics"

    def _read_profile(self):
        width = ctypes.c_int()
        height = ctypes.c_int()
        fps = ctypes.c_int()
        target_z = ctypes.c_double()
        serial = ctypes.create_string_buffer(self._SERIAL_CAPACITY)
        error = ctypes.create_string_buffer(self._ERROR_CAPACITY)
        succeeded = self._library.rgb_base0_geometry_profile(
            self._handle,
            ctypes.byref(width), ctypes.byref(height), ctypes.byref(fps),
            ctypes.byref(target_z), serial, len(serial), error, len(error),
        )
        if not succeeded:
            raise CalibrationStartupError(self._decode_error(error))
        self.width = width.value
        self.height = height.value
        self.fps = fps.value
        self.profile_format = "MJPG"
        self.target_z_mm = target_z.value
        self.camera_serial_number = serial.value.decode("utf-8", errors="strict")
        if (
            self.width <= 0 or self.height <= 0 or self.fps <= 0
            or not math.isfinite(self.target_z_mm) or not self.camera_serial_number
        ):
            raise CalibrationStartupError("C++ bridge returned invalid calibration profile metadata")

    def project(self, u, v):
        if not self._handle.value:
            raise RuntimeError("RGB-to-Base0 geometry handle is closed")
        u = float(u)
        v = float(v)
        if not math.isfinite(u) or not math.isfinite(v):
            raise ValueError("Pixel center must be finite")

        base0_x = ctypes.c_double()
        base0_y = ctypes.c_double()
        base0_z = ctypes.c_double()
        error = ctypes.create_string_buffer(self._ERROR_CAPACITY)
        succeeded = self._library.rgb_base0_geometry_project(
            self._handle, u, v,
            ctypes.byref(base0_x), ctypes.byref(base0_y), ctypes.byref(base0_z),
            error, len(error),
        )
        if not succeeded:
            raise ValueError(self._decode_error(error))
        result = (base0_x.value, base0_y.value, base0_z.value)
        if not all(math.isfinite(value) for value in result):
            raise ValueError("C++ geometry returned a non-finite Base0 point")
        if abs(result[2] - self.target_z_mm) > 1e-7:
            raise ValueError("C++ geometry returned a point on the wrong target Z plane")
        return result

    def close(self):
        handle = getattr(self, "_handle", None)
        library = getattr(self, "_library", None)
        if handle is not None and handle.value and library is not None:
            library.rgb_base0_geometry_destroy(handle)
            handle.value = None

    def __enter__(self):
        return self

    def __exit__(self, _exception_type, _exception, _traceback):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
