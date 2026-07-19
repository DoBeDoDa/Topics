#include "BilliardConfig.h"

namespace BilliardConfig {

const char* const ARM_IP = "192.168.0.1";
const char* const VISION_SERVER_IP = "127.0.0.1";
const int VISION_SERVER_PORT = 12345;
const int ALIGN_SERVER_PORT = 12346;
const int CALIBRATION_SERVER_PORT = 12347;

const int NORMAL_SPEED_RATIO = 20;
const int ALIGN_SPEED_RATIO = 5;
const int TOOL_NUMBER = 1;
const int PNEUMATIC_OUTPUT = 1;

const double BALL_DIAMETER_MM = 49.52;
const double YAW_OFFSET_DEG = 0.0;
const double MIN_AIM_DISTANCE_MM = 5.0;
const double MAX_REACH_RADIUS_MM = 850.0;

const double CAMERA_OFFSET_X_MM = 0.0;
const double CAMERA_OFFSET_Y_MM = 0.0;
const double CAMERA_REFERENCE_X_MM = -400.0;
const double CAMERA_REFERENCE_Y_MM = 600.0;
const double CAMERA_COMPENSATION_KX = 0.0;
const double CAMERA_COMPENSATION_KY = 0.0;

const double ALIGN_TOLERANCE_PX = 3.0;
const double ALIGN_KP = 0.05;
const double ALIGN_MAX_STEP_MM = 10.0;

const unsigned long CAMERA_SETTLE_MS = 800;
const unsigned long TRANSIT_SETTLE_MS = 500;

const std::array<double, 6> CAMERA_JOINT = {
    0.0, -33.564, 49.53, 0.0, -15.574, -90.0
};

const std::array<double, 6> TRANSIT_JOINT = {
    -5.0, -53.0, 8.0, -3.62, -46.497, -130.0
};

const MotionProfile PRODUCTION_MOTION = {
    -319.769,  // strikeZ
    0.0,       // safeZ
    0.0,       // rxDeg
    10.0,      // tiltRyDeg
    20.0,      // moveBackMm
    30.0       // standoffExtraMm
};

const MotionProfile TEST_MOTION = {
    -290.0,  // strikeZ：測試時保留桌面安全距離
    -125.0,  // safeZ
    5.0,     // rxDeg
    20.0,    // tiltRyDeg
    0.0,     // moveBackMm
    0.0      // standoffExtraMm
};

}  // namespace BilliardConfig
