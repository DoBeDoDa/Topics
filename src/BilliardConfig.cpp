// 定義手臂連線、速度、Tool、關節點位及正式／測試動作參數。
#include "BilliardConfig.h"

namespace BilliardConfig {

const char* const ARM_IP = "192.168.0.1";
const char* const VISION_SERVER_IP = "127.0.0.1";
const int VISION_SERVER_PORT = 12345;
const int CALIBRATION_SERVER_PORT = 12347;

const int NORMAL_SPEED_RATIO = 20;
const int TOOL_NUMBER = 1;  // 使用控制器中定義的 Tool 1
const int BASE_NUMBER = 0;  // 視覺校正與動作座標共同使用的 Base 0
const int PNEUMATIC_OUTPUT = 1;

const double BALL_DIAMETER_MM = 49.52;  // 撞球直徑
const double YAW_OFFSET_DEG = 0.0;      // RZ 瞄準角的固定補償量
const double MIN_AIM_DISTANCE_MM = 5.0; // 拒絕建立路徑的最小瞄準向量長度
const double MAX_REACH_RADIUS_MM = 850.0;

const double CAMERA_OFFSET_X_MM = 0.0;
const double CAMERA_OFFSET_Y_MM = 0.0;
const double CAMERA_REFERENCE_X_MM = -400.0;
const double CAMERA_REFERENCE_Y_MM = 600.0;
const double CAMERA_COMPENSATION_KX = 0.0;
const double CAMERA_COMPENSATION_KY = 0.0;

const unsigned long CAMERA_SETTLE_MS = 800;
const unsigned long TRANSIT_SETTLE_MS = 500;
const unsigned long MOTION_TIMEOUT_MS = 60000;
const unsigned long MOTION_POLL_INTERVAL_MS = 50;

const std::array<double, 6> CAMERA_JOINT = {
    0.0, -11.049, 28.921, 0.0, -15.574, -90.0
};  // 拍照關節點 {A1, A2, A3, A4, A5, A6}

const std::array<double, 6> TRANSIT_JOINT = {
    -5.0, -53.0, 8.0, -3.62, -46.497, 0.0
};  // 中繼關節點 {A1, A2, A3, A4, A5, A6}

const MotionProfile PRODUCTION_MOTION = {
    -216.0,    // strikeZ：實際擊球高度
    -160.0,    // safeZ：正式擊球前的預備高度
    0.0,       // rxDeg
    15.0,      // tiltRyDeg
    20.0,      // moveBackMm
    30.0       // standoffExtraMm
};

const MotionProfile TEST_MOTION = {
    -140.0,  // strikeZ：測試時不下降至實際擊球高度
    -150.0,  // safeZ：測試點位統一使用此高度
    0.0,     // rxDeg
    15.0,    // tiltRyDeg
    0.0,     // moveBackMm
    0.0      // standoffExtraMm
};  // 測試模式動作參數

}  // namespace BilliardConfig
