// 宣告全專案共用的連線、幾何、速度、點位與動作設定。
#pragma once

#include <array>

namespace BilliardConfig {

struct MotionProfile {
    double strikeZ;
    double safeZ;
    double rxDeg;
    double tiltRyDeg;
    double moveBackMm;
    double standoffExtraMm;
};

extern const char* const ARM_IP;
extern const char* const VISION_SERVER_IP;
extern const int VISION_SERVER_PORT;
extern const int CALIBRATION_SERVER_PORT;

extern const int NORMAL_SPEED_RATIO;
extern const int TOOL_NUMBER;
extern const int BASE_NUMBER;
extern const int PNEUMATIC_OUTPUT;

extern const double BALL_DIAMETER_MM;
extern const double YAW_OFFSET_DEG;
extern const double MIN_AIM_DISTANCE_MM;
extern const double MAX_REACH_RADIUS_MM;

extern const double CAMERA_OFFSET_X_MM;
extern const double CAMERA_OFFSET_Y_MM;
extern const double CAMERA_REFERENCE_X_MM;
extern const double CAMERA_REFERENCE_Y_MM;
extern const double CAMERA_COMPENSATION_KX;
extern const double CAMERA_COMPENSATION_KY;

extern const unsigned long CAMERA_SETTLE_MS;
extern const unsigned long TRANSIT_SETTLE_MS;
extern const unsigned long MOTION_TIMEOUT_MS;
extern const unsigned long MOTION_POLL_INTERVAL_MS;

extern const std::array<double, 6> CAMERA_JOINT;
extern const std::array<double, 6> TRANSIT_JOINT;

// 正式擊球使用的高度與姿態。
extern const MotionProfile PRODUCTION_MOTION;

// 定位測試刻意與桌面保持距離，避免末端工具碰撞桌面。
extern const MotionProfile TEST_MOTION;

}  // namespace BilliardConfig
