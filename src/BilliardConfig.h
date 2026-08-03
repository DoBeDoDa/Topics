// 宣告全專案共用的連線、幾何、速度、點位與動作設定。
#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>

#include "Point.h"

namespace BilliardConfig {

enum class PocketId : std::size_t {
    Pocket1,
    Pocket2,
    Pocket3,
    Pocket4,
    Pocket5,
    Pocket6
};

enum class RailId : std::size_t {
    Rail1,
    Rail2,
    Rail3,
    Rail4,
    Rail5,
    Rail6
};

enum class PocketType {
    Corner,
    Side
};

struct PocketModelConfig {
    PocketId id;
    PocketType type;
    Vector2D outwardUnitNormal;
    double virtualTargetOffsetMm;
    Segment2D pocketExitSegmentOffsetsFromEntrance;
    double corridorHalfWidthMm;
    double pocketBoundaryProbeEpsilonMm;
    double exitCrossingEpsilon;
    double maxEntryAngleDeg;
};

struct PhysicalRailConfig {
    RailId id;
    Segment2D segment;
    Vector2D inwardUnitNormal;
    double startExclusionMm;
    double endExclusionMm;
};

struct TableGeometryConfig {
    std::string calibrationRevision;
    AxisAlignedBounds2D physicalPlayingSurface;
    double ballRadiusMm;
    double ballDiameterMm;
    double collisionMarginMm;
    std::array<PocketModelConfig, 6> pockets;
    std::array<PhysicalRailConfig, 6> rails;
};

struct KickGeometryConfig {
    double maxKickRailAngleDeg;
    double reflectionDirectionTolerance;
    double reflectionAngleToleranceDeg;
};

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

extern const std::optional<std::size_t> VISION_MAX_FRAME_BYTES;
extern const std::optional<unsigned long> VISION_RECEIVE_TIMEOUT_MS;
extern const std::optional<AxisAlignedBounds2D> VISION_OBSERVATION_BOUNDS;
extern const std::optional<double> STABLE_FRAME_TOLERANCE_MM;
extern const std::optional<double> POCKET_STABILITY_TOLERANCE_MM;
extern const std::optional<unsigned long> MAX_INTER_FRAME_INTERVAL_MS;
extern const std::optional<TableGeometryConfig> TABLE_GEOMETRY;
extern const std::optional<KickGeometryConfig> KICK_GEOMETRY;

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
