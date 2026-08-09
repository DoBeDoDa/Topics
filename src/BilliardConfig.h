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
    Vector2D outwardUnitNormal;  // 袋口朝桌外方向。
    double virtualTargetOffsetMm;  // 虛擬袋口目標外移距離。
    Segment2D pocketExitSegmentOffsetsFromEntrance;  // 有效出口線段。
    double corridorHalfWidthMm;  // 進袋走廊半寬。
    double pocketBoundaryProbeEpsilonMm;  // 袋口內外判斷小距離。
    double exitCrossingEpsilon;  // 穿越出口方向門檻。
    double maxEntryAngleDeg;  // 最大進袋角。
};

struct PhysicalRailConfig {
    RailId id;
    Segment2D segment;  // 實體庫邊端點。
    Vector2D inwardUnitNormal;  // 朝桌內方向。
    double startExclusionMm;  // 起點不可反彈區。
    double endExclusionMm;  // 終點不可反彈區。
};

struct TableGeometryConfig {
    std::string calibrationRevision;  // 本組球桌標定版本。
    AxisAlignedBounds2D physicalPlayingSurface;  // Base0球桌XY邊界。
    double ballRadiusMm;  // 球半徑。
    double ballDiameterMm;  // 球直徑。
    double collisionMarginMm;  // 碰撞額外安全距離。
    std::array<PocketModelConfig, 6> pockets;
    std::array<PhysicalRailConfig, 6> rails;
};

struct KickGeometryConfig {
    double maxKickRailAngleDeg;  // 最大碰庫角。
    double reflectionDirectionTolerance;  // 反射方向容差。
    double reflectionAngleToleranceDeg;  // 入射／反射角容差。
};

enum class PlanningMode {
    PotOnly,
    ManualResearch
};

struct ScoringWeights {
    double kickPenalty;  // 使用Kick的成本。
    double cuttingAngle;  // 切球角成本。
    double totalDistance;  // 總路徑距離成本。
    double clearanceRisk;  // 障礙淨空風險成本。
    double pocketEntryAngle;  // 進袋角成本。
    double kickRailAngleRisk;  // Kick碰庫角風險成本。

    [[nodiscard]] double sum() const noexcept
    {
        return kickPenalty + cuttingAngle + totalDistance + clearanceRisk +
            pocketEntryAngle + kickRailAngleRisk;
    }
};

struct ScoringConfig {
    ScoringWeights rawWeights;
    double effectiveWeightSumTolerance;  // 正規化權重總和容差。
    double maxCutAngleDeg;  // 切球角正規化上限。
    double minDistanceMm;  // 距離正規化下限。
    double maxDistanceMm;  // 距離正規化上限。
    double preferredClearanceMm;  // 理想安全淨空。
    double tieEpsilon;  // 近似平手容差。
    PlanningMode planningMode;  // PotOnly或ManualResearch。
};

struct BrainConfig {
    std::optional<std::string> base0PlanarCalibrationRevision;
    std::optional<KickGeometryConfig> kickGeometry;
    std::optional<ScoringConfig> scoring;
};

struct MotionProfile {
    double strikeZ;
    double safeZ;
    double rxDeg;
    double tiltRyDeg;
    double moveBackMm;
    double standoffExtraMm;
};

enum class PoseSearchOrder {
    AThenB,
    BThenA
};

enum class AxisOffsetOrder {
    LowerThenHigher,
    HigherThenLower
};

enum class PoseTieBreak {
    FirstInApprovedSearchOrder
};

enum class ExecutionPolicyMode {
    PlanningTest,
    RealHardware
};

struct FixedForceEnvelopeLimits {
    bool enabled;
    double minTotalPathLengthMm;
    double maxTotalPathLengthMm;
    std::optional<double> maxCuttingAngleDeg;
    std::optional<double> maxPocketEntryAngleDeg;
    std::optional<double> maxExecutableKickRailAngleDeg;
};

struct FixedForceEnvelopeConfig {
    std::string calibrationRevision;
    FixedForceEnvelopeLimits directPot;
    FixedForceEnvelopeLimits kickPot;
    FixedForceEnvelopeLimits directLegalContact;
    FixedForceEnvelopeLimits kickLegalContact;
};

struct PneumaticTimingProfileReference {
    std::string calibrationRevision;
    unsigned long pneumaticPulseMs;
    unsigned long directionChangeDelayMs;
    unsigned long mechanismCompletionWaitMs;
};

struct MotionPlanningConfig {
    std::optional<std::string> calibrationRevision;  // P2-01姿態標定版本。
    std::optional<std::string> base0PlanarCalibrationRevision;  // 必須與ShotPlan一致。
    std::optional<std::string> cueForwardAxisCalibrationRevision;  // Tool軸人工校正版號。
    std::optional<double> strikeZMm;  // 人工核准的擊球Z。
    std::optional<double> safeApproachZMm;  // 人工核准的接近Z。
    std::optional<double> readyGapMm;  // 縮回桿尖至母球表面的間距。
    std::optional<double> safeLiftHeightMm;  // 從runtime actual pose垂直上升量。
    std::optional<double> a0Deg;  // 人工核准A基準。
    std::optional<double> b0Deg;  // 人工核准B基準。
    std::optional<double> deltaADeg;  // A基準兩側核准範圍。
    std::optional<double> deltaBDeg;  // B基準兩側核准範圍。
    std::optional<double> stepADeg;  // A搜尋固定step。
    std::optional<double> stepBDeg;  // B搜尋固定step。
    std::optional<PoseSearchOrder> searchOrder;
    std::optional<AxisOffsetOrder> axisOffsetOrder;
    std::optional<PoseTieBreak> tieBreak;
    std::optional<double> cToolOffsetDeg;  // 擊球方向到C的人工校正offset。
    std::optional<std::array<double, 3>> cueForwardAxisTool;  // Tool-local球桿forward axis。
    std::optional<double> maxCueDirectionErrorDeg;  // Base0 XY投影允許方向誤差。
    std::optional<double> directionUnitTolerance;  // ShotPlan單位向量容差。
    std::optional<std::string> executionPolicyRevision;  // 獨立版本化的執行政策識別。
    std::optional<ExecutionPolicyMode> policyMode;
    std::optional<bool> legalContactExecutionAuthorized;  // 預設及real hardware必須false。
    // 固定力度只作可執行範圍gate；P2-01不執行氣動命令。
    std::optional<FixedForceEnvelopeConfig> fixedForceEnvelope;
    // 僅保存後續executor所需的版本化timing reference。
    std::optional<PneumaticTimingProfileReference> pneumaticTimingProfile;
};

enum class RobotAngleComponent {
    A,
    B,
    C
};

struct HrSdkAngleMappingConfig {
    std::string calibrationRevision;
    // RX／RY／RZ各自取用哪個核心A／B／C角度。
    std::array<RobotAngleComponent, 3> rxRyRzSources;
    std::array<double, 3> scales;
    std::array<double, 3> offsetsDeg;
};

struct RealHardwareExecutionConfig {
    std::optional<std::string> authorizationRevision;
    bool realHardwareExecutionEnabled;
    int baseNumber;
    int toolNumber;
    std::optional<std::string> base0CalibrationRevision;
    std::optional<std::string> tool1ControllerCalibrationRevision;
    std::optional<HrSdkAngleMappingConfig> angleMapping;
    std::optional<std::string> safeUpCalibrationRevision;
    // ExecutionPolicy獨立要求的三個revision，必須逐一符合deployment校正。
    std::optional<std::string> requiredTool1CalibrationRevision;
    std::optional<std::string> requiredAbcMappingRevision;
    std::optional<std::string> requiredSafeUpCalibrationRevision;
    std::optional<bool> base0PositiveZSafeConfirmed;
    std::optional<int> strikeDoIndex;
    std::optional<int> retractDoIndex;
    std::optional<PneumaticTimingProfileReference> approvedTimingProfile;
};

// 連線：人工部署的控制器、視覺服務與連接埠。
extern const char* const ARM_IP;  // 手臂控制器IP。
extern const char* const VISION_SERVER_IP;  // 視覺服務IP。
extern const int VISION_SERVER_PORT;  // 32值CSV服務連接埠。
extern const int CALIBRATION_SERVER_PORT;  // 標定服務連接埠。

extern const int NORMAL_SPEED_RATIO;
extern const int TOOL_NUMBER;
extern const int BASE_NUMBER;
extern const int PNEUMATIC_OUTPUT;

extern const double BALL_DIAMETER_MM;
extern const double YAW_OFFSET_DEG;
extern const double MIN_AIM_DISTANCE_MM;
extern const double MAX_REACH_RADIUS_MM;

// 影像：單筆32值CSV上限、完整資料等待時間與Base0 XY合法範圍。
extern const std::optional<std::size_t> VISION_MAX_FRAME_BYTES;  // 單筆32值CSV最大長度。
extern const std::optional<unsigned long> VISION_RECEIVE_TIMEOUT_MS;  // 等待完整資料最長時間。
extern const std::optional<AxisAlignedBounds2D> VISION_OBSERVATION_BOUNDS;  // Base0 XY合法範圍。
// 穩定：球／袋口三次偵測容差與相鄰有效資料最大時間。
extern const std::optional<double> STABLE_FRAME_TOLERANCE_MM;  // 球三次偵測最大允許偏移。
extern const std::optional<double> POCKET_STABILITY_TOLERANCE_MM;  // 袋口最大允許偏移。
extern const std::optional<unsigned long> MAX_INTER_FRAME_INTERVAL_MS;  // 相鄰有效資料最大時間。
// 球桌、Kick與評分：尚未核准時保持nullopt並fail closed。
extern const std::optional<TableGeometryConfig> TABLE_GEOMETRY;
extern const std::optional<KickGeometryConfig> KICK_GEOMETRY;
extern const ScoringWeights INITIAL_EXPERIMENTAL_SCORING_WEIGHTS;
extern const std::optional<ScoringConfig> SCORING_CONFIG;
extern const std::optional<std::string> BASE0_PLANAR_CALIBRATION_REVISION;  // Base0平面標定版本。
extern const BrainConfig BRAIN_CONFIG;

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

// P2-01：未完成人工姿態／Tool軸校正前不得建立ExecutionPlan。
extern const std::optional<MotionPlanningConfig> MOTION_PLANNING_CONFIG;
// Production runtime僅允許零硬體規劃測試與明確授權的真實硬體。
extern const ExecutionPolicyMode PRODUCTION_RUNTIME_MODE;
// P2-03：未完成人工／實機驗收前，真實motion與DO一律保持停用。
extern const std::optional<RealHardwareExecutionConfig>
    REAL_HARDWARE_EXECUTION_CONFIG;

}  // namespace BilliardConfig
