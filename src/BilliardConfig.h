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

struct PhysicalRailConfig {
    RailId id;
    PocketId startPocket;  // 庫邊起點對應袋口，實際座標於每輪Vision週期查表取得。
    PocketId endPocket;  // 庫邊終點對應袋口。
    double startExclusionMm;  // 起點不可反彈區。
    double endExclusionMm;  // 終點不可反彈區。
    double cushionInsetMm;  // 緩衝墊壓縮造成的碰撞面內縮量，沿inwardUnitNormal方向，疊加於ballRadiusMm之後。
};

struct TableGeometryConfig {
    std::string calibrationRevision;  // 本組球桌標定版本。
    AxisAlignedBounds2D physicalPlayingSurface;  // Base0球桌XY邊界。
    double ballRadiusMm;  // 球半徑。
    double ballDiameterMm;  // 球直徑。
    double collisionMarginMm;  // 碰撞額外安全距離。
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
    double kickRailAngleRisk;  // Kick碰庫角風險成本。

    [[nodiscard]] double sum() const noexcept
    {
        return kickPenalty + cuttingAngle + totalDistance + clearanceRisk +
            kickRailAngleRisk;
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

// 版本化／核准的準備（standby）姿態Joint reference；H/P流程PTP的唯一權威來源。
// jointDeg可先保存使用者已確認的關節角度；calibrationRevision在使用者核准
// 正式追蹤標籤前必須保持nullopt，isValid()須fail closed，不得以非空字串
// 佔位充當已核准版本。
struct StandbyJointReference {
    std::optional<std::string> calibrationRevision;
    std::array<double, 6> jointDeg;

    [[nodiscard]] bool isValid() const noexcept;
};

// 單一shot-cycle競賽計時契約：15秒deadline、5秒execution reserve、10秒
// planning retry cutoff。三者必須維持deadline >= retryCutoff且
// (deadline - retryCutoff) >= reserve，確保重拍截止後仍留有足夠執行預留。
struct ShotCycleTimingConfig {
    unsigned long shotDeadlineMs;
    unsigned long minimumExecutionReserveMs;
    unsigned long planningRetryCutoffMs;

    [[nodiscard]] bool isValid() const noexcept;
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
    std::optional<double> maxExecutableKickRailAngleDeg;
};

struct FixedForceEnvelopeConfig {
    std::string calibrationRevision;
    FixedForceEnvelopeLimits directPot;
    FixedForceEnvelopeLimits kickPot;
    FixedForceEnvelopeLimits directLegalContact;
    FixedForceEnvelopeLimits kickLegalContact;
    FixedForceEnvelopeLimits cueBallContactOnly;
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
    // 從nominal strike XY沿擊球方向微調；Push為+b*d，Pull為-b*d。
    std::optional<double> strikePositionBiasMm;
    // 人工可調／實驗門檻：母球中心距physicalPlayingSurface下沿超過此值
    // 且擊球方向朝tableDownDirectionBase0XY時選Pull。
    std::optional<double> pullModeMinBottomDistanceMm = 300.0;
    // 已確認Robot正對球桌長邊且無平面旋轉偏移，球桌往下即Base0 Y-。
    std::optional<Vector2D> tableDownDirectionBase0XY = Vector2D{0.0, -1.0};
    // safe-lift高度唯一權威為ExecutionPlan::safeApproachPose.z；不在此重複設定。
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
    // 唯一Tool1的Push擊球方向；已確認為Tool1 local +X。Pull使用其反向local -X。
    std::optional<std::array<double, 3>> cueForwardAxisTool;
    std::optional<double> maxCueDirectionErrorDeg;  // Base0 XY投影允許方向誤差。
    std::optional<double> directionUnitTolerance;  // ShotPlan單位向量容差。
    std::optional<std::string> executionPolicyRevision;  // 獨立版本化的執行政策識別。
    std::optional<ExecutionPolicyMode> policyMode;
    std::optional<bool> legalContactExecutionAuthorized;  // 預設及real hardware必須false。
    std::optional<bool> productionLegalContactFallbackAuthorized;
    // 固定力度只作可執行範圍gate；P2-01不執行氣動命令。
    std::optional<FixedForceEnvelopeConfig> fixedForceEnvelope;
    // 僅保存後續executor所需的版本化timing reference。
    std::optional<PneumaticTimingProfileReference> pneumaticTimingProfile;
    std::optional<std::string> tool1ControllerCalibrationRevision;
    // 貼庫安全繞行：母球中心距最近庫邊小於此值時，改用平行庫邊方向而非
    // Phase1算出的入袋方向。nullopt＝功能關閉（維持今天的行為）。
    std::optional<double> railHuggingTriggerDistanceMm;
    // 貼庫安全繞行專用的strikeReady offset，取代（不是疊加）一般
    // strikePositionBiasMm；貼庫時力道需求跟一般擊球不同，用獨立參數
    // 才不會互相牽動。nullopt＝功能關閉（沿用strikePositionBiasMm）。
    std::optional<double> railHuggingReadyGapMm;
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
    int tool1Number;  // 唯一Tool1；Push／Pull共用同一個controller TCP。
    std::optional<std::string> base0CalibrationRevision;
    std::optional<std::string> tool1ControllerCalibrationRevision;
    std::optional<HrSdkAngleMappingConfig> angleMapping;
    std::optional<std::string> safeUpCalibrationRevision;
    // ExecutionPolicy獨立要求的三個revision，必須逐一符合deployment校正。
    std::optional<std::string> requiredTool1CalibrationRevision;
    std::optional<std::string> requiredAbcMappingRevision;
    std::optional<std::string> requiredSafeUpCalibrationRevision;
    std::optional<bool> base0PositiveZSafeConfirmed;
    std::optional<int> extendDoIndex;  // DO1：striker伸出pulse。
    std::optional<int> retractDoIndex;  // DO2：striker收回pulse。
    std::optional<PneumaticTimingProfileReference> approvedTimingProfile;
    // 競賽用實體Start按鈕，接在此DI index（例如DI1）；功能等同鍵盤H，
    // 觸發同一套pollStartControl edge-gate。nullopt＝功能關閉，只用H鍵。
    std::optional<int> startDigitalInputIndex;
};

// 連線：人工部署的控制器、視覺服務與連接埠。
extern const char* const ARM_IP;  // 手臂控制器IP。
extern const char* const VISION_SERVER_IP;  // 視覺服務IP。
extern const int VISION_SERVER_PORT;  // 32值CSV服務連接埠。
extern const int CALIBRATION_SERVER_PORT;  // 標定服務連接埠。

extern const int NORMAL_SPEED_RATIO;
// 加減速比／PTP速度／LIN速度：HRSDK可設定但目前完全沒被呼叫過，手臂沿用
// 出廠或教導器上次手動設定值。nullopt＝不主動設定（維持現狀，不改變
// 行為）；要調整請填入實測後確認安全的數值。
extern const std::optional<int> ACC_DEC_RATIO;
extern const std::optional<int> PTP_SPEED;
extern const std::optional<double> LIN_SPEED;
extern const int TOOL_NUMBER;
extern const int BASE_NUMBER;
extern const int START_BUTTON_DI_INDEX;
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

extern const unsigned long CAMERA_SETTLE_MS;
extern const unsigned long MOTION_START_CONFIRMATION_TIMEOUT_MS;
extern const unsigned long MOTION_TIMEOUT_MS;
extern const unsigned long MOTION_POLL_INTERVAL_MS;
extern const unsigned long MOTOR_OFF_CONFIRMATION_TIMEOUT_MS;

// 推桿後方障礙檢查（rear-obstacle check）：不是完整Tool掃掠體積模型，
// 只是「母球中心沿執行方向反方向Lback=ballRadiusMm+BACK_OBSTACLE_EXTRA_MM
// 的有限線段」跟其他球中心的最短距離門檻。之後量到實際pusherRadiusMm，
// 可把BACK_OBSTACLE_LATERAL_MARGIN_MM的門檻改成
// ballRadiusMm+pusherRadiusMm+margin，不需重新設計。
extern const double BACK_OBSTACLE_EXTRA_MM;
extern const double BACK_OBSTACLE_LATERAL_MARGIN_MM;

// [演算法研究值，非實測]
// CueBallContactOnly保底360度方向搜尋的角度間距。
extern const double CUE_BALL_CONTACT_ONLY_ANGULAR_STEP_DEG;

// [演算法研究值，非實測]
// Legal Contact擦撞（grazing）角度掃描間距：head-on以外，往左右每隔
// 這個角度多算一個接觸點候選，直到切線極限（超過就會先撞到目標球
// 本體，不是真的碰到那個角度）。
extern const double LEGAL_CONTACT_GRAZING_ANGULAR_STEP_DEG;

// [演算法研究值，非實測]
// Kick Legal Contact擦撞（grazing）角度掃描間距：對每個rail的head-on kick
// 候選以外，以鏡射母球到目標的方向為基準，往左右每隔這個角度多算一個
// 接觸點候選，直到切線極限。跟LEGAL_CONTACT_GRAZING_ANGULAR_STEP_DEG各自
// 獨立設定，未來可分開校正。
extern const double LEGAL_CONTACT_KICK_GRAZING_ANGULAR_STEP_DEG;

extern const std::array<double, 6> CAMERA_JOINT;
extern const double CAMERA_JOINT_TOLERANCE_DEG;

// 準備（standby）姿態：核准流程唯一authoritative joint reference。
extern const StandbyJointReference STANDBY_JOINT_REFERENCE;
// [已核准 / 人工可調研究值] 六軸都在此誤差內時，視為已位於standby姿態。
// 獨立於CAMERA_JOINT_TOLERANCE_DEG，即使初始值相同也不得共用同一常數。
extern const double STANDBY_JOINT_TOLERANCE_DEG;
// [已核准 / 人工可調研究值] Vision reconnect gate每次重試連線之間的等待間隔。
extern const unsigned long VISION_RECONNECT_POLL_INTERVAL_MS;

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

// [暫定競賽參數] 15秒shot deadline／5秒execution reserve／10秒planning retry
// cutoff。minimumExecutionReserveMs為人工可調研究初值，須以實機最慢執行時間
// 校正；不是固定機械規格。production計時必須使用單調時鐘量測，本設定僅提供
// 門檻數值。
extern const ShotCycleTimingConfig SHOT_CYCLE_TIMING;

}  // namespace BilliardConfig
