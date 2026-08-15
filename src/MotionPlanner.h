// 將有效Phase 1 ShotPlan轉成離線可驗證的Robot動作計畫；不執行硬體。
#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>

#include "BilliardConfig.h"
#include "TableState.h"

struct RobotPoseABC {
    double x;
    double y;
    double z;
    double a;
    double b;
    double c;

    [[nodiscard]] bool isFinite() const noexcept;
};

// safe-lift高度唯一權威是ExecutionPlan::safeApproachPose.z；不存在第二套
// 高度來源。buildSafeLiftTarget()是建立safe-lift target的唯一方式，
// isValidSafeLiftTarget()是執行前的fail-closed驗證。
[[nodiscard]] RobotPoseABC buildSafeLiftTarget(
    const RobotPoseABC& postStrikeActualPose,
    double safeApproachPoseZ) noexcept;

[[nodiscard]] bool isValidSafeLiftTarget(
    const RobotPoseABC& postStrikeActualPose,
    double safeApproachPoseZ,
    const RobotPoseABC& safeLiftTarget) noexcept;

enum class PlannedMotionIntent {
    CartesianPtpToSafeApproach,
    LinearToStrikeReady,
    RuntimeActualPoseVerticalSafeLift,
    JointPtpToStandby
};

enum class ExecutionPolicyDecision {
    PotAccepted,
    LegalContactExplicitlyAuthorized,
    LegalContactPlanningTestSelected,
    LegalContactProductionFallbackAccepted
};

enum class StrikeMode {
    Push,
    Pull
};

struct FixedForceEnvelopeEvaluation {
    std::string calibrationRevision;
    double totalPathLengthMm;
    std::optional<double> cuttingAngleDeg;
    std::optional<double> kickRailAngleDeg;
    double minTotalPathLengthMm;
    double maxTotalPathLengthMm;
    std::optional<double> maxCuttingAngleDeg;
    std::optional<double> maxExecutableKickRailAngleDeg;

    [[nodiscard]] bool isValid() const noexcept;
};

enum class PlannedStagePrecondition {
    PreviousStageSucceeded,
    PolicyAndCalibrationAccepted,
    RuntimeActualPoseAndPneumaticCompletion
};

enum class PlannedStageSuccessCondition {
    TargetReachedAndStopped,
    LinearTargetReachedAndStopped,
    SafeLiftReachedAndStopped
};

enum class PlannedStageFailureTransition {
    StopFailClosed
};

enum class PlannedPathCheck {
    ApprovedPtpPolicyAndTargetReachability,
    MotionCheckLinRequired
};

struct PlannedStageContract {
    PlannedMotionIntent intent;
    PlannedStagePrecondition precondition;
    PlannedStageSuccessCondition successCondition;
    PlannedStageFailureTransition failureTransition;
    PlannedPathCheck pathCheck;

    [[nodiscard]] bool isValid() const noexcept;
};

struct PoseSearchAudit {
    double a0Deg;
    double b0Deg;
    double deltaADeg;
    double deltaBDeg;
    double stepADeg;
    double stepBDeg;
    BilliardConfig::PoseSearchOrder searchOrder;
    BilliardConfig::AxisOffsetOrder axisOffsetOrder;
    BilliardConfig::PoseTieBreak tieBreak;
    std::size_t evaluatedCandidateCount;

    [[nodiscard]] bool contains(
        double selectedADeg,
        double selectedBDeg,
        std::size_t selectedOrdinal) const noexcept;
};

struct ExecutionPlan {
    Phase1PlanIdentity sourcePlanIdentity;
    ShotPlanType sourceShotType;
    std::string base0PlanarCalibrationRevision;
    std::string tableGeometryRevision;
    std::string motionCalibrationRevision;
    std::string cueForwardAxisCalibrationRevision;
    Point cueBallCenterBase0Mm;
    Vector2D shotDirectionXY;
    StrikeMode strikeMode;
    AxisAlignedBounds2D physicalPlayingSurfaceBase0Mm;
    Vector2D tableDownDirectionBase0XY;
    double pullModeMinBottomDistanceMm;
    double bottomDistanceMm;
    double tableDownDirectionDot;
    double strikePositionBiasMm;
    double ballRadiusMm;
    double readyGapMm;
    double directionUnitTolerance;
    double cToolOffsetDeg;
    Vector2D validatedStrikeDirectionXY;
    double cueDirectionErrorDeg;
    double maxCueDirectionErrorDeg;
    RobotPoseABC safeApproachPose;
    RobotPoseABC strikeReadyPose;
    double selectedADeg;
    double selectedBDeg;
    double selectedCDeg;
    std::size_t selectedSearchOrdinal;
    PoseSearchAudit poseSearchAudit;
    std::string standbyJointCalibrationRevision;
    std::array<double, 6> standbyJointReference;
    std::array<PlannedMotionIntent, 4> motionIntents;
    std::array<PlannedStageContract, 4> stageContracts;
    FixedForceEnvelopeEvaluation fixedForceEnvelope;
    BilliardConfig::PneumaticTimingProfileReference pneumaticTimingProfile;
    std::string executionPolicyRevision;
    BilliardConfig::ExecutionPolicyMode policyMode;
    ExecutionPolicyDecision policyDecision;
    int tool1Number = BilliardConfig::TOOL_NUMBER;
    std::string tool1ControllerCalibrationRevision;
    bool rankedPotCandidatesExhausted = false;

    [[nodiscard]] bool isValid() const noexcept;
};

enum class ExecutionPlanStatus {
    Success,
    ConfigurationMissing,
    InvalidConfiguration,
    InvalidShotPlan,
    InvalidExecutionPlan,
    NoExecutablePlan
};

enum class ExecutionPlanFailureReason {
    None,
    MissingMotionCalibration,
    MissingTableGeometry,
    MissingValidationSeam,
    CalibrationRevisionMismatch,
    TableGeometryRevisionMismatch,
    InvalidTableGeometry,
    InvalidMotionCalibration,
    InputIsNotShotPlan,
    InvalidShotPlanContract,
    InvalidShotDirection,
    LegalContactNotAuthorized,
    FixedForceEnvelopeRejected,
    InvalidExecutionPlanValue,
    NoAcceptedPoseCandidate,
    NumericalFailure,
    ReachabilityCheckFailed
};

struct ExecutionPlanDiagnostic {
    ExecutionPlanFailureReason reason;
    std::size_t evaluatedPoseCandidates;
};

class ExecutionPlanResult {
public:
    [[nodiscard]] static ExecutionPlanResult success(ExecutionPlan value);
    [[nodiscard]] static ExecutionPlanResult rejected(
        ExecutionPlanStatus status,
        ExecutionPlanFailureReason reason,
        std::size_t evaluatedPoseCandidates = 0);

    [[nodiscard]] ExecutionPlanStatus status() const noexcept;
    [[nodiscard]] const std::optional<ExecutionPlan>& value() const noexcept;
    [[nodiscard]] const std::optional<ExecutionPlanDiagnostic>& diagnostic()
        const noexcept;
    [[nodiscard]] bool isValid() const noexcept;

private:
    ExecutionPlanResult(
        ExecutionPlanStatus status,
        std::optional<ExecutionPlan> value,
        std::optional<ExecutionPlanDiagnostic> diagnostic);

    ExecutionPlanStatus status_;
    std::optional<ExecutionPlan> value_;
    std::optional<ExecutionPlanDiagnostic> diagnostic_;
};

struct MotionPlanningChecks {
    // 回傳nullopt代表檢查本身失敗；false代表該Pose不可接受。
    std::function<std::optional<bool>(const RobotPoseABC&)> poseAccepted;
    // 此callback封裝已核准的Tool A/B/C旋轉語意；P2-01不得自行猜Euler順序。
    std::function<std::optional<Vector2D>(
        const RobotPoseABC&,
        const std::array<double, 3>&)> projectCueForwardAxisToBase0XY;
    // 回傳nullopt代表LIN檢查本身失敗；false代表該路徑不可接受。
    std::function<std::optional<bool>(
        const RobotPoseABC&,
        const RobotPoseABC&)> linearPathAccepted;
};

class MotionPlanner {
public:
    [[nodiscard]] ExecutionPlanResult createExecutionPlan(
        const PlanningResult& planningResult,
        const std::optional<BilliardConfig::TableGeometryConfig>& tableGeometry,
        const std::optional<BilliardConfig::MotionPlanningConfig>& config,
        const MotionPlanningChecks& checks,
        bool rankedPotCandidatesExhausted = false,
        const std::optional<ResolvedTableGeometry>& resolvedTableGeometry =
            std::nullopt) const;

#ifdef BILLIARDS_P2_01_TEST_SEAM
    [[nodiscard]] static std::optional<double> directionToCDegForTest(
        Vector2D direction,
        double cToolOffsetDeg,
        double unitTolerance) noexcept;
#endif
};
