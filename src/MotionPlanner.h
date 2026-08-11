// 將有效Phase 1 ShotPlan轉成離線可驗證的Robot動作計畫；不執行硬體。
#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>

#include "BilliardConfig.h"
#include "TableState.h"

// P2-02尚未遷移的既有執行相容型別；P2-01不再產生此型別。
struct MotionPlan {
    std::array<double, 6> transitJoint;
    std::array<double, 6> readyPose;
    std::array<double, 6> strikePose;
    double aimAngleDeg = 0.0;
};

struct RobotPoseABC {
    double x;
    double y;
    double z;
    double a;
    double b;
    double c;

    [[nodiscard]] bool isFinite() const noexcept;
};

enum class SafeLiftDerivation {
    RuntimeActualPoseKeepXYABCIncreaseZ
};

struct SafeLiftDerivationRule {
    SafeLiftDerivation derivation;
    double heightMm;

    [[nodiscard]] bool isValid() const noexcept;
};

enum class PlannedMotionIntent {
    JointPtpToTransit,
    CartesianPtpToSafeApproach,
    LinearToStrikeReady,
    RuntimeActualPoseVerticalSafeLift,
    JointPtpToCamera
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
    SafeLiftDerivationRule safeLiftRule;
    std::array<double, 6> transitJointReference;
    std::array<double, 6> cameraJointReference;
    std::array<PlannedMotionIntent, 5> motionIntents;
    std::array<PlannedStageContract, 5> stageContracts;
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
    NumericalFailure
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
};

class MotionPlanner {
public:
    [[nodiscard]] ExecutionPlanResult createExecutionPlan(
        const PlanningResult& planningResult,
        const std::optional<BilliardConfig::TableGeometryConfig>& tableGeometry,
        const std::optional<BilliardConfig::MotionPlanningConfig>& config,
        const MotionPlanningChecks& checks,
        bool rankedPotCandidatesExhausted = false) const;

#ifdef BILLIARDS_P2_01_TEST_SEAM
    [[nodiscard]] static std::optional<double> directionToCDegForTest(
        Vector2D direction,
        double cToolOffsetDeg,
        double unitTolerance) noexcept;
#endif
};
