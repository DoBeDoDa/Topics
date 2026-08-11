// 將Phase 1 ShotPlan原地轉成ExecutionPlan；本檔不連結或呼叫HRSDK。
#include "MotionPlanner.h"

#include "MathUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double RANGE_TOLERANCE = 1e-9;
constexpr std::size_t MAX_AXIS_CANDIDATES = 10001;

bool finiteArray(const std::array<double, 6>& values) noexcept
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

bool validBounds(const AxisAlignedBounds2D& bounds) noexcept
{
    return std::isfinite(bounds.minX) && std::isfinite(bounds.maxX) &&
        std::isfinite(bounds.minY) && std::isfinite(bounds.maxY) &&
        bounds.minX < bounds.maxX && bounds.minY < bounds.maxY;
}

double dot(Vector2D first, Vector2D second) noexcept
{
    return first.x * second.x + first.y * second.y;
}

StrikeMode resolveStrikeMode(
    double bottomDistanceMm,
    double pullModeMinBottomDistanceMm,
    double tableDownDirectionDot) noexcept
{
    return bottomDistanceMm > pullModeMinBottomDistanceMm &&
            tableDownDirectionDot > 0.0
        ? StrikeMode::Pull
        : StrikeMode::Push;
}

std::array<double, 3> strikeAxisForMode(
    const std::array<double, 3>& pushAxis,
    StrikeMode mode) noexcept
{
    if (mode == StrikeMode::Push) return pushAxis;
    return {-pushAxis[0], -pushAxis[1], -pushAxis[2]};
}

double normalizeAngleDeg(double angle) noexcept
{
    if (!std::isfinite(angle)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double normalized = std::fmod(angle + 180.0, 360.0);
    if (normalized < 0.0) normalized += 360.0;
    return normalized - 180.0;
}

std::optional<double> directionToCDeg(
    Vector2D direction,
    double cToolOffsetDeg,
    double unitTolerance) noexcept
{
    const double length = std::hypot(direction.x, direction.y);
    if (!BilliardMath::isFinite(direction) || !std::isfinite(length) ||
        length <= 0.0 || !std::isfinite(cToolOffsetDeg) ||
        !std::isfinite(unitTolerance) || unitTolerance <= 0.0 ||
        std::fabs(length - 1.0) > unitTolerance) {
        return std::nullopt;
    }
    const double value = normalizeAngleDeg(
        std::atan2(direction.y, direction.x) * 180.0 / PI + cToolOffsetDeg);
    return std::isfinite(value) ? std::optional<double>{value} : std::nullopt;
}

bool isLegalContact(ShotPlanType type) noexcept
{
    return type == ShotPlanType::DirectLegalContact ||
        type == ShotPlanType::KickLegalContact;
}

bool isProductionLegalContactFallback(const ShotPlan& plan) noexcept
{
    if (const auto* direct =
            std::get_if<DirectLegalContactShotPlanPayload>(&plan.payload)) {
        return direct->audit.activationAuthority ==
            LegalContactAuditFields::ActivationAuthority::
                ProductionFallbackEligible;
    }
    if (const auto* kick =
            std::get_if<KickLegalContactShotPlanPayload>(&plan.payload)) {
        return kick->audit.activationAuthority ==
            LegalContactAuditFields::ActivationAuthority::
                ProductionFallbackEligible;
    }
    return false;
}

bool isPot(ShotPlanType type) noexcept
{
    return type == ShotPlanType::DirectPot || type == ShotPlanType::KickPot;
}

struct ShotExecutionMetrics {
    double totalPathLengthMm;
    std::optional<double> cuttingAngleDeg;
    std::optional<double> kickRailAngleDeg;
};

std::optional<ShotExecutionMetrics> shotMetrics(const ShotPlan& plan) noexcept
{
    if (const auto* value = std::get_if<DirectPotShotPlanPayload>(&plan.payload)) {
        return ShotExecutionMetrics{
            value->scoring.rawCosts.totalDistanceMm,
            value->candidate.cuttingAngleDeg,
            std::nullopt};
    }
    if (const auto* value = std::get_if<KickPotShotPlanPayload>(&plan.payload)) {
        return ShotExecutionMetrics{
            value->scoring.rawCosts.totalDistanceMm,
            value->candidate.cuttingAngleDeg,
            value->candidate.incidenceAngleDeg};
    }
    if (const auto* value =
            std::get_if<DirectLegalContactShotPlanPayload>(&plan.payload)) {
        return ShotExecutionMetrics{
            value->audit.totalPathLengthMm,
            std::nullopt,
            std::nullopt};
    }
    if (const auto* value =
            std::get_if<KickLegalContactShotPlanPayload>(&plan.payload)) {
        return ShotExecutionMetrics{
            value->audit.totalPathLengthMm,
            std::nullopt,
            value->candidate.incidenceAngleDeg};
    }
    return std::nullopt;
}

const BilliardConfig::FixedForceEnvelopeLimits* envelopeFor(
    const BilliardConfig::FixedForceEnvelopeConfig& config,
    ShotPlanType type) noexcept
{
    switch (type) {
    case ShotPlanType::DirectPot: return &config.directPot;
    case ShotPlanType::KickPot: return &config.kickPot;
    case ShotPlanType::DirectLegalContact: return &config.directLegalContact;
    case ShotPlanType::KickLegalContact: return &config.kickLegalContact;
    }
    return nullptr;
}

bool validEnvelopeLimits(
    const BilliardConfig::FixedForceEnvelopeLimits& limits) noexcept
{
    const auto validAngle = [](const std::optional<double>& value) {
        return !value || (std::isfinite(*value) && *value >= 0.0 && *value <= 180.0);
    };
    return std::isfinite(limits.minTotalPathLengthMm) &&
        std::isfinite(limits.maxTotalPathLengthMm) &&
        limits.minTotalPathLengthMm >= 0.0 &&
        limits.maxTotalPathLengthMm >= limits.minTotalPathLengthMm &&
        validAngle(limits.maxCuttingAngleDeg) &&
        validAngle(limits.maxExecutableKickRailAngleDeg);
}

bool validFixedForceConfig(
    const BilliardConfig::FixedForceEnvelopeConfig& config) noexcept
{
    return !config.calibrationRevision.empty() &&
        validEnvelopeLimits(config.directPot) &&
        validEnvelopeLimits(config.kickPot) &&
        validEnvelopeLimits(config.directLegalContact) &&
        validEnvelopeLimits(config.kickLegalContact) &&
        config.directPot.maxCuttingAngleDeg &&
        !config.directPot.maxExecutableKickRailAngleDeg &&
        config.kickPot.maxCuttingAngleDeg &&
        config.kickPot.maxExecutableKickRailAngleDeg &&
        !config.directLegalContact.maxCuttingAngleDeg &&
        !config.directLegalContact.maxExecutableKickRailAngleDeg &&
        !config.kickLegalContact.maxCuttingAngleDeg &&
        config.kickLegalContact.maxExecutableKickRailAngleDeg;
}

bool validTimingProfile(
    const BilliardConfig::PneumaticTimingProfileReference& profile) noexcept
{
    return !profile.calibrationRevision.empty() && profile.pneumaticPulseMs > 0 &&
        profile.directionChangeDelayMs > 0 &&
        profile.mechanismCompletionWaitMs > 0;
}

bool completeConfig(const BilliardConfig::MotionPlanningConfig& config) noexcept
{
    return config.calibrationRevision &&
        config.base0PlanarCalibrationRevision &&
        config.cueForwardAxisCalibrationRevision && config.strikeZMm &&
        config.safeApproachZMm && config.readyGapMm &&
        config.strikePositionBiasMm && config.pullModeMinBottomDistanceMm &&
        config.tableDownDirectionBase0XY && config.safeLiftHeightMm &&
        config.a0Deg && config.b0Deg && config.deltaADeg && config.deltaBDeg &&
        config.stepADeg && config.stepBDeg && config.searchOrder &&
        config.axisOffsetOrder && config.tieBreak && config.cToolOffsetDeg &&
        config.cueForwardAxisTool && config.maxCueDirectionErrorDeg &&
        config.directionUnitTolerance && config.executionPolicyRevision &&
        config.policyMode &&
        config.legalContactExecutionAuthorized && config.fixedForceEnvelope &&
        config.pneumaticTimingProfile &&
        config.tool1ControllerCalibrationRevision;
}

bool validConfig(const BilliardConfig::MotionPlanningConfig& config) noexcept
{
    const auto finite = [](double value) { return std::isfinite(value); };
    if (!completeConfig(config) || config.calibrationRevision->empty() ||
        config.base0PlanarCalibrationRevision->empty() ||
        config.cueForwardAxisCalibrationRevision->empty() ||
        config.tool1ControllerCalibrationRevision->empty() ||
        config.executionPolicyRevision->empty() ||
        !finite(*config.strikeZMm) || !finite(*config.safeApproachZMm) ||
        !finite(*config.readyGapMm) || *config.readyGapMm < 0.0 ||
        !finite(*config.strikePositionBiasMm) ||
        *config.strikePositionBiasMm < 0.0 ||
        !finite(*config.pullModeMinBottomDistanceMm) ||
        *config.pullModeMinBottomDistanceMm < 0.0 ||
        !BilliardMath::isFinite(*config.tableDownDirectionBase0XY) ||
        !finite(*config.safeLiftHeightMm) || *config.safeLiftHeightMm <= 0.0 ||
        !finite(*config.a0Deg) || !finite(*config.b0Deg) ||
        !finite(*config.deltaADeg) || *config.deltaADeg < 0.0 ||
        !finite(*config.deltaBDeg) || *config.deltaBDeg < 0.0 ||
        !finite(*config.stepADeg) || *config.stepADeg <= 0.0 ||
        !finite(*config.stepBDeg) || *config.stepBDeg <= 0.0 ||
        !finite(*config.cToolOffsetDeg) ||
        !finite(*config.maxCueDirectionErrorDeg) ||
        *config.maxCueDirectionErrorDeg < 0.0 ||
        *config.maxCueDirectionErrorDeg > 180.0 ||
        !finite(*config.directionUnitTolerance) ||
        *config.directionUnitTolerance <= 0.0 ||
        *config.directionUnitTolerance >= 1.0 ||
        (*config.searchOrder != BilliardConfig::PoseSearchOrder::AThenB &&
         *config.searchOrder != BilliardConfig::PoseSearchOrder::BThenA) ||
        (*config.axisOffsetOrder !=
            BilliardConfig::AxisOffsetOrder::LowerThenHigher &&
         *config.axisOffsetOrder !=
            BilliardConfig::AxisOffsetOrder::HigherThenLower) ||
        *config.tieBreak !=
            BilliardConfig::PoseTieBreak::FirstInApprovedSearchOrder ||
        (*config.policyMode != BilliardConfig::ExecutionPolicyMode::PlanningTest &&
         *config.policyMode !=
            BilliardConfig::ExecutionPolicyMode::RealHardware)) {
        return false;
    }
    if (!validFixedForceConfig(*config.fixedForceEnvelope) ||
        !validTimingProfile(*config.pneumaticTimingProfile)) {
        return false;
    }
    const double axisLength = std::sqrt(
        (*config.cueForwardAxisTool)[0] * (*config.cueForwardAxisTool)[0] +
        (*config.cueForwardAxisTool)[1] * (*config.cueForwardAxisTool)[1] +
        (*config.cueForwardAxisTool)[2] * (*config.cueForwardAxisTool)[2]);
    const double tableDownLength = std::hypot(
        config.tableDownDirectionBase0XY->x,
        config.tableDownDirectionBase0XY->y);
    if (!finite((*config.cueForwardAxisTool)[0]) ||
        !finite((*config.cueForwardAxisTool)[1]) ||
        !finite((*config.cueForwardAxisTool)[2]) || !finite(axisLength) ||
        std::fabs(axisLength - 1.0) > *config.directionUnitTolerance ||
        std::fabs((*config.cueForwardAxisTool)[0] - 1.0) >
            *config.directionUnitTolerance ||
        std::fabs((*config.cueForwardAxisTool)[1]) >
            *config.directionUnitTolerance ||
        std::fabs((*config.cueForwardAxisTool)[2]) >
            *config.directionUnitTolerance ||
        !finite(tableDownLength) ||
        std::fabs(tableDownLength - 1.0) > *config.directionUnitTolerance ||
        std::fabs(config.tableDownDirectionBase0XY->x) >
            *config.directionUnitTolerance ||
        std::fabs(config.tableDownDirectionBase0XY->y + 1.0) >
            *config.directionUnitTolerance) {
        return false;
    }
    const double aSteps = *config.deltaADeg / *config.stepADeg;
    const double bSteps = *config.deltaBDeg / *config.stepBDeg;
    return finite(aSteps) && finite(bSteps) &&
        aSteps <= static_cast<double>(MAX_AXIS_CANDIDATES) &&
        bSteps <= static_cast<double>(MAX_AXIS_CANDIDATES) &&
        std::fabs(aSteps - std::round(aSteps)) <= RANGE_TOLERANCE &&
        std::fabs(bSteps - std::round(bSteps)) <= RANGE_TOLERANCE;
}

std::vector<double> axisCandidates(
    double center,
    double delta,
    double step,
    BilliardConfig::AxisOffsetOrder order)
{
    std::vector<double> values{center};
    const std::size_t steps = static_cast<std::size_t>(std::llround(delta / step));
    values.reserve(1 + 2 * steps);
    for (std::size_t index = 1; index <= steps; ++index) {
        const double offset = static_cast<double>(index) * step;
        const double lower = center - offset;
        const double higher = center + offset;
        if (order == BilliardConfig::AxisOffsetOrder::LowerThenHigher) {
            values.push_back(lower);
            values.push_back(higher);
        } else {
            values.push_back(higher);
            values.push_back(lower);
        }
    }
    return values;
}

ExecutionPlanResult reject(
    ExecutionPlanStatus status,
    ExecutionPlanFailureReason reason,
    std::size_t evaluated = 0)
{
    return ExecutionPlanResult::rejected(status, reason, evaluated);
}
}

bool RobotPoseABC::isFinite() const noexcept
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
        std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

bool SafeLiftDerivationRule::isValid() const noexcept
{
    return derivation == SafeLiftDerivation::RuntimeActualPoseKeepXYABCIncreaseZ &&
        std::isfinite(heightMm) && heightMm > 0.0;
}

bool FixedForceEnvelopeEvaluation::isValid() const noexcept
{
    const auto withinOptional = [](const std::optional<double>& value,
                                   const std::optional<double>& maximum) {
        return value.has_value() == maximum.has_value() &&
            (!value || (std::isfinite(*value) && std::isfinite(*maximum) &&
                        *value >= 0.0 && *value <= *maximum));
    };
    return !calibrationRevision.empty() &&
        std::isfinite(totalPathLengthMm) &&
        std::isfinite(minTotalPathLengthMm) &&
        std::isfinite(maxTotalPathLengthMm) &&
        minTotalPathLengthMm >= 0.0 &&
        totalPathLengthMm >= minTotalPathLengthMm &&
        totalPathLengthMm <= maxTotalPathLengthMm &&
        withinOptional(cuttingAngleDeg, maxCuttingAngleDeg) &&
        withinOptional(kickRailAngleDeg, maxExecutableKickRailAngleDeg);
}

bool PlannedStageContract::isValid() const noexcept
{
    const bool validIntent =
        intent == PlannedMotionIntent::JointPtpToTransit ||
        intent == PlannedMotionIntent::CartesianPtpToSafeApproach ||
        intent == PlannedMotionIntent::LinearToStrikeReady ||
        intent == PlannedMotionIntent::RuntimeActualPoseVerticalSafeLift ||
        intent == PlannedMotionIntent::JointPtpToCamera;
    const bool validPrecondition =
        precondition == PlannedStagePrecondition::PreviousStageSucceeded ||
        precondition == PlannedStagePrecondition::PolicyAndCalibrationAccepted ||
        precondition ==
            PlannedStagePrecondition::RuntimeActualPoseAndPneumaticCompletion;
    const bool validSuccess =
        successCondition == PlannedStageSuccessCondition::TargetReachedAndStopped ||
        successCondition ==
            PlannedStageSuccessCondition::LinearTargetReachedAndStopped ||
        successCondition ==
            PlannedStageSuccessCondition::SafeLiftReachedAndStopped;
    const bool validPathCheck =
        pathCheck == PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability ||
        pathCheck == PlannedPathCheck::MotionCheckLinRequired;
    return validIntent && validPrecondition && validSuccess &&
        failureTransition == PlannedStageFailureTransition::StopFailClosed &&
        validPathCheck;
}

bool PoseSearchAudit::contains(
    double selectedA,
    double selectedB,
    std::size_t selectedOrdinal) const noexcept
{
    const auto axisCount = [](double center, double delta, double step)
        -> std::optional<std::size_t> {
        const double steps = delta / step;
        if (!std::isfinite(center) || !std::isfinite(delta) || delta < 0.0 ||
            !std::isfinite(step) || step <= 0.0 || !std::isfinite(steps) ||
            steps > static_cast<double>(MAX_AXIS_CANDIDATES) ||
            std::fabs(steps - std::round(steps)) > RANGE_TOLERANCE) {
            return std::nullopt;
        }
        return 1U + 2U * static_cast<std::size_t>(std::llround(steps));
    };
    const auto valueAt = [](double center, double step, std::size_t index,
                            BilliardConfig::AxisOffsetOrder order) {
        if (index == 0) return center;
        const double offset = static_cast<double>((index + 1U) / 2U) * step;
        const bool firstInPair = (index % 2U) == 1U;
        const bool lower = order == BilliardConfig::AxisOffsetOrder::LowerThenHigher
            ? firstInPair
            : !firstInPair;
        return center + (lower ? -offset : offset);
    };
    const auto aCount = axisCount(a0Deg, deltaADeg, stepADeg);
    const auto bCount = axisCount(b0Deg, deltaBDeg, stepBDeg);
    if (!aCount || !bCount || !std::isfinite(selectedA) ||
        !std::isfinite(selectedB) ||
        (searchOrder != BilliardConfig::PoseSearchOrder::AThenB &&
         searchOrder != BilliardConfig::PoseSearchOrder::BThenA) ||
        (axisOffsetOrder != BilliardConfig::AxisOffsetOrder::LowerThenHigher &&
         axisOffsetOrder != BilliardConfig::AxisOffsetOrder::HigherThenLower) ||
        tieBreak != BilliardConfig::PoseTieBreak::FirstInApprovedSearchOrder ||
        evaluatedCandidateCount != selectedOrdinal + 1U ||
        selectedOrdinal >= (*aCount * *bCount)) {
        return false;
    }
    const std::size_t aIndex =
        searchOrder == BilliardConfig::PoseSearchOrder::AThenB
        ? selectedOrdinal / *bCount
        : selectedOrdinal % *aCount;
    const std::size_t bIndex =
        searchOrder == BilliardConfig::PoseSearchOrder::AThenB
        ? selectedOrdinal % *bCount
        : selectedOrdinal / *aCount;
    const double expectedA = valueAt(a0Deg, stepADeg, aIndex, axisOffsetOrder);
    const double expectedB = valueAt(b0Deg, stepBDeg, bIndex, axisOffsetOrder);
    return std::fabs(selectedA - expectedA) <= RANGE_TOLERANCE &&
        std::fabs(selectedB - expectedB) <= RANGE_TOLERANCE;
}

bool ExecutionPlan::isValid() const noexcept
{
    const bool legal = isLegalContact(sourceShotType);
    const std::array<PlannedMotionIntent, 5> requiredIntents{
        PlannedMotionIntent::JointPtpToTransit,
        PlannedMotionIntent::CartesianPtpToSafeApproach,
        PlannedMotionIntent::LinearToStrikeReady,
        PlannedMotionIntent::RuntimeActualPoseVerticalSafeLift,
        PlannedMotionIntent::JointPtpToCamera};
    const std::array<PlannedStageContract, 5> requiredStages{{
        {requiredIntents[0], PlannedStagePrecondition::PolicyAndCalibrationAccepted,
         PlannedStageSuccessCondition::TargetReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability},
        {requiredIntents[1], PlannedStagePrecondition::PreviousStageSucceeded,
         PlannedStageSuccessCondition::TargetReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability},
        {requiredIntents[2], PlannedStagePrecondition::PreviousStageSucceeded,
         PlannedStageSuccessCondition::LinearTargetReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::MotionCheckLinRequired},
        {requiredIntents[3],
         PlannedStagePrecondition::RuntimeActualPoseAndPneumaticCompletion,
         PlannedStageSuccessCondition::SafeLiftReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::MotionCheckLinRequired},
        {requiredIntents[4], PlannedStagePrecondition::PreviousStageSucceeded,
         PlannedStageSuccessCondition::TargetReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability}}};
    const bool validStages = std::equal(
        stageContracts.begin(), stageContracts.end(), requiredStages.begin(),
        [](const PlannedStageContract& actual,
           const PlannedStageContract& required) {
            return actual.isValid() && actual.intent == required.intent &&
                actual.precondition == required.precondition &&
                actual.successCondition == required.successCondition &&
                actual.failureTransition == required.failureTransition &&
                actual.pathCheck == required.pathCheck;
        });
    const bool validPolicyMode =
        policyMode == BilliardConfig::ExecutionPolicyMode::PlanningTest ||
        policyMode == BilliardConfig::ExecutionPolicyMode::RealHardware;
    const bool validTiming = validTimingProfile(pneumaticTimingProfile);
    const bool validEnvelopeShape =
        (sourceShotType == ShotPlanType::DirectPot &&
         fixedForceEnvelope.cuttingAngleDeg &&
         !fixedForceEnvelope.kickRailAngleDeg) ||
        (sourceShotType == ShotPlanType::KickPot &&
         fixedForceEnvelope.cuttingAngleDeg &&
         fixedForceEnvelope.kickRailAngleDeg) ||
        (sourceShotType == ShotPlanType::DirectLegalContact &&
         !fixedForceEnvelope.cuttingAngleDeg &&
         !fixedForceEnvelope.kickRailAngleDeg) ||
        (sourceShotType == ShotPlanType::KickLegalContact &&
         !fixedForceEnvelope.cuttingAngleDeg &&
         fixedForceEnvelope.kickRailAngleDeg);
    const double directionLength = std::hypot(
        shotDirectionXY.x, shotDirectionXY.y);
    const double tableDownLength = std::hypot(
        tableDownDirectionBase0XY.x,
        tableDownDirectionBase0XY.y);
    const double expectedBottomDistance =
        cueBallCenterBase0Mm.y - physicalPlayingSurfaceBase0Mm.minY;
    const double expectedDirectionDot =
        dot(shotDirectionXY, tableDownDirectionBase0XY);
    const StrikeMode expectedMode = resolveStrikeMode(
        expectedBottomDistance,
        pullModeMinBottomDistanceMm,
        expectedDirectionDot);
    const double centerToTcpMm = ballRadiusMm + readyGapMm;
    const double nominalX =
        cueBallCenterBase0Mm.x - centerToTcpMm * shotDirectionXY.x;
    const double nominalY =
        cueBallCenterBase0Mm.y - centerToTcpMm * shotDirectionXY.y;
    const double biasSign = strikeMode == StrikeMode::Push ? 1.0 : -1.0;
    const double expectedX =
        nominalX + biasSign * strikePositionBiasMm * shotDirectionXY.x;
    const double expectedY =
        nominalY + biasSign * strikePositionBiasMm * shotDirectionXY.y;
    const std::optional<double> pushC = directionToCDeg(
        shotDirectionXY, cToolOffsetDeg, directionUnitTolerance);
    const std::optional<double> expectedC = pushC
        ? std::optional<double>{strikeMode == StrikeMode::Pull
              ? normalizeAngleDeg(*pushC + 180.0)
              : *pushC}
        : std::nullopt;
    const std::optional<double> measuredError =
        BilliardMath::getAngleBetweenVectorsDeg(
            validatedStrikeDirectionXY, shotDirectionXY);
    return (isPot(sourceShotType) || legal) && sourcePlanIdentity.isValid() &&
        !base0PlanarCalibrationRevision.empty() &&
        !tableGeometryRevision.empty() && !motionCalibrationRevision.empty() &&
        !cueForwardAxisCalibrationRevision.empty() &&
        std::isfinite(cueBallCenterBase0Mm.x) &&
        std::isfinite(cueBallCenterBase0Mm.y) &&
        BilliardMath::isFinite(shotDirectionXY) &&
        (strikeMode == StrikeMode::Push || strikeMode == StrikeMode::Pull) &&
        strikeMode == expectedMode && validBounds(physicalPlayingSurfaceBase0Mm) &&
        cueBallCenterBase0Mm.x >= physicalPlayingSurfaceBase0Mm.minX &&
        cueBallCenterBase0Mm.x <= physicalPlayingSurfaceBase0Mm.maxX &&
        cueBallCenterBase0Mm.y >= physicalPlayingSurfaceBase0Mm.minY &&
        cueBallCenterBase0Mm.y <= physicalPlayingSurfaceBase0Mm.maxY &&
        BilliardMath::isFinite(tableDownDirectionBase0XY) &&
        std::isfinite(tableDownLength) &&
        std::fabs(tableDownLength - 1.0) <= directionUnitTolerance &&
        std::fabs(tableDownDirectionBase0XY.x) <= directionUnitTolerance &&
        std::fabs(tableDownDirectionBase0XY.y + 1.0) <= directionUnitTolerance &&
        std::isfinite(pullModeMinBottomDistanceMm) &&
        pullModeMinBottomDistanceMm >= 0.0 &&
        std::isfinite(bottomDistanceMm) && bottomDistanceMm >= 0.0 &&
        std::fabs(bottomDistanceMm - expectedBottomDistance) <= RANGE_TOLERANCE &&
        std::isfinite(tableDownDirectionDot) &&
        std::fabs(tableDownDirectionDot - expectedDirectionDot) <= RANGE_TOLERANCE &&
        std::isfinite(strikePositionBiasMm) && strikePositionBiasMm >= 0.0 &&
        std::isfinite(ballRadiusMm) && ballRadiusMm > 0.0 &&
        std::isfinite(readyGapMm) && readyGapMm >= 0.0 &&
        std::isfinite(directionUnitTolerance) && directionUnitTolerance > 0.0 &&
        std::isfinite(directionLength) &&
        std::fabs(directionLength - 1.0) <= directionUnitTolerance &&
        std::isfinite(cToolOffsetDeg) &&
        BilliardMath::isFinite(validatedStrikeDirectionXY) &&
        std::fabs(std::hypot(
            validatedStrikeDirectionXY.x, validatedStrikeDirectionXY.y) - 1.0) <=
            directionUnitTolerance &&
        std::isfinite(cueDirectionErrorDeg) && cueDirectionErrorDeg >= 0.0 &&
        std::isfinite(maxCueDirectionErrorDeg) &&
        maxCueDirectionErrorDeg >= 0.0 &&
        cueDirectionErrorDeg <= maxCueDirectionErrorDeg && measuredError &&
        std::fabs(*measuredError - cueDirectionErrorDeg) <= RANGE_TOLERANCE &&
        safeApproachPose.isFinite() && strikeReadyPose.isFinite() &&
        std::isfinite(expectedX) && std::isfinite(expectedY) && expectedC &&
        std::fabs(strikeReadyPose.x - expectedX) <= RANGE_TOLERANCE &&
        std::fabs(strikeReadyPose.y - expectedY) <= RANGE_TOLERANCE &&
        std::fabs(strikeReadyPose.c - *expectedC) <= RANGE_TOLERANCE &&
        selectedADeg == strikeReadyPose.a &&
        selectedBDeg == strikeReadyPose.b &&
        selectedCDeg == strikeReadyPose.c &&
        poseSearchAudit.contains(
            selectedADeg, selectedBDeg, selectedSearchOrdinal) &&
        safeApproachPose.x == strikeReadyPose.x &&
        safeApproachPose.y == strikeReadyPose.y &&
        safeApproachPose.a == strikeReadyPose.a &&
        safeApproachPose.b == strikeReadyPose.b &&
        safeApproachPose.c == strikeReadyPose.c &&
        safeLiftRule.isValid() && finiteArray(transitJointReference) &&
        finiteArray(cameraJointReference) && motionIntents == requiredIntents &&
        validStages && fixedForceEnvelope.isValid() && validEnvelopeShape &&
        validTiming && !executionPolicyRevision.empty() &&
        validPolicyMode &&
        tool1Number == BilliardConfig::TOOL_NUMBER &&
        !tool1ControllerCalibrationRevision.empty() &&
        ((!legal && policyDecision == ExecutionPolicyDecision::PotAccepted) ||
         (legal && policyDecision ==
            ExecutionPolicyDecision::LegalContactExplicitlyAuthorized &&
          policyMode != BilliardConfig::ExecutionPolicyMode::RealHardware) ||
         (legal && policyDecision ==
            ExecutionPolicyDecision::LegalContactPlanningTestSelected &&
          policyMode == BilliardConfig::ExecutionPolicyMode::PlanningTest) ||
         (legal && policyDecision ==
            ExecutionPolicyDecision::LegalContactProductionFallbackAccepted &&
          policyMode == BilliardConfig::ExecutionPolicyMode::RealHardware &&
          rankedPotCandidatesExhausted));
}

ExecutionPlanResult ExecutionPlanResult::success(ExecutionPlan value)
{
    if (!value.isValid()) {
        return rejected(
            ExecutionPlanStatus::InvalidExecutionPlan,
            ExecutionPlanFailureReason::InvalidExecutionPlanValue);
    }
    return ExecutionPlanResult{
        ExecutionPlanStatus::Success,
        std::optional<ExecutionPlan>{std::move(value)},
        std::nullopt};
}

ExecutionPlanResult ExecutionPlanResult::rejected(
    ExecutionPlanStatus status,
    ExecutionPlanFailureReason reason,
    std::size_t evaluatedPoseCandidates)
{
    return ExecutionPlanResult{
        status,
        std::nullopt,
        ExecutionPlanDiagnostic{reason, evaluatedPoseCandidates}};
}

ExecutionPlanResult::ExecutionPlanResult(
    ExecutionPlanStatus status,
    std::optional<ExecutionPlan> value,
    std::optional<ExecutionPlanDiagnostic> diagnostic)
    : status_(status), value_(std::move(value)), diagnostic_(std::move(diagnostic))
{
}

ExecutionPlanStatus ExecutionPlanResult::status() const noexcept { return status_; }

const std::optional<ExecutionPlan>& ExecutionPlanResult::value() const noexcept
{
    return value_;
}

const std::optional<ExecutionPlanDiagnostic>&
ExecutionPlanResult::diagnostic() const noexcept
{
    return diagnostic_;
}

bool ExecutionPlanResult::isValid() const noexcept
{
    if (status_ == ExecutionPlanStatus::Success) {
        return value_ && value_->isValid() && !diagnostic_;
    }
    return !value_ && diagnostic_ &&
        diagnostic_->reason != ExecutionPlanFailureReason::None;
}

ExecutionPlanResult MotionPlanner::createExecutionPlan(
    const PlanningResult& planningResult,
    const std::optional<BilliardConfig::TableGeometryConfig>& tableGeometry,
    const std::optional<BilliardConfig::MotionPlanningConfig>& config,
    const MotionPlanningChecks& checks,
    bool rankedPotCandidatesExhausted) const
{
    if (!planningResult.isValid()) {
        return reject(
            ExecutionPlanStatus::InvalidShotPlan,
            ExecutionPlanFailureReason::InvalidShotPlanContract);
    }
    const ShotPlan* shotPlan = std::get_if<ShotPlan>(&planningResult.value());
    if (!shotPlan) {
        return reject(
            ExecutionPlanStatus::InvalidShotPlan,
            ExecutionPlanFailureReason::InputIsNotShotPlan);
    }
    if (!tableGeometry) {
        return reject(
            ExecutionPlanStatus::ConfigurationMissing,
            ExecutionPlanFailureReason::MissingTableGeometry);
    }
    if (!config) {
        return reject(
            ExecutionPlanStatus::ConfigurationMissing,
            ExecutionPlanFailureReason::MissingMotionCalibration);
    }
    if (!checks.poseAccepted || !checks.projectCueForwardAxisToBase0XY) {
        return reject(
            ExecutionPlanStatus::ConfigurationMissing,
            ExecutionPlanFailureReason::MissingValidationSeam);
    }
    if (!completeConfig(*config)) {
        return reject(
            ExecutionPlanStatus::ConfigurationMissing,
            ExecutionPlanFailureReason::MissingMotionCalibration);
    }
    if (!validConfig(*config)) {
        return reject(
            ExecutionPlanStatus::InvalidConfiguration,
            ExecutionPlanFailureReason::InvalidMotionCalibration);
    }
    if (shotPlan->source.base0PlanarCalibrationRevision !=
        *config->base0PlanarCalibrationRevision) {
        return reject(
            ExecutionPlanStatus::InvalidConfiguration,
            ExecutionPlanFailureReason::CalibrationRevisionMismatch);
    }
    if (tableGeometry->calibrationRevision.empty() ||
        !validBounds(tableGeometry->physicalPlayingSurface) ||
        !std::isfinite(tableGeometry->ballRadiusMm) ||
        tableGeometry->ballRadiusMm <= 0.0 ||
        std::fabs(tableGeometry->ballRadiusMm - shotPlan->source.ballRadiusMm) >
            RANGE_TOLERANCE) {
        return reject(
            ExecutionPlanStatus::InvalidConfiguration,
            ExecutionPlanFailureReason::InvalidTableGeometry);
    }
    if (shotPlan->source.tableGeometryRevision !=
        tableGeometry->calibrationRevision) {
        return reject(
            ExecutionPlanStatus::InvalidConfiguration,
            ExecutionPlanFailureReason::TableGeometryRevisionMismatch);
    }
    const bool legal = isLegalContact(shotPlan->type);
    const bool planningTestLegal = legal && rankedPotCandidatesExhausted &&
        *config->policyMode == BilliardConfig::ExecutionPolicyMode::PlanningTest;
    const bool productionLegalFallback =
        legal && isProductionLegalContactFallback(*shotPlan) &&
        *config->policyMode == BilliardConfig::ExecutionPolicyMode::RealHardware;
    if (legal &&
        ((!planningTestLegal && !productionLegalFallback &&
          !*config->legalContactExecutionAuthorized) ||
         (productionLegalFallback &&
           (!config->productionLegalContactFallbackAuthorized ||
            !*config->productionLegalContactFallbackAuthorized ||
            !rankedPotCandidatesExhausted)))) {
        return reject(
            ExecutionPlanStatus::NoExecutablePlan,
            ExecutionPlanFailureReason::LegalContactNotAuthorized);
    }

    const std::optional<ShotExecutionMetrics> metrics = shotMetrics(*shotPlan);
    const BilliardConfig::FixedForceEnvelopeLimits* envelope =
        envelopeFor(*config->fixedForceEnvelope, shotPlan->type);
    if (!envelope) {
        return reject(
            ExecutionPlanStatus::InvalidShotPlan,
            ExecutionPlanFailureReason::InvalidShotPlanContract);
    }
    std::optional<double> phase1KickMaximum;
    if (const auto* kick = std::get_if<KickPotShotPlanPayload>(&shotPlan->payload)) {
        phase1KickMaximum = kick->kickGeometry.maxKickRailAngleDeg;
    } else if (const auto* legalKick =
                   std::get_if<KickLegalContactShotPlanPayload>(&shotPlan->payload)) {
        phase1KickMaximum = legalKick->kickGeometry.maxKickRailAngleDeg;
    }
    const FixedForceEnvelopeEvaluation envelopeEvaluation{
        config->fixedForceEnvelope->calibrationRevision,
        metrics ? metrics->totalPathLengthMm
                : std::numeric_limits<double>::quiet_NaN(),
        metrics ? metrics->cuttingAngleDeg : std::nullopt,
        metrics ? metrics->kickRailAngleDeg : std::nullopt,
        envelope->minTotalPathLengthMm,
        envelope->maxTotalPathLengthMm,
        envelope->maxCuttingAngleDeg,
        envelope->maxExecutableKickRailAngleDeg};
    if (phase1KickMaximum &&
        (!envelope->maxExecutableKickRailAngleDeg ||
         *envelope->maxExecutableKickRailAngleDeg > *phase1KickMaximum)) {
        return reject(
            ExecutionPlanStatus::InvalidConfiguration,
            ExecutionPlanFailureReason::InvalidMotionCalibration);
    }
    if (!envelope->enabled || !metrics || !envelopeEvaluation.isValid()) {
        return reject(
            ExecutionPlanStatus::NoExecutablePlan,
            ExecutionPlanFailureReason::FixedForceEnvelopeRejected);
    }

    const double directionLength = std::hypot(
        shotPlan->shotDirectionXY.x,
        shotPlan->shotDirectionXY.y);
    if (!BilliardMath::isFinite(shotPlan->shotDirectionXY) ||
        !std::isfinite(directionLength) || directionLength <= 0.0 ||
        std::fabs(directionLength - 1.0) > *config->directionUnitTolerance) {
        return reject(
            ExecutionPlanStatus::InvalidShotPlan,
            ExecutionPlanFailureReason::InvalidShotDirection);
    }
    const AxisAlignedBounds2D& physicalSurface =
        tableGeometry->physicalPlayingSurface;
    const Point cueBall = shotPlan->source.cueBallSnapshot;
    if (!BilliardMath::isFinite(cueBall) || cueBall.x < physicalSurface.minX ||
        cueBall.x > physicalSurface.maxX || cueBall.y < physicalSurface.minY ||
        cueBall.y > physicalSurface.maxY) {
        return reject(
            ExecutionPlanStatus::InvalidShotPlan,
            ExecutionPlanFailureReason::InvalidShotPlanContract);
    }
    const double bottomDistanceMm = cueBall.y - physicalSurface.minY;
    const double tableDownDirectionDot = dot(
        shotPlan->shotDirectionXY,
        *config->tableDownDirectionBase0XY);
    if (!std::isfinite(bottomDistanceMm) || bottomDistanceMm < 0.0 ||
        !std::isfinite(tableDownDirectionDot)) {
        return reject(
            ExecutionPlanStatus::NoExecutablePlan,
            ExecutionPlanFailureReason::NumericalFailure);
    }
    const StrikeMode strikeMode = resolveStrikeMode(
        bottomDistanceMm,
        *config->pullModeMinBottomDistanceMm,
        tableDownDirectionDot);
    const std::optional<double> pushCDeg = directionToCDeg(
        shotPlan->shotDirectionXY,
        *config->cToolOffsetDeg,
        *config->directionUnitTolerance);
    const std::optional<double> cDeg = pushCDeg
        ? std::optional<double>{strikeMode == StrikeMode::Pull
              ? normalizeAngleDeg(*pushCDeg + 180.0)
              : *pushCDeg}
        : std::nullopt;
    const double centerToTcpMm =
        shotPlan->source.ballRadiusMm + *config->readyGapMm;
    const Point nominalStrikeXY{
        cueBall.x - centerToTcpMm * shotPlan->shotDirectionXY.x,
        cueBall.y - centerToTcpMm * shotPlan->shotDirectionXY.y};
    const double biasSign = strikeMode == StrikeMode::Push ? 1.0 : -1.0;
    const Point readyXY{
        nominalStrikeXY.x + biasSign * *config->strikePositionBiasMm *
            shotPlan->shotDirectionXY.x,
        nominalStrikeXY.y + biasSign * *config->strikePositionBiasMm *
            shotPlan->shotDirectionXY.y};
    if (!cDeg || !std::isfinite(centerToTcpMm) ||
        centerToTcpMm <= 0.0 || !BilliardMath::isFinite(nominalStrikeXY) ||
        !BilliardMath::isFinite(readyXY)) {
        return reject(
            ExecutionPlanStatus::NoExecutablePlan,
            ExecutionPlanFailureReason::NumericalFailure);
    }

    const std::vector<double> aValues = axisCandidates(
        *config->a0Deg,
        *config->deltaADeg,
        *config->stepADeg,
        *config->axisOffsetOrder);
    const std::vector<double> bValues = axisCandidates(
        *config->b0Deg,
        *config->deltaBDeg,
        *config->stepBDeg,
        *config->axisOffsetOrder);
    std::size_t evaluated = 0;
    std::optional<RobotPoseABC> selectedReady;
    std::optional<RobotPoseABC> selectedApproach;
    std::optional<Vector2D> selectedProjectedStrikeDirection;
    std::optional<double> selectedDirectionError;
    const std::array<double, 3> strikeAxisTool = strikeAxisForMode(
        *config->cueForwardAxisTool,
        strikeMode);
    const auto evaluate = [&](double aDeg, double bDeg) {
        ++evaluated;
        const RobotPoseABC ready{
            readyXY.x, readyXY.y, *config->strikeZMm, aDeg, bDeg, *cDeg};
        const RobotPoseABC approach{
            readyXY.x, readyXY.y, *config->safeApproachZMm, aDeg, bDeg, *cDeg};
        if (!ready.isFinite() || !approach.isFinite()) return false;
        const std::optional<bool> readyAccepted = checks.poseAccepted(ready);
        const std::optional<bool> approachAccepted = checks.poseAccepted(approach);
        if (!readyAccepted || !approachAccepted || !*readyAccepted ||
            !*approachAccepted) {
            return false;
        }
        const std::optional<Vector2D> projected =
            checks.projectCueForwardAxisToBase0XY(
                ready,
                strikeAxisTool);
        if (!projected || !BilliardMath::isFinite(*projected)) return false;
        const std::optional<Vector2D> normalized = BilliardMath::normalize(*projected);
        const std::optional<double> error = normalized
            ? BilliardMath::getAngleBetweenVectorsDeg(
                *normalized,
                shotPlan->shotDirectionXY)
            : std::nullopt;
        if (!error || !std::isfinite(*error) ||
            *error > *config->maxCueDirectionErrorDeg) {
            return false;
        }
        selectedReady = ready;
        selectedApproach = approach;
        selectedProjectedStrikeDirection = *normalized;
        selectedDirectionError = *error;
        return true;
    };

    bool found = false;
    if (*config->searchOrder == BilliardConfig::PoseSearchOrder::AThenB) {
        for (double aDeg : aValues) {
            for (double bDeg : bValues) {
                if (evaluate(aDeg, bDeg)) {
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
    } else {
        for (double bDeg : bValues) {
            for (double aDeg : aValues) {
                if (evaluate(aDeg, bDeg)) {
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
    }
    if (!found || !selectedReady || !selectedApproach ||
        !selectedProjectedStrikeDirection || !selectedDirectionError) {
        return reject(
            ExecutionPlanStatus::NoExecutablePlan,
            ExecutionPlanFailureReason::NoAcceptedPoseCandidate,
            evaluated);
    }

    const ExecutionPlan plan{
        shotPlan->source.planIdentity,
        shotPlan->type,
        shotPlan->source.base0PlanarCalibrationRevision,
        shotPlan->source.tableGeometryRevision,
        *config->calibrationRevision,
        *config->cueForwardAxisCalibrationRevision,
        shotPlan->source.cueBallSnapshot,
        shotPlan->shotDirectionXY,
        strikeMode,
        physicalSurface,
        *config->tableDownDirectionBase0XY,
        *config->pullModeMinBottomDistanceMm,
        bottomDistanceMm,
        tableDownDirectionDot,
        *config->strikePositionBiasMm,
        shotPlan->source.ballRadiusMm,
        *config->readyGapMm,
        *config->directionUnitTolerance,
        *config->cToolOffsetDeg,
        *selectedProjectedStrikeDirection,
        *selectedDirectionError,
        *config->maxCueDirectionErrorDeg,
        *selectedApproach,
        *selectedReady,
        selectedReady->a,
        selectedReady->b,
        selectedReady->c,
        evaluated - 1,
        {*config->a0Deg,
         *config->b0Deg,
         *config->deltaADeg,
         *config->deltaBDeg,
         *config->stepADeg,
         *config->stepBDeg,
         *config->searchOrder,
         *config->axisOffsetOrder,
         *config->tieBreak,
         evaluated},
        {SafeLiftDerivation::RuntimeActualPoseKeepXYABCIncreaseZ,
         *config->safeLiftHeightMm},
        BilliardConfig::TRANSIT_JOINT,
        BilliardConfig::CAMERA_JOINT,
        {PlannedMotionIntent::JointPtpToTransit,
         PlannedMotionIntent::CartesianPtpToSafeApproach,
         PlannedMotionIntent::LinearToStrikeReady,
         PlannedMotionIntent::RuntimeActualPoseVerticalSafeLift,
         PlannedMotionIntent::JointPtpToCamera},
        {{{PlannedMotionIntent::JointPtpToTransit,
           PlannedStagePrecondition::PolicyAndCalibrationAccepted,
           PlannedStageSuccessCondition::TargetReachedAndStopped,
           PlannedStageFailureTransition::StopFailClosed,
           PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability},
          {PlannedMotionIntent::CartesianPtpToSafeApproach,
           PlannedStagePrecondition::PreviousStageSucceeded,
           PlannedStageSuccessCondition::TargetReachedAndStopped,
           PlannedStageFailureTransition::StopFailClosed,
           PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability},
          {PlannedMotionIntent::LinearToStrikeReady,
           PlannedStagePrecondition::PreviousStageSucceeded,
           PlannedStageSuccessCondition::LinearTargetReachedAndStopped,
           PlannedStageFailureTransition::StopFailClosed,
           PlannedPathCheck::MotionCheckLinRequired},
          {PlannedMotionIntent::RuntimeActualPoseVerticalSafeLift,
           PlannedStagePrecondition::RuntimeActualPoseAndPneumaticCompletion,
           PlannedStageSuccessCondition::SafeLiftReachedAndStopped,
           PlannedStageFailureTransition::StopFailClosed,
           PlannedPathCheck::MotionCheckLinRequired},
          {PlannedMotionIntent::JointPtpToCamera,
           PlannedStagePrecondition::PreviousStageSucceeded,
           PlannedStageSuccessCondition::TargetReachedAndStopped,
           PlannedStageFailureTransition::StopFailClosed,
           PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability}}},
        envelopeEvaluation,
        *config->pneumaticTimingProfile,
        *config->executionPolicyRevision,
        *config->policyMode,
        legal
            ? (planningTestLegal
                  ? ExecutionPolicyDecision::LegalContactPlanningTestSelected
                  : productionLegalFallback
                  ? ExecutionPolicyDecision::
                        LegalContactProductionFallbackAccepted
                  : ExecutionPolicyDecision::LegalContactExplicitlyAuthorized)
            : ExecutionPolicyDecision::PotAccepted,
        BilliardConfig::TOOL_NUMBER,
        *config->tool1ControllerCalibrationRevision,
        rankedPotCandidatesExhausted};
    if (!plan.isValid()) {
        return reject(
            ExecutionPlanStatus::NoExecutablePlan,
            ExecutionPlanFailureReason::NumericalFailure,
            evaluated);
    }
    return ExecutionPlanResult::success(plan);
}

#ifdef BILLIARDS_P2_01_TEST_SEAM
std::optional<double> MotionPlanner::directionToCDegForTest(
    Vector2D direction,
    double cToolOffsetDeg,
    double unitTolerance) noexcept
{
    return directionToCDeg(direction, cToolOffsetDeg, unitTolerance);
}
#endif
