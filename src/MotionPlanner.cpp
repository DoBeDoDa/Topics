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
// MAX_AXIS_CANDIDATES bounds each axis independently; without a combined
// bound, two individually-approved per-axis ranges can still multiply into
// an unbounded pose search (up to ~4e8 combinations). This caps the actual
// total search size createExecutionPlan() will iterate.
constexpr std::size_t MAX_TOTAL_POSE_CANDIDATES = 1000;

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

double distancePointToSegment(Point p, Point a, Point b) noexcept
{
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double lengthSquared = abx * abx + aby * aby;
    if (!(lengthSquared > 0.0)) {
        return std::hypot(p.x - a.x, p.y - a.y);
    }
    double t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / lengthSquared;
    t = (std::max)(0.0, (std::min)(1.0, t));
    const double closestX = a.x + t * abx;
    const double closestY = a.y + t * aby;
    return std::hypot(p.x - closestX, p.y - closestY);
}

// 貼庫安全繞行：母球中心離最近庫邊在門檻內時，改用沿庫邊、背離最近端點
// （袋口角落）的單位向量取代Phase1算出的入袋方向，只求推桿安全靠近、
// 碰到球即可，不追求精準瞄準角度。回傳nullopt代表沒有觸發（沿用原方向）。
std::optional<Vector2D> resolveRailHuggingDirection(
    Point cueBall,
    const ResolvedTableGeometry& geometry,
    double triggerDistanceMm) noexcept
{
    std::optional<std::size_t> nearestIndex;
    double nearestDistance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < geometry.physicalRails.size(); ++index) {
        const Segment2D& segment = geometry.physicalRails[index].segment;
        const double distance =
            distancePointToSegment(cueBall, segment.start, segment.end);
        if (std::isfinite(distance) && distance < nearestDistance) {
            nearestDistance = distance;
            nearestIndex = index;
        }
    }
    if (!nearestIndex || !(nearestDistance <= triggerDistanceMm)) {
        return std::nullopt;
    }
    const Segment2D& segment = geometry.physicalRails[*nearestIndex].segment;
    const std::optional<Vector2D> tangent = BilliardMath::normalize(
        Vector2D{segment.end.x - segment.start.x, segment.end.y - segment.start.y});
    if (!tangent) {
        return std::nullopt;
    }
    const double distanceToStart =
        std::hypot(cueBall.x - segment.start.x, cueBall.y - segment.start.y);
    const double distanceToEnd =
        std::hypot(cueBall.x - segment.end.x, cueBall.y - segment.end.y);
    return distanceToStart <= distanceToEnd
        ? *tangent
        : Vector2D{-tangent->x, -tangent->y};
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

// ManualResearch標籤只代表Phase1 BrainConfig用了ManualResearch
// planningMode（見Algorithm.cpp的legalAuthority），跟這裡要判斷的
// 「RealHardware執行時pot是否已窮盡」是兩個獨立的軸——Phase1不得讀取
// Phase2的policyMode，所以無法自己標成ProductionFallbackEligible。
// LegalContactAuditFields::isValid()本來就允許ManualResearch標籤對應
// potSearchExhausted==true（TableState.h），這裡兩種標籤都接受，
// productionLegalContactFallbackAuthorized這個既有核准開關才真正生效；
// legalContactExecutionAuthorized維持獨立、不受影響。
bool isProductionLegalContactFallback(const ShotPlan& plan) noexcept
{
    if (const auto* direct =
            std::get_if<DirectLegalContactShotPlanPayload>(&plan.payload)) {
        return direct->audit.activationAuthority ==
                LegalContactAuditFields::ActivationAuthority::
                    ProductionFallbackEligible ||
            direct->audit.activationAuthority ==
                LegalContactAuditFields::ActivationAuthority::ManualResearch;
    }
    if (const auto* kick =
            std::get_if<KickLegalContactShotPlanPayload>(&plan.payload)) {
        return kick->audit.activationAuthority ==
                LegalContactAuditFields::ActivationAuthority::
                    ProductionFallbackEligible ||
            kick->audit.activationAuthority ==
                LegalContactAuditFields::ActivationAuthority::ManualResearch;
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
    if (std::get_if<CueBallContactOnlyShotPlanPayload>(&plan.payload)) {
        // 沒有目標球，唯一有意義的距離量是母球到cuePathSegments終點的
        // bookkeeping長度；不代表任何真實碰撞路徑。
        if (plan.cuePathSegments.empty()) return std::nullopt;
        const Segment2D& segment = plan.cuePathSegments.front();
        const double length = std::hypot(
            segment.end.x - segment.start.x, segment.end.y - segment.start.y);
        return ShotExecutionMetrics{length, std::nullopt, std::nullopt};
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
    case ShotPlanType::CueBallContactOnly: return &config.cueBallContactOnly;
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
        config.kickLegalContact.maxExecutableKickRailAngleDeg &&
        validEnvelopeLimits(config.cueBallContactOnly) &&
        !config.cueBallContactOnly.maxCuttingAngleDeg &&
        !config.cueBallContactOnly.maxExecutableKickRailAngleDeg;
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
        config.tableDownDirectionBase0XY &&
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
        *config.safeApproachZMm <= *config.strikeZMm ||
        !finite(*config.readyGapMm) || *config.readyGapMm < 0.0 ||
        !finite(*config.strikePositionBiasMm) ||
        *config.strikePositionBiasMm < 0.0 ||
        !finite(*config.pullModeMinBottomDistanceMm) ||
        *config.pullModeMinBottomDistanceMm < 0.0 ||
        !BilliardMath::isFinite(*config.tableDownDirectionBase0XY) ||
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
    if (!finite(aSteps) || !finite(bSteps) ||
        aSteps > static_cast<double>(MAX_AXIS_CANDIDATES) ||
        bSteps > static_cast<double>(MAX_AXIS_CANDIDATES) ||
        std::fabs(aSteps - std::round(aSteps)) > RANGE_TOLERANCE ||
        std::fabs(bSteps - std::round(bSteps)) > RANGE_TOLERANCE) {
        return false;
    }
    const unsigned long long aCandidateCount =
        1ULL + 2ULL * static_cast<unsigned long long>(std::llround(aSteps));
    const unsigned long long bCandidateCount =
        1ULL + 2ULL * static_cast<unsigned long long>(std::llround(bSteps));
    return aCandidateCount * bCandidateCount <=
        static_cast<unsigned long long>(MAX_TOTAL_POSE_CANDIDATES);
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

RobotPoseABC buildSafeLiftTarget(
    const RobotPoseABC& postStrikeActualPose,
    double safeApproachPoseZ) noexcept
{
    return RobotPoseABC{
        postStrikeActualPose.x,
        postStrikeActualPose.y,
        safeApproachPoseZ,
        postStrikeActualPose.a,
        postStrikeActualPose.b,
        postStrikeActualPose.c};
}

bool isValidSafeLiftTarget(
    const RobotPoseABC& postStrikeActualPose,
    double safeApproachPoseZ,
    const RobotPoseABC& safeLiftTarget) noexcept
{
    return postStrikeActualPose.isFinite() && safeLiftTarget.isFinite() &&
        std::isfinite(safeApproachPoseZ) &&
        safeLiftTarget.x == postStrikeActualPose.x &&
        safeLiftTarget.y == postStrikeActualPose.y &&
        safeLiftTarget.a == postStrikeActualPose.a &&
        safeLiftTarget.b == postStrikeActualPose.b &&
        safeLiftTarget.c == postStrikeActualPose.c &&
        safeLiftTarget.z == safeApproachPoseZ &&
        safeLiftTarget.z > postStrikeActualPose.z;
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
        intent == PlannedMotionIntent::CartesianPtpToSafeApproach ||
        intent == PlannedMotionIntent::LinearToStrikeReady ||
        intent == PlannedMotionIntent::RuntimeActualPoseVerticalSafeLift ||
        intent == PlannedMotionIntent::JointPtpToStandby;
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
    const std::array<PlannedMotionIntent, 4> requiredIntents{
        PlannedMotionIntent::CartesianPtpToSafeApproach,
        PlannedMotionIntent::LinearToStrikeReady,
        PlannedMotionIntent::RuntimeActualPoseVerticalSafeLift,
        PlannedMotionIntent::JointPtpToStandby};
    const std::array<PlannedStageContract, 4> requiredStages{{
        {requiredIntents[0], PlannedStagePrecondition::PolicyAndCalibrationAccepted,
         PlannedStageSuccessCondition::TargetReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability},
        {requiredIntents[1], PlannedStagePrecondition::PreviousStageSucceeded,
         PlannedStageSuccessCondition::LinearTargetReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::MotionCheckLinRequired},
        {requiredIntents[2],
         PlannedStagePrecondition::RuntimeActualPoseAndPneumaticCompletion,
         PlannedStageSuccessCondition::SafeLiftReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::MotionCheckLinRequired},
        {requiredIntents[3], PlannedStagePrecondition::PreviousStageSucceeded,
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
         fixedForceEnvelope.kickRailAngleDeg) ||
        (sourceShotType == ShotPlanType::CueBallContactOnly &&
         !fixedForceEnvelope.cuttingAngleDeg &&
         !fixedForceEnvelope.kickRailAngleDeg);
    const double plannedDirectionLength = std::hypot(
        plannedShotDirectionXY.x, plannedShotDirectionXY.y);
    const bool sameDirection =
        std::fabs(shotDirectionXY.x - plannedShotDirectionXY.x) <=
            directionUnitTolerance &&
        std::fabs(shotDirectionXY.y - plannedShotDirectionXY.y) <=
            directionUnitTolerance;
    // 貼庫覆寫方向後執行方向一定跟原始方向不同（railHugging的方向來自
    // 庫邊切線，不會剛好等於原本入袋方向）；Normal政策下兩者必須逐位元
    // 一致，不允許悄悄覆寫方向卻標成Normal。
    const bool validDirectionPolicy =
        BilliardMath::isFinite(plannedShotDirectionXY) &&
        std::isfinite(plannedDirectionLength) &&
        std::fabs(plannedDirectionLength - 1.0) <= directionUnitTolerance &&
        ((executionDirectionPolicy == ExecutionDirectionPolicy::Normal &&
          sameDirection) ||
         (executionDirectionPolicy == ExecutionDirectionPolicy::RailHugging &&
          !sameDirection));
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
    // Tool1的TCP已核准校正在氣壓推桿行程中點（縮回位置沿Tool1 +X方向6cm，
    // 12cm總行程的一半），所以XY直接對齊母球中心，母球就自然落在行程中點，
    // push/pull都一樣（差異只在C軸方向）；不再額外扣ballRadiusMm/readyGapMm。
    const double biasSign = strikeMode == StrikeMode::Push ? 1.0 : -1.0;
    const double expectedX =
        cueBallCenterBase0Mm.x + biasSign * strikePositionBiasMm * shotDirectionXY.x;
    const double expectedY =
        cueBallCenterBase0Mm.y + biasSign * strikePositionBiasMm * shotDirectionXY.y;
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
    return (isPot(sourceShotType) || legal ||
            sourceShotType == ShotPlanType::CueBallContactOnly) &&
        validDirectionPolicy && sourcePlanIdentity.isValid() &&
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
        safeApproachPose.z > strikeReadyPose.z &&
        finiteArray(standbyJointReference) &&
        !standbyJointCalibrationRevision.empty() &&
        standbyJointCalibrationRevision ==
            BilliardConfig::STANDBY_JOINT_REFERENCE.calibrationRevision &&
        standbyJointReference == BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg &&
        motionIntents == requiredIntents &&
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
    bool rankedPotCandidatesExhausted,
    const std::optional<ResolvedTableGeometry>& resolvedTableGeometry,
    std::optional<StrikeMode> forcedStrikeMode) const
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
    if (!BilliardConfig::STANDBY_JOINT_REFERENCE.isValid()) {
        return reject(
            ExecutionPlanStatus::ConfigurationMissing,
            ExecutionPlanFailureReason::MissingMotionCalibration);
    }
    if (!checks.poseAccepted || !checks.projectCueForwardAxisToBase0XY ||
        !checks.linearPathAccepted) {
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
    // 固定氣動力道：距離／切球角度／反彈角度對執行力道沒有影響（力道本身
    // 不可調），使用者已確認不需要以phase1算出的幾何角度上限反過來限制
    // envelope設定值，故不比對phase1KickMaximum，只保留envelope本身的
    // 靜態範圍檢查（見BilliardConfig.cpp的fixedForceEnvelope設定）。
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
    // 貼庫安全繞行：母球貼近庫邊時，用平行庫邊、背離最近端點的方向取代
    // Phase1算出的入袋方向，只求安全碰到球；沒貼庫或功能未設定時
    // effectiveDirection就是原本的shotDirectionXY，行為完全不變。
    // directionPolicy記錄是否觸發，寫進ExecutionPlan.executionDirectionPolicy
    // audit欄位，不會悄悄覆寫方向卻不留痕跡。
    Vector2D effectiveDirection = shotPlan->shotDirectionXY;
    ExecutionDirectionPolicy directionPolicy = ExecutionDirectionPolicy::Normal;
    if (resolvedTableGeometry && config->railHuggingTriggerDistanceMm) {
        if (const std::optional<Vector2D> railDirection = resolveRailHuggingDirection(
                cueBall, *resolvedTableGeometry,
                *config->railHuggingTriggerDistanceMm)) {
            effectiveDirection = *railDirection;
            directionPolicy = ExecutionDirectionPolicy::RailHugging;
        }
    }
    // 推桿後方障礙檢查：母球中心沿執行方向反方向、長度
    // ballRadiusMm+BACK_OBSTACLE_EXTRA_MM的有限線段內，若有其他球中心
    // 太近（<= ballRadiusMm+BACK_OBSTACLE_LATERAL_MARGIN_MM），代表推桿
    // 實際伸出時會先撞到那顆球，此執行方向（含rail-hugging覆寫後的方向）
    // 不可用。不是完整Tool掃掠體積模型，只擋掉這一個候選，換下一個。
    {
        const double lback = shotPlan->source.ballRadiusMm +
            BilliardConfig::BACK_OBSTACLE_EXTRA_MM;
        const Point rearEnd{
            cueBall.x - lback * effectiveDirection.x,
            cueBall.y - lback * effectiveDirection.y};
        const double rejectDistance = shotPlan->source.ballRadiusMm +
            BilliardConfig::BACK_OBSTACLE_LATERAL_MARGIN_MM;
        for (const std::optional<Point>& obstacle :
                shotPlan->source.otherBallsSnapshot) {
            if (!obstacle) continue;
            const double distance =
                distancePointToSegment(*obstacle, cueBall, rearEnd);
            if (!std::isfinite(distance) || distance <= rejectDistance) {
                return reject(
                    ExecutionPlanStatus::NoExecutablePlan,
                    ExecutionPlanFailureReason::RearObstacleBlocked);
            }
        }
    }
    const double bottomDistanceMm = cueBall.y - physicalSurface.minY;
    const double tableDownDirectionDot = dot(
        effectiveDirection,
        *config->tableDownDirectionBase0XY);
    if (!std::isfinite(bottomDistanceMm) || bottomDistanceMm < 0.0 ||
        !std::isfinite(tableDownDirectionDot)) {
        return reject(
            ExecutionPlanStatus::NoExecutablePlan,
            ExecutionPlanFailureReason::NumericalFailure);
    }
    const StrikeMode preferredStrikeMode = forcedStrikeMode
        ? *forcedStrikeMode
        : resolveStrikeMode(
            bottomDistanceMm,
            *config->pullModeMinBottomDistanceMm,
            tableDownDirectionDot);

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

    // Push/Pull的姿態搜尋、readyXY偏移、C角全部包成一個吃strikeMode參數
    // 的嘗試，讓下面可以先試偏好模式，只有「候選局部失敗」
    // （NoAcceptedPoseCandidate）才用相反模式重試同一個execution
    // direction；安全/硬體/設定/數值類失敗不會走到重試。
    const auto attemptForStrikeMode =
        [&](StrikeMode strikeMode) -> ExecutionPlanResult {
    const std::optional<double> pushCDeg = directionToCDeg(
        effectiveDirection,
        *config->cToolOffsetDeg,
        *config->directionUnitTolerance);
    const std::optional<double> cDeg = pushCDeg
        ? std::optional<double>{strikeMode == StrikeMode::Pull
              ? normalizeAngleDeg(*pushCDeg + 180.0)
              : *pushCDeg}
        : std::nullopt;
    // Tool1的TCP已核准校正在氣壓推桿行程中點（縮回位置沿Tool1 +X方向6cm，
    // 12cm總行程的一半），所以XY直接對齊母球中心，母球就自然落在行程中點，
    // push/pull都一樣（差異只在C軸方向）；不再額外扣ballRadiusMm/readyGapMm。
    // 貼庫繞行觸發且railHuggingReadyGapMm已設定時，用這個專用值取代
    // strikePositionBiasMm（不是疊加）；貼庫時力道需求跟一般擊球不同。
    const double strikeOffsetMm =
        directionPolicy == ExecutionDirectionPolicy::RailHugging &&
            config->railHuggingReadyGapMm
        ? *config->railHuggingReadyGapMm
        : *config->strikePositionBiasMm;
    const double biasSign = strikeMode == StrikeMode::Push ? 1.0 : -1.0;
    const Point readyXY{
        cueBall.x + biasSign * strikeOffsetMm * effectiveDirection.x,
        cueBall.y + biasSign * strikeOffsetMm * effectiveDirection.y};
    if (!cDeg || !BilliardMath::isFinite(readyXY)) {
        return reject(
            ExecutionPlanStatus::NoExecutablePlan,
            ExecutionPlanFailureReason::NumericalFailure);
    }

    std::size_t evaluated = 0;
    std::optional<RobotPoseABC> selectedReady;
    std::optional<RobotPoseABC> selectedApproach;
    std::optional<Vector2D> selectedProjectedStrikeDirection;
    std::optional<double> selectedDirectionError;
    const std::array<double, 3> strikeAxisTool = strikeAxisForMode(
        *config->cueForwardAxisTool,
        strikeMode);
    bool hardFailure = false;
    const auto evaluate = [&](double aDeg, double bDeg) {
        ++evaluated;
        const RobotPoseABC ready{
            readyXY.x, readyXY.y, *config->strikeZMm, aDeg, bDeg, *cDeg};
        const RobotPoseABC approach{
            readyXY.x, readyXY.y, *config->safeApproachZMm, aDeg, bDeg, *cDeg};
        if (!ready.isFinite() || !approach.isFinite()) return false;
        const std::optional<bool> readyAccepted = checks.poseAccepted(ready);
        if (!readyAccepted) {
            hardFailure = true;
            return false;
        }
        if (!*readyAccepted) return false;
        const std::optional<bool> approachAccepted = checks.poseAccepted(approach);
        if (!approachAccepted) {
            hardFailure = true;
            return false;
        }
        if (!*approachAccepted) return false;
        const std::optional<bool> linearAccepted =
            checks.linearPathAccepted(approach, ready);
        if (!linearAccepted) {
            hardFailure = true;
            return false;
        }
        if (!*linearAccepted) return false;
        const std::optional<Vector2D> projected =
            checks.projectCueForwardAxisToBase0XY(
                ready,
                strikeAxisTool);
        if (!projected || !BilliardMath::isFinite(*projected)) return false;
        const std::optional<Vector2D> normalized = BilliardMath::normalize(*projected);
        const std::optional<double> error = normalized
            ? BilliardMath::getAngleBetweenVectorsDeg(
                *normalized,
                effectiveDirection)
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
                if (hardFailure) break;
            }
            if (found || hardFailure) break;
        }
    } else {
        for (double bDeg : bValues) {
            for (double aDeg : aValues) {
                if (evaluate(aDeg, bDeg)) {
                    found = true;
                    break;
                }
                if (hardFailure) break;
            }
            if (found || hardFailure) break;
        }
    }
    if (hardFailure) {
        return reject(
            ExecutionPlanStatus::InvalidExecutionPlan,
            ExecutionPlanFailureReason::ReachabilityCheckFailed,
            evaluated);
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
        directionPolicy,
        effectiveDirection,
        strikeMode,
        physicalSurface,
        *config->tableDownDirectionBase0XY,
        *config->pullModeMinBottomDistanceMm,
        bottomDistanceMm,
        tableDownDirectionDot,
        strikeOffsetMm,
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
        *BilliardConfig::STANDBY_JOINT_REFERENCE.calibrationRevision,
        BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg,
        {PlannedMotionIntent::CartesianPtpToSafeApproach,
         PlannedMotionIntent::LinearToStrikeReady,
         PlannedMotionIntent::RuntimeActualPoseVerticalSafeLift,
         PlannedMotionIntent::JointPtpToStandby},
        {{{PlannedMotionIntent::CartesianPtpToSafeApproach,
           PlannedStagePrecondition::PolicyAndCalibrationAccepted,
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
          {PlannedMotionIntent::JointPtpToStandby,
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
    };

    const ExecutionPlanResult preferredAttempt =
        attemptForStrikeMode(preferredStrikeMode);
    // forcedStrikeMode是呼叫端（preflight NotReachable後的對側重試）明確
    // 指定的單一模式，不再套用這裡的NoAcceptedPoseCandidate對側自動重試。
    if (forcedStrikeMode ||
        preferredAttempt.status() == ExecutionPlanStatus::Success ||
        !preferredAttempt.diagnostic() ||
        preferredAttempt.diagnostic()->reason !=
            ExecutionPlanFailureReason::NoAcceptedPoseCandidate) {
        return preferredAttempt;
    }
    return attemptForStrikeMode(
        preferredStrikeMode == StrikeMode::Push ? StrikeMode::Pull
                                                 : StrikeMode::Push);
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
