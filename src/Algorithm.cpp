#include "Algorithm.h"

#include "MathUtils.h"
#include "TargetSelector.h"

#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace {
constexpr double NUMERICAL_EPSILON = 1e-9;

bool samePoint(Point first, Point second) noexcept
{
    return first.x == second.x && first.y == second.y;
}

std::optional<DirectPotGenerationStatus> validateInputs(
    const StableTableState& table,
    const EligibleTarget& selectedTarget,
    const ResolvedTableGeometry& geometry) noexcept
{
    if (!BilliardMath::isFinite(table.cueBall)) {
        return DirectPotGenerationStatus::InvalidStableState;
    }
    for (const auto& ball : table.objectBalls) {
        if (ball && !BilliardMath::isFinite(*ball)) {
            return DirectPotGenerationStatus::InvalidStableState;
        }
    }
    for (const Point pocket : table.pockets) {
        if (!BilliardMath::isFinite(pocket)) {
            return DirectPotGenerationStatus::InvalidStableState;
        }
    }

    if (selectedTarget.ballNumber < 1 || selectedTarget.ballNumber > 9 ||
        !BilliardMath::isFinite(selectedTarget.center)) {
        return DirectPotGenerationStatus::SelectedTargetMismatch;
    }
    const std::size_t targetIndex =
        static_cast<std::size_t>(selectedTarget.ballNumber - 1);
    if (!table.objectBalls[targetIndex] ||
        !samePoint(*table.objectBalls[targetIndex], selectedTarget.center)) {
        return DirectPotGenerationStatus::SelectedTargetMismatch;
    }
    for (std::size_t index = 0; index < targetIndex; ++index) {
        if (table.objectBalls[index]) {
            return DirectPotGenerationStatus::SelectedTargetMismatch;
        }
    }

    if (geometry.calibrationRevision.empty() ||
        !std::isfinite(geometry.ballRadiusMm) || geometry.ballRadiusMm <= 0.0 ||
        !std::isfinite(geometry.ballDiameterMm) || geometry.ballDiameterMm <= 0.0 ||
        std::fabs(geometry.ballDiameterMm - 2.0 * geometry.ballRadiusMm) >
            NUMERICAL_EPSILON ||
        !std::isfinite(geometry.collisionMarginMm) || geometry.collisionMarginMm < 0.0) {
        return DirectPotGenerationStatus::InvalidGeometryConfiguration;
    }
    for (std::size_t index = 0; index < table.pockets.size(); ++index) {
        if (static_cast<std::size_t>(geometry.pockets[index].id) != index ||
            !samePoint(
                geometry.pockets[index].wirePocketCenter,
                table.pockets[index])) {
            return DirectPotGenerationStatus::InvalidGeometryConfiguration;
        }
    }
    return std::nullopt;
}

DirectPotCandidateDiagnostic rejected(
    BilliardConfig::PocketId pocketId,
    DirectPotRejectionReason reason,
    GeometryStatus geometryStatus,
    std::optional<std::size_t> relatedObstacleIndex = std::nullopt)
{
    return {pocketId, reason, geometryStatus, relatedObstacleIndex};
}

std::optional<std::size_t> relatedIndex(const GeometryCheckResult& result)
{
    return result.diagnostic()
        ? result.diagnostic()->relatedIndex
        : std::nullopt;
}

KickPotGenerationStatus mapKickInputFailure(DirectPotGenerationStatus status) noexcept
{
    switch (status) {
    case DirectPotGenerationStatus::InvalidStableState:
        return KickPotGenerationStatus::InvalidStableState;
    case DirectPotGenerationStatus::InvalidGeometryConfiguration:
        return KickPotGenerationStatus::InvalidGeometryConfiguration;
    case DirectPotGenerationStatus::SelectedTargetMismatch:
        return KickPotGenerationStatus::SelectedTargetMismatch;
    case DirectPotGenerationStatus::Success:
        break;
    }
    return KickPotGenerationStatus::InvalidGeometryConfiguration;
}

bool validKickConfig(const BilliardConfig::KickGeometryConfig& config) noexcept
{
    return std::isfinite(config.maxKickRailAngleDeg) &&
        config.maxKickRailAngleDeg >= 0.0 && config.maxKickRailAngleDeg <= 90.0 &&
        std::isfinite(config.reflectionDirectionTolerance) &&
        config.reflectionDirectionTolerance >= 0.0 &&
        config.reflectionDirectionTolerance <= 2.0 &&
        std::isfinite(config.reflectionAngleToleranceDeg) &&
        config.reflectionAngleToleranceDeg >= 0.0 &&
        config.reflectionAngleToleranceDeg <= 180.0;
}

KickPotCandidateDiagnostic kickRejected(
    BilliardConfig::PocketId pocketId,
    BilliardConfig::RailId railId,
    KickPotRejectionReason reason,
    GeometryStatus geometryStatus,
    std::optional<std::size_t> relatedObstacleIndex = std::nullopt)
{
    return {pocketId, railId, reason, geometryStatus, relatedObstacleIndex};
}

std::optional<double> angleFromNormal(Vector2D direction, Vector2D normal) noexcept
{
    if (!BilliardMath::isFinite(direction) || !BilliardMath::isFinite(normal)) {
        return std::nullopt;
    }
    const double cosine = std::clamp(
        direction.x * normal.x + direction.y * normal.y,
        -1.0,
        1.0);
    const double angle = std::acos(cosine) * 180.0 / BilliardMath::PI;
    return std::isfinite(angle) ? std::optional<double>{angle} : std::nullopt;
}

struct NormalizedWeights {
    BilliardConfig::ScoringWeights effective;
    double rawSum;
};

std::optional<NormalizedWeights> normalizeWeights(
    const BilliardConfig::ScoringConfig& config) noexcept
{
    const std::array<double, 6> raw{{
        config.rawWeights.kickPenalty,
        config.rawWeights.cuttingAngle,
        config.rawWeights.totalDistance,
        config.rawWeights.clearanceRisk,
        config.rawWeights.pocketEntryAngle,
        config.rawWeights.kickRailAngleRisk}};
    long double sum = 0.0L;
    for (const double weight : raw) {
        if (!std::isfinite(weight) || weight < 0.0) {
            return std::nullopt;
        }
        sum += static_cast<long double>(weight);
        if (!std::isfinite(sum) ||
            sum > static_cast<long double>(std::numeric_limits<double>::max())) {
            return std::nullopt;
        }
    }
    const double rawSum = static_cast<double>(sum);
    if (!std::isfinite(rawSum) || rawSum <= 0.0) {
        return std::nullopt;
    }
    BilliardConfig::ScoringWeights effective{
        raw[0] / rawSum,
        raw[1] / rawSum,
        raw[2] / rawSum,
        raw[3] / rawSum,
        raw[4] / rawSum,
        raw[5] / rawSum};
    const double effectiveSum = effective.sum();
    if (!std::isfinite(config.effectiveWeightSumTolerance) ||
        config.effectiveWeightSumTolerance < 0.0 ||
        !std::isfinite(effectiveSum) ||
        std::fabs(effectiveSum - 1.0) > config.effectiveWeightSumTolerance) {
        return std::nullopt;
    }
    return NormalizedWeights{effective, rawSum};
}

bool validScoringRanges(
    const BilliardConfig::ScoringConfig& config,
    const ResolvedTableGeometry& geometry) noexcept
{
    const double blockedThreshold = geometry.ballDiameterMm + geometry.collisionMarginMm;
    const bool validMode =
        config.planningMode == BilliardConfig::PlanningMode::PotOnly ||
        config.planningMode == BilliardConfig::PlanningMode::ManualResearch;
    return validMode &&
        std::isfinite(config.maxCutAngleDeg) && config.maxCutAngleDeg > 0.0 &&
        std::isfinite(config.minDistanceMm) && config.minDistanceMm >= 0.0 &&
        std::isfinite(config.maxDistanceMm) &&
        config.maxDistanceMm > config.minDistanceMm &&
        std::isfinite(blockedThreshold) && blockedThreshold >= 0.0 &&
        std::isfinite(config.preferredClearanceMm) &&
        config.preferredClearanceMm > blockedThreshold &&
        std::isfinite(config.tieEpsilon) && config.tieEpsilon >= 0.0;
}

std::optional<double> segmentLength(Segment2D segment) noexcept
{
    return BilliardMath::getDistance(segment.start, segment.end);
}

bool accumulateDistance(double& total, Segment2D segment) noexcept
{
    const auto length = segmentLength(segment);
    if (!length || !std::isfinite(*length) || *length <= 0.0 ||
        total > std::numeric_limits<double>::max() - *length) {
        return false;
    }
    total += *length;
    return std::isfinite(total);
}

bool accumulateClearance(
    std::optional<double>& minimum,
    Segment2D segment,
    const std::vector<Point>& obstacles,
    const std::vector<std::size_t>& exclusions) noexcept
{
    const auto clearance = BilliardPhysics::computeMinimumSegmentClearance(
        segment,
        obstacles,
        exclusions);
    if (!clearance.value()) {
        return false;
    }
    if (clearance.value()->millimeters &&
        (!minimum || *clearance.value()->millimeters < *minimum)) {
        minimum = *clearance.value()->millimeters;
    }
    return true;
}

std::optional<double> normalizedRatio(double value, double maximum) noexcept
{
    if (!std::isfinite(value) || value < 0.0 ||
        !std::isfinite(maximum) || maximum <= 0.0) {
        return std::nullopt;
    }
    return std::clamp(value / maximum, 0.0, 1.0);
}

std::optional<double> normalizedDistance(
    double value,
    const BilliardConfig::ScoringConfig& config) noexcept
{
    const double denominator = config.maxDistanceMm - config.minDistanceMm;
    if (!std::isfinite(value) || value < 0.0 ||
        !std::isfinite(denominator) || denominator <= 0.0) {
        return std::nullopt;
    }
    return std::clamp(
        (value - config.minDistanceMm) / denominator,
        0.0,
        1.0);
}

std::optional<double> normalizedClearanceRisk(
    const std::optional<double>& clearance,
    double blockedThreshold,
    double preferredClearance) noexcept
{
    if (!clearance) {
        return 0.0;
    }
    const double denominator = preferredClearance - blockedThreshold;
    if (!std::isfinite(*clearance) || *clearance <= blockedThreshold ||
        !std::isfinite(denominator) || denominator <= 0.0) {
        return std::nullopt;
    }
    return 1.0 - std::clamp(
        (*clearance - blockedThreshold) / denominator,
        0.0,
        1.0);
}

std::optional<double> weightedTotal(
    const ScoringNormalizedCosts& costs,
    const BilliardConfig::ScoringWeights& weights,
    double tolerance) noexcept
{
    const std::array<double, 6> costValues{{
        costs.kickPenalty,
        costs.cuttingAngle,
        costs.totalDistance,
        costs.clearanceRisk,
        costs.pocketEntryAngle,
        costs.kickRailAngleRisk}};
    const std::array<double, 6> weightValues{{
        weights.kickPenalty,
        weights.cuttingAngle,
        weights.totalDistance,
        weights.clearanceRisk,
        weights.pocketEntryAngle,
        weights.kickRailAngleRisk}};
    long double total = 0.0L;
    for (std::size_t index = 0; index < costValues.size(); ++index) {
        total += static_cast<long double>(costValues[index]) *
            static_cast<long double>(weightValues[index]);
    }
    if (!std::isfinite(total) || total < 0.0L ||
        total > 1.0L + static_cast<long double>(tolerance)) {
        return std::nullopt;
    }
    const double rounded = static_cast<double>(total);
    if (!std::isfinite(rounded)) {
        return std::nullopt;
    }
    return std::clamp(rounded, 0.0, 1.0);
}

bool sameEligibleTarget(EligibleTarget first, EligibleTarget second) noexcept
{
    return first.ballNumber == second.ballNumber && samePoint(first.center, second.center);
}

bool sameSegment(Segment2D first, Segment2D second) noexcept
{
    return samePoint(first.start, second.start) && samePoint(first.end, second.end);
}

bool sameDirectCandidate(
    const DirectPotCandidate& candidate,
    const DirectPotCandidate& authoritative) noexcept
{
    return sameEligibleTarget(candidate.target, authoritative.target) &&
        candidate.pocketId == authoritative.pocketId &&
        samePoint(candidate.virtualPocketTarget, authoritative.virtualPocketTarget) &&
        samePoint(candidate.ghostBallPoint.center, authoritative.ghostBallPoint.center) &&
        sameSegment(candidate.cuePath, authoritative.cuePath) &&
        sameSegment(candidate.targetPath, authoritative.targetPath) &&
        candidate.cuttingAngleDeg == authoritative.cuttingAngleDeg &&
        candidate.pocketEntryAngleDeg == authoritative.pocketEntryAngleDeg;
}

bool sameKickCandidate(
    const KickPotCandidate& candidate,
    const KickPotCandidate& authoritative) noexcept
{
    return sameEligibleTarget(candidate.target, authoritative.target) &&
        candidate.pocketId == authoritative.pocketId &&
        candidate.railId == authoritative.railId &&
        samePoint(candidate.virtualPocketTarget, authoritative.virtualPocketTarget) &&
        samePoint(candidate.ghostBallPoint.center, authoritative.ghostBallPoint.center) &&
        samePoint(candidate.reboundPoint, authoritative.reboundPoint) &&
        sameSegment(candidate.cuePathFirst, authoritative.cuePathFirst) &&
        sameSegment(candidate.cuePathSecond, authoritative.cuePathSecond) &&
        sameSegment(candidate.targetPath, authoritative.targetPath) &&
        candidate.cuttingAngleDeg == authoritative.cuttingAngleDeg &&
        candidate.pocketEntryAngleDeg == authoritative.pocketEntryAngleDeg &&
        candidate.incidenceAngleDeg == authoritative.incidenceAngleDeg &&
        candidate.reflectionAngleDeg == authoritative.reflectionAngleDeg;
}

bool sameDirectDiagnostic(
    const DirectPotCandidateDiagnostic& diagnostic,
    const DirectPotCandidateDiagnostic& authoritative) noexcept
{
    return diagnostic.pocketId == authoritative.pocketId &&
        diagnostic.reason == authoritative.reason &&
        diagnostic.geometryStatus == authoritative.geometryStatus &&
        diagnostic.relatedObstacleIndex == authoritative.relatedObstacleIndex;
}

bool sameKickDiagnostic(
    const KickPotCandidateDiagnostic& diagnostic,
    const KickPotCandidateDiagnostic& authoritative) noexcept
{
    return diagnostic.pocketId == authoritative.pocketId &&
        diagnostic.railId == authoritative.railId &&
        diagnostic.reason == authoritative.reason &&
        diagnostic.geometryStatus == authoritative.geometryStatus &&
        diagnostic.relatedObstacleIndex == authoritative.relatedObstacleIndex;
}

bool sameDirectEvaluation(
    const DirectPotEvaluation& submitted,
    const DirectPotEvaluation& authoritative) noexcept
{
    if (!sameEligibleTarget(submitted.target, authoritative.target)) {
        return false;
    }
    for (std::size_t pocket = 0; pocket < submitted.feasible.size(); ++pocket) {
        if (submitted.feasible[pocket].has_value() !=
                authoritative.feasible[pocket].has_value() ||
            submitted.rejected[pocket].has_value() !=
                authoritative.rejected[pocket].has_value() ||
            (submitted.feasible[pocket] &&
             !sameDirectCandidate(
                 *submitted.feasible[pocket],
                 *authoritative.feasible[pocket])) ||
            (submitted.rejected[pocket] &&
             !sameDirectDiagnostic(
                 *submitted.rejected[pocket],
                 *authoritative.rejected[pocket]))) {
            return false;
        }
    }
    return true;
}

bool sameKickEvaluation(
    const KickPotEvaluation& submitted,
    const KickPotEvaluation& authoritative) noexcept
{
    if (!sameEligibleTarget(submitted.target, authoritative.target)) {
        return false;
    }
    for (std::size_t pocket = 0; pocket < submitted.feasible.size(); ++pocket) {
        for (std::size_t rail = 0; rail < submitted.feasible[pocket].size(); ++rail) {
            if (submitted.feasible[pocket][rail].has_value() !=
                    authoritative.feasible[pocket][rail].has_value() ||
                submitted.rejected[pocket][rail].has_value() !=
                    authoritative.rejected[pocket][rail].has_value() ||
                (submitted.feasible[pocket][rail] &&
                 !sameKickCandidate(
                     *submitted.feasible[pocket][rail],
                     *authoritative.feasible[pocket][rail])) ||
                (submitted.rejected[pocket][rail] &&
                 !sameKickDiagnostic(
                     *submitted.rejected[pocket][rail],
                     *authoritative.rejected[pocket][rail]))) {
                return false;
            }
        }
    }
    return true;
}

bool tieBreakBetter(const ScoredPotCandidate& candidate, const ScoredPotCandidate& current) noexcept
{
    const int candidateKickCount = candidate.kind == PotCandidateKind::Kick ? 1 : 0;
    const int currentKickCount = current.kind == PotCandidateKind::Kick ? 1 : 0;
    if (candidateKickCount != currentKickCount) {
        return candidateKickCount < currentKickCount;
    }
    if (candidate.audit.rawCosts.cuttingAngleDeg !=
        current.audit.rawCosts.cuttingAngleDeg) {
        return candidate.audit.rawCosts.cuttingAngleDeg <
            current.audit.rawCosts.cuttingAngleDeg;
    }
    const auto candidateClearance = candidate.audit.rawCosts.minimumClearanceMm;
    const auto currentClearance = current.audit.rawCosts.minimumClearanceMm;
    if (candidateClearance.has_value() != currentClearance.has_value()) {
        return !candidateClearance.has_value();
    }
    if (candidateClearance && *candidateClearance != *currentClearance) {
        return *candidateClearance > *currentClearance;
    }
    if (candidate.audit.rawCosts.totalDistanceMm !=
        current.audit.rawCosts.totalDistanceMm) {
        return candidate.audit.rawCosts.totalDistanceMm <
            current.audit.rawCosts.totalDistanceMm;
    }
    if (candidate.pocketId != current.pocketId) {
        return static_cast<std::size_t>(candidate.pocketId) <
            static_cast<std::size_t>(current.pocketId);
    }
    const std::size_t candidateRail = candidate.railId
        ? static_cast<std::size_t>(*candidate.railId)
        : 0;
    const std::size_t currentRail = current.railId
        ? static_cast<std::size_t>(*current.railId)
        : 0;
    return candidateRail < currentRail;
}
}

DirectPotGenerationResult BilliardAlgorithm::generateDirectPotCandidates(
    const StableTableState& table,
    const EligibleTarget& selectedTarget,
    const ResolvedTableGeometry& geometry)
{
    if (const auto failure = validateInputs(table, selectedTarget, geometry)) {
        return DirectPotGenerationResult::rejected(*failure);
    }

    std::vector<Point> cuePathObstacles;
    cuePathObstacles.reserve(table.objectBalls.size());
    std::optional<std::size_t> selectedObstacleIndex;
    for (std::size_t index = 0; index < table.objectBalls.size(); ++index) {
        if (!table.objectBalls[index]) {
            continue;
        }
        if (index == static_cast<std::size_t>(selectedTarget.ballNumber - 1)) {
            selectedObstacleIndex = cuePathObstacles.size();
        }
        cuePathObstacles.push_back(*table.objectBalls[index]);
    }
    if (!selectedObstacleIndex) {
        return DirectPotGenerationResult::rejected(
            DirectPotGenerationStatus::SelectedTargetMismatch);
    }
    const std::vector<std::size_t> movingBallExclusion{*selectedObstacleIndex};
    std::vector<Point> targetPathObstacles = cuePathObstacles;

    DirectPotEvaluation evaluation{selectedTarget, {}, {}};
    for (std::size_t index = 0; index < geometry.pockets.size(); ++index) {
        const ResolvedPocketModel& pocket = geometry.pockets[index];
        const GhostBallResult ghost = BilliardPhysics::computeGhostBallPoint(
            table.cueBall,
            selectedTarget.center,
            pocket.virtualPocketTarget,
            geometry.ballRadiusMm);
        if (!ghost.value()) {
            evaluation.rejected[index] = rejected(
                pocket.id,
                DirectPotRejectionReason::GhostGeometryInvalid,
                ghost.status());
            continue;
        }

        const Segment2D cuePath{table.cueBall, ghost.value()->center};
        const Segment2D targetPath{
            selectedTarget.center,
            pocket.virtualPocketTarget};

        const GeometryCheckResult cueRegion =
            BilliardPhysics::checkSegmentWithinPlayableRegion(
                cuePath,
                geometry.playableBallCenterRegion);
        if (cueRegion.status() != GeometryStatus::Clear) {
            evaluation.rejected[index] = rejected(
                pocket.id,
                DirectPotRejectionReason::CuePathInvalid,
                cueRegion.status(),
                relatedIndex(cueRegion));
            continue;
        }

        const GeometryCheckResult targetRegion =
            BilliardPhysics::checkTargetPathToPocket(
                targetPath,
                pocket.id,
                geometry.pockets,
                geometry.playableBallCenterRegion);
        if (targetRegion.status() != GeometryStatus::Clear) {
            evaluation.rejected[index] = rejected(
                pocket.id,
                DirectPotRejectionReason::TargetPathInvalid,
                targetRegion.status(),
                relatedIndex(targetRegion));
            continue;
        }

        const auto pocketEntry = BilliardPhysics::computePocketEntryAngle(
            selectedTarget.center,
            pocket);
        if (!pocketEntry.value()) {
            evaluation.rejected[index] = rejected(
                pocket.id,
                DirectPotRejectionReason::PocketEntryRejected,
                pocketEntry.status());
            continue;
        }

        const auto cueDirection = BilliardMath::getVector(cuePath.start, cuePath.end);
        const auto targetDirection = BilliardMath::getVector(
            targetPath.start,
            targetPath.end);
        const auto cutAngle = cueDirection && targetDirection
            ? BilliardMath::getAngleBetweenVectorsDeg(*cueDirection, *targetDirection)
            : std::nullopt;
        if (!cutAngle || *cutAngle >= 90.0) {
            evaluation.rejected[index] = rejected(
                pocket.id,
                DirectPotRejectionReason::CutAngleInvalid,
                cutAngle ? GeometryStatus::DirectionRejected
                         : GeometryStatus::DegenerateGeometry);
            continue;
        }

        const GeometryCheckResult cueCollision =
            BilliardPhysics::checkSegmentCollision(
                cuePath,
                cuePathObstacles,
                movingBallExclusion,
                geometry.ballDiameterMm,
                geometry.collisionMarginMm);
        if (cueCollision.status() != GeometryStatus::Clear) {
            evaluation.rejected[index] = rejected(
                pocket.id,
                cueCollision.status() == GeometryStatus::Blocked
                    ? DirectPotRejectionReason::CuePathBlocked
                    : DirectPotRejectionReason::CuePathInvalid,
                cueCollision.status(),
                relatedIndex(cueCollision));
            continue;
        }

        const GeometryCheckResult targetCollision =
            BilliardPhysics::checkSegmentCollision(
                targetPath,
                targetPathObstacles,
                movingBallExclusion,
                geometry.ballDiameterMm,
                geometry.collisionMarginMm);
        if (targetCollision.status() != GeometryStatus::Clear) {
            evaluation.rejected[index] = rejected(
                pocket.id,
                targetCollision.status() == GeometryStatus::Blocked
                    ? DirectPotRejectionReason::TargetPathBlocked
                    : DirectPotRejectionReason::TargetPathInvalid,
                targetCollision.status(),
                relatedIndex(targetCollision));
            continue;
        }

        evaluation.feasible[index] = DirectPotCandidate{
            selectedTarget,
            pocket.id,
            pocket.virtualPocketTarget,
            *ghost.value(),
            cuePath,
            targetPath,
            *cutAngle,
            pocketEntry.value()->degrees};
    }

    return DirectPotGenerationResult::success(std::move(evaluation));
}

KickPotGenerationResult BilliardAlgorithm::generateKickPotCandidates(
    const StableTableState& table,
    const EligibleTarget& selectedTarget,
    const ResolvedTableGeometry& geometry,
    const std::optional<BilliardConfig::KickGeometryConfig>& config)
{
    if (!config) {
        return KickPotGenerationResult::rejected(
            KickPotGenerationStatus::ConfigurationMissing);
    }
    if (!validKickConfig(*config)) {
        return KickPotGenerationResult::rejected(
            KickPotGenerationStatus::InvalidGeometryConfiguration);
    }
    if (const auto failure = validateInputs(table, selectedTarget, geometry)) {
        return KickPotGenerationResult::rejected(mapKickInputFailure(*failure));
    }
    for (std::size_t railIndex = 0;
         railIndex < geometry.railReflectionRegion.rails.size();
         ++railIndex) {
        const EffectiveCueBallRailSegment& rail =
            geometry.railReflectionRegion.rails[railIndex];
        if (static_cast<std::size_t>(rail.physicalRailId) != railIndex ||
            BilliardPhysics::checkEffectiveRailForReflection(
                rail,
                geometry.playableBallCenterRegion).status() != GeometryStatus::Clear) {
            return KickPotGenerationResult::rejected(
                KickPotGenerationStatus::InvalidGeometryConfiguration);
        }
    }

    std::vector<Point> obstacles;
    obstacles.reserve(table.objectBalls.size());
    std::optional<std::size_t> selectedObstacleIndex;
    for (std::size_t index = 0; index < table.objectBalls.size(); ++index) {
        if (!table.objectBalls[index]) {
            continue;
        }
        if (index == static_cast<std::size_t>(selectedTarget.ballNumber - 1)) {
            selectedObstacleIndex = obstacles.size();
        }
        obstacles.push_back(*table.objectBalls[index]);
    }
    if (!selectedObstacleIndex) {
        return KickPotGenerationResult::rejected(
            KickPotGenerationStatus::SelectedTargetMismatch);
    }
    const std::vector<std::size_t> selectedTargetExclusion{*selectedObstacleIndex};

    KickPotEvaluation evaluation{selectedTarget, {}, {}};
    for (std::size_t pocketIndex = 0; pocketIndex < geometry.pockets.size(); ++pocketIndex) {
        const ResolvedPocketModel& pocket = geometry.pockets[pocketIndex];
        for (std::size_t railIndex = 0;
             railIndex < geometry.railReflectionRegion.rails.size();
             ++railIndex) {
            const EffectiveCueBallRailSegment& rail =
                geometry.railReflectionRegion.rails[railIndex];
            auto reject = [&](KickPotRejectionReason reason,
                              GeometryStatus status,
                              std::optional<std::size_t> related = std::nullopt) {
                evaluation.rejected[pocketIndex][railIndex] = kickRejected(
                    pocket.id,
                    rail.physicalRailId,
                    reason,
                    status,
                    related);
            };

            const GhostBallResult ghost = BilliardPhysics::computeGhostBallPoint(
                table.cueBall,
                selectedTarget.center,
                pocket.virtualPocketTarget,
                geometry.ballRadiusMm);
            if (!ghost.value()) {
                reject(KickPotRejectionReason::GhostGeometryInvalid, ghost.status());
                continue;
            }
            const Segment2D targetPath{
                selectedTarget.center,
                pocket.virtualPocketTarget};
            const GeometryCheckResult targetRegion =
                BilliardPhysics::checkTargetPathToPocket(
                    targetPath,
                    pocket.id,
                    geometry.pockets,
                    geometry.playableBallCenterRegion);
            if (targetRegion.status() != GeometryStatus::Clear) {
                reject(
                    KickPotRejectionReason::TargetPathInvalid,
                    targetRegion.status(),
                    relatedIndex(targetRegion));
                continue;
            }
            const auto pocketEntry = BilliardPhysics::computePocketEntryAngle(
                selectedTarget.center,
                pocket);
            if (!pocketEntry.value()) {
                reject(KickPotRejectionReason::PocketEntryRejected, pocketEntry.status());
                continue;
            }
            const GeometryCheckResult targetCollision =
                BilliardPhysics::checkSegmentCollision(
                    targetPath,
                    obstacles,
                    selectedTargetExclusion,
                    geometry.ballDiameterMm,
                    geometry.collisionMarginMm);
            if (targetCollision.status() != GeometryStatus::Clear) {
                reject(
                    targetCollision.status() == GeometryStatus::Blocked
                        ? KickPotRejectionReason::TargetPathBlocked
                        : KickPotRejectionReason::TargetPathInvalid,
                    targetCollision.status(),
                    relatedIndex(targetCollision));
                continue;
            }

            const auto mirroredGhost = BilliardPhysics::mirrorPointAcrossEffectiveRail(
                ghost.value()->center,
                rail);
            if (!mirroredGhost.value()) {
                reject(KickPotRejectionReason::RailGeometryInvalid, mirroredGhost.status());
                continue;
            }
            const auto rebound = BilliardPhysics::intersectRayWithEffectiveRail(
                table.cueBall,
                *mirroredGhost.value(),
                rail);
            if (!rebound.value()) {
                reject(
                    KickPotRejectionReason::NoRailIntersection,
                    rebound.status());
                continue;
            }

            const Segment2D cuePathFirst{table.cueBall, *rebound.value()};
            const Segment2D cuePathSecond{*rebound.value(), ghost.value()->center};
            const GeometryCheckResult firstRegion =
                BilliardPhysics::checkSegmentWithinPlayableRegion(
                    cuePathFirst,
                    geometry.playableBallCenterRegion);
            if (firstRegion.status() != GeometryStatus::Clear) {
                reject(
                    KickPotRejectionReason::CueFirstSegmentInvalid,
                    firstRegion.status(),
                    relatedIndex(firstRegion));
                continue;
            }
            const GeometryCheckResult secondRegion =
                BilliardPhysics::checkSegmentWithinPlayableRegion(
                    cuePathSecond,
                    geometry.playableBallCenterRegion);
            if (secondRegion.status() != GeometryStatus::Clear) {
                reject(
                    KickPotRejectionReason::CueSecondSegmentInvalid,
                    secondRegion.status(),
                    relatedIndex(secondRegion));
                continue;
            }

            const auto incomingRaw = BilliardMath::getVector(
                cuePathFirst.start,
                cuePathFirst.end);
            const auto outgoingRaw = BilliardMath::getVector(
                cuePathSecond.start,
                cuePathSecond.end);
            const auto incoming = incomingRaw
                ? BilliardMath::normalize(*incomingRaw)
                : std::nullopt;
            const auto outgoing = outgoingRaw
                ? BilliardMath::normalize(*outgoingRaw)
                : std::nullopt;
            if (!incoming || !outgoing) {
                reject(
                    KickPotRejectionReason::ReflectionInvariantFailed,
                    GeometryStatus::DegenerateGeometry);
                continue;
            }
            const double incomingNormal =
                incoming->x * rail.inwardUnitNormal.x +
                incoming->y * rail.inwardUnitNormal.y;
            const double outgoingNormal =
                outgoing->x * rail.inwardUnitNormal.x +
                outgoing->y * rail.inwardUnitNormal.y;
            const Vector2D idealReflected{
                incoming->x - 2.0 * incomingNormal * rail.inwardUnitNormal.x,
                incoming->y - 2.0 * incomingNormal * rail.inwardUnitNormal.y};
            const double reflectionDifference = std::hypot(
                outgoing->x - idealReflected.x,
                outgoing->y - idealReflected.y);
            const Vector2D negativeIncoming{-incoming->x, -incoming->y};
            const auto incidenceAngle = angleFromNormal(
                negativeIncoming,
                rail.inwardUnitNormal);
            const auto reflectionAngle = angleFromNormal(
                *outgoing,
                rail.inwardUnitNormal);
            if (!std::isfinite(incomingNormal) || !std::isfinite(outgoingNormal) ||
                incomingNormal >= 0.0 || outgoingNormal <= 0.0 ||
                !std::isfinite(reflectionDifference) ||
                reflectionDifference > config->reflectionDirectionTolerance ||
                !incidenceAngle || !reflectionAngle ||
                std::fabs(*incidenceAngle - *reflectionAngle) >
                    config->reflectionAngleToleranceDeg) {
                reject(
                    KickPotRejectionReason::ReflectionInvariantFailed,
                    GeometryStatus::DirectionRejected);
                continue;
            }
            if (*incidenceAngle > config->maxKickRailAngleDeg) {
                reject(
                    KickPotRejectionReason::KickAngleRejected,
                    GeometryStatus::DirectionRejected);
                continue;
            }

            const auto targetDirectionRaw = BilliardMath::getVector(
                targetPath.start,
                targetPath.end);
            const auto cutAngle = targetDirectionRaw
                ? BilliardMath::getAngleBetweenVectorsDeg(*outgoingRaw, *targetDirectionRaw)
                : std::nullopt;
            if (!cutAngle || *cutAngle >= 90.0) {
                reject(
                    KickPotRejectionReason::CutAngleInvalid,
                    cutAngle ? GeometryStatus::DirectionRejected
                             : GeometryStatus::DegenerateGeometry);
                continue;
            }

            const GeometryCheckResult firstCollision =
                BilliardPhysics::checkSegmentCollision(
                    cuePathFirst,
                    obstacles,
                    {},
                    geometry.ballDiameterMm,
                    geometry.collisionMarginMm);
            if (firstCollision.status() != GeometryStatus::Clear) {
                reject(
                    firstCollision.status() == GeometryStatus::Blocked
                        ? KickPotRejectionReason::CueFirstSegmentBlocked
                        : KickPotRejectionReason::CueFirstSegmentInvalid,
                    firstCollision.status(),
                    relatedIndex(firstCollision));
                continue;
            }
            const GeometryCheckResult secondCollision =
                BilliardPhysics::checkSegmentCollision(
                    cuePathSecond,
                    obstacles,
                    selectedTargetExclusion,
                    geometry.ballDiameterMm,
                    geometry.collisionMarginMm);
            if (secondCollision.status() != GeometryStatus::Clear) {
                reject(
                    secondCollision.status() == GeometryStatus::Blocked
                        ? KickPotRejectionReason::CueSecondSegmentBlocked
                        : KickPotRejectionReason::CueSecondSegmentInvalid,
                    secondCollision.status(),
                    relatedIndex(secondCollision));
                continue;
            }

            evaluation.feasible[pocketIndex][railIndex] = KickPotCandidate{
                selectedTarget,
                pocket.id,
                rail.physicalRailId,
                pocket.virtualPocketTarget,
                *ghost.value(),
                *rebound.value(),
                cuePathFirst,
                cuePathSecond,
                targetPath,
                *cutAngle,
                pocketEntry.value()->degrees,
                *incidenceAngle,
                *reflectionAngle};
        }
    }
    return KickPotGenerationResult::success(std::move(evaluation));
}

PotSelectionResult BilliardAlgorithm::selectBestPot(
    const StableTableState& table,
    const EligibleTarget& selectedTarget,
    const ResolvedTableGeometry& geometry,
    const DirectPotEvaluation& directCandidates,
    const KickPotEvaluation& kickCandidates,
    const std::optional<BilliardConfig::ScoringConfig>& scoringConfig,
    const std::optional<BilliardConfig::KickGeometryConfig>& kickConfig)
{
    if (!scoringConfig) {
        return PotSelectionResult::rejected(PotSelectionStatus::ConfigurationMissing);
    }
    const auto weights = normalizeWeights(*scoringConfig);
    if (!weights || !validScoringRanges(*scoringConfig, geometry)) {
        return PotSelectionResult::rejected(PotSelectionStatus::InvalidConfiguration);
    }
    if (const auto failure = validateInputs(table, selectedTarget, geometry)) {
        return PotSelectionResult::rejected(
            *failure == DirectPotGenerationStatus::InvalidGeometryConfiguration
                ? PotSelectionStatus::InvalidConfiguration
                : PotSelectionStatus::InvalidCandidateInput);
    }
    if (!directCandidates.isValid() || !kickCandidates.isValid() ||
        !sameEligibleTarget(directCandidates.target, selectedTarget) ||
        !sameEligibleTarget(kickCandidates.target, selectedTarget)) {
        return PotSelectionResult::rejected(PotSelectionStatus::InvalidCandidateInput);
    }

    if (!kickConfig) {
        return PotSelectionResult::rejected(PotSelectionStatus::ConfigurationMissing);
    }
    if (!validKickConfig(*kickConfig)) {
        return PotSelectionResult::rejected(PotSelectionStatus::InvalidConfiguration);
    }

    const auto authoritativeDirect = generateDirectPotCandidates(
        table,
        selectedTarget,
        geometry);
    if (!authoritativeDirect.isValid() || !authoritativeDirect.value()) {
        return PotSelectionResult::rejected(PotSelectionStatus::InvalidCandidateInput);
    }
    const auto regeneratedKick = generateKickPotCandidates(
        table,
        selectedTarget,
        geometry,
        kickConfig);
    if (!regeneratedKick.isValid() || !regeneratedKick.value()) {
        return PotSelectionResult::rejected(PotSelectionStatus::InvalidCandidateInput);
    }
    const KickPotEvaluation& authoritativeKick = *regeneratedKick.value();
    bool authoritativeHasKick = false;
    for (const auto& pocketCandidates : authoritativeKick.feasible) {
        for (const auto& candidate : pocketCandidates) {
            authoritativeHasKick = authoritativeHasKick || candidate.has_value();
        }
    }
    if (authoritativeHasKick && kickConfig->maxKickRailAngleDeg <= 0.0) {
        return PotSelectionResult::rejected(PotSelectionStatus::InvalidConfiguration);
    }
    if (!sameDirectEvaluation(directCandidates, *authoritativeDirect.value()) ||
        !sameKickEvaluation(kickCandidates, authoritativeKick)) {
        return PotSelectionResult::rejected(PotSelectionStatus::InvalidCandidateInput);
    }

    std::vector<Point> obstacles;
    obstacles.reserve(table.objectBalls.size());
    std::optional<std::size_t> selectedObstacleIndex;
    for (std::size_t index = 0; index < table.objectBalls.size(); ++index) {
        if (!table.objectBalls[index]) {
            continue;
        }
        if (index == static_cast<std::size_t>(selectedTarget.ballNumber - 1)) {
            selectedObstacleIndex = obstacles.size();
        }
        obstacles.push_back(*table.objectBalls[index]);
    }
    if (!selectedObstacleIndex) {
        return PotSelectionResult::rejected(PotSelectionStatus::InvalidCandidateInput);
    }
    const std::vector<std::size_t> selectedTargetExclusion{*selectedObstacleIndex};
    const double blockedThreshold = geometry.ballDiameterMm + geometry.collisionMarginMm;

    auto makeAudit = [&](ScoringRawCosts raw,
                         double maximumEntryAngle,
                         double maximumKickAngle)
        -> std::optional<PotScoringAudit> {
        const auto cut = normalizedRatio(raw.cuttingAngleDeg, scoringConfig->maxCutAngleDeg);
        const auto distance = normalizedDistance(raw.totalDistanceMm, *scoringConfig);
        const auto clearance = normalizedClearanceRisk(
            raw.minimumClearanceMm,
            blockedThreshold,
            scoringConfig->preferredClearanceMm);
        const auto entry = normalizedRatio(raw.pocketEntryAngleDeg, maximumEntryAngle);
        const auto rail = raw.kickPenalty == 0.0
            ? std::optional<double>{0.0}
            : normalizedRatio(raw.kickRailAngleDeg, maximumKickAngle);
        if (!std::isfinite(raw.kickPenalty) ||
            (raw.kickPenalty != 0.0 && raw.kickPenalty != 1.0) ||
            !cut || !distance || !clearance || !entry || !rail) {
            return std::nullopt;
        }
        const ScoringNormalizedCosts normalized{
            raw.kickPenalty,
            *cut,
            *distance,
            *clearance,
            *entry,
            *rail};
        const auto total = weightedTotal(
            normalized,
            weights->effective,
            scoringConfig->effectiveWeightSumTolerance);
        if (!total) {
            return std::nullopt;
        }
        return PotScoringAudit{
            raw,
            normalized,
            scoringConfig->rawWeights,
            weights->rawSum,
            weights->effective,
            ScoringNormalizationSnapshot{
                scoringConfig->effectiveWeightSumTolerance,
                scoringConfig->maxCutAngleDeg,
                scoringConfig->minDistanceMm,
                scoringConfig->maxDistanceMm,
                blockedThreshold,
                scoringConfig->preferredClearanceMm,
                maximumEntryAngle,
                raw.kickPenalty == 1.0
                    ? std::optional<double>{maximumKickAngle}
                    : std::nullopt,
                scoringConfig->tieEpsilon},
            *total};
    };

    std::vector<ScoredPotCandidate> scored;
    scored.reserve(42);
    for (std::size_t pocketIndex = directCandidates.feasible.size();
         pocketIndex-- > 0;) {
        if (!directCandidates.feasible[pocketIndex]) {
            continue;
        }
        const DirectPotCandidate& candidate = *directCandidates.feasible[pocketIndex];
        const ResolvedPocketModel& pocket = geometry.pockets[pocketIndex];
        const auto& expected = authoritativeDirect.value()->feasible[pocketIndex];
        if (!sameEligibleTarget(candidate.target, selectedTarget) ||
            candidate.pocketId != pocket.id ||
            !samePoint(candidate.virtualPocketTarget, pocket.virtualPocketTarget) ||
            !expected || !sameDirectCandidate(candidate, *expected)) {
            return PotSelectionResult::rejected(PotSelectionStatus::InvalidCandidateInput);
        }
        double totalDistance = 0.0;
        if (!accumulateDistance(totalDistance, candidate.cuePath) ||
            !accumulateDistance(totalDistance, candidate.targetPath)) {
            return PotSelectionResult::rejected(PotSelectionStatus::NumericalFailure);
        }
        std::optional<double> minimumClearance;
        if (!accumulateClearance(
                minimumClearance,
                candidate.cuePath,
                obstacles,
                selectedTargetExclusion) ||
            !accumulateClearance(
                minimumClearance,
                candidate.targetPath,
                obstacles,
                selectedTargetExclusion)) {
            return PotSelectionResult::rejected(PotSelectionStatus::NumericalFailure);
        }
        const auto audit = makeAudit(
            ScoringRawCosts{
                0.0,
                candidate.cuttingAngleDeg,
                totalDistance,
                minimumClearance,
                candidate.pocketEntryAngleDeg,
                0.0},
            pocket.maxEntryAngleDeg,
            1.0);
        if (!audit) {
            return PotSelectionResult::rejected(PotSelectionStatus::NumericalFailure);
        }
        scored.push_back({
            PotCandidateKind::Direct,
            selectedTarget,
            candidate.pocketId,
            std::nullopt,
            PotCandidateValue{candidate},
            *audit});
    }

    for (std::size_t pocketIndex = kickCandidates.feasible.size();
         pocketIndex-- > 0;) {
        for (std::size_t railIndex = kickCandidates.feasible[pocketIndex].size();
             railIndex-- > 0;) {
            if (!kickCandidates.feasible[pocketIndex][railIndex]) {
                continue;
            }
            const KickPotCandidate& candidate =
                *kickCandidates.feasible[pocketIndex][railIndex];
            const ResolvedPocketModel& pocket = geometry.pockets[pocketIndex];
            const auto& expected =
                authoritativeKick.feasible[pocketIndex][railIndex];
            if (!sameEligibleTarget(candidate.target, selectedTarget) ||
                candidate.pocketId != pocket.id ||
                static_cast<std::size_t>(candidate.railId) != railIndex ||
                !samePoint(candidate.virtualPocketTarget, pocket.virtualPocketTarget) ||
                !expected || !sameKickCandidate(candidate, *expected)) {
                return PotSelectionResult::rejected(PotSelectionStatus::InvalidCandidateInput);
            }
            double totalDistance = 0.0;
            if (!accumulateDistance(totalDistance, candidate.cuePathFirst) ||
                !accumulateDistance(totalDistance, candidate.cuePathSecond) ||
                !accumulateDistance(totalDistance, candidate.targetPath)) {
                return PotSelectionResult::rejected(PotSelectionStatus::NumericalFailure);
            }
            std::optional<double> minimumClearance;
            if (!accumulateClearance(
                    minimumClearance,
                    candidate.cuePathFirst,
                    obstacles,
                    {}) ||
                !accumulateClearance(
                    minimumClearance,
                    candidate.cuePathSecond,
                    obstacles,
                    selectedTargetExclusion) ||
                !accumulateClearance(
                    minimumClearance,
                    candidate.targetPath,
                    obstacles,
                    selectedTargetExclusion)) {
                return PotSelectionResult::rejected(PotSelectionStatus::NumericalFailure);
            }
            const auto audit = makeAudit(
                ScoringRawCosts{
                    1.0,
                    candidate.cuttingAngleDeg,
                    totalDistance,
                    minimumClearance,
                    candidate.pocketEntryAngleDeg,
                    candidate.incidenceAngleDeg},
                pocket.maxEntryAngleDeg,
                kickConfig->maxKickRailAngleDeg);
            if (!audit) {
                return PotSelectionResult::rejected(PotSelectionStatus::NumericalFailure);
            }
            scored.push_back({
                PotCandidateKind::Kick,
                selectedTarget,
                candidate.pocketId,
                candidate.railId,
                PotCandidateValue{candidate},
                *audit});
        }
    }

    if (scored.empty()) {
        if (scoringConfig->planningMode == BilliardConfig::PlanningMode::ManualResearch) {
            return PotSelectionResult::rejected(
                PotSelectionStatus::ManualResearchPotSearchExhausted);
        }
        PotOnlyNoPlan noPlan{
            PotNoPlanReason::NoPotCandidate,
            0,
            false,
            {},
            {}};
        for (const auto& diagnostic : directCandidates.rejected) {
            if (diagnostic) noPlan.directDiagnostics.push_back(*diagnostic);
        }
        for (const auto& pocket : kickCandidates.rejected) {
            for (const auto& diagnostic : pocket) {
                if (diagnostic) noPlan.kickDiagnostics.push_back(*diagnostic);
            }
        }
        return PotSelectionResult::success(PotSelectionOutcome{std::move(noPlan)});
    }

    double minimumTotalCost = scored.front().audit.totalCost;
    for (const ScoredPotCandidate& candidate : scored) {
        minimumTotalCost = std::min(minimumTotalCost, candidate.audit.totalCost);
    }
    const double tieLimit = minimumTotalCost + scoringConfig->tieEpsilon;
    if (!std::isfinite(tieLimit)) {
        return PotSelectionResult::rejected(PotSelectionStatus::NumericalFailure);
    }
    const ScoredPotCandidate* winner = nullptr;
    for (const ScoredPotCandidate& candidate : scored) {
        if (candidate.audit.totalCost > tieLimit) {
            continue;
        }
        if (!winner || tieBreakBetter(candidate, *winner)) {
            winner = &candidate;
        }
    }
    if (!winner) {
        return PotSelectionResult::rejected(PotSelectionStatus::NumericalFailure);
    }
    return PotSelectionResult::success(PotSelectionOutcome{*winner});
}

PlanningResult BilliardAlgorithm::planShot(
    const StableTableState& table,
    const std::optional<BilliardConfig::TableGeometryConfig>& geometryConfig,
    const BilliardConfig::BrainConfig& brainConfig)
{
    const auto noPlan = [](
        NoPlanReason reason,
        std::optional<PlanningSourceAudit> source,
        std::optional<EligibleTarget> selectedTarget,
        PlanningDiagnostic diagnostic,
        std::vector<DirectPotCandidateDiagnostic> directDiagnostics = {},
        std::vector<KickPotCandidateDiagnostic> kickDiagnostics = {}) {
        return PlanningResult::noPlan(NoPlan{
            reason,
            std::move(source),
            std::move(selectedTarget),
            0,
            false,
            std::move(directDiagnostics),
            std::move(kickDiagnostics),
            std::move(diagnostic)});
    };

    if (!brainConfig.base0PlanarCalibrationRevision ||
        brainConfig.base0PlanarCalibrationRevision->empty() ||
        !brainConfig.kickGeometry || !brainConfig.scoring ||
        brainConfig.scoring->planningMode != BilliardConfig::PlanningMode::PotOnly) {
        return noPlan(
            NoPlanReason::InvalidBrainConfiguration,
            std::nullopt,
            std::nullopt,
            {});
    }

    const auto geometryResult = BilliardPhysics::resolveTableGeometry(
        table.pockets,
        geometryConfig);
    if (!geometryResult.isValid() || !geometryResult.value()) {
        const GeometryStatus status = geometryResult.isValid()
            ? geometryResult.status()
            : GeometryStatus::InvalidConfiguration;
        return noPlan(
            NoPlanReason::InvalidBrainConfiguration,
            std::nullopt,
            std::nullopt,
            PlanningDiagnostic{
                std::nullopt,
                status,
                std::nullopt,
                std::nullopt,
                std::nullopt});
    }
    const ResolvedTableGeometry& geometry = *geometryResult.value();
    PlanningSourceAudit source{
        {table.connectionIdentity, table.shotCycleIdentity},
        table.sourceEvents,
        *brainConfig.base0PlanarCalibrationRevision,
        geometry.calibrationRevision,
        table.cueBall,
        geometry.ballRadiusMm};
    if (!source.isValid()) {
        return noPlan(
            NoPlanReason::NumericalPlanningFailure,
            std::nullopt,
            std::nullopt,
            PlanningDiagnostic{
                TargetQualificationStatus::InvalidStableState,
                std::nullopt,
                std::nullopt,
                std::nullopt,
                std::nullopt});
    }

    const TargetQualificationResult targetResult = TargetSelector{}.select(table);
    if (!targetResult.isValid() || !targetResult.value()) {
        const TargetQualificationStatus status = targetResult.isValid()
            ? targetResult.status()
            : TargetQualificationStatus::InvalidStableState;
        return noPlan(
            status == TargetQualificationStatus::NoEligibleTarget
                ? NoPlanReason::NoEligibleTarget
                : NoPlanReason::NumericalPlanningFailure,
            source,
            std::nullopt,
            PlanningDiagnostic{
                status,
                std::nullopt,
                std::nullopt,
                std::nullopt,
                std::nullopt});
    }
    const EligibleTarget selectedTarget = *targetResult.value();

    const DirectPotGenerationResult directResult = generateDirectPotCandidates(
        table,
        selectedTarget,
        geometry);
    if (!directResult.isValid() || !directResult.value()) {
        const DirectPotGenerationStatus status = directResult.isValid()
            ? directResult.status()
            : DirectPotGenerationStatus::InvalidStableState;
        return noPlan(
            status == DirectPotGenerationStatus::InvalidGeometryConfiguration
                ? NoPlanReason::InvalidBrainConfiguration
                : NoPlanReason::NumericalPlanningFailure,
            source,
            selectedTarget,
            PlanningDiagnostic{
                std::nullopt,
                std::nullopt,
                status,
                std::nullopt,
                std::nullopt});
    }
    const KickPotGenerationResult kickResult = generateKickPotCandidates(
        table,
        selectedTarget,
        geometry,
        brainConfig.kickGeometry);
    if (!kickResult.isValid() || !kickResult.value()) {
        const KickPotGenerationStatus status = kickResult.isValid()
            ? kickResult.status()
            : KickPotGenerationStatus::InvalidStableState;
        const bool configurationFailure =
            status == KickPotGenerationStatus::ConfigurationMissing ||
            status == KickPotGenerationStatus::InvalidGeometryConfiguration;
        return noPlan(
            configurationFailure
                ? NoPlanReason::InvalidBrainConfiguration
                : NoPlanReason::NumericalPlanningFailure,
            source,
            selectedTarget,
            PlanningDiagnostic{
                std::nullopt,
                std::nullopt,
                std::nullopt,
                status,
                std::nullopt});
    }

    const PotSelectionResult selection = selectBestPot(
        table,
        selectedTarget,
        geometry,
        *directResult.value(),
        *kickResult.value(),
        brainConfig.scoring,
        brainConfig.kickGeometry);
    if (!selection.isValid() || !selection.value()) {
        const PotSelectionStatus status = selection.isValid()
            ? selection.status()
            : PotSelectionStatus::InvalidCandidateInput;
        const bool configurationFailure =
            status == PotSelectionStatus::ConfigurationMissing ||
            status == PotSelectionStatus::InvalidConfiguration;
        return noPlan(
            configurationFailure
                ? NoPlanReason::InvalidBrainConfiguration
                : NoPlanReason::NumericalPlanningFailure,
            source,
            selectedTarget,
            PlanningDiagnostic{
                std::nullopt,
                std::nullopt,
                std::nullopt,
                std::nullopt,
                status});
    }

    std::vector<DirectPotCandidateDiagnostic> directDiagnostics;
    for (const auto& diagnostic : directResult.value()->rejected) {
        if (diagnostic) directDiagnostics.push_back(*diagnostic);
    }
    std::vector<KickPotCandidateDiagnostic> kickDiagnostics;
    for (const auto& pocketDiagnostics : kickResult.value()->rejected) {
        for (const auto& diagnostic : pocketDiagnostics) {
            if (diagnostic) kickDiagnostics.push_back(*diagnostic);
        }
    }

    if (const auto* empty = std::get_if<PotOnlyNoPlan>(&*selection.value())) {
        return noPlan(
            NoPlanReason::NoPotCandidate,
            source,
            selectedTarget,
            PlanningDiagnostic{
                std::nullopt,
                std::nullopt,
                std::nullopt,
                std::nullopt,
                PotSelectionStatus::Success},
            empty->directDiagnostics,
            empty->kickDiagnostics);
    }

    const auto* winner = std::get_if<ScoredPotCandidate>(&*selection.value());
    if (!winner || !winner->isValid()) {
        return noPlan(
            NoPlanReason::NumericalPlanningFailure,
            source,
            selectedTarget,
            PlanningDiagnostic{
                std::nullopt,
                std::nullopt,
                std::nullopt,
                std::nullopt,
                PotSelectionStatus::NumericalFailure});
    }

    ShotPlan plan{
        winner->kind == PotCandidateKind::Direct
            ? ShotPlanType::DirectPot
            : ShotPlanType::KickPot,
        source,
        selectedTarget,
        {},
        {},
        {},
        winner->audit.rawCosts.minimumClearanceMm,
        FixedForceMode::Fixed,
        {true, true, true},
        std::move(directDiagnostics),
        std::move(kickDiagnostics),
        DirectPotShotPlanPayload{}};

    Point initialPathEnd{};
    if (winner->kind == PotCandidateKind::Direct) {
        const auto* candidate = std::get_if<DirectPotCandidate>(&winner->candidate);
        if (!candidate) {
            return noPlan(
                NoPlanReason::NumericalPlanningFailure,
                source,
                selectedTarget,
                PlanningDiagnostic{
                    std::nullopt,
                    std::nullopt,
                    std::nullopt,
                    std::nullopt,
                    PotSelectionStatus::NumericalFailure});
        }
        initialPathEnd = candidate->cuePath.end;
        plan.ghostBallPoint = candidate->ghostBallPoint;
        plan.cuePathSegments = {candidate->cuePath};
        plan.payload = DirectPotShotPlanPayload{*candidate, winner->audit};
    } else {
        const auto* candidate = std::get_if<KickPotCandidate>(&winner->candidate);
        if (!candidate) {
            return noPlan(
                NoPlanReason::NumericalPlanningFailure,
                source,
                selectedTarget,
                PlanningDiagnostic{
                    std::nullopt,
                    std::nullopt,
                    std::nullopt,
                    std::nullopt,
                    PotSelectionStatus::NumericalFailure});
        }
        initialPathEnd = candidate->cuePathFirst.end;
        plan.ghostBallPoint = candidate->ghostBallPoint;
        plan.cuePathSegments = {candidate->cuePathFirst, candidate->cuePathSecond};
        plan.payload = KickPotShotPlanPayload{
            *candidate,
            winner->audit,
            *brainConfig.kickGeometry};
    }
    const auto initialVector = BilliardMath::getVector(table.cueBall, initialPathEnd);
    const auto direction = initialVector
        ? BilliardMath::normalize(*initialVector)
        : std::nullopt;
    if (!direction) {
        return noPlan(
            NoPlanReason::NumericalPlanningFailure,
            source,
            selectedTarget,
            PlanningDiagnostic{
                std::nullopt,
                std::nullopt,
                std::nullopt,
                std::nullopt,
                PotSelectionStatus::NumericalFailure});
    }
    plan.shotDirectionXY = *direction;
    if (!plan.isValid()) {
        return noPlan(
            NoPlanReason::NumericalPlanningFailure,
            source,
            selectedTarget,
            PlanningDiagnostic{
                std::nullopt,
                std::nullopt,
                std::nullopt,
                std::nullopt,
                PotSelectionStatus::NumericalFailure});
    }
    return PlanningResult::shotPlan(std::move(plan));
}

#ifdef BILLIARDS_P1_08_TEST_SEAM
bool BilliardAlgorithm::tieBreakBetterForTest(
    const ScoredPotCandidate& candidate,
    const ScoredPotCandidate& current) noexcept
{
    return tieBreakBetter(candidate, current);
}
#endif
