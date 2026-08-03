#include "Algorithm.h"

#include "MathUtils.h"

#include <cmath>
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
