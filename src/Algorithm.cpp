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
