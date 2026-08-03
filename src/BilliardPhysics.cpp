#include "BilliardPhysics.h"

#include "MathUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {
constexpr double NUMERICAL_EPSILON = 1e-9;

bool finite(double value) noexcept
{
    return std::isfinite(value);
}

bool finite(Segment2D segment) noexcept
{
    return BilliardMath::isFinite(segment.start) &&
        BilliardMath::isFinite(segment.end);
}

bool finite(AxisAlignedBounds2D bounds) noexcept
{
    return finite(bounds.minX) && finite(bounds.maxX) &&
        finite(bounds.minY) && finite(bounds.maxY);
}

bool validBounds(AxisAlignedBounds2D bounds) noexcept
{
    return finite(bounds) && bounds.minX < bounds.maxX && bounds.minY < bounds.maxY;
}

bool approximatelyEqual(double first, double second) noexcept
{
    return finite(first) && finite(second) &&
        std::fabs(first - second) <= NUMERICAL_EPSILON;
}

bool approximatelyEqual(Point first, Point second) noexcept
{
    return approximatelyEqual(first.x, second.x) &&
        approximatelyEqual(first.y, second.y);
}

bool samePoint(Point first, Point second) noexcept
{
    return first.x == second.x && first.y == second.y;
}

bool unitVector(Vector2D vector) noexcept
{
    const auto length = BilliardMath::getLength(vector);
    return length && std::fabs(*length - 1.0) <= NUMERICAL_EPSILON;
}

double cross(Vector2D first, Vector2D second) noexcept
{
    return first.x * second.y - first.y * second.x;
}

double dot(Vector2D first, Vector2D second) noexcept
{
    return first.x * second.x + first.y * second.y;
}

Point add(Point point, Vector2D vector, double scale) noexcept
{
    return {point.x + vector.x * scale, point.y + vector.y * scale};
}

bool containsInclusive(AxisAlignedBounds2D bounds, Point point) noexcept
{
    return BilliardMath::isFinite(point) &&
        point.x >= bounds.minX && point.x <= bounds.maxX &&
        point.y >= bounds.minY && point.y <= bounds.maxY;
}

bool containsInterior(AxisAlignedBounds2D bounds, Point point) noexcept
{
    return BilliardMath::isFinite(point) &&
        point.x > bounds.minX && point.x < bounds.maxX &&
        point.y > bounds.minY && point.y < bounds.maxY;
}

bool containsInclusive(
    const PlayableBallCenterRegion& region,
    Point point) noexcept
{
    if (!containsInclusive(region.bounds, point)) {
        return false;
    }
    for (const PocketBoundaryCut& boundary : region.pocketBoundaries) {
        const auto offset = BilliardMath::getVector(boundary.entranceCenter, point);
        if (!offset || !unitVector(boundary.outwardUnitNormal) ||
            dot(*offset, boundary.outwardUnitNormal) > NUMERICAL_EPSILON) {
            return false;
        }
    }
    return true;
}

bool containsInterior(
    const PlayableBallCenterRegion& region,
    Point point) noexcept
{
    if (!containsInterior(region.bounds, point)) {
        return false;
    }
    for (const PocketBoundaryCut& boundary : region.pocketBoundaries) {
        const auto offset = BilliardMath::getVector(boundary.entranceCenter, point);
        if (!offset || !unitVector(boundary.outwardUnitNormal) ||
            dot(*offset, boundary.outwardUnitNormal) >= -NUMERICAL_EPSILON) {
            return false;
        }
    }
    return true;
}

bool onBoundsBoundary(AxisAlignedBounds2D bounds, Point point) noexcept
{
    return containsInclusive(bounds, point) &&
        (approximatelyEqual(point.x, bounds.minX) ||
         approximatelyEqual(point.x, bounds.maxX) ||
         approximatelyEqual(point.y, bounds.minY) ||
         approximatelyEqual(point.y, bounds.maxY));
}

Segment2D translateSegment(Point origin, Segment2D offsets) noexcept
{
    return {
        {origin.x + offsets.start.x, origin.y + offsets.start.y},
        {origin.x + offsets.end.x, origin.y + offsets.end.y}};
}

Point snapToBounds(Point point, AxisAlignedBounds2D bounds) noexcept
{
    if (approximatelyEqual(point.x, bounds.minX)) point.x = bounds.minX;
    if (approximatelyEqual(point.x, bounds.maxX)) point.x = bounds.maxX;
    if (approximatelyEqual(point.y, bounds.minY)) point.y = bounds.minY;
    if (approximatelyEqual(point.y, bounds.maxY)) point.y = bounds.maxY;
    return point;
}

Segment2D translateAndSnapSegment(
    Point origin,
    Segment2D offsets,
    AxisAlignedBounds2D bounds) noexcept
{
    Segment2D result = translateSegment(origin, offsets);
    result.start = snapToBounds(result.start, bounds);
    result.end = snapToBounds(result.end, bounds);
    return result;
}

bool pointInRelativeInterior(Point point, Segment2D segment) noexcept
{
    const auto direction = BilliardMath::getVector(segment.start, segment.end);
    const auto toPoint = BilliardMath::getVector(segment.start, point);
    if (!direction || !toPoint) {
        return false;
    }
    const double lengthSquared = dot(*direction, *direction);
    if (!finite(lengthSquared) || lengthSquared <= NUMERICAL_EPSILON) {
        return false;
    }
    const double parameter = dot(*toPoint, *direction) / lengthSquared;
    const double distanceFromLine = std::fabs(cross(*toPoint, *direction)) /
        std::sqrt(lengthSquared);
    return finite(parameter) && finite(distanceFromLine) &&
        parameter > NUMERICAL_EPSILON && parameter < 1.0 - NUMERICAL_EPSILON &&
        distanceFromLine <= NUMERICAL_EPSILON;
}

bool pointInCorridor(Point point, const PocketCaptureCorridor& corridor) noexcept
{
    if (!BilliardMath::isFinite(point) ||
        !BilliardMath::isFinite(corridor.entranceCenter) ||
        !BilliardMath::isFinite(corridor.virtualPocketTarget) ||
        !unitVector(corridor.outwardUnitNormal) ||
        !finite(corridor.halfWidthMm) || corridor.halfWidthMm < 0.0) {
        return false;
    }
    const auto offset = BilliardMath::getVector(corridor.entranceCenter, point);
    const auto length = BilliardMath::getDistance(
        corridor.entranceCenter,
        corridor.virtualPocketTarget);
    if (!offset || !length || *length <= NUMERICAL_EPSILON) {
        return approximatelyEqual(point, corridor.entranceCenter);
    }
    const Vector2D side{-corridor.outwardUnitNormal.y, corridor.outwardUnitNormal.x};
    const double longitudinal = dot(*offset, corridor.outwardUnitNormal);
    const double lateral = std::fabs(dot(*offset, side));
    return finite(longitudinal) && finite(lateral) &&
        longitudinal >= -NUMERICAL_EPSILON &&
        longitudinal <= *length + NUMERICAL_EPSILON &&
        lateral <= corridor.halfWidthMm + NUMERICAL_EPSILON;
}

struct SegmentIntersection {
    bool intersects;
    bool parallel;
    bool coincident;
    std::optional<Point> point;
    double firstParameter;
};

SegmentIntersection intersectSegments(Segment2D first, Segment2D second) noexcept
{
    const auto firstDirection = BilliardMath::getVector(first.start, first.end);
    const auto secondDirection = BilliardMath::getVector(second.start, second.end);
    const auto starts = BilliardMath::getVector(first.start, second.start);
    if (!firstDirection || !secondDirection || !starts) {
        return {false, false, false, std::nullopt, 0.0};
    }
    const double denominator = cross(*firstDirection, *secondDirection);
    if (!finite(denominator) || std::fabs(denominator) <= NUMERICAL_EPSILON) {
        const bool coincident = std::fabs(cross(*starts, *firstDirection)) <=
            NUMERICAL_EPSILON;
        return {false, !coincident, coincident, std::nullopt, 0.0};
    }
    const double firstParameter = cross(*starts, *secondDirection) / denominator;
    const double secondParameter = cross(*starts, *firstDirection) / denominator;
    if (!finite(firstParameter) || !finite(secondParameter) ||
        firstParameter < -NUMERICAL_EPSILON || firstParameter > 1.0 + NUMERICAL_EPSILON ||
        secondParameter < -NUMERICAL_EPSILON || secondParameter > 1.0 + NUMERICAL_EPSILON) {
        return {false, false, false, std::nullopt, firstParameter};
    }
    const Point point = add(first.start, *firstDirection, firstParameter);
    return {
        BilliardMath::isFinite(point),
        false,
        false,
        BilliardMath::isFinite(point) ? std::optional<Point>{point} : std::nullopt,
        firstParameter};
}

bool segmentsOverlapOrTouch(Segment2D first, Segment2D second) noexcept
{
    const auto intersection = intersectSegments(first, second);
    if (intersection.intersects) {
        return true;
    }
    if (!intersection.coincident) {
        return false;
    }
    const auto direction = BilliardMath::getVector(first.start, first.end);
    const auto length = direction ? BilliardMath::getLength(*direction) : std::nullopt;
    if (!direction || !length || *length <= NUMERICAL_EPSILON) {
        return true;
    }
    const Vector2D axis{direction->x / *length, direction->y / *length};
    const auto secondStart = BilliardMath::getVector(first.start, second.start);
    const auto secondEnd = BilliardMath::getVector(first.start, second.end);
    if (!secondStart || !secondEnd) {
        return true;
    }
    const double secondMin = std::min(dot(*secondStart, axis), dot(*secondEnd, axis));
    const double secondMax = std::max(dot(*secondStart, axis), dot(*secondEnd, axis));
    return secondMax >= -NUMERICAL_EPSILON && secondMin <= *length + NUMERICAL_EPSILON;
}

std::optional<double> firstRegionExitParameter(
    Segment2D segment,
    const PlayableBallCenterRegion& region) noexcept
{
    const auto direction = BilliardMath::getVector(segment.start, segment.end);
    if (!direction || !containsInclusive(region, segment.start)) {
        return std::nullopt;
    }
    double result = std::numeric_limits<double>::infinity();
    if (direction->x > NUMERICAL_EPSILON) {
        result = std::min(result, (region.bounds.maxX - segment.start.x) / direction->x);
    } else if (direction->x < -NUMERICAL_EPSILON) {
        result = std::min(result, (region.bounds.minX - segment.start.x) / direction->x);
    }
    if (direction->y > NUMERICAL_EPSILON) {
        result = std::min(result, (region.bounds.maxY - segment.start.y) / direction->y);
    } else if (direction->y < -NUMERICAL_EPSILON) {
        result = std::min(result, (region.bounds.minY - segment.start.y) / direction->y);
    }
    for (const PocketBoundaryCut& boundary : region.pocketBoundaries) {
        const auto startOffset = BilliardMath::getVector(
            boundary.entranceCenter,
            segment.start);
        if (!startOffset) {
            return std::nullopt;
        }
        const double rate = dot(*direction, boundary.outwardUnitNormal);
        if (rate > NUMERICAL_EPSILON) {
            const double parameter = -dot(*startOffset, boundary.outwardUnitNormal) / rate;
            if (!finite(parameter)) {
                return std::nullopt;
            }
            result = std::min(result, parameter);
        }
    }
    return finite(result) ? std::optional<double>{result} : std::nullopt;
}

bool excluded(std::size_t index, const std::vector<std::size_t>& exclusions)
{
    return std::find(exclusions.begin(), exclusions.end(), index) != exclusions.end();
}

bool validResolvedPocketModel(
    const ResolvedPocketModel& pocket,
    std::size_t expectedIndex,
    const PlayableBallCenterRegion& region) noexcept
{
    const auto exitLength = finite(pocket.pocketExitSegment)
        ? BilliardMath::getDistance(
            pocket.pocketExitSegment.start,
            pocket.pocketExitSegment.end)
        : std::nullopt;
    if (static_cast<std::size_t>(pocket.id) != expectedIndex ||
        pocket.captureCorridor.pocketId != pocket.id ||
        !BilliardMath::isFinite(pocket.wirePocketCenter) ||
        !BilliardMath::isFinite(pocket.virtualPocketTarget) ||
        !unitVector(pocket.outwardUnitNormal) ||
        !exitLength || *exitLength <= NUMERICAL_EPSILON ||
        !pointInRelativeInterior(pocket.wirePocketCenter, pocket.pocketExitSegment) ||
        !onBoundsBoundary(region.bounds, pocket.pocketExitSegment.start) ||
        !onBoundsBoundary(region.bounds, pocket.pocketExitSegment.end) ||
        !containsInclusive(region, pocket.pocketExitSegment.start) ||
        !containsInclusive(region, pocket.pocketExitSegment.end) ||
        !approximatelyEqual(
            pocket.captureCorridor.entranceCenter,
            pocket.wirePocketCenter) ||
        !approximatelyEqual(
            pocket.captureCorridor.virtualPocketTarget,
            pocket.virtualPocketTarget) ||
        !approximatelyEqual(
            pocket.captureCorridor.outwardUnitNormal.x,
            pocket.outwardUnitNormal.x) ||
        !approximatelyEqual(
            pocket.captureCorridor.outwardUnitNormal.y,
            pocket.outwardUnitNormal.y) ||
        !finite(pocket.captureCorridor.halfWidthMm) ||
        pocket.captureCorridor.halfWidthMm <= 0.0 ||
        !finite(pocket.pocketBoundaryProbeEpsilonMm) ||
        pocket.pocketBoundaryProbeEpsilonMm <= 0.0 ||
        !finite(pocket.exitCrossingEpsilon) ||
        pocket.exitCrossingEpsilon < 0.0 || pocket.exitCrossingEpsilon >= 1.0 ||
        !finite(pocket.maxEntryAngleDeg) ||
        pocket.maxEntryAngleDeg < 0.0 || pocket.maxEntryAngleDeg > 180.0 ||
        !pointInCorridor(pocket.virtualPocketTarget, pocket.captureCorridor)) {
        return false;
    }
    const Point innerProbe = add(
        pocket.wirePocketCenter,
        pocket.outwardUnitNormal,
        -pocket.pocketBoundaryProbeEpsilonMm);
    const Point outerProbe = add(
        pocket.wirePocketCenter,
        pocket.outwardUnitNormal,
        pocket.pocketBoundaryProbeEpsilonMm);
    if (!containsInterior(region, innerProbe) ||
        containsInclusive(region, outerProbe) ||
        !pointInCorridor(outerProbe, pocket.captureCorridor)) {
        return false;
    }
    for (const PocketBoundaryCut& boundary : region.pocketBoundaries) {
        if (boundary.pocketId == pocket.id) {
            return approximatelyEqual(boundary.entranceCenter, pocket.wirePocketCenter) &&
                approximatelyEqual(
                    boundary.pocketExitSegment.start,
                    pocket.pocketExitSegment.start) &&
                approximatelyEqual(
                    boundary.pocketExitSegment.end,
                    pocket.pocketExitSegment.end) &&
                approximatelyEqual(
                    boundary.outwardUnitNormal.x,
                    pocket.outwardUnitNormal.x) &&
                approximatelyEqual(
                    boundary.outwardUnitNormal.y,
                    pocket.outwardUnitNormal.y);
        }
    }
    return false;
}
}

GhostBallResult::GhostBallResult(
    GeometryStatus status,
    std::optional<GhostBallPoint> value,
    GhostBallDiagnostic diagnostic)
    : status_(status), value_(std::move(value)), diagnostic_(std::move(diagnostic))
{
}

GhostBallResult GhostBallResult::success(
    GhostBallPoint value,
    Point surfaceContactPoint)
{
    return GhostBallResult(
        GeometryStatus::Success,
        std::optional<GhostBallPoint>{value},
        GhostBallDiagnostic{
            GeometryStatus::Success,
            std::optional<Point>{surfaceContactPoint}});
}

GhostBallResult GhostBallResult::failure(GeometryStatus status)
{
    return GhostBallResult(status, std::nullopt, GhostBallDiagnostic{status, std::nullopt});
}

GeometryStatus GhostBallResult::status() const noexcept { return status_; }
const std::optional<GhostBallPoint>& GhostBallResult::value() const noexcept { return value_; }
const GhostBallDiagnostic& GhostBallResult::diagnostic() const noexcept { return diagnostic_; }

bool GhostBallResult::isValid() const noexcept
{
    const bool succeeded = status_ == GeometryStatus::Success;
    return value_.has_value() == succeeded && diagnostic_.status == status_ &&
        diagnostic_.ballSurfaceContactPoint.has_value() == succeeded;
}

GeometryValueResult<PlayableBallCenterRegion>
BilliardPhysics::derivePlayableBallCenterRegion(
    AxisAlignedBounds2D physicalPlayingSurface,
    double ballRadiusMm)
{
    if (!validBounds(physicalPlayingSurface) || !finite(ballRadiusMm) || ballRadiusMm <= 0.0) {
        return GeometryValueResult<PlayableBallCenterRegion>::failure(
            GeometryStatus::InvalidConfiguration);
    }
    const AxisAlignedBounds2D playable{
        physicalPlayingSurface.minX + ballRadiusMm,
        physicalPlayingSurface.maxX - ballRadiusMm,
        physicalPlayingSurface.minY + ballRadiusMm,
        physicalPlayingSurface.maxY - ballRadiusMm};
    if (!validBounds(playable)) {
        return GeometryValueResult<PlayableBallCenterRegion>::failure(
            GeometryStatus::InvalidConfiguration);
    }
    return GeometryValueResult<PlayableBallCenterRegion>::success({playable, {}});
}

GeometryValueResult<ResolvedPocketModel> BilliardPhysics::resolvePocketModel(
    Point wirePocketCenter,
    const BilliardConfig::PocketModelConfig& config,
    const PlayableBallCenterRegion& playableRegion)
{
    const Segment2D pocketExitSegment = translateAndSnapSegment(
        wirePocketCenter,
        config.pocketExitSegmentOffsetsFromEntrance,
        playableRegion.bounds);
    if (!BilliardMath::isFinite(wirePocketCenter) ||
        !validBounds(playableRegion.bounds) ||
        !unitVector(config.outwardUnitNormal) ||
        !finite(config.pocketExitSegmentOffsetsFromEntrance) ||
        !finite(pocketExitSegment) ||
        !finite(config.virtualTargetOffsetMm) || config.virtualTargetOffsetMm <= 0.0 ||
        !finite(config.corridorHalfWidthMm) || config.corridorHalfWidthMm <= 0.0 ||
        !finite(config.pocketBoundaryProbeEpsilonMm) ||
        config.pocketBoundaryProbeEpsilonMm <= 0.0 ||
        config.pocketBoundaryProbeEpsilonMm >= config.virtualTargetOffsetMm ||
        !finite(config.exitCrossingEpsilon) || config.exitCrossingEpsilon < 0.0 ||
        config.exitCrossingEpsilon >= 1.0 ||
        !finite(config.maxEntryAngleDeg) || config.maxEntryAngleDeg < 0.0 ||
        config.maxEntryAngleDeg > 180.0 ||
        !pointInRelativeInterior(wirePocketCenter, pocketExitSegment) ||
        !onBoundsBoundary(playableRegion.bounds, pocketExitSegment.start) ||
        !onBoundsBoundary(playableRegion.bounds, pocketExitSegment.end) ||
        !containsInclusive(playableRegion, pocketExitSegment.start) ||
        !containsInclusive(playableRegion, pocketExitSegment.end)) {
        return GeometryValueResult<ResolvedPocketModel>::failure(
            GeometryStatus::InvalidConfiguration);
    }

    const auto exitDirection = BilliardMath::getVector(
        pocketExitSegment.start,
        pocketExitSegment.end);
    const auto normalizedExit = exitDirection
        ? BilliardMath::normalize(*exitDirection)
        : std::nullopt;
    if (!normalizedExit ||
        std::fabs(dot(*normalizedExit, config.outwardUnitNormal)) > NUMERICAL_EPSILON) {
        return GeometryValueResult<ResolvedPocketModel>::failure(
            GeometryStatus::InvalidConfiguration);
    }

    const Point virtualTarget = add(
        wirePocketCenter,
        config.outwardUnitNormal,
        config.virtualTargetOffsetMm);
    const PocketCaptureCorridor corridor{
        config.id,
        wirePocketCenter,
        virtualTarget,
        config.outwardUnitNormal,
        config.corridorHalfWidthMm};
    const Point innerProbe = add(
        wirePocketCenter,
        config.outwardUnitNormal,
        -config.pocketBoundaryProbeEpsilonMm);
    const Point outerProbe = add(
        wirePocketCenter,
        config.outwardUnitNormal,
        config.pocketBoundaryProbeEpsilonMm);
    if (!BilliardMath::isFinite(virtualTarget) ||
        !containsInterior(playableRegion, innerProbe) ||
        containsInclusive(playableRegion, outerProbe) ||
        !pointInCorridor(outerProbe, corridor) ||
        !pointInCorridor(virtualTarget, corridor)) {
        return GeometryValueResult<ResolvedPocketModel>::failure(
            GeometryStatus::InvalidConfiguration);
    }

    return GeometryValueResult<ResolvedPocketModel>::success({
        config.id,
        config.type,
        wirePocketCenter,
        config.outwardUnitNormal,
        virtualTarget,
        pocketExitSegment,
        corridor,
        config.exitCrossingEpsilon,
        config.maxEntryAngleDeg,
        config.pocketBoundaryProbeEpsilonMm});
}

GeometryValueResult<EffectiveCueBallRailSegment> BilliardPhysics::deriveEffectiveRail(
    const BilliardConfig::PhysicalRailConfig& physicalRail,
    const PlayableBallCenterRegion& playableRegion,
    double ballRadiusMm)
{
    const auto direction = BilliardMath::getVector(
        physicalRail.segment.start,
        physicalRail.segment.end);
    const auto length = direction ? BilliardMath::getLength(*direction) : std::nullopt;
    const auto tangent = direction ? BilliardMath::normalize(*direction) : std::nullopt;
    if (!finite(physicalRail.segment) || !validBounds(playableRegion.bounds) ||
        !unitVector(physicalRail.inwardUnitNormal) || !length || !tangent ||
        *length <= NUMERICAL_EPSILON || !finite(ballRadiusMm) || ballRadiusMm <= 0.0 ||
        !finite(physicalRail.startExclusionMm) || physicalRail.startExclusionMm < 0.0 ||
        !finite(physicalRail.endExclusionMm) || physicalRail.endExclusionMm < 0.0 ||
        physicalRail.startExclusionMm + physicalRail.endExclusionMm >= *length) {
        return GeometryValueResult<EffectiveCueBallRailSegment>::failure(
            GeometryStatus::InvalidConfiguration);
    }

    const Point offsetStart = add(
        physicalRail.segment.start,
        physicalRail.inwardUnitNormal,
        ballRadiusMm);
    const Point offsetEnd = add(
        physicalRail.segment.end,
        physicalRail.inwardUnitNormal,
        ballRadiusMm);
    const Point effectiveStart = add(offsetStart, *tangent, physicalRail.startExclusionMm);
    const Point effectiveEnd = add(offsetEnd, *tangent, -physicalRail.endExclusionMm);
    const Segment2D effective{effectiveStart, effectiveEnd};
    const auto effectiveLength = BilliardMath::getDistance(effectiveStart, effectiveEnd);
    const Point physicalMidpoint{
        (physicalRail.segment.start.x + physicalRail.segment.end.x) / 2.0,
        (physicalRail.segment.start.y + physicalRail.segment.end.y) / 2.0};
    const Point effectiveMidpoint = add(
        physicalMidpoint,
        physicalRail.inwardUnitNormal,
        ballRadiusMm);
    const double directionProbeMm = ballRadiusMm / 4.0;
    const Point inwardProbe = add(
        effectiveMidpoint,
        physicalRail.inwardUnitNormal,
        directionProbeMm);
    const Point outwardProbe = add(
        effectiveMidpoint,
        physicalRail.inwardUnitNormal,
        -directionProbeMm);
    if (!finite(effective) || !effectiveLength || *effectiveLength <= NUMERICAL_EPSILON ||
        !containsInclusive(playableRegion, effectiveStart) ||
        !containsInclusive(playableRegion, effectiveEnd) ||
        !containsInclusive(playableRegion, effectiveMidpoint) ||
        !containsInterior(playableRegion, inwardProbe) ||
        containsInterior(playableRegion, outwardProbe)) {
        return GeometryValueResult<EffectiveCueBallRailSegment>::failure(
            GeometryStatus::InvalidConfiguration);
    }

    return GeometryValueResult<EffectiveCueBallRailSegment>::success({
        physicalRail.id,
        effective,
        physicalRail.inwardUnitNormal});
}

GeometryValueResult<ResolvedTableGeometry> BilliardPhysics::resolveTableGeometry(
    const std::array<Point, 6>& currentCyclePocketCenters,
    const std::optional<BilliardConfig::TableGeometryConfig>& config)
{
    if (!config) {
        return GeometryValueResult<ResolvedTableGeometry>::failure(
            GeometryStatus::ConfigurationMissing);
    }
    if (config->calibrationRevision.empty() ||
        !finite(config->ballRadiusMm) || config->ballRadiusMm <= 0.0 ||
        !finite(config->ballDiameterMm) || config->ballDiameterMm <= 0.0 ||
        !approximatelyEqual(config->ballDiameterMm, 2.0 * config->ballRadiusMm) ||
        !finite(config->collisionMarginMm) || config->collisionMarginMm < 0.0) {
        return GeometryValueResult<ResolvedTableGeometry>::failure(
            GeometryStatus::InvalidConfiguration);
    }
    const auto playable = derivePlayableBallCenterRegion(
        config->physicalPlayingSurface,
        config->ballRadiusMm);
    if (!playable.value()) {
        return GeometryValueResult<ResolvedTableGeometry>::failure(playable.status());
    }

    PlayableBallCenterRegion resolvedPlayable = *playable.value();
    resolvedPlayable.pocketBoundaries.reserve(6);
    for (std::size_t index = 0; index < currentCyclePocketCenters.size(); ++index) {
        if (static_cast<std::size_t>(config->pockets[index].id) != index ||
            !BilliardMath::isFinite(currentCyclePocketCenters[index]) ||
            !unitVector(config->pockets[index].outwardUnitNormal) ||
            !finite(config->pockets[index].pocketExitSegmentOffsetsFromEntrance)) {
            return GeometryValueResult<ResolvedTableGeometry>::failure(
                GeometryStatus::InvalidConfiguration,
                index);
        }
        resolvedPlayable.pocketBoundaries.push_back({
            config->pockets[index].id,
            currentCyclePocketCenters[index],
            translateAndSnapSegment(
                currentCyclePocketCenters[index],
                config->pockets[index].pocketExitSegmentOffsetsFromEntrance,
                resolvedPlayable.bounds),
            config->pockets[index].outwardUnitNormal});
    }

    std::array<ResolvedPocketModel, 6> pockets{};
    std::array<PhysicalRailSegment, 6> physicalRails{};
    std::array<EffectiveCueBallRailSegment, 6> effectiveRails{};
    for (std::size_t index = 0; index < pockets.size(); ++index) {
        if (static_cast<std::size_t>(config->pockets[index].id) != index ||
            static_cast<std::size_t>(config->rails[index].id) != index ||
            !BilliardMath::isFinite(currentCyclePocketCenters[index])) {
            return GeometryValueResult<ResolvedTableGeometry>::failure(
                GeometryStatus::InvalidConfiguration,
                index);
        }
        const auto pocket = resolvePocketModel(
            currentCyclePocketCenters[index],
            config->pockets[index],
            resolvedPlayable);
        if (!pocket.value()) {
            return GeometryValueResult<ResolvedTableGeometry>::failure(
                pocket.status(),
                index);
        }
        pockets[index] = *pocket.value();

        physicalRails[index] = {
            config->rails[index].id,
            config->rails[index].segment,
            config->rails[index].inwardUnitNormal,
            config->rails[index].startExclusionMm,
            config->rails[index].endExclusionMm};
        const auto effective = deriveEffectiveRail(
            config->rails[index],
            resolvedPlayable,
            config->ballRadiusMm);
        if (!effective.value()) {
            return GeometryValueResult<ResolvedTableGeometry>::failure(
                effective.status(),
                index);
        }
        effectiveRails[index] = *effective.value();
    }

    for (std::size_t first = 0; first < pockets.size(); ++first) {
        for (std::size_t second = first + 1; second < pockets.size(); ++second) {
            if (segmentsOverlapOrTouch(
                    pockets[first].pocketExitSegment,
                    pockets[second].pocketExitSegment)) {
                return GeometryValueResult<ResolvedTableGeometry>::failure(
                    GeometryStatus::InvalidConfiguration,
                    second);
            }
        }
        for (std::size_t rail = 0; rail < effectiveRails.size(); ++rail) {
            if (segmentsOverlapOrTouch(
                    pockets[first].pocketExitSegment,
                    effectiveRails[rail].segment)) {
                return GeometryValueResult<ResolvedTableGeometry>::failure(
                    GeometryStatus::InvalidConfiguration,
                    first);
            }
        }
    }

    return GeometryValueResult<ResolvedTableGeometry>::success({
        config->calibrationRevision,
        resolvedPlayable,
        pockets,
        physicalRails,
        RailReflectionRegion{effectiveRails},
        config->ballRadiusMm,
        config->ballDiameterMm,
        config->collisionMarginMm});
}

GhostBallResult BilliardPhysics::computeGhostBallPoint(
    Point cueBallCenter,
    Point targetBallCenter,
    Point virtualPocketTarget,
    double ballRadiusMm)
{
    if (!BilliardMath::isFinite(cueBallCenter) ||
        !BilliardMath::isFinite(targetBallCenter) ||
        !BilliardMath::isFinite(virtualPocketTarget) ||
        !finite(ballRadiusMm) || ballRadiusMm <= 0.0) {
        return GhostBallResult::failure(GeometryStatus::InvalidInput);
    }
    const auto cueDistance = BilliardMath::getDistance(cueBallCenter, targetBallCenter);
    if (samePoint(cueBallCenter, targetBallCenter) ||
        (cueDistance && *cueDistance < 2.0 * ballRadiusMm)) {
        return GhostBallResult::failure(GeometryStatus::DegenerateGeometry);
    }
    if (!cueDistance) {
        return GhostBallResult::failure(GeometryStatus::InvalidInput);
    }
    if (samePoint(targetBallCenter, virtualPocketTarget)) {
        return GhostBallResult::failure(GeometryStatus::DegenerateGeometry);
    }
    const auto targetDirection = BilliardMath::getVector(
        targetBallCenter,
        virtualPocketTarget);
    const auto unit = targetDirection ? BilliardMath::normalize(*targetDirection) : std::nullopt;
    if (!unit) {
        return GhostBallResult::failure(GeometryStatus::InvalidInput);
    }
    const Point ghost = add(targetBallCenter, *unit, -2.0 * ballRadiusMm);
    const Point surfaceContact = add(targetBallCenter, *unit, -ballRadiusMm);
    if (!BilliardMath::isFinite(ghost) || !BilliardMath::isFinite(surfaceContact)) {
        return GhostBallResult::failure(GeometryStatus::InvalidInput);
    }
    return GhostBallResult::success({ghost}, surfaceContact);
}

GeometryValueResult<PocketEntryAngle> BilliardPhysics::computePocketEntryAngle(
    Point targetBallCenter,
    const ResolvedPocketModel& pocket)
{
    if (!BilliardMath::isFinite(targetBallCenter) ||
        !BilliardMath::isFinite(pocket.virtualPocketTarget) ||
        !unitVector(pocket.outwardUnitNormal) ||
        pocket.captureCorridor.pocketId != pocket.id ||
        !approximatelyEqual(
            pocket.captureCorridor.entranceCenter,
            pocket.wirePocketCenter) ||
        !approximatelyEqual(
            pocket.captureCorridor.virtualPocketTarget,
            pocket.virtualPocketTarget) ||
        !approximatelyEqual(
            pocket.captureCorridor.outwardUnitNormal.x,
            pocket.outwardUnitNormal.x) ||
        !approximatelyEqual(
            pocket.captureCorridor.outwardUnitNormal.y,
            pocket.outwardUnitNormal.y) ||
        !finite(pocket.captureCorridor.halfWidthMm) ||
        pocket.captureCorridor.halfWidthMm <= 0.0 ||
        !finite(pocket.maxEntryAngleDeg) || pocket.maxEntryAngleDeg < 0.0 ||
        pocket.maxEntryAngleDeg > 180.0) {
        return GeometryValueResult<PocketEntryAngle>::failure(
            GeometryStatus::InvalidConfiguration);
    }
    if (samePoint(targetBallCenter, pocket.virtualPocketTarget)) {
        return GeometryValueResult<PocketEntryAngle>::failure(
            GeometryStatus::DegenerateGeometry);
    }
    const auto direction = BilliardMath::getVector(
        targetBallCenter,
        pocket.virtualPocketTarget);
    if (!direction) {
        return GeometryValueResult<PocketEntryAngle>::failure(
            GeometryStatus::InvalidInput);
    }
    const auto unit = BilliardMath::normalize(*direction);
    if (!unit) {
        return GeometryValueResult<PocketEntryAngle>::failure(
            GeometryStatus::DegenerateGeometry);
    }
    const double cosine = std::clamp(dot(*unit, pocket.outwardUnitNormal), -1.0, 1.0);
    const double angle = std::acos(cosine) * 180.0 / BilliardMath::PI;
    if (!finite(angle)) {
        return GeometryValueResult<PocketEntryAngle>::failure(
            GeometryStatus::InvalidInput);
    }
    if (angle > pocket.maxEntryAngleDeg + NUMERICAL_EPSILON) {
        return GeometryValueResult<PocketEntryAngle>::failure(
            GeometryStatus::DirectionRejected);
    }
    return GeometryValueResult<PocketEntryAngle>::success({angle});
}

GeometryCheckResult BilliardPhysics::checkPocketExitDirection(
    Vector2D unitMovementDirection,
    const ResolvedPocketModel& pocket)
{
    if (!unitVector(unitMovementDirection)) {
        return GeometryCheckResult::rejected(GeometryStatus::InvalidInput);
    }
    if (!unitVector(pocket.outwardUnitNormal) ||
        !finite(pocket.exitCrossingEpsilon) ||
        pocket.exitCrossingEpsilon < 0.0 || pocket.exitCrossingEpsilon >= 1.0) {
        return GeometryCheckResult::rejected(GeometryStatus::InvalidConfiguration);
    }
    if (dot(unitMovementDirection, pocket.outwardUnitNormal) <=
        pocket.exitCrossingEpsilon) {
        return GeometryCheckResult::rejected(GeometryStatus::DirectionRejected);
    }
    return GeometryCheckResult::clear();
}

GeometryCheckResult BilliardPhysics::checkSegmentWithinPlayableRegion(
    Segment2D path,
    const PlayableBallCenterRegion& playableRegion)
{
    const auto length = finite(path)
        ? BilliardMath::getDistance(path.start, path.end)
        : std::nullopt;
    if (samePoint(path.start, path.end)) {
        return GeometryCheckResult::rejected(GeometryStatus::DegenerateGeometry);
    }
    if (!validBounds(playableRegion.bounds) || !length) {
        return GeometryCheckResult::rejected(GeometryStatus::InvalidInput);
    }
    if (!containsInclusive(playableRegion, path.start) ||
        !containsInclusive(playableRegion, path.end)) {
        return GeometryCheckResult::rejected(GeometryStatus::OutsideValidRegion);
    }
    return GeometryCheckResult::clear();
}

GeometryCheckResult BilliardPhysics::checkTargetPathToPocket(
    Segment2D targetPath,
    BilliardConfig::PocketId selectedPocket,
    const std::array<ResolvedPocketModel, 6>& pockets,
    const PlayableBallCenterRegion& playableRegion)
{
    const auto length = finite(targetPath)
        ? BilliardMath::getDistance(targetPath.start, targetPath.end)
        : std::nullopt;
    if (samePoint(targetPath.start, targetPath.end)) {
        return GeometryCheckResult::rejected(GeometryStatus::DegenerateGeometry);
    }
    if (!length || !validBounds(playableRegion.bounds) ||
        !containsInclusive(playableRegion, targetPath.start)) {
        return GeometryCheckResult::rejected(GeometryStatus::InvalidInput);
    }
    const std::size_t selectedIndex = static_cast<std::size_t>(selectedPocket);
    for (std::size_t index = 0; index < pockets.size(); ++index) {
        if (!validResolvedPocketModel(pockets[index], index, playableRegion)) {
            return GeometryCheckResult::rejected(
                GeometryStatus::InvalidConfiguration,
                index);
        }
        for (std::size_t other = index + 1; other < pockets.size(); ++other) {
            if (segmentsOverlapOrTouch(
                    pockets[index].pocketExitSegment,
                    pockets[other].pocketExitSegment)) {
                return GeometryCheckResult::rejected(
                    GeometryStatus::InvalidConfiguration,
                    other);
            }
        }
    }
    if (selectedIndex >= pockets.size() || pockets[selectedIndex].id != selectedPocket ||
        !approximatelyEqual(targetPath.end, pockets[selectedIndex].virtualPocketTarget)) {
        return GeometryCheckResult::rejected(GeometryStatus::InvalidConfiguration);
    }
    const ResolvedPocketModel& selected = pockets[selectedIndex];
    const auto direction = BilliardMath::getVector(targetPath.start, targetPath.end);
    const auto unit = direction ? BilliardMath::normalize(*direction) : std::nullopt;
    if (!unit) {
        return GeometryCheckResult::rejected(GeometryStatus::InvalidInput);
    }
    const auto exitDirection = checkPocketExitDirection(*unit, selected);
    if (exitDirection.status() != GeometryStatus::Clear) {
        return exitDirection;
    }

    const SegmentIntersection selectedIntersection = intersectSegments(
        targetPath,
        selected.pocketExitSegment);
    if (!selectedIntersection.intersects) {
        return GeometryCheckResult::rejected(GeometryStatus::NoIntersection);
    }
    for (std::size_t index = 0; index < pockets.size(); ++index) {
        if (index == selectedIndex) {
            continue;
        }
        const auto other = intersectSegments(targetPath, pockets[index].pocketExitSegment);
        if (other.intersects &&
            other.firstParameter <= selectedIntersection.firstParameter + NUMERICAL_EPSILON) {
            return GeometryCheckResult::rejected(GeometryStatus::OutsideValidRegion, index);
        }
    }

    const auto boundsExit = firstRegionExitParameter(targetPath, playableRegion);
    if (!boundsExit ||
        std::fabs(*boundsExit - selectedIntersection.firstParameter) > NUMERICAL_EPSILON ||
        !selectedIntersection.point ||
        !pointInCorridor(*selectedIntersection.point, selected.captureCorridor) ||
        !pointInCorridor(targetPath.end, selected.captureCorridor)) {
        return GeometryCheckResult::rejected(GeometryStatus::OutsideValidRegion);
    }
    return GeometryCheckResult::clear();
}

GeometryCheckResult BilliardPhysics::checkSegmentCollision(
    Segment2D path,
    const std::vector<Point>& obstacles,
    const std::vector<std::size_t>& excludedObstacleIndices,
    double ballDiameterMm,
    double collisionMarginMm)
{
    const auto direction = finite(path)
        ? BilliardMath::getVector(path.start, path.end)
        : std::nullopt;
    const auto length = direction ? BilliardMath::getLength(*direction) : std::nullopt;
    const double threshold = ballDiameterMm + collisionMarginMm;
    if (samePoint(path.start, path.end)) {
        return GeometryCheckResult::rejected(GeometryStatus::DegenerateGeometry);
    }
    if (!direction || !length ||
        !finite(ballDiameterMm) || ballDiameterMm <= 0.0 ||
        !finite(collisionMarginMm) || collisionMarginMm < 0.0 ||
        !finite(threshold)) {
        return GeometryCheckResult::rejected(GeometryStatus::InvalidInput);
    }
    for (const std::size_t index : excludedObstacleIndices) {
        if (index >= obstacles.size()) {
            return GeometryCheckResult::rejected(GeometryStatus::InvalidInput, index);
        }
    }
    for (std::size_t index = 0; index < obstacles.size(); ++index) {
        if (!BilliardMath::isFinite(obstacles[index])) {
            return GeometryCheckResult::rejected(GeometryStatus::InvalidInput, index);
        }
    }
    const double lengthSquared = (*length) * (*length);
    if (!finite(lengthSquared) || lengthSquared <= 0.0) {
        return GeometryCheckResult::rejected(GeometryStatus::InvalidInput);
    }
    for (std::size_t index = 0; index < obstacles.size(); ++index) {
        if (excluded(index, excludedObstacleIndices)) {
            continue;
        }
        const auto toObstacle = BilliardMath::getVector(path.start, obstacles[index]);
        if (!toObstacle) {
            return GeometryCheckResult::rejected(GeometryStatus::InvalidInput, index);
        }
        const double parameter = std::clamp(
            dot(*toObstacle, *direction) / lengthSquared,
            0.0,
            1.0);
        const Point nearest = add(path.start, *direction, parameter);
        const auto distance = BilliardMath::getDistance(nearest, obstacles[index]);
        if (!distance) {
            return GeometryCheckResult::rejected(GeometryStatus::InvalidInput, index);
        }
        if (*distance <= threshold) {
            return GeometryCheckResult::rejected(GeometryStatus::Blocked, index);
        }
    }
    return GeometryCheckResult::clear();
}

GeometryValueResult<Point> BilliardPhysics::mirrorPointAcrossEffectiveRail(
    Point point,
    const EffectiveCueBallRailSegment& rail)
{
    const auto direction = finite(rail.segment)
        ? BilliardMath::getVector(rail.segment.start, rail.segment.end)
        : std::nullopt;
    const auto length = direction ? BilliardMath::getLength(*direction) : std::nullopt;
    const auto offset = BilliardMath::isFinite(point)
        ? BilliardMath::getVector(rail.segment.start, point)
        : std::nullopt;
    if (!BilliardMath::isFinite(point)) {
        return GeometryValueResult<Point>::failure(GeometryStatus::InvalidInput);
    }
    if (!finite(rail.segment) || samePoint(rail.segment.start, rail.segment.end) ||
        !unitVector(rail.inwardUnitNormal)) {
        return GeometryValueResult<Point>::failure(GeometryStatus::InvalidConfiguration);
    }
    if (!direction || !length || !offset || *length <= NUMERICAL_EPSILON) {
        return GeometryValueResult<Point>::failure(GeometryStatus::InvalidInput);
    }
    const double signedDistance = dot(*offset, rail.inwardUnitNormal);
    const Point mirrored = add(point, rail.inwardUnitNormal, -2.0 * signedDistance);
    if (!BilliardMath::isFinite(mirrored)) {
        return GeometryValueResult<Point>::failure(GeometryStatus::InvalidInput);
    }
    return GeometryValueResult<Point>::success(mirrored);
}

GeometryValueResult<Point> BilliardPhysics::intersectRayWithEffectiveRail(
    Point rayStart,
    Point rayThrough,
    const EffectiveCueBallRailSegment& rail)
{
    const auto ray = BilliardMath::getVector(rayStart, rayThrough);
    const auto segment = BilliardMath::getVector(rail.segment.start, rail.segment.end);
    const auto starts = BilliardMath::getVector(rayStart, rail.segment.start);
    const auto rayLength = ray ? BilliardMath::getLength(*ray) : std::nullopt;
    const auto segmentLength = segment ? BilliardMath::getLength(*segment) : std::nullopt;
    if (!BilliardMath::isFinite(rayStart) || !BilliardMath::isFinite(rayThrough)) {
        return GeometryValueResult<Point>::failure(GeometryStatus::InvalidInput);
    }
    if (!finite(rail.segment) || samePoint(rail.segment.start, rail.segment.end) ||
        !unitVector(rail.inwardUnitNormal)) {
        return GeometryValueResult<Point>::failure(GeometryStatus::InvalidConfiguration);
    }
    if (samePoint(rayStart, rayThrough)) {
        return GeometryValueResult<Point>::failure(GeometryStatus::DegenerateGeometry);
    }
    if (!ray || !segment || !starts || !rayLength || !segmentLength ||
        *rayLength <= NUMERICAL_EPSILON || *segmentLength <= NUMERICAL_EPSILON) {
        return GeometryValueResult<Point>::failure(GeometryStatus::InvalidInput);
    }
    const double denominator = cross(*ray, *segment);
    if (!finite(denominator) || std::fabs(denominator) <= NUMERICAL_EPSILON) {
        const double alignment = cross(*starts, *ray);
        return GeometryValueResult<Point>::failure(
            finite(alignment) && std::fabs(alignment) <= NUMERICAL_EPSILON
                ? GeometryStatus::Coincident
                : GeometryStatus::Parallel);
    }
    const double rayParameter = cross(*starts, *segment) / denominator;
    const double segmentParameter = cross(*starts, *ray) / denominator;
    if (!finite(rayParameter) || !finite(segmentParameter)) {
        return GeometryValueResult<Point>::failure(GeometryStatus::InvalidInput);
    }
    if (rayParameter < -NUMERICAL_EPSILON ||
        segmentParameter < -NUMERICAL_EPSILON ||
        segmentParameter > 1.0 + NUMERICAL_EPSILON) {
        return GeometryValueResult<Point>::failure(GeometryStatus::NoIntersection);
    }
    const Point intersection = add(rayStart, *ray, rayParameter);
    if (!BilliardMath::isFinite(intersection)) {
        return GeometryValueResult<Point>::failure(GeometryStatus::InvalidInput);
    }
    return GeometryValueResult<Point>::success(intersection);
}
