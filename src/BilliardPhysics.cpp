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

std::optional<double> pointToSegmentDistance(
    Segment2D path,
    Point point) noexcept
{
    const auto direction = BilliardMath::getVector(path.start, path.end);
    const auto length = direction ? BilliardMath::getLength(*direction) : std::nullopt;
    const auto toPoint = BilliardMath::getVector(path.start, point);
    if (!finite(path) || !BilliardMath::isFinite(point) || !direction || !length ||
        !toPoint || *length <= NUMERICAL_EPSILON) {
        return std::nullopt;
    }
    const double lengthSquared = (*length) * (*length);
    if (!finite(lengthSquared) || lengthSquared <= 0.0) {
        return std::nullopt;
    }
    const double parameter = std::clamp(
        dot(*toPoint, *direction) / lengthSquared,
        0.0,
        1.0);
    const Point nearest = add(path.start, *direction, parameter);
    return BilliardMath::getDistance(nearest, point);
}

bool excluded(std::size_t index, const std::vector<std::size_t>& exclusions)
{
    return std::find(exclusions.begin(), exclusions.end(), index) != exclusions.end();
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

GeometryValueResult<EffectiveCueBallRailSegment> BilliardPhysics::deriveEffectiveRail(
    const BilliardConfig::PhysicalRailConfig& physicalRail,
    const Segment2D& segment,
    Vector2D inwardUnitNormal,
    double ballRadiusMm)
{
    const auto direction = BilliardMath::getVector(segment.start, segment.end);
    const auto length = direction ? BilliardMath::getLength(*direction) : std::nullopt;
    const auto tangent = direction ? BilliardMath::normalize(*direction) : std::nullopt;
    const double exclusionMm =
        physicalRail.startExclusionMm + physicalRail.endExclusionMm;
    if (!finite(segment) || !BilliardMath::isFinite(inwardUnitNormal) ||
        !unitVector(inwardUnitNormal) || !length || !tangent ||
        *length <= NUMERICAL_EPSILON || !finite(ballRadiusMm) || ballRadiusMm <= 0.0 ||
        !finite(physicalRail.cushionInsetMm) || physicalRail.cushionInsetMm < 0.0 ||
        !finite(physicalRail.startExclusionMm) || physicalRail.startExclusionMm < 0.0 ||
        !finite(physicalRail.endExclusionMm) || physicalRail.endExclusionMm < 0.0 ||
        !finite(exclusionMm) || exclusionMm >= *length ||
        !finite(dot(*tangent, inwardUnitNormal)) ||
        std::fabs(dot(*tangent, inwardUnitNormal)) > NUMERICAL_EPSILON) {
        return GeometryValueResult<EffectiveCueBallRailSegment>::failure(
            GeometryStatus::InvalidConfiguration);
    }

    const double normalInsetMm = ballRadiusMm + physicalRail.cushionInsetMm;
    const Point offsetStart = add(
        segment.start,
        inwardUnitNormal,
        normalInsetMm);
    const Point offsetEnd = add(
        segment.end,
        inwardUnitNormal,
        normalInsetMm);
    const Point effectiveStart = add(offsetStart, *tangent, physicalRail.startExclusionMm);
    const Point effectiveEnd = add(offsetEnd, *tangent, -physicalRail.endExclusionMm);
    const Segment2D effective{effectiveStart, effectiveEnd};
    const auto effectiveLength = BilliardMath::getDistance(effectiveStart, effectiveEnd);
    if (!finite(normalInsetMm) || !finite(effective) || !effectiveLength ||
        *effectiveLength <= NUMERICAL_EPSILON) {
        return GeometryValueResult<EffectiveCueBallRailSegment>::failure(
            GeometryStatus::InvalidConfiguration);
    }

    return GeometryValueResult<EffectiveCueBallRailSegment>::success({
        physicalRail.id,
        effective,
        inwardUnitNormal});
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
    std::array<ResolvedPocket, 6> pockets{};
    std::array<PhysicalRailSegment, 6> physicalRails{};
    std::array<EffectiveCueBallRailSegment, 6> effectiveRails{};
    for (std::size_t index = 0; index < pockets.size(); ++index) {
        if (!BilliardMath::isFinite(currentCyclePocketCenters[index])) {
            return GeometryValueResult<ResolvedTableGeometry>::failure(
                GeometryStatus::InvalidConfiguration,
                index);
        }
        pockets[index] = ResolvedPocket{
            static_cast<BilliardConfig::PocketId>(index),
            currentCyclePocketCenters[index]};
    }

    std::array<Segment2D, 6> rawSegments{};
    std::array<Vector2D, 6> tangents{};
    std::array<std::size_t, 6> startIndices{};
    std::array<std::size_t, 6> endIndices{};
    std::array<bool, 6> visitedStarts{};
    double signedArea2 = 0.0;
    for (std::size_t index = 0; index < config->rails.size(); ++index) {
        const auto& rail = config->rails[index];
        const std::size_t startIndex =
            static_cast<std::size_t>(rail.startPocket);
        const std::size_t endIndex =
            static_cast<std::size_t>(rail.endPocket);
        if (static_cast<std::size_t>(rail.id) != index ||
            startIndex >= currentCyclePocketCenters.size() ||
            endIndex >= currentCyclePocketCenters.size() ||
            startIndex == endIndex || visitedStarts[startIndex]) {
            return GeometryValueResult<ResolvedTableGeometry>::failure(
                GeometryStatus::InvalidConfiguration,
                index);
        }

        const Segment2D railSegment{
            currentCyclePocketCenters[startIndex],
            currentCyclePocketCenters[endIndex]};
        const auto direction = BilliardMath::getVector(
            railSegment.start,
            railSegment.end);
        const auto length = direction
            ? BilliardMath::getLength(*direction)
            : std::nullopt;
        const auto tangent = direction
            ? BilliardMath::normalize(*direction)
            : std::nullopt;
        const double areaTerm =
            railSegment.start.x * railSegment.end.y -
            railSegment.end.x * railSegment.start.y;
        if (!direction || !length || !tangent ||
            *length <= NUMERICAL_EPSILON || !finite(areaTerm) ||
            !finite(signedArea2 + areaTerm)) {
            return GeometryValueResult<ResolvedTableGeometry>::failure(
                GeometryStatus::InvalidConfiguration,
                index);
        }
        rawSegments[index] = railSegment;
        tangents[index] = *tangent;
        startIndices[index] = startIndex;
        endIndices[index] = endIndex;
        visitedStarts[startIndex] = true;
        signedArea2 += areaTerm;
    }

    // TableGeometryConfig::rails is a RailId-indexed ordered array: the
    // id==index contract fixes Rail1..Rail6 as the approved boundary traversal
    // order, so closed-chain adjacency is intentionally checked in array order.
    for (std::size_t index = 0; index < config->rails.size(); ++index) {
        const std::size_t nextIndex = (index + 1) % config->rails.size();
        if (endIndices[index] != startIndices[nextIndex]) {
            return GeometryValueResult<ResolvedTableGeometry>::failure(
                GeometryStatus::InvalidConfiguration,
                index);
        }
    }
    if (!finite(signedArea2) ||
        std::fabs(signedArea2) <= NUMERICAL_EPSILON) {
        return GeometryValueResult<ResolvedTableGeometry>::failure(
            GeometryStatus::InvalidConfiguration);
    }

    const bool counterClockwise = signedArea2 > 0.0;
    for (std::size_t index = 0; index < config->rails.size(); ++index) {
        const Vector2D tangent = tangents[index];
        const Vector2D inwardUnitNormal = counterClockwise
            ? Vector2D{-tangent.y, tangent.x}
            : Vector2D{tangent.y, -tangent.x};
        const double tangentNormalDot = dot(tangent, inwardUnitNormal);
        if (!BilliardMath::isFinite(tangent) ||
            !BilliardMath::isFinite(inwardUnitNormal) ||
            !unitVector(tangent) || !unitVector(inwardUnitNormal) ||
            !finite(tangentNormalDot) ||
            std::fabs(tangentNormalDot) > NUMERICAL_EPSILON) {
            return GeometryValueResult<ResolvedTableGeometry>::failure(
                GeometryStatus::InvalidConfiguration,
                index);
        }
        physicalRails[index] = {
            config->rails[index].id,
            rawSegments[index],
            inwardUnitNormal,
            config->rails[index].startExclusionMm,
            config->rails[index].endExclusionMm};
        const auto effective = deriveEffectiveRail(
            config->rails[index],
            rawSegments[index],
            inwardUnitNormal,
            config->ballRadiusMm);
        if (!effective.value()) {
            return GeometryValueResult<ResolvedTableGeometry>::failure(
                effective.status(),
                index);
        }
        effectiveRails[index] = *effective.value();
    }

    return GeometryValueResult<ResolvedTableGeometry>::success({
        config->calibrationRevision,
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
    Point pocketTarget,
    double ballRadiusMm)
{
    if (!BilliardMath::isFinite(cueBallCenter) ||
        !BilliardMath::isFinite(targetBallCenter) ||
        !BilliardMath::isFinite(pocketTarget) ||
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
    if (samePoint(targetBallCenter, pocketTarget)) {
        return GhostBallResult::failure(GeometryStatus::DegenerateGeometry);
    }
    const auto targetDirection = BilliardMath::getVector(
        targetBallCenter,
        pocketTarget);
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

GeometryCheckResult BilliardPhysics::checkTargetPathToPocket(
    Segment2D targetPath,
    Point pocketTarget)
{
    const auto length = finite(targetPath)
        ? BilliardMath::getDistance(targetPath.start, targetPath.end)
        : std::nullopt;
    if (samePoint(targetPath.start, targetPath.end)) {
        return GeometryCheckResult::rejected(GeometryStatus::DegenerateGeometry);
    }
    if (!length || !BilliardMath::isFinite(pocketTarget)) {
        return GeometryCheckResult::rejected(GeometryStatus::InvalidInput);
    }
    if (!approximatelyEqual(targetPath.end, pocketTarget)) {
        return GeometryCheckResult::rejected(GeometryStatus::InvalidConfiguration);
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
        const auto distance = pointToSegmentDistance(path, obstacles[index]);
        if (!distance) {
            return GeometryCheckResult::rejected(GeometryStatus::InvalidInput, index);
        }
        if (*distance <= threshold) {
            return GeometryCheckResult::rejected(GeometryStatus::Blocked, index);
        }
    }
    return GeometryCheckResult::clear();
}

GeometryValueResult<MinimumClearance> BilliardPhysics::computeMinimumSegmentClearance(
    Segment2D path,
    const std::vector<Point>& obstacles,
    const std::vector<std::size_t>& excludedObstacleIndices)
{
    if (!finite(path) || samePoint(path.start, path.end)) {
        return GeometryValueResult<MinimumClearance>::failure(
            samePoint(path.start, path.end)
                ? GeometryStatus::DegenerateGeometry
                : GeometryStatus::InvalidInput);
    }
    for (const std::size_t index : excludedObstacleIndices) {
        if (index >= obstacles.size()) {
            return GeometryValueResult<MinimumClearance>::failure(
                GeometryStatus::InvalidInput,
                index);
        }
    }
    std::optional<double> minimum;
    for (std::size_t index = 0; index < obstacles.size(); ++index) {
        if (excluded(index, excludedObstacleIndices)) {
            continue;
        }
        const auto distance = pointToSegmentDistance(path, obstacles[index]);
        if (!distance || !finite(*distance) || *distance < 0.0) {
            return GeometryValueResult<MinimumClearance>::failure(
                GeometryStatus::InvalidInput,
                index);
        }
        if (!minimum || *distance < *minimum) {
            minimum = *distance;
        }
    }
    return GeometryValueResult<MinimumClearance>::success({minimum});
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
    const Point intersection = add(
        rail.segment.start,
        *segment,
        std::clamp(segmentParameter, 0.0, 1.0));
    if (!BilliardMath::isFinite(intersection)) {
        return GeometryValueResult<Point>::failure(GeometryStatus::InvalidInput);
    }
    return GeometryValueResult<Point>::success(intersection);
}
