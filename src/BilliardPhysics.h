#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "BilliardConfig.h"
#include "GeometryResults.h"
#include "Point.h"

struct PocketBoundaryCut {
    BilliardConfig::PocketId pocketId;
    Point entranceCenter;
    Segment2D pocketExitSegment;
    Vector2D outwardUnitNormal;
};

struct PlayableBallCenterRegion {
    AxisAlignedBounds2D bounds;
    std::vector<PocketBoundaryCut> pocketBoundaries;
};

struct PocketCaptureCorridor {
    BilliardConfig::PocketId pocketId;
    Point entranceCenter;
    Point virtualPocketTarget;
    Vector2D outwardUnitNormal;
    double halfWidthMm;
};

struct ResolvedPocketModel {
    BilliardConfig::PocketId id;
    BilliardConfig::PocketType type;
    Point wirePocketCenter;
    Vector2D outwardUnitNormal;
    Point virtualPocketTarget;
    Segment2D pocketExitSegment;
    PocketCaptureCorridor captureCorridor;
    double exitCrossingEpsilon;
    double maxEntryAngleDeg;
    double pocketBoundaryProbeEpsilonMm;
};

struct PhysicalRailSegment {
    BilliardConfig::RailId id;
    Segment2D segment;
    Vector2D inwardUnitNormal;
    double startExclusionMm;
    double endExclusionMm;
};

struct EffectiveCueBallRailSegment {
    BilliardConfig::RailId physicalRailId;
    Segment2D segment;
    Vector2D inwardUnitNormal;
};

struct RailReflectionRegion {
    std::array<EffectiveCueBallRailSegment, 6> rails;
};

struct ResolvedTableGeometry {
    std::string calibrationRevision;
    PlayableBallCenterRegion playableBallCenterRegion;
    std::array<ResolvedPocketModel, 6> pockets;
    std::array<PhysicalRailSegment, 6> physicalRails;
    RailReflectionRegion railReflectionRegion;
    double ballRadiusMm;
    double ballDiameterMm;
    double collisionMarginMm;
};

struct GhostBallPoint {
    Point center;
};

struct GhostBallDiagnostic {
    GeometryStatus status;
    std::optional<Point> ballSurfaceContactPoint;
};

class GhostBallResult {
public:
    static GhostBallResult success(GhostBallPoint value, Point surfaceContactPoint);
    static GhostBallResult failure(GeometryStatus status);

    [[nodiscard]] GeometryStatus status() const noexcept;
    [[nodiscard]] const std::optional<GhostBallPoint>& value() const noexcept;
    [[nodiscard]] const GhostBallDiagnostic& diagnostic() const noexcept;
    [[nodiscard]] bool isValid() const noexcept;

private:
    GhostBallResult(
        GeometryStatus status,
        std::optional<GhostBallPoint> value,
        GhostBallDiagnostic diagnostic);

    GeometryStatus status_;
    std::optional<GhostBallPoint> value_;
    GhostBallDiagnostic diagnostic_;
};

struct PocketEntryAngle {
    double degrees;
};

class BilliardPhysics {
public:
    static GeometryValueResult<PlayableBallCenterRegion> derivePlayableBallCenterRegion(
        AxisAlignedBounds2D physicalPlayingSurface,
        double ballRadiusMm);

    static GeometryValueResult<ResolvedTableGeometry> resolveTableGeometry(
        const std::array<Point, 6>& currentCyclePocketCenters,
        const std::optional<BilliardConfig::TableGeometryConfig>& config);

    static GeometryValueResult<ResolvedPocketModel> resolvePocketModel(
        Point wirePocketCenter,
        const BilliardConfig::PocketModelConfig& config,
        const PlayableBallCenterRegion& playableRegion);

    static GeometryValueResult<EffectiveCueBallRailSegment> deriveEffectiveRail(
        const BilliardConfig::PhysicalRailConfig& physicalRail,
        const PlayableBallCenterRegion& playableRegion,
        double ballRadiusMm);

    static GhostBallResult computeGhostBallPoint(
        Point cueBallCenter,
        Point targetBallCenter,
        Point virtualPocketTarget,
        double ballRadiusMm);

    static GeometryValueResult<PocketEntryAngle> computePocketEntryAngle(
        Point targetBallCenter,
        const ResolvedPocketModel& pocket);

    static GeometryCheckResult checkPocketExitDirection(
        Vector2D unitMovementDirection,
        const ResolvedPocketModel& pocket);

    static GeometryCheckResult checkSegmentWithinPlayableRegion(
        Segment2D path,
        const PlayableBallCenterRegion& playableRegion);

    static GeometryCheckResult checkTargetPathToPocket(
        Segment2D targetPath,
        BilliardConfig::PocketId selectedPocket,
        const std::array<ResolvedPocketModel, 6>& pockets,
        const PlayableBallCenterRegion& playableRegion);

    static GeometryCheckResult checkSegmentCollision(
        Segment2D path,
        const std::vector<Point>& obstacles,
        const std::vector<std::size_t>& excludedObstacleIndices,
        double ballDiameterMm,
        double collisionMarginMm);

    static GeometryValueResult<Point> mirrorPointAcrossEffectiveRail(
        Point point,
        const EffectiveCueBallRailSegment& rail);

    static GeometryValueResult<Point> intersectRayWithEffectiveRail(
        Point rayStart,
        Point rayThrough,
        const EffectiveCueBallRailSegment& rail);

    static GeometryCheckResult checkEffectiveRailForReflection(
        const EffectiveCueBallRailSegment& rail,
        const PlayableBallCenterRegion& playableRegion);
};
