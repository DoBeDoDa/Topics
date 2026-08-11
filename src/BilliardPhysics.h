#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "BilliardConfig.h"
#include "GeometryResults.h"
#include "Point.h"

struct ResolvedPocket {
    BilliardConfig::PocketId id;

    // 當次Vision取得的Robot Base0 XY袋口點
    Point center;
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
    std::array<ResolvedPocket, 6> pockets;
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

struct MinimumClearance {
    // nullopt means no non-excluded stationary obstacle exists.
    std::optional<double> millimeters;
};

class BilliardPhysics {
public:
    static GeometryValueResult<ResolvedTableGeometry> resolveTableGeometry(
        const std::array<Point, 6>& currentCyclePocketCenters,
        const std::optional<BilliardConfig::TableGeometryConfig>& config);

    static GeometryValueResult<EffectiveCueBallRailSegment> deriveEffectiveRail(
        const BilliardConfig::PhysicalRailConfig& physicalRail,
        const Segment2D& segment,
        Vector2D inwardUnitNormal,
        double ballRadiusMm);

    static GhostBallResult computeGhostBallPoint(
        Point cueBallCenter,
        Point targetBallCenter,
        Point pocketTarget,
        double ballRadiusMm);

    static GeometryCheckResult checkTargetPathToPocket(
        Segment2D targetPath,
        Point pocketTarget);

    static GeometryCheckResult checkSegmentCollision(
        Segment2D path,
        const std::vector<Point>& obstacles,
        const std::vector<std::size_t>& excludedObstacleIndices,
        double ballDiameterMm,
        double collisionMarginMm);

    static GeometryValueResult<MinimumClearance> computeMinimumSegmentClearance(
        Segment2D path,
        const std::vector<Point>& obstacles,
        const std::vector<std::size_t>& excludedObstacleIndices);

    static GeometryValueResult<Point> mirrorPointAcrossEffectiveRail(
        Point point,
        const EffectiveCueBallRailSegment& rail);

    static GeometryValueResult<Point> intersectRayWithEffectiveRail(
        Point rayStart,
        Point rayThrough,
        const EffectiveCueBallRailSegment& rail);
};
