#include "TestHarness.h"

#include "../src/BilliardPhysics.h"
#include "../src/TableState.h"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace {
constexpr double TOLERANCE = 1e-9;
constexpr double ROOT_HALF = 0.70710678118654752440;

BilliardConfig::PocketModelConfig pocketConfig(
    BilliardConfig::PocketId id,
    BilliardConfig::PocketType type,
    Vector2D outward)
{
    const Vector2D side{-outward.y, outward.x};
    const double exitHalfLength =
        type == BilliardConfig::PocketType::Corner ? 14.142135623730951 : 20.0;
    return {
        id,
        type,
        outward,
        30.0,
        {{-side.x * exitHalfLength, -side.y * exitHalfLength},
         {side.x * exitHalfLength, side.y * exitHalfLength}},
        15.0,
        2.0,
        0.01,
        45.0};
}

BilliardConfig::TableGeometryConfig validConfig()
{
    using namespace BilliardConfig;
    return {
        "table-geometry-test-v1",
        {0.0, 1000.0, 0.0, 500.0},
        10.0,
        20.0,
        2.0,
        {{
            pocketConfig(PocketId::Pocket1, PocketType::Corner, {-ROOT_HALF, -ROOT_HALF}),
            pocketConfig(PocketId::Pocket2, PocketType::Side, {0.0, -1.0}),
            pocketConfig(PocketId::Pocket3, PocketType::Corner, {ROOT_HALF, -ROOT_HALF}),
            pocketConfig(PocketId::Pocket4, PocketType::Corner, {-ROOT_HALF, ROOT_HALF}),
            pocketConfig(PocketId::Pocket5, PocketType::Side, {0.0, 1.0}),
            pocketConfig(PocketId::Pocket6, PocketType::Corner, {ROOT_HALF, ROOT_HALF})
        }},
        {{
            {RailId::Rail1, {{0.0, 0.0}, {500.0, 0.0}}, {0.0, 1.0}, 40.0, 40.0},
            {RailId::Rail2, {{500.0, 0.0}, {1000.0, 0.0}}, {0.0, 1.0}, 40.0, 40.0},
            {RailId::Rail3, {{0.0, 500.0}, {500.0, 500.0}}, {0.0, -1.0}, 40.0, 40.0},
            {RailId::Rail4, {{500.0, 500.0}, {1000.0, 500.0}}, {0.0, -1.0}, 40.0, 40.0},
            {RailId::Rail5, {{0.0, 0.0}, {0.0, 500.0}}, {1.0, 0.0}, 40.0, 40.0},
            {RailId::Rail6, {{1000.0, 0.0}, {1000.0, 500.0}}, {-1.0, 0.0}, 40.0, 40.0}
        }}};
}

StableTableState stableStateWithNoNumberedBalls()
{
    std::array<std::optional<Point>, 9> balls{};
    const std::array<Point, 6> pockets{{
        {20.0, 20.0}, {500.0, 10.0}, {980.0, 20.0},
        {20.0, 480.0}, {500.0, 490.0}, {980.0, 480.0}}};
    const auto now = std::chrono::steady_clock::now();
    return StableTableState{
        balls,
        {250.0, 250.0},
        pockets,
        1,
        1,
        {{{1, now}, {2, now}, {3, now}}}};
}
}

int main()
{
    TestHarness tests;
    const auto config = validConfig();
    const auto stable = stableStateWithNoNumberedBalls();

    const auto missing = BilliardPhysics::resolveTableGeometry(stable.pockets, std::nullopt);
    tests.expectTrue(
        missing.status() == GeometryStatus::ConfigurationMissing && !missing.value(),
        "missing production geometry fails closed without a value");
    tests.expectTrue(missing.isValid() && missing.diagnostic().has_value(),
        "configuration failure carries diagnostic but no success geometry");

    const auto resolved = BilliardPhysics::resolveTableGeometry(stable.pockets, config);
    tests.expectTrue(resolved.isValid() && resolved.value().has_value(),
        "finite calibrated table geometry resolves");
    if (!resolved.value()) {
        return tests.exitCode();
    }
    const auto& table = *resolved.value();
    tests.expectNear(table.playableBallCenterRegion.bounds.minX, 10.0, TOLERANCE,
        "playable region is physical surface inset by one radius");
    for (std::size_t index = 0; index < table.pockets.size(); ++index) {
        tests.expectNear(table.pockets[index].wirePocketCenter.x, stable.pockets[index].x,
            TOLERANCE, "resolved pocket preserves current-cycle wire X");
        tests.expectNear(table.pockets[index].wirePocketCenter.y, stable.pockets[index].y,
            TOLERANCE, "resolved pocket preserves current-cycle wire Y");
        tests.expectTrue(static_cast<std::size_t>(table.pockets[index].id) == index,
            "fixed pocket ID remains associated with wire pocket index");
    }
    auto internalExitCenters = stable.pockets;
    internalExitCenters[1] = {500.0, 20.0};
    tests.expectTrue(
        BilliardPhysics::resolveTableGeometry(internalExitCenters, config).status() ==
            GeometryStatus::InvalidConfiguration,
        "pocket exit wholly inside the playable bounds is rejected");
    auto overlappingConfig = config;
    auto overlappingCenters = stable.pockets;
    overlappingConfig.pockets[0] = overlappingConfig.pockets[1];
    overlappingConfig.pockets[0].id = BilliardConfig::PocketId::Pocket1;
    overlappingCenters[0] = overlappingCenters[1];
    tests.expectTrue(
        BilliardPhysics::resolveTableGeometry(overlappingCenters, overlappingConfig).status() ==
            GeometryStatus::InvalidConfiguration,
        "overlapping pocket exits are rejected");
    auto railOverlapConfig = config;
    railOverlapConfig.rails[0].endExclusionMm = 0.0;
    tests.expectTrue(
        BilliardPhysics::resolveTableGeometry(stable.pockets, railOverlapConfig).status() ==
            GeometryStatus::InvalidConfiguration,
        "pocket exit cannot overlap an effective reflection rail");

    const auto ghost = BilliardPhysics::computeGhostBallPoint(
        {100.0, 100.0}, {300.0, 100.0}, {400.0, 100.0}, 10.0);
    tests.expectTrue(ghost.isValid() && ghost.value().has_value(),
        "finite nondegenerate ghost geometry succeeds");
    if (ghost.value()) {
        tests.expectNear(ghost.value()->center.x, 280.0, TOLERANCE,
            "ghost center is two radii behind target");
        tests.expectNear(ghost.value()->center.y, 100.0, TOLERANCE,
            "ghost, target and virtual pocket target are collinear");
        tests.expectTrue(ghost.value()->center.x < 300.0,
            "ghost lies on the side opposite the virtual pocket target");
        const double separation = std::hypot(
            ghost.value()->center.x - 300.0,
            ghost.value()->center.y - 100.0);
        tests.expectNear(separation, 20.0, TOLERANCE, "ghost-target separation equals 2r");
    }
    tests.expectTrue(ghost.diagnostic().ballSurfaceContactPoint.has_value(),
        "surface contact point is diagnostic metadata");
    if (ghost.diagnostic().ballSurfaceContactPoint) {
        tests.expectNear(ghost.diagnostic().ballSurfaceContactPoint->x, 290.0, TOLERANCE,
            "surface contact diagnostic is one radius behind target");
    }
    tests.expectTrue(
        BilliardPhysics::computeGhostBallPoint(
            {295.0, 100.0}, {300.0, 100.0}, {400.0, 100.0}, 10.0).status() ==
            GeometryStatus::DegenerateGeometry,
        "overlapping cue and target balls fail closed");
    tests.expectTrue(
        BilliardPhysics::computeGhostBallPoint(
            {100.0, 100.0}, {300.0, 100.0}, {300.0, 100.0}, 10.0).status() ==
            GeometryStatus::DegenerateGeometry,
        "zero target direction fails closed");

    const auto& bottomPocket = table.pockets[1];
    const auto entryZero = BilliardPhysics::computePocketEntryAngle(
        {500.0, 100.0}, bottomPocket);
    tests.expectTrue(entryZero.value().has_value(), "zero-degree pocket entry succeeds");
    if (entryZero.value()) {
        tests.expectNear(entryZero.value()->degrees, 0.0, TOLERANCE,
            "outward capture direction is zero degrees");
    }
    const auto entryThreshold = BilliardPhysics::computePocketEntryAngle(
        {600.0, 80.0}, bottomPocket);
    tests.expectTrue(entryThreshold.value().has_value(), "entry at angle threshold succeeds");
    if (entryThreshold.value()) {
        tests.expectNear(entryThreshold.value()->degrees, 45.0, TOLERANCE,
            "threshold entry angle uses unique outward-normal formula");
    }
    tests.expectTrue(
        BilliardPhysics::computePocketEntryAngle({500.0, -30.0}, bottomPocket).status() ==
            GeometryStatus::DirectionRejected,
        "180-degree reverse pocket entry is rejected");
    tests.expectTrue(
        BilliardPhysics::checkPocketExitDirection({0.0, -1.0}, bottomPocket).status() ==
            GeometryStatus::Clear,
        "positive outward pocket crossing direction succeeds");
    tests.expectTrue(
        BilliardPhysics::checkPocketExitDirection({1.0, 0.0}, bottomPocket).status() ==
            GeometryStatus::DirectionRejected,
        "tangential pocket exit direction is rejected");
    tests.expectTrue(
        BilliardPhysics::checkPocketExitDirection({0.0, 1.0}, bottomPocket).status() ==
            GeometryStatus::DirectionRejected,
        "reverse corridor-to-table direction is rejected");
    auto thresholdPocket = bottomPocket;
    thresholdPocket.exitCrossingEpsilon = 0.6;
    tests.expectTrue(
        BilliardPhysics::checkPocketExitDirection({0.8, -0.6}, thresholdPocket).status() ==
            GeometryStatus::DirectionRejected,
        "exit direction equal to threshold is rejected by strict crossing rule");
    auto reversedPocketConfig = config.pockets[1];
    reversedPocketConfig.outwardUnitNormal = {0.0, 1.0};
    tests.expectTrue(
        BilliardPhysics::resolvePocketModel(
            stable.pockets[1],
            reversedPocketConfig,
            table.playableBallCenterRegion).status() ==
            GeometryStatus::InvalidConfiguration,
        "pocket outward normal inconsistent with capture direction fails closed");
    auto degenerateCorridorConfig = config.pockets[1];
    degenerateCorridorConfig.corridorHalfWidthMm = 0.0;
    tests.expectTrue(
        BilliardPhysics::resolvePocketModel(
            stable.pockets[1],
            degenerateCorridorConfig,
            table.playableBallCenterRegion).status() ==
            GeometryStatus::InvalidConfiguration,
        "degenerate pocket corridor fails closed");

    const Segment2D targetPath{{500.0, 100.0}, bottomPocket.virtualPocketTarget};
    tests.expectTrue(
        BilliardPhysics::checkTargetPathToPocket(
            targetPath,
            BilliardConfig::PocketId::Pocket2,
            table.pockets,
            table.playableBallCenterRegion).status() == GeometryStatus::Clear,
        "target may leave playable region only through selected pocket corridor");
    auto conflictingPockets = table.pockets;
    conflictingPockets[0].pocketExitSegment = {{480.0, 50.0}, {520.0, 50.0}};
    tests.expectTrue(
        BilliardPhysics::checkTargetPathToPocket(
            targetPath,
            BilliardConfig::PocketId::Pocket2,
            conflictingPockets,
            table.playableBallCenterRegion).status() == GeometryStatus::InvalidConfiguration,
        "untrusted wrong-exit topology is rejected before path acceptance");
    auto malformedOtherPocket = table.pockets;
    malformedOtherPocket[0].pocketExitSegment.start.x =
        std::numeric_limits<double>::quiet_NaN();
    tests.expectTrue(
        BilliardPhysics::checkTargetPathToPocket(
            targetPath,
            BilliardConfig::PocketId::Pocket2,
            malformedOtherPocket,
            table.playableBallCenterRegion).status() ==
            GeometryStatus::InvalidConfiguration,
        "malformed non-selected pocket geometry invalidates the whole path check");
    tests.expectTrue(
        BilliardPhysics::checkSegmentWithinPlayableRegion(
            {{30.0, 10.0}, {970.0, 490.0}},
            table.playableBallCenterRegion).status() == GeometryStatus::Clear,
        "playable boundary is inclusive for ball-center paths");
    tests.expectTrue(
        BilliardPhysics::checkSegmentWithinPlayableRegion(
            {{30.0, 10.0}, {1001.0, 490.0}},
            table.playableBallCenterRegion).status() == GeometryStatus::OutsideValidRegion,
        "ordinary rail exit is rejected");

    const auto& effective = table.railReflectionRegion.rails[0];
    for (std::size_t index = 0; index < table.railReflectionRegion.rails.size(); ++index) {
        tests.expectTrue(
            static_cast<std::size_t>(
                table.railReflectionRegion.rails[index].physicalRailId) == index,
            "all six effective rails preserve their physical rail IDs");
    }
    tests.expectTrue(
        effective.physicalRailId == BilliardConfig::RailId::Rail1,
        "effective rail preserves physical rail ID");
    tests.expectNear(effective.segment.start.y, 10.0, TOLERANCE,
        "effective rail is offset inward by radius once without collision margin");
    tests.expectNear(effective.segment.start.x, 40.0, TOLERANCE,
        "physical start exclusion maps longitudinally to effective rail");
    tests.expectNear(effective.segment.end.x, 460.0, TOLERANCE,
        "physical end exclusion maps longitudinally to effective rail");
    auto reversedRail = config.rails[0];
    reversedRail.inwardUnitNormal = {0.0, -1.0};
    tests.expectTrue(
        BilliardPhysics::deriveEffectiveRail(
            reversedRail,
            table.playableBallCenterRegion,
            config.ballRadiusMm).status() == GeometryStatus::InvalidConfiguration,
        "outward/reversed rail normal fails closed");
    auto emptyRail = config.rails[0];
    emptyRail.startExclusionMm = 250.0;
    emptyRail.endExclusionMm = 250.0;
    tests.expectTrue(
        BilliardPhysics::deriveEffectiveRail(
            emptyRail,
            table.playableBallCenterRegion,
            config.ballRadiusMm).status() == GeometryStatus::InvalidConfiguration,
        "empty effective rail fails closed");
    auto zeroLengthRail = config.rails[0];
    zeroLengthRail.segment.end = zeroLengthRail.segment.start;
    tests.expectTrue(
        BilliardPhysics::deriveEffectiveRail(
            zeroLengthRail,
            table.playableBallCenterRegion,
            config.ballRadiusMm).status() == GeometryStatus::InvalidConfiguration,
        "zero-length physical rail fails closed");
    auto nonUnitRail = config.rails[0];
    nonUnitRail.inwardUnitNormal = {0.0, 2.0};
    tests.expectTrue(
        BilliardPhysics::deriveEffectiveRail(
            nonUnitRail,
            table.playableBallCenterRegion,
            config.ballRadiusMm).status() == GeometryStatus::InvalidConfiguration,
        "non-unit rail normal fails closed");
    auto nonFiniteRail = config.rails[0];
    nonFiniteRail.inwardUnitNormal = {
        0.0, std::numeric_limits<double>::infinity()};
    tests.expectTrue(
        BilliardPhysics::deriveEffectiveRail(
            nonFiniteRail,
            table.playableBallCenterRegion,
            config.ballRadiusMm).status() == GeometryStatus::InvalidConfiguration,
        "non-finite rail normal fails closed");

    const auto mirror = BilliardPhysics::mirrorPointAcrossEffectiveRail({100.0, 50.0}, effective);
    tests.expectTrue(mirror.value().has_value(), "point mirror across effective rail succeeds");
    if (mirror.value()) {
        tests.expectNear(mirror.value()->y, -30.0, TOLERANCE,
            "mirror uses effective ball-center rail rather than physical rail");
    }
    const auto rebound = BilliardPhysics::intersectRayWithEffectiveRail(
        {100.0, -30.0}, {100.0, 100.0}, effective);
    tests.expectTrue(rebound.value().has_value(), "ray intersects effective rail segment");
    if (rebound.value()) {
        tests.expectNear(rebound.value()->y, 10.0, TOLERANCE,
            "rebound point represents cue-ball center on effective rail");
    }
    const auto endpointRebound = BilliardPhysics::intersectRayWithEffectiveRail(
        {40.0, -30.0}, {40.0, 100.0}, effective);
    tests.expectTrue(endpointRebound.value().has_value(),
        "effective rail endpoint is a valid ray intersection");
    tests.expectTrue(
        BilliardPhysics::intersectRayWithEffectiveRail(
            {100.0, 20.0}, {200.0, 20.0}, effective).status() == GeometryStatus::Parallel,
        "parallel ray has a named failure");
    tests.expectTrue(
        BilliardPhysics::intersectRayWithEffectiveRail(
            {100.0, 10.0}, {200.0, 10.0}, effective).status() == GeometryStatus::Coincident,
        "coincident ray has a named failure");
    tests.expectTrue(
        BilliardPhysics::intersectRayWithEffectiveRail(
            {100.0, 50.0}, {100.0, 100.0}, effective).status() == GeometryStatus::NoIntersection,
        "ray pointing away from rail has a named no-intersection failure");

    const std::vector<Point> obstacles{{50.0, 20.0}};
    tests.expectTrue(
        BilliardPhysics::checkSegmentCollision(
            {{0.0, 0.0}, {100.0, 0.0}}, obstacles, {}, 20.0, 0.0).status() ==
            GeometryStatus::Blocked,
        "exact tangent at configured clearance is blocked");
    tests.expectTrue(
        BilliardPhysics::checkSegmentCollision(
            {{0.0, 0.0}, {100.0, 0.0}}, {{50.0, 20.001}}, {}, 20.0, 0.0).status() ==
            GeometryStatus::Clear,
        "near tangent outside configured clearance is clear");
    tests.expectTrue(
        BilliardPhysics::checkSegmentCollision(
            {{0.0, 0.0}, {100.0, 0.0}}, {{50.0, 22.0}}, {}, 20.0, 2.0).status() ==
            GeometryStatus::Blocked,
        "collision margin is applied once to the ball-to-ball threshold");
    tests.expectTrue(
        BilliardPhysics::checkSegmentCollision(
            {{0.0, 0.0}, {100.0, 0.0}}, {{-20.0, 0.0}}, {}, 20.0, 0.0).status() ==
            GeometryStatus::Blocked,
        "nearest-distance collision clamps to segment endpoint");
    tests.expectTrue(
        BilliardPhysics::checkSegmentCollision(
            {{0.0, 0.0}, {100.0, 0.0}}, obstacles, {0}, 20.0, 0.0).status() ==
            GeometryStatus::Clear,
        "explicit path endpoint-ball exclusion is honored");
    tests.expectTrue(
        BilliardPhysics::checkSegmentCollision(
            {{0.0, 0.0}, {0.0, 0.0}}, obstacles, {}, 20.0, 0.0).status() ==
            GeometryStatus::DegenerateGeometry,
        "zero-length collision path fails closed");
    tests.expectTrue(
        BilliardPhysics::checkSegmentCollision(
            {{0.0, 0.0}, {100.0, 0.0}},
            {{std::numeric_limits<double>::quiet_NaN(), 0.0}}, {}, 20.0, 0.0).status() ==
            GeometryStatus::InvalidInput,
        "non-finite obstacle fails closed");
    tests.expectTrue(
        BilliardPhysics::checkSegmentCollision(
            {{0.0, 0.0}, {100.0, 0.0}},
            {{50.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0}},
            {}, 20.0, 0.0).status() == GeometryStatus::InvalidInput,
        "invalid input takes priority over an earlier blocking obstacle");

    return tests.exitCode();
}
