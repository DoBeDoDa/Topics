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

BilliardConfig::TableGeometryConfig validConfig()
{
    using namespace BilliardConfig;
    // rail的segment不再寫死，改由startPocket/endPocket於resolveTableGeometry
    // 查stableStateWithNoNumberedBalls()的pockets即時組成。下方pocket六邊形
    // 座標即對應(0,0)-(500,0)-(1000,0)-(1000,500)-(500,500)-(0,500)逆時針一圈，
    // 讓Rail1實際線段與舊版寫死值({0,0}-{500,0})完全相同。
    return {
        "table-geometry-test-v1",
        {0.0, 1000.0, 0.0, 500.0},
        10.0,
        20.0,
        2.0,
        {{
            {RailId::Rail1, PocketId::Pocket1, PocketId::Pocket2, {0.0, 1.0}, 40.0, 40.0, 0.0},
            {RailId::Rail2, PocketId::Pocket2, PocketId::Pocket3, {0.0, 1.0}, 40.0, 40.0, 0.0},
            {RailId::Rail3, PocketId::Pocket3, PocketId::Pocket4, {-1.0, 0.0}, 40.0, 40.0, 0.0},
            {RailId::Rail4, PocketId::Pocket4, PocketId::Pocket5, {0.0, -1.0}, 40.0, 40.0, 0.0},
            {RailId::Rail5, PocketId::Pocket5, PocketId::Pocket6, {0.0, -1.0}, 40.0, 40.0, 0.0},
            {RailId::Rail6, PocketId::Pocket6, PocketId::Pocket1, {1.0, 0.0}, 40.0, 40.0, 0.0}
        }}};
}

StableTableState stableStateWithNoNumberedBalls()
{
    std::array<std::optional<Point>, 9> balls{};
    const std::array<Point, 6> pockets{{
        {0.0, 0.0}, {500.0, 0.0}, {1000.0, 0.0},
        {1000.0, 500.0}, {500.0, 500.0}, {0.0, 500.0}}};
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
    for (std::size_t index = 0; index < table.pockets.size(); ++index) {
        tests.expectNear(table.pockets[index].center.x, stable.pockets[index].x,
            TOLERANCE, "resolved pocket preserves current-cycle vision X");
        tests.expectNear(table.pockets[index].center.y, stable.pockets[index].y,
            TOLERANCE, "resolved pocket preserves current-cycle vision Y");
        tests.expectTrue(static_cast<std::size_t>(table.pockets[index].id) == index,
            "fixed pocket ID remains associated with vision pocket index");
    }

    const auto ghost = BilliardPhysics::computeGhostBallPoint(
        {100.0, 100.0}, {300.0, 100.0}, {400.0, 100.0}, 10.0);
    tests.expectTrue(ghost.isValid() && ghost.value().has_value(),
        "finite nondegenerate ghost geometry succeeds");
    if (ghost.value()) {
        tests.expectNear(ghost.value()->center.x, 280.0, TOLERANCE,
            "ghost center is two radii behind target");
        tests.expectNear(ghost.value()->center.y, 100.0, TOLERANCE,
            "ghost, target and pocket target are collinear");
        tests.expectTrue(ghost.value()->center.x < 300.0,
            "ghost lies on the side opposite the pocket target");
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

    // 獨立於已配置袋口，僅作為checkTargetPathToPocket的通用幾何測試點。
    const Point bottomPocketTarget{500.0, 10.0};
    const Segment2D targetPath{{500.0, 100.0}, bottomPocketTarget};
    tests.expectTrue(
        BilliardPhysics::checkTargetPathToPocket(
            targetPath,
            bottomPocketTarget).status() == GeometryStatus::Clear,
        "target path ending exactly at the pocket target is accepted");
    tests.expectTrue(
        BilliardPhysics::checkTargetPathToPocket(
            {{500.0, 100.0}, {500.0, 100.0}},
            bottomPocketTarget).status() == GeometryStatus::DegenerateGeometry,
        "zero-length target path fails closed");
    tests.expectTrue(
        BilliardPhysics::checkTargetPathToPocket(
            targetPath,
            {600.0, 100.0}).status() == GeometryStatus::InvalidConfiguration,
        "target path ending away from the pocket target is rejected");

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
    const Segment2D rail1Segment{{0.0, 0.0}, {500.0, 0.0}};

    auto emptyRail = config.rails[0];
    emptyRail.startExclusionMm = 250.0;
    emptyRail.endExclusionMm = 250.0;
    tests.expectTrue(
        BilliardPhysics::deriveEffectiveRail(
            emptyRail,
            rail1Segment,
            {0.0, 1.0},
            config.ballRadiusMm).status() == GeometryStatus::InvalidConfiguration,
        "empty effective rail fails closed");
    const Segment2D zeroLengthSegment{{0.0, 0.0}, {0.0, 0.0}};
    tests.expectTrue(
        BilliardPhysics::deriveEffectiveRail(
            config.rails[0],
            zeroLengthSegment,
            {0.0, 1.0},
            config.ballRadiusMm).status() == GeometryStatus::InvalidConfiguration,
        "zero-length physical rail fails closed");
    tests.expectTrue(
        BilliardPhysics::deriveEffectiveRail(
            config.rails[0],
            rail1Segment,
            {0.0, 2.0},
            config.ballRadiusMm).status() == GeometryStatus::InvalidConfiguration,
        "non-unit rail normal fails closed");
    tests.expectTrue(
        BilliardPhysics::deriveEffectiveRail(
            config.rails[0],
            rail1Segment,
            {0.0, std::numeric_limits<double>::infinity()},
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
