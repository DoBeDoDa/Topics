#include "TestHarness.h"

#include "../src/Algorithm.h"
#include "../src/BilliardPhysics.h"
#include "../src/TargetSelector.h"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

namespace {
constexpr double ROOT_HALF = 0.70710678118654752440;
constexpr double TOLERANCE = 1e-9;

BilliardConfig::PocketModelConfig pocketConfig(
    BilliardConfig::PocketId id,
    BilliardConfig::PocketType type,
    Vector2D outward)
{
    const Vector2D side{-outward.y, outward.x};
    const double halfLength =
        type == BilliardConfig::PocketType::Corner ? 14.142135623730951 : 20.0;
    return {
        id,
        type,
        outward,
        30.0,
        {{-side.x * halfLength, -side.y * halfLength},
         {side.x * halfLength, side.y * halfLength}},
        15.0,
        2.0,
        0.01,
        45.0};
}

BilliardConfig::TableGeometryConfig tableConfig()
{
    using namespace BilliardConfig;
    return {
        "p1-06-test-v1",
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

std::array<Point, 6> pocketCenters()
{
    return {{
        {20.0, 20.0}, {500.0, 10.0}, {980.0, 20.0},
        {20.0, 480.0}, {500.0, 490.0}, {980.0, 480.0}}};
}

StableTableState state(
    Point cueBall,
    std::array<std::optional<Point>, 9> balls)
{
    const auto now = std::chrono::steady_clock::now();
    return {
        std::move(balls),
        cueBall,
        pocketCenters(),
        1,
        1,
        {{{1, now}, {2, now}, {3, now}}}};
}

std::optional<ResolvedTableGeometry> resolvedGeometry(TestHarness& tests)
{
    const auto result = BilliardPhysics::resolveTableGeometry(
        pocketCenters(),
        tableConfig());
    tests.expectTrue(result.value().has_value(), "P1-05 geometry fixture resolves");
    return result.value();
}

StableTableState alignedStateForPocket(
    const ResolvedTableGeometry& geometry,
    std::size_t pocketIndex)
{
    const auto& pocket = geometry.pockets[pocketIndex];
    const Vector2D outward = pocket.outwardUnitNormal;
    const Point target{
        pocket.wirePocketCenter.x - 100.0 * outward.x,
        pocket.wirePocketCenter.y - 100.0 * outward.y};
    const Point ghost{
        target.x - 2.0 * geometry.ballRadiusMm * outward.x,
        target.y - 2.0 * geometry.ballRadiusMm * outward.y};
    const Point cue{
        ghost.x - 100.0 * outward.x,
        ghost.y - 100.0 * outward.y};
    std::array<std::optional<Point>, 9> balls{};
    balls[0] = target;
    return state(cue, balls);
}

const DirectPotCandidateDiagnostic* diagnosticFor(
    const DirectPotEvaluation& evaluation,
    std::size_t pocketIndex)
{
    const auto& diagnostic = evaluation.rejected[pocketIndex];
    return diagnostic ? &*diagnostic : nullptr;
}
}

int main()
{
    TestHarness tests;
    TargetSelector selector;

    std::array<std::optional<Point>, 9> balls{};
    balls[2] = Point{300.0, 200.0};
    balls[5] = Point{600.0, 200.0};
    const auto lowest = selector.select(state({500.0, 300.0}, balls));
    tests.expectTrue(lowest.isValid() && lowest.value().has_value(),
        "TargetSelector returns a successful lowest target value");
    tests.expectTrue(lowest.audit().expectedBallSetNotApplied,
        "successful selection audits the V1 ExpectedBallSet limitation");
    if (lowest.value()) {
        tests.expectTrue(lowest.value()->ballNumber == 3,
            "TargetSelector selects the lowest present numbered ball");
        tests.expectNear(lowest.value()->center.x, 300.0, TOLERANCE,
            "TargetSelector preserves Base0 X without compensation");
        tests.expectNear(lowest.value()->center.y, 200.0, TOLERANCE,
            "TargetSelector preserves Base0 Y without compensation");
    }

    const auto noTarget = selector.select(state({500.0, 300.0}, {}));
    tests.expectTrue(
        noTarget.status() == TargetQualificationStatus::NoEligibleTarget &&
        !noTarget.value() && noTarget.isValid(),
        "zero numbered balls produces only NoEligibleTarget");
    tests.expectTrue(noTarget.audit().expectedBallSetNotApplied,
        "NoEligibleTarget audits the V1 ExpectedBallSet limitation");

    balls = {};
    balls[0] = Point{std::numeric_limits<double>::quiet_NaN(), 100.0};
    const auto invalidTargetState = selector.select(state({500.0, 300.0}, balls));
    tests.expectTrue(
        invalidTargetState.status() == TargetQualificationStatus::InvalidStableState &&
        !invalidTargetState.value(),
        "non-finite numbered-ball state fails closed");

    const auto geometryValue = resolvedGeometry(tests);
    if (!geometryValue) {
        return tests.exitCode();
    }
    const auto& geometry = *geometryValue;
    for (std::size_t pocketIndex = 0; pocketIndex < geometry.pockets.size(); ++pocketIndex) {
        const auto stable = alignedStateForPocket(geometry, pocketIndex);
        const auto target = selector.select(stable);
        tests.expectTrue(target.value().has_value(), "aligned fixture has a target");
        if (!target.value()) continue;
        const auto direct = BilliardAlgorithm::generateDirectPotCandidates(
            stable,
            *target.value(),
            geometry);
        tests.expectTrue(direct.isValid() && direct.value().has_value(),
            "six-pocket Direct evaluation succeeds as a business result");
        if (!direct.value()) continue;
        const auto& evaluation = *direct.value();
        tests.expectTrue(evaluation.isValid(),
            "each pocket has exactly one feasible candidate or rejection diagnostic");
        tests.expectTrue(evaluation.feasible[pocketIndex].has_value(),
            "each of the six current-cycle pockets can independently form DirectPot");
        if (evaluation.feasible[pocketIndex]) {
            const auto& candidate = *evaluation.feasible[pocketIndex];
            tests.expectTrue(candidate.target.ballNumber == 1,
                "Direct candidate keeps the selected lowest target fixed");
            tests.expectTrue(static_cast<std::size_t>(candidate.pocketId) == pocketIndex,
                "Direct candidate preserves pocket ID");
            tests.expectNear(candidate.virtualPocketTarget.x,
                geometry.pockets[pocketIndex].virtualPocketTarget.x,
                TOLERANCE, "Direct candidate uses resolved virtual target X");
            tests.expectNear(candidate.virtualPocketTarget.y,
                geometry.pockets[pocketIndex].virtualPocketTarget.y,
                TOLERANCE, "Direct candidate uses resolved virtual target Y");
            tests.expectNear(candidate.cuePath.end.x, candidate.ghostBallPoint.center.x,
                TOLERANCE, "cue path ends at GhostBallPoint X");
            tests.expectNear(candidate.cuePath.end.y, candidate.ghostBallPoint.center.y,
                TOLERANCE, "cue path ends at GhostBallPoint Y");
            tests.expectNear(candidate.targetPath.end.x, candidate.virtualPocketTarget.x,
                TOLERANCE, "target path ends at VirtualPocketTarget X");
            tests.expectNear(candidate.targetPath.end.y, candidate.virtualPocketTarget.y,
                TOLERANCE, "target path ends at VirtualPocketTarget Y");
            tests.expectNear(
                std::hypot(
                    candidate.target.center.x - candidate.ghostBallPoint.center.x,
                    candidate.target.center.y - candidate.ghostBallPoint.center.y),
                2.0 * geometry.ballRadiusMm,
                TOLERANCE,
                "Direct GhostBallPoint is exactly 2r from target");
        }
    }

    auto nearContactState = alignedStateForPocket(geometry, 1);
    const Point nearContactTarget = *nearContactState.objectBalls[0];
    const Vector2D nearContactOutward = geometry.pockets[1].outwardUnitNormal;
    const Point nearContactGhost{
        nearContactTarget.x - 2.0 * geometry.ballRadiusMm * nearContactOutward.x,
        nearContactTarget.y - 2.0 * geometry.ballRadiusMm * nearContactOutward.y};
    nearContactState.cueBall = {
        nearContactGhost.x - nearContactOutward.x,
        nearContactGhost.y - nearContactOutward.y};
    const auto nearContactSelection = selector.select(nearContactState);
    tests.expectTrue(nearContactSelection.value().has_value(),
        "near-contact fixture has a selected target");
    if (!nearContactSelection.value()) {
        return tests.exitCode();
    }
    const auto nearContact = BilliardAlgorithm::generateDirectPotCandidates(
        nearContactState, *nearContactSelection.value(), geometry);
    tests.expectTrue(nearContact.value().has_value(),
        "near-contact Direct evaluation returns a business value");
    if (!nearContact.value()) {
        return tests.exitCode();
    }
    tests.expectTrue(nearContact.value()->feasible[1].has_value(),
        "pre-shot cue center is not a stationary target-path obstacle");
    if (nearContact.value()->feasible[1]) {
        auto malformedEvaluation = *nearContact.value();
        malformedEvaluation.feasible[1]->cuePath.end.x =
            std::numeric_limits<double>::quiet_NaN();
        tests.expectFalse(malformedEvaluation.isValid(),
            "Direct success payload invariant rejects non-finite path geometry");
    }

    auto blockedCueState = alignedStateForPocket(geometry, 1);
    const Point cueMid{
        (blockedCueState.cueBall.x +
         (blockedCueState.objectBalls[0]->x -
          2.0 * geometry.ballRadiusMm * geometry.pockets[1].outwardUnitNormal.x)) / 2.0,
        (blockedCueState.cueBall.y +
         (blockedCueState.objectBalls[0]->y -
          2.0 * geometry.ballRadiusMm * geometry.pockets[1].outwardUnitNormal.y)) / 2.0};
    blockedCueState.objectBalls[1] = cueMid;
    const auto blockedCueTarget = selector.select(blockedCueState);
    if (!blockedCueTarget.value()) {
        tests.expectTrue(false, "blocked-cue fixture has a selected target");
        return tests.exitCode();
    }
    const auto blockedCue = BilliardAlgorithm::generateDirectPotCandidates(
        blockedCueState, *blockedCueTarget.value(), geometry);
    if (!blockedCue.value()) {
        tests.expectTrue(false, "blocked-cue evaluation returns a business value");
        return tests.exitCode();
    }
    tests.expectTrue(
        diagnosticFor(*blockedCue.value(), 1) &&
        diagnosticFor(*blockedCue.value(), 1)->reason ==
            DirectPotRejectionReason::CuePathBlocked,
        "blocked cue path is rejected with a named diagnostic");

    auto blockedTargetState = alignedStateForPocket(geometry, 1);
    blockedTargetState.objectBalls[1] = Point{
        (blockedTargetState.objectBalls[0]->x +
         geometry.pockets[1].virtualPocketTarget.x) / 2.0,
        (blockedTargetState.objectBalls[0]->y +
         geometry.pockets[1].virtualPocketTarget.y) / 2.0};
    const auto blockedTargetSelection = selector.select(blockedTargetState);
    if (!blockedTargetSelection.value()) {
        tests.expectTrue(false, "blocked-target fixture has a selected target");
        return tests.exitCode();
    }
    const auto blockedTarget = BilliardAlgorithm::generateDirectPotCandidates(
        blockedTargetState, *blockedTargetSelection.value(), geometry);
    if (!blockedTarget.value()) {
        tests.expectTrue(false, "blocked-target evaluation returns a business value");
        return tests.exitCode();
    }
    tests.expectTrue(
        diagnosticFor(*blockedTarget.value(), 1) &&
        diagnosticFor(*blockedTarget.value(), 1)->reason ==
            DirectPotRejectionReason::TargetPathBlocked,
        "blocked target path is rejected independently");

    auto degenerateState = alignedStateForPocket(geometry, 1);
    degenerateState.cueBall = *degenerateState.objectBalls[0];
    const auto degenerateSelection = selector.select(degenerateState);
    if (!degenerateSelection.value()) {
        tests.expectTrue(false, "degenerate fixture has a selected target");
        return tests.exitCode();
    }
    const auto degenerate = BilliardAlgorithm::generateDirectPotCandidates(
        degenerateState, *degenerateSelection.value(), geometry);
    if (!degenerate.value()) {
        tests.expectTrue(false, "degenerate evaluation returns a business value");
        return tests.exitCode();
    }
    tests.expectTrue(
        diagnosticFor(*degenerate.value(), 1) &&
        diagnosticFor(*degenerate.value(), 1)->reason ==
            DirectPotRejectionReason::GhostGeometryInvalid,
        "overlapping cue and target geometry is rejected without fallback Point");

    auto wrongExitGeometry = geometry;
    wrongExitGeometry.pockets[0].pocketExitSegment.start.x =
        std::numeric_limits<double>::quiet_NaN();
    const auto normalState = alignedStateForPocket(geometry, 1);
    const auto normalSelection = selector.select(normalState);
    if (!normalSelection.value()) {
        tests.expectTrue(false, "wrong-exit fixture has a selected target");
        return tests.exitCode();
    }
    const auto wrongExit = BilliardAlgorithm::generateDirectPotCandidates(
        normalState, *normalSelection.value(), wrongExitGeometry);
    if (!wrongExit.value()) {
        tests.expectTrue(false, "wrong-exit evaluation returns a business value");
        return tests.exitCode();
    }
    tests.expectTrue(
        diagnosticFor(*wrongExit.value(), 1) &&
        diagnosticFor(*wrongExit.value(), 1)->reason ==
            DirectPotRejectionReason::TargetPathInvalid,
        "wrong or malformed pocket exit rejects Direct candidate");

    auto entryGeometry = geometry;
    entryGeometry.pockets[1].maxEntryAngleDeg = 5.0;
    auto entryState = alignedStateForPocket(entryGeometry, 1);
    const Point entryTarget{520.0, 110.0};
    const Vector2D toPocket{
        entryGeometry.pockets[1].virtualPocketTarget.x - entryTarget.x,
        entryGeometry.pockets[1].virtualPocketTarget.y - entryTarget.y};
    const double toPocketLength = std::hypot(toPocket.x, toPocket.y);
    const Vector2D entryUnit{toPocket.x / toPocketLength, toPocket.y / toPocketLength};
    const Point entryGhost{
        entryTarget.x - 2.0 * geometry.ballRadiusMm * entryUnit.x,
        entryTarget.y - 2.0 * geometry.ballRadiusMm * entryUnit.y};
    entryState.objectBalls[0] = entryTarget;
    entryState.cueBall = {
        entryGhost.x - 100.0 * entryUnit.x,
        entryGhost.y - 100.0 * entryUnit.y};
    const auto entrySelection = selector.select(entryState);
    if (!entrySelection.value()) {
        tests.expectTrue(false, "entry-angle fixture has a selected target");
        return tests.exitCode();
    }
    const auto entryRejected = BilliardAlgorithm::generateDirectPotCandidates(
        entryState, *entrySelection.value(), entryGeometry);
    if (!entryRejected.value()) {
        tests.expectTrue(false, "entry-angle evaluation returns a business value");
        return tests.exitCode();
    }
    tests.expectTrue(
        diagnosticFor(*entryRejected.value(), 1) &&
        diagnosticFor(*entryRejected.value(), 1)->reason ==
            DirectPotRejectionReason::PocketEntryRejected,
        "entry angle over the pocket threshold rejects Direct candidate");

    auto cutState = alignedStateForPocket(geometry, 1);
    const Point targetPoint = *cutState.objectBalls[0];
    const Point ghostPoint{
        targetPoint.x - 2.0 * geometry.ballRadiusMm *
            geometry.pockets[1].outwardUnitNormal.x,
        targetPoint.y - 2.0 * geometry.ballRadiusMm *
            geometry.pockets[1].outwardUnitNormal.y};
    cutState.cueBall = {
        ghostPoint.x + 100.0 * geometry.pockets[1].outwardUnitNormal.x,
        ghostPoint.y + 100.0 * geometry.pockets[1].outwardUnitNormal.y};
    const auto cutSelection = selector.select(cutState);
    if (!cutSelection.value()) {
        tests.expectTrue(false, "cut-angle fixture has a selected target");
        return tests.exitCode();
    }
    const auto cutRejected = BilliardAlgorithm::generateDirectPotCandidates(
        cutState, *cutSelection.value(), geometry);
    if (!cutRejected.value()) {
        tests.expectTrue(false, "cut-angle evaluation returns a business value");
        return tests.exitCode();
    }
    tests.expectTrue(
        diagnosticFor(*cutRejected.value(), 1) &&
        diagnosticFor(*cutRejected.value(), 1)->reason ==
            DirectPotRejectionReason::CutAngleInvalid,
        "non-forward cutting angle rejects Direct candidate");

    auto mismatchState = alignedStateForPocket(geometry, 1);
    mismatchState.objectBalls[1] = Point{700.0, 250.0};
    EligibleTarget forcedHigher{2, *mismatchState.objectBalls[1]};
    const auto mismatch = BilliardAlgorithm::generateDirectPotCandidates(
        mismatchState, forcedHigher, geometry);
    tests.expectTrue(
        mismatch.status() == DirectPotGenerationStatus::SelectedTargetMismatch &&
        !mismatch.value(),
        "Algorithm refuses a higher-number target while a lower ball is present");

    return tests.exitCode();
}
