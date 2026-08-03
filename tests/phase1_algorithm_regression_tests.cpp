#include "TestHarness.h"

#include "../src/Algorithm.h"
#include "../src/BilliardPhysics.h"
#include "../src/TargetSelector.h"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

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

BilliardConfig::KickGeometryConfig kickConfig()
{
    return {89.0, 1e-8, 1e-8};
}

StableTableState kickStateForRebound(
    const ResolvedTableGeometry& geometry,
    std::size_t railIndex,
    std::size_t pocketIndex,
    Point rebound,
    double incomingDistanceMm = 120.0)
{
    const auto& rail = geometry.railReflectionRegion.rails[railIndex];
    const auto& pocket = geometry.pockets[pocketIndex];
    const Point target{
        pocket.wirePocketCenter.x - 100.0 * pocket.outwardUnitNormal.x,
        pocket.wirePocketCenter.y - 100.0 * pocket.outwardUnitNormal.y};
    const Point ghost{
        target.x - 2.0 * geometry.ballRadiusMm * pocket.outwardUnitNormal.x,
        target.y - 2.0 * geometry.ballRadiusMm * pocket.outwardUnitNormal.y};
    const Vector2D outgoingRaw{ghost.x - rebound.x, ghost.y - rebound.y};
    const double outgoingLength = std::hypot(outgoingRaw.x, outgoingRaw.y);
    const Vector2D outgoing{
        outgoingRaw.x / outgoingLength,
        outgoingRaw.y / outgoingLength};
    const double normalProjection =
        outgoing.x * rail.inwardUnitNormal.x + outgoing.y * rail.inwardUnitNormal.y;
    const Vector2D incoming{
        outgoing.x - 2.0 * normalProjection * rail.inwardUnitNormal.x,
        outgoing.y - 2.0 * normalProjection * rail.inwardUnitNormal.y};
    const Point cue{
        rebound.x - incomingDistanceMm * incoming.x,
        rebound.y - incomingDistanceMm * incoming.y};
    std::array<std::optional<Point>, 9> balls{};
    balls[0] = target;
    return state(cue, balls);
}

StableTableState alignedKickState(
    const ResolvedTableGeometry& geometry,
    std::size_t railIndex,
    std::size_t pocketIndex)
{
    const auto& rail = geometry.railReflectionRegion.rails[railIndex];
    return kickStateForRebound(
        geometry,
        railIndex,
        pocketIndex,
        {(rail.segment.start.x + rail.segment.end.x) / 2.0,
         (rail.segment.start.y + rail.segment.end.y) / 2.0});
}

template <typename Candidate, typename = void>
struct HasSecondReboundPoint : std::false_type {};

template <typename Candidate>
struct HasSecondReboundPoint<
    Candidate,
    std::void_t<decltype(std::declval<Candidate>().secondReboundPoint)>>
    : std::true_type {};

const KickPotCandidateDiagnostic* kickDiagnosticFor(
    const KickPotEvaluation& evaluation,
    std::size_t pocketIndex,
    std::size_t railIndex)
{
    const auto& diagnostic = evaluation.rejected[pocketIndex][railIndex];
    return diagnostic ? &*diagnostic : nullptr;
}
}

int main()
{
    using KickGeneratorSignature = KickPotGenerationResult (*)(
        const StableTableState&,
        const EligibleTarget&,
        const ResolvedTableGeometry&,
        const std::optional<BilliardConfig::KickGeometryConfig>&);
    static_assert(
        std::is_same_v<
            decltype(&BilliardAlgorithm::generateKickPotCandidates),
            KickGeneratorSignature>,
        "P1-07 API must not accept dynamics or fixed-force parameters");
    static_assert(
        !HasSecondReboundPoint<KickPotCandidate>::value,
        "P1-07 candidate must not represent a second rail rebound");

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

    const std::array<std::size_t, 6> oppositePocketForRail{{4, 4, 1, 1, 2, 0}};
    for (std::size_t railIndex = 0;
         railIndex < geometry.railReflectionRegion.rails.size();
         ++railIndex) {
        const std::size_t pocketIndex = oppositePocketForRail[railIndex];
        const auto kickState = alignedKickState(geometry, railIndex, pocketIndex);
        const auto kickTarget = selector.select(kickState);
        tests.expectTrue(kickTarget.value().has_value(),
            "one-rail fixture has the lowest target");
        if (!kickTarget.value()) {
            return tests.exitCode();
        }
        const auto kick = BilliardAlgorithm::generateKickPotCandidates(
            kickState,
            *kickTarget.value(),
            geometry,
            std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
        tests.expectTrue(kick.isValid() && kick.value().has_value(),
            "one-rail evaluation returns a valid business value");
        if (!kick.value()) {
            return tests.exitCode();
        }
        const auto& evaluation = *kick.value();
        tests.expectTrue(evaluation.isValid(),
            "each pocket/rail pair has one candidate or rejection diagnostic");
        const auto& candidate = evaluation.feasible[pocketIndex][railIndex];
        tests.expectTrue(candidate.has_value(),
            "each effective rail can produce an ideal one-rail Kick fixture");
        if (!candidate) {
            continue;
        }
        tests.expectTrue(candidate->target.ballNumber == 1,
            "Kick preserves the selected lowest target");
        tests.expectTrue(candidate->cuePathFirst.start.x == kickState.cueBall.x &&
            candidate->cuePathFirst.start.y == kickState.cueBall.y,
            "Kick first cue segment starts at C");
        tests.expectTrue(candidate->cuePathFirst.end.x == candidate->reboundPoint.x &&
            candidate->cuePathFirst.end.y == candidate->reboundPoint.y &&
            candidate->cuePathSecond.start.x == candidate->reboundPoint.x &&
            candidate->cuePathSecond.start.y == candidate->reboundPoint.y,
            "Kick cue path is C to R to GhostBallPoint");
        const auto& expectedRail = geometry.railReflectionRegion.rails[railIndex];
        tests.expectNear(candidate->reboundPoint.x,
            (expectedRail.segment.start.x + expectedRail.segment.end.x) / 2.0,
            TOLERANCE,
            "Kick rebound X lies at the fixture point on the effective rail");
        tests.expectNear(candidate->reboundPoint.y,
            (expectedRail.segment.start.y + expectedRail.segment.end.y) / 2.0,
            TOLERANCE,
            "Kick rebound Y lies at the fixture point on the effective rail");
        tests.expectTrue(candidate->cuePathSecond.end.x == candidate->ghostBallPoint.center.x &&
            candidate->cuePathSecond.end.y == candidate->ghostBallPoint.center.y,
            "Kick second cue segment ends at GhostBallPoint");
        tests.expectNear(
            std::hypot(
                candidate->target.center.x - candidate->ghostBallPoint.center.x,
                candidate->target.center.y - candidate->ghostBallPoint.center.y),
            2.0 * geometry.ballRadiusMm,
            TOLERANCE,
            "Kick GhostBallPoint remains exactly 2r from target");
        tests.expectTrue(
            candidate->targetPath.end.x ==
                geometry.pockets[pocketIndex].virtualPocketTarget.x &&
            candidate->targetPath.end.y ==
                geometry.pockets[pocketIndex].virtualPocketTarget.y,
            "Kick target path uses the same resolved VirtualPocketTarget as Direct");
        tests.expectNear(candidate->incidenceAngleDeg,
            candidate->reflectionAngleDeg,
            kickConfig().reflectionAngleToleranceDeg,
            "ideal one-rail incidence and reflection angles are equal");
        const Vector2D incomingRaw{
            candidate->cuePathFirst.end.x - candidate->cuePathFirst.start.x,
            candidate->cuePathFirst.end.y - candidate->cuePathFirst.start.y};
        const Vector2D outgoingRaw{
            candidate->cuePathSecond.end.x - candidate->cuePathSecond.start.x,
            candidate->cuePathSecond.end.y - candidate->cuePathSecond.start.y};
        const double incomingLength = std::hypot(incomingRaw.x, incomingRaw.y);
        const double outgoingLength = std::hypot(outgoingRaw.x, outgoingRaw.y);
        const Vector2D incoming{
            incomingRaw.x / incomingLength,
            incomingRaw.y / incomingLength};
        const Vector2D outgoing{
            outgoingRaw.x / outgoingLength,
            outgoingRaw.y / outgoingLength};
        const auto& rail = geometry.railReflectionRegion.rails[railIndex];
        const double normalProjection =
            incoming.x * rail.inwardUnitNormal.x +
            incoming.y * rail.inwardUnitNormal.y;
        const Vector2D idealReflected{
            incoming.x - 2.0 * normalProjection * rail.inwardUnitNormal.x,
            incoming.y - 2.0 * normalProjection * rail.inwardUnitNormal.y};
        tests.expectNear(outgoing.x, idealReflected.x,
            kickConfig().reflectionDirectionTolerance,
            "Kick outgoing X equals the ideal reflected direction");
        tests.expectNear(outgoing.y, idealReflected.y,
            kickConfig().reflectionDirectionTolerance,
            "Kick outgoing Y equals the ideal reflected direction");
    }

    const auto kickFixture = alignedKickState(geometry, 0, 4);
    const auto kickFixtureTarget = selector.select(kickFixture);
    if (!kickFixtureTarget.value()) {
        tests.expectTrue(false, "Kick rejection fixture has a selected target");
        return tests.exitCode();
    }
    const auto missingKickConfig = BilliardAlgorithm::generateKickPotCandidates(
        kickFixture, *kickFixtureTarget.value(), geometry, std::nullopt);
    tests.expectTrue(
        missingKickConfig.status() == KickPotGenerationStatus::ConfigurationMissing &&
        !missingKickConfig.value(),
        "missing approved Kick geometry parameters fail closed");

    auto zeroAngleConfig = kickConfig();
    zeroAngleConfig.maxKickRailAngleDeg = 0.0;
    const auto zeroAngle = BilliardAlgorithm::generateKickPotCandidates(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{zeroAngleConfig});
    tests.expectTrue(
        zeroAngle.status() == KickPotGenerationStatus::Success && zeroAngle.value(),
        "zero-degree Kick threshold is a valid fail-closed geometry gate");

    auto reversedRailGeometry = geometry;
    reversedRailGeometry.railReflectionRegion.rails[0].inwardUnitNormal.x *= -1.0;
    reversedRailGeometry.railReflectionRegion.rails[0].inwardUnitNormal.y *= -1.0;
    const auto reversedRail = BilliardAlgorithm::generateKickPotCandidates(
        kickFixture,
        *kickFixtureTarget.value(),
        reversedRailGeometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    tests.expectTrue(
        reversedRail.status() == KickPotGenerationStatus::InvalidGeometryConfiguration &&
        !reversedRail.value(),
        "reversed effective-rail normal fails closed");

    auto nonFiniteKickConfig = kickConfig();
    nonFiniteKickConfig.maxKickRailAngleDeg =
        std::numeric_limits<double>::quiet_NaN();
    const auto invalidKickConfig = BilliardAlgorithm::generateKickPotCandidates(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{nonFiniteKickConfig});
    tests.expectTrue(
        invalidKickConfig.status() ==
            KickPotGenerationStatus::InvalidGeometryConfiguration &&
        !invalidKickConfig.value(),
        "non-finite Kick geometry parameters fail closed");

    auto nonFiniteKickState = kickFixture;
    nonFiniteKickState.cueBall.x = std::numeric_limits<double>::infinity();
    const auto nonFiniteKick = BilliardAlgorithm::generateKickPotCandidates(
        nonFiniteKickState,
        *kickFixtureTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    tests.expectTrue(
        nonFiniteKick.status() == KickPotGenerationStatus::InvalidStableState &&
        !nonFiniteKick.value(),
        "non-finite Kick table geometry fails closed");

    auto stalePocketGeometry = geometry;
    stalePocketGeometry.pockets[0].wirePocketCenter.x += 1.0;
    const auto stalePocket = BilliardAlgorithm::generateKickPotCandidates(
        kickFixture,
        *kickFixtureTarget.value(),
        stalePocketGeometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    tests.expectTrue(
        stalePocket.status() == KickPotGenerationStatus::InvalidGeometryConfiguration &&
        !stalePocket.value(),
        "static geometry cannot replace current-cycle wire pocket centers");

    const auto baselineKick = BilliardAlgorithm::generateKickPotCandidates(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    if (!baselineKick.value() || !baselineKick.value()->feasible[4][0]) {
        tests.expectTrue(false, "Kick angle fixture produces a baseline candidate");
        return tests.exitCode();
    }
    const auto samePocketDirect = BilliardAlgorithm::generateDirectPotCandidates(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry);
    tests.expectTrue(
        samePocketDirect.value() && samePocketDirect.value()->feasible[4],
        "same target/pocket fixture also exposes the Direct geometry");
    if (!samePocketDirect.value() || !samePocketDirect.value()->feasible[4]) {
        return tests.exitCode();
    }
    auto strictKickConfig = kickConfig();
    strictKickConfig.maxKickRailAngleDeg =
        baselineKick.value()->feasible[4][0]->incidenceAngleDeg - 0.1;
    const auto angleRejected = BilliardAlgorithm::generateKickPotCandidates(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{strictKickConfig});
    tests.expectTrue(
        angleRejected.value() && kickDiagnosticFor(*angleRejected.value(), 4, 0) &&
        kickDiagnosticFor(*angleRejected.value(), 4, 0)->reason ==
            KickPotRejectionReason::KickAngleRejected,
        "Kick rail angle above the Phase 1 threshold is diagnostic-only");

    auto exactKickConfig = kickConfig();
    exactKickConfig.maxKickRailAngleDeg =
        baselineKick.value()->feasible[4][0]->incidenceAngleDeg;
    const auto exactAngleAccepted = BilliardAlgorithm::generateKickPotCandidates(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{exactKickConfig});
    tests.expectTrue(
        exactAngleAccepted.value() && exactAngleAccepted.value()->feasible[4][0],
        "Kick rail angle exactly at the Phase 1 threshold remains feasible");

    auto blockedKickState = kickFixture;
    const auto& baselineCandidate = *baselineKick.value()->feasible[4][0];
    const auto& samePocketDirectCandidate = *samePocketDirect.value()->feasible[4];
    tests.expectTrue(
        baselineCandidate.targetPath.end.x == samePocketDirectCandidate.targetPath.end.x &&
        baselineCandidate.targetPath.end.y == samePocketDirectCandidate.targetPath.end.y &&
        baselineCandidate.virtualPocketTarget.x ==
            samePocketDirectCandidate.virtualPocketTarget.x &&
        baselineCandidate.virtualPocketTarget.y ==
            samePocketDirectCandidate.virtualPocketTarget.y,
        "Direct and Kick share the selected ResolvedPocketModel target semantics");
    tests.expectTrue(
        baselineCandidate.pocketId == samePocketDirectCandidate.pocketId &&
        baselineCandidate.targetPath.start.x == samePocketDirectCandidate.targetPath.start.x &&
        baselineCandidate.targetPath.start.y == samePocketDirectCandidate.targetPath.start.y &&
        baselineCandidate.pocketEntryAngleDeg ==
            samePocketDirectCandidate.pocketEntryAngleDeg,
        "Direct and Kick share pocket ID, target path and pocket-entry geometry");
    const auto directPocketPathCheck = BilliardPhysics::checkTargetPathToPocket(
        samePocketDirectCandidate.targetPath,
        samePocketDirectCandidate.pocketId,
        geometry.pockets,
        geometry.playableBallCenterRegion);
    const auto kickPocketPathCheck = BilliardPhysics::checkTargetPathToPocket(
        baselineCandidate.targetPath,
        baselineCandidate.pocketId,
        geometry.pockets,
        geometry.playableBallCenterRegion);
    tests.expectTrue(
        directPocketPathCheck.status() == GeometryStatus::Clear &&
        kickPocketPathCheck.status() == GeometryStatus::Clear,
        "Direct and Kick traverse the same PocketExitSegment and capture corridor");
    tests.expectTrue(
        baselineCandidate.ghostBallPoint.center.x ==
            samePocketDirectCandidate.ghostBallPoint.center.x &&
        baselineCandidate.ghostBallPoint.center.y ==
            samePocketDirectCandidate.ghostBallPoint.center.y &&
        (baselineCandidate.cuePathFirst.end.x !=
             samePocketDirectCandidate.cuePath.end.x ||
         baselineCandidate.cuePathFirst.end.y !=
             samePocketDirectCandidate.cuePath.end.y),
        "Kick changes only the pre-contact cue path while preserving Gpot");
    blockedKickState.objectBalls[1] = Point{
        (baselineCandidate.cuePathFirst.start.x + baselineCandidate.cuePathFirst.end.x) / 2.0,
        (baselineCandidate.cuePathFirst.start.y + baselineCandidate.cuePathFirst.end.y) / 2.0};
    const auto blockedKickTarget = selector.select(blockedKickState);
    if (!blockedKickTarget.value()) {
        tests.expectTrue(false, "blocked-first fixture has a selected target");
        return tests.exitCode();
    }
    const auto blockedKick = BilliardAlgorithm::generateKickPotCandidates(
        blockedKickState,
        *blockedKickTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    tests.expectTrue(
        blockedKick.value() && kickDiagnosticFor(*blockedKick.value(), 4, 0) &&
        kickDiagnosticFor(*blockedKick.value(), 4, 0)->reason ==
            KickPotRejectionReason::CueFirstSegmentBlocked,
        "blocked first Kick cue segment fails closed");

    auto blockedSecondState = kickFixture;
    blockedSecondState.objectBalls[1] = Point{
        (baselineCandidate.cuePathSecond.start.x +
         baselineCandidate.cuePathSecond.end.x) / 2.0,
        (baselineCandidate.cuePathSecond.start.y +
         baselineCandidate.cuePathSecond.end.y) / 2.0};
    const auto blockedSecondTarget = selector.select(blockedSecondState);
    if (!blockedSecondTarget.value()) {
        tests.expectTrue(false, "blocked-second fixture has a selected target");
        return tests.exitCode();
    }
    const auto blockedSecond = BilliardAlgorithm::generateKickPotCandidates(
        blockedSecondState,
        *blockedSecondTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    tests.expectTrue(
        blockedSecond.value() && kickDiagnosticFor(*blockedSecond.value(), 4, 0) &&
        kickDiagnosticFor(*blockedSecond.value(), 4, 0)->reason ==
            KickPotRejectionReason::CueSecondSegmentBlocked,
        "blocked second Kick cue segment fails closed");

    auto blockedKickTargetState = kickFixture;
    blockedKickTargetState.objectBalls[1] = Point{
        (baselineCandidate.targetPath.start.x + baselineCandidate.targetPath.end.x) / 2.0,
        (baselineCandidate.targetPath.start.y + baselineCandidate.targetPath.end.y) / 2.0};
    const auto blockedKickTargetSelection = selector.select(blockedKickTargetState);
    if (!blockedKickTargetSelection.value()) {
        tests.expectTrue(false, "blocked Kick target-path fixture has a selected target");
        return tests.exitCode();
    }
    const auto blockedKickTargetResult = BilliardAlgorithm::generateKickPotCandidates(
        blockedKickTargetState,
        *blockedKickTargetSelection.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    tests.expectTrue(
        blockedKickTargetResult.value() &&
        kickDiagnosticFor(*blockedKickTargetResult.value(), 4, 0) &&
        kickDiagnosticFor(*blockedKickTargetResult.value(), 4, 0)->reason ==
            KickPotRejectionReason::TargetPathBlocked,
        "blocked Kick target path fails closed independently");

    auto noIntersectionState = kickFixture;
    noIntersectionState.cueBall = {900.0, 100.0};
    const auto noIntersectionTarget = selector.select(noIntersectionState);
    if (!noIntersectionTarget.value()) {
        tests.expectTrue(false, "no-intersection fixture has a selected target");
        return tests.exitCode();
    }
    const auto noIntersection = BilliardAlgorithm::generateKickPotCandidates(
        noIntersectionState,
        *noIntersectionTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    tests.expectTrue(
        noIntersection.value() && kickDiagnosticFor(*noIntersection.value(), 4, 0) &&
        kickDiagnosticFor(*noIntersection.value(), 4, 0)->reason ==
            KickPotRejectionReason::NoRailIntersection,
        "ray missing the effective rail segment is diagnostic-only");

    const auto& bottomRail = geometry.railReflectionRegion.rails[0];
    const Point excludedPhysicalRailPoint{20.0, bottomRail.segment.start.y};
    const auto excludedRailState = kickStateForRebound(
        geometry,
        0,
        4,
        excludedPhysicalRailPoint,
        5.0);
    const auto excludedRailTarget = selector.select(excludedRailState);
    if (!excludedRailTarget.value()) {
        tests.expectTrue(false, "rail-exclusion fixture has a selected target");
        return tests.exitCode();
    }
    const auto excludedRail = BilliardAlgorithm::generateKickPotCandidates(
        excludedRailState,
        *excludedRailTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    tests.expectTrue(
        excludedRail.value() && kickDiagnosticFor(*excludedRail.value(), 4, 0) &&
        kickDiagnosticFor(*excludedRail.value(), 4, 0)->reason ==
            KickPotRejectionReason::NoRailIntersection,
        "physical-rail point inside the mapped endpoint exclusion is rejected");

    tests.expectTrue(
        BilliardPhysics::intersectRayWithEffectiveRail(
            {100.0, bottomRail.segment.start.y + 10.0},
            {200.0, bottomRail.segment.start.y + 10.0},
            bottomRail).status() == GeometryStatus::Parallel,
        "parallel Kick mirror ray fails closed with a named status");
    tests.expectTrue(
        BilliardPhysics::intersectRayWithEffectiveRail(
            {100.0, bottomRail.segment.start.y},
            {200.0, bottomRail.segment.start.y},
            bottomRail).status() == GeometryStatus::Coincident,
        "coincident Kick mirror ray fails closed with a named status");

    const auto repeatedKick = BilliardAlgorithm::generateKickPotCandidates(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    tests.expectTrue(
        repeatedKick.value() && repeatedKick.value()->feasible[4][0] &&
        repeatedKick.value()->feasible[4][0]->reboundPoint.x ==
            baselineCandidate.reboundPoint.x &&
        repeatedKick.value()->feasible[4][0]->reboundPoint.y ==
            baselineCandidate.reboundPoint.y &&
        repeatedKick.value()->feasible[4][0]->incidenceAngleDeg ==
            baselineCandidate.incidenceAngleDeg,
        "Kick geometry is deterministic without dynamics or fixed-force inputs");

    auto degenerateKickState = kickFixture;
    degenerateKickState.cueBall = *degenerateKickState.objectBalls[0];
    const auto degenerateKickTarget = selector.select(degenerateKickState);
    if (!degenerateKickTarget.value()) {
        tests.expectTrue(false, "degenerate Kick fixture has a selected target");
        return tests.exitCode();
    }
    const auto degenerateKick = BilliardAlgorithm::generateKickPotCandidates(
        degenerateKickState,
        *degenerateKickTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    tests.expectTrue(
        degenerateKick.value() && kickDiagnosticFor(*degenerateKick.value(), 4, 0) &&
        kickDiagnosticFor(*degenerateKick.value(), 4, 0)->reason ==
            KickPotRejectionReason::GhostGeometryInvalid,
        "overlapping Kick cue/target geometry fails closed without fallback");

    EligibleTarget kickForcedHigher{2, Point{700.0, 250.0}};
    auto kickMismatchState = kickFixture;
    kickMismatchState.objectBalls[1] = kickForcedHigher.center;
    const auto kickMismatch = BilliardAlgorithm::generateKickPotCandidates(
        kickMismatchState,
        kickForcedHigher,
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()});
    tests.expectTrue(
        kickMismatch.status() == KickPotGenerationStatus::SelectedTargetMismatch &&
        !kickMismatch.value(),
        "Kick orchestration refuses to reselect a higher-number target");

    return tests.exitCode();
}
