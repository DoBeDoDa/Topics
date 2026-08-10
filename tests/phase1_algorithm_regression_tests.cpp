#include "TestHarness.h"

#include "../src/Algorithm.h"
#include "../src/BilliardPhysics.h"
#include "../src/MathUtils.h"
#include "../src/TargetSelector.h"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

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

template <typename Payload, typename = void>
struct HasPocketId : std::false_type {};

template <typename Payload>
struct HasPocketId<Payload, std::void_t<decltype(std::declval<Payload>().pocketId)>>
    : std::true_type {};

template <typename Payload, typename = void>
struct HasPotScoring : std::false_type {};

template <typename Payload>
struct HasPotScoring<Payload, std::void_t<decltype(std::declval<Payload>().scoring)>>
    : std::true_type {};

const KickPotCandidateDiagnostic* kickDiagnosticFor(
    const KickPotEvaluation& evaluation,
    std::size_t pocketIndex,
    std::size_t railIndex)
{
    const auto& diagnostic = evaluation.rejected[pocketIndex][railIndex];
    return diagnostic ? &*diagnostic : nullptr;
}

BilliardConfig::ScoringConfig scoringConfig()
{
    return {
        BilliardConfig::INITIAL_EXPERIMENTAL_SCORING_WEIGHTS,
        1e-9,
        90.0,
        0.0,
        3000.0,
        200.0,
        1e-9,
        BilliardConfig::PlanningMode::PotOnly};
}

BilliardConfig::BrainConfig brainConfig()
{
    return {
        std::optional<std::string>{"base0-planar-test-v1"},
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()},
        std::optional<BilliardConfig::ScoringConfig>{scoringConfig()}};
}

DirectPotEvaluation keepOnlyDirect(
    DirectPotEvaluation evaluation,
    std::size_t keptPocket)
{
    for (std::size_t pocket = 0; pocket < evaluation.feasible.size(); ++pocket) {
        if (pocket == keptPocket) {
            continue;
        }
        evaluation.feasible[pocket].reset();
        if (!evaluation.rejected[pocket]) {
            evaluation.rejected[pocket] = DirectPotCandidateDiagnostic{
                static_cast<BilliardConfig::PocketId>(pocket),
                DirectPotRejectionReason::CuePathInvalid,
                GeometryStatus::DirectionRejected,
                std::nullopt};
        }
    }
    return evaluation;
}

KickPotEvaluation keepOnlyKick(
    KickPotEvaluation evaluation,
    std::size_t keptPocket,
    std::size_t keptRail)
{
    for (std::size_t pocket = 0; pocket < evaluation.feasible.size(); ++pocket) {
        for (std::size_t rail = 0; rail < evaluation.feasible[pocket].size(); ++rail) {
            if (pocket == keptPocket && rail == keptRail) {
                continue;
            }
            evaluation.feasible[pocket][rail].reset();
            if (!evaluation.rejected[pocket][rail]) {
                evaluation.rejected[pocket][rail] = KickPotCandidateDiagnostic{
                    static_cast<BilliardConfig::PocketId>(pocket),
                    static_cast<BilliardConfig::RailId>(rail),
                    KickPotRejectionReason::CueFirstSegmentInvalid,
                    GeometryStatus::DirectionRejected,
                    std::nullopt};
            }
        }
    }
    return evaluation;
}

DirectPotEvaluation noDirectCandidates(DirectPotEvaluation evaluation)
{
    const std::size_t noPocket = evaluation.feasible.size();
    return keepOnlyDirect(std::move(evaluation), noPocket);
}

KickPotEvaluation noKickCandidates(KickPotEvaluation evaluation)
{
    const std::size_t noPocket = evaluation.feasible.size();
    const std::size_t noRail = evaluation.feasible[0].size();
    return keepOnlyKick(
        std::move(evaluation),
        noPocket,
        noRail);
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

    const DirectPotEvaluation& fullDirect = *samePocketDirect.value();
    const KickPotEvaluation& fullKick = *baselineKick.value();
    const auto scoreConfig = scoringConfig();
    const auto kickGeometryConfig = kickConfig();

    const BilliardConfig::KickGeometryConfig directOnlyKickConfig{
        0.0,
        kickGeometryConfig.reflectionDirectionTolerance,
        kickGeometryConfig.reflectionAngleToleranceDeg};
    const auto directOnlyKick = BilliardAlgorithm::generateKickPotCandidates(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        std::optional<BilliardConfig::KickGeometryConfig>{directOnlyKickConfig});
    bool directOnlyHasKick = false;
    if (directOnlyKick.value()) {
        for (const auto& pocketCandidates : directOnlyKick.value()->feasible) {
            for (const auto& candidate : pocketCandidates) {
                directOnlyHasKick = directOnlyHasKick || candidate.has_value();
            }
        }
    }
    tests.expectTrue(
        directOnlyKick.value() && !directOnlyHasKick,
        "Direct-only fixture has no authoritative feasible Kick");
    if (!directOnlyKick.value() || directOnlyHasKick) {
        return tests.exitCode();
    }

    const auto selectedDirect = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        *directOnlyKick.value(),
        std::optional<BilliardConfig::ScoringConfig>{scoreConfig},
        std::optional<BilliardConfig::KickGeometryConfig>{directOnlyKickConfig});
    const auto* directWinner = selectedDirect.value()
        ? std::get_if<ScoredPotCandidate>(&*selectedDirect.value())
        : nullptr;
    tests.expectTrue(
        selectedDirect.status() == PotSelectionStatus::Success &&
        selectedDirect.isValid() && directWinner &&
        directWinner->kind == PotCandidateKind::Direct,
        "Direct-only feasible set selects the Direct candidate");

    std::optional<StableTableState> kickOnlyState;
    std::optional<EligibleTarget> kickOnlyTarget;
    std::optional<DirectPotEvaluation> kickOnlyDirect;
    std::optional<KickPotEvaluation> kickOnlyKick;
    for (std::size_t railIndex = 0;
         railIndex < geometry.railReflectionRegion.rails.size() && !kickOnlyState;
         ++railIndex) {
        for (std::size_t pocketIndex = 0;
             pocketIndex < geometry.pockets.size() && !kickOnlyState;
             ++pocketIndex) {
            StableTableState candidateState = alignedKickState(
                geometry,
                railIndex,
                pocketIndex);
            const auto candidateTarget = selector.select(candidateState);
            if (!candidateTarget.value()) continue;
            const auto candidateDirect = BilliardAlgorithm::generateDirectPotCandidates(
                candidateState,
                *candidateTarget.value(),
                geometry);
            if (!candidateDirect.value()) continue;
            std::size_t obstacleIndex = 1;
            for (const auto& candidate : candidateDirect.value()->feasible) {
                if (!candidate || obstacleIndex >= candidateState.objectBalls.size()) {
                    continue;
                }
                candidateState.objectBalls[obstacleIndex++] = Point{
                    (candidate->cuePath.start.x + candidate->cuePath.end.x) / 2.0,
                    (candidate->cuePath.start.y + candidate->cuePath.end.y) / 2.0};
            }
            const auto blockedFixtureTarget = selector.select(candidateState);
            if (!blockedFixtureTarget.value()) continue;
            const auto blockedDirect = BilliardAlgorithm::generateDirectPotCandidates(
                candidateState,
                *blockedFixtureTarget.value(),
                geometry);
            const auto survivingKick = BilliardAlgorithm::generateKickPotCandidates(
                candidateState,
                *blockedFixtureTarget.value(),
                geometry,
                std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
            if (!blockedDirect.value() || !survivingKick.value()) continue;
            bool hasDirect = false;
            bool hasKick = false;
            for (const auto& candidate : blockedDirect.value()->feasible) {
                hasDirect = hasDirect || candidate.has_value();
            }
            for (const auto& pocketCandidates : survivingKick.value()->feasible) {
                for (const auto& candidate : pocketCandidates) {
                    hasKick = hasKick || candidate.has_value();
                }
            }
            if (!hasDirect && hasKick) {
                kickOnlyState = candidateState;
                kickOnlyTarget = *blockedFixtureTarget.value();
                kickOnlyDirect = *blockedDirect.value();
                kickOnlyKick = *survivingKick.value();
            }
        }
    }
    tests.expectTrue(
        kickOnlyState && kickOnlyTarget && kickOnlyDirect && kickOnlyKick,
        "Kick-only fixture retains an authoritative Kick after all Directs are blocked");
    if (!kickOnlyState || !kickOnlyTarget || !kickOnlyDirect || !kickOnlyKick) {
        return tests.exitCode();
    }
    const auto selectedKick = BilliardAlgorithm::selectBestPot(
        *kickOnlyState,
        *kickOnlyTarget,
        geometry,
        *kickOnlyDirect,
        *kickOnlyKick,
        std::optional<BilliardConfig::ScoringConfig>{scoreConfig},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    const auto* kickWinner = selectedKick.value()
        ? std::get_if<ScoredPotCandidate>(&*selectedKick.value())
        : nullptr;
    tests.expectTrue(
        selectedKick.status() == PotSelectionStatus::Success &&
        kickWinner && kickWinner->kind == PotCandidateKind::Kick,
        "Kick-only feasible set selects the Kick candidate");

    if (directWinner) {
        tests.expectNear(directWinner->audit.rawWeights.kickPenalty, 0.30,
            TOLERANCE, "audit preserves raw kick weight");
        tests.expectNear(directWinner->audit.rawWeightSum, 1.0,
            TOLERANCE, "audit preserves raw weight sum");
        tests.expectNear(directWinner->audit.effectiveWeights.sum(), 1.0,
            scoreConfig.effectiveWeightSumTolerance,
            "effective scoring weights normalize to one");
        tests.expectTrue(
            directWinner->audit.rawCosts.kickPenalty == 0.0 &&
            directWinner->audit.normalizedCosts.kickPenalty == 0.0,
            "Direct soft preference uses zero kick penalty only");
        tests.expectNear(
            directWinner->audit.normalizedCosts.cuttingAngle,
            std::clamp(
                directWinner->audit.rawCosts.cuttingAngleDeg /
                    scoreConfig.maxCutAngleDeg,
                0.0,
                1.0),
            TOLERANCE,
            "cutting angle uses the approved normalized ratio");
        tests.expectNear(
            directWinner->audit.normalizedCosts.totalDistance,
            std::clamp(
                (directWinner->audit.rawCosts.totalDistanceMm -
                 scoreConfig.minDistanceMm) /
                    (scoreConfig.maxDistanceMm - scoreConfig.minDistanceMm),
                0.0,
                1.0),
            TOLERANCE,
            "total segmented distance uses the approved range formula");
        tests.expectNear(
            directWinner->audit.normalizedCosts.clearanceRisk,
            directWinner->audit.rawCosts.minimumClearanceMm
                ? 1.0 - std::clamp(
                    (*directWinner->audit.rawCosts.minimumClearanceMm -
                     (geometry.ballDiameterMm + geometry.collisionMarginMm)) /
                        (scoreConfig.preferredClearanceMm -
                         (geometry.ballDiameterMm + geometry.collisionMarginMm)),
                    0.0,
                    1.0)
                : 0.0,
            TOLERANCE,
            "clearance risk uses blocked and preferred clearance bounds");
        tests.expectNear(
            directWinner->audit.normalizedCosts.pocketEntryAngle,
            std::clamp(
                directWinner->audit.rawCosts.pocketEntryAngleDeg /
                    geometry.pockets[static_cast<std::size_t>(directWinner->pocketId)]
                        .maxEntryAngleDeg,
                0.0,
                1.0),
            TOLERANCE,
            "pocket entry uses the selected pocket class threshold");
        tests.expectTrue(
            directWinner->audit.normalizedCosts.kickRailAngleRisk == 0.0,
            "Direct kick-rail angle cost is explicitly zero");
        tests.expectNear(
            directWinner->audit.normalization.maxCutAngleDeg,
            scoreConfig.maxCutAngleDeg,
            TOLERANCE,
            "audit snapshots the cutting-angle normalization range");
        tests.expectNear(
            directWinner->audit.normalization.blockedClearanceThresholdMm,
            geometry.ballDiameterMm + geometry.collisionMarginMm,
            TOLERANCE,
            "audit snapshots the derived blocked-clearance threshold");
        tests.expectTrue(
            !directWinner->audit.normalization.maxKickRailAngleDeg,
            "Direct audit marks the Kick-only angle range not applicable");
    }
    if (kickWinner) {
        tests.expectTrue(
            kickWinner->audit.rawCosts.kickPenalty == 1.0 &&
            kickWinner->audit.normalizedCosts.kickPenalty == 1.0,
            "Kick candidate uses unit kick penalty");
        tests.expectNear(
            kickWinner->audit.normalizedCosts.kickRailAngleRisk,
            std::clamp(
                kickWinner->audit.rawCosts.kickRailAngleDeg /
                    kickGeometryConfig.maxKickRailAngleDeg,
                0.0,
                1.0),
            TOLERANCE,
            "Kick rail angle risk uses the approved Phase 1 angle range");
        tests.expectTrue(
            kickWinner->audit.normalization.maxKickRailAngleDeg &&
            *kickWinner->audit.normalization.maxKickRailAngleDeg ==
                kickGeometryConfig.maxKickRailAngleDeg,
            "Kick audit snapshots its rail-angle normalization range");
    }

    const auto baselineUnified = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{scoreConfig},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    const auto* baselineUnifiedWinner = baselineUnified.value()
        ? std::get_if<ScoredPotCandidate>(&*baselineUnified.value())
        : nullptr;
    tests.expectTrue(
        baselineUnifiedWinner && baselineUnifiedWinner->kind == PotCandidateKind::Direct,
        "Direct receives a strong but non-exclusive soft preference");
    const auto rankedUnified = BilliardAlgorithm::rankPotCandidates(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{scoreConfig},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    const bool rankOnePreserved = rankedUnified.value() &&
        !rankedUnified.value()->empty() && baselineUnifiedWinner &&
        rankedUnified.value()->front().kind == baselineUnifiedWinner->kind &&
        rankedUnified.value()->front().pocketId == baselineUnifiedWinner->pocketId &&
        rankedUnified.value()->front().railId == baselineUnifiedWinner->railId &&
        rankedUnified.value()->front().audit.totalCost ==
            baselineUnifiedWinner->audit.totalCost;
    tests.expectTrue(
        rankedUnified.isValid() && rankedUnified.value() &&
        rankedUnified.value()->size() > 1 && rankOnePreserved,
        "one-pass ranked Pot output preserves the old selectBestPot rank-one identity");

    bool validKickWon = false;
    auto qualityScoringConfig = scoreConfig;
    qualityScoringConfig.maxCutAngleDeg = 60.0;
    qualityScoringConfig.maxDistanceMm = 1.0e9;
    for (std::size_t railIndex = 0;
         railIndex < geometry.railReflectionRegion.rails.size() && !validKickWon;
         ++railIndex) {
        const auto& rail = geometry.railReflectionRegion.rails[railIndex].segment;
        for (std::size_t pocketIndex = 0;
             pocketIndex < geometry.pockets.size() && !validKickWon;
             ++pocketIndex) {
            const auto& pocket = geometry.pockets[pocketIndex];
            const Point target{
                pocket.wirePocketCenter.x - 100.0 * pocket.outwardUnitNormal.x,
                pocket.wirePocketCenter.y - 100.0 * pocket.outwardUnitNormal.y};
            const Point ghost{
                target.x - 2.0 * geometry.ballRadiusMm * pocket.outwardUnitNormal.x,
                target.y - 2.0 * geometry.ballRadiusMm * pocket.outwardUnitNormal.y};
            const Vector2D railDirection{
                rail.end.x - rail.start.x,
                rail.end.y - rail.start.y};
            const double railLengthSquared =
                railDirection.x * railDirection.x +
                railDirection.y * railDirection.y;
            const double projection = std::clamp(
                ((ghost.x - rail.start.x) * railDirection.x +
                 (ghost.y - rail.start.y) * railDirection.y) /
                    railLengthSquared,
                0.0,
                1.0);
            std::vector<Point> reboundPoints;
            for (int step = 1; step < 20; ++step) {
                const double fraction = static_cast<double>(step) / 20.0;
                reboundPoints.push_back({
                    rail.start.x + fraction * railDirection.x,
                    rail.start.y + fraction * railDirection.y});
            }
            reboundPoints.push_back({
                rail.start.x + projection * railDirection.x,
                rail.start.y + projection * railDirection.y});
            for (const Point rebound : reboundPoints) {
                const StableTableState qualityState = kickStateForRebound(
                    geometry,
                    railIndex,
                    pocketIndex,
                    rebound);
                const auto qualityTarget = selector.select(qualityState);
                if (!qualityTarget.value()) continue;
                const auto unblockedDirect = BilliardAlgorithm::generateDirectPotCandidates(
                    qualityState,
                    *qualityTarget.value(),
                    geometry);
                if (!unblockedDirect.value()) continue;
                for (std::size_t keptDirectPocket = 0;
                     keptDirectPocket < unblockedDirect.value()->feasible.size();
                     ++keptDirectPocket) {
                    if (!unblockedDirect.value()->feasible[keptDirectPocket]) continue;
                    StableTableState constrainedState = qualityState;
                    std::size_t obstacleIndex = 1;
                    for (std::size_t directPocket = 0;
                         directPocket < unblockedDirect.value()->feasible.size() &&
                         obstacleIndex < constrainedState.objectBalls.size();
                         ++directPocket) {
                        if (directPocket == keptDirectPocket ||
                            !unblockedDirect.value()->feasible[directPocket]) {
                            continue;
                        }
                        const Segment2D path =
                            unblockedDirect.value()->feasible[directPocket]->cuePath;
                        constrainedState.objectBalls[obstacleIndex++] = Point{
                            (path.start.x + path.end.x) / 2.0,
                            (path.start.y + path.end.y) / 2.0};
                    }
                    if (obstacleIndex < constrainedState.objectBalls.size()) {
                        const Segment2D keptPath =
                            unblockedDirect.value()->feasible[keptDirectPocket]->cuePath;
                        const Vector2D pathDirection{
                            keptPath.end.x - keptPath.start.x,
                            keptPath.end.y - keptPath.start.y};
                        const double pathLength = std::hypot(
                            pathDirection.x,
                            pathDirection.y);
                        const double nearClearance = geometry.ballDiameterMm +
                            geometry.collisionMarginMm + 0.1;
                        constrainedState.objectBalls[obstacleIndex] = Point{
                            (keptPath.start.x + keptPath.end.x) / 2.0 -
                                pathDirection.y / pathLength * nearClearance,
                            (keptPath.start.y + keptPath.end.y) / 2.0 +
                                pathDirection.x / pathLength * nearClearance};
                    }
                    const auto constrainedTarget = selector.select(constrainedState);
                    if (!constrainedTarget.value()) continue;
                    const auto qualityDirect =
                        BilliardAlgorithm::generateDirectPotCandidates(
                            constrainedState,
                            *constrainedTarget.value(),
                            geometry);
                    const auto qualityKick =
                        BilliardAlgorithm::generateKickPotCandidates(
                            constrainedState,
                            *constrainedTarget.value(),
                            geometry,
                            std::optional<BilliardConfig::KickGeometryConfig>{
                                kickGeometryConfig});
                    if (!qualityDirect.value() || !qualityKick.value()) continue;
                    bool hasDirect = false;
                    bool hasKick = false;
                    for (const auto& candidate : qualityDirect.value()->feasible) {
                        hasDirect = hasDirect || candidate.has_value();
                    }
                    for (const auto& pocketCandidates : qualityKick.value()->feasible) {
                        for (const auto& candidate : pocketCandidates) {
                            hasKick = hasKick || candidate.has_value();
                        }
                    }
                    if (!hasDirect || !hasKick) continue;
                    const auto selection = BilliardAlgorithm::selectBestPot(
                        constrainedState,
                        *constrainedTarget.value(),
                        geometry,
                        *qualityDirect.value(),
                        *qualityKick.value(),
                        std::optional<BilliardConfig::ScoringConfig>{qualityScoringConfig},
                        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
                    const auto* qualityWinner = selection.value()
                        ? std::get_if<ScoredPotCandidate>(&*selection.value())
                        : nullptr;
                    if (qualityWinner && qualityWinner->kind == PotCandidateKind::Kick) {
                        validKickWon = true;
                        break;
                    }
                }
            }
        }
    }
    tests.expectTrue(
        validKickWon,
        "a genuinely feasible higher-quality Kick can beat the Direct soft preference");

    auto tieConfig = scoreConfig;
    tieConfig.tieEpsilon = 1.0;
    const auto deterministicTie = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{tieConfig},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    const auto* tieWinner = deterministicTie.value()
        ? std::get_if<ScoredPotCandidate>(&*deterministicTie.value())
        : nullptr;
    tests.expectTrue(
        tieWinner && tieWinner->kind == PotCandidateKind::Direct,
        "deterministic tie-break chooses fewer kicks within epsilon");

#ifdef BILLIARDS_P1_08_TEST_SEAM
    if (!directWinner || !kickWinner) {
        tests.expectTrue(false, "tie-break seam fixtures require valid Direct and Kick bases");
        return tests.exitCode();
    }
    const auto setTieFields = [](
        ScoredPotCandidate candidate,
        PotCandidateKind kind,
        double cuttingAngleDeg,
        std::optional<double> clearanceMm,
        double distanceMm,
        BilliardConfig::PocketId pocketId,
        std::optional<BilliardConfig::RailId> railId) {
        candidate.kind = kind;
        candidate.audit.rawCosts.cuttingAngleDeg = cuttingAngleDeg;
        candidate.audit.rawCosts.minimumClearanceMm = clearanceMm;
        candidate.audit.rawCosts.totalDistanceMm = distanceMm;
        candidate.pocketId = pocketId;
        candidate.railId = railId;
        return candidate;
    };
    const auto productionComparatorChooses = [](
        const ScoredPotCandidate& expected,
        const ScoredPotCandidate& other) {
        return BilliardAlgorithm::tieBreakBetterForTest(expected, other) &&
            !BilliardAlgorithm::tieBreakBetterForTest(other, expected);
    };

    const auto directTieBase = setTieFields(
        *directWinner,
        PotCandidateKind::Direct,
        20.0,
        100.0,
        500.0,
        BilliardConfig::PocketId::Pocket3,
        std::nullopt);
    const auto kickTieBase = setTieFields(
        *kickWinner,
        PotCandidateKind::Kick,
        20.0,
        100.0,
        500.0,
        BilliardConfig::PocketId::Pocket3,
        BilliardConfig::RailId::Rail3);
    tests.expectTrue(
        productionComparatorChooses(directTieBase, kickTieBase),
        "tie-break layer 1 prefers fewer Kick segments");

    const auto largerCut = setTieFields(
        directTieBase,
        PotCandidateKind::Direct,
        30.0,
        100.0,
        500.0,
        BilliardConfig::PocketId::Pocket3,
        std::nullopt);
    tests.expectTrue(
        productionComparatorChooses(directTieBase, largerCut),
        "tie-break layer 2 prefers the smaller cutting angle");

    const auto tighterClearance = setTieFields(
        directTieBase,
        PotCandidateKind::Direct,
        20.0,
        60.0,
        500.0,
        BilliardConfig::PocketId::Pocket3,
        std::nullopt);
    tests.expectTrue(
        productionComparatorChooses(directTieBase, tighterClearance),
        "tie-break layer 3 prefers the larger minimum clearance");

    const auto longerPath = setTieFields(
        directTieBase,
        PotCandidateKind::Direct,
        20.0,
        100.0,
        700.0,
        BilliardConfig::PocketId::Pocket3,
        std::nullopt);
    tests.expectTrue(
        productionComparatorChooses(directTieBase, longerPath),
        "tie-break layer 4 prefers the shorter total path");

    const auto lowerPocket = setTieFields(
        directTieBase,
        PotCandidateKind::Direct,
        20.0,
        100.0,
        500.0,
        BilliardConfig::PocketId::Pocket2,
        std::nullopt);
    const auto higherPocket = setTieFields(
        directTieBase,
        PotCandidateKind::Direct,
        20.0,
        100.0,
        500.0,
        BilliardConfig::PocketId::Pocket5,
        std::nullopt);
    tests.expectTrue(
        productionComparatorChooses(lowerPocket, higherPocket),
        "tie-break layer 5 prefers the lower Pocket ID");

    const auto lowerRail = setTieFields(
        kickTieBase,
        PotCandidateKind::Kick,
        20.0,
        100.0,
        500.0,
        BilliardConfig::PocketId::Pocket3,
        BilliardConfig::RailId::Rail2);
    const auto higherRail = setTieFields(
        kickTieBase,
        PotCandidateKind::Kick,
        20.0,
        100.0,
        500.0,
        BilliardConfig::PocketId::Pocket3,
        BilliardConfig::RailId::Rail5);
    tests.expectTrue(
        productionComparatorChooses(lowerRail, higherRail),
        "tie-break layer 6 prefers the lower Rail ID");

    const std::array<ScoredPotCandidate, 6> tournamentCandidates{{
        setTieFields(directTieBase, PotCandidateKind::Direct, 5.0, 100.0, 300.0,
            BilliardConfig::PocketId::Pocket1, std::nullopt),
        setTieFields(directTieBase, PotCandidateKind::Direct, 10.0, 100.0, 300.0,
            BilliardConfig::PocketId::Pocket1, std::nullopt),
        setTieFields(directTieBase, PotCandidateKind::Direct, 5.0, 80.0, 300.0,
            BilliardConfig::PocketId::Pocket1, std::nullopt),
        setTieFields(directTieBase, PotCandidateKind::Direct, 5.0, 100.0, 400.0,
            BilliardConfig::PocketId::Pocket1, std::nullopt),
        setTieFields(directTieBase, PotCandidateKind::Direct, 5.0, 100.0, 300.0,
            BilliardConfig::PocketId::Pocket2, std::nullopt),
        setTieFields(kickTieBase, PotCandidateKind::Kick, 0.0, 1000.0, 1.0,
            BilliardConfig::PocketId::Pocket1, BilliardConfig::RailId::Rail1)}};
    std::array<std::size_t, 6> tournamentOrder{{0, 1, 2, 3, 4, 5}};
    bool sameTournamentWinner = true;
    for (int permutation = 0; permutation < 20; ++permutation) {
        std::size_t winnerIndex = tournamentOrder[0];
        for (std::size_t position = 1; position < tournamentOrder.size(); ++position) {
            const std::size_t challengerIndex = tournamentOrder[position];
            if (BilliardAlgorithm::tieBreakBetterForTest(
                    tournamentCandidates[challengerIndex],
                    tournamentCandidates[winnerIndex])) {
                winnerIndex = challengerIndex;
            }
        }
        sameTournamentWinner = sameTournamentWinner && winnerIndex == 0;
        std::next_permutation(tournamentOrder.begin(), tournamentOrder.end());
    }
    tests.expectTrue(
        sameTournamentWinner,
        "the production comparator selects one identity across 20 tournament orders");
#endif

    std::array<std::optional<Point>, 9> noPotBalls{};
    noPotBalls[0] = Point{500.0, 250.0};
    const StableTableState noPotState = state({500.0, 250.0}, noPotBalls);
    const auto noPotTarget = selector.select(noPotState);
    const auto noPotDirect = noPotTarget.value()
        ? BilliardAlgorithm::generateDirectPotCandidates(
            noPotState,
            *noPotTarget.value(),
            geometry)
        : DirectPotGenerationResult::rejected(
            DirectPotGenerationStatus::InvalidStableState);
    const auto noPotKick = noPotTarget.value()
        ? BilliardAlgorithm::generateKickPotCandidates(
            noPotState,
            *noPotTarget.value(),
            geometry,
            std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig})
        : KickPotGenerationResult::rejected(
            KickPotGenerationStatus::InvalidStableState);
    tests.expectTrue(
        noPotTarget.value() && noPotDirect.value() && noPotKick.value(),
        "NoPot fixture produces authoritative candidate evaluations");
    if (!noPotTarget.value() || !noPotDirect.value() || !noPotKick.value()) {
        return tests.exitCode();
    }
    const auto noPot = BilliardAlgorithm::selectBestPot(
        noPotState,
        *noPotTarget.value(),
        geometry,
        *noPotDirect.value(),
        *noPotKick.value(),
        std::optional<BilliardConfig::ScoringConfig>{scoreConfig},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    const auto* noPlan = noPot.value()
        ? std::get_if<PotOnlyNoPlan>(&*noPot.value())
        : nullptr;
    tests.expectTrue(
        noPot.status() == PotSelectionStatus::Success && noPlan &&
        noPlan->reason == PotNoPlanReason::NoPotCandidate &&
        noPlan->feasiblePotCount == 0 &&
        !noPlan->proceededToLegalContact &&
        !noPlan->directDiagnostics.empty() &&
        !noPlan->kickDiagnostics.empty() && noPot.isValid(),
        "PotOnly empty candidate set returns NoPlan without LegalContact fallback");

    const auto p109Config = brainConfig();
    const auto directPlanning = BilliardAlgorithm::planShot(
        kickFixture,
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        p109Config);
    const auto* directPlan = std::get_if<ShotPlan>(&directPlanning.value());
    const auto* directPayload = directPlan
        ? std::get_if<DirectPotShotPlanPayload>(&directPlan->payload)
        : nullptr;
    tests.expectTrue(
        directPlanning.isValid() && directPlan && directPayload &&
        baselineUnifiedWinner &&
        directPlan->type == ShotPlanType::DirectPot &&
        directPlan->source.planIdentity.connectionIdentity ==
            kickFixture.connectionIdentity &&
        directPlan->source.planIdentity.shotCycleIdentity ==
            kickFixture.shotCycleIdentity &&
        directPlan->source.sourceEvents[0].eventId ==
            kickFixture.sourceEvents[0].eventId &&
        directPlan->source.sourceEvents[2].eventId ==
            kickFixture.sourceEvents[2].eventId &&
        directPlan->source.base0PlanarCalibrationRevision ==
            *p109Config.base0PlanarCalibrationRevision &&
        directPlan->source.tableGeometryRevision == geometry.calibrationRevision &&
        directPlan->selectedTarget.ballNumber == baselineUnifiedWinner->target.ballNumber &&
        directPayload->candidate.pocketId == baselineUnifiedWinner->pocketId &&
        directPayload->scoring.totalCost == baselineUnifiedWinner->audit.totalCost &&
        directPlan->cuePathSegments.size() == 1 &&
        directPlan->forceMode == FixedForceMode::Fixed,
        "P1-09 preserves the P1-08 Direct winner and complete Phase 1 audit identity");
    if (directPlan) {
        ShotPlan perpendicularDirection = *directPlan;
        perpendicularDirection.shotDirectionXY = {
            -directPlan->shotDirectionXY.y,
            directPlan->shotDirectionXY.x};
        ShotPlan reverseDirection = *directPlan;
        reverseDirection.shotDirectionXY = {
            -directPlan->shotDirectionXY.x,
            -directPlan->shotDirectionXY.y};
        ShotPlan nonFiniteDirection = *directPlan;
        nonFiniteDirection.shotDirectionXY.x =
            std::numeric_limits<double>::quiet_NaN();
        tests.expectTrue(directPlan->isValid(),
            "ShotPlan accepts the normalized first cue-path direction");
        tests.expectFalse(perpendicularDirection.isValid(),
            "ShotPlan rejects a 90-degree direction mismatch");
        tests.expectFalse(reverseDirection.isValid(),
            "ShotPlan rejects a reversed direction");
        tests.expectFalse(nonFiniteDirection.isValid(),
            "ShotPlan rejects a non-finite direction");
    }

    const auto kickPlanning = BilliardAlgorithm::planShot(
        *kickOnlyState,
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        p109Config);
    const auto* kickPlan = std::get_if<ShotPlan>(&kickPlanning.value());
    const auto* kickPayload = kickPlan
        ? std::get_if<KickPotShotPlanPayload>(&kickPlan->payload)
        : nullptr;
    tests.expectTrue(
        kickPlanning.isValid() && kickPlan && kickPayload && kickWinner &&
        kickPlan->type == ShotPlanType::KickPot &&
        kickPayload->candidate.pocketId == kickWinner->pocketId &&
        kickPayload->candidate.railId == *kickWinner->railId &&
        kickPayload->scoring.totalCost == kickWinner->audit.totalCost &&
        kickPayload->kickGeometry.maxKickRailAngleDeg ==
            p109Config.kickGeometry->maxKickRailAngleDeg &&
        kickPayload->kickGeometry.reflectionDirectionTolerance ==
            p109Config.kickGeometry->reflectionDirectionTolerance &&
        kickPayload->kickGeometry.reflectionAngleToleranceDeg ==
            p109Config.kickGeometry->reflectionAngleToleranceDeg &&
        kickPlan->cuePathSegments.size() == 2 &&
        kickPlan->cuePathSegments[0].end.x ==
            kickPayload->candidate.reboundPoint.x &&
        kickPlan->cuePathSegments[0].end.y ==
            kickPayload->candidate.reboundPoint.y,
        "P1-09 preserves the P1-08 Kick pocket, rail, rebound, and score identity");
    if (kickPlan) {
        ShotPlan perpendicularDirection = *kickPlan;
        perpendicularDirection.shotDirectionXY = {
            -kickPlan->shotDirectionXY.y,
            kickPlan->shotDirectionXY.x};
        ShotPlan reverseDirection = *kickPlan;
        reverseDirection.shotDirectionXY = {
            -kickPlan->shotDirectionXY.x,
            -kickPlan->shotDirectionXY.y};
        ShotPlan nonFiniteDirection = *kickPlan;
        nonFiniteDirection.shotDirectionXY.y =
            std::numeric_limits<double>::infinity();
        tests.expectTrue(kickPlan->isValid(),
            "Kick ShotPlan direction follows C-to-rebound first cue segment");
        tests.expectFalse(perpendicularDirection.isValid(),
            "Kick ShotPlan rejects a 90-degree first-segment mismatch");
        tests.expectFalse(reverseDirection.isValid(),
            "Kick ShotPlan rejects a reversed first-segment direction");
        tests.expectFalse(nonFiniteDirection.isValid(),
            "Kick ShotPlan rejects a non-finite direction");
    }

    const auto noPotPlanning = BilliardAlgorithm::planShot(
        noPotState,
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        p109Config);
    const auto* integratedNoPot = std::get_if<NoPlan>(&noPotPlanning.value());
    tests.expectTrue(
        noPotPlanning.isValid() && integratedNoPot &&
        integratedNoPot->reason == NoPlanReason::NoPotCandidate &&
        integratedNoPot->selectedTarget &&
        integratedNoPot->feasiblePotCount == 0 &&
        !integratedNoPot->proceededToLegalContact &&
        !integratedNoPot->directCandidateDiagnostics.empty() &&
        !integratedNoPot->kickCandidateDiagnostics.empty(),
        "P1-09 maps PotOnly exhaustion to auditable NoPotCandidate without fallback");

    std::array<std::optional<Point>, 9> manualBalls{};
    manualBalls[0] = Point{500.0, 250.0};
    for (std::size_t pocket = 0; pocket < geometry.pockets.size(); ++pocket) {
        const Point virtualTarget = geometry.pockets[pocket].virtualPocketTarget;
        manualBalls[pocket + 1] = Point{
            manualBalls[0]->x + 0.75 * (virtualTarget.x - manualBalls[0]->x),
            manualBalls[0]->y + 0.75 * (virtualTarget.y - manualBalls[0]->y)};
    }
    const StableTableState manualDirectState = state({400.0, 250.0}, manualBalls);
    const auto potOnlyBlocked = BilliardAlgorithm::planShot(
        manualDirectState,
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        p109Config);
    const auto* blockedPotOnlyNoPlan = std::get_if<NoPlan>(&potOnlyBlocked.value());
    tests.expectTrue(
        potOnlyBlocked.isValid() && blockedPotOnlyNoPlan &&
        blockedPotOnlyNoPlan->reason == NoPlanReason::NoPotCandidate &&
        !blockedPotOnlyNoPlan->proceededToLegalContact &&
        !potOnlyBlocked.executionCandidates().legalContactPlans.empty(),
        "PotOnly keeps its NoPlan outcome while precomputing production LegalContact candidates");

    auto manualResearchConfig = p109Config;
    manualResearchConfig.scoring->planningMode =
        BilliardConfig::PlanningMode::ManualResearch;
    const auto manualWithPotPlanning = BilliardAlgorithm::planShot(
        kickFixture,
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        manualResearchConfig);
    const auto* manualWithPotPlan =
        std::get_if<ShotPlan>(&manualWithPotPlanning.value());
    tests.expectTrue(
        manualWithPotPlanning.isValid() && manualWithPotPlan &&
        (manualWithPotPlan->type == ShotPlanType::DirectPot ||
         manualWithPotPlan->type == ShotPlanType::KickPot),
        "ManualResearch preserves a feasible Pot and does not enter LegalContact");
    const auto directLegalPlanning = BilliardAlgorithm::planShot(
        manualDirectState,
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        manualResearchConfig);
    const auto* directLegalPlan =
        std::get_if<ShotPlan>(&directLegalPlanning.value());
    const auto* directLegalPayload = directLegalPlan
        ? std::get_if<DirectLegalContactShotPlanPayload>(&directLegalPlan->payload)
        : nullptr;
    tests.expectTrue(
        directLegalPlanning.isValid() && directLegalPlan && directLegalPayload &&
        directLegalPlan->type == ShotPlanType::DirectLegalContact &&
        directLegalPayload->audit.legalFirstContactGuaranteed &&
        directLegalPayload->audit.directPriorityApplied &&
        directLegalPayload->audit.potSearchStatus ==
            PotSelectionStatus::ManualResearchPotSearchExhausted &&
        directLegalPayload->audit.potSearchExhausted &&
        directLegalPayload->audit.requiresExplicitExecutionAuthorization &&
        !directLegalPayload->audit.realHardwareExecutionDefaultEnabled &&
        directLegalPlan->cuePathSegments.size() == 1 &&
        std::fabs(std::hypot(
            directLegalPlan->selectedTarget.center.x -
                directLegalPlan->ghostBallPoint.center.x,
            directLegalPlan->selectedTarget.center.y -
                directLegalPlan->ghostBallPoint.center.y) -
            2.0 * geometry.ballRadiusMm) <= TOLERANCE,
        "ManualResearch enters DirectLegalContact only after Pot exhaustion");
    static_assert(!HasPocketId<DirectLegalContactShotPlanPayload>::value,
        "LegalContact payload must not expose a pocket ID");
    static_assert(!HasPotScoring<DirectLegalContactShotPlanPayload>::value,
        "LegalContact payload must not expose Pot scoring");

    StableTableState manualKickState = manualDirectState;
    manualKickState.objectBalls[7] = Point{440.0, 250.0};
    const auto kickLegalPlanning = BilliardAlgorithm::planShot(
        manualKickState,
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        manualResearchConfig);
    const auto* kickLegalPlan = std::get_if<ShotPlan>(&kickLegalPlanning.value());
    const auto* kickLegalPayload = kickLegalPlan
        ? std::get_if<KickLegalContactShotPlanPayload>(&kickLegalPlan->payload)
        : nullptr;
    tests.expectTrue(
        kickLegalPlanning.isValid() && kickLegalPlan && kickLegalPayload &&
        kickLegalPlan->type == ShotPlanType::KickLegalContact &&
        kickLegalPlan->cuePathSegments.size() == 2 &&
        kickLegalPayload->candidate.cuePathFirst.end.x ==
            kickLegalPayload->candidate.reboundPoint.x &&
        kickLegalPayload->candidate.cuePathFirst.end.y ==
            kickLegalPayload->candidate.reboundPoint.y &&
        kickLegalPayload->candidate.cuePathSecond.end.x ==
            kickLegalPayload->candidate.ghostBallPoint.center.x &&
        kickLegalPayload->candidate.cuePathSecond.end.y ==
            kickLegalPayload->candidate.ghostBallPoint.center.y &&
        kickLegalPayload->audit.railId == kickLegalPayload->candidate.railId &&
        kickLegalPayload->audit.requiresExplicitExecutionAuthorization &&
        !kickLegalPayload->audit.realHardwareExecutionDefaultEnabled,
        "ManualResearch selects a one-rail cue-ball KickLegalContact without Pot fields");
    static_assert(!HasPocketId<KickLegalContactShotPlanPayload>::value,
        "KickLegalContact payload must not expose a pocket ID");
    static_assert(!HasPotScoring<KickLegalContactShotPlanPayload>::value,
        "KickLegalContact payload must not expose Pot scoring");

    std::optional<KickLegalContactCandidate> legalKickFixture;
    for (std::size_t railIndex = 0;
         railIndex < geometry.railReflectionRegion.rails.size();
         ++railIndex) {
        const auto legalState = alignedKickState(
            geometry,
            railIndex,
            oppositePocketForRail[railIndex]);
        const auto legalTarget = selector.select(legalState);
        tests.expectTrue(legalTarget.value().has_value(),
            "LegalContact rail fixture preserves the lowest target");
        if (!legalTarget.value()) {
            continue;
        }
        const auto evaluation = BilliardAlgorithm::generateLegalContactForTest(
            legalState,
            *legalTarget.value(),
            geometry,
            kickConfig());
        const auto& candidate = evaluation.kicks[railIndex];
        if (evaluation.direct && candidate) {
            tests.expectTrue(evaluation.selectedDirect,
                "LegalContact selection applies Direct priority before Kick tie-breaks");
        }
        tests.expectTrue(candidate.has_value(),
            "each effective rail independently produces at most one observable KickLegalContact");
        if (!candidate) {
            continue;
        }
        if (!legalKickFixture) {
            legalKickFixture = *candidate;
        }
        const auto& rail = geometry.railReflectionRegion.rails[railIndex];
        const auto mirrored = BilliardPhysics::mirrorPointAcrossEffectiveRail(
            legalState.cueBall,
            rail);
        const auto directionRaw = mirrored.value()
            ? BilliardMath::getVector(*mirrored.value(), legalTarget.value()->center)
            : std::nullopt;
        const auto direction = directionRaw
            ? BilliardMath::normalize(*directionRaw)
            : std::nullopt;
        tests.expectTrue(mirrored.value().has_value() && direction.has_value(),
            "KickLegalContact fixture has finite Cmirror and vKickContact");
        if (!mirrored.value() || !direction) {
            continue;
        }
        const Point expectedGhost{
            legalTarget.value()->center.x -
                2.0 * geometry.ballRadiusMm * direction->x,
            legalTarget.value()->center.y -
                2.0 * geometry.ballRadiusMm * direction->y};
        const auto expectedRebound = BilliardPhysics::intersectRayWithEffectiveRail(
            *mirrored.value(),
            expectedGhost,
            rail);
        tests.expectNear(candidate->ghostBallPoint.center.x,
            expectedGhost.x, TOLERANCE,
            "KickLegalContact G X follows the approved mirror formula");
        tests.expectNear(candidate->ghostBallPoint.center.y,
            expectedGhost.y, TOLERANCE,
            "KickLegalContact G Y follows the approved mirror formula");
        tests.expectTrue(expectedRebound.value().has_value(),
            "KickLegalContact has one valid effective-rail intersection");
        if (expectedRebound.value()) {
            tests.expectNear(candidate->reboundPoint.x,
                expectedRebound.value()->x, TOLERANCE,
                "KickLegalContact rebound X is the unique mirror intersection");
            tests.expectNear(candidate->reboundPoint.y,
                expectedRebound.value()->y, TOLERANCE,
                "KickLegalContact rebound Y is the unique mirror intersection");
        }
        tests.expectTrue(
            candidate->cuePathFirst.start.x == legalState.cueBall.x &&
            candidate->cuePathFirst.start.y == legalState.cueBall.y &&
            candidate->cuePathFirst.end.x == candidate->reboundPoint.x &&
            candidate->cuePathFirst.end.y == candidate->reboundPoint.y &&
            candidate->cuePathSecond.start.x == candidate->reboundPoint.x &&
            candidate->cuePathSecond.start.y == candidate->reboundPoint.y &&
            candidate->cuePathSecond.end.x == candidate->ghostBallPoint.center.x &&
            candidate->cuePathSecond.end.y == candidate->ghostBallPoint.center.y,
            "KickLegalContact path is exactly C to R to G for every effective rail");
        tests.expectNear(std::hypot(
            candidate->target.center.x - candidate->ghostBallPoint.center.x,
            candidate->target.center.y - candidate->ghostBallPoint.center.y),
            2.0 * geometry.ballRadiusMm,
            TOLERANCE,
            "KickLegalContact G remains exactly 2r from the target");
    }

    if (legalKickFixture) {
        auto higherClearance = *legalKickFixture;
        auto lowerClearance = *legalKickFixture;
        higherClearance.minimumClearanceMm = 50.0;
        lowerClearance.minimumClearanceMm = 40.0;
        tests.expectTrue(BilliardAlgorithm::legalKickBetterForTest(
            higherClearance, lowerClearance),
            "LegalContact Kick tie-break prefers greater minimum clearance");

        auto shorter = *legalKickFixture;
        auto longer = *legalKickFixture;
        shorter.minimumClearanceMm = 40.0;
        longer.minimumClearanceMm = 40.0;
        shorter.totalPathLengthMm = 100.0;
        longer.totalPathLengthMm = 110.0;
        tests.expectTrue(BilliardAlgorithm::legalKickBetterForTest(shorter, longer),
            "LegalContact Kick tie-break next prefers shorter total path");

        auto lowerRailCandidate = *legalKickFixture;
        auto higherRailCandidate = *legalKickFixture;
        lowerRailCandidate.minimumClearanceMm = 40.0;
        higherRailCandidate.minimumClearanceMm = 40.0;
        lowerRailCandidate.totalPathLengthMm = 100.0;
        higherRailCandidate.totalPathLengthMm = 100.0;
        lowerRailCandidate.railId = BilliardConfig::RailId::Rail1;
        higherRailCandidate.railId = BilliardConfig::RailId::Rail2;
        tests.expectTrue(BilliardAlgorithm::legalKickBetterForTest(
            lowerRailCandidate, higherRailCandidate),
            "LegalContact Kick final tie-break prefers lower rail ID");
    }

    if (kickLegalPlan && kickLegalPayload) {
        ShotPlan targetEndpoint = *kickLegalPlan;
        auto& targetPayload =
            std::get<KickLegalContactShotPlanPayload>(targetEndpoint.payload);
        targetEndpoint.ghostBallPoint.center = targetEndpoint.selectedTarget.center;
        targetEndpoint.cuePathSegments.back().end =
            targetEndpoint.selectedTarget.center;
        targetPayload.candidate.ghostBallPoint = targetEndpoint.ghostBallPoint;
        targetPayload.candidate.cuePathSecond.end =
            targetEndpoint.selectedTarget.center;
        targetPayload.audit.selectedContactGhostBallPoint =
            targetEndpoint.ghostBallPoint;
        tests.expectFalse(targetEndpoint.isValid(),
            "LegalContact rejects target center as a GhostBallPoint endpoint");

        ShotPlan surfaceEndpoint = *kickLegalPlan;
        auto& surfacePayload =
            std::get<KickLegalContactShotPlanPayload>(surfaceEndpoint.payload);
        const Point surfaceContact{
            (surfaceEndpoint.selectedTarget.center.x +
                surfaceEndpoint.ghostBallPoint.center.x) / 2.0,
            (surfaceEndpoint.selectedTarget.center.y +
                surfaceEndpoint.ghostBallPoint.center.y) / 2.0};
        surfaceEndpoint.ghostBallPoint.center = surfaceContact;
        surfaceEndpoint.cuePathSegments.back().end = surfaceContact;
        surfacePayload.candidate.ghostBallPoint = {surfaceContact};
        surfacePayload.candidate.cuePathSecond.end = surfaceContact;
        surfacePayload.audit.selectedContactGhostBallPoint = {surfaceContact};
        tests.expectFalse(surfaceEndpoint.isValid(),
            "LegalContact rejects BallSurfaceContactPoint as a cue-center endpoint");
    }

    const auto baseLegalState = alignedKickState(geometry, 0, 4);
    const auto baseLegalTarget = selector.select(baseLegalState);
    if (baseLegalTarget.value()) {
        const auto baseline = BilliardAlgorithm::generateLegalContactForTest(
            baseLegalState,
            *baseLegalTarget.value(),
            geometry,
            kickConfig());
        if (baseline.kicks[0]) {
            const auto legalBaselineKick = *baseline.kicks[0];
            auto firstBlockedState = baseLegalState;
            firstBlockedState.objectBalls[1] = Point{
                (legalBaselineKick.cuePathFirst.start.x +
                    legalBaselineKick.cuePathFirst.end.x) / 2.0,
                (legalBaselineKick.cuePathFirst.start.y +
                    legalBaselineKick.cuePathFirst.end.y) / 2.0};
            const auto firstBlocked = BilliardAlgorithm::generateLegalContactForTest(
                firstBlockedState,
                *baseLegalTarget.value(),
                geometry,
                kickConfig());
            tests.expectFalse(firstBlocked.kicks[0].has_value(),
                "KickLegalContact rejects collision on C-to-R");

            auto secondBlockedState = baseLegalState;
            secondBlockedState.objectBalls[1] = Point{
                (legalBaselineKick.cuePathSecond.start.x +
                    legalBaselineKick.cuePathSecond.end.x) / 2.0,
                (legalBaselineKick.cuePathSecond.start.y +
                    legalBaselineKick.cuePathSecond.end.y) / 2.0};
            const auto secondBlocked = BilliardAlgorithm::generateLegalContactForTest(
                secondBlockedState,
                *baseLegalTarget.value(),
                geometry,
                kickConfig());
            tests.expectFalse(secondBlocked.kicks[0].has_value(),
                "KickLegalContact rejects collision on R-to-G");
        }

        const auto& firstRail = geometry.railReflectionRegion.rails[0];
        const Vector2D railDirectionRaw{
            firstRail.segment.end.x - firstRail.segment.start.x,
            firstRail.segment.end.y - firstRail.segment.start.y};
        const double railLength = std::hypot(
            railDirectionRaw.x,
            railDirectionRaw.y);
        const Point outsideEffectiveRail{
            firstRail.segment.start.x - railDirectionRaw.x / railLength,
            firstRail.segment.start.y - railDirectionRaw.y / railLength};
        const auto exclusionState = kickStateForRebound(
            geometry,
            0,
            4,
            outsideEffectiveRail);
        const auto exclusionTarget = selector.select(exclusionState);
        if (exclusionTarget.value()) {
            const auto exclusion = BilliardAlgorithm::generateLegalContactForTest(
                exclusionState,
                *exclusionTarget.value(),
                geometry,
                kickConfig());
            tests.expectFalse(exclusion.kicks[0].has_value(),
                "KickLegalContact rejects a rebound outside the effective rail exclusion boundary");
        }

        const Point railMidpoint{
            (firstRail.segment.start.x + firstRail.segment.end.x) / 2.0,
            (firstRail.segment.start.y + firstRail.segment.end.y) / 2.0};
        const auto zeroFirstSegmentState = kickStateForRebound(
            geometry,
            0,
            4,
            railMidpoint,
            0.0);
        const auto zeroFirstSegmentTarget = selector.select(zeroFirstSegmentState);
        if (zeroFirstSegmentTarget.value()) {
            const auto zeroFirstSegment =
                BilliardAlgorithm::generateLegalContactForTest(
                    zeroFirstSegmentState,
                    *zeroFirstSegmentTarget.value(),
                    geometry,
                    kickConfig());
            bool namedZeroLengthDiagnostic = false;
            for (const auto& diagnostic : zeroFirstSegment.diagnostics) {
                namedZeroLengthDiagnostic = namedZeroLengthDiagnostic ||
                    (diagnostic.railId == BilliardConfig::RailId::Rail1 &&
                     diagnostic.reason ==
                        LegalContactRejectionReason::FirstSegmentInvalid &&
                     diagnostic.geometryStatus ==
                        GeometryStatus::DegenerateGeometry);
            }
            tests.expectTrue(
                !zeroFirstSegment.kicks[0] && namedZeroLengthDiagnostic,
                "KickLegalContact rejects zero-length C-to-R with a named diagnostic");
        }
    }

    std::array<std::optional<Point>, 9> zeroLengthBalls{};
    zeroLengthBalls[0] = Point{500.0, 250.0};
    const StableTableState zeroLengthLegalState =
        state({480.0, 250.0}, zeroLengthBalls);
    const EligibleTarget zeroLengthTarget{1, *zeroLengthBalls[0]};
    const auto zeroLengthLegal = BilliardAlgorithm::generateLegalContactForTest(
        zeroLengthLegalState,
        zeroLengthTarget,
        geometry,
        kickConfig());
    tests.expectFalse(zeroLengthLegal.direct.has_value(),
        "DirectLegalContact rejects a zero-length C-to-G path");

    auto nonFiniteLegalState = baseLegalState;
    nonFiniteLegalState.cueBall.x = std::numeric_limits<double>::quiet_NaN();
    if (baseLegalTarget.value()) {
        const auto nonFiniteLegal = BilliardAlgorithm::generateLegalContactForTest(
            nonFiniteLegalState,
            *baseLegalTarget.value(),
            geometry,
            kickConfig());
        bool anyKick = false;
        for (const auto& candidate : nonFiniteLegal.kicks) {
            anyKick = anyKick || candidate.has_value();
        }
        tests.expectTrue(!nonFiniteLegal.direct && !anyKick,
            "LegalContact rejects non-finite cue geometry without fallback");
    }

    std::array<std::optional<Point>, 9> parallelBalls{};
    parallelBalls[0] = Point{200.0, -80.0};
    const StableTableState parallelState = state({100.0, 100.0}, parallelBalls);
    const EligibleTarget parallelTarget{1, *parallelBalls[0]};
    const auto parallelLegal = BilliardAlgorithm::generateLegalContactForTest(
        parallelState,
        parallelTarget,
        geometry,
        kickConfig());
    bool parallelRejected = false;
    for (const auto& diagnostic : parallelLegal.diagnostics) {
        parallelRejected = parallelRejected ||
            (diagnostic.railId == BilliardConfig::RailId::Rail1 &&
             diagnostic.reason == LegalContactRejectionReason::NoRailIntersection);
    }
    tests.expectTrue(!parallelLegal.kicks[0] && parallelRejected,
        "KickLegalContact fails closed when the mirror ray is parallel to the rail");

    auto noLegalConfig = p109Config;
    noLegalConfig.scoring->planningMode =
        BilliardConfig::PlanningMode::ManualResearch;
    const auto noLegalPlanning = BilliardAlgorithm::planShot(
        noPotState,
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        noLegalConfig);
    const auto* noLegal = std::get_if<NoPlan>(&noLegalPlanning.value());
    tests.expectTrue(
        noLegalPlanning.isValid() && noLegal &&
        noLegal->reason == NoPlanReason::NoLegalContact &&
        noLegal->proceededToLegalContact &&
        !noLegal->legalContactDiagnostics.empty(),
        "ManualResearch returns NoLegalContact when direct and one-rail contact fail");

    std::array<std::optional<Point>, 9> noNumberedBalls{};
    const StableTableState noTargetState = state({500.0, 250.0}, noNumberedBalls);
    const auto noTargetPlanning = BilliardAlgorithm::planShot(
        noTargetState,
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        p109Config);
    const auto* integratedNoTarget = std::get_if<NoPlan>(&noTargetPlanning.value());
    tests.expectTrue(
        noTargetPlanning.isValid() && integratedNoTarget &&
        integratedNoTarget->reason == NoPlanReason::NoEligibleTarget &&
        !integratedNoTarget->selectedTarget &&
        integratedNoTarget->diagnostic.targetStatus ==
            TargetQualificationStatus::NoEligibleTarget,
        "P1-09 preserves TargetSelector NoEligibleTarget without higher-ball fallback");

    const auto missingBrainConfig = BilliardAlgorithm::planShot(
        kickFixture,
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        BilliardConfig::BrainConfig{});
    const auto* invalidBrain = std::get_if<NoPlan>(&missingBrainConfig.value());
    tests.expectTrue(
        missingBrainConfig.isValid() && invalidBrain &&
        invalidBrain->reason == NoPlanReason::InvalidBrainConfiguration &&
        !invalidBrain->selectedTarget,
        "missing P1-09 BrainConfig fails closed as a named planning result");

    const auto makeReceiveEvent = [&](ReceiveEventId id, long long milliseconds) {
        return ReceiveEvent{
            77,
            88,
            id,
            std::chrono::steady_clock::time_point{
                std::chrono::milliseconds{milliseconds}},
            ValidatedVisionFrame{
                kickFixture.objectBalls,
                kickFixture.cueBall,
                kickFixture.pockets}};
    };
    ThreeEventStability phase1PipelineStability({
        std::optional<double>{0.0},
        std::optional<double>{0.0},
        std::optional<std::chrono::milliseconds>{std::chrono::milliseconds{100}}});
    const StabilityResult pipelineFirst =
        phase1PipelineStability.accept(makeReceiveEvent(1, 0));
    const StabilityResult pipelineSecond =
        phase1PipelineStability.accept(makeReceiveEvent(2, 10));
    const StabilityResult pipelineStable =
        phase1PipelineStability.accept(makeReceiveEvent(3, 20));
    const auto pipelineWaiting = Phase1PipelineResult::fromStability(pipelineFirst);
    tests.expectTrue(
        pipelineFirst.status() == StabilityStatus::NeedMoreEvents &&
        pipelineSecond.status() == StabilityStatus::NeedMoreEvents &&
        pipelineWaiting && pipelineWaiting->isValid() &&
        !pipelineWaiting->planningResult() &&
        pipelineStable.status() == StabilityStatus::Stable &&
        pipelineStable.value(),
        "fake ReceiveEvent stream remains in the pipeline until Stable success");
    if (pipelineStable.value()) {
        const PlanningResult plannedStable = BilliardAlgorithm::planShot(
            *pipelineStable.value(),
            std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
            p109Config);
        const Phase1PipelineResult completed =
            Phase1PipelineResult::planningCompleted(plannedStable);
        tests.expectTrue(
            completed.status() == Phase1PipelineStatus::PlanningCompleted &&
            completed.isValid() && completed.planningResult() &&
            std::holds_alternative<ShotPlan>(completed.planningResult()->value()) &&
            !completed.diagnostic(),
            "Stable fake-event pipeline wraps the independent Brain PlanningResult");
    }

    const Phase1PipelineResult parserFailure = Phase1PipelineResult::inputFailure(
        SingleFrameDiagnostic{SingleFrameStatus::InvalidNumericToken, 0});
    const auto unstablePipeline = Phase1PipelineResult::fromStability(
        StabilityResult::failure(
            StabilityStatus::Unstable,
            StabilityFailureReason::BallMoved,
            2));
    const auto timeoutPipeline = Phase1PipelineResult::fromStability(
        StabilityResult::failure(
            StabilityStatus::TimedOut,
            StabilityFailureReason::TimedOut,
            2));
    tests.expectTrue(
        parserFailure.isValid() && !parserFailure.planningResult() &&
        unstablePipeline && unstablePipeline->isValid() &&
        !unstablePipeline->planningResult() && timeoutPipeline &&
        timeoutPipeline->isValid() && !timeoutPipeline->planningResult(),
        "parser, Unstable, and timeout pipeline failures cannot become NoPlan");

    const auto omittedCandidates = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        noDirectCandidates(fullDirect),
        noKickCandidates(fullKick),
        std::optional<BilliardConfig::ScoringConfig>{scoreConfig},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        omittedCandidates.status() == PotSelectionStatus::InvalidCandidateInput &&
        !omittedCandidates.value(),
        "caller cannot suppress authoritative feasible candidates to force NoPlan");

    auto manualConfig = scoreConfig;
    manualConfig.planningMode = BilliardConfig::PlanningMode::ManualResearch;
    const auto manualEmpty = BilliardAlgorithm::selectBestPot(
        noPotState,
        *noPotTarget.value(),
        geometry,
        *noPotDirect.value(),
        *noPotKick.value(),
        std::optional<BilliardConfig::ScoringConfig>{manualConfig},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        manualEmpty.status() == PotSelectionStatus::ManualResearchPotSearchExhausted &&
        !manualEmpty.value(),
        "manual research exhaustion stays isolated and does not create LegalContact here");

    const auto missingScoring = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::nullopt,
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        missingScoring.status() == PotSelectionStatus::ConfigurationMissing &&
        !missingScoring.value(),
        "missing scoring ranges fail closed");

    auto invalidWeights = scoreConfig;
    invalidWeights.rawWeights.kickPenalty = -0.1;
    const auto rejectedWeights = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{invalidWeights},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedWeights.status() == PotSelectionStatus::InvalidConfiguration &&
        !rejectedWeights.value(),
        "negative raw scoring weight fails closed");

    auto nonFiniteWeights = scoreConfig;
    nonFiniteWeights.rawWeights.cuttingAngle =
        std::numeric_limits<double>::quiet_NaN();
    const auto rejectedNanWeights = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{nonFiniteWeights},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedNanWeights.status() == PotSelectionStatus::InvalidConfiguration &&
        !rejectedNanWeights.value(),
        "NaN raw scoring weight fails closed");

    nonFiniteWeights = scoreConfig;
    nonFiniteWeights.rawWeights.totalDistance =
        std::numeric_limits<double>::infinity();
    const auto rejectedInfiniteWeights = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{nonFiniteWeights},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedInfiniteWeights.status() == PotSelectionStatus::InvalidConfiguration &&
        !rejectedInfiniteWeights.value(),
        "infinite raw scoring weight fails closed");

    auto zeroWeights = scoreConfig;
    zeroWeights.rawWeights = {};
    const auto rejectedZeroWeights = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{zeroWeights},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedZeroWeights.status() == PotSelectionStatus::InvalidConfiguration &&
        !rejectedZeroWeights.value(),
        "zero raw weight sum fails closed");

    auto scaledWeights = scoreConfig;
    scaledWeights.rawWeights = {0.60, 0.60, 0.40, 0.20, 0.10, 0.10};
    const auto normalizedScaledWeights = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{scaledWeights},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    const auto* scaledWinner = normalizedScaledWeights.value()
        ? std::get_if<ScoredPotCandidate>(&*normalizedScaledWeights.value())
        : nullptr;
    tests.expectTrue(scaledWinner != nullptr,
        "raw weights need not sum to exactly one");
    if (scaledWinner) {
        tests.expectNear(scaledWinner->audit.rawWeightSum, 2.0,
            TOLERANCE, "scaled raw weight sum is audited");
        tests.expectNear(scaledWinner->audit.effectiveWeights.kickPenalty, 0.30,
            TOLERANCE, "scaled raw weights normalize before scoring");
    }

    auto overflowWeights = scoreConfig;
    overflowWeights.rawWeights = {
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        0.0,
        0.0,
        0.0,
        0.0};
    const auto rejectedOverflowWeights = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{overflowWeights},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedOverflowWeights.status() == PotSelectionStatus::InvalidConfiguration &&
        !rejectedOverflowWeights.value(),
        "overflowing raw weight sum fails closed");

    auto invalidRange = scoreConfig;
    invalidRange.maxDistanceMm = invalidRange.minDistanceMm;
    const auto rejectedRange = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{invalidRange},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedRange.status() == PotSelectionStatus::InvalidConfiguration &&
        !rejectedRange.value(),
        "degenerate normalization range fails closed");

    auto invalidClearanceRange = scoreConfig;
    invalidClearanceRange.preferredClearanceMm =
        geometry.ballDiameterMm + geometry.collisionMarginMm;
    const auto rejectedClearanceRange = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{invalidClearanceRange},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedClearanceRange.status() == PotSelectionStatus::InvalidConfiguration &&
        !rejectedClearanceRange.value(),
        "degenerate clearance normalization range fails closed");

    auto invalidCutRange = scoreConfig;
    invalidCutRange.maxCutAngleDeg = 0.0;
    const auto rejectedCutRange = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{invalidCutRange},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedCutRange.status() == PotSelectionStatus::InvalidConfiguration &&
        !rejectedCutRange.value(),
        "invalid cutting-angle normalization range fails closed");

    auto invalidEffectiveTolerance = scoreConfig;
    invalidEffectiveTolerance.effectiveWeightSumTolerance = -1.0;
    const auto rejectedEffectiveTolerance = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{invalidEffectiveTolerance},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedEffectiveTolerance.status() == PotSelectionStatus::InvalidConfiguration &&
        !rejectedEffectiveTolerance.value(),
        "invalid effective-weight tolerance fails closed");

    auto invalidTie = scoreConfig;
    invalidTie.tieEpsilon = std::numeric_limits<double>::quiet_NaN();
    const auto rejectedTie = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{invalidTie},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedTie.status() == PotSelectionStatus::InvalidConfiguration &&
        !rejectedTie.value(),
        "non-finite tie epsilon fails closed");

    const auto missingKickRange = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        fullDirect,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{scoreConfig},
        std::nullopt);
    tests.expectTrue(
        missingKickRange.status() == PotSelectionStatus::ConfigurationMissing &&
        !missingKickRange.value(),
        "Kick scoring requires its approved rail-angle normalization range");

    auto nonFiniteCandidate = fullDirect;
    nonFiniteCandidate.feasible[4]->cuttingAngleDeg =
        std::numeric_limits<double>::quiet_NaN();
    const auto rejectedCandidate = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        nonFiniteCandidate,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{scoreConfig},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedCandidate.status() == PotSelectionStatus::InvalidCandidateInput &&
        !rejectedCandidate.value(),
        "non-finite feasible-candidate input fails closed and is not revived");

    auto contradictoryCandidate = fullDirect;
    contradictoryCandidate.feasible[4]->cuePath.start = {-10000.0, -10000.0};
    contradictoryCandidate.feasible[4]->cuttingAngleDeg += 1.0;
    const auto rejectedContradictoryCandidate = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        contradictoryCandidate,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{scoreConfig},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedContradictoryCandidate.status() ==
            PotSelectionStatus::InvalidCandidateInput &&
        !rejectedContradictoryCandidate.value(),
        "finite but contradictory feasible candidate fails provenance validation");

    auto overflowingDistance = fullDirect;
    overflowingDistance.feasible[4]->cuePath.start = {
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()};
    overflowingDistance.feasible[4]->cuePath.end = {
        -std::numeric_limits<double>::max(),
        -std::numeric_limits<double>::max()};
    const auto rejectedDistance = BilliardAlgorithm::selectBestPot(
        kickFixture,
        *kickFixtureTarget.value(),
        geometry,
        overflowingDistance,
        fullKick,
        std::optional<BilliardConfig::ScoringConfig>{scoreConfig},
        std::optional<BilliardConfig::KickGeometryConfig>{kickGeometryConfig});
    tests.expectTrue(
        rejectedDistance.status() == PotSelectionStatus::InvalidCandidateInput ||
        rejectedDistance.status() == PotSelectionStatus::NumericalFailure,
        "overflowing candidate distance fails closed without fallback score");

    return tests.exitCode();
}
