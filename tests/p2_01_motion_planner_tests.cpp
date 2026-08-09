#include "TestHarness.h"

#include "../src/Algorithm.h"
#include "../src/BilliardPhysics.h"
#include "../src/MotionPlanner.h"
#include "../src/TargetSelector.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <variant>

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
        id, type, outward, 30.0,
        {{-side.x * halfLength, -side.y * halfLength},
         {side.x * halfLength, side.y * halfLength}},
        15.0, 2.0, 0.01, 45.0};
}

BilliardConfig::TableGeometryConfig tableConfig()
{
    using namespace BilliardConfig;
    return {
        "p2-01-table-v1", {0.0, 1000.0, 0.0, 500.0}, 10.0, 20.0, 2.0,
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
    return {{{20.0, 20.0}, {500.0, 10.0}, {980.0, 20.0},
             {20.0, 480.0}, {500.0, 490.0}, {980.0, 480.0}}};
}

StableTableState state(
    Point cueBall,
    std::array<std::optional<Point>, 9> balls)
{
    const auto now = std::chrono::steady_clock::now();
    return {
        std::move(balls), cueBall, pocketCenters(), 11, 22,
        {{{101, now}, {102, now}, {103, now}}}};
}

BilliardConfig::KickGeometryConfig kickConfig()
{
    return {89.0, 1e-8, 1e-8};
}

BilliardConfig::ScoringConfig scoringConfig(
    BilliardConfig::PlanningMode mode = BilliardConfig::PlanningMode::PotOnly)
{
    return {
        BilliardConfig::INITIAL_EXPERIMENTAL_SCORING_WEIGHTS,
        1e-9, 90.0, 0.0, 3000.0, 200.0, 1e-9, mode};
}

BilliardConfig::BrainConfig brainConfig(
    BilliardConfig::PlanningMode mode = BilliardConfig::PlanningMode::PotOnly)
{
    return {
        std::optional<std::string>{"p2-01-base0-v1"},
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()},
        std::optional<BilliardConfig::ScoringConfig>{scoringConfig(mode)}};
}

StableTableState kickStateForRebound(
    const ResolvedTableGeometry& geometry,
    std::size_t railIndex,
    std::size_t pocketIndex)
{
    const auto& rail = geometry.railReflectionRegion.rails[railIndex];
    const Point rebound{
        (rail.segment.start.x + rail.segment.end.x) / 2.0,
        (rail.segment.start.y + rail.segment.end.y) / 2.0};
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
        outgoingRaw.x / outgoingLength, outgoingRaw.y / outgoingLength};
    const double projection =
        outgoing.x * rail.inwardUnitNormal.x +
        outgoing.y * rail.inwardUnitNormal.y;
    const Vector2D incoming{
        outgoing.x - 2.0 * projection * rail.inwardUnitNormal.x,
        outgoing.y - 2.0 * projection * rail.inwardUnitNormal.y};
    std::array<std::optional<Point>, 9> balls{};
    balls[0] = target;
    return state(
        {rebound.x - 120.0 * incoming.x,
         rebound.y - 120.0 * incoming.y},
        balls);
}

std::optional<PlanningResult> directPlanning(const ResolvedTableGeometry& geometry)
{
    const StableTableState fixture = kickStateForRebound(geometry, 0, 4);
    PlanningResult result = BilliardAlgorithm::planShot(
        fixture,
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        brainConfig());
    const ShotPlan* plan = std::get_if<ShotPlan>(&result.value());
    if (!result.isValid() || !plan || plan->type != ShotPlanType::DirectPot) {
        return std::nullopt;
    }
    return result;
}

std::optional<PlanningResult> kickPlanning(const ResolvedTableGeometry& geometry)
{
    TargetSelector selector;
    for (std::size_t railIndex = 0;
         railIndex < geometry.railReflectionRegion.rails.size();
         ++railIndex) {
        for (std::size_t pocketIndex = 0;
             pocketIndex < geometry.pockets.size();
             ++pocketIndex) {
            StableTableState fixture = kickStateForRebound(
                geometry, railIndex, pocketIndex);
            const auto selected = selector.select(fixture);
            if (!selected.value()) continue;
            const auto direct = BilliardAlgorithm::generateDirectPotCandidates(
                fixture, *selected.value(), geometry);
            if (!direct.value()) continue;
            std::size_t obstacleIndex = 1;
            for (const auto& candidate : direct.value()->feasible) {
                if (!candidate || obstacleIndex >= fixture.objectBalls.size()) continue;
                fixture.objectBalls[obstacleIndex++] = Point{
                    (candidate->cuePath.start.x + candidate->cuePath.end.x) / 2.0,
                    (candidate->cuePath.start.y + candidate->cuePath.end.y) / 2.0};
            }
            PlanningResult result = BilliardAlgorithm::planShot(
                fixture,
                std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
                brainConfig());
            const ShotPlan* plan = std::get_if<ShotPlan>(&result.value());
            if (result.isValid() && plan && plan->type == ShotPlanType::KickPot) {
                return result;
            }
        }
    }
    return std::nullopt;
}

std::optional<PlanningResult> legalPlanning(const ResolvedTableGeometry& geometry)
{
    std::array<std::optional<Point>, 9> balls{};
    balls[0] = Point{500.0, 250.0};
    for (std::size_t pocket = 0; pocket < geometry.pockets.size(); ++pocket) {
        const Point target = geometry.pockets[pocket].virtualPocketTarget;
        balls[pocket + 1] = Point{
            balls[0]->x + 0.75 * (target.x - balls[0]->x),
            balls[0]->y + 0.75 * (target.y - balls[0]->y)};
    }
    PlanningResult result = BilliardAlgorithm::planShot(
        state({400.0, 250.0}, balls),
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        brainConfig(BilliardConfig::PlanningMode::ManualResearch));
    const ShotPlan* plan = std::get_if<ShotPlan>(&result.value());
    if (!result.isValid() || !plan ||
        (plan->type != ShotPlanType::DirectLegalContact &&
         plan->type != ShotPlanType::KickLegalContact)) {
        return std::nullopt;
    }
    return result;
}

BilliardConfig::MotionPlanningConfig motionConfig()
{
    BilliardConfig::MotionPlanningConfig config;
    config.calibrationRevision = "p2-01-motion-v1";
    config.base0PlanarCalibrationRevision = "p2-01-base0-v1";
    config.cueForwardAxisCalibrationRevision = "tool1-axis-test-v1";
    config.strikeZMm = -216.0;
    config.safeApproachZMm = -160.0;
    config.readyGapMm = 15.0;
    config.safeLiftHeightMm = 50.0;
    config.a0Deg = 0.0;
    config.b0Deg = 0.0;
    config.deltaADeg = 1.0;
    config.deltaBDeg = 1.0;
    config.stepADeg = 1.0;
    config.stepBDeg = 1.0;
    config.searchOrder = BilliardConfig::PoseSearchOrder::AThenB;
    config.axisOffsetOrder = BilliardConfig::AxisOffsetOrder::LowerThenHigher;
    config.tieBreak = BilliardConfig::PoseTieBreak::FirstInApprovedSearchOrder;
    config.cToolOffsetDeg = 7.0;
    config.cueForwardAxisTool = std::array<double, 3>{0.0, 1.0, 0.0};
    config.maxCueDirectionErrorDeg = 0.01;
    config.directionUnitTolerance = 1e-9;
    config.executionPolicyRevision = "execution-policy-test-v1";
    config.policyMode = BilliardConfig::ExecutionPolicyMode::PlanningTest;
    config.legalContactExecutionAuthorized = false;
    const BilliardConfig::FixedForceEnvelopeLimits directPot{
        true, 0.0, 5000.0, 90.0, 180.0, std::nullopt};
    const BilliardConfig::FixedForceEnvelopeLimits kickPot{
        true, 0.0, 5000.0, 90.0, 180.0, 89.0};
    const BilliardConfig::FixedForceEnvelopeLimits directLegal{
        true, 0.0, 5000.0, std::nullopt, std::nullopt, std::nullopt};
    const BilliardConfig::FixedForceEnvelopeLimits kickLegal{
        true, 0.0, 5000.0, std::nullopt, std::nullopt, 89.0};
    config.fixedForceEnvelope = BilliardConfig::FixedForceEnvelopeConfig{
        "fixed-force-test-v1", directPot, kickPot, directLegal, kickLegal};
    config.pneumaticTimingProfile =
        BilliardConfig::PneumaticTimingProfileReference{
            "pneumatic-test-v1", 100, 50, 100};
    return config;
}

MotionPlanningChecks alignedChecks(double cOffsetDeg)
{
    return {
        [](const RobotPoseABC&) { return std::optional<bool>{true}; },
        [cOffsetDeg](
            const RobotPoseABC& pose,
            const std::array<double, 3>& axis) -> std::optional<Vector2D> {
            if (axis != std::array<double, 3>{0.0, 1.0, 0.0}) {
                return std::nullopt;
            }
            const double angle = (pose.c - cOffsetDeg) *
                3.14159265358979323846 / 180.0;
            return Vector2D{std::cos(angle), std::sin(angle)};
        }};
}
}

int main()
{
    TestHarness tests;
    const auto resolved = BilliardPhysics::resolveTableGeometry(
        pocketCenters(), tableConfig());
    tests.expectTrue(resolved.value().has_value(), "P2-01 geometry fixture resolves");
    if (!resolved.value()) return tests.exitCode();
    const ResolvedTableGeometry geometry = *resolved.value();
    const auto direct = directPlanning(geometry);
    const auto kick = kickPlanning(geometry);
    const auto legal = legalPlanning(geometry);
    tests.expectTrue(direct.has_value(), "P2-01 has a valid DirectPot input");
    tests.expectTrue(kick.has_value(), "P2-01 has a valid KickPot input");
    tests.expectTrue(legal.has_value(), "P2-01 has a valid LegalContact input");
    if (!direct || !kick || !legal) return tests.exitCode();

    MotionPlanner planner;
    const auto config = motionConfig();
    const auto checks = alignedChecks(*config.cToolOffsetDeg);
    const ExecutionPlanResult directResult = planner.createExecutionPlan(
        *direct, config, checks);
    tests.expectTrue(
        directResult.isValid() && directResult.value() &&
        directResult.value()->sourceShotType == ShotPlanType::DirectPot,
        "valid DirectPot creates one validated ExecutionPlan");
    if (!directResult.value()) return tests.exitCode();
    const ShotPlan& directShot = std::get<ShotPlan>(direct->value());
    const ExecutionPlan& directPlan = *directResult.value();
    const double expectedDistance =
        directShot.source.ballRadiusMm + *config.readyGapMm;
    tests.expectNear(directPlan.strikeReadyPose.x,
        directShot.source.cueBallSnapshot.x -
            expectedDistance * directShot.shotDirectionXY.x,
        TOLERANCE,
        "StrikeReady X uses Base0 cue center without a second transform");
    tests.expectNear(directPlan.strikeReadyPose.y,
        directShot.source.cueBallSnapshot.y -
            expectedDistance * directShot.shotDirectionXY.y,
        TOLERANCE,
        "StrikeReady Y uses Base0 cue center without a second transform");
    tests.expectNear(directPlan.strikeReadyPose.z, *config.strikeZMm,
        TOLERANCE, "StrikeReady Z comes only from approved manual calibration");
    tests.expectNear(directPlan.strikeReadyPose.c,
        *MotionPlanner::directionToCDegForTest(
            directShot.shotDirectionXY,
            *config.cToolOffsetDeg,
            *config.directionUnitTolerance),
        TOLERANCE,
        "C is atan2(shotDirection)+configured Tool offset");
    tests.expectTrue(
        directPlan.safeLiftRule.derivation ==
            SafeLiftDerivation::RuntimeActualPoseKeepXYABCIncreaseZ &&
        directPlan.safeLiftRule.heightMm == *config.safeLiftHeightMm,
        "ExecutionPlan records only the runtime-actual-pose safe-lift derivation rule");
    tests.expectTrue(
        directPlan.fixedForceEnvelope.isValid() &&
        directPlan.fixedForceEnvelope.calibrationRevision ==
            config.fixedForceEnvelope->calibrationRevision &&
        directPlan.pneumaticTimingProfile.pneumaticPulseMs ==
            config.pneumaticTimingProfile->pneumaticPulseMs,
        "ExecutionPlan carries the accepted fixed-force gate and timing reference");
    tests.expectTrue(
        directPlan.executionPolicyRevision == *config.executionPolicyRevision,
        "ExecutionPlan preserves the independent ExecutionPolicy revision");
    tests.expectTrue(std::all_of(
        directPlan.stageContracts.begin(), directPlan.stageContracts.end(),
        [](const PlannedStageContract& stage) { return stage.isValid(); }),
        "ExecutionPlan carries fail-closed contracts for every planned stage");
    tests.expectTrue(
        directPlan.strikeReadyPose.x != directShot.ghostBallPoint.center.x ||
        directPlan.strikeReadyPose.y != directShot.ghostBallPoint.center.y,
        "GhostBallPoint is not reused as the Robot TCP StrikeReady position");
    ExecutionPlan nonFinitePose = directPlan;
    nonFinitePose.strikeReadyPose.z =
        std::numeric_limits<double>::quiet_NaN();
    tests.expectFalse(nonFinitePose.isValid(),
        "ExecutionPlan invariant rejects a non-finite Robot Pose");
    const ExecutionPlanResult validFactory =
        ExecutionPlanResult::success(directPlan);
    tests.expectTrue(
        validFactory.status() == ExecutionPlanStatus::Success &&
        validFactory.value().has_value(),
        "success factory accepts a fully valid ExecutionPlan");
    const ExecutionPlanResult invalidFactory =
        ExecutionPlanResult::success(nonFinitePose);
    tests.expectTrue(
        invalidFactory.status() == ExecutionPlanStatus::InvalidExecutionPlan &&
        !invalidFactory.value() && invalidFactory.diagnostic() &&
        invalidFactory.diagnostic()->reason ==
            ExecutionPlanFailureReason::InvalidExecutionPlanValue,
        "success factory rejects an invalid ExecutionPlan without success value");
    ExecutionPlan invalidPolicy = directPlan;
    invalidPolicy.policyMode = static_cast<BilliardConfig::ExecutionPolicyMode>(99);
    tests.expectFalse(invalidPolicy.isValid(),
        "ExecutionPlan invariant rejects an unknown policy mode");

    const ExecutionPlanResult repeated = planner.createExecutionPlan(
        *direct, config, checks);
    tests.expectTrue(repeated.value() &&
        repeated.value()->strikeReadyPose.a == directPlan.strikeReadyPose.a &&
        repeated.value()->strikeReadyPose.b == directPlan.strikeReadyPose.b &&
        repeated.value()->strikeReadyPose.c == directPlan.strikeReadyPose.c,
        "same ShotPlan and calibration produce deterministic ExecutionPlan");

    const ExecutionPlanResult kickResult = planner.createExecutionPlan(
        *kick, config, checks);
    const ShotPlan& kickShot = std::get<ShotPlan>(kick->value());
    tests.expectTrue(kickResult.value() &&
        kickResult.value()->sourceShotType == ShotPlanType::KickPot &&
        kickResult.value()->shotDirectionXY.x == kickShot.shotDirectionXY.x &&
        kickResult.value()->shotDirectionXY.y == kickShot.shotDirectionXY.y,
        "Kick ExecutionPlan preserves Phase 1 C-to-R shotDirection");
    tests.expectTrue(kickResult.value() &&
        kickResult.value()->pneumaticTimingProfile.pneumaticPulseMs ==
            directPlan.pneumaticTimingProfile.pneumaticPulseMs,
        "Direct and Kick use one versioned fixed pneumatic pulse reference");
    tests.expectTrue(
        !directPlan.fixedForceEnvelope.maxExecutableKickRailAngleDeg &&
        kickResult.value() &&
        kickResult.value()->fixedForceEnvelope.maxExecutableKickRailAngleDeg ==
            config.fixedForceEnvelope->kickPot.maxExecutableKickRailAngleDeg,
        "DirectPot and KickPot retain their own configured envelope semantics");

    const std::array<std::pair<Vector2D, double>, 4> quadrantCases{{
        {{1.0, 0.0}, 7.0}, {{0.0, 1.0}, 97.0},
        {{-1.0, 0.0}, -173.0}, {{0.0, -1.0}, -83.0}}};
    for (const auto& testCase : quadrantCases) {
        const auto c = MotionPlanner::directionToCDegForTest(
            testCase.first, 7.0, 1e-9);
        tests.expectTrue(c.has_value(), "finite unit direction produces C");
        if (c) tests.expectNear(*c, testCase.second, TOLERANCE,
            "C normalization is correct in every XY quadrant");
    }

    ShotPlan zeroDirection = directShot;
    zeroDirection.shotDirectionXY = {0.0, 0.0};
    const auto zeroResult = planner.createExecutionPlan(
        PlanningResult::shotPlan(zeroDirection), config, checks);
    tests.expectTrue(
        zeroResult.status() == ExecutionPlanStatus::InvalidShotPlan &&
        !zeroResult.value(),
        "zero ShotPlan direction fails closed without a Pose");
    ShotPlan nonFiniteDirection = directShot;
    nonFiniteDirection.shotDirectionXY.x =
        std::numeric_limits<double>::quiet_NaN();
    const auto nonFiniteResult = planner.createExecutionPlan(
        PlanningResult::shotPlan(nonFiniteDirection), config, checks);
    tests.expectTrue(
        nonFiniteResult.status() == ExecutionPlanStatus::InvalidShotPlan &&
        !nonFiniteResult.value(),
        "non-finite ShotPlan direction fails closed without a Pose");

    auto missingStrikeZ = config;
    missingStrikeZ.strikeZMm.reset();
    tests.expectTrue(planner.createExecutionPlan(*direct, missingStrikeZ, checks).status() ==
        ExecutionPlanStatus::ConfigurationMissing,
        "missing Strike Z is ConfigurationMissing");
    auto missingA = config;
    missingA.a0Deg.reset();
    tests.expectTrue(planner.createExecutionPlan(*direct, missingA, checks).status() ==
        ExecutionPlanStatus::ConfigurationMissing,
        "missing A baseline is ConfigurationMissing");
    auto missingRange = config;
    missingRange.deltaBDeg.reset();
    tests.expectTrue(planner.createExecutionPlan(*direct, missingRange, checks).status() ==
        ExecutionPlanStatus::ConfigurationMissing,
        "missing A/B range is ConfigurationMissing");
    auto missingStep = config;
    missingStep.stepADeg.reset();
    tests.expectTrue(planner.createExecutionPlan(*direct, missingStep, checks).status() ==
        ExecutionPlanStatus::ConfigurationMissing,
        "missing A/B step is ConfigurationMissing");
    auto missingAxis = config;
    missingAxis.cueForwardAxisTool.reset();
    tests.expectTrue(planner.createExecutionPlan(*direct, missingAxis, checks).status() ==
        ExecutionPlanStatus::ConfigurationMissing,
        "missing Tool forward-axis calibration is ConfigurationMissing");
    auto missingEnvelope = config;
    missingEnvelope.fixedForceEnvelope.reset();
    tests.expectTrue(planner.createExecutionPlan(
        *direct, missingEnvelope, checks).status() ==
        ExecutionPlanStatus::ConfigurationMissing,
        "missing fixed-force envelope is ConfigurationMissing");
    auto missingTiming = config;
    missingTiming.pneumaticTimingProfile.reset();
    tests.expectTrue(planner.createExecutionPlan(
        *direct, missingTiming, checks).status() ==
        ExecutionPlanStatus::ConfigurationMissing,
        "missing pneumatic timing reference is ConfigurationMissing");
    auto missingPolicyRevision = config;
    missingPolicyRevision.executionPolicyRevision.reset();
    tests.expectTrue(planner.createExecutionPlan(
        *direct, missingPolicyRevision, checks).status() ==
        ExecutionPlanStatus::ConfigurationMissing,
        "missing ExecutionPolicy revision is ConfigurationMissing");
    auto emptyPolicyRevision = config;
    emptyPolicyRevision.executionPolicyRevision = "";
    tests.expectTrue(planner.createExecutionPlan(
        *direct, emptyPolicyRevision, checks).status() ==
        ExecutionPlanStatus::InvalidConfiguration,
        "empty ExecutionPolicy revision is InvalidConfiguration");
    auto directOutsideEnvelope = config;
    directOutsideEnvelope.fixedForceEnvelope->directPot.maxTotalPathLengthMm = 1.0;
    tests.expectTrue(planner.createExecutionPlan(
        *direct, directOutsideEnvelope, checks).status() ==
        ExecutionPlanStatus::NoExecutablePlan,
        "geometrically valid plan outside fixed-force envelope is not executable");
    auto kickEnvelopeTooWide = config;
    kickEnvelopeTooWide.fixedForceEnvelope->kickPot.maxExecutableKickRailAngleDeg =
        90.0;
    tests.expectTrue(planner.createExecutionPlan(
        *kick, kickEnvelopeTooWide, checks).status() ==
        ExecutionPlanStatus::InvalidConfiguration,
        "Phase 2 Kick envelope cannot be wider than Phase 1 geometry gate");
    const MotionPlanningChecks missingProjector{
        [](const RobotPoseABC&) { return std::optional<bool>{true}; }, {}};
    tests.expectTrue(planner.createExecutionPlan(
        *direct, config, missingProjector).status() ==
        ExecutionPlanStatus::ConfigurationMissing,
        "missing approved Tool-axis projection semantics is ConfigurationMissing");

    const MotionPlanningChecks mismatchedDirection{
        [](const RobotPoseABC&) { return std::optional<bool>{true}; },
        [](const RobotPoseABC&, const std::array<double, 3>&) {
            return std::optional<Vector2D>{{0.0, 1.0}};
        }};
    const auto mismatch = planner.createExecutionPlan(
        *direct, config, mismatchedDirection);
    tests.expectTrue(
        mismatch.status() == ExecutionPlanStatus::NoExecutablePlan &&
        mismatch.diagnostic() &&
        mismatch.diagnostic()->reason ==
            ExecutionPlanFailureReason::NoAcceptedPoseCandidate,
        "cue-axis projection beyond allowed error rejects every candidate");
    const MotionPlanningChecks zeroProjection{
        [](const RobotPoseABC&) { return std::optional<bool>{true}; },
        [](const RobotPoseABC&, const std::array<double, 3>&) {
            return std::optional<Vector2D>{{0.0, 0.0}};
        }};
    tests.expectTrue(planner.createExecutionPlan(*direct, config, zeroProjection).status() ==
        ExecutionPlanStatus::NoExecutablePlan,
        "zero projected Tool direction fails closed");

    auto invalidPoseConfig = config;
    invalidPoseConfig.strikeZMm = std::numeric_limits<double>::infinity();
    tests.expectTrue(planner.createExecutionPlan(
        *direct, invalidPoseConfig, checks).status() ==
        ExecutionPlanStatus::InvalidConfiguration,
        "non-finite Pose calibration is rejected");

    const MotionPlanningChecks boundedChecks{
        [](const RobotPoseABC& pose) {
            return std::optional<bool>{pose.a == -1.0 && pose.b == 1.0};
        },
        checks.projectCueForwardAxisToBase0XY};
    const auto bounded = planner.createExecutionPlan(*direct, config, boundedChecks);
    tests.expectTrue(bounded.value() &&
        bounded.value()->selectedADeg == -1.0 &&
        bounded.value()->selectedBDeg == 1.0 &&
        bounded.value()->selectedSearchOrdinal == 5,
        "A/B bounded search follows approved deterministic order without expansion");
    if (bounded.value()) {
        const PoseSearchAudit& audit = bounded.value()->poseSearchAudit;
        tests.expectTrue(audit.contains(-1.0, 1.0, 5),
            "PoseSearchAudit accepts the exact deterministic ordinal/A/B pair");
        tests.expectFalse(audit.contains(-1.0, -1.0, 4),
            "PoseSearchAudit rejects an ordinal inconsistent with evaluated count");
        tests.expectFalse(audit.contains(-1.0, 1.0, 9),
            "PoseSearchAudit rejects an out-of-range ordinal");
        PoseSearchAudit fiveEvaluated = audit;
        fiveEvaluated.evaluatedCandidateCount = 5;
        tests.expectFalse(fiveEvaluated.contains(-1.0, 1.0, 4),
            "PoseSearchAudit rejects legal ordinal with mismatched selected A/B");
    }
    const MotionPlanningChecks noPoseChecks{
        [](const RobotPoseABC&) { return std::optional<bool>{false}; },
        checks.projectCueForwardAxisToBase0XY};
    tests.expectTrue(planner.createExecutionPlan(*direct, config, noPoseChecks).status() ==
        ExecutionPlanStatus::NoExecutablePlan,
        "all rejected A/B candidates return NoExecutablePlan without fallback");

    const auto legalBlocked = planner.createExecutionPlan(*legal, config, checks);
    tests.expectTrue(
        legalBlocked.status() == ExecutionPlanStatus::NoExecutablePlan &&
        legalBlocked.diagnostic() &&
        legalBlocked.diagnostic()->reason ==
            ExecutionPlanFailureReason::LegalContactNotAuthorized &&
        !legalBlocked.value(),
        "LegalContact remains blocked by default execution policy");
    auto authorizedLegalConfig = config;
    authorizedLegalConfig.legalContactExecutionAuthorized = true;
    const auto authorizedLegal = planner.createExecutionPlan(
        *legal, authorizedLegalConfig, checks);
    tests.expectTrue(authorizedLegal.value() &&
        authorizedLegal.value()->policyDecision ==
            ExecutionPolicyDecision::LegalContactExplicitlyAuthorized &&
        authorizedLegal.value()->pneumaticTimingProfile.pneumaticPulseMs ==
            directPlan.pneumaticTimingProfile.pneumaticPulseMs,
        "offline LegalContact requires an explicit policy authorization");
    authorizedLegalConfig.policyMode =
        BilliardConfig::ExecutionPolicyMode::RealHardware;
    tests.expectTrue(planner.createExecutionPlan(
        *legal, authorizedLegalConfig, checks).status() ==
        ExecutionPlanStatus::NoExecutablePlan,
        "P2-01 cannot authorize LegalContact for real hardware");

    auto revisionMismatch = config;
    revisionMismatch.base0PlanarCalibrationRevision = "wrong-base0-revision";
    tests.expectTrue(planner.createExecutionPlan(
        *direct, revisionMismatch, checks).status() ==
        ExecutionPlanStatus::InvalidConfiguration,
        "Base0 calibration revision mismatch rejects ExecutionPlan");

    ShotPlan missingConnection = directShot;
    missingConnection.source.planIdentity.connectionIdentity = 0;
    tests.expectTrue(planner.createExecutionPlan(
        PlanningResult::shotPlan(missingConnection), config, checks).status() ==
        ExecutionPlanStatus::InvalidShotPlan,
        "missing source connection identity rejects ExecutionPlan");
    ShotPlan missingCycle = directShot;
    missingCycle.source.planIdentity.shotCycleIdentity = 0;
    tests.expectTrue(planner.createExecutionPlan(
        PlanningResult::shotPlan(missingCycle), config, checks).status() ==
        ExecutionPlanStatus::InvalidShotPlan,
        "missing source shot-cycle identity rejects ExecutionPlan");
    ShotPlan inconsistentEvents = directShot;
    inconsistentEvents.source.sourceEvents[1].eventId =
        inconsistentEvents.source.sourceEvents[0].eventId;
    tests.expectTrue(planner.createExecutionPlan(
        PlanningResult::shotPlan(inconsistentEvents), config, checks).status() ==
        ExecutionPlanStatus::InvalidShotPlan,
        "inconsistent source event IDs reject ExecutionPlan");

    const auto expectMalformedVariantRejected = [&](ShotPlan malformed,
                                                     const char* message) {
        tests.expectTrue(planner.createExecutionPlan(
            PlanningResult::shotPlan(std::move(malformed)), config, checks).status() ==
            ExecutionPlanStatus::InvalidShotPlan,
            message);
    };
    ShotPlan malformedDirect = directShot;
    malformedDirect.payload = std::get<KickPotShotPlanPayload>(kickShot.payload);
    expectMalformedVariantRejected(
        std::move(malformedDirect), "malformed DirectPot payload is rejected");
    ShotPlan malformedKick = directShot;
    malformedKick.type = ShotPlanType::KickPot;
    expectMalformedVariantRejected(
        std::move(malformedKick), "malformed KickPot payload is rejected");
    ShotPlan malformedDirectLegal = directShot;
    malformedDirectLegal.type = ShotPlanType::DirectLegalContact;
    expectMalformedVariantRejected(std::move(malformedDirectLegal),
        "malformed DirectLegalContact payload is rejected");
    ShotPlan malformedKickLegal = directShot;
    malformedKickLegal.type = ShotPlanType::KickLegalContact;
    expectMalformedVariantRejected(std::move(malformedKickLegal),
        "malformed KickLegalContact payload is rejected");
    ShotPlan unknownType = directShot;
    unknownType.type = static_cast<ShotPlanType>(99);
    const auto unknownTypeResult = planner.createExecutionPlan(
        PlanningResult::shotPlan(unknownType), config, checks);
    tests.expectTrue(
        unknownTypeResult.status() == ExecutionPlanStatus::InvalidShotPlan &&
        !unknownTypeResult.value(),
        "unknown ShotPlanType fails closed without a DirectPot envelope fallback");

    std::array<std::optional<Point>, 9> noBalls{};
    const PlanningResult noPlanInput = BilliardAlgorithm::planShot(
        state({500.0, 250.0}, noBalls),
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        brainConfig());
    tests.expectTrue(planner.createExecutionPlan(noPlanInput, config, checks).status() ==
        ExecutionPlanStatus::InvalidShotPlan,
        "Phase 1 NoPlan cannot be revived into an ExecutionPlan");

    return tests.exitCode();
}
