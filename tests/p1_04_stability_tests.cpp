#include "TestHarness.h"

#include "../src/TableState.h"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

namespace {

using Clock = std::chrono::steady_clock;

ValidatedVisionFrame makeFrame(
    Point cueBall = {100.0, 200.0},
    std::optional<Point> firstBall = Point{300.0, 400.0},
    double pocketOffset = 0.0)
{
    std::array<std::optional<Point>, 9> balls{};
    balls[0] = firstBall;

    std::array<Point, 6> pockets{};
    for (std::size_t index = 0; index < pockets.size(); ++index) {
        pockets[index] = {
            500.0 + static_cast<double>(index) * 10.0 + pocketOffset,
            600.0 + static_cast<double>(index) * 10.0};
    }
    return ValidatedVisionFrame{balls, cueBall, pockets};
}

ReceiveEvent makeEvent(
    ReceiveEventId eventId,
    long long receiveMilliseconds,
    ValidatedVisionFrame frame,
    ConnectionIdentity connectionIdentity = 10,
    ShotCycleIdentity shotCycleIdentity = 20)
{
    return {
        connectionIdentity,
        shotCycleIdentity,
        eventId,
        Clock::time_point{std::chrono::milliseconds{receiveMilliseconds}},
        std::move(frame)};
}

StabilityConfig validConfig(
    double ballToleranceMm = 2.0,
    double pocketToleranceMm = 1.0,
    long long maximumIntervalMilliseconds = 100)
{
    return {
        ballToleranceMm,
        pocketToleranceMm,
        std::chrono::milliseconds{maximumIntervalMilliseconds}};
}

void expectNoSuccessValue(
    TestHarness& tests,
    const StabilityResult& result,
    const char* message)
{
    tests.expectFalse(result.value().has_value(), message);
    tests.expectTrue(result.diagnostic().has_value(), message);
    tests.expectTrue(result.isValid(), message);
}

}  // namespace

int main()
{
    TestHarness tests;

    static_assert(!std::is_constructible_v<ThreeEventStability, SingleFrameResult>);

    {
        ThreeEventStability stability({std::nullopt, 1.0, std::chrono::milliseconds{100}});
        const auto result = stability.accept(makeEvent(1, 0, makeFrame()));
        tests.expectTrue(result.status() == StabilityStatus::InvalidConfiguration, "missing tolerance rejects");
        tests.expectTrue(
            result.diagnostic() &&
                result.diagnostic()->reason == StabilityFailureReason::ConfigurationMissing,
            "missing tolerance has named diagnostic");
        expectNoSuccessValue(tests, result, "missing configuration has no stable value");
    }

    {
        ThreeEventStability negativeTolerance(validConfig(-1.0, 1.0, 100));
        tests.expectTrue(
            negativeTolerance.accept(makeEvent(1, 0, makeFrame())).status() ==
                StabilityStatus::InvalidConfiguration,
            "negative tolerance rejects");
        ThreeEventStability nonFiniteTolerance(
            validConfig(std::numeric_limits<double>::infinity(), 1.0, 100));
        tests.expectTrue(
            nonFiniteTolerance.accept(makeEvent(1, 0, makeFrame())).status() ==
                StabilityStatus::InvalidConfiguration,
            "nonfinite tolerance rejects");
        ThreeEventStability zeroInterval(validConfig(1.0, 1.0, 0));
        tests.expectTrue(
            zeroInterval.accept(makeEvent(1, 0, makeFrame())).status() ==
                StabilityStatus::InvalidConfiguration,
            "nonpositive maximum interval rejects");
    }

    {
        ThreeEventStability stability(validConfig());
        const auto first = stability.accept(makeEvent(1, 0, makeFrame()));
        const auto second = stability.accept(makeEvent(2, 10, makeFrame()));
        const auto third = stability.accept(makeEvent(3, 20, makeFrame()));
        tests.expectTrue(first.status() == StabilityStatus::NeedMoreEvents, "first event waits");
        tests.expectTrue(second.status() == StabilityStatus::NeedMoreEvents, "second event waits");
        expectNoSuccessValue(tests, first, "first event has no stable value");
        expectNoSuccessValue(tests, second, "second event has no stable value");
        const auto waitingPipelineResult = Phase1PipelineResult::fromStability(first);
        tests.expectTrue(
            waitingPipelineResult &&
                waitingPipelineResult->status() == Phase1PipelineStatus::Waiting &&
                waitingPipelineResult->isValid(),
            "NeedMoreEvents maps to pipeline Waiting without planning result");
        tests.expectTrue(third.status() == StabilityStatus::Stable, "third identical event is stable");
        tests.expectTrue(third.value().has_value(), "stable status carries stable value");
        tests.expectTrue(third.isValid(), "stable result invariant holds");
        tests.expectFalse(
            Phase1PipelineResult::fromStability(third).has_value(),
            "Stable continues to the future ShotBrain seam instead of becoming a nonplanning pipeline result");
        if (third.value()) {
            tests.expectTrue(third.value()->connectionIdentity == 10, "stable connection identity preserved");
            tests.expectTrue(third.value()->shotCycleIdentity == 20, "stable cycle identity preserved");
            tests.expectTrue(third.value()->sourceEvents[0].eventId == 1, "first source event preserved");
            tests.expectTrue(third.value()->sourceEvents[2].eventId == 3, "third source event preserved");
        }
        tests.expectTrue(stability.accumulatedEventCount() == 0, "stable output closes the three-event window");
        tests.expectTrue(
            stability.accept(makeEvent(4, 30, makeFrame())).status() == StabilityStatus::NeedMoreEvents,
            "event after stable starts a new accumulation");
    }

    {
        ThreeEventStability stability(validConfig(1.5, 1.0, 100));
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame({0.0, 2.0}))).status() == StabilityStatus::NeedMoreEvents, "median fixture first event waits");
        tests.expectTrue(stability.accept(makeEvent(2, 10, makeFrame({1.0, 0.0}))).status() == StabilityStatus::NeedMoreEvents, "median fixture second event waits");
        const auto result = stability.accept(makeEvent(3, 20, makeFrame({2.0, 1.0})));
        tests.expectTrue(result.status() == StabilityStatus::Stable, "within tolerance is stable");
        if (result.value()) {
            tests.expectNear(result.value()->cueBall.x, 1.0, 0.0, "X median can come from second event");
            tests.expectNear(result.value()->cueBall.y, 1.0, 0.0, "Y median can come from third event");
        }
    }

    {
        ThreeEventStability stability(validConfig(1.0, 1.0, 100));
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame({-1.0, 0.0}))).status() == StabilityStatus::NeedMoreEvents, "boundary fixture first event waits");
        tests.expectTrue(stability.accept(makeEvent(2, 10, makeFrame({0.0, 0.0}))).status() == StabilityStatus::NeedMoreEvents, "boundary fixture second event waits");
        const auto result = stability.accept(makeEvent(3, 20, makeFrame({1.0, 0.0})));
        tests.expectTrue(result.status() == StabilityStatus::Stable, "distance equal to tolerance passes");
    }

    {
        ThreeEventStability stability(validConfig(2.0, 1.0, 100));
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "interval boundary first event waits");
        tests.expectTrue(stability.accept(makeEvent(2, 100, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "interval equal to maximum passes");
        tests.expectTrue(stability.accept(makeEvent(3, 200, makeFrame())).status() == StabilityStatus::Stable, "both adjacent intervals at boundary are stable");
    }

    {
        ThreeEventStability stability(validConfig(2.0, 1.5, 100));
        auto firstFrame = makeFrame();
        auto secondFrame = makeFrame();
        auto thirdFrame = makeFrame();
        firstFrame.pockets[0] = {0.0, 2.0};
        secondFrame.pockets[0] = {1.0, 0.0};
        thirdFrame.pockets[0] = {2.0, 1.0};
        tests.expectTrue(stability.accept(makeEvent(1, 0, std::move(firstFrame))).status() == StabilityStatus::NeedMoreEvents, "pocket median first event waits");
        tests.expectTrue(stability.accept(makeEvent(2, 10, std::move(secondFrame))).status() == StabilityStatus::NeedMoreEvents, "pocket median second event waits");
        const auto result = stability.accept(makeEvent(3, 20, std::move(thirdFrame)));
        tests.expectTrue(result.status() == StabilityStatus::Stable, "pocket coordinates within tolerance are stable");
        if (result.value()) {
            tests.expectNear(result.value()->pockets[0].x, 1.0, 0.0, "pocket X median is retained");
            tests.expectNear(result.value()->pockets[0].y, 1.0, 0.0, "pocket Y median is retained");
        }
    }

    {
        ThreeEventStability stability(validConfig(1.0, 1.0, 100));
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame({0.0, 0.0}))).status() == StabilityStatus::NeedMoreEvents, "jump fixture first event waits");
        tests.expectTrue(stability.accept(makeEvent(2, 10, makeFrame({0.0, 0.0}))).status() == StabilityStatus::NeedMoreEvents, "jump fixture second event waits");
        const auto jump = stability.accept(makeEvent(3, 20, makeFrame({3.0, 0.0})));
        tests.expectTrue(jump.status() == StabilityStatus::Unstable, "ball jump is unstable");
        expectNoSuccessValue(tests, jump, "jump has no partial stable value");
        tests.expectTrue(stability.accumulatedEventCount() == 0, "jump completely resets");
        tests.expectTrue(stability.lastResetReason() == StabilityFailureReason::BallMoved, "jump reset preserves precise reason");
        const auto failurePipelineResult = Phase1PipelineResult::fromStability(jump);
        tests.expectTrue(
            failurePipelineResult &&
                failurePipelineResult->status() == Phase1PipelineStatus::StabilityFailure &&
                failurePipelineResult->isValid(),
            "Unstable maps to pipeline StabilityFailure, not a planning result");
        tests.expectTrue(
            stability.accept(makeEvent(4, 30, makeFrame())).status() == StabilityStatus::NeedMoreEvents,
            "event after jump becomes first event");
    }

    {
        ThreeEventStability stability(validConfig());
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "presence fixture first event waits");
        tests.expectTrue(stability.accept(makeEvent(2, 10, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "presence fixture second event waits");
        const auto changed = stability.accept(makeEvent(3, 20, makeFrame({100.0, 200.0}, std::nullopt)));
        tests.expectTrue(changed.status() == StabilityStatus::Unstable, "presence change is unstable");
        tests.expectTrue(
            changed.diagnostic() && changed.diagnostic()->reason == StabilityFailureReason::PresenceChanged,
            "presence change has named reason");
        tests.expectTrue(stability.accumulatedEventCount() == 0, "presence change completely resets");
        tests.expectTrue(stability.lastResetReason() == StabilityFailureReason::PresenceChanged, "presence reset preserves precise reason");
    }

    {
        ThreeEventStability stability(validConfig());
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame({100.0, 200.0}, std::nullopt))).status() == StabilityStatus::NeedMoreEvents, "zero-ball first event waits");
        tests.expectTrue(stability.accept(makeEvent(2, 10, makeFrame({100.0, 200.0}, std::nullopt))).status() == StabilityStatus::NeedMoreEvents, "zero-ball second event waits");
        const auto result = stability.accept(makeEvent(3, 20, makeFrame({100.0, 200.0}, std::nullopt)));
        tests.expectTrue(result.status() == StabilityStatus::Stable, "all numbered balls absent can be stable");
        if (result.value()) {
            for (const auto& ball : result.value()->objectBalls) {
                tests.expectFalse(ball.has_value(), "absent numbered ball stays absent without default Point");
            }
        }
    }

    {
        ThreeEventStability stability(validConfig());
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "cross-cycle first event waits");
        const auto crossCycle = stability.accept(makeEvent(2, 10, makeFrame(), 10, 21));
        tests.expectTrue(crossCycle.status() == StabilityStatus::Unstable, "cross-cycle event rejects");
        tests.expectTrue(stability.accumulatedEventCount() == 0, "cross-cycle event resets");

        tests.expectTrue(stability.accept(makeEvent(3, 20, makeFrame(), 10, 22)).status() == StabilityStatus::NeedMoreEvents, "cross-connection first event waits");
        const auto crossConnection = stability.accept(makeEvent(4, 30, makeFrame(), 11, 22));
        tests.expectTrue(crossConnection.status() == StabilityStatus::Unstable, "cross-connection event rejects");
        tests.expectTrue(stability.accumulatedEventCount() == 0, "cross-connection event resets");
    }

    {
        ThreeEventStability stability(validConfig(2.0, 1.0, 100));
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "timeout first event waits");
        const auto timeout = stability.accept(makeEvent(2, 101, makeFrame()));
        tests.expectTrue(timeout.status() == StabilityStatus::TimedOut, "interval above maximum times out");
        expectNoSuccessValue(tests, timeout, "timeout has no stable value");
        tests.expectTrue(stability.accumulatedEventCount() == 0, "timeout resets");
        tests.expectTrue(stability.lastResetReason() == StabilityFailureReason::TimedOut, "timeout reset preserves precise reason");
        tests.expectTrue(
            stability.accept(makeEvent(3, 102, makeFrame())).status() == StabilityStatus::NeedMoreEvents,
            "event after timeout becomes first event");
    }

    {
        ThreeEventStability stability(validConfig());
        tests.expectTrue(stability.accept(makeEvent(2, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "event-ID fixture first event waits");
        const auto duplicateId = stability.accept(makeEvent(2, 10, makeFrame()));
        tests.expectTrue(duplicateId.status() == StabilityStatus::Unstable, "event ID must strictly increase");
        tests.expectTrue(stability.accumulatedEventCount() == 0, "bad event ID resets");
    }

    {
        ThreeEventStability stability(validConfig(2.0, 0.5, 100));
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "pocket fixture first event waits");
        tests.expectTrue(stability.accept(makeEvent(2, 10, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "pocket fixture second event waits");
        const auto pocketJump = stability.accept(makeEvent(3, 20, makeFrame({100.0, 200.0}, Point{300.0, 400.0}, 2.0)));
        tests.expectTrue(pocketJump.status() == StabilityStatus::Unstable, "pocket uses separate tolerance");
        tests.expectTrue(
            pocketJump.diagnostic() && pocketJump.diagnostic()->reason == StabilityFailureReason::PocketMoved,
            "pocket jump has named reason");
    }

    {
        ThreeEventStability stability(validConfig());
        auto invalidFrame = makeFrame();
        invalidFrame.cueBall.x = std::numeric_limits<double>::quiet_NaN();
        const auto invalid = stability.accept(makeEvent(1, 0, std::move(invalidFrame)));
        tests.expectTrue(invalid.status() == StabilityStatus::Unstable, "invalid fake event fails closed");
        tests.expectTrue(stability.accumulatedEventCount() == 0, "invalid fake event resets");
    }

    {
        ThreeEventStability stability(validConfig());
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "disconnect fixture first event waits");
        stability.reset(StabilityFailureReason::Disconnected);
        tests.expectTrue(stability.accumulatedEventCount() == 0, "disconnect reset clears events");
        tests.expectTrue(
            stability.accept(makeEvent(2, 10, makeFrame())).status() == StabilityStatus::NeedMoreEvents,
            "event after reconnect reset becomes first event");
    }

    {
        ThreeEventStability stability(validConfig());
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "parser-reset fixture first event waits");
        stability.reset(StabilityFailureReason::ParserFailure);
        tests.expectTrue(stability.accumulatedEventCount() == 0, "parser failure immediately clears accumulation");
        tests.expectTrue(
            stability.accept(makeEvent(2, 10, makeFrame(), 10, 21)).status() == StabilityStatus::NeedMoreEvents,
            "first valid event after parser reset is event one of the new cycle");
        stability.reset(StabilityFailureReason::CycleChanged);
        tests.expectTrue(stability.accumulatedEventCount() == 0, "cycle change immediately clears accumulation");
    }

    {
        const auto inputFailure = Phase1PipelineResult::inputFailure(
            SingleFrameDiagnostic{SingleFrameStatus::InvalidSentinelPair, 0});
        tests.expectTrue(
            inputFailure.status() == Phase1PipelineStatus::InputFailure && inputFailure.isValid(),
            "P1-03 rejection maps to pipeline InputFailure, never NoPlan");
    }

    return tests.exitCode();
}
