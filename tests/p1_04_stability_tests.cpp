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

    std::array<std::optional<Point>, 6> pockets{};
    for (std::size_t index = 0; index < pockets.size(); ++index) {
        pockets[index] = Point{
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
        // cueBall超出容差：滑動視窗設計下不再整批reset，改成NeedMoreEvents
        // 並保留視窗繼續往前滑（cueBall/袋口是StableTableState的必要欄位，
        // 沒收斂就還不能送出任何結果，但也不因此丟棄已經累積的3筆事件）。
        ThreeEventStability stability(validConfig(1.0, 1.0, 100));
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame({0.0, 0.0}))).status() == StabilityStatus::NeedMoreEvents, "jump fixture first event waits");
        tests.expectTrue(stability.accept(makeEvent(2, 10, makeFrame({0.0, 0.0}))).status() == StabilityStatus::NeedMoreEvents, "jump fixture second event waits");
        const auto jump = stability.accept(makeEvent(3, 20, makeFrame({3.0, 0.0})));
        tests.expectTrue(jump.status() == StabilityStatus::NeedMoreEvents, "ball jump keeps sliding instead of hard-resetting");
        expectNoSuccessValue(tests, jump, "jump has no partial stable value");
        tests.expectTrue(
            jump.diagnostic() && jump.diagnostic()->reason == StabilityFailureReason::BallMoved,
            "jump result names the precise reason directly on the diagnostic");
        tests.expectTrue(stability.accumulatedEventCount() == 3, "jump does not clear the sliding window");
        const auto waitingPipelineResult = Phase1PipelineResult::fromStability(jump);
        tests.expectTrue(
            waitingPipelineResult &&
                waitingPipelineResult->status() == Phase1PipelineStatus::Waiting &&
                waitingPipelineResult->isValid(),
            "NeedMoreEvents (even from a not-yet-converged jump) maps to pipeline Waiting, "
            "not StabilityFailure");
        tests.expectTrue(
            stability.accept(makeEvent(4, 30, makeFrame())).status() == StabilityStatus::NeedMoreEvents,
            "next event slides the window and still has not converged");
    }

    {
        // 母球在單幀parse階段跟編號球/袋口一樣可以缺席（不再FrameRejected），
        // 但母球在StableTableState裡仍是必要欄位：只要目前3事件視窗內有
        // 任一筆缺母球，就還不能定案，維持NeedMoreEvents繼續滑動；直到那筆
        // flicker事件被滑出視窗、視窗內3筆都有母球且一致時，才送出Stable。
        ThreeEventStability stability(validConfig());
        auto flickeringFrame = makeFrame();
        flickeringFrame.cueBall = std::nullopt;
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "cue-ball-flicker fixture first event waits");
        const auto duringFlicker = stability.accept(makeEvent(2, 10, std::move(flickeringFrame)));
        tests.expectTrue(
            duringFlicker.status() == StabilityStatus::NeedMoreEvents,
            "a cue ball missing from just one window event keeps sliding, unlike numbered balls it cannot go Stable with a hole");
        expectNoSuccessValue(tests, duringFlicker, "cue-ball flicker mid-window has no partial stable value");
        tests.expectTrue(stability.accumulatedEventCount() == 2, "cue-ball flicker does not clear the sliding window");

        const auto stillInWindow3 = stability.accept(makeEvent(3, 20, makeFrame()));
        tests.expectTrue(
            stillInWindow3.status() == StabilityStatus::NeedMoreEvents,
            "the flickering event is still inside the 3-window after event 3");
        tests.expectTrue(
            stillInWindow3.diagnostic() && stillInWindow3.diagnostic()->reason == StabilityFailureReason::BallMoved,
            "unresolved cue ball names BallMoved directly on the diagnostic");
        tests.expectTrue(stability.accumulatedEventCount() == 3, "still-unresolved cue ball does not clear the sliding window");

        const auto stillInWindow4 = stability.accept(makeEvent(4, 30, makeFrame()));
        tests.expectTrue(
            stillInWindow4.status() == StabilityStatus::NeedMoreEvents,
            "the flickering event (#2) is still inside the 3-window after event 4 ([2,3,4])");
        tests.expectTrue(stability.accumulatedEventCount() == 3, "sliding window keeps its size at 3, not cleared");

        const auto recovered = stability.accept(makeEvent(5, 40, makeFrame()));
        tests.expectTrue(
            recovered.status() == StabilityStatus::Stable,
            "once the flickering event slides out of the 3-window, a fully-present agreeing window becomes Stable");
        if (recovered.value()) {
            tests.expectNear(
                recovered.value()->cueBall.x, 100.0, 0.0,
                "recovered cue ball reports the converged coordinate, not a stale or guessed position");
        }
    }

    {
        // 一顆球在單一事件缺席（跟母球/袋口一樣）：Item 4C——三事件視窗
        // 內presence不一致（mixed present/absent）時，這一輪還不能定案，
        // 必須維持NeedMoreEvents繼續滑動，絕不能把那顆球單獨收斂成
        // "unknown/nullopt"、放行其餘欄位照常送出Stable（那樣會讓一顆
        // 其實還在桌上、只是這一瞬間被遮擋/漏偵測的球，被Phase1誤判成
        // 「確定不在桌上」）。直到flicker事件被滑出視窗、視窗內3筆
        // presence一致時，才送出Stable——跟母球/袋口既有行為完全對齊。
        ThreeEventStability stability(validConfig());
        auto flickeringFrame = makeFrame({100.0, 200.0}, std::nullopt);
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "ball-flicker fixture first event waits");
        const auto duringFlicker = stability.accept(makeEvent(2, 10, std::move(flickeringFrame)));
        tests.expectTrue(
            duringFlicker.status() == StabilityStatus::NeedMoreEvents,
            "a ball missing from just one window event keeps sliding; mixed presence must never collapse into a stable absent");
        expectNoSuccessValue(tests, duringFlicker, "ball flicker mid-window has no partial stable value");
        tests.expectTrue(stability.accumulatedEventCount() == 2, "ball flicker does not clear the sliding window");

        const auto stillInWindow3 = stability.accept(makeEvent(3, 20, makeFrame()));
        tests.expectTrue(
            stillInWindow3.status() == StabilityStatus::NeedMoreEvents,
            "the flickering event is still inside the 3-window after event 3");
        tests.expectTrue(
            stillInWindow3.diagnostic() && stillInWindow3.diagnostic()->reason == StabilityFailureReason::BallMoved,
            "unresolved ball presence names BallMoved directly on the diagnostic, same as cue-ball/pocket");
        tests.expectTrue(stability.accumulatedEventCount() == 3, "still-unresolved ball does not clear the sliding window");

        const auto stillInWindow4 = stability.accept(makeEvent(4, 30, makeFrame()));
        tests.expectTrue(
            stillInWindow4.status() == StabilityStatus::NeedMoreEvents,
            "the flickering event (#2) is still inside the 3-window after event 4 ([2,3,4])");
        tests.expectTrue(stability.accumulatedEventCount() == 3, "sliding window keeps its size at 3, not cleared");

        const auto recovered = stability.accept(makeEvent(5, 40, makeFrame()));
        tests.expectTrue(
            recovered.status() == StabilityStatus::Stable,
            "once the flickering event slides out of the 3-window, a fully-present agreeing window becomes Stable");
        if (recovered.value()) {
            tests.expectTrue(
                recovered.value()->objectBalls[0].has_value(),
                "the previously-flickering ball is reported present once the window fully agrees again, "
                "never silently downgraded to absent");
            tests.expectNear(
                recovered.value()->cueBall.x, 100.0, 0.0,
                "cueBall, unaffected by the flicker, is still reported correctly");
        }
        tests.expectTrue(
            stability.accumulatedEventCount() == 0,
            "a Stable result closes and resets the window");
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
        // 袋口是StableTableState的必要（非optional）欄位，跟cueBall一樣沒
        // 收斂就不能送出結果，但一樣不整批reset——NeedMoreEvents、視窗保留。
        ThreeEventStability stability(validConfig(2.0, 0.5, 100));
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "pocket fixture first event waits");
        tests.expectTrue(stability.accept(makeEvent(2, 10, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "pocket fixture second event waits");
        const auto pocketJump = stability.accept(makeEvent(3, 20, makeFrame({100.0, 200.0}, Point{300.0, 400.0}, 2.0)));
        tests.expectTrue(
            pocketJump.status() == StabilityStatus::NeedMoreEvents,
            "pocket beyond its separate (tighter) tolerance keeps sliding instead of hard-resetting");
        tests.expectTrue(
            pocketJump.diagnostic() && pocketJump.diagnostic()->reason == StabilityFailureReason::PocketMoved,
            "pocket jump names the precise reason directly on the diagnostic");
        tests.expectTrue(stability.accumulatedEventCount() == 3, "pocket jump does not clear the sliding window");
    }

    {
        // 袋口在單幀parse階段跟編號球一樣可以缺席（不再FrameRejected），
        // 但袋口在StableTableState裡仍是必要欄位：只要目前3事件視窗內有
        // 任一筆缺這顆袋口，就還不能定案，維持NeedMoreEvents繼續滑動；
        // 直到那筆flicker事件被滑出視窗、視窗內3筆都有該袋口且一致時，
        // 才送出Stable。
        ThreeEventStability stability(validConfig());
        auto flickeringFrame = makeFrame();
        flickeringFrame.pockets[3] = std::nullopt;
        tests.expectTrue(stability.accept(makeEvent(1, 0, makeFrame())).status() == StabilityStatus::NeedMoreEvents, "pocket-flicker fixture first event waits");
        const auto duringFlicker = stability.accept(makeEvent(2, 10, std::move(flickeringFrame)));
        tests.expectTrue(
            duringFlicker.status() == StabilityStatus::NeedMoreEvents,
            "a pocket missing from just one window event keeps sliding, unlike numbered balls it cannot go Stable with a hole");
        expectNoSuccessValue(tests, duringFlicker, "pocket flicker mid-window has no partial stable value");
        tests.expectTrue(stability.accumulatedEventCount() == 2, "pocket flicker does not clear the sliding window");

        // 事件3、4：flicker事件（#2）仍在視窗內（視窗依序是[1,2,3]→[2,3,4]），
        // 兩次都還無法定案。
        const auto stillInWindow3 = stability.accept(makeEvent(3, 20, makeFrame()));
        tests.expectTrue(
            stillInWindow3.status() == StabilityStatus::NeedMoreEvents,
            "the flickering event is still inside the 3-window after event 3");
        tests.expectTrue(
            stillInWindow3.diagnostic() && stillInWindow3.diagnostic()->reason == StabilityFailureReason::PocketMoved,
            "unresolved pocket names PocketMoved directly on the diagnostic");
        tests.expectTrue(stability.accumulatedEventCount() == 3, "still-unresolved pocket does not clear the sliding window");

        const auto stillInWindow4 = stability.accept(makeEvent(4, 30, makeFrame()));
        tests.expectTrue(
            stillInWindow4.status() == StabilityStatus::NeedMoreEvents,
            "the flickering event (#2) is still inside the 3-window after event 4 ([2,3,4])");
        tests.expectTrue(stability.accumulatedEventCount() == 3, "sliding window keeps its size at 3, not cleared");

        // 事件5：視窗滑成[3,4,5]，flicker事件(#2)終於被滑出去，3筆都有
        // 且一致，送出Stable。
        const auto recovered = stability.accept(makeEvent(5, 40, makeFrame()));
        tests.expectTrue(
            recovered.status() == StabilityStatus::Stable,
            "once the flickering event slides out of the 3-window, a fully-present agreeing window becomes Stable");
        if (recovered.value()) {
            tests.expectNear(
                recovered.value()->pockets[3].x, 530.0, 0.0,
                "recovered pocket reports the converged coordinate, not a stale or guessed position");
        }
    }

    {
        ThreeEventStability stability(validConfig());
        auto invalidFrame = makeFrame();
        invalidFrame.cueBall->x = std::numeric_limits<double>::quiet_NaN();
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
