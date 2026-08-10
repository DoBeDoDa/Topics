#include "TestHarness.h"

#include "../src/BilliardConfig.h"
#include "../src/SocketClient.h"
#include "../src/VisionDataParser.h"

#include <chrono>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<std::string> validTokens()
{
    std::vector<std::string> tokens;
    tokens.reserve(32);
    for (int index = 0; index < 32; ++index) {
        tokens.push_back(std::to_string(100 + index));
    }
    return tokens;
}

std::string join(const std::vector<std::string>& tokens)
{
    std::ostringstream stream;
    for (std::size_t index = 0; index < tokens.size(); ++index) {
        if (index != 0) {
            stream << ',';
        }
        stream << tokens[index];
    }
    return stream.str();
}

void expectParserStatus(
    TestHarness& tests,
    const VisionDataParser& parser,
    const std::vector<std::string>& tokens,
    SingleFrameStatus expected,
    const char* message)
{
    const SingleFrameResult result = parser.parse(join(tokens));
    tests.expectTrue(result.isValid(), "SingleFrameResult invariant");
    tests.expectTrue(result.status() == expected, message);
    tests.expectTrue(
        result.value().has_value() == (expected == SingleFrameStatus::Success),
        "only parser success carries a value");
}

}  // namespace

int main()
{
    TestHarness tests;
    const AxisAlignedBounds2D testObservationBounds{-850.0, 850.0, -850.0, 850.0};
    const VisionDataParser parser(testObservationBounds);

    expectParserStatus(
        tests,
        VisionDataParser(std::optional<AxisAlignedBounds2D>{}),
        validTokens(),
        SingleFrameStatus::ConfigurationMissing,
        "missing production observation bounds fail closed");

    {
        const auto result = parser.parse(join(validTokens()));
        tests.expectTrue(result.status() == SingleFrameStatus::Success, "32 finite values succeed");
        tests.expectTrue(result.value().has_value(), "success has ValidatedVisionFrame");
        if (result.value()) {
            tests.expectNear(result.value()->objectBalls[0]->x, 100.0, 0.0, "Base0 X is unchanged");
            tests.expectNear(result.value()->cueBall.x, 118.0, 0.0, "cue-ball X is unchanged");
            tests.expectNear(result.value()->pockets[5].y, 131.0, 0.0, "pocket Y is unchanged");
        }
    }

    {
        auto tokens = validTokens();
        tokens.pop_back();
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::WrongFieldCount, "31 fields reject");
        tokens.push_back("131");
        tokens.push_back("132");
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::WrongFieldCount, "33 fields reject");
    }

    {
        auto tokens = validTokens();
        tokens[4].clear();
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::EmptyToken, "empty token rejects");
        tokens = validTokens();
        tokens[4] = "12abc";
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::InvalidNumericToken, "trailing numeric text rejects");
        tokens[4] = " 12.5 \t";
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::Success, "outer ASCII whitespace is accepted consistently");
    }

    {
        auto tokens = validTokens();
        tokens[0] = "nan";
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::NonFiniteValue, "NaN rejects");
        tokens[0] = "+inf";
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::NonFiniteValue, "+Infinity rejects");
        tokens[0] = "-inf";
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::NonFiniteValue, "-Infinity rejects");
        tokens[0] = "1e99999";
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::NumericOverflow, "numeric overflow rejects");
    }

    {
        auto tokens = validTokens();
        tokens[0] = "-9999";
        tokens[1] = "-9999.0";
        const auto result = parser.parse(join(tokens));
        tests.expectTrue(result.status() == SingleFrameStatus::Success, "paired sentinel succeeds");
        tests.expectTrue(result.value() && !result.value()->objectBalls[0], "paired sentinel becomes nullopt");

        tokens = validTokens();
        tokens[0] = "-9999";
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::InvalidSentinelPair, "X-only sentinel rejects");
        tokens = validTokens();
        tokens[1] = "-9999";
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::InvalidSentinelPair, "Y-only sentinel rejects");
    }

    {
        auto tokens = validTokens();
        for (std::size_t index = 0; index < 18; ++index) {
            tokens[index] = "-9999";
        }
        const auto result = parser.parse(join(tokens));
        tests.expectTrue(result.status() == SingleFrameStatus::Success, "all numbered balls absent still succeeds");
        if (result.value()) {
            bool allAbsent = true;
            for (const auto& ball : result.value()->objectBalls) {
                allAbsent = allAbsent && !ball.has_value();
            }
            tests.expectTrue(allAbsent, "all numbered balls remain explicit absence");
        }
    }

    {
        auto tokens = validTokens();
        tokens[18] = "-9999";
        tokens[19] = "-9999";
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::MissingRequiredCueBall, "missing cue ball rejects");
        tokens = validTokens();
        tokens[24] = "-9999";
        tokens[25] = "-9999";
        expectParserStatus(tests, parser, tokens, SingleFrameStatus::MissingRequiredPocket, "missing pocket rejects");
    }

    {
        const VisionDataParser invalidBoundsParser({1.0, -1.0, 0.0, 1.0});
        tests.expectTrue(
            invalidBoundsParser.configurationStatus() == SingleFrameStatus::InvalidConfiguration,
            "invalid observation bounds are rejected during application preflight");
        expectParserStatus(
            tests,
            invalidBoundsParser,
            validTokens(),
            SingleFrameStatus::InvalidConfiguration,
            "reversed observation bounds reject configuration");
        auto tokens = validTokens();
        tokens[0] = "851";
        expectParserStatus(
            tests,
            parser,
            tokens,
            SingleFrameStatus::OutOfObservationBounds,
            "present point outside approved observation bounds rejects");

        const VisionDataParser missingBoundsParser(std::nullopt);
        tests.expectTrue(
            missingBoundsParser.configurationStatus() == SingleFrameStatus::ConfigurationMissing,
            "missing observation bounds remain ConfigurationMissing during preflight");
    }

    {
        NewlineFrameBuffer missingMaximum(0);
        tests.expectTrue(
            missingMaximum.append("1\n", 2) == FrameBufferStatus::InvalidConfiguration,
            "explicit zero maximum frame bytes is invalid configuration");

        SocketClient missingSocketConfiguration;
        tests.expectTrue(
            missingSocketConfiguration.configurationStatus() ==
                SocketConfigurationStatus::ConfigurationMissing,
            "missing frame maximum and receive timeout remain ConfigurationMissing");
        tests.expectTrue(
            missingSocketConfiguration.connectToServerResult("127.0.0.1", 5000).status ==
                SocketConnectStatus::ConfigurationMissing,
            "missing socket configuration is named before any connection attempt");

        SocketClient invalidMaximum(0, 1000);
        tests.expectTrue(
            invalidMaximum.configurationStatus() == SocketConfigurationStatus::InvalidConfiguration,
            "explicit zero maximum frame bytes is InvalidConfiguration");
        SocketClient invalidTimeout(64, 0);
        tests.expectTrue(
            invalidTimeout.configurationStatus() == SocketConfigurationStatus::InvalidConfiguration,
            "explicit zero receive timeout is InvalidConfiguration");

        NewlineFrameBuffer framing(64);
        tests.expectTrue(framing.append("1,2", 3) == FrameBufferStatus::Accepted, "partial frame accepted");
        tests.expectTrue(framing.partialFrameBytes() == 3, "partial byte count is bounded and observable");
        std::string frame;
        tests.expectFalse(framing.popFrame(frame), "partial frame is not emitted");
        tests.expectTrue(framing.append(",3\n", 3) == FrameBufferStatus::Accepted, "split frame completes");
        tests.expectTrue(framing.popFrame(frame) && frame == "1,2,3", "split frame reassembles");

        tests.expectTrue(framing.append("a\nb\n", 4) == FrameBufferStatus::Accepted, "multiple frames accepted");
        tests.expectTrue(framing.popFrame(frame) && frame == "a", "first coalesced frame emitted");
        tests.expectTrue(framing.popFrame(frame) && frame == "b", "second coalesced frame emitted");

        framing.reset();
        const std::string exactLimit(64, '6');
        tests.expectTrue(
            framing.append((exactLimit + "\n").data(), exactLimit.size() + 1) == FrameBufferStatus::Accepted,
            "frame exactly at configured maximum succeeds");
        tests.expectTrue(framing.popFrame(frame) && frame == exactLimit, "maximum-sized frame is emitted intact");

        const std::string oversized(65, '7');
        tests.expectTrue(
            framing.append(oversized.data(), oversized.size()) == FrameBufferStatus::FrameTooLong,
            "unfinished oversized frame fails closed");
        framing.reset();
        tests.expectTrue(framing.append("ok\n", 3) == FrameBufferStatus::Accepted, "reset accepts next frame");
        tests.expectTrue(framing.popFrame(frame) && frame == "ok", "oversized frame does not contaminate next frame");

        tests.expectTrue(framing.append("stale", 5) == FrameBufferStatus::Accepted, "old connection partial frame buffered");
        framing.reset();
        tests.expectTrue(framing.append("fresh\n", 6) == FrameBufferStatus::Accepted, "reconnect reset accepts fresh frame");
        tests.expectTrue(framing.popFrame(frame) && frame == "fresh", "old partial frame cannot cross reconnect");
    }

    {
        tests.expectTrue(
            classifyTransportOutcome(0, 0) == SocketReceiveStatus::CleanClose,
            "clean peer close is distinct");
        tests.expectTrue(
            classifyTransportOutcome(SOCKET_ERROR, WSAETIMEDOUT) == SocketReceiveStatus::TimedOut,
            "receive timeout is distinct");
        tests.expectTrue(
            classifyTransportOutcome(SOCKET_ERROR, WSAECONNRESET) == SocketReceiveStatus::SocketError,
            "socket error is distinct");

        LocalConnectionLifecycle connections;
        const auto first = connections.open();
        connections.invalidate();
        const auto second = connections.open();
        tests.expectTrue(first != 0 && second > first, "reconnect receives a new identity");
        tests.expectTrue(connections.current() == second, "new connection is current");

        SocketReceiveState receiveState(64);
        const ConnectionIdentity receiveConnection1 = receiveState.openConnection();
        tests.expectTrue(receiveState.append("old", 3) == FrameBufferStatus::Accepted, "Socket receive state buffers partial bytes");
        receiveState.invalidateConnection();
        const ConnectionIdentity receiveConnection2 = receiveState.openConnection();
        tests.expectTrue(receiveConnection2 > receiveConnection1, "Socket receive state reconnect changes identity");
        tests.expectTrue(receiveState.append("fresh\n", 6) == FrameBufferStatus::Accepted, "new connection accepts fresh bytes");
        std::string frame;
        tests.expectTrue(receiveState.popFrame(frame) && frame == "fresh", "Socket reconnect atomically drops old partial bytes");
        tests.expectTrue(receiveState.append("discard", 7) == FrameBufferStatus::Accepted, "Socket receive state buffers pre-flush bytes");
        receiveState.flush();
        tests.expectTrue(receiveState.append("current\n", 8) == FrameBufferStatus::Accepted, "Socket receive state accepts post-flush bytes");
        tests.expectTrue(receiveState.popFrame(frame) && frame == "current", "Socket flush atomically drops stale bytes");
    }

    {
        ReceiveEventFactory events(parser);
        const ConnectionIdentity connection = 10;
        const ShotCycleIdentity firstCycle = 20;
        const auto now = std::chrono::steady_clock::now();
        events.beginCycle(connection, firstCycle);
        tests.expectTrue(events.hasActiveCycle(), "beginCycle creates active local cycle");
        tests.expectTrue(events.currentCycleIdentity() == firstCycle, "cycle identity is retained");
        tests.expectFalse(events.isCaptureWindowOpen(), "capture starts closed");

        auto result = events.accept(join(validTokens()), connection, now);
        tests.expectTrue(result.status() == ReceiveEventStatus::CaptureWindowClosed, "pre-window frame is discarded");
        tests.expectFalse(result.value().has_value(), "discard has no ReceiveEvent");

        tests.expectTrue(events.openCaptureWindow(connection, firstCycle), "capture window opens for active identity");
        tests.expectTrue(events.isCaptureWindowOpen(), "capture window state is observable");
        const auto first = events.accept(join(validTokens()), connection, now);
        const auto second = events.accept(join(validTokens()), connection, now + std::chrono::milliseconds(1));
        tests.expectTrue(first.status() == ReceiveEventStatus::Success, "first event succeeds");
        tests.expectTrue(second.status() == ReceiveEventStatus::Success, "second identical payload event succeeds");
        if (first.value() && second.value()) {
            tests.expectTrue(second.value()->eventId > first.value()->eventId, "local event IDs strictly increase");
            tests.expectTrue(second.value()->receivedAt > first.value()->receivedAt, "monotonic time is preserved");
        }

        events.beginCycle(connection, firstCycle + 1);
        tests.expectTrue(
            events.accept(join(validTokens()), connection, now).status() == ReceiveEventStatus::CaptureWindowClosed,
            "cycle change invalidates old capture window");
        tests.expectFalse(events.openCaptureWindow(connection, firstCycle), "old cycle cannot reopen current window");
        tests.expectTrue(events.openCaptureWindow(connection, firstCycle + 1), "new cycle can open window");
        tests.expectTrue(
            events.accept(join(validTokens()), connection + 1, now).status() == ReceiveEventStatus::ConnectionMismatch,
            "old/new connection events cannot mix");

        events.invalidate(ReceiveEventInvalidationReason::ExplicitFlush);
        tests.expectTrue(
            events.accept(join(validTokens()), connection, now).status() == ReceiveEventStatus::NoActiveCycle,
            "explicit flush invalidates old cycle events");
        events.beginCycle(connection, firstCycle + 2);
        tests.expectTrue(events.openCaptureWindow(connection, firstCycle + 2), "capture can restart after flush");

        auto invalidTokens = validTokens();
        invalidTokens[0] = "-9999";
        const auto invalid = events.accept(join(invalidTokens), connection, now);
        tests.expectTrue(invalid.status() == ReceiveEventStatus::FrameRejected, "parser failure rejects event");
        tests.expectTrue(invalid.resetRequired(), "parser failure requests downstream reset");
        tests.expectTrue(
            events.lastInvalidationReason() == ReceiveEventInvalidationReason::ParserFailure,
            "parser failure records its invalidation reason");
        tests.expectTrue(
            events.accept(join(validTokens()), connection, now).status() == ReceiveEventStatus::NoActiveCycle,
            "parser failure invalidates the old cycle before another event");

        events.beginCycle(connection, firstCycle + 3);
        tests.expectTrue(events.openCaptureWindow(connection, firstCycle + 3), "new Start cycle can follow parser failure");
        events.invalidate(ReceiveEventInvalidationReason::Timeout);
        tests.expectTrue(
            events.accept(join(validTokens()), connection, now).status() == ReceiveEventStatus::NoActiveCycle,
            "timeout invalidates old event lifecycle");
    }

    return tests.exitCode();
}
