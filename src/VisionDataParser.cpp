// 嚴格解析Python既有32值CSV，並建立不含sender metadata的本地ReceiveEvent。
#include "VisionDataParser.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "BilliardConfig.h"

namespace {

constexpr double MISSING_COORDINATE = -9999.0;
constexpr std::size_t EXPECTED_FIELD_COUNT = 32;

bool isAsciiWhitespace(char value)
{
    return value == ' ' || value == '\t' || value == '\r' ||
        value == '\n' || value == '\f' || value == '\v';
}

std::string trimAsciiWhitespace(const std::string& token)
{
    std::size_t begin = 0;
    while (begin < token.size() && isAsciiWhitespace(token[begin])) {
        ++begin;
    }

    std::size_t end = token.size();
    while (end > begin && isAsciiWhitespace(token[end - 1])) {
        --end;
    }
    return token.substr(begin, end - begin);
}

std::vector<std::string> splitPreservingEmptyTokens(const std::string& message)
{
    std::vector<std::string> tokens;
    std::size_t begin = 0;
    while (true) {
        const std::size_t comma = message.find(',', begin);
        if (comma == std::string::npos) {
            tokens.push_back(message.substr(begin));
            return tokens;
        }
        tokens.push_back(message.substr(begin, comma - begin));
        begin = comma + 1;
    }
}

bool validBounds(const AxisAlignedBounds2D& bounds)
{
    return std::isfinite(bounds.minX) && std::isfinite(bounds.maxX) &&
        std::isfinite(bounds.minY) && std::isfinite(bounds.maxY) &&
        bounds.minX <= bounds.maxX && bounds.minY <= bounds.maxY;
}

bool withinBounds(Point point, const AxisAlignedBounds2D& bounds)
{
    return point.x >= bounds.minX && point.x <= bounds.maxX &&
        point.y >= bounds.minY && point.y <= bounds.maxY;
}

struct NumericParseResult {
    SingleFrameStatus status;
    std::optional<double> value;
};

NumericParseResult parseNumber(const std::string& rawToken)
{
    const std::string token = trimAsciiWhitespace(rawToken);
    if (token.empty()) {
        return {SingleFrameStatus::EmptyToken, std::nullopt};
    }

    errno = 0;
    char* end = nullptr;
    const char* begin = token.c_str();
    const double value = std::strtod(begin, &end);
    if (end == begin || end != begin + token.size()) {
        return {SingleFrameStatus::InvalidNumericToken, std::nullopt};
    }
    if (errno == ERANGE) {
        return {SingleFrameStatus::NumericOverflow, std::nullopt};
    }
    if (!std::isfinite(value)) {
        return {SingleFrameStatus::NonFiniteValue, std::nullopt};
    }
    return {SingleFrameStatus::Success, value};
}

struct ParsedPointResult {
    SingleFrameStatus status;
    std::optional<Point> point;
    std::optional<std::size_t> fieldIndex;
};

ParsedPointResult parsePoint(
    const std::vector<double>& values,
    std::size_t fieldIndex,
    const AxisAlignedBounds2D& bounds)
{
    const double x = values[fieldIndex];
    const double y = values[fieldIndex + 1];
    const bool missingX = x == MISSING_COORDINATE;
    const bool missingY = y == MISSING_COORDINATE;
    if (missingX != missingY) {
        return {
            SingleFrameStatus::InvalidSentinelPair,
            std::nullopt,
            fieldIndex};
    }
    if (missingX) {
        return {SingleFrameStatus::Success, std::nullopt, std::nullopt};
    }

    const Point point{x, y};
    if (!withinBounds(point, bounds)) {
        return {
            SingleFrameStatus::OutOfObservationBounds,
            std::nullopt,
            fieldIndex};
    }
    return {SingleFrameStatus::Success, point, std::nullopt};
}

}  // namespace

VisionDataParser::VisionDataParser()
    : VisionDataParser(BilliardConfig::VISION_OBSERVATION_BOUNDS)
{
}

VisionDataParser::VisionDataParser(AxisAlignedBounds2D observationBounds)
    : VisionDataParser(std::optional<AxisAlignedBounds2D>{observationBounds})
{
}

VisionDataParser::VisionDataParser(
    std::optional<AxisAlignedBounds2D> observationBounds)
    : observationBounds_(std::move(observationBounds))
{
}

SingleFrameStatus VisionDataParser::configurationStatus() const noexcept
{
    if (!observationBounds_) {
        return SingleFrameStatus::ConfigurationMissing;
    }
    return validBounds(*observationBounds_)
        ? SingleFrameStatus::Success
        : SingleFrameStatus::InvalidConfiguration;
}

SingleFrameResult VisionDataParser::parse(const std::string& message) const
{
    const SingleFrameStatus parserConfiguration = configurationStatus();
    if (parserConfiguration != SingleFrameStatus::Success) {
        return SingleFrameResult::rejected(parserConfiguration);
    }

    const std::vector<std::string> tokens = splitPreservingEmptyTokens(message);
    if (tokens.size() != EXPECTED_FIELD_COUNT) {
        return SingleFrameResult::rejected(
            SingleFrameStatus::WrongFieldCount);
    }

    std::vector<double> values(EXPECTED_FIELD_COUNT);
    for (std::size_t index = 0; index < tokens.size(); ++index) {
        const NumericParseResult numberResult = parseNumber(tokens[index]);
        if (numberResult.status != SingleFrameStatus::Success || !numberResult.value) {
            return SingleFrameResult::rejected(numberResult.status, index);
        }
        values[index] = *numberResult.value;
    }

    ParsedVisionFrame parsed;
    for (std::size_t index = 0; index < parsed.objectBalls.size(); ++index) {
        const ParsedPointResult point = parsePoint(values, index * 2, *observationBounds_);
        if (point.status != SingleFrameStatus::Success) {
            return SingleFrameResult::rejected(point.status, point.fieldIndex);
        }
        parsed.objectBalls[index] = point.point;
    }

    const ParsedPointResult cueBall = parsePoint(values, 18, *observationBounds_);
    if (cueBall.status != SingleFrameStatus::Success) {
        return SingleFrameResult::rejected(cueBall.status, cueBall.fieldIndex);
    }
    // 母球跟編號球/袋口一樣，單幀缺席不再讓整幀作廢；是否收斂為必要值
    // 交給ThreeEventStability的三幀累積判斷。
    parsed.cueBall = cueBall.point;

    for (std::size_t index = 0; index < parsed.pockets.size(); ++index) {
        const std::size_t fieldIndex = 20 + index * 2;
        const ParsedPointResult pocket = parsePoint(values, fieldIndex, *observationBounds_);
        if (pocket.status != SingleFrameStatus::Success) {
            return SingleFrameResult::rejected(pocket.status, pocket.fieldIndex);
        }
        // 袋口跟編號球一樣，單幀缺席不再讓整幀作廢；是否收斂為必要值
        // 交給ThreeEventStability的三幀累積判斷。
        parsed.pockets[index] = pocket.point;
    }

    return SingleFrameResult::success(ValidatedVisionFrame(
        std::move(parsed.objectBalls),
        parsed.cueBall,
        parsed.pockets));
}

ReceiveEventFactory::ReceiveEventFactory(const VisionDataParser& parser)
    : parser_(parser),
      connectionIdentity_(0),
      shotCycleIdentity_(0),
      nextEventId_(1),
      hasLastReceiveTime_(false),
      activeCycle_(false),
      captureWindowOpen_(false),
      lastInvalidationReason_(ReceiveEventInvalidationReason::ExplicitFlush)
{
}

void ReceiveEventFactory::beginCycle(
    ConnectionIdentity connectionIdentity,
    ShotCycleIdentity shotCycleIdentity)
{
    connectionIdentity_ = connectionIdentity;
    shotCycleIdentity_ = shotCycleIdentity;
    nextEventId_ = 1;
    hasLastReceiveTime_ = false;
    activeCycle_ = connectionIdentity != 0 && shotCycleIdentity != 0;
    captureWindowOpen_ = false;
    lastInvalidationReason_ = ReceiveEventInvalidationReason::CycleChanged;
}

bool ReceiveEventFactory::openCaptureWindow(
    ConnectionIdentity connectionIdentity,
    ShotCycleIdentity shotCycleIdentity)
{
    if (!activeCycle_ || connectionIdentity != connectionIdentity_ ||
        shotCycleIdentity != shotCycleIdentity_) {
        return false;
    }
    captureWindowOpen_ = true;
    lastInvalidationReason_ = ReceiveEventInvalidationReason::CaptureWindowRestart;
    return true;
}

void ReceiveEventFactory::invalidate(ReceiveEventInvalidationReason reason)
{
    activeCycle_ = false;
    captureWindowOpen_ = false;
    hasLastReceiveTime_ = false;
    connectionIdentity_ = 0;
    shotCycleIdentity_ = 0;
    nextEventId_ = 1;
    lastInvalidationReason_ = reason;
}

ReceiveEventResult ReceiveEventFactory::accept(
    const std::string& payload,
    ConnectionIdentity connectionIdentity,
    std::chrono::steady_clock::time_point receivedAt)
{
    if (!activeCycle_) {
        return ReceiveEventResult::rejected(
            ReceiveEventStatus::NoActiveCycle,
            true);
    }
    if (connectionIdentity != connectionIdentity_) {
        invalidate(ReceiveEventInvalidationReason::Reconnect);
        return ReceiveEventResult::rejected(
            ReceiveEventStatus::ConnectionMismatch,
            true);
    }
    if (!captureWindowOpen_) {
        return ReceiveEventResult::rejected(
            ReceiveEventStatus::CaptureWindowClosed,
            false);
    }
    if (hasLastReceiveTime_ && receivedAt < lastReceiveTime_) {
        invalidate(ReceiveEventInvalidationReason::Timeout);
        return ReceiveEventResult::rejected(
            ReceiveEventStatus::NonMonotonicReceiveTime,
            true);
    }
    if (nextEventId_ == 0 || nextEventId_ == (std::numeric_limits<ReceiveEventId>::max)()) {
        invalidate(ReceiveEventInvalidationReason::CycleChanged);
        return ReceiveEventResult::rejected(
            ReceiveEventStatus::EventIdExhausted,
            true);
    }

    SingleFrameResult frame = parser_.parse(payload);
    if (frame.status() != SingleFrameStatus::Success || !frame.value()) {
        const std::optional<SingleFrameDiagnostic> diagnostic = frame.diagnostic();
        invalidate(ReceiveEventInvalidationReason::ParserFailure);
        return ReceiveEventResult::rejected(
            ReceiveEventStatus::FrameRejected,
            true,
            diagnostic);
    }

    ReceiveEvent event{
        connectionIdentity_,
        shotCycleIdentity_,
        nextEventId_++,
        receivedAt,
        *frame.value()};
    lastReceiveTime_ = receivedAt;
    hasLastReceiveTime_ = true;
    return ReceiveEventResult::success(std::move(event));
}

bool ReceiveEventFactory::hasActiveCycle() const noexcept
{
    return activeCycle_;
}

bool ReceiveEventFactory::isCaptureWindowOpen() const noexcept
{
    return captureWindowOpen_;
}

ShotCycleIdentity ReceiveEventFactory::currentCycleIdentity() const noexcept
{
    return shotCycleIdentity_;
}

ReceiveEventInvalidationReason ReceiveEventFactory::lastInvalidationReason() const noexcept
{
    return lastInvalidationReason_;
}
