// 定義視覺偵測結果、球桌狀態與目標選擇結果等領域資料。
#pragma once

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "BilliardPhysics.h"
#include "Point.h"

class BilliardAlgorithm;

// 單一影像時間點的完整球桌偵測狀態。
struct DetectedPoint {
    bool detected = false;
    Point position = {0.0, 0.0};
};

struct TableState {
    std::array<DetectedPoint, 9> objectBalls;
    DetectedPoint cueBall;
    std::array<DetectedPoint, 6> pockets;
};

// P1-03嚴格輸入邊界使用的單幀型別。缺失編號球只能以nullopt表示；
// 母球與六袋在ValidatedVisionFrame中必定存在。
struct ParsedVisionFrame {
    std::array<std::optional<Point>, 9> objectBalls;
    std::optional<Point> cueBall;
    std::array<std::optional<Point>, 6> pockets;
};

class ValidatedVisionFrame {
public:
    ValidatedVisionFrame(
        std::array<std::optional<Point>, 9> numberedBalls,
        Point requiredCueBall,
        std::array<Point, 6> requiredPockets)
        : objectBalls(std::move(numberedBalls)),
          cueBall(requiredCueBall),
          pockets(std::move(requiredPockets))
    {
    }

    std::array<std::optional<Point>, 9> objectBalls;
    Point cueBall;
    std::array<Point, 6> pockets;
};

enum class SingleFrameStatus {
    Success,
    WrongFieldCount,
    EmptyToken,
    InvalidNumericToken,
    NumericOverflow,
    NonFiniteValue,
    InvalidSentinelPair,
    MissingRequiredCueBall,
    MissingRequiredPocket,
    OutOfObservationBounds,
    ConfigurationMissing,
    InvalidConfiguration
};

struct SingleFrameDiagnostic {
    SingleFrameStatus status;
    std::optional<std::size_t> fieldIndex;
};

class SingleFrameResult {
public:
    static SingleFrameResult success(ValidatedVisionFrame frame)
    {
        return SingleFrameResult(
            SingleFrameStatus::Success,
            std::optional<ValidatedVisionFrame>{std::move(frame)},
            std::nullopt);
    }

    static SingleFrameResult rejected(
        SingleFrameStatus status,
        std::optional<std::size_t> fieldIndex = std::nullopt)
    {
        return SingleFrameResult(
            status,
            std::nullopt,
            SingleFrameDiagnostic{status, fieldIndex});
    }

    [[nodiscard]] SingleFrameStatus status() const noexcept
    {
        return status_;
    }

    [[nodiscard]] const std::optional<ValidatedVisionFrame>& value() const noexcept
    {
        return value_;
    }

    [[nodiscard]] const std::optional<SingleFrameDiagnostic>& diagnostic() const noexcept
    {
        return diagnostic_;
    }

    [[nodiscard]] bool isValid() const noexcept
    {
        const bool succeeded = status_ == SingleFrameStatus::Success;
        return value_.has_value() == succeeded &&
            diagnostic_.has_value() != succeeded &&
            (!diagnostic_ || diagnostic_->status == status_);
    }

private:
    SingleFrameResult(
        SingleFrameStatus status,
        std::optional<ValidatedVisionFrame> value,
        std::optional<SingleFrameDiagnostic> diagnostic)
        : status_(status),
          value_(std::move(value)),
          diagnostic_(std::move(diagnostic))
    {
    }

    SingleFrameStatus status_;
    std::optional<ValidatedVisionFrame> value_;
    std::optional<SingleFrameDiagnostic> diagnostic_;
};

using ConnectionIdentity = std::uint64_t;
using ShotCycleIdentity = std::uint64_t;
using ReceiveEventId = std::uint64_t;

struct ReceiveEvent {
    ConnectionIdentity connectionIdentity;
    ShotCycleIdentity shotCycleIdentity;
    ReceiveEventId eventId;
    std::chrono::steady_clock::time_point receivedAt;
    ValidatedVisionFrame frame;
};

enum class ReceiveEventStatus {
    Success,
    NoActiveCycle,
    CaptureWindowClosed,
    ConnectionMismatch,
    NonMonotonicReceiveTime,
    EventIdExhausted,
    FrameRejected
};

enum class ReceiveEventInvalidationReason {
    ExplicitFlush,
    CaptureWindowRestart,
    CycleChanged,
    Disconnect,
    Reconnect,
    Timeout,
    ParserFailure
};

struct ReceiveEventDiagnostic {
    ReceiveEventStatus status;
    std::optional<SingleFrameDiagnostic> frameDiagnostic;
    bool resetRequired;
};

class ReceiveEventResult {
public:
    static ReceiveEventResult success(ReceiveEvent event)
    {
        return ReceiveEventResult(
            ReceiveEventStatus::Success,
            std::optional<ReceiveEvent>{std::move(event)},
            std::nullopt);
    }

    static ReceiveEventResult rejected(
        ReceiveEventStatus status,
        bool resetRequired,
        std::optional<SingleFrameDiagnostic> frameDiagnostic = std::nullopt)
    {
        return ReceiveEventResult(
            status,
            std::nullopt,
            ReceiveEventDiagnostic{status, std::move(frameDiagnostic), resetRequired});
    }

    [[nodiscard]] ReceiveEventStatus status() const noexcept
    {
        return status_;
    }

    [[nodiscard]] const std::optional<ReceiveEvent>& value() const noexcept
    {
        return value_;
    }

    [[nodiscard]] const std::optional<ReceiveEventDiagnostic>& diagnostic() const noexcept
    {
        return diagnostic_;
    }

    [[nodiscard]] bool resetRequired() const noexcept
    {
        return diagnostic_ && diagnostic_->resetRequired;
    }

    [[nodiscard]] bool isValid() const noexcept
    {
        const bool succeeded = status_ == ReceiveEventStatus::Success;
        return value_.has_value() == succeeded &&
            diagnostic_.has_value() != succeeded &&
            (!diagnostic_ || diagnostic_->status == status_);
    }

private:
    ReceiveEventResult(
        ReceiveEventStatus status,
        std::optional<ReceiveEvent> value,
        std::optional<ReceiveEventDiagnostic> diagnostic)
        : status_(status),
          value_(std::move(value)),
          diagnostic_(std::move(diagnostic))
    {
    }

    ReceiveEventStatus status_;
    std::optional<ReceiveEvent> value_;
    std::optional<ReceiveEventDiagnostic> diagnostic_;
};

// P1-04唯一的三事件穩定設定。production值由BilliardConfig提供；
// 離線測試可明確注入fixture，不使用隱含預設值。
struct StabilityConfig {
    std::optional<double> stableFrameToleranceMm;
    std::optional<double> pocketStabilityToleranceMm;
    std::optional<std::chrono::milliseconds> maxInterFrameInterval;
};

struct StableSourceEventMetadata {
    ReceiveEventId eventId;
    std::chrono::steady_clock::time_point receivedAt;
};

class StableTableState {
public:
    StableTableState(
        std::array<std::optional<Point>, 9> numberedBalls,
        Point stableCueBall,
        std::array<Point, 6> stablePockets,
        ConnectionIdentity sourceConnectionIdentity,
        ShotCycleIdentity sourceShotCycleIdentity,
        std::array<StableSourceEventMetadata, 3> sourceEventMetadata)
        : objectBalls(std::move(numberedBalls)),
          cueBall(stableCueBall),
          pockets(std::move(stablePockets)),
          connectionIdentity(sourceConnectionIdentity),
          shotCycleIdentity(sourceShotCycleIdentity),
          sourceEvents(std::move(sourceEventMetadata))
    {
    }

    std::array<std::optional<Point>, 9> objectBalls;
    Point cueBall;
    std::array<Point, 6> pockets;
    ConnectionIdentity connectionIdentity;
    ShotCycleIdentity shotCycleIdentity;
    std::array<StableSourceEventMetadata, 3> sourceEvents;
};

enum class StabilityStatus {
    NeedMoreEvents,
    Stable,
    Unstable,
    TimedOut,
    InvalidConfiguration
};

enum class StabilityFailureReason {
    AwaitingEvents,
    ConfigurationMissing,
    InvalidConfiguration,
    InvalidEvent,
    ConnectionChanged,
    CycleChanged,
    EventIdNotIncreasing,
    ReceiveTimeWentBackward,
    TimedOut,
    PresenceChanged,
    BallMoved,
    PocketMoved,
    Disconnected,
    Reconnected,
    ParserFailure,
    ExplicitReset
};

struct StabilityDiagnostic {
    StabilityStatus status;
    StabilityFailureReason reason;
    std::size_t acceptedEventCount;
    bool resetRequired;
};

class StabilityResult {
public:
    static StabilityResult needMoreEvents(std::size_t acceptedEventCount)
    {
        return StabilityResult(
            StabilityStatus::NeedMoreEvents,
            std::nullopt,
            StabilityDiagnostic{
                StabilityStatus::NeedMoreEvents,
                StabilityFailureReason::AwaitingEvents,
                acceptedEventCount,
                false});
    }

    static StabilityResult stable(StableTableState state)
    {
        return StabilityResult(
            StabilityStatus::Stable,
            std::optional<StableTableState>{std::move(state)},
            std::nullopt);
    }

    static StabilityResult failure(
        StabilityStatus status,
        StabilityFailureReason reason,
        std::size_t acceptedEventCount)
    {
        return StabilityResult(
            status,
            std::nullopt,
            StabilityDiagnostic{status, reason, acceptedEventCount, true});
    }

    [[nodiscard]] StabilityStatus status() const noexcept
    {
        return status_;
    }

    [[nodiscard]] const std::optional<StableTableState>& value() const noexcept
    {
        return value_;
    }

    [[nodiscard]] const std::optional<StabilityDiagnostic>& diagnostic() const noexcept
    {
        return diagnostic_;
    }

    [[nodiscard]] bool isValid() const noexcept
    {
        const bool stableStatus = status_ == StabilityStatus::Stable;
        return value_.has_value() == stableStatus &&
            diagnostic_.has_value() != stableStatus &&
            (!diagnostic_ || diagnostic_->status == status_);
    }

private:
    StabilityResult(
        StabilityStatus status,
        std::optional<StableTableState> value,
        std::optional<StabilityDiagnostic> diagnostic)
        : status_(status),
          value_(std::move(value)),
          diagnostic_(std::move(diagnostic))
    {
    }

    StabilityStatus status_;
    std::optional<StableTableState> value_;
    std::optional<StabilityDiagnostic> diagnostic_;
};

enum class Phase1PipelineStatus {
    Waiting,
    InputFailure,
    StabilityFailure,
    PlanningCompleted
};

struct Phase1PipelineDiagnostic {
    Phase1PipelineStatus status;
    std::optional<SingleFrameDiagnostic> inputDiagnostic;
    std::optional<StabilityDiagnostic> stabilityDiagnostic;
};

// P1-04只建立非規劃結果。Stable不在此被錯標為PlanningCompleted，
// 而是繼續以StableTableState交給後續ShotBrain seam。
class Phase1PipelineResult {
public:
    static Phase1PipelineResult inputFailure(SingleFrameDiagnostic diagnostic)
    {
        return Phase1PipelineResult{
            Phase1PipelineStatus::InputFailure,
            Phase1PipelineDiagnostic{
                Phase1PipelineStatus::InputFailure,
                std::optional<SingleFrameDiagnostic>{std::move(diagnostic)},
                std::nullopt}};
    }

    [[nodiscard]] static std::optional<Phase1PipelineResult> fromStability(
        const StabilityResult& result)
    {
        if (result.status() == StabilityStatus::Stable) {
            return std::nullopt;
        }

        const Phase1PipelineStatus pipelineStatus =
            result.status() == StabilityStatus::NeedMoreEvents
            ? Phase1PipelineStatus::Waiting
            : Phase1PipelineStatus::StabilityFailure;
        std::optional<StabilityDiagnostic> diagnostic = result.diagnostic();
        if (!result.isValid() || !diagnostic) {
            diagnostic = StabilityDiagnostic{
                StabilityStatus::Unstable,
                StabilityFailureReason::InvalidEvent,
                0,
                true};
        }
        return Phase1PipelineResult{
            pipelineStatus,
            Phase1PipelineDiagnostic{
                pipelineStatus,
                std::nullopt,
                std::move(diagnostic)}};
    }

    [[nodiscard]] Phase1PipelineStatus status() const noexcept
    {
        return status_;
    }

    [[nodiscard]] const Phase1PipelineDiagnostic& diagnostic() const noexcept
    {
        return diagnostic_;
    }

    [[nodiscard]] bool isValid() const noexcept
    {
        if (diagnostic_.status != status_) {
            return false;
        }
        if (status_ == Phase1PipelineStatus::InputFailure) {
            return diagnostic_.inputDiagnostic.has_value() &&
                !diagnostic_.stabilityDiagnostic.has_value();
        }
        if (status_ == Phase1PipelineStatus::Waiting) {
            return !diagnostic_.inputDiagnostic.has_value() &&
                diagnostic_.stabilityDiagnostic.has_value() &&
                diagnostic_.stabilityDiagnostic->status == StabilityStatus::NeedMoreEvents;
        }
        if (status_ == Phase1PipelineStatus::StabilityFailure) {
            return !diagnostic_.inputDiagnostic.has_value() &&
                diagnostic_.stabilityDiagnostic.has_value() &&
                diagnostic_.stabilityDiagnostic->status != StabilityStatus::NeedMoreEvents &&
                diagnostic_.stabilityDiagnostic->status != StabilityStatus::Stable;
        }
        return false;
    }

private:
    Phase1PipelineResult(
        Phase1PipelineStatus status,
        Phase1PipelineDiagnostic diagnostic)
        : status_(status),
          diagnostic_(std::move(diagnostic))
    {
    }

    Phase1PipelineStatus status_;
    Phase1PipelineDiagnostic diagnostic_;
};

// TableState既有owner中的唯一三事件狀態物件；不解析CSV、不選target，
// 也不建立任何幾何、規劃或硬體結果。
class ThreeEventStability {
public:
    explicit ThreeEventStability(StabilityConfig config)
        : config_(std::move(config)),
          lastResetReason_(StabilityFailureReason::ExplicitReset)
    {
        events_.reserve(3);
    }

    [[nodiscard]] StabilityResult accept(ReceiveEvent event)
    {
        const auto configurationFailure = validateConfiguration();
        if (configurationFailure) {
            reset(*configurationFailure);
            return StabilityResult::failure(
                StabilityStatus::InvalidConfiguration,
                *configurationFailure,
                0);
        }
        if (!validEvent(event)) {
            reset(StabilityFailureReason::InvalidEvent);
            return StabilityResult::failure(
                StabilityStatus::Unstable,
                StabilityFailureReason::InvalidEvent,
                0);
        }

        if (!events_.empty()) {
            const ReceiveEvent& previous = events_.back();
            if (event.connectionIdentity != previous.connectionIdentity) {
                return rejectAndReset(
                    StabilityStatus::Unstable,
                    StabilityFailureReason::ConnectionChanged);
            }
            if (event.shotCycleIdentity != previous.shotCycleIdentity) {
                return rejectAndReset(
                    StabilityStatus::Unstable,
                    StabilityFailureReason::CycleChanged);
            }
            if (event.eventId <= previous.eventId) {
                return rejectAndReset(
                    StabilityStatus::Unstable,
                    StabilityFailureReason::EventIdNotIncreasing);
            }
            if (event.receivedAt < previous.receivedAt) {
                return rejectAndReset(
                    StabilityStatus::Unstable,
                    StabilityFailureReason::ReceiveTimeWentBackward);
            }
            if (event.receivedAt - previous.receivedAt > *config_.maxInterFrameInterval) {
                return rejectAndReset(
                    StabilityStatus::TimedOut,
                    StabilityFailureReason::TimedOut);
            }
            if (!samePresence(events_.front().frame, event.frame)) {
                return rejectAndReset(
                    StabilityStatus::Unstable,
                    StabilityFailureReason::PresenceChanged);
            }
        }

        events_.push_back(std::move(event));
        if (events_.size() < 3) {
            return StabilityResult::needMoreEvents(events_.size());
        }

        const auto stableState = buildStableState();
        if (!stableState.state) {
            return rejectAndReset(StabilityStatus::Unstable, stableState.reason);
        }

        StableTableState result = std::move(*stableState.state);
        reset(StabilityFailureReason::ExplicitReset);
        return StabilityResult::stable(std::move(result));
    }

    void reset(StabilityFailureReason reason) noexcept
    {
        events_.clear();
        lastResetReason_ = reason;
    }

    [[nodiscard]] std::size_t accumulatedEventCount() const noexcept
    {
        return events_.size();
    }

    [[nodiscard]] StabilityFailureReason lastResetReason() const noexcept
    {
        return lastResetReason_;
    }

    [[nodiscard]] std::optional<StabilityFailureReason> configurationFailure() const noexcept
    {
        return validateConfiguration();
    }

private:
    struct StableBuildResult {
        std::optional<StableTableState> state;
        StabilityFailureReason reason;
    };

    [[nodiscard]] std::optional<StabilityFailureReason> validateConfiguration() const noexcept
    {
        if (!config_.stableFrameToleranceMm ||
            !config_.pocketStabilityToleranceMm ||
            !config_.maxInterFrameInterval) {
            return StabilityFailureReason::ConfigurationMissing;
        }
        if (!std::isfinite(*config_.stableFrameToleranceMm) ||
            !std::isfinite(*config_.pocketStabilityToleranceMm) ||
            *config_.stableFrameToleranceMm < 0.0 ||
            *config_.pocketStabilityToleranceMm < 0.0 ||
            config_.maxInterFrameInterval->count() <= 0) {
            return StabilityFailureReason::InvalidConfiguration;
        }
        return std::nullopt;
    }

    [[nodiscard]] static bool finitePoint(Point point) noexcept
    {
        return std::isfinite(point.x) && std::isfinite(point.y);
    }

    [[nodiscard]] static bool validEvent(const ReceiveEvent& event) noexcept
    {
        if (event.connectionIdentity == 0 || event.shotCycleIdentity == 0 ||
            event.eventId == 0 || !finitePoint(event.frame.cueBall)) {
            return false;
        }
        for (const auto& ball : event.frame.objectBalls) {
            if (ball && !finitePoint(*ball)) {
                return false;
            }
        }
        for (const Point pocket : event.frame.pockets) {
            if (!finitePoint(pocket)) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] static bool samePresence(
        const ValidatedVisionFrame& first,
        const ValidatedVisionFrame& current) noexcept
    {
        for (std::size_t index = 0; index < first.objectBalls.size(); ++index) {
            if (first.objectBalls[index].has_value() !=
                current.objectBalls[index].has_value()) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] static double median(double first, double second, double third) noexcept
    {
        std::array<double, 3> values{first, second, third};
        std::sort(values.begin(), values.end());
        return values[1];
    }

    [[nodiscard]] static Point medianPoint(Point first, Point second, Point third) noexcept
    {
        return {
            median(first.x, second.x, third.x),
            median(first.y, second.y, third.y)};
    }

    [[nodiscard]] static bool withinTolerance(
        Point point,
        Point center,
        double tolerance) noexcept
    {
        const double distance = std::hypot(point.x - center.x, point.y - center.y);
        return std::isfinite(distance) && distance <= tolerance;
    }

    [[nodiscard]] StableBuildResult buildStableState() const
    {
        std::array<std::optional<Point>, 9> balls{};
        for (std::size_t index = 0; index < balls.size(); ++index) {
            if (!events_[0].frame.objectBalls[index]) {
                balls[index] = std::nullopt;
                continue;
            }
            const Point center = medianPoint(
                *events_[0].frame.objectBalls[index],
                *events_[1].frame.objectBalls[index],
                *events_[2].frame.objectBalls[index]);
            for (const ReceiveEvent& event : events_) {
                if (!event.frame.objectBalls[index] ||
                    !withinTolerance(
                        *event.frame.objectBalls[index],
                        center,
                        *config_.stableFrameToleranceMm)) {
                    return {std::nullopt, StabilityFailureReason::BallMoved};
                }
            }
            balls[index] = center;
        }

        const Point cueBall = medianPoint(
            events_[0].frame.cueBall,
            events_[1].frame.cueBall,
            events_[2].frame.cueBall);
        for (const ReceiveEvent& event : events_) {
            if (!withinTolerance(
                    event.frame.cueBall,
                    cueBall,
                    *config_.stableFrameToleranceMm)) {
                return {std::nullopt, StabilityFailureReason::BallMoved};
            }
        }

        std::array<Point, 6> pockets{};
        for (std::size_t index = 0; index < pockets.size(); ++index) {
            pockets[index] = medianPoint(
                events_[0].frame.pockets[index],
                events_[1].frame.pockets[index],
                events_[2].frame.pockets[index]);
            for (const ReceiveEvent& event : events_) {
                if (!withinTolerance(
                        event.frame.pockets[index],
                        pockets[index],
                        *config_.pocketStabilityToleranceMm)) {
                    return {std::nullopt, StabilityFailureReason::PocketMoved};
                }
            }
        }

        std::array<StableSourceEventMetadata, 3> sourceEvents{};
        for (std::size_t index = 0; index < sourceEvents.size(); ++index) {
            sourceEvents[index] = {events_[index].eventId, events_[index].receivedAt};
        }
        return {
            StableTableState{
                std::move(balls),
                cueBall,
                std::move(pockets),
                events_[0].connectionIdentity,
                events_[0].shotCycleIdentity,
                std::move(sourceEvents)},
            StabilityFailureReason::AwaitingEvents};
    }

    [[nodiscard]] StabilityResult rejectAndReset(
        StabilityStatus status,
        StabilityFailureReason reason)
    {
        const std::size_t acceptedEventCount = events_.size();
        reset(reason);
        return StabilityResult::failure(status, reason, acceptedEventCount);
    }

    StabilityConfig config_;
    std::vector<ReceiveEvent> events_;
    StabilityFailureReason lastResetReason_;
};

struct EligibleTarget {
    int ballNumber;
    Point center;
};

enum class TargetQualificationStatus {
    Success,
    NoEligibleTarget,
    InvalidStableState
};

struct TargetQualificationDiagnostic {
    TargetQualificationStatus status;
};

struct TargetQualificationAudit {
    bool expectedBallSetNotApplied;
};

class TargetQualificationResult {
public:
    static TargetQualificationResult success(EligibleTarget target)
    {
        return TargetQualificationResult(
            TargetQualificationStatus::Success,
            std::optional<EligibleTarget>{target},
            std::nullopt);
    }

    static TargetQualificationResult rejected(TargetQualificationStatus status)
    {
        return TargetQualificationResult(
            status,
            std::nullopt,
            TargetQualificationDiagnostic{status});
    }

    [[nodiscard]] TargetQualificationStatus status() const noexcept { return status_; }
    [[nodiscard]] const std::optional<EligibleTarget>& value() const noexcept { return value_; }
    [[nodiscard]] const std::optional<TargetQualificationDiagnostic>& diagnostic() const noexcept
    {
        return diagnostic_;
    }
    [[nodiscard]] const TargetQualificationAudit& audit() const noexcept { return audit_; }
    [[nodiscard]] bool isValid() const noexcept
    {
        const bool succeeded = status_ == TargetQualificationStatus::Success;
        return value_.has_value() == succeeded &&
            diagnostic_.has_value() != succeeded &&
            (!diagnostic_ || diagnostic_->status == status_);
    }

private:
    TargetQualificationResult(
        TargetQualificationStatus status,
        std::optional<EligibleTarget> value,
        std::optional<TargetQualificationDiagnostic> diagnostic)
        : status_(status), value_(std::move(value)), diagnostic_(std::move(diagnostic))
    {
    }

    TargetQualificationStatus status_;
    std::optional<EligibleTarget> value_;
    std::optional<TargetQualificationDiagnostic> diagnostic_;
    TargetQualificationAudit audit_{true};
};

enum class DirectPotRejectionReason {
    GhostGeometryInvalid,
    CuePathInvalid,
    CuePathBlocked,
    TargetPathInvalid,
    TargetPathBlocked,
    PocketEntryRejected,
    CutAngleInvalid
};

struct DirectPotCandidateDiagnostic {
    BilliardConfig::PocketId pocketId;
    DirectPotRejectionReason reason;
    GeometryStatus geometryStatus;
    std::optional<std::size_t> relatedObstacleIndex;
};

struct DirectPotCandidate {
    EligibleTarget target;
    BilliardConfig::PocketId pocketId;
    Point virtualPocketTarget;
    GhostBallPoint ghostBallPoint;
    Segment2D cuePath;
    Segment2D targetPath;
    double cuttingAngleDeg;
    double pocketEntryAngleDeg;
};

struct DirectPotEvaluation {
    EligibleTarget target;
    std::array<std::optional<DirectPotCandidate>, 6> feasible;
    std::array<std::optional<DirectPotCandidateDiagnostic>, 6> rejected;

    [[nodiscard]] bool isValid() const noexcept
    {
        const auto finitePoint = [](Point point) noexcept {
            return std::isfinite(point.x) && std::isfinite(point.y);
        };
        const auto samePoint = [](Point first, Point second) noexcept {
            return first.x == second.x && first.y == second.y;
        };
        if (target.ballNumber < 1 || target.ballNumber > 9 ||
            !finitePoint(target.center)) {
            return false;
        }
        for (std::size_t index = 0; index < feasible.size(); ++index) {
            if (feasible[index].has_value() == rejected[index].has_value()) {
                return false;
            }
            if (feasible[index]) {
                const DirectPotCandidate& candidate = *feasible[index];
                if (static_cast<std::size_t>(candidate.pocketId) != index ||
                    candidate.target.ballNumber != target.ballNumber ||
                    !samePoint(candidate.target.center, target.center) ||
                    !finitePoint(candidate.virtualPocketTarget) ||
                    !finitePoint(candidate.ghostBallPoint.center) ||
                    !finitePoint(candidate.cuePath.start) ||
                    !finitePoint(candidate.cuePath.end) ||
                    !finitePoint(candidate.targetPath.start) ||
                    !finitePoint(candidate.targetPath.end) ||
                    !samePoint(candidate.cuePath.end, candidate.ghostBallPoint.center) ||
                    !samePoint(candidate.targetPath.start, target.center) ||
                    !samePoint(candidate.targetPath.end, candidate.virtualPocketTarget) ||
                    !std::isfinite(candidate.cuttingAngleDeg) ||
                    candidate.cuttingAngleDeg < 0.0 || candidate.cuttingAngleDeg >= 90.0 ||
                    !std::isfinite(candidate.pocketEntryAngleDeg) ||
                    candidate.pocketEntryAngleDeg < 0.0) {
                    return false;
                }
            }
            if (rejected[index] &&
                static_cast<std::size_t>(rejected[index]->pocketId) != index) {
                return false;
            }
        }
        return true;
    }
};

enum class DirectPotGenerationStatus {
    Success,
    InvalidStableState,
    InvalidGeometryConfiguration,
    SelectedTargetMismatch
};

struct DirectPotGenerationDiagnostic {
    DirectPotGenerationStatus status;
};

class DirectPotGenerationResult {
public:
    static DirectPotGenerationResult success(DirectPotEvaluation evaluation)
    {
        return DirectPotGenerationResult(
            DirectPotGenerationStatus::Success,
            std::optional<DirectPotEvaluation>{std::move(evaluation)},
            std::nullopt);
    }

    static DirectPotGenerationResult rejected(DirectPotGenerationStatus status)
    {
        return DirectPotGenerationResult(
            status,
            std::nullopt,
            DirectPotGenerationDiagnostic{status});
    }

    [[nodiscard]] DirectPotGenerationStatus status() const noexcept { return status_; }
    [[nodiscard]] const std::optional<DirectPotEvaluation>& value() const noexcept
    {
        return value_;
    }
    [[nodiscard]] const std::optional<DirectPotGenerationDiagnostic>& diagnostic() const noexcept
    {
        return diagnostic_;
    }
    [[nodiscard]] bool isValid() const noexcept
    {
        const bool succeeded = status_ == DirectPotGenerationStatus::Success;
        return value_.has_value() == succeeded &&
            diagnostic_.has_value() != succeeded &&
            (!value_ || value_->isValid()) &&
            (!diagnostic_ || diagnostic_->status == status_);
    }

private:
    DirectPotGenerationResult(
        DirectPotGenerationStatus status,
        std::optional<DirectPotEvaluation> value,
        std::optional<DirectPotGenerationDiagnostic> diagnostic)
        : status_(status), value_(std::move(value)), diagnostic_(std::move(diagnostic))
    {
    }

    DirectPotGenerationStatus status_;
    std::optional<DirectPotEvaluation> value_;
    std::optional<DirectPotGenerationDiagnostic> diagnostic_;
};

enum class KickPotRejectionReason {
    GhostGeometryInvalid,
    RailGeometryInvalid,
    NoRailIntersection,
    ReflectionInvariantFailed,
    KickAngleRejected,
    CueFirstSegmentInvalid,
    CueFirstSegmentBlocked,
    CueSecondSegmentInvalid,
    CueSecondSegmentBlocked,
    TargetPathInvalid,
    TargetPathBlocked,
    PocketEntryRejected,
    CutAngleInvalid
};

struct KickPotCandidateDiagnostic {
    BilliardConfig::PocketId pocketId;
    BilliardConfig::RailId railId;
    KickPotRejectionReason reason;
    GeometryStatus geometryStatus;
    std::optional<std::size_t> relatedObstacleIndex;
};

struct KickPotCandidate {
    EligibleTarget target;
    BilliardConfig::PocketId pocketId;
    BilliardConfig::RailId railId;
    Point virtualPocketTarget;
    GhostBallPoint ghostBallPoint;
    Point reboundPoint;
    Segment2D cuePathFirst;
    Segment2D cuePathSecond;
    Segment2D targetPath;
    double cuttingAngleDeg;
    double pocketEntryAngleDeg;
    double incidenceAngleDeg;
    double reflectionAngleDeg;
};

using KickPotCandidateGrid =
    std::array<std::array<std::optional<KickPotCandidate>, 6>, 6>;
using KickPotDiagnosticGrid =
    std::array<std::array<std::optional<KickPotCandidateDiagnostic>, 6>, 6>;

struct KickPotEvaluation {
    EligibleTarget target;
    KickPotCandidateGrid feasible;
    KickPotDiagnosticGrid rejected;

    [[nodiscard]] bool isValid() const noexcept
    {
        const auto finitePoint = [](Point point) noexcept {
            return std::isfinite(point.x) && std::isfinite(point.y);
        };
        const auto samePoint = [](Point first, Point second) noexcept {
            return first.x == second.x && first.y == second.y;
        };
        if (target.ballNumber < 1 || target.ballNumber > 9 ||
            !finitePoint(target.center)) {
            return false;
        }
        for (std::size_t pocket = 0; pocket < feasible.size(); ++pocket) {
            for (std::size_t rail = 0; rail < feasible[pocket].size(); ++rail) {
                if (feasible[pocket][rail].has_value() ==
                    rejected[pocket][rail].has_value()) {
                    return false;
                }
                if (feasible[pocket][rail]) {
                    const KickPotCandidate& candidate = *feasible[pocket][rail];
                    if (static_cast<std::size_t>(candidate.pocketId) != pocket ||
                        static_cast<std::size_t>(candidate.railId) != rail ||
                        candidate.target.ballNumber != target.ballNumber ||
                        !samePoint(candidate.target.center, target.center) ||
                        !finitePoint(candidate.virtualPocketTarget) ||
                        !finitePoint(candidate.ghostBallPoint.center) ||
                        !finitePoint(candidate.reboundPoint) ||
                        !finitePoint(candidate.cuePathFirst.start) ||
                        !finitePoint(candidate.cuePathFirst.end) ||
                        !finitePoint(candidate.cuePathSecond.start) ||
                        !finitePoint(candidate.cuePathSecond.end) ||
                        !finitePoint(candidate.targetPath.start) ||
                        !finitePoint(candidate.targetPath.end) ||
                        !samePoint(candidate.cuePathFirst.end, candidate.reboundPoint) ||
                        !samePoint(candidate.cuePathSecond.start, candidate.reboundPoint) ||
                        !samePoint(candidate.cuePathSecond.end,
                            candidate.ghostBallPoint.center) ||
                        !samePoint(candidate.targetPath.start, target.center) ||
                        !samePoint(candidate.targetPath.end,
                            candidate.virtualPocketTarget) ||
                        !std::isfinite(candidate.cuttingAngleDeg) ||
                        candidate.cuttingAngleDeg < 0.0 ||
                        candidate.cuttingAngleDeg >= 90.0 ||
                        !std::isfinite(candidate.pocketEntryAngleDeg) ||
                        candidate.pocketEntryAngleDeg < 0.0 ||
                        !std::isfinite(candidate.incidenceAngleDeg) ||
                        candidate.incidenceAngleDeg < 0.0 ||
                        candidate.incidenceAngleDeg > 90.0 ||
                        !std::isfinite(candidate.reflectionAngleDeg) ||
                        candidate.reflectionAngleDeg < 0.0 ||
                        candidate.reflectionAngleDeg > 90.0) {
                        return false;
                    }
                }
                if (rejected[pocket][rail] &&
                    (static_cast<std::size_t>(rejected[pocket][rail]->pocketId) != pocket ||
                     static_cast<std::size_t>(rejected[pocket][rail]->railId) != rail)) {
                    return false;
                }
            }
        }
        return true;
    }
};

enum class KickPotGenerationStatus {
    Success,
    ConfigurationMissing,
    InvalidStableState,
    InvalidGeometryConfiguration,
    SelectedTargetMismatch
};

struct KickPotGenerationDiagnostic {
    KickPotGenerationStatus status;
};

class KickPotGenerationResult {
public:
    static KickPotGenerationResult success(KickPotEvaluation evaluation)
    {
        return KickPotGenerationResult(
            KickPotGenerationStatus::Success,
            std::optional<KickPotEvaluation>{std::move(evaluation)},
            std::nullopt);
    }

    static KickPotGenerationResult rejected(KickPotGenerationStatus status)
    {
        return KickPotGenerationResult(
            status,
            std::nullopt,
            KickPotGenerationDiagnostic{status});
    }

    [[nodiscard]] KickPotGenerationStatus status() const noexcept { return status_; }
    [[nodiscard]] const std::optional<KickPotEvaluation>& value() const noexcept
    {
        return value_;
    }
    [[nodiscard]] const std::optional<KickPotGenerationDiagnostic>& diagnostic() const noexcept
    {
        return diagnostic_;
    }
    [[nodiscard]] bool isValid() const noexcept
    {
        const bool succeeded = status_ == KickPotGenerationStatus::Success;
        return value_.has_value() == succeeded &&
            diagnostic_.has_value() != succeeded &&
            (!value_ || value_->isValid()) &&
            (!diagnostic_ || diagnostic_->status == status_);
    }

private:
    KickPotGenerationResult(
        KickPotGenerationStatus status,
        std::optional<KickPotEvaluation> value,
        std::optional<KickPotGenerationDiagnostic> diagnostic)
        : status_(status), value_(std::move(value)), diagnostic_(std::move(diagnostic))
    {
    }

    KickPotGenerationStatus status_;
    std::optional<KickPotEvaluation> value_;
    std::optional<KickPotGenerationDiagnostic> diagnostic_;
};

enum class PotCandidateKind {
    Direct,
    Kick
};

struct ScoringRawCosts {
    double kickPenalty;
    double cuttingAngleDeg;
    double totalDistanceMm;
    std::optional<double> minimumClearanceMm;
    double pocketEntryAngleDeg;
    double kickRailAngleDeg;
};

struct ScoringNormalizedCosts {
    double kickPenalty;
    double cuttingAngle;
    double totalDistance;
    double clearanceRisk;
    double pocketEntryAngle;
    double kickRailAngleRisk;
};

struct ScoringNormalizationSnapshot {
    double effectiveWeightSumTolerance;
    double maxCutAngleDeg;
    double minDistanceMm;
    double maxDistanceMm;
    double blockedClearanceThresholdMm;
    double preferredClearanceMm;
    double pocketMaxEntryAngleDeg;
    std::optional<double> maxKickRailAngleDeg;
    double tieEpsilon;
};

struct PotScoringAudit {
    ScoringRawCosts rawCosts;
    ScoringNormalizedCosts normalizedCosts;
    BilliardConfig::ScoringWeights rawWeights;
    double rawWeightSum;
    BilliardConfig::ScoringWeights effectiveWeights;
    ScoringNormalizationSnapshot normalization;
    double totalCost;

    [[nodiscard]] bool isValid() const noexcept
    {
        const auto finiteNonNegative = [](double value) noexcept {
            return std::isfinite(value) && value >= 0.0;
        };
        const auto unitCost = [&](double value) noexcept {
            return finiteNonNegative(value) && value <= 1.0;
        };
        const std::array<double, 6> rawWeightValues{{
            rawWeights.kickPenalty,
            rawWeights.cuttingAngle,
            rawWeights.totalDistance,
            rawWeights.clearanceRisk,
            rawWeights.pocketEntryAngle,
            rawWeights.kickRailAngleRisk}};
        const std::array<double, 6> effectiveWeightValues{{
            effectiveWeights.kickPenalty,
            effectiveWeights.cuttingAngle,
            effectiveWeights.totalDistance,
            effectiveWeights.clearanceRisk,
            effectiveWeights.pocketEntryAngle,
            effectiveWeights.kickRailAngleRisk}};
        for (const double weight : rawWeightValues) {
            if (!finiteNonNegative(weight)) return false;
        }
        for (const double weight : effectiveWeightValues) {
            if (!finiteNonNegative(weight)) return false;
        }
        if (!finiteNonNegative(rawCosts.kickPenalty) ||
            (rawCosts.kickPenalty != 0.0 && rawCosts.kickPenalty != 1.0) ||
            !finiteNonNegative(rawCosts.cuttingAngleDeg) ||
            !finiteNonNegative(rawCosts.totalDistanceMm) ||
            (rawCosts.minimumClearanceMm &&
             !finiteNonNegative(*rawCosts.minimumClearanceMm)) ||
            !finiteNonNegative(rawCosts.pocketEntryAngleDeg) ||
            !finiteNonNegative(rawCosts.kickRailAngleDeg) ||
            !unitCost(normalizedCosts.kickPenalty) ||
            !unitCost(normalizedCosts.cuttingAngle) ||
            !unitCost(normalizedCosts.totalDistance) ||
            !unitCost(normalizedCosts.clearanceRisk) ||
            !unitCost(normalizedCosts.pocketEntryAngle) ||
            !unitCost(normalizedCosts.kickRailAngleRisk) ||
            !std::isfinite(rawWeightSum) || rawWeightSum <= 0.0 ||
            !finiteNonNegative(normalization.effectiveWeightSumTolerance) ||
            !std::isfinite(normalization.maxCutAngleDeg) ||
            normalization.maxCutAngleDeg <= 0.0 ||
            !finiteNonNegative(normalization.minDistanceMm) ||
            !std::isfinite(normalization.maxDistanceMm) ||
            normalization.maxDistanceMm <= normalization.minDistanceMm ||
            !finiteNonNegative(normalization.blockedClearanceThresholdMm) ||
            !std::isfinite(normalization.preferredClearanceMm) ||
            normalization.preferredClearanceMm <=
                normalization.blockedClearanceThresholdMm ||
            !std::isfinite(normalization.pocketMaxEntryAngleDeg) ||
            normalization.pocketMaxEntryAngleDeg <= 0.0 ||
            !finiteNonNegative(normalization.tieEpsilon) ||
            !unitCost(totalCost)) {
            return false;
        }
        const bool kick = rawCosts.kickPenalty == 1.0;
        if (kick != normalization.maxKickRailAngleDeg.has_value() ||
            (normalization.maxKickRailAngleDeg &&
             (!std::isfinite(*normalization.maxKickRailAngleDeg) ||
              *normalization.maxKickRailAngleDeg <= 0.0)) ||
            (!kick && (rawCosts.kickRailAngleDeg != 0.0 ||
                       normalizedCosts.kickRailAngleRisk != 0.0))) {
            return false;
        }
        long double calculatedRawSum = 0.0L;
        for (const double weight : rawWeightValues) {
            calculatedRawSum += static_cast<long double>(weight);
        }
        if (!std::isfinite(calculatedRawSum) ||
            calculatedRawSum >
                static_cast<long double>((std::numeric_limits<double>::max)()) ||
            static_cast<double>(calculatedRawSum) != rawWeightSum) {
            return false;
        }
        for (std::size_t index = 0; index < rawWeightValues.size(); ++index) {
            const double expected = rawWeightValues[index] / rawWeightSum;
            if (!std::isfinite(expected) ||
                std::fabs(effectiveWeightValues[index] - expected) >
                    normalization.effectiveWeightSumTolerance) {
                return false;
            }
        }
        const auto ratio = [](double value, double maximum) noexcept {
            return std::clamp(value / maximum, 0.0, 1.0);
        };
        const double expectedDistance = std::clamp(
            (rawCosts.totalDistanceMm - normalization.minDistanceMm) /
                (normalization.maxDistanceMm - normalization.minDistanceMm),
            0.0,
            1.0);
        const double expectedClearance = rawCosts.minimumClearanceMm
            ? 1.0 - std::clamp(
                (*rawCosts.minimumClearanceMm -
                 normalization.blockedClearanceThresholdMm) /
                    (normalization.preferredClearanceMm -
                     normalization.blockedClearanceThresholdMm),
                0.0,
                1.0)
            : 0.0;
        const double expectedKickRail = kick
            ? ratio(rawCosts.kickRailAngleDeg,
                *normalization.maxKickRailAngleDeg)
            : 0.0;
        const std::array<double, 6> expectedNormalized{{
            rawCosts.kickPenalty,
            ratio(rawCosts.cuttingAngleDeg, normalization.maxCutAngleDeg),
            expectedDistance,
            expectedClearance,
            ratio(rawCosts.pocketEntryAngleDeg,
                normalization.pocketMaxEntryAngleDeg),
            expectedKickRail}};
        const std::array<double, 6> actualNormalized{{
            normalizedCosts.kickPenalty,
            normalizedCosts.cuttingAngle,
            normalizedCosts.totalDistance,
            normalizedCosts.clearanceRisk,
            normalizedCosts.pocketEntryAngle,
            normalizedCosts.kickRailAngleRisk}};
        long double calculatedTotal = 0.0L;
        for (std::size_t index = 0; index < actualNormalized.size(); ++index) {
            if (std::fabs(actualNormalized[index] - expectedNormalized[index]) >
                normalization.effectiveWeightSumTolerance) {
                return false;
            }
            calculatedTotal += static_cast<long double>(actualNormalized[index]) *
                static_cast<long double>(effectiveWeightValues[index]);
        }
        if (!std::isfinite(calculatedTotal) || calculatedTotal < 0.0L ||
            calculatedTotal > 1.0L + static_cast<long double>(
                normalization.effectiveWeightSumTolerance)) {
            return false;
        }
        const double expectedTotal = std::clamp(
            static_cast<double>(calculatedTotal),
            0.0,
            1.0);
        const double effectiveSum = effectiveWeights.sum();
        return std::isfinite(effectiveSum) &&
            std::fabs(effectiveSum - 1.0) <=
                normalization.effectiveWeightSumTolerance &&
            std::fabs(totalCost - expectedTotal) <=
                normalization.effectiveWeightSumTolerance;
    }
};

using PotCandidateValue = std::variant<DirectPotCandidate, KickPotCandidate>;

struct ScoredPotCandidate {
    PotCandidateKind kind;
    EligibleTarget target;
    BilliardConfig::PocketId pocketId;
    std::optional<BilliardConfig::RailId> railId;
    PotCandidateValue candidate;
    PotScoringAudit audit;

    [[nodiscard]] bool isValid() const noexcept
    {
        const auto segmentLength = [](Segment2D segment) noexcept {
            return std::hypot(
                segment.end.x - segment.start.x,
                segment.end.y - segment.start.y);
        };
        if (target.ballNumber < 1 || target.ballNumber > 9 ||
            !std::isfinite(target.center.x) || !std::isfinite(target.center.y) ||
            static_cast<std::size_t>(pocketId) >= 6 || !audit.isValid()) {
            return false;
        }
        if (kind == PotCandidateKind::Direct) {
            const auto* direct = std::get_if<DirectPotCandidate>(&candidate);
            if (!direct || railId || audit.rawCosts.kickPenalty != 0.0 ||
                direct->target.ballNumber != target.ballNumber ||
                direct->target.center.x != target.center.x ||
                direct->target.center.y != target.center.y ||
                direct->pocketId != pocketId ||
                direct->cuttingAngleDeg != audit.rawCosts.cuttingAngleDeg ||
                direct->pocketEntryAngleDeg != audit.rawCosts.pocketEntryAngleDeg) {
                return false;
            }
            const double totalDistance = segmentLength(direct->cuePath) +
                segmentLength(direct->targetPath);
            if (!std::isfinite(totalDistance) ||
                totalDistance != audit.rawCosts.totalDistanceMm) {
                return false;
            }
            DirectPotEvaluation validation{target, {}, {}};
            for (std::size_t index = 0; index < validation.feasible.size(); ++index) {
                if (index == static_cast<std::size_t>(pocketId)) {
                    validation.feasible[index] = *direct;
                } else {
                    validation.rejected[index] = DirectPotCandidateDiagnostic{
                        static_cast<BilliardConfig::PocketId>(index),
                        DirectPotRejectionReason::CuePathInvalid,
                        GeometryStatus::DirectionRejected,
                        std::nullopt};
                }
            }
            return validation.isValid();
        }
        const auto* kick = std::get_if<KickPotCandidate>(&candidate);
        if (!kick || !railId || audit.rawCosts.kickPenalty != 1.0 ||
            static_cast<std::size_t>(*railId) >= 6 ||
            kick->target.ballNumber != target.ballNumber ||
            kick->target.center.x != target.center.x ||
            kick->target.center.y != target.center.y ||
            kick->pocketId != pocketId || kick->railId != *railId ||
            kick->cuttingAngleDeg != audit.rawCosts.cuttingAngleDeg ||
            kick->pocketEntryAngleDeg != audit.rawCosts.pocketEntryAngleDeg ||
            kick->incidenceAngleDeg != audit.rawCosts.kickRailAngleDeg) {
            return false;
        }
        const double totalDistance = segmentLength(kick->cuePathFirst) +
            segmentLength(kick->cuePathSecond) + segmentLength(kick->targetPath);
        if (!std::isfinite(totalDistance) ||
            totalDistance != audit.rawCosts.totalDistanceMm) {
            return false;
        }
        KickPotEvaluation validation{target, {}, {}};
        for (std::size_t pocket = 0; pocket < validation.feasible.size(); ++pocket) {
            for (std::size_t rail = 0; rail < validation.feasible[pocket].size(); ++rail) {
                if (pocket == static_cast<std::size_t>(pocketId) &&
                    rail == static_cast<std::size_t>(*railId)) {
                    validation.feasible[pocket][rail] = *kick;
                } else {
                    validation.rejected[pocket][rail] = KickPotCandidateDiagnostic{
                        static_cast<BilliardConfig::PocketId>(pocket),
                        static_cast<BilliardConfig::RailId>(rail),
                        KickPotRejectionReason::CueFirstSegmentInvalid,
                        GeometryStatus::DirectionRejected,
                        std::nullopt};
                }
            }
        }
        return validation.isValid();
    }
};

enum class PotNoPlanReason {
    NoPotCandidate
};

struct PotOnlyNoPlan {
    PotNoPlanReason reason;
    std::size_t feasiblePotCount;
    bool proceededToLegalContact;
    std::vector<DirectPotCandidateDiagnostic> directDiagnostics;
    std::vector<KickPotCandidateDiagnostic> kickDiagnostics;

    [[nodiscard]] bool isValid() const noexcept
    {
        return reason == PotNoPlanReason::NoPotCandidate &&
            feasiblePotCount == 0 && !proceededToLegalContact;
    }
};

using PotSelectionOutcome = std::variant<ScoredPotCandidate, PotOnlyNoPlan>;

enum class PotSelectionStatus {
    Success,
    ManualResearchPotSearchExhausted,
    ConfigurationMissing,
    InvalidConfiguration,
    InvalidCandidateInput,
    NumericalFailure
};

struct PotSelectionDiagnostic {
    PotSelectionStatus status;
};

class PotSelectionResult {
public:
    static PotSelectionResult rejected(PotSelectionStatus status)
    {
        return PotSelectionResult(
            status,
            std::nullopt,
            PotSelectionDiagnostic{status});
    }

    [[nodiscard]] PotSelectionStatus status() const noexcept { return status_; }
    [[nodiscard]] const std::optional<PotSelectionOutcome>& value() const noexcept
    {
        return value_;
    }
    [[nodiscard]] const std::optional<PotSelectionDiagnostic>& diagnostic() const noexcept
    {
        return diagnostic_;
    }
    [[nodiscard]] bool isValid() const noexcept
    {
        const bool succeeded = status_ == PotSelectionStatus::Success;
        const bool validValue = !value_ || std::visit(
            [](const auto& outcome) noexcept { return outcome.isValid(); },
            *value_);
        return value_.has_value() == succeeded && validValue &&
            diagnostic_.has_value() != succeeded &&
            (!diagnostic_ || diagnostic_->status == status_);
    }

private:
    friend class BilliardAlgorithm;

    static PotSelectionResult success(PotSelectionOutcome outcome)
    {
        return PotSelectionResult(
            PotSelectionStatus::Success,
            std::optional<PotSelectionOutcome>{std::move(outcome)},
            std::nullopt);
    }

    PotSelectionResult(
        PotSelectionStatus status,
        std::optional<PotSelectionOutcome> value,
        std::optional<PotSelectionDiagnostic> diagnostic)
        : status_(status), value_(std::move(value)), diagnostic_(std::move(diagnostic))
    {
    }

    PotSelectionStatus status_;
    std::optional<PotSelectionOutcome> value_;
    std::optional<PotSelectionDiagnostic> diagnostic_;
};
