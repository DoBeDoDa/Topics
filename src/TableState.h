// 定義視覺偵測結果、球桌狀態與目標選擇結果等領域資料。
#pragma once

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "BilliardPhysics.h"
#include "Point.h"

class BilliardAlgorithm;
class PlanningResult;

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

// P1-03嚴格輸入邊界使用的單幀型別。缺失編號球、母球或袋口只能以nullopt
// 表示，單幀缺席都不拒絕整幀——是否收斂為必要值統一交給
// ThreeEventStability的三幀累積判斷（母球在StableTableState仍是必要
// 欄位，只是不再靠單幀parse階段fail closed來保證）。
struct ParsedVisionFrame {
    std::array<std::optional<Point>, 9> objectBalls;
    std::optional<Point> cueBall;
    std::array<std::optional<Point>, 6> pockets;
};

class ValidatedVisionFrame {
public:
    ValidatedVisionFrame(
        std::array<std::optional<Point>, 9> numberedBalls,
        std::optional<Point> possibleCueBall,
        std::array<std::optional<Point>, 6> possiblePockets)
        : objectBalls(std::move(numberedBalls)),
          cueBall(possibleCueBall),
          pockets(std::move(possiblePockets))
    {
    }

    std::array<std::optional<Point>, 9> objectBalls;
    std::optional<Point> cueBall;
    std::array<std::optional<Point>, 6> pockets;
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
            nullptr,
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
            nullptr,
            Phase1PipelineDiagnostic{
                pipelineStatus,
                std::nullopt,
                std::move(diagnostic)}};
    }

    [[nodiscard]] static Phase1PipelineResult planningCompleted(
        PlanningResult result);

    [[nodiscard]] Phase1PipelineStatus status() const noexcept
    {
        return status_;
    }

    [[nodiscard]] const std::optional<Phase1PipelineDiagnostic>& diagnostic() const noexcept
    {
        return diagnostic_;
    }

    [[nodiscard]] const std::shared_ptr<const PlanningResult>& planningResult() const noexcept
    {
        return planningResult_;
    }

    [[nodiscard]] bool isValid() const noexcept;

private:
    Phase1PipelineResult(
        Phase1PipelineStatus status,
        std::shared_ptr<const PlanningResult> planningResult,
        std::optional<Phase1PipelineDiagnostic> diagnostic)
        : status_(status),
          planningResult_(std::move(planningResult)),
          diagnostic_(std::move(diagnostic))
    {
    }

    Phase1PipelineStatus status_;
    std::shared_ptr<const PlanningResult> planningResult_;
    std::optional<Phase1PipelineDiagnostic> diagnostic_;
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
            // presence一致性檢查下放到buildStableState()逐物件個別判斷
            // （見下方滑動視窗設計），這裡不再因單一物件（例如短暫閃爍的
            // 球）presence不一致就整批reset。
        }

        events_.push_back(std::move(event));
        if (events_.size() > 3) {
            // 滑動視窗：捨棄最舊、保留最新3筆，不因單一物件這一輪還沒收斂
            // 就把已經累積的其餘資料整批丟棄重來。
            events_.erase(events_.begin());
        }
        if (events_.size() < 3) {
            return StabilityResult::needMoreEvents(events_.size());
        }

        const auto stableState = buildStableState();
        if (!stableState.state) {
            // 不reset：視窗繼續往前滑動，讓還沒收斂的物件（cueBall或袋口
            // 未達容差）下一輪有機會追上；對呼叫端而言跟NeedMoreEvents
            // 走同一條「繼續餵事件、不中止本次capture」路徑。
            return StabilityResult::failure(
                StabilityStatus::NeedMoreEvents, stableState.reason, events_.size());
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
            event.eventId == 0) {
            return false;
        }
        if (event.frame.cueBall && !finitePoint(*event.frame.cueBall)) {
            return false;
        }
        for (const auto& ball : event.frame.objectBalls) {
            if (ball && !finitePoint(*ball)) {
                return false;
            }
        }
        for (const auto& pocket : event.frame.pockets) {
            if (pocket && !finitePoint(*pocket)) {
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
            const bool allAbsent =
                !events_[0].frame.objectBalls[index] &&
                !events_[1].frame.objectBalls[index] &&
                !events_[2].frame.objectBalls[index];
            if (allAbsent) {
                balls[index] = std::nullopt;
                continue;
            }
            const bool allPresent =
                events_[0].frame.objectBalls[index] &&
                events_[1].frame.objectBalls[index] &&
                events_[2].frame.objectBalls[index];
            if (!allPresent) {
                // 這3筆Logical Frame內presence不一致（mixed present/
                // absent）：既不是確定在桌上，也不是確定不在——絕不能
                // 擅自收斂成nullopt（等同「確定不在」）讓Phase1誤判成
                // 缺席，整批StableTableState這一輪不能定案，跟母球/袋口
                // 一樣走NeedMoreEvents讓視窗繼續滑動等收斂。
                return {std::nullopt, StabilityFailureReason::BallMoved};
            }
            const Point center = medianPoint(
                *events_[0].frame.objectBalls[index],
                *events_[1].frame.objectBalls[index],
                *events_[2].frame.objectBalls[index]);
            bool withinAll = true;
            for (const ReceiveEvent& event : events_) {
                if (!withinTolerance(
                        *event.frame.objectBalls[index],
                        center,
                        *config_.stableFrameToleranceMm)) {
                    withinAll = false;
                    break;
                }
            }
            if (!withinAll) {
                // 3筆都present但位置還沒收斂到容差內：同樣不得收斂成
                // nullopt，整批這一輪不能定案。
                return {std::nullopt, StabilityFailureReason::BallMoved};
            }
            balls[index] = center;
        }

        // 母球跟袋口一樣是StableTableState的必要（非optional）欄位：單幀
        // 缺席（閃爍）不再讓整批結果作廢，而是這一輪還無法定案，走
        // NeedMoreEvents讓視窗繼續往前滑，等母球在視窗內3筆都存在且
        // 一致時才收斂送出。
        const bool cueBallAllPresent = events_[0].frame.cueBall &&
            events_[1].frame.cueBall && events_[2].frame.cueBall;
        if (!cueBallAllPresent) {
            return {std::nullopt, StabilityFailureReason::BallMoved};
        }
        const Point cueBall = medianPoint(
            *events_[0].frame.cueBall,
            *events_[1].frame.cueBall,
            *events_[2].frame.cueBall);
        for (const ReceiveEvent& event : events_) {
            if (!withinTolerance(
                    *event.frame.cueBall,
                    cueBall,
                    *config_.stableFrameToleranceMm)) {
                return {std::nullopt, StabilityFailureReason::BallMoved};
            }
        }

        std::array<Point, 6> pockets{};
        for (std::size_t index = 0; index < pockets.size(); ++index) {
            // 袋口是StableTableState的必要（非optional）欄位，跟cueBall一樣：
            // 單幀缺席已在parse階段放行（跟編號球一樣可以先存），但這裡累積
            // 收斂時仍要求視窗內3筆都存在且互相在容差內，否則這一輪還不能
            // 定案——不當整批reset，而是走NeedMoreEvents繼續滑動。
            const bool allPresent =
                events_[0].frame.pockets[index] &&
                events_[1].frame.pockets[index] &&
                events_[2].frame.pockets[index];
            if (!allPresent) {
                return {std::nullopt, StabilityFailureReason::PocketMoved};
            }
            const Point center = medianPoint(
                *events_[0].frame.pockets[index],
                *events_[1].frame.pockets[index],
                *events_[2].frame.pockets[index]);
            for (const ReceiveEvent& event : events_) {
                if (!withinTolerance(
                        *event.frame.pockets[index],
                        center,
                        *config_.pocketStabilityToleranceMm)) {
                    return {std::nullopt, StabilityFailureReason::PocketMoved};
                }
            }
            pockets[index] = center;
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
    Point pocketTarget;
    GhostBallPoint ghostBallPoint;
    Segment2D cuePath;
    Segment2D targetPath;
    double cuttingAngleDeg;
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
                    !finitePoint(candidate.pocketTarget) ||
                    !finitePoint(candidate.ghostBallPoint.center) ||
                    !finitePoint(candidate.cuePath.start) ||
                    !finitePoint(candidate.cuePath.end) ||
                    !finitePoint(candidate.targetPath.start) ||
                    !finitePoint(candidate.targetPath.end) ||
                    !samePoint(candidate.cuePath.end, candidate.ghostBallPoint.center) ||
                    !samePoint(candidate.targetPath.start, target.center) ||
                    !samePoint(candidate.targetPath.end, candidate.pocketTarget) ||
                    !std::isfinite(candidate.cuttingAngleDeg) ||
                    candidate.cuttingAngleDeg < 0.0 || candidate.cuttingAngleDeg >= 90.0) {
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
    Point pocketTarget;
    GhostBallPoint ghostBallPoint;
    Point reboundPoint;
    Segment2D cuePathFirst;
    Segment2D cuePathSecond;
    Segment2D targetPath;
    double cuttingAngleDeg;
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
                        !finitePoint(candidate.pocketTarget) ||
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
                            candidate.pocketTarget) ||
                        !std::isfinite(candidate.cuttingAngleDeg) ||
                        candidate.cuttingAngleDeg < 0.0 ||
                        candidate.cuttingAngleDeg >= 90.0 ||
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
    double kickRailAngleDeg;
};

struct ScoringNormalizedCosts {
    double kickPenalty;
    double cuttingAngle;
    double totalDistance;
    double clearanceRisk;
    double kickRailAngleRisk;
};

struct ScoringNormalizationSnapshot {
    double effectiveWeightSumTolerance;
    double maxCutAngleDeg;
    double minDistanceMm;
    double maxDistanceMm;
    double blockedClearanceThresholdMm;
    double preferredClearanceMm;
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
        const std::array<double, 5> rawWeightValues{{
            rawWeights.kickPenalty,
            rawWeights.cuttingAngle,
            rawWeights.totalDistance,
            rawWeights.clearanceRisk,
            rawWeights.kickRailAngleRisk}};
        const std::array<double, 5> effectiveWeightValues{{
            effectiveWeights.kickPenalty,
            effectiveWeights.cuttingAngle,
            effectiveWeights.totalDistance,
            effectiveWeights.clearanceRisk,
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
            !finiteNonNegative(rawCosts.kickRailAngleDeg) ||
            !unitCost(normalizedCosts.kickPenalty) ||
            !unitCost(normalizedCosts.cuttingAngle) ||
            !unitCost(normalizedCosts.totalDistance) ||
            !unitCost(normalizedCosts.clearanceRisk) ||
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
        const std::array<double, 5> expectedNormalized{{
            rawCosts.kickPenalty,
            ratio(rawCosts.cuttingAngleDeg, normalization.maxCutAngleDeg),
            expectedDistance,
            expectedClearance,
            expectedKickRail}};
        const std::array<double, 5> actualNormalized{{
            normalizedCosts.kickPenalty,
            normalizedCosts.cuttingAngle,
            normalizedCosts.totalDistance,
            normalizedCosts.clearanceRisk,
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
                direct->cuttingAngleDeg != audit.rawCosts.cuttingAngleDeg) {
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

class RankedPotSelectionResult {
public:
    [[nodiscard]] static RankedPotSelectionResult success(
        std::vector<ScoredPotCandidate> ranked)
    {
        return RankedPotSelectionResult{
            PotSelectionStatus::Success,
            std::optional<std::vector<ScoredPotCandidate>>{std::move(ranked)},
            std::nullopt};
    }

    [[nodiscard]] static RankedPotSelectionResult rejected(
        PotSelectionStatus status)
    {
        return RankedPotSelectionResult{
            status, std::nullopt, PotSelectionDiagnostic{status}};
    }

    [[nodiscard]] PotSelectionStatus status() const noexcept { return status_; }
    [[nodiscard]] const std::optional<std::vector<ScoredPotCandidate>>& value()
        const noexcept
    {
        return value_;
    }
    [[nodiscard]] bool isValid() const noexcept
    {
        const bool success = status_ == PotSelectionStatus::Success;
        return value_.has_value() == success &&
            diagnostic_.has_value() != success &&
            (!value_ || std::all_of(
                value_->begin(), value_->end(),
                [](const ScoredPotCandidate& candidate) {
                    return candidate.isValid();
                })) &&
            (!diagnostic_ || diagnostic_->status == status_);
    }

private:
    RankedPotSelectionResult(
        PotSelectionStatus status,
        std::optional<std::vector<ScoredPotCandidate>> value,
        std::optional<PotSelectionDiagnostic> diagnostic)
        : status_(status),
          value_(std::move(value)),
          diagnostic_(std::move(diagnostic))
    {
    }

    PotSelectionStatus status_;
    std::optional<std::vector<ScoredPotCandidate>> value_;
    std::optional<PotSelectionDiagnostic> diagnostic_;
};

enum class ShotPlanType {
    DirectPot,
    KickPot,
    DirectLegalContact,
    KickLegalContact,
    // 保底：Pot與LegalContact候選都窮盡時，只求母球被安全推出、不要求
    // 碰到任何特定目標球/袋口（給對面自由球也可以接受）。
    CueBallContactOnly
};

enum class FixedForceMode {
    Fixed
};

struct Phase1PlanIdentity {
    ConnectionIdentity connectionIdentity;
    ShotCycleIdentity shotCycleIdentity;

    [[nodiscard]] bool isValid() const noexcept
    {
        return connectionIdentity != 0 && shotCycleIdentity != 0;
    }
};

struct PlanningSourceAudit {
    Phase1PlanIdentity planIdentity;
    std::array<StableSourceEventMetadata, 3> sourceEvents;
    std::string base0PlanarCalibrationRevision;
    std::string tableGeometryRevision;
    Point cueBallSnapshot;
    double ballRadiusMm;
    // 該輪穩定球位快照（index=球號-1，缺席代表這輪沒偵測到／已不在場上）。
    // 供Phase2 rear-obstacle check使用，不是Phase1選球依據。
    std::array<std::optional<Point>, 9> otherBallsSnapshot;

    [[nodiscard]] bool isValid() const noexcept
    {
        if (!planIdentity.isValid() || base0PlanarCalibrationRevision.empty() ||
            tableGeometryRevision.empty() || !std::isfinite(cueBallSnapshot.x) ||
            !std::isfinite(cueBallSnapshot.y) || !std::isfinite(ballRadiusMm) ||
            ballRadiusMm <= 0.0) {
            return false;
        }
        for (const auto& ball : otherBallsSnapshot) {
            if (ball && (!std::isfinite(ball->x) || !std::isfinite(ball->y))) {
                return false;
            }
        }
        for (std::size_t index = 0; index < sourceEvents.size(); ++index) {
            if (sourceEvents[index].eventId == 0 ||
                (index > 0 &&
                 (sourceEvents[index].eventId <= sourceEvents[index - 1].eventId ||
                  sourceEvents[index].receivedAt < sourceEvents[index - 1].receivedAt))) {
                return false;
            }
        }
        return true;
    }
};

struct Phase1ModelLimitations {
    bool fixedForceNotOptimized;
    bool postCollisionOutcomeNotModeled;
    bool spinFrictionAndEnergyLossNotModeled;

    [[nodiscard]] bool isValid() const noexcept
    {
        return fixedForceNotOptimized && postCollisionOutcomeNotModeled &&
            spinFrictionAndEnergyLossNotModeled;
    }
};

struct DirectPotShotPlanPayload {
    DirectPotCandidate candidate;
    PotScoringAudit scoring;

    [[nodiscard]] bool isValid() const noexcept
    {
        return ScoredPotCandidate{
            PotCandidateKind::Direct,
            candidate.target,
            candidate.pocketId,
            std::nullopt,
            PotCandidateValue{candidate},
            scoring}.isValid();
    }
};

struct KickPotShotPlanPayload {
    KickPotCandidate candidate;
    PotScoringAudit scoring;
    BilliardConfig::KickGeometryConfig kickGeometry;

    [[nodiscard]] bool isValid() const noexcept
    {
        if (!std::isfinite(kickGeometry.maxKickRailAngleDeg) ||
            kickGeometry.maxKickRailAngleDeg < 0.0 ||
            kickGeometry.maxKickRailAngleDeg > 90.0 ||
            !std::isfinite(kickGeometry.reflectionDirectionTolerance) ||
            kickGeometry.reflectionDirectionTolerance < 0.0 ||
            kickGeometry.reflectionDirectionTolerance > 2.0 ||
            !std::isfinite(kickGeometry.reflectionAngleToleranceDeg) ||
            kickGeometry.reflectionAngleToleranceDeg < 0.0 ||
            kickGeometry.reflectionAngleToleranceDeg > 180.0 ||
            !scoring.normalization.maxKickRailAngleDeg ||
            *scoring.normalization.maxKickRailAngleDeg !=
                kickGeometry.maxKickRailAngleDeg) {
            return false;
        }
        return ScoredPotCandidate{
            PotCandidateKind::Kick,
            candidate.target,
            candidate.pocketId,
            std::optional<BilliardConfig::RailId>{candidate.railId},
            PotCandidateValue{candidate},
            scoring}.isValid();
    }
};

enum class LegalContactRejectionReason {
    GhostGeometryInvalid,
    RailGeometryInvalid,
    NoRailIntersection,
    FirstSegmentInvalid,
    FirstSegmentBlocked,
    SecondSegmentInvalid,
    SecondSegmentBlocked,
    ReflectionInvariantFailed,
    KickAngleRejected,
    NumericalFailure
};

struct LegalContactCandidateDiagnostic {
    std::optional<BilliardConfig::RailId> railId;
    LegalContactRejectionReason reason;
    GeometryStatus geometryStatus;
    std::optional<std::size_t> relatedObstacleIndex;
};

struct DirectLegalContactCandidate {
    EligibleTarget target;
    GhostBallPoint ghostBallPoint;
    Segment2D cuePath;
    std::optional<double> minimumClearanceMm;
    double totalPathLengthMm;
};

struct KickLegalContactCandidate {
    EligibleTarget target;
    BilliardConfig::RailId railId;
    GhostBallPoint ghostBallPoint;
    Point reboundPoint;
    Segment2D effectiveRailReference;
    Segment2D cuePathFirst;
    Segment2D cuePathSecond;
    std::optional<double> minimumClearanceMm;
    double totalPathLengthMm;
    double incidenceAngleDeg;
};

struct LegalContactAuditFields {
    enum class ActivationAuthority {
        ManualResearch,
        ProductionFallbackEligible
    };

    bool legalFirstContactGuaranteed;
    GhostBallPoint selectedContactGhostBallPoint;
    bool directPriorityApplied;
    std::optional<double> minimumClearanceMm;
    double totalPathLengthMm;
    std::optional<BilliardConfig::RailId> railId;
    bool potSearchExhausted;
    PotSelectionStatus potSearchStatus;
    bool requiresExplicitExecutionAuthorization;
    bool realHardwareExecutionDefaultEnabled;
    ActivationAuthority activationAuthority;
    std::vector<LegalContactCandidateDiagnostic> candidateDiagnostics;

    [[nodiscard]] bool isValid() const noexcept
    {
        return legalFirstContactGuaranteed &&
            std::isfinite(selectedContactGhostBallPoint.center.x) &&
            std::isfinite(selectedContactGhostBallPoint.center.y) &&
            directPriorityApplied &&
            (!minimumClearanceMm ||
             (std::isfinite(*minimumClearanceMm) && *minimumClearanceMm >= 0.0)) &&
            std::isfinite(totalPathLengthMm) && totalPathLengthMm > 0.0 &&
            (!railId || static_cast<std::size_t>(*railId) < 6) &&
            ((activationAuthority == ActivationAuthority::ManualResearch &&
              potSearchExhausted &&
              potSearchStatus ==
                  PotSelectionStatus::ManualResearchPotSearchExhausted) ||
             (activationAuthority ==
                  ActivationAuthority::ProductionFallbackEligible &&
              (potSearchStatus == PotSelectionStatus::Success ||
               potSearchStatus ==
                   PotSelectionStatus::ManualResearchPotSearchExhausted))) &&
            requiresExplicitExecutionAuthorization &&
            !realHardwareExecutionDefaultEnabled;
    }
};

struct DirectLegalContactShotPlanPayload {
    DirectLegalContactCandidate candidate;
    LegalContactAuditFields audit;
};

struct KickLegalContactShotPlanPayload {
    KickLegalContactCandidate candidate;
    BilliardConfig::KickGeometryConfig kickGeometry;
    LegalContactAuditFields audit;
};

// CueBallContactOnly：沒有真正的目標球幾何，executionDirectionXY就是唯一
// 權威的執行方向（跟ShotPlan.shotDirectionXY逐位元相同）。
struct CueBallContactOnlyShotPlanPayload {
    Vector2D executionDirectionXY;

    [[nodiscard]] bool isValid() const noexcept
    {
        const double length = std::hypot(
            executionDirectionXY.x, executionDirectionXY.y);
        return std::isfinite(length) && std::fabs(length - 1.0) < 1e-9;
    }
};

using ShotPlanPayload = std::variant<
    DirectPotShotPlanPayload,
    KickPotShotPlanPayload,
    DirectLegalContactShotPlanPayload,
    KickLegalContactShotPlanPayload,
    CueBallContactOnlyShotPlanPayload>;

struct ShotPlan {
    ShotPlanType type;
    PlanningSourceAudit source;
    EligibleTarget selectedTarget;
    Vector2D shotDirectionXY;
    GhostBallPoint ghostBallPoint;
    std::vector<Segment2D> cuePathSegments;
    std::optional<double> minimumClearanceMm;
    FixedForceMode forceMode;
    Phase1ModelLimitations limitations;
    std::vector<DirectPotCandidateDiagnostic> directCandidateDiagnostics;
    std::vector<KickPotCandidateDiagnostic> kickCandidateDiagnostics;
    ShotPlanPayload payload;

    [[nodiscard]] bool isValid() const noexcept
    {
        constexpr double UNIT_VECTOR_TOLERANCE = 1e-9;
        const auto samePoint = [](Point first, Point second) noexcept {
            return first.x == second.x && first.y == second.y;
        };
        const double directionLength = std::hypot(
            shotDirectionXY.x,
            shotDirectionXY.y);
        const Vector2D firstPathDirection{
            cuePathSegments.empty()
                ? std::numeric_limits<double>::quiet_NaN()
                : cuePathSegments.front().end.x - cuePathSegments.front().start.x,
            cuePathSegments.empty()
                ? std::numeric_limits<double>::quiet_NaN()
                : cuePathSegments.front().end.y - cuePathSegments.front().start.y};
        const double firstPathLength = std::hypot(
            firstPathDirection.x,
            firstPathDirection.y);
        if (!source.isValid() || selectedTarget.ballNumber < 1 ||
            selectedTarget.ballNumber > 9 || !std::isfinite(selectedTarget.center.x) ||
            !std::isfinite(selectedTarget.center.y) ||
            !std::isfinite(directionLength) ||
            std::fabs(directionLength - 1.0) > UNIT_VECTOR_TOLERANCE ||
            !std::isfinite(ghostBallPoint.center.x) ||
            !std::isfinite(ghostBallPoint.center.y) || cuePathSegments.empty() ||
            cuePathSegments.size() > 2 ||
            !std::isfinite(firstPathLength) || firstPathLength <= 0.0 ||
            std::fabs(shotDirectionXY.x -
                firstPathDirection.x / firstPathLength) > UNIT_VECTOR_TOLERANCE ||
            std::fabs(shotDirectionXY.y -
                firstPathDirection.y / firstPathLength) > UNIT_VECTOR_TOLERANCE ||
            (minimumClearanceMm &&
             (!std::isfinite(*minimumClearanceMm) || *minimumClearanceMm < 0.0)) ||
            forceMode != FixedForceMode::Fixed || !limitations.isValid()) {
            return false;
        }
        if (!samePoint(cuePathSegments.front().start, source.cueBallSnapshot) ||
            !samePoint(cuePathSegments.back().end, ghostBallPoint.center)) {
            return false;
        }
        if (type == ShotPlanType::DirectPot) {
            const auto* direct = std::get_if<DirectPotShotPlanPayload>(&payload);
            return direct && cuePathSegments.size() == 1 && direct->isValid() &&
                direct->candidate.target.ballNumber == selectedTarget.ballNumber &&
                samePoint(direct->candidate.target.center, selectedTarget.center) &&
                samePoint(direct->candidate.ghostBallPoint.center,
                    ghostBallPoint.center) &&
                samePoint(direct->candidate.cuePath.start,
                    cuePathSegments[0].start) &&
                samePoint(direct->candidate.cuePath.end,
                    cuePathSegments[0].end) &&
                direct->scoring.rawCosts.minimumClearanceMm == minimumClearanceMm;
        }
        if (type == ShotPlanType::KickPot) {
            const auto* kick = std::get_if<KickPotShotPlanPayload>(&payload);
            return kick && cuePathSegments.size() == 2 && kick->isValid() &&
                kick->candidate.target.ballNumber == selectedTarget.ballNumber &&
                samePoint(kick->candidate.target.center, selectedTarget.center) &&
                samePoint(kick->candidate.ghostBallPoint.center,
                    ghostBallPoint.center) &&
                samePoint(kick->candidate.cuePathFirst.start,
                    cuePathSegments[0].start) &&
                samePoint(kick->candidate.cuePathFirst.end,
                    cuePathSegments[0].end) &&
                samePoint(kick->candidate.cuePathSecond.start,
                    cuePathSegments[1].start) &&
                samePoint(kick->candidate.cuePathSecond.end,
                    cuePathSegments[1].end) &&
                kick->scoring.rawCosts.minimumClearanceMm == minimumClearanceMm;
        }
        if (type == ShotPlanType::CueBallContactOnly) {
            const auto* fallback =
                std::get_if<CueBallContactOnlyShotPlanPayload>(&payload);
            return fallback && cuePathSegments.size() == 1 &&
                fallback->isValid() &&
                fallback->executionDirectionXY.x == shotDirectionXY.x &&
                fallback->executionDirectionXY.y == shotDirectionXY.y;
        }
        const double contactDistance = std::hypot(
            selectedTarget.center.x - ghostBallPoint.center.x,
            selectedTarget.center.y - ghostBallPoint.center.y);
        if (!std::isfinite(contactDistance) ||
            std::fabs(contactDistance - 2.0 * source.ballRadiusMm) >
                UNIT_VECTOR_TOLERANCE) {
            return false;
        }
        if (type == ShotPlanType::DirectLegalContact) {
            const auto* direct =
                std::get_if<DirectLegalContactShotPlanPayload>(&payload);
            const double pathLength = direct
                ? std::hypot(
                    direct->candidate.cuePath.end.x -
                        direct->candidate.cuePath.start.x,
                    direct->candidate.cuePath.end.y -
                        direct->candidate.cuePath.start.y)
                : std::numeric_limits<double>::quiet_NaN();
            return direct && cuePathSegments.size() == 1 &&
                direct->audit.isValid() &&
                samePoint(direct->audit.selectedContactGhostBallPoint.center,
                    ghostBallPoint.center) &&
                direct->audit.minimumClearanceMm == minimumClearanceMm &&
                direct->audit.totalPathLengthMm ==
                    direct->candidate.totalPathLengthMm &&
                !direct->audit.railId &&
                direct->candidate.target.ballNumber == selectedTarget.ballNumber &&
                samePoint(direct->candidate.target.center, selectedTarget.center) &&
                samePoint(direct->candidate.ghostBallPoint.center,
                    ghostBallPoint.center) &&
                samePoint(direct->candidate.cuePath.start,
                    cuePathSegments[0].start) &&
                samePoint(direct->candidate.cuePath.end,
                    cuePathSegments[0].end) &&
                direct->candidate.minimumClearanceMm == minimumClearanceMm &&
                std::isfinite(direct->candidate.totalPathLengthMm) &&
                std::isfinite(pathLength) && pathLength > 0.0 &&
                std::fabs(direct->candidate.totalPathLengthMm - pathLength) <=
                    UNIT_VECTOR_TOLERANCE;
        }
        const auto* kick =
            std::get_if<KickLegalContactShotPlanPayload>(&payload);
        if (!kick || cuePathSegments.size() != 2 || !kick->audit.isValid() ||
            !samePoint(kick->audit.selectedContactGhostBallPoint.center,
                ghostBallPoint.center) ||
            kick->audit.minimumClearanceMm != minimumClearanceMm ||
            kick->audit.totalPathLengthMm !=
                kick->candidate.totalPathLengthMm ||
            kick->audit.railId != kick->candidate.railId ||
            !std::isfinite(kick->kickGeometry.maxKickRailAngleDeg) ||
            kick->kickGeometry.maxKickRailAngleDeg < 0.0 ||
            kick->kickGeometry.maxKickRailAngleDeg > 90.0 ||
            !std::isfinite(kick->kickGeometry.reflectionDirectionTolerance) ||
            kick->kickGeometry.reflectionDirectionTolerance < 0.0 ||
            kick->kickGeometry.reflectionDirectionTolerance > 2.0 ||
            !std::isfinite(kick->kickGeometry.reflectionAngleToleranceDeg) ||
            kick->kickGeometry.reflectionAngleToleranceDeg < 0.0 ||
            kick->kickGeometry.reflectionAngleToleranceDeg > 180.0) {
            return false;
        }
        const auto segmentLength = [](Segment2D segment) noexcept {
            return std::hypot(
                segment.end.x - segment.start.x,
                segment.end.y - segment.start.y);
        };
        const double firstLength = segmentLength(kick->candidate.cuePathFirst);
        const double secondLength = segmentLength(kick->candidate.cuePathSecond);
        const Segment2D rail = kick->candidate.effectiveRailReference;
        const Vector2D railVector{
            rail.end.x - rail.start.x,
            rail.end.y - rail.start.y};
        const Vector2D reboundOffset{
            kick->candidate.reboundPoint.x - rail.start.x,
            kick->candidate.reboundPoint.y - rail.start.y};
        const double railLengthSquared =
            railVector.x * railVector.x + railVector.y * railVector.y;
        const double railCross =
            railVector.x * reboundOffset.y - railVector.y * reboundOffset.x;
        const double railProjection =
            railVector.x * reboundOffset.x + railVector.y * reboundOffset.y;
        return
            static_cast<std::size_t>(kick->candidate.railId) < 6 &&
            std::isfinite(railLengthSquared) && railLengthSquared > 0.0 &&
            std::isfinite(railCross) &&
            std::fabs(railCross) <= UNIT_VECTOR_TOLERANCE &&
            std::isfinite(railProjection) && railProjection >= 0.0 &&
            railProjection <= railLengthSquared &&
            kick->candidate.target.ballNumber == selectedTarget.ballNumber &&
            samePoint(kick->candidate.target.center, selectedTarget.center) &&
            samePoint(kick->candidate.ghostBallPoint.center,
                ghostBallPoint.center) &&
            samePoint(kick->candidate.reboundPoint,
                kick->candidate.cuePathFirst.end) &&
            samePoint(kick->candidate.reboundPoint,
                kick->candidate.cuePathSecond.start) &&
            samePoint(kick->candidate.cuePathFirst.start,
                cuePathSegments[0].start) &&
            samePoint(kick->candidate.cuePathSecond.end,
                cuePathSegments[1].end) &&
            kick->candidate.minimumClearanceMm == minimumClearanceMm &&
            std::isfinite(kick->candidate.totalPathLengthMm) &&
            std::isfinite(firstLength) && firstLength > 0.0 &&
            std::isfinite(secondLength) && secondLength > 0.0 &&
            std::fabs(kick->candidate.totalPathLengthMm -
                (firstLength + secondLength)) <= UNIT_VECTOR_TOLERANCE &&
            std::isfinite(kick->candidate.incidenceAngleDeg) &&
            kick->candidate.incidenceAngleDeg >= 0.0 &&
            kick->candidate.incidenceAngleDeg <=
                kick->kickGeometry.maxKickRailAngleDeg;
    }
};

enum class NoPlanReason {
    NoEligibleTarget,
    NoPotCandidate,
    NoLegalContact,
    InvalidBrainConfiguration,
    NumericalPlanningFailure
};

struct PlanningDiagnostic {
    std::optional<TargetQualificationStatus> targetStatus;
    std::optional<GeometryStatus> geometryStatus;
    std::optional<DirectPotGenerationStatus> directStatus;
    std::optional<KickPotGenerationStatus> kickStatus;
    std::optional<PotSelectionStatus> selectionStatus;
};

struct NoPlan {
    NoPlanReason reason;
    std::optional<PlanningSourceAudit> source;
    std::optional<EligibleTarget> selectedTarget;
    std::size_t feasiblePotCount;
    bool proceededToLegalContact;
    std::vector<DirectPotCandidateDiagnostic> directCandidateDiagnostics;
    std::vector<KickPotCandidateDiagnostic> kickCandidateDiagnostics;
    std::vector<LegalContactCandidateDiagnostic> legalContactDiagnostics;
    PlanningDiagnostic diagnostic;

    [[nodiscard]] bool isValid() const noexcept
    {
        if (feasiblePotCount != 0) {
            return false;
        }
        if (source && !source->isValid()) {
            return false;
        }
        if (reason == NoPlanReason::NoEligibleTarget) {
            return !proceededToLegalContact && source && !selectedTarget &&
                diagnostic.targetStatus ==
                    TargetQualificationStatus::NoEligibleTarget;
        }
        if (reason == NoPlanReason::NoPotCandidate) {
            return !proceededToLegalContact && source && selectedTarget &&
                selectedTarget->ballNumber >= 1 && selectedTarget->ballNumber <= 9 &&
                diagnostic.selectionStatus == PotSelectionStatus::Success;
        }
        if (reason == NoPlanReason::NoLegalContact) {
            return proceededToLegalContact && source && selectedTarget &&
                selectedTarget->ballNumber >= 1 && selectedTarget->ballNumber <= 9 &&
                diagnostic.selectionStatus ==
                    PotSelectionStatus::ManualResearchPotSearchExhausted &&
                !legalContactDiagnostics.empty();
        }
        if (reason == NoPlanReason::InvalidBrainConfiguration) {
            return !proceededToLegalContact &&
                (diagnostic.geometryStatus == GeometryStatus::ConfigurationMissing ||
                diagnostic.geometryStatus == GeometryStatus::InvalidConfiguration ||
                diagnostic.kickStatus == KickPotGenerationStatus::ConfigurationMissing ||
                diagnostic.kickStatus == KickPotGenerationStatus::InvalidGeometryConfiguration ||
                diagnostic.selectionStatus == PotSelectionStatus::ConfigurationMissing ||
                diagnostic.selectionStatus == PotSelectionStatus::InvalidConfiguration ||
                (!source && !diagnostic.targetStatus && !diagnostic.geometryStatus &&
                 !diagnostic.directStatus && !diagnostic.kickStatus &&
                 !diagnostic.selectionStatus));
        }
        return !proceededToLegalContact &&
            (diagnostic.targetStatus == TargetQualificationStatus::InvalidStableState ||
            diagnostic.directStatus == DirectPotGenerationStatus::InvalidStableState ||
            diagnostic.directStatus == DirectPotGenerationStatus::SelectedTargetMismatch ||
            diagnostic.kickStatus == KickPotGenerationStatus::InvalidStableState ||
            diagnostic.kickStatus == KickPotGenerationStatus::SelectedTargetMismatch ||
            diagnostic.selectionStatus == PotSelectionStatus::InvalidCandidateInput ||
            diagnostic.selectionStatus == PotSelectionStatus::NumericalFailure);
    }
};

using PlanningOutcome = std::variant<ShotPlan, NoPlan>;

struct Phase1ExecutionCandidates {
    std::vector<ShotPlan> rankedPotPlans;
    std::vector<ShotPlan> legalContactPlans;
    // Pot與LegalContact都窮盡時的保底候選；只有兩者都空時才會產生。
    std::vector<ShotPlan> cueBallContactOnlyPlans;

    [[nodiscard]] bool isValid() const noexcept
    {
        const ShotPlan* anchor = !rankedPotPlans.empty()
            ? &rankedPotPlans.front()
            : (!legalContactPlans.empty() ? &legalContactPlans.front()
                : (!cueBallContactOnlyPlans.empty()
                    ? &cueBallContactOnlyPlans.front() : nullptr));
        const auto samePhase1Target = [anchor](const ShotPlan& plan) {
            return !anchor ||
                (plan.source.planIdentity.connectionIdentity ==
                     anchor->source.planIdentity.connectionIdentity &&
                 plan.source.planIdentity.shotCycleIdentity ==
                     anchor->source.planIdentity.shotCycleIdentity &&
                 plan.selectedTarget.ballNumber ==
                     anchor->selectedTarget.ballNumber &&
                 plan.selectedTarget.center.x ==
                     anchor->selectedTarget.center.x &&
                 plan.selectedTarget.center.y ==
                     anchor->selectedTarget.center.y);
        };
        return std::all_of(
                   rankedPotPlans.begin(), rankedPotPlans.end(),
                   [&](const ShotPlan& plan) {
                       return plan.isValid() &&
                           samePhase1Target(plan) &&
                           (plan.type == ShotPlanType::DirectPot ||
                            plan.type == ShotPlanType::KickPot);
                   }) &&
            std::all_of(
                legalContactPlans.begin(), legalContactPlans.end(),
                [&](const ShotPlan& plan) {
                    return plan.isValid() &&
                        samePhase1Target(plan) &&
                        (plan.type == ShotPlanType::DirectLegalContact ||
                         plan.type == ShotPlanType::KickLegalContact);
                }) &&
            std::all_of(
                cueBallContactOnlyPlans.begin(), cueBallContactOnlyPlans.end(),
                [&](const ShotPlan& plan) {
                    return plan.isValid() && samePhase1Target(plan) &&
                        plan.type == ShotPlanType::CueBallContactOnly;
                });
    }
};

class PlanningResult {
public:
    static PlanningResult shotPlan(
        ShotPlan value,
        Phase1ExecutionCandidates candidates = {})
    {
        return PlanningResult{
            PlanningOutcome{std::move(value)}, std::move(candidates)};
    }

    static PlanningResult noPlan(
        NoPlan value,
        Phase1ExecutionCandidates candidates = {})
    {
        return PlanningResult{
            PlanningOutcome{std::move(value)}, std::move(candidates)};
    }

    [[nodiscard]] const PlanningOutcome& value() const noexcept { return value_; }
    [[nodiscard]] const Phase1ExecutionCandidates& executionCandidates()
        const noexcept
    {
        return executionCandidates_;
    }

    [[nodiscard]] bool isValid() const noexcept
    {
        return std::visit(
            [](const auto& value) noexcept { return value.isValid(); },
            value_) && executionCandidates_.isValid();
    }

private:
    PlanningResult(
        PlanningOutcome value,
        Phase1ExecutionCandidates candidates)
        : value_(std::move(value)),
          executionCandidates_(std::move(candidates))
    {
    }

    PlanningOutcome value_;
    Phase1ExecutionCandidates executionCandidates_;
};

inline Phase1PipelineResult Phase1PipelineResult::planningCompleted(
    PlanningResult result)
{
    return Phase1PipelineResult{
        Phase1PipelineStatus::PlanningCompleted,
        std::make_shared<const PlanningResult>(std::move(result)),
        std::nullopt};
}

inline bool Phase1PipelineResult::isValid() const noexcept
{
    if (status_ == Phase1PipelineStatus::PlanningCompleted) {
        return planningResult_ && planningResult_->isValid() && !diagnostic_;
    }
    if (planningResult_ || !diagnostic_ || diagnostic_->status != status_) {
        return false;
    }
    if (status_ == Phase1PipelineStatus::InputFailure) {
        return diagnostic_->inputDiagnostic.has_value() &&
            !diagnostic_->stabilityDiagnostic.has_value();
    }
    if (status_ == Phase1PipelineStatus::Waiting) {
        return !diagnostic_->inputDiagnostic.has_value() &&
            diagnostic_->stabilityDiagnostic.has_value() &&
            diagnostic_->stabilityDiagnostic->status == StabilityStatus::NeedMoreEvents;
    }
    return status_ == Phase1PipelineStatus::StabilityFailure &&
        !diagnostic_->inputDiagnostic.has_value() &&
        diagnostic_->stabilityDiagnostic.has_value() &&
        diagnostic_->stabilityDiagnostic->status != StabilityStatus::NeedMoreEvents &&
        diagnostic_->stabilityDiagnostic->status != StabilityStatus::Stable;
}
