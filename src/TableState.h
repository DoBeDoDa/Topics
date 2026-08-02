// 定義視覺偵測結果、球桌狀態與目標選擇結果等領域資料。
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "Point.h"

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

// 從 TableState 選出的本次擊球目標與障礙資料。
struct TargetSelection {
    Point cueBall;
    Point targetBall;
    Point destinationPocket;
    Point railA;
    Point railB;
    std::vector<Point> obstacles;
    int targetBallNumber = -1;
    int pocketNumber = -1;
    double pocketAngleDeg = 0.0;
    std::string targetName;
};
