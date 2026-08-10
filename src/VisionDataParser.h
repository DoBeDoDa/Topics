// 宣告既有32值視覺CSV的嚴格單幀解析與本地ReceiveEvent邊界。
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "TableState.h"

class VisionDataParser {
public:
    VisionDataParser();
    explicit VisionDataParser(AxisAlignedBounds2D observationBounds);
    explicit VisionDataParser(std::optional<AxisAlignedBounds2D> observationBounds);

    [[nodiscard]] SingleFrameStatus configurationStatus() const noexcept;
    [[nodiscard]] SingleFrameResult parse(const std::string& message) const;

private:
    std::optional<AxisAlignedBounds2D> observationBounds_;
};

class ReceiveEventFactory {
public:
    explicit ReceiveEventFactory(const VisionDataParser& parser);

    void beginCycle(
        ConnectionIdentity connectionIdentity,
        ShotCycleIdentity shotCycleIdentity);
    [[nodiscard]] bool openCaptureWindow(
        ConnectionIdentity connectionIdentity,
        ShotCycleIdentity shotCycleIdentity);
    void invalidate(ReceiveEventInvalidationReason reason);

    [[nodiscard]] ReceiveEventResult accept(
        const std::string& payload,
        ConnectionIdentity connectionIdentity,
        std::chrono::steady_clock::time_point receivedAt);

    [[nodiscard]] bool hasActiveCycle() const noexcept;
    [[nodiscard]] bool isCaptureWindowOpen() const noexcept;
    [[nodiscard]] ShotCycleIdentity currentCycleIdentity() const noexcept;
    [[nodiscard]] ReceiveEventInvalidationReason lastInvalidationReason() const noexcept;

private:
    const VisionDataParser& parser_;
    ConnectionIdentity connectionIdentity_;
    ShotCycleIdentity shotCycleIdentity_;
    ReceiveEventId nextEventId_;
    std::chrono::steady_clock::time_point lastReceiveTime_;
    bool hasLastReceiveTime_;
    bool activeCycle_;
    bool captureWindowOpen_;
    ReceiveEventInvalidationReason lastInvalidationReason_;
};
