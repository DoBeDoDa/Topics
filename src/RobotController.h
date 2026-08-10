// 宣告 HRSDK 控制器封裝、動作結果與手臂控制介面。
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>

#include "MotionPlanner.h"

struct HrSdkApi {
    std::function<int(const char*)> openConnection;
    std::function<void(int)> closeConnection;
    std::function<int(int)> clearAlarm;
    std::function<int(int, int)> setMotorState;
    std::function<int(int, int)> setOverrideRatio;
    std::function<int(int, int)> setToolNumber;
    std::function<int(int, int)> setBaseNumber;
    std::function<int(int)> getToolNumber;
    std::function<int(int)> getBaseNumber;
    std::function<int(int, double*)> getCurrentPosition;
    std::function<int(int, double*)> getCurrentJoints;
    std::function<int(int, double*, bool&)> checkReachable;
    std::function<int(int, double*, double*, bool&)> checkLinearPath;
    std::function<int(int, int, double*)> movePtpPosition;
    std::function<int(int, int, double, double*)> moveLinearPosition;
    std::function<int(int, int, double*)> movePtpAxis;
    std::function<int(int)> getMotionState;
    std::function<int(int)> abortMotion;
    std::function<int(int, int, bool)> setDigitalOutput;
    std::function<int(int, int)> getDigitalOutput;
    std::function<int(int, int&, std::uint64_t*)> getAlarmCodes;
    std::function<unsigned long()> tickCountMs;
    std::function<void(unsigned long)> sleepMs;
};

enum class RobotAdapterStatus {
    Success,
    ConfigurationMissing,
    InvalidConfiguration,
    Unauthorized,
    NotConnected,
    SdkFailure,
    NotReachable,
    NotStopped,
    UnknownUnsafe
};

struct RobotAdapterResult {
    RobotAdapterStatus status;
    int sdkCode;

    [[nodiscard]] bool succeeded() const noexcept
    {
        return status == RobotAdapterStatus::Success;
    }
};

struct RobotPoseAdapterResult {
    RobotAdapterStatus status;
    int sdkCode;
    std::optional<RobotPoseABC> value;

    [[nodiscard]] bool isValid() const noexcept
    {
        return (status == RobotAdapterStatus::Success) == value.has_value() &&
            (!value || value->isFinite());
    }
};

struct HrSdkPoseResult {
    RobotAdapterStatus status;
    std::optional<std::array<double, 6>> value;

    [[nodiscard]] bool isValid() const noexcept
    {
        return (status == RobotAdapterStatus::Success) == value.has_value();
    }
};

enum class RealPneumaticStatus {
    Completed,
    KnownSafeFailure,
    UnknownUnsafe
};

enum class RealPneumaticEvidence {
    OutputOffConfirmed
};

struct RealPneumaticResult {
    RealPneumaticStatus status;
    std::optional<RealPneumaticEvidence> evidence;
    int sdkCode;

    [[nodiscard]] bool isValid() const noexcept
    {
        const bool knownStatus = status == RealPneumaticStatus::Completed ||
            status == RealPneumaticStatus::KnownSafeFailure ||
            status == RealPneumaticStatus::UnknownUnsafe;
        const bool knownEvidence = !evidence ||
            *evidence == RealPneumaticEvidence::OutputOffConfirmed;
        return knownStatus && knownEvidence &&
            ((status == RealPneumaticStatus::Completed) == evidence.has_value());
    }
};

struct MotionResult {
    bool success;
    bool timedOut;
    int sdkCode;
    int finalMotionState;
    int abortSdkCode;

    MotionResult()
        : success(false), timedOut(false), sdkCode(-1), finalMotionState(-1),
          abortSdkCode(-1) {}
};

class RobotController {
private:
    int id;
    bool connected;
    bool unknownUnsafeLatched;
    HrSdkApi api;

    MotionResult waitForMotion(int sdkCode, bool wait);
    [[nodiscard]] static HrSdkApi productionApi();
    [[nodiscard]] RobotAdapterResult validateRealExecution(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
        const;
    [[nodiscard]] HrSdkPoseResult mapPoseToHrSdk(
        const RobotPoseABC& pose,
        const BilliardConfig::HrSdkAngleMappingConfig& mapping) const;
    [[nodiscard]] RobotPoseAdapterResult mapPoseFromHrSdk(
        const std::array<double, 6>& pose,
        const BilliardConfig::HrSdkAngleMappingConfig& mapping) const;
    [[nodiscard]] bool outputsElectricallyOff(
        const BilliardConfig::RealHardwareExecutionConfig& config,
        int& sdkCode) const;
    [[nodiscard]] bool bestEffortOutputsOff(
        const BilliardConfig::RealHardwareExecutionConfig& config,
        int& sdkCode);
    [[nodiscard]] RealPneumaticResult latchUnknownUnsafe(int sdkCode) noexcept;
    [[nodiscard]] RealPneumaticResult pulseOutput(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config,
        int activeOutput,
        int mutuallyExclusiveOutput);

public:
    RobotController();
    explicit RobotController(HrSdkApi injectedApi);
    ~RobotController();

    bool connect(const std::string& ip);
    RobotAdapterResult disconnect();
    RobotAdapterResult clearAlarm();
    int getId() const;
    bool isConnected() const;

    RobotAdapterResult setMotorState(int state);
    RobotAdapterResult setOverrideRatio(int ratio);
    RobotAdapterResult setToolNumber(int toolNumber);
    RobotAdapterResult setBaseNumber(int baseNumber);
    int getCurrentToolNumber() const;
    int getCurrentBaseNumber() const;
    bool getCurrentPosition(std::array<double, 6>& position, int& sdkCode) const;
    bool getCurrentJoints(std::array<double, 6>& joints, int& sdkCode) const;
    bool checkReachable(
        const std::array<double, 6>& position,
        bool& reachable,
        int& sdkCode
    ) const;
    bool checkLinearPath(
        const std::array<double, 6>& start,
        const std::array<double, 6>& end,
        bool& reachable,
        int& sdkCode
    ) const;
    std::vector<uint64_t> getAlarmCodes(int& sdkCode) const;

    MotionResult moveToAxis(const double joint[6], bool wait = true);
    MotionResult moveToPosition(const double position[6], bool wait = true);
    MotionResult moveLinearTo(const double position[6], bool wait = true);
    [[nodiscard]] static RobotAdapterResult validateRealHardwareConfiguration(
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] static RobotAdapterResult validateRealExecutionConfiguration(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult establishSafeOutputsOff(
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult activateConfiguredToolAndBase(
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult activateConfiguredToolAndBase(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult preflightExecution(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult checkedConfiguredJointPtp(
        const std::array<double, 6>& joints,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult checkedPtp(
        const ExecutionPlan& plan,
        const RobotPoseABC& pose,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult checkedJointPtp(
        const ExecutionPlan& plan,
        const std::array<double, 6>& joints,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult checkedLin(
        const ExecutionPlan& plan,
        const RobotPoseABC& start,
        const RobotPoseABC& end,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult checkVerticalSafeLift(
        const ExecutionPlan& plan,
        const RobotPoseABC& actual,
        const RobotPoseABC& target,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult checkedVerticalSafeLift(
        const ExecutionPlan& plan,
        const RobotPoseABC& actual,
        const RobotPoseABC& target,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotPoseAdapterResult readActualPose(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult confirmStopped() const;
#ifdef BILLIARDS_P2_03_TEST_SEAM
    [[nodiscard]] RealPneumaticResult executePneumaticSequence(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
#endif
    [[nodiscard]] RealPneumaticResult pulseExtend(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RealPneumaticResult pulseRetract(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult waitDirectionChangeDelay(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] RobotAdapterResult waitMechanismCompletion(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
};
