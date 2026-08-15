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
    std::function<int(int, int)> getDigitalInput;
    std::function<int(int, int&, std::uint64_t*)> getAlarmCodes;
    std::function<unsigned long()> tickCountMs;
    std::function<void(unsigned long)> sleepMs;
    // 可選：未注入（如既有test fake）時clearAlarm()略過馬達斷電確認，
    // 行為與加入前相同。
    std::function<int(int)> getMotorState;
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

// Read-only查詢的完整結果：重用RobotAdapterStatus，不新增語意重疊enum。
// status==Success時value必有值（true/false為確定結果）；status為其他值時
// value必為nullopt。ConfigurationMissing／Unauthorized／InvalidConfiguration／
// NotConnected為已知精確狀態，不latch；UnknownUnsafe代表SDK／readback本身
// 失敗，呼叫端必定已被latch，之後任何motion命令都會被擋下。
struct RobotBoolAdapterResult {
    RobotAdapterStatus status;
    int sdkCode;
    std::optional<bool> value;

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
    std::string lastIp;

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
    // 斷線後用上次連線的ip重新連線一次；ip未知（從未成功連線過）時回傳
    // false。只給read-only確認階段用來甩掉控制器殘留狀態，不得在motion
    // 執行中途呼叫。
    bool reconnect();
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
    // Read-only：讀單一實體DI目前的0/1狀態。nullopt代表未連線／SDK未提供
    // 讀取函式／讀值本身非0非1（讀取失敗）——這不是安全危害（頂多這一輪
    // 沒偵測到Start訊號，等同沒人按），所以不latch unknownUnsafeLatched，
    // 也不影響其他任何motion命令。
    [[nodiscard]] std::optional<bool> readDigitalInput(int index) const;
    // Read-only：只讀目前joints並比對，不下任何motion命令。
    // SDK讀取失敗時latch unknownUnsafeLatched並回傳UnknownUnsafe＋原始
    // sdkCode；未連線／輸入無效回傳對應精確狀態，不latch。
    [[nodiscard]] RobotBoolAdapterResult isAtConfiguredJoint(
        const std::array<double, 6>& joints,
        double toleranceDeg);

    MotionResult moveToAxis(const double joint[6], bool wait = true);
    MotionResult moveToPosition(const double position[6], bool wait = true);
    MotionResult moveLinearTo(const double position[6], bool wait = true);
    [[nodiscard]] static RobotAdapterResult validateRealHardwareConfiguration(
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] static RobotAdapterResult validateRealExecutionConfiguration(
        const ExecutionPlan& plan,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    [[nodiscard]] std::optional<bool> checkPoseReachable(
        const RobotPoseABC& pose,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
        const;
    [[nodiscard]] std::optional<bool> checkLinearPathAccepted(
        const RobotPoseABC& approach,
        const RobotPoseABC& ready,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
        const;
    // Read-only：只讀DO1/DO2目前電位，不下setDigitalOutput命令。
    // SDK讀取失敗時latch unknownUnsafeLatched並回傳UnknownUnsafe＋原始
    // sdkCode；未連線／設定缺失回傳對應精確狀態，不latch。讀值成功時
    // value=true代表DO1/DO2確認皆為OFF，false代表至少一個不是OFF。
    [[nodiscard]] RobotBoolAdapterResult confirmPneumaticOutputsOff(
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
    [[nodiscard]] RobotAdapterResult confirmStopped();
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
