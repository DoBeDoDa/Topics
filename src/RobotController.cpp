// 實作 HRSDK 連線、狀態檢查、警報讀取與手臂動作命令。
#include "RobotController.h"

#include "BilliardConfig.h"
#ifndef BILLIARDS_P2_03_TEST_SEAM
#include "HRSDK.h"
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {

RobotAdapterResult motionAdapterResult(
    const MotionResult& motion,
    bool unknownUnsafeLatched) noexcept
{
    if (motion.success) {
        return {RobotAdapterStatus::Success, motion.sdkCode};
    }
    int diagnosticCode = motion.sdkCode;
    if (diagnosticCode == 0 && motion.finalMotionState < 0) {
        diagnosticCode = motion.finalMotionState;
    } else if (diagnosticCode == 0 && motion.abortSdkCode != 0) {
        diagnosticCode = motion.abortSdkCode;
    }
    return {
        unknownUnsafeLatched
            ? RobotAdapterStatus::UnknownUnsafe
            : RobotAdapterStatus::SdkFailure,
        diagnosticCode};
}

}

#ifndef BILLIARDS_P2_03_TEST_SEAM
#pragma comment(lib, "HRSDK.lib")

void __stdcall arm_callback(uint16_t, uint16_t, unsigned short*, int) {}
#endif

HrSdkApi RobotController::productionApi()
{
#ifdef BILLIARDS_P2_03_TEST_SEAM
    return {};
#else
    return {
        [](const char* ip) {
            return open_connection(const_cast<char*>(ip), 1, arm_callback);
        },
        [](int robot) { close_connection(robot); },
        [](int robot) { return clear_alarm(robot); },
        [](int robot, int state) { return set_motor_state(robot, state); },
        [](int robot, int ratio) { return set_override_ratio(robot, ratio); },
        [](int robot, int tool) { return set_tool_number(robot, tool); },
        [](int robot, int base) { return set_base_number(robot, base); },
        [](int robot) { return get_tool_number(robot); },
        [](int robot) { return get_base_number(robot); },
        [](int robot, double* pose) { return get_current_position(robot, pose); },
        [](int robot, double* joints) { return get_current_joint(robot, joints); },
        [](int robot, double* pose, bool& reachable) {
            return motion_reachable(robot, pose, reachable);
        },
        [](int robot, double* start, double* end, bool& reachable) {
            return motion_check_lin(robot, start, end, reachable);
        },
        [](int robot, int mode, double* pose) {
            return ptp_pos(robot, mode, pose);
        },
        [](int robot, int mode, double smooth, double* pose) {
            return lin_pos(robot, mode, smooth, pose);
        },
        [](int robot, int mode, double* joints) {
            return ptp_axis(robot, mode, joints);
        },
        [](int robot) { return get_motion_state(robot); },
        [](int robot) { return motion_abort(robot); },
        [](int robot, int index, bool state) {
            return set_digital_output(robot, index, state);
        },
        [](int robot, int index) { return get_digital_output(robot, index); },
        [](int robot, int index) { return get_digital_input(robot, index); },
        [](int robot, int& count, std::uint64_t* alarms) {
            return get_alarm_code(robot, count, alarms);
        },
        [] { return GetTickCount(); },
        [](unsigned long durationMs) { Sleep(durationMs); },
        [](int robot) { return get_motor_state(robot); }};
#endif
}

RobotController::RobotController()
    : id(-1), connected(false), unknownUnsafeLatched(false),
      api(productionApi()) {}

RobotController::RobotController(HrSdkApi injectedApi)
    : id(-1), connected(false), unknownUnsafeLatched(false),
      api(std::move(injectedApi)) {}

RobotController::~RobotController() {
    (void)disconnect();
}

bool RobotController::connect(const std::string& ip) {
    if (connected) {
        return false;
    }
    if (!api.openConnection || !api.closeConnection) {
        return false;
    }
    id = api.openConnection(ip.c_str());
    if (id < 0) {
        connected = false;
        return false;
    }
    connected = true;
    lastIp = ip;
    return true;
}

bool RobotController::reconnect() {
    if (lastIp.empty()) return false;
    (void)disconnect();
    return connect(lastIp);
}

RobotAdapterResult RobotController::clearAlarm()
{
    if (!connected) return {RobotAdapterStatus::NotConnected, -1};
    if (!api.clearAlarm) return {RobotAdapterStatus::SdkFailure, -1};
    // 沒有active alarm時直接跳過clear_alarm：語意上沒有東西要清，呼叫
    // 本身是多餘的，且控制器在無alarm狀態下可能因其他前提（如馬達尚未
    // 斷電）拒絕清除（sdkCode=300，即使get_alarm_code查無alarm，容易
    // 誤判成alarm清除失敗）。查詢本身失敗（sdkCode!=0）時無法確認有無
    // alarm，照原路徑嘗試清除。
    int alarmSdkCode = -1;
    if (getAlarmCodes(alarmSdkCode).empty() && alarmSdkCode == 0) {
        return {RobotAdapterStatus::Success, 0};
    }
    // 確定有alarm要清：set_motor_state(0)回傳成功只代表指令已送出，控制
    // 器實際斷電可能有延遲；clear_alarm前先bounded poll確認馬達真的已
    // 斷電，逾時仍照常嘗試（不無限等待），讓原本的sdkCode錯誤路徑處理。
    if (api.getMotorState && api.tickCountMs && api.sleepMs) {
        const unsigned long pollStart = api.tickCountMs();
        while (api.getMotorState(id) != 0) {
            if (api.tickCountMs() - pollStart >=
                BilliardConfig::MOTOR_OFF_CONFIRMATION_TIMEOUT_MS) {
                break;
            }
            api.sleepMs(BilliardConfig::MOTION_POLL_INTERVAL_MS);
        }
    }
    const int sdkCode = api.clearAlarm(id);
    if (sdkCode != 0) {
        return {RobotAdapterStatus::SdkFailure, sdkCode};
    }
    if (api.sleepMs) api.sleepMs(200);
    return {RobotAdapterStatus::Success, 0};
}

RobotAdapterResult RobotController::disconnect() {
    RobotAdapterResult result{RobotAdapterStatus::Success, 0};
    if (connected && id >= 0) {
        if (!api.setMotorState) {
            unknownUnsafeLatched = true;
            result = {RobotAdapterStatus::UnknownUnsafe, -1};
        } else {
            const int motorOffCode = api.setMotorState(id, 0);
            if (motorOffCode != 0) {
                unknownUnsafeLatched = true;
                result = {RobotAdapterStatus::UnknownUnsafe, motorOffCode};
            }
        }
        if (api.closeConnection) api.closeConnection(id);
        connected = false;
        id = -1;
    }
    return result;
}

int RobotController::getId() const {
    return id;
}

bool RobotController::isConnected() const {
    return connected;
}

RobotAdapterResult RobotController::setMotorState(int state) {
    if (!connected) return {RobotAdapterStatus::NotConnected, -1};
    if (!api.setMotorState) return {RobotAdapterStatus::SdkFailure, -1};
    const int sdkCode = api.setMotorState(id, state);
    return {sdkCode == 0 ? RobotAdapterStatus::Success : RobotAdapterStatus::SdkFailure,
        sdkCode};
}

RobotAdapterResult RobotController::setOverrideRatio(int ratio) {
    if (!connected) return {RobotAdapterStatus::NotConnected, -1};
    if (!api.setOverrideRatio) return {RobotAdapterStatus::SdkFailure, -1};
    const int sdkCode = api.setOverrideRatio(id, ratio);
    return {sdkCode == 0 ? RobotAdapterStatus::Success : RobotAdapterStatus::SdkFailure,
        sdkCode};
}

RobotAdapterResult RobotController::setToolNumber(int toolNumber) {
    if (!connected) return {RobotAdapterStatus::NotConnected, -1};
    if (!api.setToolNumber) return {RobotAdapterStatus::SdkFailure, -1};
    const int sdkCode = api.setToolNumber(id, toolNumber);
    return {sdkCode == 0 ? RobotAdapterStatus::Success : RobotAdapterStatus::SdkFailure,
        sdkCode};
}

RobotAdapterResult RobotController::setBaseNumber(int baseNumber) {
    if (!connected) return {RobotAdapterStatus::NotConnected, -1};
    if (!api.setBaseNumber) return {RobotAdapterStatus::SdkFailure, -1};
    const int sdkCode = api.setBaseNumber(id, baseNumber);
    return {sdkCode == 0 ? RobotAdapterStatus::Success : RobotAdapterStatus::SdkFailure,
        sdkCode};
}

int RobotController::getCurrentToolNumber() const {
    return connected && api.getToolNumber ? api.getToolNumber(id) : -1;
}

int RobotController::getCurrentBaseNumber() const {
    return connected && api.getBaseNumber ? api.getBaseNumber(id) : -1;
}

bool RobotController::getCurrentPosition(
    std::array<double, 6>& position,
    int& sdkCode
) const {
    if (!connected) {
        sdkCode = -1;
        return false;
    }
    if (!api.getCurrentPosition) {
        sdkCode = -1;
        return false;
    }
    sdkCode = api.getCurrentPosition(id, position.data());
    return sdkCode == 0;
}

bool RobotController::getCurrentJoints(
    std::array<double, 6>& joints,
    int& sdkCode
) const {
    if (!connected) {
        sdkCode = -1;
        return false;
    }
    if (!api.getCurrentJoints) {
        sdkCode = -1;
        return false;
    }
    sdkCode = api.getCurrentJoints(id, joints.data());
    return sdkCode == 0;
}

bool RobotController::checkReachable(
    const std::array<double, 6>& position,
    bool& reachable,
    int& sdkCode
) const {
    reachable = false;
    if (!connected) {
        sdkCode = -1;
        return false;
    }
    std::array<double, 6> copy = position;
    if (!api.checkReachable) {
        sdkCode = -1;
        return false;
    }
    sdkCode = api.checkReachable(id, copy.data(), reachable);
    return sdkCode == 0;
}

bool RobotController::checkLinearPath(
    const std::array<double, 6>& start,
    const std::array<double, 6>& end,
    bool& reachable,
    int& sdkCode
) const {
    reachable = false;
    if (!connected) {
        sdkCode = -1;
        return false;
    }
    std::array<double, 6> startCopy = start;
    std::array<double, 6> endCopy = end;
    if (!api.checkLinearPath) {
        sdkCode = -1;
        return false;
    }
    sdkCode = api.checkLinearPath(id, startCopy.data(), endCopy.data(), reachable);
    return sdkCode == 0;
}

std::vector<uint64_t> RobotController::getAlarmCodes(int& sdkCode) const {
    std::vector<uint64_t> result;
    if (!connected) {
        sdkCode = -1;
        return result;
    }

    const int maxAlarmCount = 20;
    uint64_t alarms[maxAlarmCount] = {};
    int count = maxAlarmCount;
    if (!api.getAlarmCodes) {
        sdkCode = -1;
        return result;
    }
    sdkCode = api.getAlarmCodes(id, count, alarms);
    if (sdkCode != 0) {
        return result;
    }

    if (count < 0) {
        count = 0;
    } else if (count > maxAlarmCount) {
        count = maxAlarmCount;
    }
    result.assign(alarms, alarms + count);
    return result;
}

std::optional<bool> RobotController::readDigitalInput(int index) const
{
    if (!connected || !api.getDigitalInput || index < 0) {
        return std::nullopt;
    }
    const int state = api.getDigitalInput(id, index);
    if (state != 0 && state != 1) {
        return std::nullopt;
    }
    return state == 1;
}

RobotBoolAdapterResult RobotController::isAtConfiguredJoint(
    const std::array<double, 6>& joints,
    double toleranceDeg)
{
    if (unknownUnsafeLatched) {
        return {RobotAdapterStatus::UnknownUnsafe, -1, std::nullopt};
    }
    if (!connected) {
        return {RobotAdapterStatus::NotConnected, -1, std::nullopt};
    }
    if (!std::isfinite(toleranceDeg) || toleranceDeg < 0.0 ||
        !std::all_of(joints.begin(), joints.end(),
            [](double value) { return std::isfinite(value); })) {
        return {RobotAdapterStatus::InvalidConfiguration, -1, std::nullopt};
    }
    std::array<double, 6> current{};
    int sdkCode = -1;
    if (!getCurrentJoints(current, sdkCode) ||
        !std::all_of(current.begin(), current.end(),
            [](double value) { return std::isfinite(value); })) {
        unknownUnsafeLatched = true;
        return {RobotAdapterStatus::UnknownUnsafe, sdkCode, std::nullopt};
    }
    const bool atJoint = std::equal(
        current.begin(), current.end(), joints.begin(),
        [toleranceDeg](double actual, double target) {
            return std::fabs(actual - target) <= toleranceDeg;
        });
    return {RobotAdapterStatus::Success, sdkCode, atJoint};
}

MotionResult RobotController::waitForMotion(int sdkCode, bool wait) {
    MotionResult result;
    result.sdkCode = sdkCode;
    if (sdkCode != 0) {
        return result;
    }
    if (!wait) {
        result.success = true;
        return result;
    }

    if (!api.getMotionState || !api.tickCountMs || !api.sleepMs ||
        !api.abortMotion) {
        return result;
    }
    const unsigned long confirmationStart = api.tickCountMs();
    while (true) {
        result.finalMotionState = api.getMotionState(id);
        if (result.finalMotionState < 0) {
            result.abortSdkCode = api.abortMotion(id);
            unknownUnsafeLatched = true;
            return result;
        }
        if (result.finalMotionState != 1) {
            break;
        }
        if (api.tickCountMs() - confirmationStart >=
            BilliardConfig::MOTION_START_CONFIRMATION_TIMEOUT_MS) {
            result.timedOut = true;
            result.abortSdkCode = api.abortMotion(id);
            unknownUnsafeLatched = true;
            return result;
        }
        api.sleepMs(BilliardConfig::MOTION_POLL_INTERVAL_MS);
    }

    const unsigned long completionStart = api.tickCountMs();
    while (true) {
        result.finalMotionState = api.getMotionState(id);
        if (result.finalMotionState < 0) {
            result.abortSdkCode = api.abortMotion(id);
            unknownUnsafeLatched = true;
            return result;
        }
        if (result.finalMotionState == 1) {
            result.success = true;
            return result;
        }
        if (api.tickCountMs() - completionStart >=
            BilliardConfig::MOTION_TIMEOUT_MS) {
            result.timedOut = true;
            result.abortSdkCode = api.abortMotion(id);
            unknownUnsafeLatched = true;
            return result;
        }
        api.sleepMs(BilliardConfig::MOTION_POLL_INTERVAL_MS);
    }
}

MotionResult RobotController::moveToAxis(const double joint[6], bool wait) {
    if (!connected || unknownUnsafeLatched) return MotionResult();
    double copy[6];
    std::copy(joint, joint + 6, copy);
    if (!api.movePtpAxis) return MotionResult();
    return waitForMotion(api.movePtpAxis(id, 0, copy), wait);
}

MotionResult RobotController::moveToPosition(const double position[6], bool wait) {
    if (!connected || unknownUnsafeLatched) return MotionResult();
    double copy[6];
    std::copy(position, position + 6, copy);
    if (!api.movePtpPosition) return MotionResult();
    return waitForMotion(api.movePtpPosition(id, 0, copy), wait);
}

MotionResult RobotController::moveLinearTo(const double position[6], bool wait) {
    if (!connected || unknownUnsafeLatched) return MotionResult();
    double copy[6];
    std::copy(position, position + 6, copy);
    if (!api.moveLinearPosition) return MotionResult();
    return waitForMotion(api.moveLinearPosition(id, 0, 0, copy), wait);
}

RobotAdapterResult RobotController::validateRealHardwareConfiguration(
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (!config || !config->authorizationRevision ||
        !config->base0CalibrationRevision ||
        !config->tool1ControllerCalibrationRevision ||
        !config->angleMapping ||
        !config->safeUpCalibrationRevision ||
        !config->requiredTool1CalibrationRevision ||
        !config->requiredAbcMappingRevision ||
        !config->requiredSafeUpCalibrationRevision ||
        !config->base0PositiveZSafeConfirmed || !config->extendDoIndex ||
        !config->retractDoIndex || !config->approvedTimingProfile) {
        return {RobotAdapterStatus::ConfigurationMissing, -1};
    }
    if (!config->realHardwareExecutionEnabled ||
        !*config->base0PositiveZSafeConfirmed) {
        return {RobotAdapterStatus::Unauthorized, -1};
    }
    const auto& mapping = *config->angleMapping;
    const auto& timing = *config->approvedTimingProfile;
    const bool mappingSourcesKnown =
        std::all_of(mapping.rxRyRzSources.begin(), mapping.rxRyRzSources.end(),
            [](BilliardConfig::RobotAngleComponent source) {
                return source == BilliardConfig::RobotAngleComponent::A ||
                    source == BilliardConfig::RobotAngleComponent::B ||
                    source == BilliardConfig::RobotAngleComponent::C;
            });
    const bool mappingSourcesArePermutation =
        mapping.rxRyRzSources[0] != mapping.rxRyRzSources[1] &&
        mapping.rxRyRzSources[0] != mapping.rxRyRzSources[2] &&
        mapping.rxRyRzSources[1] != mapping.rxRyRzSources[2];
    const bool mappingValuesValid =
        std::all_of(mapping.scales.begin(), mapping.scales.end(),
            [](double value) { return std::isfinite(value) && value != 0.0; }) &&
        std::all_of(mapping.offsetsDeg.begin(), mapping.offsetsDeg.end(),
            [](double value) { return std::isfinite(value); });
    if (config->authorizationRevision->empty() ||
        config->base0CalibrationRevision->empty() ||
        config->tool1ControllerCalibrationRevision->empty() ||
        config->safeUpCalibrationRevision->empty() ||
        config->requiredTool1CalibrationRevision->empty() ||
        config->requiredAbcMappingRevision->empty() ||
        config->requiredSafeUpCalibrationRevision->empty() ||
        mapping.calibrationRevision.empty() ||
        *config->requiredTool1CalibrationRevision !=
            *config->tool1ControllerCalibrationRevision ||
        *config->requiredAbcMappingRevision != mapping.calibrationRevision ||
        *config->requiredSafeUpCalibrationRevision !=
            *config->safeUpCalibrationRevision ||
        config->baseNumber != BilliardConfig::BASE_NUMBER ||
        config->tool1Number != BilliardConfig::TOOL_NUMBER ||
        *config->extendDoIndex < 0 || *config->retractDoIndex < 0 ||
        *config->extendDoIndex == *config->retractDoIndex ||
        (config->startDigitalInputIndex && *config->startDigitalInputIndex < 0) ||
        !mappingSourcesKnown || !mappingSourcesArePermutation ||
        !mappingValuesValid ||
        timing.calibrationRevision.empty() || timing.pneumaticPulseMs == 0 ||
        timing.directionChangeDelayMs == 0 ||
        timing.mechanismCompletionWaitMs == 0) {
        return {RobotAdapterStatus::InvalidConfiguration, -1};
    }
    return {RobotAdapterStatus::Success, 0};
}

std::optional<bool> RobotController::checkPoseReachable(
    const RobotPoseABC& pose,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
    const
{
    if (unknownUnsafeLatched || !connected || !pose.isFinite() ||
        !validateRealHardwareConfiguration(config).succeeded()) {
        return std::nullopt;
    }
    const HrSdkPoseResult mapped = mapPoseToHrSdk(pose, *config->angleMapping);
    if (!mapped.isValid() || !mapped.value) return std::nullopt;
    bool reachable = false;
    int sdkCode = -1;
    if (!checkReachable(*mapped.value, reachable, sdkCode)) {
        return std::nullopt;
    }
    return reachable;
}

std::optional<bool> RobotController::checkLinearPathAccepted(
    const RobotPoseABC& approach,
    const RobotPoseABC& ready,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
    const
{
    if (unknownUnsafeLatched || !connected || !approach.isFinite() ||
        !ready.isFinite() ||
        !validateRealHardwareConfiguration(config).succeeded()) {
        return std::nullopt;
    }
    const HrSdkPoseResult mappedApproach =
        mapPoseToHrSdk(approach, *config->angleMapping);
    const HrSdkPoseResult mappedReady =
        mapPoseToHrSdk(ready, *config->angleMapping);
    if (!mappedApproach.isValid() || !mappedApproach.value ||
        !mappedReady.isValid() || !mappedReady.value) {
        return std::nullopt;
    }
    bool reachable = false;
    int sdkCode = -1;
    if (!checkLinearPath(
            *mappedApproach.value, *mappedReady.value, reachable, sdkCode)) {
        return std::nullopt;
    }
    return reachable;
}

RobotBoolAdapterResult RobotController::confirmPneumaticOutputsOff(
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (unknownUnsafeLatched) {
        return {RobotAdapterStatus::UnknownUnsafe, -1, std::nullopt};
    }
    const RobotAdapterResult validation = validateRealHardwareConfiguration(config);
    if (!validation.succeeded()) {
        return {validation.status, validation.sdkCode, std::nullopt};
    }
    if (!connected) {
        return {RobotAdapterStatus::NotConnected, -1, std::nullopt};
    }
    if (!api.getDigitalOutput) {
        unknownUnsafeLatched = true;
        return {RobotAdapterStatus::UnknownUnsafe, -1, std::nullopt};
    }
    int sdkCode = -1;
    const bool off = outputsElectricallyOff(*config, sdkCode);
    if (sdkCode != 0) {
        unknownUnsafeLatched = true;
        return {RobotAdapterStatus::UnknownUnsafe, sdkCode, std::nullopt};
    }
    return {RobotAdapterStatus::Success, sdkCode, off};
}

RobotAdapterResult RobotController::establishSafeOutputsOff(
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (unknownUnsafeLatched) {
        return {RobotAdapterStatus::UnknownUnsafe, -1};
    }
    const RobotAdapterResult validation = validateRealHardwareConfiguration(config);
    if (!validation.succeeded()) return validation;
    if (!connected) return {RobotAdapterStatus::NotConnected, -1};
    if (!api.setDigitalOutput || !api.getDigitalOutput) {
        unknownUnsafeLatched = true;
        return {RobotAdapterStatus::UnknownUnsafe, -1};
    }
    const int strikeOff =
        api.setDigitalOutput(id, *config->extendDoIndex, false);
    if (strikeOff != 0) {
        unknownUnsafeLatched = true;
        return {RobotAdapterStatus::UnknownUnsafe, strikeOff};
    }
    const int strikeState = api.getDigitalOutput(id, *config->extendDoIndex);
    if (strikeState != 0) {
        unknownUnsafeLatched = true;
        return {RobotAdapterStatus::UnknownUnsafe, strikeState};
    }
    const int retractOff =
        api.setDigitalOutput(id, *config->retractDoIndex, false);
    if (retractOff != 0) {
        unknownUnsafeLatched = true;
        return {RobotAdapterStatus::UnknownUnsafe, retractOff};
    }
    const int retractState = api.getDigitalOutput(id, *config->retractDoIndex);
    if (retractState != 0) {
        unknownUnsafeLatched = true;
        return {RobotAdapterStatus::UnknownUnsafe, retractState};
    }
    return {RobotAdapterStatus::Success, 0};
}

RobotAdapterResult RobotController::validateRealExecutionConfiguration(
    const ExecutionPlan& plan,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    const RobotAdapterResult configuration =
        validateRealHardwareConfiguration(config);
    if (!configuration.succeeded()) return configuration;
    const bool acceptedPolicy =
        plan.policyDecision == ExecutionPolicyDecision::PotAccepted ||
        plan.policyDecision == ExecutionPolicyDecision::
            LegalContactProductionFallbackAccepted;
    if (plan.policyMode != BilliardConfig::ExecutionPolicyMode::RealHardware ||
        !acceptedPolicy) {
        return {RobotAdapterStatus::Unauthorized, -1};
    }
    const auto& timing = *config->approvedTimingProfile;
    const bool timingMatches =
        timing.calibrationRevision == plan.pneumaticTimingProfile.calibrationRevision &&
        timing.pneumaticPulseMs == plan.pneumaticTimingProfile.pneumaticPulseMs &&
        timing.directionChangeDelayMs ==
            plan.pneumaticTimingProfile.directionChangeDelayMs &&
        timing.mechanismCompletionWaitMs ==
            plan.pneumaticTimingProfile.mechanismCompletionWaitMs;
    if (!plan.isValid() ||
        *config->authorizationRevision != plan.executionPolicyRevision ||
        *config->base0CalibrationRevision != plan.base0PlanarCalibrationRevision ||
        plan.tool1Number != config->tool1Number ||
        plan.tool1ControllerCalibrationRevision !=
            *config->requiredTool1CalibrationRevision ||
        !timingMatches) {
        return {RobotAdapterStatus::InvalidConfiguration, -1};
    }
    return {RobotAdapterStatus::Success, 0};
}

RobotAdapterResult RobotController::validateRealExecution(
    const ExecutionPlan& plan,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
    const
{
    const RobotAdapterResult configuration =
        validateRealExecutionConfiguration(plan, config);
    if (!configuration.succeeded()) return configuration;
    if (!connected) {
        return {RobotAdapterStatus::NotConnected, -1};
    }
    return {RobotAdapterStatus::Success, 0};
}

HrSdkPoseResult RobotController::mapPoseToHrSdk(
    const RobotPoseABC& pose,
    const BilliardConfig::HrSdkAngleMappingConfig& mapping) const
{
    if (!pose.isFinite() || mapping.calibrationRevision.empty()) {
        return {RobotAdapterStatus::InvalidConfiguration, std::nullopt};
    }
    const std::array<double, 3> abc{pose.a, pose.b, pose.c};
    std::array<double, 6> mapped{pose.x, pose.y, pose.z, 0.0, 0.0, 0.0};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const std::size_t source =
            static_cast<std::size_t>(mapping.rxRyRzSources[axis]);
        if (source >= abc.size() || !std::isfinite(mapping.scales[axis]) ||
            mapping.scales[axis] == 0.0 ||
            !std::isfinite(mapping.offsetsDeg[axis])) {
            return {RobotAdapterStatus::InvalidConfiguration, std::nullopt};
        }
        mapped[axis + 3] =
            abc[source] * mapping.scales[axis] + mapping.offsetsDeg[axis];
    }
    if (!std::all_of(mapped.begin(), mapped.end(),
            [](double value) { return std::isfinite(value); })) {
        return {RobotAdapterStatus::InvalidConfiguration, std::nullopt};
    }
    return {RobotAdapterStatus::Success, mapped};
}

RobotPoseAdapterResult RobotController::mapPoseFromHrSdk(
    const std::array<double, 6>& pose,
    const BilliardConfig::HrSdkAngleMappingConfig& mapping) const
{
    if (!std::all_of(pose.begin(), pose.end(),
            [](double value) { return std::isfinite(value); })) {
        return {RobotAdapterStatus::InvalidConfiguration, -1, std::nullopt};
    }
    std::array<double, 3> abc{};
    std::array<bool, 3> assigned{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const std::size_t target =
            static_cast<std::size_t>(mapping.rxRyRzSources[axis]);
        if (target >= abc.size() || assigned[target] ||
            !std::isfinite(mapping.scales[axis]) ||
            mapping.scales[axis] == 0.0 ||
            !std::isfinite(mapping.offsetsDeg[axis])) {
            return {RobotAdapterStatus::InvalidConfiguration, -1, std::nullopt};
        }
        abc[target] =
            (pose[axis + 3] - mapping.offsetsDeg[axis]) /
            mapping.scales[axis];
        assigned[target] = true;
    }
    const RobotPoseABC result{pose[0], pose[1], pose[2], abc[0], abc[1], abc[2]};
    if (!result.isFinite()) {
        return {RobotAdapterStatus::InvalidConfiguration, -1, std::nullopt};
    }
    return {RobotAdapterStatus::Success, 0, result};
}

RobotAdapterResult RobotController::activateConfiguredToolAndBase(
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (unknownUnsafeLatched) {
        return {RobotAdapterStatus::UnknownUnsafe, -1};
    }
    const RobotAdapterResult validation = validateRealHardwareConfiguration(config);
    if (!validation.succeeded()) return validation;
    if (!connected) return {RobotAdapterStatus::NotConnected, -1};
    if (!api.setToolNumber || !api.setBaseNumber || !api.getToolNumber ||
        !api.getBaseNumber) {
        return {RobotAdapterStatus::SdkFailure, -1};
    }
    int sdkCode = api.setToolNumber(id, config->tool1Number);
    if (sdkCode != 0) return {RobotAdapterStatus::SdkFailure, sdkCode};
    sdkCode = api.setBaseNumber(id, config->baseNumber);
    if (sdkCode != 0) return {RobotAdapterStatus::SdkFailure, sdkCode};
    const int actualTool = api.getToolNumber(id);
    const int actualBase = api.getBaseNumber(id);
    if (actualTool < 0 || actualBase < 0) {
        return {RobotAdapterStatus::SdkFailure,
            actualTool < 0 ? actualTool : actualBase};
    }
    if (actualTool != config->tool1Number ||
        actualBase != config->baseNumber) {
        return {RobotAdapterStatus::InvalidConfiguration, -1};
    }
    return {RobotAdapterStatus::Success, 0};
}

RobotAdapterResult RobotController::preflightExecution(
    const ExecutionPlan& plan,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (unknownUnsafeLatched) {
        return {RobotAdapterStatus::UnknownUnsafe, -1};
    }
    const RobotAdapterResult validation = validateRealExecution(plan, config);
    if (!validation.succeeded()) return validation;
    const RobotAdapterResult frame = activateConfiguredToolAndBase(plan, config);
    if (!frame.succeeded()) return frame;

    const HrSdkPoseResult approach =
        mapPoseToHrSdk(plan.safeApproachPose, *config->angleMapping);
    const HrSdkPoseResult ready =
        mapPoseToHrSdk(plan.strikeReadyPose, *config->angleMapping);
    if (!approach.isValid() || !ready.isValid() || !approach.value ||
        !ready.value) {
        return {RobotAdapterStatus::InvalidConfiguration, -1};
    }

    bool reachable = false;
    int sdkCode = -1;
    if (!checkReachable(*approach.value, reachable, sdkCode)) {
        return {RobotAdapterStatus::SdkFailure, sdkCode};
    }
    if (!reachable) return {RobotAdapterStatus::NotReachable, sdkCode};
    if (!checkReachable(*ready.value, reachable, sdkCode)) {
        return {RobotAdapterStatus::SdkFailure, sdkCode};
    }
    if (!reachable) return {RobotAdapterStatus::NotReachable, sdkCode};
    if (!checkLinearPath(*approach.value, *ready.value, reachable, sdkCode)) {
        return {RobotAdapterStatus::SdkFailure, sdkCode};
    }
    return reachable
        ? RobotAdapterResult{RobotAdapterStatus::Success, sdkCode}
        : RobotAdapterResult{RobotAdapterStatus::NotReachable, sdkCode};
}

RobotAdapterResult RobotController::activateConfiguredToolAndBase(
    const ExecutionPlan& plan,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    const RobotAdapterResult validation = validateRealExecution(plan, config);
    if (!validation.succeeded()) return validation;
    if (!api.setToolNumber || !api.setBaseNumber || !api.getToolNumber ||
        !api.getBaseNumber) {
        return {RobotAdapterStatus::SdkFailure, -1};
    }
    int sdkCode = api.setToolNumber(id, plan.tool1Number);
    if (sdkCode != 0) return {RobotAdapterStatus::SdkFailure, sdkCode};
    sdkCode = api.setBaseNumber(id, config->baseNumber);
    if (sdkCode != 0) return {RobotAdapterStatus::SdkFailure, sdkCode};
    const int actualTool = api.getToolNumber(id);
    const int actualBase = api.getBaseNumber(id);
    if (actualTool < 0 || actualBase < 0) {
        return {RobotAdapterStatus::SdkFailure,
            actualTool < 0 ? actualTool : actualBase};
    }
    return actualTool == plan.tool1Number &&
            actualBase == config->baseNumber
        ? RobotAdapterResult{RobotAdapterStatus::Success, 0}
        : RobotAdapterResult{RobotAdapterStatus::InvalidConfiguration, -1};
}

RobotAdapterResult RobotController::checkedPtp(
    const ExecutionPlan& plan,
    const RobotPoseABC& pose,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (unknownUnsafeLatched) {
        return {RobotAdapterStatus::UnknownUnsafe, -1};
    }
    const RobotAdapterResult validation = validateRealExecution(plan, config);
    if (!validation.succeeded()) return validation;
    const RobotAdapterResult frame = activateConfiguredToolAndBase(plan, config);
    if (!frame.succeeded()) return frame;
    const HrSdkPoseResult mapped = mapPoseToHrSdk(pose, *config->angleMapping);
    if (!mapped.isValid() || !mapped.value) {
        return {RobotAdapterStatus::InvalidConfiguration, -1};
    }
    bool reachable = false;
    int sdkCode = -1;
    if (!checkReachable(*mapped.value, reachable, sdkCode)) {
        return {RobotAdapterStatus::SdkFailure, sdkCode};
    }
    if (!reachable) return {RobotAdapterStatus::NotReachable, sdkCode};
    const MotionResult motion = moveToPosition(mapped.value->data(), true);
    return motionAdapterResult(motion, unknownUnsafeLatched);
}

RobotAdapterResult RobotController::checkedConfiguredJointPtp(
    const std::array<double, 6>& joints,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (unknownUnsafeLatched) {
        return {RobotAdapterStatus::UnknownUnsafe, -1};
    }
    const RobotAdapterResult validation = validateRealHardwareConfiguration(config);
    if (!validation.succeeded()) return validation;
    if (!connected) return {RobotAdapterStatus::NotConnected, -1};
    if (!std::all_of(joints.begin(), joints.end(),
            [](double value) { return std::isfinite(value); })) {
        return {RobotAdapterStatus::InvalidConfiguration, -1};
    }
    const RobotAdapterResult frame = activateConfiguredToolAndBase(config);
    if (!frame.succeeded()) return frame;
    if (joints == BilliardConfig::CAMERA_JOINT) {
        RobotAdapterResult stopped = confirmStopped();
        if (stopped.status == RobotAdapterStatus::NotStopped) {
            if (!api.tickCountMs || !api.sleepMs) {
                return {RobotAdapterStatus::SdkFailure, -1};
            }
            const unsigned long waitStart = api.tickCountMs();
            while (stopped.status == RobotAdapterStatus::NotStopped) {
                if (api.tickCountMs() - waitStart >=
                    BilliardConfig::MOTION_TIMEOUT_MS) {
                    return stopped;
                }
                api.sleepMs(BilliardConfig::MOTION_POLL_INTERVAL_MS);
                stopped = confirmStopped();
            }
        }
        if (!stopped.succeeded()) return stopped;

        const RobotBoolAdapterResult alreadyAtCamera = isAtConfiguredJoint(
            joints, BilliardConfig::CAMERA_JOINT_TOLERANCE_DEG);
        if (alreadyAtCamera.status != RobotAdapterStatus::Success) {
            return {alreadyAtCamera.status, alreadyAtCamera.sdkCode};
        }
        if (*alreadyAtCamera.value) {
            return {RobotAdapterStatus::Success, alreadyAtCamera.sdkCode};
        }
    }
    const MotionResult motion = moveToAxis(joints.data(), true);
    return motionAdapterResult(motion, unknownUnsafeLatched);
}

RobotAdapterResult RobotController::checkedJointPtp(
    const ExecutionPlan& plan,
    const std::array<double, 6>& joints,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (unknownUnsafeLatched) {
        return {RobotAdapterStatus::UnknownUnsafe, -1};
    }
    const RobotAdapterResult validation = validateRealExecution(plan, config);
    if (!validation.succeeded()) return validation;
    const RobotAdapterResult frame = activateConfiguredToolAndBase(plan, config);
    if (!frame.succeeded()) return frame;
    if (!std::all_of(joints.begin(), joints.end(),
            [](double value) { return std::isfinite(value); })) {
        return {RobotAdapterStatus::InvalidConfiguration, -1};
    }
    const MotionResult motion = moveToAxis(joints.data(), true);
    return motionAdapterResult(motion, unknownUnsafeLatched);
}

RobotAdapterResult RobotController::checkedLin(
    const ExecutionPlan& plan,
    const RobotPoseABC& start,
    const RobotPoseABC& end,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (unknownUnsafeLatched) {
        return {RobotAdapterStatus::UnknownUnsafe, -1};
    }
    const RobotAdapterResult validation = validateRealExecution(plan, config);
    if (!validation.succeeded()) return validation;
    const RobotAdapterResult frame = activateConfiguredToolAndBase(plan, config);
    if (!frame.succeeded()) return frame;
    const HrSdkPoseResult mappedStart =
        mapPoseToHrSdk(start, *config->angleMapping);
    const HrSdkPoseResult mappedEnd =
        mapPoseToHrSdk(end, *config->angleMapping);
    if (!mappedStart.isValid() || !mappedEnd.isValid() || !mappedStart.value ||
        !mappedEnd.value) {
        return {RobotAdapterStatus::InvalidConfiguration, -1};
    }
    bool reachable = false;
    int sdkCode = -1;
    if (!checkLinearPath(*mappedStart.value, *mappedEnd.value, reachable, sdkCode)) {
        return {RobotAdapterStatus::SdkFailure, sdkCode};
    }
    if (!reachable) return {RobotAdapterStatus::NotReachable, sdkCode};
    const MotionResult motion = moveLinearTo(mappedEnd.value->data(), true);
    return motionAdapterResult(motion, unknownUnsafeLatched);
}

RobotAdapterResult RobotController::checkVerticalSafeLift(
    const ExecutionPlan& plan,
    const RobotPoseABC& actual,
    const RobotPoseABC& target,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (unknownUnsafeLatched) {
        return {RobotAdapterStatus::UnknownUnsafe, -1};
    }
    const RobotAdapterResult validation = validateRealExecution(plan, config);
    if (!validation.succeeded()) return validation;
    if (!isValidSafeLiftTarget(actual, plan.safeApproachPose.z, target)) {
        return {RobotAdapterStatus::InvalidConfiguration, -1};
    }
    const RobotAdapterResult frame = activateConfiguredToolAndBase(plan, config);
    if (!frame.succeeded()) return frame;
    const HrSdkPoseResult mappedStart =
        mapPoseToHrSdk(actual, *config->angleMapping);
    const HrSdkPoseResult mappedEnd =
        mapPoseToHrSdk(target, *config->angleMapping);
    if (!mappedStart.isValid() || !mappedEnd.isValid() || !mappedStart.value ||
        !mappedEnd.value) {
        return {RobotAdapterStatus::InvalidConfiguration, -1};
    }
    bool reachable = false;
    int sdkCode = -1;
    if (!checkLinearPath(*mappedStart.value, *mappedEnd.value, reachable, sdkCode)) {
        return {RobotAdapterStatus::SdkFailure, sdkCode};
    }
    return reachable
        ? RobotAdapterResult{RobotAdapterStatus::Success, sdkCode}
        : RobotAdapterResult{RobotAdapterStatus::NotReachable, sdkCode};
}

RobotAdapterResult RobotController::checkedVerticalSafeLift(
    const ExecutionPlan& plan,
    const RobotPoseABC& actual,
    const RobotPoseABC& target,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    const RobotAdapterResult checked =
        checkVerticalSafeLift(plan, actual, target, config);
    if (!checked.succeeded()) return checked;
    const HrSdkPoseResult mappedEnd =
        mapPoseToHrSdk(target, *config->angleMapping);
    if (!mappedEnd.isValid() || !mappedEnd.value) {
        return {RobotAdapterStatus::InvalidConfiguration, -1};
    }
    const MotionResult motion = moveLinearTo(mappedEnd.value->data(), true);
    return motionAdapterResult(motion, unknownUnsafeLatched);
}

RobotPoseAdapterResult RobotController::readActualPose(
    const ExecutionPlan& plan,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    const RobotAdapterResult validation = validateRealExecution(plan, config);
    if (!validation.succeeded()) {
        return {validation.status, validation.sdkCode, std::nullopt};
    }
    const RobotAdapterResult frame = activateConfiguredToolAndBase(plan, config);
    if (!frame.succeeded()) {
        return {frame.status, frame.sdkCode, std::nullopt};
    }
    std::array<double, 6> raw{};
    int sdkCode = -1;
    if (!getCurrentPosition(raw, sdkCode)) {
        return {RobotAdapterStatus::SdkFailure, sdkCode, std::nullopt};
    }
    RobotPoseAdapterResult result = mapPoseFromHrSdk(raw, *config->angleMapping);
    result.sdkCode = sdkCode;
    return result;
}

RobotAdapterResult RobotController::confirmStopped()
{
    if (unknownUnsafeLatched) {
        return {RobotAdapterStatus::UnknownUnsafe, -1};
    }
    if (!connected) return {RobotAdapterStatus::NotConnected, -1};
    if (!api.getMotionState) {
        unknownUnsafeLatched = true;
        return {RobotAdapterStatus::UnknownUnsafe, -1};
    }
    const int state = api.getMotionState(id);
    if (state < 0) {
        unknownUnsafeLatched = true;
        return {RobotAdapterStatus::UnknownUnsafe, state};
    }
    return state == 1
        ? RobotAdapterResult{RobotAdapterStatus::Success, 0}
        : RobotAdapterResult{RobotAdapterStatus::NotStopped, state};
}

bool RobotController::outputsElectricallyOff(
    const BilliardConfig::RealHardwareExecutionConfig& config,
    int& sdkCode) const
{
    if (!connected || !api.getDigitalOutput || !config.extendDoIndex ||
        !config.retractDoIndex) {
        sdkCode = -1;
        return false;
    }
    const int strikeState = api.getDigitalOutput(id, *config.extendDoIndex);
    if (strikeState < 0) {
        sdkCode = strikeState;
        return false;
    }
    const int retractState = api.getDigitalOutput(id, *config.retractDoIndex);
    if (retractState < 0) {
        sdkCode = retractState;
        return false;
    }
    sdkCode = 0;
    return strikeState == 0 && retractState == 0;
}

bool RobotController::bestEffortOutputsOff(
    const BilliardConfig::RealHardwareExecutionConfig& config,
    int& sdkCode)
{
    if (!connected || !api.setDigitalOutput || !config.extendDoIndex ||
        !config.retractDoIndex) {
        sdkCode = -1;
        return false;
    }
    const int strikeOff = api.setDigitalOutput(id, *config.extendDoIndex, false);
    const int retractOff = api.setDigitalOutput(id, *config.retractDoIndex, false);
    if (strikeOff != 0 || retractOff != 0) {
        sdkCode = strikeOff != 0 ? strikeOff : retractOff;
        return false;
    }
    return outputsElectricallyOff(config, sdkCode);
}

RealPneumaticResult RobotController::latchUnknownUnsafe(int sdkCode) noexcept
{
    unknownUnsafeLatched = true;
    return {RealPneumaticStatus::UnknownUnsafe, std::nullopt, sdkCode};
}

RealPneumaticResult RobotController::pulseOutput(
    const ExecutionPlan& plan,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config,
    int activeOutput,
    int mutuallyExclusiveOutput)
{
    if (unknownUnsafeLatched) {
        return {RealPneumaticStatus::UnknownUnsafe, std::nullopt, -1};
    }
    const RobotAdapterResult validation = validateRealExecution(plan, config);
    if (!validation.succeeded()) {
        return validation.status == RobotAdapterStatus::SdkFailure ||
                validation.status == RobotAdapterStatus::NotConnected
            ? latchUnknownUnsafe(validation.sdkCode)
            : RealPneumaticResult{RealPneumaticStatus::KnownSafeFailure,
                std::nullopt, validation.sdkCode};
    }
    if (!api.setDigitalOutput || !api.getDigitalOutput || !api.sleepMs) {
        return latchUnknownUnsafe(-1);
    }
    const RobotAdapterResult stopped = confirmStopped();
    if (!stopped.succeeded()) {
        if (stopped.status == RobotAdapterStatus::UnknownUnsafe) {
            return {RealPneumaticStatus::UnknownUnsafe, std::nullopt,
                stopped.sdkCode};
        }
        return {RealPneumaticStatus::KnownSafeFailure, std::nullopt,
            stopped.sdkCode};
    }

    int sdkCode = api.setDigitalOutput(id, mutuallyExclusiveOutput, false);
    if (sdkCode != 0 || api.getDigitalOutput(id, mutuallyExclusiveOutput) != 0) {
        return latchUnknownUnsafe(sdkCode);
    }
    sdkCode = api.setDigitalOutput(id, activeOutput, true);
    if (sdkCode != 0) {
        int offCode = -1;
        return bestEffortOutputsOff(*config, offCode)
            ? RealPneumaticResult{RealPneumaticStatus::KnownSafeFailure,
                std::nullopt, sdkCode}
            : latchUnknownUnsafe(offCode);
    }
    const int activeState = api.getDigitalOutput(id, activeOutput);
    const int otherState = api.getDigitalOutput(id, mutuallyExclusiveOutput);
    if (activeState != 1 || otherState != 0) {
        int offCode = -1;
        (void)bestEffortOutputsOff(*config, offCode);
        return latchUnknownUnsafe(activeState < 0 ? activeState : otherState);
    }
    api.sleepMs(config->approvedTimingProfile->pneumaticPulseMs);
    sdkCode = api.setDigitalOutput(id, activeOutput, false);
    if (sdkCode != 0) {
        int ignored = -1;
        (void)bestEffortOutputsOff(*config, ignored);
        return latchUnknownUnsafe(sdkCode);
    }
    if (!outputsElectricallyOff(*config, sdkCode)) {
        int ignored = -1;
        (void)bestEffortOutputsOff(*config, ignored);
        return latchUnknownUnsafe(sdkCode);
    }
    return {RealPneumaticStatus::Completed,
        RealPneumaticEvidence::OutputOffConfirmed, 0};
}

RealPneumaticResult RobotController::pulseExtend(
    const ExecutionPlan& plan,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (!config || !config->extendDoIndex || !config->retractDoIndex) {
        return {RealPneumaticStatus::KnownSafeFailure, std::nullopt, -1};
    }
    return pulseOutput(plan, config, *config->extendDoIndex,
        *config->retractDoIndex);
}

RealPneumaticResult RobotController::pulseRetract(
    const ExecutionPlan& plan,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (!config || !config->extendDoIndex || !config->retractDoIndex) {
        return {RealPneumaticStatus::KnownSafeFailure, std::nullopt, -1};
    }
    return pulseOutput(plan, config, *config->retractDoIndex,
        *config->extendDoIndex);
}

RobotAdapterResult RobotController::waitDirectionChangeDelay(
    const ExecutionPlan& plan,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    const RobotAdapterResult validation = validateRealExecution(plan, config);
    if (!validation.succeeded()) return validation;
    if (!api.sleepMs) return {RobotAdapterStatus::SdkFailure, -1};
    api.sleepMs(config->approvedTimingProfile->directionChangeDelayMs);
    return {RobotAdapterStatus::Success, 0};
}

RobotAdapterResult RobotController::waitMechanismCompletion(
    const ExecutionPlan& plan,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    const RobotAdapterResult validation = validateRealExecution(plan, config);
    if (!validation.succeeded()) return validation;
    if (!api.sleepMs) return {RobotAdapterStatus::SdkFailure, -1};
    api.sleepMs(config->approvedTimingProfile->mechanismCompletionWaitMs);
    return {RobotAdapterStatus::Success, 0};
}

#ifdef BILLIARDS_P2_03_TEST_SEAM
RealPneumaticResult RobotController::executePneumaticSequence(
    const ExecutionPlan& plan,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (unknownUnsafeLatched) {
        return {RealPneumaticStatus::UnknownUnsafe, std::nullopt, -1};
    }
    const RobotAdapterResult validation = validateRealExecution(plan, config);
    if (!validation.succeeded()) {
        if (validation.status == RobotAdapterStatus::NotConnected ||
            validation.status == RobotAdapterStatus::SdkFailure) {
            return latchUnknownUnsafe(validation.sdkCode);
        }
        return {RealPneumaticStatus::KnownSafeFailure, std::nullopt,
            validation.sdkCode};
    }
    if (!api.setDigitalOutput || !api.getDigitalOutput || !api.sleepMs) {
        return latchUnknownUnsafe(-1);
    }
    int sdkCode = -1;
    if (!bestEffortOutputsOff(*config, sdkCode)) {
        return latchUnknownUnsafe(sdkCode);
    }
    const RobotAdapterResult stopped = confirmStopped();
    if (!stopped.succeeded()) {
        if (stopped.status == RobotAdapterStatus::UnknownUnsafe) {
            return {RealPneumaticStatus::UnknownUnsafe, std::nullopt,
                stopped.sdkCode};
        }
        return {RealPneumaticStatus::KnownSafeFailure, std::nullopt,
            stopped.sdkCode};
    }
    const int strikeDo = *config->extendDoIndex;
    const int retractDo = *config->retractDoIndex;
    const auto knownSafeOrUnknown = [&]() {
        int offCode = -1;
        if (bestEffortOutputsOff(*config, offCode)) {
            return RealPneumaticResult{RealPneumaticStatus::KnownSafeFailure,
                std::nullopt, sdkCode};
        }
        return latchUnknownUnsafe(offCode);
    };

    sdkCode = api.setDigitalOutput(id, strikeDo, true);
    if (sdkCode != 0) return knownSafeOrUnknown();
    const int strikeOn = api.getDigitalOutput(id, strikeDo);
    const int retractOff = api.getDigitalOutput(id, retractDo);
    if (strikeOn != 1 || retractOff != 0) {
        sdkCode = strikeOn < 0 ? strikeOn : retractOff;
        int ignoredOffCode = -1;
        const bool offAttempted = bestEffortOutputsOff(*config, ignoredOffCode);
        (void)offAttempted;
        return latchUnknownUnsafe(sdkCode);
    }
    api.sleepMs(config->approvedTimingProfile->pneumaticPulseMs);
    sdkCode = api.setDigitalOutput(id, strikeDo, false);
    if (sdkCode != 0) {
        const int originalCode = sdkCode;
        int ignoredOffCode = -1;
        const bool offAttempted = bestEffortOutputsOff(*config, ignoredOffCode);
        (void)offAttempted;
        return latchUnknownUnsafe(originalCode);
    }
    if (!outputsElectricallyOff(*config, sdkCode)) {
        int ignoredOffCode = -1;
        const bool offAttempted = bestEffortOutputsOff(*config, ignoredOffCode);
        (void)offAttempted;
        return latchUnknownUnsafe(sdkCode);
    }
    api.sleepMs(config->approvedTimingProfile->directionChangeDelayMs);

    sdkCode = api.setDigitalOutput(id, retractDo, true);
    if (sdkCode != 0) return knownSafeOrUnknown();
    const int strikeOff = api.getDigitalOutput(id, strikeDo);
    const int retractOn = api.getDigitalOutput(id, retractDo);
    if (strikeOff != 0 || retractOn != 1) {
        sdkCode = strikeOff < 0 ? strikeOff : retractOn;
        int ignoredOffCode = -1;
        const bool offAttempted = bestEffortOutputsOff(*config, ignoredOffCode);
        (void)offAttempted;
        return latchUnknownUnsafe(sdkCode);
    }
    api.sleepMs(config->approvedTimingProfile->pneumaticPulseMs);
    sdkCode = api.setDigitalOutput(id, retractDo, false);
    if (sdkCode != 0) {
        const int originalCode = sdkCode;
        int ignoredOffCode = -1;
        const bool offAttempted = bestEffortOutputsOff(*config, ignoredOffCode);
        (void)offAttempted;
        return latchUnknownUnsafe(originalCode);
    }
    if (!outputsElectricallyOff(*config, sdkCode)) {
        int ignoredOffCode = -1;
        const bool offAttempted = bestEffortOutputsOff(*config, ignoredOffCode);
        (void)offAttempted;
        return latchUnknownUnsafe(sdkCode);
    }
    api.sleepMs(config->approvedTimingProfile->mechanismCompletionWaitMs);
    if (!outputsElectricallyOff(*config, sdkCode)) {
        return latchUnknownUnsafe(sdkCode);
    }
    return {RealPneumaticStatus::Completed,
        RealPneumaticEvidence::OutputOffConfirmed, 0};
}
#endif
