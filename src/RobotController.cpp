// 實作 HRSDK 連線、狀態檢查、警報讀取與手臂動作命令。
#include "RobotController.h"

#include "BilliardConfig.h"
#include "HRSDK.h"

void __stdcall arm_callback(uint16_t, uint16_t, unsigned short*, int) {}

RobotController::RobotController() : id(-1), connected(false) {}

RobotController::~RobotController() {
    disconnect();
}

bool RobotController::connect(const std::string& ip) {
    id = open_connection(const_cast<char*>(ip.c_str()), 1, arm_callback);
    if (id < 0) {
        connected = false;
        return false;
    }
    connected = true;
    clear_alarm(id);
    Sleep(200);
    return true;
}

void RobotController::disconnect() {
    if (connected && id >= 0) {
        set_motor_state(id, 0);
        close_connection(id);
        connected = false;
        id = -1;
    }
}

int RobotController::getId() const {
    return id;
}

bool RobotController::isConnected() const {
    return connected;
}

void RobotController::setMotorState(int state) {
    if (connected) {
        set_motor_state(id, state);
    }
}

void RobotController::setOverrideRatio(int ratio) {
    if (connected) {
        set_override_ratio(id, ratio);
    }
}

void RobotController::setToolNumber(int toolNumber) {
    if (connected) {
        set_tool_number(id, toolNumber);
    }
}

int RobotController::getCurrentToolNumber() const {
    return connected ? get_tool_number(id) : -1;
}

int RobotController::getCurrentBaseNumber() const {
    return connected ? get_base_number(id) : -1;
}

bool RobotController::getCurrentPosition(
    std::array<double, 6>& position,
    int& sdkCode
) const {
    if (!connected) {
        sdkCode = -1;
        return false;
    }
    sdkCode = get_current_position(id, position.data());
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
    sdkCode = get_current_joint(id, joints.data());
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
    sdkCode = motion_reachable(id, copy.data(), reachable);
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
    sdkCode = motion_check_lin(id, startCopy.data(), endCopy.data(), reachable);
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
    sdkCode = get_alarm_code(id, count, alarms);
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

    const DWORD startTime = GetTickCount();
    while (true) {
        result.finalMotionState = get_motion_state(id);
        if (result.finalMotionState == 1) {
            result.success = true;
            return result;
        }
        if (result.finalMotionState < 0) {
            return result;
        }
        if (GetTickCount() - startTime >= BilliardConfig::MOTION_TIMEOUT_MS) {
            result.timedOut = true;
            result.abortSdkCode = motion_abort(id);
            return result;
        }
        Sleep(BilliardConfig::MOTION_POLL_INTERVAL_MS);
    }
}

MotionResult RobotController::moveToAxis(const double joint[6], bool wait) {
    if (!connected) return MotionResult();
    double copy[6];
    std::copy(joint, joint + 6, copy);
    return waitForMotion(ptp_axis(id, 0, copy), wait);
}

MotionResult RobotController::moveToPosition(const double position[6], bool wait) {
    if (!connected) return MotionResult();
    double copy[6];
    std::copy(position, position + 6, copy);
    return waitForMotion(ptp_pos(id, 0, copy), wait);
}

MotionResult RobotController::moveLinearTo(const double position[6], bool wait) {
    if (!connected) return MotionResult();
    double copy[6];
    std::copy(position, position + 6, copy);
    return waitForMotion(lin_pos(id, 0, 0, copy), wait);
}

void RobotController::setDigitalOutput(int index, bool state) {
    if (connected) {
        set_digital_output(id, index, state);
    }
}

void RobotController::firePneumatic(int index, DWORD durationMs) {
    if (!connected) return;
    set_digital_output(id, index, true);
    Sleep(durationMs);
    set_digital_output(id, index, false);
}
