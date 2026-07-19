#include "RobotController.h"
#include "HRSDK.h"
#include <iostream>

// HRSDK 需要的回標函數宣告
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

void RobotController::setToolNumber(int tool_num) {
    if (connected) {
        set_tool_number(id, tool_num);
    }
}

int RobotController::getMotionState() {
    if (connected) {
        return get_motion_state(id);
    }
    return -1;
}

void RobotController::moveToAxis(const double joint[6], bool wait) {
    if (!connected) return;
    ptp_axis(id, 0, const_cast<double*>(joint));
    if (wait) {
        while (get_motion_state(id) != 1) {
            Sleep(50);
        }
    }
}

void RobotController::moveToPosition(const double pos[6], bool wait) {
    if (!connected) return;
    ptp_pos(id, 0, const_cast<double*>(pos));
    if (wait) {
        while (get_motion_state(id) != 1) {
            Sleep(50);
        }
    }
}

void RobotController::moveLinearTo(const double pos[6], bool wait) {
    if (!connected) return;
    lin_pos(id, 0, 0, const_cast<double*>(pos));
    if (wait) {
        while (get_motion_state(id) != 1) {
            Sleep(50);
        }
    }
}

void RobotController::moveLinearRelative(const double rel[6], bool wait) {
    if (!connected) return;
    lin_rel_pos(id, 0, 0.0, const_cast<double*>(rel));
    if (wait) {
        while (get_motion_state(id) != 1) {
            Sleep(20);
        }
    }
}

void RobotController::setDigitalOutput(int index, bool state) {
    if (connected) {
        set_digital_output(id, index, state);
    }
}

void RobotController::firePneumatic(int index, DWORD duration_ms) {
    if (!connected) return;
    set_digital_output(id, index, true);
    Sleep(duration_ms);
    set_digital_output(id, index, false);
}
