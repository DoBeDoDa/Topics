#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>

#pragma comment(lib, "HRSDK.lib")

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

    MotionResult waitForMotion(int sdkCode, bool wait);

public:
    RobotController();
    ~RobotController();

    bool connect(const std::string& ip);
    void disconnect();
    int getId() const;
    bool isConnected() const;

    void setMotorState(int state);
    void setOverrideRatio(int ratio);
    void setToolNumber(int toolNumber);
    int getMotionState();
    int getCurrentToolNumber() const;
    int getCurrentBaseNumber() const;
    bool getCurrentPosition(std::array<double, 6>& position, int& sdkCode) const;
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
    MotionResult moveLinearRelative(const double relative[6], bool wait = true);
    void setDigitalOutput(int index, bool state);
    void firePneumatic(int index, DWORD durationMs);
};
