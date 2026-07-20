// 宣告撞球應用流程協調器及其依賴元件。
#pragma once

#include <array>
#include <string>

#include "Algorithm.h"
#include "MotionPlanner.h"
#include "RobotController.h"
#include "SocketClient.h"
#include "TargetSelector.h"
#include "VisionDataParser.h"

class BilliardApp {
private:
    RobotController robot;
    SocketClient visionClient;
    VisionDataParser visionParser;
    TargetSelector targetSelector;
    MotionPlanner motionPlanner;
    bool needCameraMove;

public:
    BilliardApp();

    bool initialize();
    void run();

private:
    void moveToCameraPosition();
    bool processVisionData(const std::string& dataString);
    bool executeMotionPlan(const MotionPlan& plan);
    bool requireReachable(
        const std::string& pointName,
        const std::array<double, 6>& pose
    );
    bool requireMotionSuccess(
        const std::string& stepName,
        const MotionResult& result
    );
    void printPose(
        const std::string& label,
        const std::array<double, 6>& pose
    ) const;
    void printAlarmCodes() const;
};
