#pragma once

#include <string>

#include "MotionPlanner.h"
#include "RobotController.h"
#include "ShotPlanner.h"
#include "SocketClient.h"
#include "TargetSelector.h"
#include "VisionDataParser.h"

class BilliardApp {
private:
    RobotController robot;
    SocketClient visionClient;
    VisionDataParser visionParser;
    TargetSelector targetSelector;
    ShotPlanner shotPlanner;
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
    void runContourAlignment();
};
