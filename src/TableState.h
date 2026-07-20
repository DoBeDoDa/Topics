// 定義視覺偵測結果、球桌狀態與目標選擇結果等領域資料。
#pragma once

#include <array>
#include <string>
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
