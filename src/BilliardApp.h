#pragma once
#include <string>
#include <winsock2.h>
#include "RobotController.h"
#include "SocketClient.h"

class BilliardApp {
private:
    // 系統硬體與通訊埠參數
    const std::string ARM_IP = "192.168.0.1";
    const int PYTHON_PORT = 12345;
    const int ALIGN_PORT = 12346;
    
    const int PNEUMATIC_DO = 1;
    const double BALL_D = 49.52;
    const double STRIKE_Z = -294.960;
    const double SAFE_Z = 0.0;
    const double YAW_OFFSET = 0.0;
    
    const double TILT_RY_DEG = 10.0;
    const double MOVE_BACK_MM = 20.0;
    
    // 預設關節點位
    const double CAM_JOINT[6] = {0.0, -32.319, 51.653, 0.0, -18.813, -90.0};
    const double BREAK_JOINT[6] = {-2.599, -87.736, 48.629, 3.794, -50.23, 103.847};
    const double SECOND_JOINT[6] = {63.243, -93.544, 63.324, 0.000, -67.246, 36.322};
    
    const DWORD TASK2_EXTEND_MS = 150;
    
    RobotController robot;
    SocketClient yoloClient;
    int shotCount;
    bool needCameraMove;

public:
    BilliardApp();
    
    // 初始化連線與設定
    bool initialize();
    
    // 執行撞球系統主程式迴圈
    void run();

private:
    // 各子程序功能
    void moveToCameraPosition();
    bool handleFirstShot();
    bool handleSecondShot();
    bool processVisionData(char* dataString);
    void runContourAlignment();
};
