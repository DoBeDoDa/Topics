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
    const double STRIKE_Z = -319.769;
    const double SAFE_Z = 0.0;
    const double YAW_OFFSET = 0.0;
    
    const double TILT_RY_DEG = 10.0;
    const double MOVE_BACK_MM = 20.0;
    
    // 預設關節點位
    const double CAM_JOINT[6] = {0.0, -33.564, 49.53, 0.0, -15.574, -90.0};
    
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
    bool processVisionData(char* dataString);
    void runContourAlignment();
};
