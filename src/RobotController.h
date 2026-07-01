#pragma once
#include <string>
#include <winsock2.h> // 包含 DWORD

#pragma comment(lib, "HRSDK.lib")

class RobotController {
private:
    int id;
    bool connected;
public:
    RobotController();
    ~RobotController();

    // 手臂連線與中斷
    bool connect(const std::string& ip);
    void disconnect();
    int getId() const;
    bool isConnected() const;

    // 手臂屬性設定與狀態讀取
    void setMotorState(int state);
    void setOverrideRatio(int ratio);
    void setToolNumber(int tool_num);
    int getMotionState();

    // 各種手臂控制指令
    void moveToAxis(const double joint[6], bool wait = true);
    void moveToPosition(const double pos[6], bool wait = true);
    void moveLinearTo(const double pos[6], bool wait = true);
    void moveLinearRelative(const double rel[6], bool wait = true);
    void setDigitalOutput(int index, bool state);
    void firePneumatic(int index, DWORD duration_ms);
};
