#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include <iostream>
#include <winsock2.h>
#include <windows.h> 
#include <conio.h>  
#include "HRSDK.h"

#pragma comment(lib, "HRSDK.lib")
#pragma comment(lib, "ws2_32.lib") 

using namespace std;

// 安全回呼函式
void __stdcall arm_callback(uint16_t rob_idx, uint16_t status, unsigned short* msg, int msg_len) {}

int main() {
    SetConsoleOutputCP(CP_UTF8);
    cout << "--- 手臂關節角度移動測試 (安靜模式) ---" << endl;

    // 1. 建立手臂連線
    char arm_ip[] = "192.168.0.1";
    int device_id = open_connection(arm_ip, 1, arm_callback);

    if (device_id < 0) {
        cout << "[錯誤] 無法連線至手臂！錯誤碼: " << device_id << endl;
        system("pause");
        return -1;
    }
    cout << "[成功] 手臂已連線，ID: " << device_id << endl;

    // 2. 初始化硬體
    clear_alarm(device_id);
    Sleep(200);
    set_motor_state(device_id, 1);
    Sleep(500); 

    // 設定移動速度為 50%
    set_override_ratio(device_id, 50);

    // 設定目標角度：J3 轉 90 度，J6 轉 90 度
    double target_joint[6] = { 0.0, 0.0, 90.0, 0.0, 0.0, 90.0};
    double cam_joint[6] = {0.0, -15.0, 30.0, 0.0, 0.0, 90.0 };


    cout << "[動作] 發送移動指令..." << endl;
    ptp_axis(device_id, 0, target_joint);

    // ==========================================
    // 安靜等待邏輯
    // ==========================================
    int wait_start = 0;

    // (A) 等待指令被控制器接受
    while (get_motion_state(device_id) == 1 && wait_start < 20) {
        Sleep(50);
        wait_start++;
    }

    if (wait_start >= 20) {
        cout << "[警告] 控制器拒絕了移動指令！請檢查 AUT 模式或警報。" << endl;
    }
    else {
        // (B) 確定開始動了，安靜等待停止
        cout << "[狀態] 手臂移動中，請稍候..." << endl;
        
        // 核心修改：迴圈內只做 Sleep，不執行 printf
        while (get_motion_state(device_id) != 1) {
            Sleep(100); // 增加間隔減少 CPU 負擔
        }

        // 移動結束後，只印一次最終角度來確認
        double final_joints[6];
        if (get_current_joint(device_id, final_joints) == 0) {
            cout << "[成功] 手臂已抵達目標位置！" << endl;
            printf("最終角度 -> J1:%.2f, J2:%.2f, J3:%.2f, J4:%.2f, J5:%.2f, J6:%.2f\n", 
                   final_joints[0], final_joints[1], final_joints[2], 
                   final_joints[3], final_joints[4], final_joints[5]);
        }
    }

    // 4. 關閉連線
    close_connection(device_id);
    cout << "\n測試完成，按任意鍵結束程式..." << endl;
    system("pause");
    return 0;
}