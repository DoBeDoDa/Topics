#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include <iostream>
#include <string>
#include <winsock2.h>
#include <conio.h>
#include "RobotController.h"
#include "SocketClient.h"
#include "HRSDK.h"

#pragma comment(lib, "ws2_32.lib")

using namespace std;

// 拍照點位關節角度 (A1 ~ A6)
const double CAM_JOINT[6] = {0.0, -32.319, 51.653, 0.0, -18.813, -90.0};

// 回傳 true 表示確認，false 表示重新記錄 (按 Delete/Backspace/D)
bool askConfirmation() {
    cout << "  --> [Enter] 確認此點並前往下一球 | [Delete] 或 [D] 鍵刪除並重新量測..." << endl;
    while (true) {
        if (_kbhit()) {
            int ch = _getch();
            if (ch == 13 || ch == 10) { // Enter 鍵
                return true;
            }
            if (ch == 'd' || ch == 'D') {
                return false;
            }
            if (ch == 224 || ch == 0) { // 擴充鍵 (如 Delete, 方向鍵)
                int next_ch = _getch();
                if (next_ch == 83) { // Delete 鍵的掃描碼
                    return false;
                }
            }
            if (ch == 8) { // Backspace 鍵
                return false;
            }
        }
        Sleep(10);
    }
}

int main() {
    // 支援繁體中文輸出
    setlocale(LC_ALL, "zh_TW.UTF-8");
    cout << "=========================================" << endl;
    cout << "   上銀機械手臂 Homography 自動標定程序   " << endl;
    cout << "=========================================" << endl;

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cout << "[錯誤] WSAStartup 失敗。" << endl;
        return -1;
    }

    RobotController robot;
    cout << "[系統] 正在連線至機械手臂 (IP: 192.168.0.1)..." << endl;
    if (!robot.connect("192.168.0.1")) {
        cout << "[錯誤] 手臂連線失敗。請確認手臂 IP 以及網路線連接。" << endl;
        WSACleanup();
        system("pause");
        return -1;
    }
    robot.setMotorState(1);
    robot.setOverrideRatio(20);
    robot.setToolNumber(1);  // 使用工具軸 1 座標系
    cout << "[系統] 機械手臂連線成功，馬達已啟動，已切換至工具軸 1。" << endl;

    // 在連線至 Python 之前，先將手臂移到拍照點位以避免相機視野受阻
    cout << "\n[系統] 正在自動移動至拍照位置 (CAM_JOINT)..." << endl;
    robot.moveToAxis(CAM_JOINT, true);
    cout << "[系統] 已到達拍照位置。請在球桌上擺放好球。" << endl;

    SocketClient pythonClient;
    cout << "[系統] 正在連線至 Python 影像偵測端 (Port: 12347)..." << endl;
    while (!pythonClient.connectToServer("127.0.0.1", 12347)) {
        Sleep(1000);
    }
    cout << "[系統] 連線至 Python 影像端成功！" << endl;

    char recvbuf[512];
    string balls[] = { "ptA", "ptB" };
    string ball_names[] = { "棋盤左上角點 A (紅圈標記)", "棋盤右下角點 B (藍圈標記)" };

    // 1. 首次啟動：已到達拍照點，直接等待按 Enter 才拍照
    cout << "\n準備就緒後，請在【此視窗】按下 [Enter] 鍵開始拍照辨識..." << endl;
    string init_dummy;
    getline(cin, init_dummy);
    pythonClient.sendData("START_DETECTION\n");

    while (true) {
        cout << "\n[相機] 等待影像偵測端鎖定所有球點像素位置..." << endl;
        
        int bytes = pythonClient.receiveData(recvbuf, sizeof(recvbuf) - 1);
        if (bytes <= 0) {
            cout << "[網路] 與 Python 端斷開連線。" << endl;
            break;
        }
        
        recvbuf[bytes] = '\0';
        string msg(recvbuf);
        if (!msg.empty() && msg.back() == '\n') msg.pop_back();

        if (msg == "START_CALIBRATION") {
            cout << "\n=========================================" << endl;
            cout << "  標定回合開始！請依提示移動手臂末端" << endl;
            cout << "=========================================" << endl;

            for (int i = 0; i < 2; i++) {
                bool point_confirmed = false;
                while (!point_confirmed) {
                    cout << "\n-----------------------------------------" << endl;
                    cout << "  步驟 " << (i + 1) << "/2：請對準【" << ball_names[i] << "】" << endl;
                    cout << "  --> 請手動將手臂末端移動到該球的實體中心位置。" << endl;
                    cout << "  對準後，請在【此視窗】按下 [Enter] 鍵記錄座標，或輸入 [s] 鍵跳過此球..." << endl;

                    // 清理緩衝區並等待 Enter
                    cin.clear();
                    fflush(stdin);
                    string dummy;
                    getline(cin, dummy);

                    // 檢查使用者是否輸入 s/S 跳過
                    if (dummy == "s" || dummy == "S") {
                        cout << "[系統] 跳過此球！將其標記為未採集。" << endl;
                        string send_msg = balls[i] + ",-9999.0,-9999.0\n";
                        pythonClient.sendData(send_msg);
                        point_confirmed = true;
                        continue;
                    }

                    // 取得目前手臂座標 (工具軸 1 座標系)
                    robot.setToolNumber(1);
                    double cart[6] = {0.0};
                    if (get_current_position(robot.getId(), cart) == 0) {
                        double x = cart[0];
                        double y = cart[1];
                        cout << "[記錄] 工具軸 1 座標：X = " << x << " mm, Y = " << y << " mm" << endl;

                        // 進行點位確認 (支援 Enter 確認，Delete/Backspace/D 重新記錄)
                        if (askConfirmation()) {
                            cout << "[系統] 已確認此點座標。" << endl;
                            string send_msg = balls[i] + "," + to_string(x) + "," + to_string(y) + "\n";
                            pythonClient.sendData(send_msg);
                            point_confirmed = true;
                        } else {
                            cout << "[系統] 刪除此點，請重新調整位置後再次記錄..." << endl;
                        }
                    } else {
                        cout << "[錯誤] 無法取得手臂座標，發送 0,0 代替。" << endl;
                        string send_msg = balls[i] + ",0.0,0.0\n";
                        pythonClient.sendData(send_msg);
                        point_confirmed = true;
                    }
                }
            }

            cout << "\n[系統] 棋盤基準點座標量測完畢。" << endl;
            
            // 2. 標定完成後，等待使用者確認才移動回拍照位置
            cout << "\n[安全鎖] 標定採集完畢！手臂即將移動回拍照位置 (CAM_JOINT)。" << endl;
            cout << "請確認手臂路徑安全（無障礙物），隨後在【此視窗】按下 [Enter] 鍵啟動移動: ";
            cin.clear();
            fflush(stdin);
            string ready_move_back;
            getline(cin, ready_move_back);

            cout << "[手臂] 正在移動回拍照位置 (CAM_JOINT)..." << endl;
            robot.moveToAxis(CAM_JOINT, true);
            cout << "[手臂] 已到達拍照位置。請重新擺放球點。" << endl;
            cout << "準備就緒後，請在【此視窗】按下 [Enter] 開始下一輪拍照與標定..." << endl;
            
            cin.clear();
            fflush(stdin);
            string next_dummy;
            getline(cin, next_dummy);
            pythonClient.sendData("START_DETECTION\n");
        }
    }

    robot.disconnect();
    pythonClient.closeConnection();
    WSACleanup();
    return 0;
}
