#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include <iostream>
#include <string>
#include <winsock2.h>
#include "RobotController.h"
#include "SocketClient.h"
#include "HRSDK.h"

#pragma comment(lib, "ws2_32.lib")

using namespace std;

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
    cout << "[系統] 機械手臂連線成功，馬達已啟動。" << endl;

    SocketClient pythonClient;
    cout << "[系統] 正在連線至 Python 影像偵測端 (Port: 12347)..." << endl;
    while (!pythonClient.connectToServer("127.0.0.1", 12347)) {
        Sleep(1000);
    }
    cout << "[系統] 連線至 Python 影像端成功！" << endl;

    char recvbuf[512];
    string balls[] = { "bw", "b1", "b2", "b3" };
    string ball_names[] = { "母球 (bw)", "1號球 (b1)", "2號球 (b2)", "3號球 (b3)" };

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

            // 清理 C++ 輸入緩衝區，確保不會直接讀取到先前的殘留輸入
            cin.clear();
            fflush(stdin);

            for (int i = 0; i < 4; i++) {
                cout << "\n-----------------------------------------" << endl;
                cout << "  步驟 " << (i + 1) << "/4：請對準【" << ball_names[i] << "】" << endl;
                cout << "  --> 請手動將手臂末端移動到該球的實體中心位置。" << endl;
                cout << "  對準後，請在【此視窗】按下 [Enter] 鍵繼續..." << endl;

                // 等待 Enter 按下
                string dummy;
                getline(cin, dummy);

                // 取得目前手臂末端座標
                double cart[6] = {0.0};
                if (get_current_position(robot.getId(), cart) == 0) {
                    double x = cart[0];
                    double y = cart[1];
                    cout << "[記錄] 手臂目前座標：X = " << x << " mm, Y = " << y << " mm" << endl;

                    // 發送給 Python 格式為 "ball_name,X,Y\n"
                    string send_msg = balls[i] + "," + to_string(x) + "," + to_string(y) + "\n";
                    pythonClient.sendData(send_msg);
                } else {
                    cout << "[錯誤] 無法取得手臂座標，發送 0,0 代替。" << endl;
                    string send_msg = balls[i] + ",0.0,0.0\n";
                    pythonClient.sendData(send_msg);
                }
            }

            cout << "\n[系統] 四點座標量測完畢，等待影像端計算透視變換矩陣..." << endl;
        }
    }

    robot.disconnect();
    pythonClient.closeConnection();
    WSACleanup();
    return 0;
}
