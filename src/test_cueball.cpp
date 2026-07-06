#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include <iostream>
#include <string>
#include <winsock2.h>
#include "RobotController.h"
#include "SocketClient.h"
#include "MathUtils.h"
#include "Point.h"
#include "HRSDK.h"

#pragma comment(lib, "ws2_32.lib")

using namespace std;

int main() {
    // 設定編碼支援繁體中文輸出
    setlocale(LC_ALL, "zh_TW.UTF-8");
    cout << "==================================================" << endl;
    cout << "      撞球 AI 視覺定位測試程式 (母球測試版)       " << endl;
    cout << "==================================================" << endl;

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cout << "[錯誤] Winsock 初始化 (WSAStartup) 失敗。" << endl;
        return -1;
    }

    RobotController robot;
    SocketClient yoloClient;

    // 1. 連線至手臂控制箱
    const string ARM_IP = "192.168.0.1";
    cout << "[步驟 1] 正在連線至手臂控制箱 (" << ARM_IP << ")..." << endl;
    if (!robot.connect(ARM_IP)) {
        cout << "[錯誤] 手臂連線失敗，請檢查網路連線與控制箱電源。" << endl;
        WSACleanup();
        return -1;
    }
    
    robot.setMotorState(1);     // 啟動伺服馬達
    robot.setOverrideRatio(40); // 限制運行速度在 40% 以策安全
    robot.setToolNumber(1);     // 使用工具軸 1 座標系 (與校正一致)
    cout << "[成功] 手臂連線成功，伺服馬達已啟動，已切換至工具軸 1。" << endl;

    // 2. 移動至拍照基準點，避免遮擋相機視野
    const double CAM_JOINT[6] = {0.0, -32.319, 51.653, 0.0, -18.813, -90.0};
    cout << "\n[步驟 2] 移動手臂返回拍照位置 (CAM_JOINT)..." << endl;
    cout << "請確認手臂路徑安全無障礙物，隨後在【此視窗】按下 [Enter] 鍵開始移動: ";
    cin.clear();
    fflush(stdin);
    string confirm_move;
    getline(cin, confirm_move);

    robot.moveToAxis(CAM_JOINT);
    Sleep(1000);
    cout << "[動作] 手臂已抵達拍照點。" << endl;

    // 3. 連線至 Python YOLO 辨識伺服器
    const string PYTHON_IP = "127.0.0.1";
    const int PYTHON_PORT = 12345;
    cout << "\n[步驟 3] 正在建立與 Python 影像辨識端的連線 (" << PYTHON_IP << ":" << PYTHON_PORT << ")..." << endl;
    cout << "--> 請先確保 python python/robot.py 已經啟動並處於運行狀態。" << endl;
    while (!yoloClient.connectToServer(PYTHON_IP, PYTHON_PORT)) {
        cout << "正在等待 Python 伺服器回應中..." << endl;
        Sleep(1000);
    }
    cout << "[成功] 與 Python 視覺端建立連線！開始偵測影像。" << endl;

    // 4. 連續接收視覺數據，直至偵測到母球
    char recvbuf[2048];
    double bwx = -9999.0, bwy = -9999.0;
    bool found_cueball = false;

    cout << "\n[步驟 4] 正在搜尋畫面中的母球 (bw)..." << endl;
    while (!found_cueball) {
        int bytes = yoloClient.receiveData(recvbuf, sizeof(recvbuf) - 1);
        if (bytes > 0) {
            recvbuf[bytes] = '\0';
            
            // 定義 32 個解析欄位
            double b1x, b1y, b2x, b2y, b3x, b3y, b4x, b4y, b5x, b5y, b6x, b6y, b7x, b7y, b8x, b8y, b9x, b9y, p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, p5x, p5y, p6x, p6y;
            
            int parsed = sscanf_s(recvbuf, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                &b1x, &b1y, &b2x, &b2y, &b3x, &b3y, &b4x, &b4y, &b5x, &b5y, &b6x, &b6y, &b7x, &b7y, &b8x, &b8y, &b9x, &b9y, &bwx, &bwy, &p1x, &p1y, &p2x, &p2y, &p3x, &p3y, &p4x, &p4y, &p5x, &p5y, &p6x, &p6y);
            
            if (parsed == 32) {
                if (bwx > -9000.0) {
                    cout << "\n[鎖定] 成功偵測到母球像素轉換後的物理座標：" << endl;
                    cout << "   - 原始辨識：X = " << bwx << " mm, Y = " << bwy << " mm" << endl;
                    found_cueball = true;
                } else {
                    cout << "\r[尋找中] 尚未偵測到母球... 請將母球放置於球桌上" << flush;
                }
            }
        } else if (bytes <= 0) {
            cout << "\n[錯誤] 與 Python 端通訊中斷。" << endl;
            break;
        }
        Sleep(100);
    }

    if (found_cueball) {
        // 5. 進行相機視角與傾斜度補償
        Point bw_compensated = BilliardMath::applyCameraCompensation({ bwx, bwy });
        cout << "   - 畸變補償：X = " << bw_compensated.x << " mm, Y = " << bw_compensated.y << " mm" << endl;

        // 6. 前往母球點位，Z 軸高度設為 -107
        double current_cart[6] = {0.0};
        if (get_current_position(robot.getId(), current_cart) == 0) {
            double target_pos[6];
            target_pos[0] = bw_compensated.x;  // X 軸 (補償後座標)
            target_pos[1] = bw_compensated.y;  // Y 軸 (補償後座標)
            target_pos[2] = -107.0;            // Z 軸固定於 -107.0 mm
            target_pos[3] = current_cart[3];   // RX 姿勢維持拍照角度
            target_pos[4] = current_cart[4];   // RY 姿勢維持拍照角度
            target_pos[5] = current_cart[5];   // RZ 姿勢維持拍照角度

            cout << "\n[步驟 5] 準備讓末端移動至母球點位！" << endl;
            cout << "   - 目標 Cartesian 座標：X = " << target_pos[0] 
                 << ", Y = " << target_pos[1] 
                 << ", Z = " << target_pos[2] << " mm" << endl;
            cout << "==================================================" << endl;
            cout << "【安全鎖】請確認手臂運行軌跡無障礙，且 Z 軸高度不會撞擊桌面。" << endl;
            cout << "確認完畢後，請在【此視窗】按下 [Enter] 鍵開始移動：" << endl;

            string confirm_move2;
            cin.clear();
            fflush(stdin);
            getline(cin, confirm_move2);

            cout << "[動作] 手臂正在移往母球點位 (Z = -107.0)..." << endl;
            robot.moveToPosition(target_pos, true);
            cout << "[成功] 已抵達目標母球點位。" << endl;
        } else {
            cout << "[錯誤] 無法讀取手臂目前姿態，為安全起見取消定位移動。" << endl;
        }
    }

    // 關閉資源
    yoloClient.closeConnection();
    robot.disconnect();
    WSACleanup();
    cout << "\n[系統] 測試結束，程式關閉。" << endl;
    return 0;
}
