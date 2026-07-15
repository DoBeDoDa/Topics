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
#include "Algorithm.h"
#include <vector>
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
    robot.setOverrideRatio(20); // 限制運行速度在 20% 以策安全
    robot.setToolNumber(1);     // 使用工具軸 1 座標系 (與校正一致)
    cout << "[成功] 手臂連線成功，伺服馬達已啟動，已切換至工具軸 1。" << endl;

    // 2. 移動至拍照基準點，避免遮擋相機視野
    const double CAM_JOINT[6] = {0.0, -33.564, 49.53, 0.0, -15.574, -90.0};
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
    double b1x = -9999.0, b1y = -9999.0, b2x = -9999.0, b2y = -9999.0, b3x = -9999.0, b3y = -9999.0, b4x = -9999.0, b4y = -9999.0;
    double b5x = -9999.0, b5y = -9999.0, b6x = -9999.0, b6y = -9999.0, b7x = -9999.0, b7y = -9999.0, b8x = -9999.0, b8y = -9999.0, b9x = -9999.0, b9y = -9999.0;
    double p1x = -9999.0, p1y = -9999.0, p2x = -9999.0, p2y = -9999.0, p3x = -9999.0, p3y = -9999.0, p4x = -9999.0, p4y = -9999.0, p5x = -9999.0, p5y = -9999.0, p6x = -9999.0, p6y = -9999.0;
    bool found_cueball = false;

    cout << "\n[步驟 4] 正在搜尋畫面中的母球 (bw)..." << endl;
    while (!found_cueball) {
        int bytes = yoloClient.receiveData(recvbuf, sizeof(recvbuf) - 1);
        if (bytes > 0) {
            recvbuf[bytes] = '\0';
            
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
        Point bw = BilliardMath::applyCameraCompensation({ bwx, bwy });
        cout << "   - 母球 (bw) 補償座標：X = " << bw.x << " mm, Y = " << bw.y << " mm" << endl;

        Point target_arm = {-9999.0, -9999.0};
        string target_name = "";

        if (b1x > -9000.0) { target_arm = BilliardMath::applyCameraCompensation({b1x, b1y}); target_name = "1號球"; }
        else if (b2x > -9000.0) { target_arm = BilliardMath::applyCameraCompensation({b2x, b2y}); target_name = "2號球"; }
        else if (b3x > -9000.0) { target_arm = BilliardMath::applyCameraCompensation({b3x, b3y}); target_name = "3號球"; }
        else if (b4x > -9000.0) { target_arm = BilliardMath::applyCameraCompensation({b4x, b4y}); target_name = "4號球"; }
        else if (b5x > -9000.0) { target_arm = BilliardMath::applyCameraCompensation({b5x, b5y}); target_name = "5號球"; }
        else if (b6x > -9000.0) { target_arm = BilliardMath::applyCameraCompensation({b6x, b6y}); target_name = "6號球"; }
        else if (b7x > -9000.0) { target_arm = BilliardMath::applyCameraCompensation({b7x, b7y}); target_name = "7號球"; }
        else if (b8x > -9000.0) { target_arm = BilliardMath::applyCameraCompensation({b8x, b8y}); target_name = "8號球"; }
        else if (b9x > -9000.0) { target_arm = BilliardMath::applyCameraCompensation({b9x, b9y}); target_name = "9號球"; }

        if (target_arm.x < -9000.0) {
            cout << "[錯誤] 沒有偵測到任何目標球，無法進行演算法幾何計算。" << endl;
            return -1;
        }

        // 收集所有偵測到的有效球袋
        std::vector<std::pair<int, Point>> valid_pockets;
        if (p1x > -9000.0) valid_pockets.push_back({1, BilliardMath::applyCameraCompensation({ p1x, p1y })});
        if (p2x > -9000.0) valid_pockets.push_back({2, BilliardMath::applyCameraCompensation({ p2x, p2y })});
        if (p3x > -9000.0) valid_pockets.push_back({3, BilliardMath::applyCameraCompensation({ p3x, p3y })});
        if (p4x > -9000.0) valid_pockets.push_back({4, BilliardMath::applyCameraCompensation({ p4x, p4y })});
        if (p5x > -9000.0) valid_pockets.push_back({5, BilliardMath::applyCameraCompensation({ p5x, p5y })});
        if (p6x > -9000.0) valid_pockets.push_back({6, BilliardMath::applyCameraCompensation({ p6x, p6y })});

        if (valid_pockets.empty()) {
            cout << "[錯誤] 沒有偵測到任何有效球袋，無法進行路徑計算。" << endl;
            return -1;
        }

        // 尋找與「母球 -> 目標球」向量夾角最小的「目標球 -> 球袋」向量
        int best_pocket_idx = -1;
        Point destination = {-9999.0, -9999.0};
        double min_angle = 9999.0;
        Vector2D vec_cue = BilliardMath::getVector(bw, target_arm);

        for (const auto& pkt : valid_pockets) {
            Vector2D vec_pocket = BilliardMath::getVector(target_arm, pkt.second);
            double angle = BilliardMath::getAngleBetweenVectors(vec_cue.x, vec_cue.y, vec_pocket.x, vec_pocket.y);
            if (angle < min_angle) {
                min_angle = angle;
                best_pocket_idx = pkt.first;
                destination = pkt.second;
            }
        }

        if (p2x < -9000.0 || p3x < -9000.0) {
            cout << "[錯誤] 缺少 2 號或 3 號球袋座標，無法建立顆星牆壁。" << endl;
            return -1;
        }
        Point rail_A = BilliardMath::applyCameraCompensation({ p2x, p2y });
        Point rail_B = BilliardMath::applyCameraCompensation({ p3x, p3y });

        vector<Point> obs_list;
        if (b1x > -9000.0 && target_name != "1號球") obs_list.push_back(BilliardMath::applyCameraCompensation({b1x, b1y}));
        if (b2x > -9000.0 && target_name != "2號球") obs_list.push_back(BilliardMath::applyCameraCompensation({b2x, b2y}));
        if (b3x > -9000.0 && target_name != "3號球") obs_list.push_back(BilliardMath::applyCameraCompensation({b3x, b3y}));
        if (b4x > -9000.0 && target_name != "4號球") obs_list.push_back(BilliardMath::applyCameraCompensation({b4x, b4y}));
        if (b5x > -9000.0 && target_name != "5號球") obs_list.push_back(BilliardMath::applyCameraCompensation({b5x, b5y}));
        if (b6x > -9000.0 && target_name != "6號球") obs_list.push_back(BilliardMath::applyCameraCompensation({b6x, b6y}));
        if (b7x > -9000.0 && target_name != "7號球") obs_list.push_back(BilliardMath::applyCameraCompensation({b7x, b7y}));
        if (b8x > -9000.0 && target_name != "8號球") obs_list.push_back(BilliardMath::applyCameraCompensation({b8x, b8y}));
        if (b9x > -9000.0 && target_name != "9號球") obs_list.push_back(BilliardMath::applyCameraCompensation({b9x, b9y}));

        const double BALL_D = 49.52;
        // 呼叫全新的擊球決策演算法
        ShotDecision decision = BilliardAlgorithm::decideShot(
            bw, target_arm, destination, rail_A, rail_B, obs_list, BALL_D, best_pocket_idx
        );

        Point best_aim_target = decision.best_aim_target;
        string strategy_name = decision.strategy_name;
        double angle_deg = decision.angle_deg;

        cout << "\n[決策結果] 策略: " << strategy_name << " | 夾角: " << angle_deg << " 度" << endl;

        Vector2D v_dir = BilliardMath::getVector(bw, best_aim_target);
        double v_dist = BilliardMath::getLength(v_dir.x, v_dir.y);
        
        const double YAW_OFFSET = 0.0;
        double arm_rz = BilliardMath::getVectorAngle(v_dir.x, v_dir.y) + YAW_OFFSET;

        // 設定打擊目標點 (使工具軸 1 末端直接與母球球心重合，Z軸在 -290.0)
        double target_pos[6];
        target_pos[0] = bw.x;      // 與球心 X 重合
        target_pos[1] = bw.y;      // 與球心 Y 重合
        target_pos[2] = -290.0;    // Z 軸設定在 -290.0 mm
        target_pos[3] = 0.0;       // RX
        const double TILT_RY_DEG = 10.0;
        target_pos[4] = TILT_RY_DEG; // RY (傾斜 10 度)
        target_pos[5] = arm_rz;    // RZ (瞄準角)

        double ready_pos[6];
        memcpy(ready_pos, target_pos, sizeof(ready_pos));
        ready_pos[2] = -125.0;     // 安全預備點高度 (Z = -125.0)

        // 6. 移動至中繼點與打擊點 (中繼點用來進行手腕組態轉換，避開奇異點)
        const double TRANSIT_JOINT[6] = {-12.0, -44.0, -17.0, -14.0, 42.0, -150.0};
        cout << "\n[步驟 5] 準備移動手臂至中繼關節位置 (TRANSIT_JOINT) 切換組態..." << endl;
        cout << "請確認安全，並按 [Enter] 開始移動: ";
        string confirm_transit;
        cin.clear();
        fflush(stdin);
        getline(cin, confirm_transit);
        
        cout << "[動作] 移動至中繼點位..." << endl;
        robot.moveToAxis(TRANSIT_JOINT, true);
        Sleep(800);

        cout << "\n[步驟 6] 準備移動至目標點位..." << endl;
        cout << "   - 預備點座標：X = " << ready_pos[0] 
             << ", Y = " << ready_pos[1] 
             << ", Z = " << ready_pos[2] << " mm" << endl;
        cout << "   - 球心點座標：X = " << target_pos[0] 
             << ", Y = " << target_pos[1] 
             << ", Z = " << target_pos[2] << " mm" << endl;
        cout << "   - 姿勢角：RX = " << target_pos[3] 
             << ", RY = " << target_pos[4] 
             << ", RZ = " << target_pos[5] << endl;
        cout << "==================================================" << endl;
        cout << "【安全鎖】請確認平面平移路徑無障礙，且 Z 軸高度不會撞擊桌面。" << endl;
        cout << "確認完畢後，請在【此視窗】按下 [Enter] 鍵開始移動：" << endl;

        string confirm_move2;
        cin.clear();
        fflush(stdin);
        getline(cin, confirm_move2);

        cout << "[動作] 手臂以 PTP 方式移往預備點 (Z = -125.0)..." << endl;
        robot.moveToPosition(ready_pos, true);
        Sleep(200);

        cout << "[動作] 手臂直線下降至球心高度 (Z = -290.0)..." << endl;
        robot.moveLinearTo(target_pos, true);
        cout << "[成功] 已抵達目標球心點位。" << endl;

        char return_confirm = 'n';
        while (return_confirm != 'y' && return_confirm != 'Y') {
            cout << "\a\n[定位確認] 已抵達目標位置！請確認筆尖是否與球心重合。輸入 'y' 返回拍照點: ";
            cin >> return_confirm;
        }

        cout << "[動作] 手臂直線抬升至預備點..." << endl;
        robot.moveLinearTo(ready_pos, true);

        cout << "[動作] 手臂返回拍照點..." << endl;
        robot.moveToAxis(CAM_JOINT, true);
        Sleep(800);
    }

    // 關閉資源
    yoloClient.closeConnection();
    robot.disconnect();
    WSACleanup();
    cout << "\n[系統] 測試結束，程式關閉。" << endl;
    return 0;
}
