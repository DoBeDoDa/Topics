// 母球定位與安全高度測試工具，不下降至正式擊球高度。
#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include <algorithm>
#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <winsock2.h>
#include "RobotController.h"
#include "SocketClient.h"
#include "MathUtils.h"
#include "Point.h"
#include "Algorithm.h"
#include "BilliardConfig.h"
#include <vector>
#include "HRSDK.h"

#pragma comment(lib, "ws2_32.lib")

using namespace std;

namespace {

constexpr double RX_SEARCH_MIN_DEG = -20.0;
constexpr double RX_SEARCH_MAX_DEG = 20.0;
constexpr double RY_SEARCH_MIN_DEG = 15.0;
constexpr double RY_SEARCH_MAX_DEG = 30.0;
constexpr double ORIENTATION_SEARCH_STEP_DEG = 5.0;

struct OrientationCandidate {
    double rxDeg;
    double ryDeg;
    double distanceSquared;
};

void printSixValues(const string& label, const array<double, 6>& values) {
    cout << fixed << setprecision(3) << label;
    for (size_t index = 0; index < values.size(); ++index) {
        cout << (index == 0 ? "" : ", ") << values[index];
    }
    cout << defaultfloat << endl;
}

void printAlarmCodes(const RobotController& robot) {
    int sdkCode = -1;
    vector<uint64_t> alarms = robot.getAlarmCodes(sdkCode);
    if (sdkCode != 0) {
        cout << "[警報] get_alarm_code 失敗，SDK code=" << sdkCode << endl;
        return;
    }
    if (alarms.empty()) {
        cout << "[警報] 控制器目前沒有回報有效警報碼。" << endl;
        return;
    }
    cout << "[警報] 目前警報碼：";
    for (size_t index = 0; index < alarms.size(); ++index) {
        cout << " 0x" << hex << uppercase << alarms[index];
    }
    cout << dec << nouppercase << endl;
}

bool requireMotionSuccess(
    const string& stepName,
    const MotionResult& result,
    const RobotController& robot
) {
    if (result.success) {
        return true;
    }
    cout << "[錯誤] " << stepName << " 失敗，SDK code=" << result.sdkCode
         << "，motion state=" << result.finalMotionState;
    if (result.timedOut) {
        cout << "，等待逾時，motion_abort SDK code=" << result.abortSdkCode;
    }
    cout << endl;
    printAlarmCodes(robot);
    return false;
}

bool findReachableOrientation(
    const array<double, 6>& fixedPose,
    const RobotController& robot,
    array<double, 6>& reachablePose
) {
    vector<OrientationCandidate> candidates;
    for (double rx = RX_SEARCH_MIN_DEG;
         rx <= RX_SEARCH_MAX_DEG;
         rx += ORIENTATION_SEARCH_STEP_DEG) {
        for (double ry = RY_SEARCH_MIN_DEG;
             ry <= RY_SEARCH_MAX_DEG;
             ry += ORIENTATION_SEARCH_STEP_DEG) {
            const double rxDifference = rx - fixedPose[3];
            const double ryDifference = ry - fixedPose[4];
            candidates.push_back({
                rx,
                ry,
                rxDifference * rxDifference + ryDifference * ryDifference
            });
        }
    }

    stable_sort(
        candidates.begin(),
        candidates.end(),
        [](const OrientationCandidate& lhs, const OrientationCandidate& rhs) {
            return lhs.distanceSquared < rhs.distanceSquared;
        }
    );

    cout << "[姿態搜尋] 固定 X、Y、Z、RZ，只搜尋工具姿態：RX="
         << RX_SEARCH_MIN_DEG << " 至 " << RX_SEARCH_MAX_DEG
         << " 度，RY=" << RY_SEARCH_MIN_DEG << " 至 "
         << RY_SEARCH_MAX_DEG << " 度，間隔 "
         << ORIENTATION_SEARCH_STEP_DEG << " 度。" << endl;

    for (size_t index = 0; index < candidates.size(); ++index) {
        array<double, 6> candidatePose = fixedPose;
        candidatePose[3] = candidates[index].rxDeg;
        candidatePose[4] = candidates[index].ryDeg;

        bool reachable = false;
        int sdkCode = -1;
        const bool callSucceeded = robot.checkReachable(
            candidatePose,
            reachable,
            sdkCode
        );
        cout << "[姿態搜尋 " << (index + 1) << "/" << candidates.size()
             << "] RX=" << candidatePose[3]
             << "，RY=" << candidatePose[4]
             << "：SDK code=" << sdkCode
             << "，reachable=" << (reachable ? "true" : "false")
             << endl;

        if (!callSucceeded) {
            cout << "[錯誤] motion_reachable 呼叫失敗，停止姿態搜尋。" << endl;
            printAlarmCodes(robot);
            return false;
        }
        if (reachable) {
            reachablePose = candidatePose;
            return true;
        }
    }

    cout << "[安全停止] 搜尋範圍內沒有可達的 RX/RY 組合，不送出移動指令。"
         << endl;
    printAlarmCodes(robot);
    return false;
}

}  // namespace

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
    const auto failAndExit = [&]() {
        yoloClient.closeConnection();
        robot.disconnect();
        WSACleanup();
        return -1;
    };

    // 1. 連線至手臂控制箱
    cout << "[步驟 1] 正在連線至手臂控制箱 (" << BilliardConfig::ARM_IP << ")..." << endl;
    if (!robot.connect(BilliardConfig::ARM_IP)) {
        cout << "[錯誤] 手臂連線失敗，請檢查網路連線與控制箱電源。" << endl;
        WSACleanup();
        return -1;
    }
    
    robot.setMotorState(1);     // 啟動伺服馬達
    robot.setOverrideRatio(BilliardConfig::NORMAL_SPEED_RATIO);
    robot.setToolNumber(BilliardConfig::TOOL_NUMBER);
    const int activeTool = robot.getCurrentToolNumber();
    const int activeBase = robot.getCurrentBaseNumber();
    cout << "[診斷] Tool=" << activeTool << "，Base=" << activeBase << endl;
    if (activeTool != BilliardConfig::TOOL_NUMBER ||
        activeBase != BilliardConfig::BASE_NUMBER) {
        cout << "[錯誤] 測試要求 Tool=" << BilliardConfig::TOOL_NUMBER
             << "、Base=" << BilliardConfig::BASE_NUMBER
             << "，請先修正控制器座標系。" << endl;
        printAlarmCodes(robot);
        return failAndExit();
    }
    cout << "[成功] 手臂連線成功，伺服馬達已啟動，Tool "
         << BilliardConfig::TOOL_NUMBER << "／Base "
         << BilliardConfig::BASE_NUMBER << " 已確認。" << endl;

    // 2. 移動至拍照基準點，避免遮擋相機視野
    cout << "\n[步驟 2] 移動手臂返回拍照位置 (CAM_JOINT)..." << endl;
    cout << "請確認手臂路徑安全無障礙物，隨後在【此視窗】按下 [Enter] 鍵開始移動: ";
    cin.clear();
    fflush(stdin);
    string confirm_move;
    getline(cin, confirm_move);

    if (!requireMotionSuccess(
        "PTP 移動至拍照點",
        robot.moveToAxis(BilliardConfig::CAMERA_JOINT.data(), true),
        robot
    )) {
        return failAndExit();
    }
    Sleep(1000);
    cout << "[動作] 手臂已抵達拍照點。" << endl;

    // 3. 連線至 Python YOLO 辨識伺服器
    cout << "\n[步驟 3] 正在建立與 Python 影像辨識端的連線 ("
         << BilliardConfig::VISION_SERVER_IP << ":"
         << BilliardConfig::VISION_SERVER_PORT << ")..." << endl;
    cout << "--> 請先確保 python python/robot.py 已經啟動並處於運行狀態。" << endl;
    while (!yoloClient.connectToServer(
        BilliardConfig::VISION_SERVER_IP,
        BilliardConfig::VISION_SERVER_PORT
    )) {
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

        // 呼叫全新的擊球決策演算法
        ShotDecision decision = BilliardAlgorithm::decideShot(
            bw, target_arm, destination, rail_A, rail_B, obs_list,
            BilliardConfig::BALL_DIAMETER_MM, best_pocket_idx
        );

        Point best_aim_target = decision.best_aim_target;
        string strategy_name = decision.strategy_name;
        double angle_deg = decision.angle_deg;

        cout << "\n[決策結果] 策略: " << strategy_name << " | 夾角: " << angle_deg << " 度" << endl;

        Vector2D v_dir = BilliardMath::getVector(bw, best_aim_target);

        double arm_rz = BilliardMath::getVectorAngle(v_dir.x, v_dir.y) +
            BilliardConfig::YAW_OFFSET_DEG;

        // 設定安全測試點（不下降至正式擊球高度）
        array<double, 6> target_pos = {
            bw.x,
            bw.y,
            BilliardConfig::TEST_MOTION.strikeZ,
            BilliardConfig::TEST_MOTION.rxDeg,
            BilliardConfig::TEST_MOTION.tiltRyDeg,
            arm_rz
        };
        // 6. 移動至中繼點與打擊點 (中繼點用來進行手腕組態轉換，避開奇異點)
        cout << "\n[步驟 5] 準備移動手臂至中繼關節位置 (TRANSIT_JOINT) 切換組態..." << endl;
        cout << "請確認安全，並按 [Enter] 開始移動: ";
        string confirm_transit;
        cin.clear();
        fflush(stdin);
        getline(cin, confirm_transit);
        
        cout << "[動作] 移動至中繼點位..." << endl;
        if (!requireMotionSuccess(
            "PTP 移動至中繼關節點",
            robot.moveToAxis(BilliardConfig::TRANSIT_JOINT.data(), true),
            robot
        )) {
            return failAndExit();
        }
        Sleep(800);

        const int transitTool = robot.getCurrentToolNumber();
        const int transitBase = robot.getCurrentBaseNumber();
        cout << "[診斷] 中繼點 Tool=" << transitTool
             << "，Base=" << transitBase << endl;
        if (transitTool != BilliardConfig::TOOL_NUMBER ||
            transitBase != BilliardConfig::BASE_NUMBER) {
            cout << "[錯誤] 中繼點的 Tool／Base 與測試設定不一致，停止動作。" << endl;
            printAlarmCodes(robot);
            return failAndExit();
        }

        int sdkCode = -1;
        array<double, 6> actualTransitPose = {};
        if (!robot.getCurrentPosition(actualTransitPose, sdkCode)) {
            cout << "[錯誤] 無法取得中繼點實際姿態，SDK code="
                 << sdkCode << endl;
            printAlarmCodes(robot);
            return failAndExit();
        }
        printSixValues(
            "[診斷] 中繼點實際姿態 {X, Y, Z, RX, RY, RZ} = ",
            actualTransitPose
        );

        array<double, 6> actualTransitJoints = {};
        if (!robot.getCurrentJoints(actualTransitJoints, sdkCode)) {
            cout << "[錯誤] 無法取得中繼點關節角度，SDK code="
                 << sdkCode << endl;
            printAlarmCodes(robot);
            return failAndExit();
        }
        printSixValues(
            "[診斷] 中繼點實際關節 {A1, A2, A3, A4, A5, A6} = ",
            actualTransitJoints
        );

        cout << "\n[步驟 6] 準備移動至目標點位..." << endl;
        cout << "   - 測試擊球點座標：X = " << target_pos[0]
             << ", Y = " << target_pos[1] 
             << ", Z = " << target_pos[2] << " mm" << endl;
        cout << "   - 原始姿勢角：RX = " << target_pos[3]
             << ", RY = " << target_pos[4] 
             << ", RZ = " << target_pos[5] << endl;
        cout << "==================================================" << endl;

        array<double, 6> reachableTargetPos = {};
        if (!findReachableOrientation(target_pos, robot, reachableTargetPos)) {
            return failAndExit();
        }

        cout << "[姿態搜尋成功] 將使用以下測試擊球姿態：" << endl;
        printSixValues(
            "   {X, Y, Z, RX, RY, RZ} = ",
            reachableTargetPos
        );

        cout << "【安全鎖】請確認平面平移路徑無障礙，且 Z 軸高度不會撞擊桌面。" << endl;
        cout << "確認上列 RX/RY 姿態與路徑安全後，請在【此視窗】按下 [Enter] 鍵開始移動：" << endl;

        string confirm_move2;
        cin.clear();
        fflush(stdin);
        getline(cin, confirm_move2);

        cout << "[動作] 手臂以 PTP 方式移往測試擊球點 (Z = "
             << reachableTargetPos[2] << ")..." << endl;
        if (!requireMotionSuccess(
            "PTP 從中繼點移動至測試擊球點",
            robot.moveToPosition(reachableTargetPos.data(), true),
            robot
        )) {
            return failAndExit();
        }
        Sleep(200);

        cout << "[成功] 已抵達 RX/RY 搜尋得到的測試擊球點。" << endl;

        char return_confirm = 'n';
        while (return_confirm != 'y' && return_confirm != 'Y') {
            cout << "\a\n[定位確認] 已抵達目標位置！請確認筆尖是否與球心重合。輸入 'y' 返回拍照點: ";
            cin >> return_confirm;
        }

        cout << "[動作] 手臂返回拍照點..." << endl;
        if (!requireMotionSuccess(
            "PTP 返回拍照點",
            robot.moveToAxis(BilliardConfig::CAMERA_JOINT.data(), true),
            robot
        )) {
            return failAndExit();
        }
        Sleep(800);
    }

    // 關閉資源
    yoloClient.closeConnection();
    robot.disconnect();
    WSACleanup();
    cout << "\n[系統] 測試結束，程式關閉。" << endl;
    return 0;
}
