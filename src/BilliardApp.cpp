#include "BilliardApp.h"
#include "BilliardPhysics.h"
#include "MathUtils.h"
#include "Point.h"
#include "Algorithm.h"
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

BilliardApp::BilliardApp() : shotCount(1), needCameraMove(true) {}

bool BilliardApp::initialize() {
    if (!robot.connect(ARM_IP)) {
        cout << "[錯誤] 手臂連線失敗。" << endl;
        return false;
    }
    
    robot.setMotorState(1);
    robot.setOverrideRatio(20);
    robot.setToolNumber(1);  // 統一使用工具軸 1 座標系

    // 在連線至 Python 之前，先將手臂移到拍照位置以避免相機視野受阻
    moveToCameraPosition();

    cout << "[系統] 等待 Python 連線..." << endl;
    while (!yoloClient.connectToServer("127.0.0.1", PYTHON_PORT)) {
        Sleep(1000);
    }
    cout << "[系統] 與 Python 服務連線成功！" << endl;
    return true;
}

void BilliardApp::run() {
    char recvbuf[2048];
    while (true) {
        if (needCameraMove) {
            moveToCameraPosition();
            yoloClient.flushBuffer();
            needCameraMove = false;
        }

        robot.setToolNumber(1);
        int bytes = yoloClient.receiveData(recvbuf, sizeof(recvbuf) - 1);
        if (bytes > 0) {
            recvbuf[bytes] = '\0';
            if (!processVisionData(recvbuf)) {
                continue;
            }
        } else if (bytes == 0) {
            cout << "[系統] 影像端 Socket 連線正常關閉。" << endl;
            break;
        } else {
            cout << "[系統] 影像端 Socket 發生錯誤或斷線。" << endl;
            break;
        }
    }
}

void BilliardApp::moveToCameraPosition() {
    cout << "\n[安全鎖] 準備返回拍照點..." << endl;
    cout << "請確認手臂前方安全無障礙物，隨後在【此視窗】按下 [Enter] 鍵繼續: ";
    cin.clear();
    fflush(stdin);
    string confirm;
    getline(cin, confirm);

    cout << "[動作] 移動至拍照點..." << endl;
    robot.moveToAxis(CAM_JOINT);
    Sleep(800);
}

bool BilliardApp::processVisionData(char* dataString) {
    double b1x, b1y, b2x, b2y, b3x, b3y, b4x, b4y, b5x, b5y, b6x, b6y, b7x, b7y, b8x, b8y, b9x, b9y, bwx, bwy, p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, p5x, p5y, p6x, p6y;

    if (sscanf_s(dataString, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
        &b1x, &b1y, &b2x, &b2y, &b3x, &b3y, &b4x, &b4y, &b5x, &b5y, &b6x, &b6y, &b7x, &b7y, &b8x, &b8y, &b9x, &b9y, &bwx, &bwy, &p1x, &p1y, &p2x, &p2y, &p3x, &p3y, &p4x, &p4y, &p5x, &p5y, &p6x, &p6y) == 32) {
        
        if (bwx < -9000.0) { 
            cout << "\r[狀態] 尋找母球中...                  " << flush; 
            return false; 
        }
        Point bw = BilliardMath::applyCameraCompensation({ bwx, bwy });

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

        // 收集所有偵測到的有效球袋並進行相機補償
        std::vector<std::pair<int, Point>> valid_pockets;
        if (p1x > -9000.0) valid_pockets.push_back({1, BilliardMath::applyCameraCompensation({ p1x, p1y })});
        if (p2x > -9000.0) valid_pockets.push_back({2, BilliardMath::applyCameraCompensation({ p2x, p2y })});
        if (p3x > -9000.0) valid_pockets.push_back({3, BilliardMath::applyCameraCompensation({ p3x, p3y })});
        if (p4x > -9000.0) valid_pockets.push_back({4, BilliardMath::applyCameraCompensation({ p4x, p4y })});
        if (p5x > -9000.0) valid_pockets.push_back({5, BilliardMath::applyCameraCompensation({ p5x, p5y })});
        if (p6x > -9000.0) valid_pockets.push_back({6, BilliardMath::applyCameraCompensation({ p6x, p6y })});

        if (valid_pockets.empty()) {
            cout << "\r[錯誤] 核心錯誤：沒有偵測到任何有效球袋座標，拒絕執行擊打...  " << flush;
            return false;
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

        cout << "\n[決策] 選擇夾角最小的目標球袋為 p" << best_pocket_idx << " (夾角: " << min_angle << " 度)" << endl;

        if (p2x < -9000.0 || p3x < -9000.0) {
            cout << "\r[錯誤] 核心錯誤：缺少 2 號或 3 號球袋座標，無法建立顆星牆壁...  " << flush;
            return false;
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
            bw, target_arm, destination, rail_A, rail_B, obs_list, BALL_D, best_pocket_idx
        );

        Point best_aim_target = decision.best_aim_target;
        string strategy_name = decision.strategy_name;
        double angle_deg = decision.angle_deg;

        Vector2D v_dir = BilliardMath::getVector(bw, best_aim_target);
        
        // 使用 MathUtils 函式庫計算母球到最佳擊球點的距離
        double v_dist = BilliardMath::getLength(v_dir.x, v_dir.y);
        if (v_dist < 5.0) return false; 
        
        // 使用 MathUtils 函式庫計算向量平面角度
        double arm_rz = BilliardMath::getVectorAngle(v_dir.x, v_dir.y) + YAW_OFFSET;
        double standoff = (BALL_D / 2.0) + 30.0; 
        double strike_x = bw.x - (v_dir.x / v_dist) * standoff;
        double strike_y = bw.y - (v_dir.y / v_dist) * standoff;

        Offset3D offset = BilliardMath::getTiltOffset(arm_rz, TILT_RY_DEG, MOVE_BACK_MM);

        double ready[6] = { strike_x + offset.x, strike_y + offset.y, SAFE_Z, 0.0, TILT_RY_DEG, arm_rz };
        double down[6]  = { strike_x + offset.x, strike_y + offset.y, STRIKE_Z + offset.z, 0.0, TILT_RY_DEG, arm_rz };

        cout << "\n\n--- 幾何決策面板 (核心鎖定 p1) ---" << endl;
        cout << "[分析] 軌跡夾角: " << angle_deg << " 度" << endl;
        cout << "[決策] 執行策略: " << strategy_name << endl;
        
        cout << "出發至預備點? (y:確認 / n:重算 / r:重拍): ";
        char confirm; cin >> confirm;
        if (confirm == 'r' || confirm == 'R') { 
            needCameraMove = true; 
            return false; 
        } 
        else if (confirm != 'y' && confirm != 'Y') {
            return false; 
        }

        // 先移動至中繼關節點位 (與拍照點 CAM_JOINT 相同)
        const double TRANSIT_JOINT[6] = {0.0, -33.564, 49.53, 0.0, -15.574, -90.0};
        cout << "[動作] 先移動至中繼關節點位..." << endl;
        robot.moveToAxis(TRANSIT_JOINT, true);
        Sleep(500);

        cout << "[動作] 以 PTP 方式直接移動至打擊點..." << endl;
        robot.moveToPosition(down, true);
        
        robot.setOverrideRatio(20); 
        robot.setToolNumber(1);  // 維持工具軸 1 座標系
        
        char return_confirm = 'n';
        while (return_confirm != 'y' && return_confirm != 'Y') {
            cout << "\a\n[定位確認] 已抵達打擊點！請確認筆尖與母球位置。輸入 'y' 返回拍照點: ";
            cin >> return_confirm;
        }

        cout << "[動作] 手臂返回拍照點..." << endl;
        robot.moveToAxis(CAM_JOINT, true);
        
        needCameraMove = false; // 已手動回拍照點，不需要再次觸發 moveToCameraPosition 的 Enter 提示
        return true;
    }
    return false;
}

void BilliardApp::runContourAlignment() {
    SocketClient alignClient;
    cout << "[視覺伺服] 正在連線至對齊伺服器" << flush;
    bool align_connected = false;
    
    for (int i = 0; i < 10; i++) {
        if (alignClient.connectToServer("127.0.0.1", ALIGN_PORT)) {
            align_connected = true;
            break;
        }
        cout << "." << flush;
        Sleep(1000); 
    }
    cout << endl;

    if (align_connected) {
        char alignBuf[512];
        bool align_done = false;
        
        robot.setToolNumber(1);
        robot.setOverrideRatio(5);  

        while (!align_done) {
            int r_bytes = alignClient.receiveData(alignBuf, sizeof(alignBuf) - 1);
            if (r_bytes > 0) {
                alignBuf[r_bytes] = '\0';
                double pixel_error = -9999.0;
                if (sscanf_s(alignBuf, "%lf", &pixel_error) == 1 && pixel_error > -9000.0) {
                    
                    cout << "\r[微調] 偏差: " << pixel_error << " 像素" << flush;

                    if (abs(pixel_error) <= 3.0) {
                        cout << "\n[微調] 達成對齊！停止微調。" << endl;
                        align_done = true;
                    } else {
                        double k_p = 0.05; 
                        double move_y = pixel_error * k_p;
                        
                        if (move_y > 10.0) move_y = 10.0;
                        if (move_y < -10.0) move_y = -10.0;

                        cout << " | 工具 Y 軸平移: " << move_y << " mm   " << flush;

                        robot.setToolNumber(1);
                        double rel_move[6] = {0.0, move_y, 0.0, 0.0, 0.0, 0.0};
                        robot.moveLinearRelative(rel_move);
                    }
                }
            } else { 
                break; 
            }
        }
        alignClient.closeConnection();
    } else {
        cout << "[警告] 無法連線至 align.py，跳過二次微調。" << endl;
    }
}
