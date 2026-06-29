#include "BilliardApp.h"
#include "BilliardPhysics.h"
#include "MathUtils.h"
#include "Point.h"
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
    robot.setOverrideRatio(50);

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

            if (shotCount == 1) {
                if (handleFirstShot()) {
                    shotCount = 2;
                    needCameraMove = true;
                    continue;
                }
                shotCount = 2;
            }
            
            if (shotCount == 2) {
                if (handleSecondShot()) {
                    shotCount = 3;
                    yoloClient.flushBuffer();
                    needCameraMove = false;
                    continue;
                }
                shotCount = 3;
                yoloClient.flushBuffer();
                needCameraMove = false;
            }
            
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
    cout << "\n[動作] 移動至拍照點..." << endl;
    robot.moveToAxis(CAM_JOINT);
    Sleep(800);
}

bool BilliardApp::handleFirstShot() {
    cout << "\n[系統] 偵測為第 1 桿。是否移動至開球點位 (y: 是 / n: 否，跳過)? ";
    char do_break;
    cin >> do_break;
    if (do_break == 'y' || do_break == 'Y') {
        cout << "[動作] 移動至開球點..." << endl;
        robot.moveToAxis(BREAK_JOINT);
        
        cout << "\a\n[安全鎖] 已抵達開球點！輸入 'yes' 執行擊打並返回: ";
        string confirm_break;
        cin >> confirm_break;
        while (confirm_break != "yes" && confirm_break != "YES") {
            cout << "請完整輸入 'yes' 以執行開球: ";
            cin >> confirm_break;
        }
        
        cout << "[動作] 第一桿氣壓缸擊發！" << endl;
        robot.firePneumatic(PNEUMATIC_DO, TASK2_EXTEND_MS);
        
        cout << "[動作] 開球完畢，返回拍照點..." << endl;
        moveToCameraPosition();
        return true;
    }
    return false;
}

bool BilliardApp::handleSecondShot() {
    cout << "\n[系統] 偵測為第 2 桿。是否移動至第 2 桿預設點位 (y: 是 / n: 否，跳過)? ";
    char do_second;
    cin >> do_second;
    if (do_second == 'y' || do_second == 'Y') {
        cout << "[動作] 移動至第 2 桿預設點..." << endl;
        robot.moveToAxis(SECOND_JOINT);
        
        cout << "\a\n[安全鎖] 已抵達第 2 桿點位！輸入 'yes' 執行擊打並返回: ";
        string confirm_second;
        cin >> confirm_second;
        while (confirm_second != "yes" && confirm_second != "YES") {
            cout << "請完整輸入 'yes' 以執行擊打: ";
            cin >> confirm_second;
        }
        
        cout << "[動作] 第 2 桿氣壓缸擊發！" << endl;
        robot.firePneumatic(PNEUMATIC_DO, TASK2_EXTEND_MS);
        
        cout << "[動作] 第 2 桿擊打完畢，返回拍照點..." << endl;
        moveToCameraPosition();
        return true;
    }
    return false;
}

bool BilliardApp::processVisionData(char* dataString) {
    double b1x, b1y, b2x, b2y, b3x, b3y, bwx, bwy, p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, p5x, p5y, p6x, p6y;

    if (sscanf_s(dataString, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
        &b1x, &b1y, &b2x, &b2y, &b3x, &b3y, &bwx, &bwy, &p1x, &p1y, &p2x, &p2y, &p3x, &p3y, &p4x, &p4y, &p5x, &p5y, &p6x, &p6y) == 20) {
        
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

        Point destination = {-9999.0, -9999.0};
        if (p1x > -9000.0) {
            destination = BilliardMath::applyCameraCompensation({ p1x, p1y });
        } else {
            cout << "\r[錯誤] 核心錯誤：缺少 1 號球袋 (p1) 座標，拒絕執行擊打...  " << flush;
            return false;
        }

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

        Point ghost_direct = BilliardPhysics::getGhostBall(destination, target_arm, BALL_D);
        bool direct_path_blocked = false;

        for (const auto& obs : obs_list) {
            if (BilliardPhysics::isPathBlocked(bw, ghost_direct, obs, BALL_D)) direct_path_blocked = true;      
            if (BilliardPhysics::isPathBlocked(target_arm, destination, obs, BALL_D)) direct_path_blocked = true; 
        }
        if (BilliardPhysics::isPathBlocked(target_arm, destination, bw, BALL_D)) direct_path_blocked = true;      

        Vector2D vec1 = BilliardMath::getVector(target_arm, destination);
        Vector2D vec2 = BilliardMath::getVector(bw, target_arm);

        // 使用 MathUtils 函式庫計算向量夾角
        double angle_deg = BilliardMath::getAngleBetweenVectors(vec1.x, vec1.y, vec2.x, vec2.y);

        Point best_aim_target;
        string strategy_name = "";

        if (angle_deg > 90.0 || direct_path_blocked) {
            if (direct_path_blocked) {
                cout << "\n[防撞提示] 偵測到直擊路徑受阻，自動切換至顆星解球模式。" << endl;
            }
            
            Point mirrored_pocket = BilliardPhysics::getSlantedBankTarget(destination, rail_A, rail_B);
            Point mirrored_target = BilliardPhysics::getSlantedBankTarget(target_arm, rail_A, rail_B);
            Point ghost_bank = BilliardPhysics::getGhostBall(mirrored_pocket, mirrored_target, BALL_D);

            Point pt_wall;
            bool bank_route_safe = false;

            if (BilliardPhysics::getIntersection(bw, ghost_bank, rail_A, rail_B, pt_wall)) {
                bool current_bank_blocked = false;

                for (const auto& obs : obs_list) {
                    if (BilliardPhysics::isPathBlocked(bw, pt_wall, obs, BALL_D)) current_bank_blocked = true;
                    if (BilliardPhysics::isPathBlocked(pt_wall, target_arm, obs, BALL_D)) current_bank_blocked = true;
                    if (BilliardPhysics::isPathBlocked(target_arm, destination, obs, BALL_D)) current_bank_blocked = true;
                }
                if (BilliardPhysics::isPathBlocked(target_arm, destination, bw, BALL_D)) current_bank_blocked = true;

                if (!current_bank_blocked) bank_route_safe = true;
            }

            best_aim_target = ghost_bank;
            if (bank_route_safe) {
                strategy_name = "雙鏡射顆星擊球 (洞2-洞3牆壁 -> p1)";
            } else {
                strategy_name = "雙鏡射顆星擊球 (安全路徑受阻，強制開火洞2-洞3牆壁)";
            }
        } 
        else {
            strategy_name = "直線直擊 (Direct Shot -> p1)";
            best_aim_target = ghost_direct;
        }

        double MAX_REACH_RADIUS = 850.0; 
        
        // 使用 MathUtils 函式庫計算與原點距離
        double target_reach = BilliardMath::getLength(best_aim_target.x, best_aim_target.y);
        
        if (target_reach > MAX_REACH_RADIUS) {
            cout << "\n[警告] 計算出之擊球點超出工作半徑 (" << target_reach << " > " << MAX_REACH_RADIUS << " mm)！" << endl;
            cout << "[降級] 放棄當前路徑，強制切換為直線直擊策略..." << endl;
            
            best_aim_target = ghost_direct; 
            strategy_name = "[降級] 強制直線直擊";
        }

        Vector2D v_dir = BilliardMath::getVector(bw, best_aim_target);
        
        // 使用 MathUtils 函式庫計算母球到最佳擊球點的距離
        double v_dist = BilliardMath::getLength(v_dir.x, v_dir.y);
        if (v_dist < 5.0) return false; 
        
        // 使用 MathUtils 函式庫計算向量平面角度
        double arm_rz = BilliardMath::getVectorAngle(v_dir.x, v_dir.y) + YAW_OFFSET;
        double standoff = (BALL_D / 2.0) + 10.0; 
        double strike_x = bw.x - (v_dir.x / v_dist) * standoff;
        double strike_y = bw.y - (v_dir.y / v_dist) * standoff;

        Offset3D offset = BilliardMath::getTiltOffset(arm_rz, TILT_RY_DEG, MOVE_BACK_MM);

        double ready[6] = { strike_x + offset.x, strike_y + offset.y, SAFE_Z, 180, TILT_RY_DEG, arm_rz };
        double down[6]  = { strike_x + offset.x, strike_y + offset.y, STRIKE_Z + offset.z, 180, TILT_RY_DEG, arm_rz };

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

        cout << "[動作] 平移至預備點..." << endl;
        robot.moveToPosition(ready); 
        
        cout << "[動作] 直線下降至傾斜擊球高度..." << endl;
        robot.moveLinearTo(down);
        
        robot.setOverrideRatio(50); 
        robot.setToolNumber(0);
        
        char strike_confirm = 'n';
        while (strike_confirm != 'y' && strike_confirm != 'Y') {
            cout << "\a\n[安全鎖] 準備就緒！輸入 'y' 執行擊打並回歸: ";
            cin >> strike_confirm;
        }

        cout << "[動作] 氣壓缸出桿擊打！" << endl;
        robot.firePneumatic(PNEUMATIC_DO, TASK2_EXTEND_MS); 
        
        cout << "[動作] 抬升回歸..." << endl;
        robot.moveLinearTo(ready); 
        
        needCameraMove = true; 
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
