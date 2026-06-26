#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include <iostream>
#include <winsock2.h>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <conio.h>
#include "HRSDK.h" 

#pragma comment(lib, "HRSDK.lib")
#pragma comment(lib, "ws2_32.lib")

using namespace std;

const double PI = 3.14159265358979323846;

// ==========================================
// 1. 基礎結構體宣告
// ==========================================
struct Point { double x, y; };
struct Rail { Point pA, pB; string name; };

// ==========================================
// 2. 幾何運算與物理核心類別 (BilliardPhysics)
// ==========================================
class BilliardPhysics {
public:
    static bool isPathBlocked(Point start, Point end, Point obs, double ball_d) {
        double line_dx = end.x - start.x;
        double line_dy = end.y - start.y;
        double line_len_sq = line_dx * line_dx + line_dy * line_dy;
        if (line_len_sq < 0.001) return false;

        double t = ((obs.x - start.x) * line_dx + (obs.y - start.y) * line_dy) / line_len_sq;
        double dist = 0.0;

        if (t < 0.0) {
            dist = sqrt(pow(obs.x - start.x, 2) + pow(obs.y - start.y, 2));
        } else if (t > 1.0) {
            dist = sqrt(pow(obs.x - end.x, 2) + pow(obs.y - end.y, 2));
        } else {
            Point proj = { start.x + t * line_dx, start.y + t * line_dy };
            dist = sqrt(pow(obs.x - proj.x, 2) + pow(obs.y - proj.y, 2));
        }
        return (dist < ball_d); 
    }

    static Point getPerpendicularTarget(Point base, Point edgeA, Point edgeB, double backward_dist) {
        double dx = edgeB.x - edgeA.x;
        double dy = edgeB.y - edgeA.y;
        double len = sqrt(dx * dx + dy * dy);
        if (len < 0.001) return base;
        double ux = dx / len, uy = dy / len;
        return { base.x - uy * backward_dist, base.y + ux * backward_dist };
    }

    static Point getGhostBall(Point destination, Point target_ball, double ball_d) {
        double dx = target_ball.x - destination.x;
        double dy = target_ball.y - destination.y;
        double dist = sqrt(dx * dx + dy * dy);
        Point ghost = target_ball;
        if (dist > 0.001) {
            ghost.x += (dx / dist) * ball_d;
            ghost.y += (dy / dist) * ball_d;
        }
        return ghost;
    }

    static Point getSlantedBankTarget(Point I, Point p1, Point p2) {
        double line_dx = p2.x - p1.x;
        double line_dy = p2.y - p1.y;
        double line_length_sq = line_dx * line_dx + line_dy * line_dy;
        if (line_length_sq < 0.001) return I;

        double t = (((I.x - p1.x) * line_dx) + ((I.y - p1.y) * line_dy)) / line_length_sq;
        Point proj = { p1.x + t * line_dx, p1.y + t * line_dy };
        return { proj.x * 2 - I.x, proj.y * 2 - I.y };
    }

    static bool getIntersection(Point ray_start, Point ray_target, Point segA, Point segB, Point &out_intersect) {
        double den = (ray_start.x - ray_target.x) * (segA.y - segB.y) - (ray_start.y - ray_target.y) * (segA.x - segB.x);
        if (abs(den) < 0.001) return false;

        double t_ray = ((ray_start.x - segA.x) * (segA.y - segB.y) - (ray_start.y - segA.y) * (segA.x - segB.x)) / den;
        double u_seg = -((ray_start.x - ray_target.x) * (ray_start.y - segA.y) - (ray_start.y - ray_target.y) * (ray_start.x - segA.x)) / den;

        if (t_ray >= 0.0 && u_seg >= 0.0 && u_seg <= 1.0) {
            out_intersect = { ray_start.x + t_ray * (ray_target.x - ray_start.x), ray_start.y + t_ray * (ray_target.y - ray_start.y) };
            return true;
        }
        return false;
    }
};

// HRSDK 連線所需的回標函數
void __stdcall arm_callback(uint16_t rob_idx, uint16_t status, unsigned short* msg, int msg_len) {}

// ==========================================
// 3. 上銀機械手臂控制類別 (RobotController)
// ==========================================
class RobotController {
private:
    int id;
    bool connected;
public:
    RobotController() : id(-1), connected(false) {}

    ~RobotController() {
        disconnect();
    }

    bool connect(const string& ip) {
        id = open_connection(const_cast<char*>(ip.c_str()), 1, arm_callback);
        if (id < 0) {
            connected = false;
            return false;
        }
        connected = true;
        clear_alarm(id);
        Sleep(200);
        return true;
    }

    void disconnect() {
        if (connected && id >= 0) {
            set_motor_state(id, 0);
            close_connection(id);
            connected = false;
            id = -1;
        }
    }

    int getId() const { return id; }
    bool isConnected() const { return connected; }

    void setMotorState(int state) {
        if (connected) set_motor_state(id, state);
    }

    void setOverrideRatio(int ratio) {
        if (connected) set_override_ratio(id, ratio);
    }

    void setToolNumber(int tool_num) {
        if (connected) set_tool_number(id, tool_num);
    }

    int getMotionState() {
        if (connected) return get_motion_state(id);
        return -1;
    }

    void moveToAxis(const double joint[6], bool wait = true) {
        if (!connected) return;
        ptp_axis(id, 0, const_cast<double*>(joint));
        if (wait) {
            while (get_motion_state(id) != 1) Sleep(50);
        }
    }

    void moveToPosition(const double pos[6], bool wait = true) {
        if (!connected) return;
        ptp_pos(id, 0, const_cast<double*>(pos));
        if (wait) {
            while (get_motion_state(id) != 1) Sleep(50);
        }
    }

    void moveLinearTo(const double pos[6], bool wait = true) {
        if (!connected) return;
        lin_pos(id, 0, 0, const_cast<double*>(pos));
        if (wait) {
            while (get_motion_state(id) != 1) Sleep(50);
        }
    }

    void moveLinearRelative(const double rel[6], bool wait = true) {
        if (!connected) return;
        lin_rel_pos(id, 0, 0.0, const_cast<double*>(rel));
        if (wait) {
            while (get_motion_state(id) != 1) Sleep(20);
        }
    }

    void setDigitalOutput(int index, bool state) {
        if (connected) set_digital_output(id, index, state);
    }

    void firePneumatic(int index, DWORD duration_ms) {
        if (!connected) return;
        set_digital_output(id, index, true);
        Sleep(duration_ms);
        set_digital_output(id, index, false);
    }
};

// ==========================================
// 4. Socket 通訊用戶端類別 (SocketClient)
// ==========================================
class SocketClient {
private:
    SOCKET clientSocket;
    bool connected;
public:
    SocketClient() : clientSocket(INVALID_SOCKET), connected(false) {}

    ~SocketClient() {
        closeConnection();
    }

    bool connectToServer(const string& ip, int port) {
        clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (clientSocket == INVALID_SOCKET) return false;

        sockaddr_in serverAddr = { AF_INET, htons(port) };
        serverAddr.sin_addr.s_addr = inet_addr(ip.c_str());

        if (connect(clientSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
            closesocket(clientSocket);
            clientSocket = INVALID_SOCKET;
            connected = false;
            return false;
        }
        connected = true;
        return true;
    }

    void closeConnection() {
        if (clientSocket != INVALID_SOCKET) {
            closesocket(clientSocket);
            clientSocket = INVALID_SOCKET;
        }
        connected = false;
    }

    bool isConnected() const { return connected; }

    int receiveData(char* buf, int max_len) {
        if (!connected) return -1;
        return recv(clientSocket, buf, max_len, 0);
    }

    void flushBuffer() {
        if (!connected) return;
        char dummy[1024];
        u_long bytes_available;
        ioctlsocket(clientSocket, FIONREAD, &bytes_available);
        while (bytes_available > 0) {
            recv(clientSocket, dummy, sizeof(dummy) - 1, 0);
            ioctlsocket(clientSocket, FIONREAD, &bytes_available);
        }
    }
};

// ==========================================
// 5. 撞球系統整合應用程式類別 (BilliardApp)
// ==========================================
class BilliardApp {
private:
    // 系統參數區
    const string ARM_IP = "192.168.0.1";
    const int PYTHON_PORT = 12345;
    const int ALIGN_PORT = 12346;
    
    const int PNEUMATIC_DO = 1;
    const double BALL_D = 49.52;
    const double STRIKE_Z = -294.960;
    const double SAFE_Z = 0.0;
    const double YAW_OFFSET = 0.0;
    
    const double TILT_RY_DEG = 10.0;
    const double MOVE_BACK_MM = 20.0;
    
    // 拍照與固定桿點位
    const double CAM_JOINT[6] = {2.173, -19.536, 74.059, 0.0, -44.365, 88.147};
    const double BREAK_JOINT[6] = {-2.599, -87.736, 48.629, 3.794, -50.23, 103.847};
    const double SECOND_JOINT[6] = {63.243, -93.544, 63.324, 0.000, -67.246, 36.322};
    
    const DWORD TASK2_EXTEND_MS = 150;
    
    RobotController robot;
    SocketClient yoloClient;
    int shotCount;
    bool needCameraMove;

public:
    BilliardApp() : shotCount(1), needCameraMove(true) {}

    bool initialize() {
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

    void run() {
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

private:
    void moveToCameraPosition() {
        cout << "\n[動作] 移動至拍照點..." << endl;
        robot.moveToAxis(CAM_JOINT);
        Sleep(800);
    }

    bool handleFirstShot() {
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

    bool handleSecondShot() {
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

    bool processVisionData(char* dataString) {
        double b1x, b1y, b2x, b2y, b3x, b3y, bwx, bwy, p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, p5x, p5y, p6x, p6y;

        if (sscanf_s(dataString, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &b1x, &b1y, &b2x, &b2y, &b3x, &b3y, &bwx, &bwy, &p1x, &p1y, &p2x, &p2y, &p3x, &p3y, &p4x, &p4y, &p5x, &p5y, &p6x, &p6y) == 20) {
            
            if (bwx < -9000.0) { 
                cout << "\r[狀態] 尋找母球中...                  " << flush; 
                return false; 
            }
            Point bw = { bwx, bwy };

            Point target_arm = {-9999.0, -9999.0}, destination = {-9999.0, -9999.0};
            string target_name = "";

            if (b1x > -9000.0) { target_arm = {b1x, b1y}; target_name = "1號球"; }
            else if (b2x > -9000.0) { target_arm = {b2x, b2y}; target_name = "2號球"; }
            else if (b3x > -9000.0) { target_arm = {b3x, b3y}; target_name = "3號球"; }

            if (p1x > -9000.0) {
                destination = { p1x, p1y };
            } else {
                cout << "\r[錯誤] 核心錯誤：缺少 1 號球袋 (p1) 座標，拒絕執行擊打...  " << flush;
                return false;
            }

            if (p2x < -9000.0 || p3x < -9000.0) {
                cout << "\r[錯誤] 核心錯誤：缺少 2 號或 3 號球袋座標，無法建立顆星牆壁...  " << flush;
                return false;
            }
            Point rail_A = { p2x, p2y };
            Point rail_B = { p3x, p3y };

            vector<Point> obs_list;
            if (b1x > -9000.0 && target_name != "1號球") obs_list.push_back({b1x, b1y});
            if (b2x > -9000.0 && target_name != "2號球") obs_list.push_back({b2x, b2y});
            if (b3x > -9000.0 && target_name != "3號球") obs_list.push_back({b3x, b3y});

            Point ghost_direct = BilliardPhysics::getGhostBall(destination, target_arm, BALL_D);
            bool direct_path_blocked = false;

            for (const auto& obs : obs_list) {
                if (BilliardPhysics::isPathBlocked(bw, ghost_direct, obs, BALL_D)) direct_path_blocked = true;      
                if (BilliardPhysics::isPathBlocked(target_arm, destination, obs, BALL_D)) direct_path_blocked = true; 
            }
            if (BilliardPhysics::isPathBlocked(target_arm, destination, bw, BALL_D)) direct_path_blocked = true;      

            double vec1_x = destination.x - target_arm.x; 
            double vec1_y = destination.y - target_arm.y;
            double vec2_x = target_arm.x - bw.x;          
            double vec2_y = target_arm.y - bw.y;
            double len1 = sqrt(vec1_x * vec1_x + vec1_y * vec1_y);
            double len2 = sqrt(vec2_x * vec2_x + vec2_y * vec2_y);
            double angle_deg = 0.0;
            if (len1 > 0.001 && len2 > 0.001) {
                double cos_theta = (vec1_x * vec2_x + vec1_y * vec2_y) / (len1 * len2);
                if (cos_theta > 1.0) cos_theta = 1.0;
                if (cos_theta < -1.0) cos_theta = -1.0;
                angle_deg = acos(cos_theta) * 180.0 / PI;
            }

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
            double target_reach = sqrt(best_aim_target.x * best_aim_target.x + best_aim_target.y * best_aim_target.y);
            
            if (target_reach > MAX_REACH_RADIUS) {
                cout << "\n[警告] 計算出之擊球點超出工作半徑 (" << target_reach << " > " << MAX_REACH_RADIUS << " mm)！" << endl;
                cout << "[降級] 放棄當前路徑，強制切換為直線直擊策略..." << endl;
                
                best_aim_target = ghost_direct; 
                strategy_name = "[降級] 強制直線直擊";
            }

            double v_dx = best_aim_target.x - bw.x;
            double v_dy = best_aim_target.y - bw.y;
            double v_dist = sqrt(v_dx * v_dx + v_dy * v_dy);
            if (v_dist < 5.0) return false; 
            
            double arm_rz = (atan2(v_dy, v_dx) * 180.0 / PI) + YAW_OFFSET;
            double standoff = (BALL_D / 2.0) + 10.0; 
            double strike_x = bw.x - (v_dx / v_dist) * standoff;
            double strike_y = bw.y - (v_dy / v_dist) * standoff;

            double rad_rz = arm_rz * PI / 180.0;
            double rad_tilt = TILT_RY_DEG * PI / 180.0;

            double delta_x = -MOVE_BACK_MM * cos(rad_tilt) * cos(rad_rz);
            double delta_y = -MOVE_BACK_MM * cos(rad_tilt) * sin(rad_rz);
            double delta_z =  MOVE_BACK_MM * sin(rad_tilt); 

            double ready[6] = { strike_x + delta_x, strike_y + delta_y, SAFE_Z, 180, TILT_RY_DEG, arm_rz };
            double down[6]  = { strike_x + delta_x, strike_y + delta_y, STRIKE_Z + delta_z, 180, TILT_RY_DEG, arm_rz };

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
            
            // 執行對齊
            runContourAlignment();

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

    void runContourAlignment() {
        cout << "\n[視覺伺服] 啟動 align.py 進行相機微調對齊..." << endl;
        system("start cmd /k C:\\Users\\10003\\miniconda3\\envs\\yolo-env1\\python.exe align.py"); 
        
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
};

// ==========================================
// 6. 主進入點
// ==========================================
int main() {
    setlocale(LC_ALL, "zh_TW.UTF-8");
    cout << "--- 撞球 AI 視覺伺服系統 (自動直擊與顆星複合版 OOP) ---" << endl;

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cout << "[錯誤] WSAStartup 失敗。" << endl;
        return -1;
    }

    BilliardApp app;
    if (app.initialize()) {
        app.run();
    }

    WSACleanup();
    return 0;
}