#include "BilliardApp.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

#include "BilliardConfig.h"

using namespace std;

BilliardApp::BilliardApp() : needCameraMove(true) {}

bool BilliardApp::initialize() {
    if (!robot.connect(BilliardConfig::ARM_IP)) {
        cout << "[錯誤] 手臂連線失敗。" << endl;
        return false;
    }

    robot.setMotorState(1);
    robot.setOverrideRatio(BilliardConfig::NORMAL_SPEED_RATIO);
    robot.setToolNumber(BilliardConfig::TOOL_NUMBER);

    moveToCameraPosition();
    needCameraMove = false;

    cout << "[系統] 等待 Python 連線..." << endl;
    while (!visionClient.connectToServer(
        BilliardConfig::VISION_SERVER_IP,
        BilliardConfig::VISION_SERVER_PORT
    )) {
        Sleep(1000);
    }
    cout << "[系統] 與 Python 服務連線成功！" << endl;
    return true;
}

void BilliardApp::run() {
    while (true) {
        if (needCameraMove) {
            moveToCameraPosition();
            visionClient.flushBuffer();
            needCameraMove = false;
        }

        robot.setToolNumber(BilliardConfig::TOOL_NUMBER);

        string message;
        int bytes = visionClient.receiveLine(message);
        if (bytes > 0) {
            processVisionData(message);
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
    string confirm;
    getline(cin, confirm);

    cout << "[動作] 移動至拍照點..." << endl;
    robot.moveToAxis(BilliardConfig::CAMERA_JOINT.data());
    Sleep(BilliardConfig::CAMERA_SETTLE_MS);
}

bool BilliardApp::processVisionData(const std::string& dataString) {
    string error;
    TableState table;
    if (!visionParser.parse(dataString, table, error)) {
        cout << "\r[影像資料錯誤] " << error << "                  " << flush;
        return false;
    }

    TargetSelection target;
    if (!targetSelector.select(table, target, error)) {
        cout << "\r[狀態] " << error << "                  " << flush;
        return false;
    }

    cout << "\n[目標] 選擇 " << target.targetName
         << "，球袋 p" << target.pocketNumber
         << "（夾角: " << target.pocketAngleDeg << " 度）" << endl;

    ShotDecision shot = BilliardAlgorithm::decideShot(target);

    MotionPlan motion;
    if (!motionPlanner.createPlan(
        target.cueBall,
        shot.best_aim_target,
        BilliardConfig::PRODUCTION_MOTION,
        motion,
        error
    )) {
        cout << "[動作規劃錯誤] " << error << endl;
        return false;
    }

    cout << "\n--- 幾何決策面板 ---" << endl;
    cout << "[分析] 軌跡夾角: " << shot.angle_deg << " 度" << endl;
    cout << "[決策] 執行策略: " << shot.strategy_name << endl;
    cout << "[姿態] 手臂 RZ: " << motion.aimAngleDeg << " 度" << endl;

    cout << "出發至預備點? (y:確認 / n:重算 / r:重拍): ";
    char confirm;
    cin >> confirm;
    cin.ignore((numeric_limits<streamsize>::max)(), '\n');
    if (confirm == 'r' || confirm == 'R') {
        needCameraMove = true;
        return false;
    }
    if (confirm != 'y' && confirm != 'Y') {
        return false;
    }

    return executeMotionPlan(motion);
}

bool BilliardApp::executeMotionPlan(const MotionPlan& plan) {
    cout << "[Motion] Move to transit joint pose with PTP..." << endl;
    if (!requireMotionSuccess(
        "PTP to transit joint pose",
        robot.moveToAxis(plan.transitJoint.data(), true)
    )) {
        return false;
    }
    Sleep(BilliardConfig::TRANSIT_SETTLE_MS);

    const int toolNumber = robot.getCurrentToolNumber();
    const int baseNumber = robot.getCurrentBaseNumber();
    cout << "[Diagnostic] Tool=" << toolNumber
         << ", Base=" << baseNumber << endl;
    if (toolNumber < 0 || baseNumber < 0) {
        cout << "[Error] Unable to read the active Tool/Base." << endl;
        printAlarmCodes();
        return false;
    }
    if (toolNumber != BilliardConfig::TOOL_NUMBER) {
        cout << "[Error] Active Tool does not match configured Tool "
             << BilliardConfig::TOOL_NUMBER << "." << endl;
        printAlarmCodes();
        return false;
    }

    array<double, 6> transitPose;
    int sdkCode = -1;
    if (!robot.getCurrentPosition(transitPose, sdkCode)) {
        cout << "[Error] get_current_position failed after transit point. SDK code="
             << sdkCode << endl;
        printAlarmCodes();
        return false;
    }
    printPose("Current pose after transit", transitPose);
    printPose("Ready pose", plan.readyPose);
    printPose("Strike pose", plan.strikePose);

    if (!requireReachable("ready pose", plan.readyPose) ||
        !requireReachable("strike pose", plan.strikePose)) {
        return false;
    }

    cout << "[Motion] Move to ready pose with PTP..." << endl;
    if (!requireMotionSuccess(
        "PTP from transit to ready pose",
        robot.moveToPosition(plan.readyPose.data(), true)
    )) {
        return false;
    }

    array<double, 6> actualReadyPose;
    if (!robot.getCurrentPosition(actualReadyPose, sdkCode)) {
        cout << "[Warning] get_current_position failed at ready pose. SDK code="
             << sdkCode << endl;
        actualReadyPose = plan.readyPose;
        printAlarmCodes();
    }

    bool linearPathReachable = false;
    if (!robot.checkLinearPath(
        actualReadyPose,
        plan.strikePose,
        linearPathReachable,
        sdkCode
    )) {
        cout << "[Diagnostic] motion_check_lin failed. SDK code="
             << sdkCode << ". PTP will still be attempted." << endl;
        printAlarmCodes();
    } else {
        cout << "[Diagnostic] Ready-to-strike LIN path: "
             << (linearPathReachable ? "reachable" : "not reachable")
             << ". This run uses PTP." << endl;
        if (!linearPathReachable) {
            printAlarmCodes();
        }
    }

    cout << "[Motion] Move from ready pose to strike pose with PTP..." << endl;
    if (!requireMotionSuccess(
        "PTP from ready to strike pose",
        robot.moveToPosition(plan.strikePose.data(), true)
    )) {
        return false;
    }

    robot.setOverrideRatio(BilliardConfig::NORMAL_SPEED_RATIO);
    robot.setToolNumber(BilliardConfig::TOOL_NUMBER);

    char returnConfirm = 'n';
    while (returnConfirm != 'y' && returnConfirm != 'Y') {
        cout << "\a\n[定位確認] 已抵達打擊點！請確認筆尖與母球位置。輸入 'y' 返回拍照點: ";
        cin >> returnConfirm;
    }
    cin.ignore((numeric_limits<streamsize>::max)(), '\n');

    cout << "[Motion] Return from strike pose to ready pose with PTP..." << endl;
    if (!requireMotionSuccess(
        "PTP from strike to ready pose",
        robot.moveToPosition(plan.readyPose.data(), true)
    )) {
        return false;
    }

    cout << "[Motion] Return to camera joint pose..." << endl;
    if (!requireMotionSuccess(
        "PTP to camera joint pose",
        robot.moveToAxis(BilliardConfig::CAMERA_JOINT.data(), true)
    )) {
        return false;
    }
    needCameraMove = false;
    return true;
}

bool BilliardApp::requireReachable(
    const string& pointName,
    const array<double, 6>& pose
) {
    bool reachable = false;
    int sdkCode = -1;
    if (!robot.checkReachable(pose, reachable, sdkCode)) {
        cout << "[Error] motion_reachable failed for " << pointName
             << ". SDK code=" << sdkCode << endl;
        printAlarmCodes();
        return false;
    }
    cout << "[Diagnostic] " << pointName << ": "
         << (reachable ? "reachable" : "not reachable") << endl;
    if (!reachable) {
        printAlarmCodes();
    }
    return reachable;
}

bool BilliardApp::requireMotionSuccess(
    const string& stepName,
    const MotionResult& result
) {
    if (result.success) {
        return true;
    }
    cout << "[Error] " << stepName << " failed. SDK code=" << result.sdkCode
         << ", motion state=" << result.finalMotionState;
    if (result.timedOut) {
        cout << ", timeout=" << BilliardConfig::MOTION_TIMEOUT_MS
             << " ms, motion_abort SDK code=" << result.abortSdkCode;
    }
    cout << endl;
    printAlarmCodes();
    return false;
}

void BilliardApp::printPose(
    const string& label,
    const array<double, 6>& pose
) const {
    cout << fixed << setprecision(3)
         << "[Pose] " << label
         << ": X=" << pose[0]
         << ", Y=" << pose[1]
         << ", Z=" << pose[2]
         << ", RX=" << pose[3]
         << ", RY=" << pose[4]
         << ", RZ=" << pose[5] << defaultfloat << endl;
}

void BilliardApp::printAlarmCodes() const {
    int sdkCode = -1;
    vector<uint64_t> alarms = robot.getAlarmCodes(sdkCode);
    if (sdkCode != 0) {
        cout << "[Alarm] get_alarm_code failed. SDK code=" << sdkCode << endl;
        return;
    }
    if (alarms.empty()) {
        cout << "[Alarm] No active alarm code reported." << endl;
        return;
    }
    cout << "[Alarm] Active codes:";
    for (size_t index = 0; index < alarms.size(); ++index) {
        cout << " 0x" << hex << uppercase << alarms[index];
    }
    cout << dec << nouppercase << endl;
}

void BilliardApp::runContourAlignment() {
    SocketClient alignClient;
    cout << "[視覺伺服] 正在連線至對齊伺服器" << flush;
    bool alignConnected = false;

    for (int attempt = 0; attempt < 10; ++attempt) {
        if (alignClient.connectToServer(
            BilliardConfig::VISION_SERVER_IP,
            BilliardConfig::ALIGN_SERVER_PORT
        )) {
            alignConnected = true;
            break;
        }
        cout << "." << flush;
        Sleep(1000);
    }
    cout << endl;

    if (!alignConnected) {
        cout << "[警告] 無法連線至 align.py，跳過二次微調。" << endl;
        return;
    }

    robot.setToolNumber(BilliardConfig::TOOL_NUMBER);
    robot.setOverrideRatio(BilliardConfig::ALIGN_SPEED_RATIO);

    string message;
    while (alignClient.receiveLine(message) > 0) {
        double pixelError;
        try {
            pixelError = stod(message);
        } catch (const std::exception&) {
            continue;
        }

        if (pixelError <= -9000.0) {
            continue;
        }

        cout << "\r[微調] 偏差: " << pixelError << " 像素" << flush;
        if (abs(pixelError) <= BilliardConfig::ALIGN_TOLERANCE_PX) {
            cout << "\n[微調] 達成對齊！停止微調。" << endl;
            break;
        }

        double moveY = pixelError * BilliardConfig::ALIGN_KP;
        if (moveY > BilliardConfig::ALIGN_MAX_STEP_MM) {
            moveY = BilliardConfig::ALIGN_MAX_STEP_MM;
        }
        if (moveY < -BilliardConfig::ALIGN_MAX_STEP_MM) {
            moveY = -BilliardConfig::ALIGN_MAX_STEP_MM;
        }

        cout << " | 工具 Y 軸平移: " << moveY << " mm   " << flush;
        double relativeMove[6] = {0.0, moveY, 0.0, 0.0, 0.0, 0.0};
        robot.moveLinearRelative(relativeMove);
    }

    alignClient.closeConnection();
}
