// 低速、人工監看的no-fire走位驗證程式。
//
// 跟正式BilliardApp::run()的H/P流程共用同一套鍵盤edge-gate偵測，走的階段
// 也完全對應正式流程（read-only確認→硬體準備→準備姿態→CameraPose→連線
// Python視覺→收三幀→Phase1選球→P2-01算擊球姿態→safeApproachPose→
// strikeReadyPose），但整個檔案裡完全不出現任何一次
// setDigitalOutput／pulseExtend／pulseRetract之類的氣動呼叫——不是用if
// 擋住，是程式碼裡根本沒有那幾行，從原始碼層級就不可能真的擊球。
//
// H：從頭跑一次到strikeReadyPose，停在那裡等下一個按鍵。
// 停在strikeReadyPose時再按H：回CameraPose重新拍照、重新規劃、重新走一次。
// P：（任何時候）PTP回準備姿態、等待下一個H。
//
// 完全獨立於BilliardApp.cpp／main.cpp，兩邊都沒有被修改，正式打球流程不
// 受影響。

#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <winsock2.h>

#include "Algorithm.h"
#include "BilliardApp.h"
#include "BilliardConfig.h"
#include "MotionPlanner.h"
#include "RobotController.h"
#include "SocketClient.h"
#include "TableState.h"
#include "VisionDataParser.h"

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "user32.lib")

using namespace std;

namespace {

void printPose(const char* label, const RobotPoseABC& pose)
{
    cout << "    " << label << ": X=" << pose.x << " Y=" << pose.y
         << " Z=" << pose.z << " A=" << pose.a << " B=" << pose.b
         << " C=" << pose.c << endl;
}

void printJoints(const char* label, const array<double, 6>& joints)
{
    cout << "    " << label << ": (" << joints[0] << ", " << joints[1] << ", "
         << joints[2] << ", " << joints[3] << ", " << joints[4] << ", "
         << joints[5] << ")" << endl;
}

const char* adapterStatusName(RobotAdapterStatus status)
{
    switch (status) {
    case RobotAdapterStatus::Success: return "Success";
    case RobotAdapterStatus::ConfigurationMissing: return "ConfigurationMissing";
    case RobotAdapterStatus::InvalidConfiguration: return "InvalidConfiguration";
    case RobotAdapterStatus::Unauthorized: return "Unauthorized";
    case RobotAdapterStatus::NotConnected: return "NotConnected";
    case RobotAdapterStatus::SdkFailure: return "SdkFailure";
    case RobotAdapterStatus::NotReachable: return "NotReachable";
    case RobotAdapterStatus::NotStopped: return "NotStopped";
    case RobotAdapterStatus::UnknownUnsafe: return "UnknownUnsafe";
    }
    return "?";
}

void printFailure(const char* step, RobotAdapterStatus status, int sdkCode)
{
    cout << "[失敗] " << step << "：" << adapterStatusName(status)
         << "（sdkCode=" << sdkCode << "）" << endl;
}

// 跟正式BilliardApp::confirmRobotReadyReadOnly完全對應：純讀取確認，
// 不寫入任何硬體狀態。
[[nodiscard]] bool confirmReadyReadOnly(
    RobotController& robot,
    const optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    cout << "[確認] Robot已連線？已停止？DO1/DO2確認OFF？...";
    if (!robot.isConnected()) {
        cout << "失敗（未連線）" << endl;
        return false;
    }
    const RobotAdapterResult stopped = robot.confirmStopped();
    if (!stopped.succeeded()) {
        cout << (stopped.status == RobotAdapterStatus::UnknownUnsafe
                     ? "UnknownUnsafe（無法確認是否已停止）"
                     : "失敗（尚未停止）")
             << endl;
        return false;
    }
    const RobotBoolAdapterResult doOff = robot.confirmPneumaticOutputsOff(config);
    if (doOff.status != RobotAdapterStatus::Success || !doOff.value || !*doOff.value) {
        cout << (doOff.status == RobotAdapterStatus::UnknownUnsafe
                     ? "UnknownUnsafe（DO讀值異常）"
                     : "失敗（DO1/DO2非確認OFF）")
             << endl;
        return false;
    }
    cout << "OK" << endl;
    return true;
}

// 跟正式BilliardApp::prepareRobotHardwareForMotion完全對應：只能在
// confirmReadyReadOnly通過後呼叫。
[[nodiscard]] bool prepareHardware(
    RobotController& robot,
    const optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    cout << "[硬體準備] establishSafeOutputsOff -> clearAlarm -> Tool1/Base0 -> "
         << "馬達on -> 速度比例" << BilliardConfig::NORMAL_SPEED_RATIO << "%..." ;
    if (const auto r = robot.establishSafeOutputsOff(config); !r.succeeded()) {
        cout << "失敗（establishSafeOutputsOff, " << adapterStatusName(r.status) << ")" << endl;
        return false;
    }
    if (const auto r = robot.clearAlarm(); !r.succeeded()) {
        cout << "失敗（clearAlarm, " << adapterStatusName(r.status) << ")" << endl;
        return false;
    }
    if (const auto r = robot.activateConfiguredToolAndBase(config); !r.succeeded()) {
        cout << "失敗（activateConfiguredToolAndBase, " << adapterStatusName(r.status) << ")" << endl;
        return false;
    }
    if (const auto r = robot.setMotorState(1); !r.succeeded()) {
        cout << "失敗（setMotorState, " << adapterStatusName(r.status) << ")" << endl;
        return false;
    }
    if (const auto r = robot.setOverrideRatio(BilliardConfig::NORMAL_SPEED_RATIO); !r.succeeded()) {
        cout << "失敗（setOverrideRatio, " << adapterStatusName(r.status) << ")" << endl;
        return false;
    }
    cout << "OK" << endl;
    return true;
}

// BilliardApp.cpp裡的同名函式是該檔案私有（匿名namespace），沒有匯出，
// 這裡照抄一份（H/P按鍵讀取邏輯本身很短，直接使用Windows Console API，
// 不牽涉任何打球邏輯）。
ConsoleKeyPoll productionConsoleKeyPoll()
{
    return [] {
        std::vector<ConsoleKeyEvent> events;
        HANDLE stdIn = GetStdHandle(STD_INPUT_HANDLE);
        if (stdIn == INVALID_HANDLE_VALUE || stdIn == nullptr) return events;
        DWORD pending = 0;
        if (!GetNumberOfConsoleInputEvents(stdIn, &pending) || pending == 0) {
            return events;
        }
        std::vector<INPUT_RECORD> records(pending);
        DWORD read = 0;
        if (!ReadConsoleInputW(stdIn, records.data(), pending, &read)) {
            return events;
        }
        for (DWORD i = 0; i < read; ++i) {
            if (records[i].EventType != KEY_EVENT) continue;
            const KEY_EVENT_RECORD& key = records[i].Event.KeyEvent;
            if (key.wVirtualKeyCode == 'H') {
                events.push_back({ConsoleKey::H, key.bKeyDown != 0});
            } else if (key.wVirtualKeyCode == 'P') {
                events.push_back({ConsoleKey::P, key.bKeyDown != 0});
            }
        }
        return events;
    };
}

ConsoleKeyDownQuery productionConsoleKeyDownQuery()
{
    return [](ConsoleKey key) {
        const int virtualKey = key == ConsoleKey::H ? 'H' : 'P';
        return (GetAsyncKeyState(virtualKey) & 0x8000) != 0;
    };
}

StabilityConfig productionStabilityConfig()
{
    optional<chrono::milliseconds> maximumInterval;
    if (BilliardConfig::MAX_INTER_FRAME_INTERVAL_MS) {
        maximumInterval = chrono::milliseconds{*BilliardConfig::MAX_INTER_FRAME_INTERVAL_MS};
    }
    return {BilliardConfig::STABLE_FRAME_TOLERANCE_MM,
        BilliardConfig::POCKET_STABILITY_TOLERANCE_MM, maximumInterval};
}

enum class StageOutcome { Success, Failure };

// 從CameraPose開始（不含準備姿態確認）一路走到strikeReadyPose，途中完全
// 沒有任何氣動DO呼叫。回傳true代表已經停在strikeReadyPose、可以拍照觀察。
[[nodiscard]] StageOutcome runCameraToStrikeReady(
    RobotController& robot,
    SocketClient& visionClient,
    ReceiveEventFactory& receiveEventFactory,
    ThreeEventStability& stability,
    MotionPlanner& motionPlanner,
    const optional<BilliardConfig::RealHardwareExecutionConfig>& hwConfig,
    uint64_t cycleIdentity)
{
    cout << "[動作] 準備PTP到 CameraPose" << endl;
    printJoints("目標joint", BilliardConfig::CAMERA_JOINT);
    if (const auto r = robot.checkedConfiguredJointPtp(BilliardConfig::CAMERA_JOINT, hwConfig);
        !r.succeeded()) {
        printFailure("PTP到CameraPose", r.status, r.sdkCode);
        return StageOutcome::Failure;
    }
    if (const auto r = robot.confirmStopped(); !r.succeeded()) {
        printFailure("CameraPose確認停止", r.status, r.sdkCode);
        return StageOutcome::Failure;
    }
    cout << "[完成] 已到CameraPose，Sleep " << BilliardConfig::CAMERA_SETTLE_MS
         << "ms穩定中..." << endl;
    Sleep(BilliardConfig::CAMERA_SETTLE_MS);

    if (!visionClient.isConnected()) {
        cout << "[連線] Python視覺尚未連線，嘗試連線 "
             << BilliardConfig::VISION_SERVER_IP << ":" << BilliardConfig::VISION_SERVER_PORT
             << "..." << endl;
        if (!visionClient.connectToServer(
                BilliardConfig::VISION_SERVER_IP, BilliardConfig::VISION_SERVER_PORT)) {
            cout << "[失敗] Python視覺連線失敗（請確認Python程式已啟動）" << endl;
            return StageOutcome::Failure;
        }
        cout << "[完成] 已連線" << endl;
    } else {
        cout << "[跳過] Python視覺已連線" << endl;
    }

    receiveEventFactory.beginCycle(visionClient.connectionIdentity(), cycleIdentity);
    if (!receiveEventFactory.openCaptureWindow(
            visionClient.connectionIdentity(), cycleIdentity)) {
        cout << "[失敗] 開啟capture window失敗" << endl;
        return StageOutcome::Failure;
    }

    cout << "[收集] 等待三幀穩定資料..." << endl;
    optional<PlanningResult> planningResult;
    int framesAccepted = 0;
    // 人工監看用途：給30秒的寬鬆等待，不是比賽用的15秒deadline邏輯（那是
    // BilliardApp私有邏輯，這裡刻意用簡化、寬鬆許多的等待方式，因為這支
    // 程式本來就假設操作者全程盯著、要真的失敗才會等到這裡）。
    const auto waitStart = chrono::steady_clock::now();
    while (chrono::steady_clock::now() - waitStart < chrono::seconds(30)) {
        const SocketReceiveResult received = visionClient.receiveFrame();
        if (!received.isValid()) {
            cout << "[失敗] SocketReceiveResult invariant失敗" << endl;
            return StageOutcome::Failure;
        }
        if (received.status != SocketReceiveStatus::FrameReady || !received.frame) {
            if (received.status == SocketReceiveStatus::TimedOut) continue;
            cout << "[失敗] Vision socket連線中斷（status="
                 << static_cast<int>(received.status) << "）" << endl;
            return StageOutcome::Failure;
        }
        const ReceiveEventResult eventResult = receiveEventFactory.accept(
            *received.frame, visionClient.connectionIdentity(), chrono::steady_clock::now());
        if (!eventResult.isValid() || eventResult.status() != ReceiveEventStatus::Success ||
            !eventResult.value()) {
            cout << "[失敗] ReceiveEvent解析/驗證失敗" << endl;
            return StageOutcome::Failure;
        }
        const StabilityResult stable = stability.accept(*eventResult.value());
        if (!stable.isValid()) {
            cout << "[失敗] StabilityResult invariant失敗" << endl;
            return StageOutcome::Failure;
        }
        if (stable.status() == StabilityStatus::NeedMoreEvents) {
            ++framesAccepted;
            cout << "    第" << framesAccepted << "幀已接受，等待更多資料穩定..." << endl;
            continue;
        }
        if (stable.status() != StabilityStatus::Stable || !stable.value()) {
            cout << "[失敗] 三幀穩定性失敗，reason="
                 << (stable.diagnostic() ? static_cast<int>(stable.diagnostic()->reason) : -1)
                 << "（可能是球或袋口在偵測間移動過多，重新收集）" << endl;
            framesAccepted = 0;
            continue;
        }
        cout << "[完成] 三幀穩定，開始Phase1選球演算法..." << endl;
        PlanningResult planning = BilliardAlgorithm::planShot(
            *stable.value(), BilliardConfig::TABLE_GEOMETRY, BilliardConfig::BRAIN_CONFIG);
        if (!planning.isValid()) {
            cout << "[失敗] PlanningResult invariant失敗" << endl;
            return StageOutcome::Failure;
        }
        planningResult = std::move(planning);
        break;
    }
    if (!planningResult) {
        cout << "[失敗] 30秒內未收到穩定的三幀資料，放棄這一輪" << endl;
        return StageOutcome::Failure;
    }

    const ShotPlan* shotPlan = std::get_if<ShotPlan>(&planningResult->value());
    if (!shotPlan) {
        const NoPlan& noPlan = std::get<NoPlan>(planningResult->value());
        cout << "[結束] Phase1沒有找到可行方案，reason="
             << static_cast<int>(noPlan.reason) << endl;
        return StageOutcome::Failure;
    }
    cout << "[ShotPlan] 目標球=" << shotPlan->selectedTarget.ballNumber
         << "  類型=" << static_cast<int>(shotPlan->type)
         << "  母球位置=(" << shotPlan->source.cueBallSnapshot.x << ","
         << shotPlan->source.cueBallSnapshot.y << ")"
         << "  擊球方向=(" << shotPlan->shotDirectionXY.x << ","
         << shotPlan->shotDirectionXY.y << ")" << endl;

    cout << "[規劃] 計算ExecutionPlan（P2-01姿態搜尋）..." << endl;
    MotionPlanningChecks hardwareChecks;
    hardwareChecks.poseAccepted = [&robot, &hwConfig](const RobotPoseABC& pose) {
        return robot.checkPoseReachable(pose, hwConfig);
    };
    hardwareChecks.linearPathAccepted =
        [&robot, &hwConfig](const RobotPoseABC& approach, const RobotPoseABC& ready) {
        return robot.checkLinearPathAccepted(approach, ready, hwConfig);
    };
    const ExecutionPlanResult planResult = motionPlanner.createExecutionPlan(
        *planningResult, BilliardConfig::TABLE_GEOMETRY, BilliardConfig::MOTION_PLANNING_CONFIG,
        hardwareChecks, false);
    if (!planResult.isValid() || planResult.status() != ExecutionPlanStatus::Success ||
        !planResult.value()) {
        cout << "[失敗] ExecutionPlan建立失敗，status="
             << static_cast<int>(planResult.status())
             << "  reason="
             << (planResult.diagnostic()
                     ? static_cast<int>(planResult.diagnostic()->reason)
                     : -1)
             << endl;
        return StageOutcome::Failure;
    }
    const ExecutionPlan& plan = *planResult.value();
    cout << "[ExecutionPlan已建立]" << endl;
    printPose("safeApproachPose", plan.safeApproachPose);
    printPose("strikeReadyPose ", plan.strikeReadyPose);
    cout << "    strikeMode=" << (plan.strikeMode == StrikeMode::Push ? "Push" : "Pull") << endl;

    cout << "[確認] preflight可達性確認中..." ;
    const RobotAdapterResult preflight = robot.preflightExecution(plan, hwConfig);
    if (!preflight.succeeded()) {
        cout << adapterStatusName(preflight.status) << endl;
        printFailure("preflightExecution", preflight.status, preflight.sdkCode);
        return StageOutcome::Failure;
    }
    cout << "OK" << endl;

    cout << "[動作] 準備移動到 safeApproachPose" << endl;
    if (const auto r = robot.checkedPtp(plan, plan.safeApproachPose, hwConfig); !r.succeeded()) {
        printFailure("PTP到safeApproachPose", r.status, r.sdkCode);
        return StageOutcome::Failure;
    }
    cout << "[完成] 已到safeApproachPose" << endl;

    const RobotPoseAdapterResult actual = robot.readActualPose(plan, hwConfig);
    if (!actual.isValid() || !actual.value) {
        printFailure("讀取實際位置", actual.status, actual.sdkCode);
        return StageOutcome::Failure;
    }
    printPose("實際讀取位置", *actual.value);

    cout << "[動作] 準備LIN移動到 strikeReadyPose" << endl;
    if (const auto r = robot.checkedLin(plan, *actual.value, plan.strikeReadyPose, hwConfig);
        !r.succeeded()) {
        printFailure("LIN到strikeReadyPose", r.status, r.sdkCode);
        return StageOutcome::Failure;
    }
    if (const auto r = robot.confirmStopped(); !r.succeeded()) {
        printFailure("strikeReadyPose確認停止", r.status, r.sdkCode);
        return StageOutcome::Failure;
    }
    cout << "\n===== 已抵達擊球點，未下任何氣動指令，等待下一個按鍵 =====" << endl;
    cout << "      再按H：回CameraPose重新規劃　按P：回準備姿態\n" << endl;
    return StageOutcome::Success;
}

[[nodiscard]] bool returnToStandby(
    RobotController& robot,
    const optional<BilliardConfig::RealHardwareExecutionConfig>& hwConfig)
{
    cout << "[動作] 準備PTP回準備姿態" << endl;
    printJoints("目標joint", BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg);
    if (const auto r = robot.checkedConfiguredJointPtp(
            BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg, hwConfig);
        !r.succeeded()) {
        printFailure("PTP回準備姿態", r.status, r.sdkCode);
        return false;
    }
    if (const auto r = robot.confirmStopped(); !r.succeeded()) {
        printFailure("準備姿態確認停止", r.status, r.sdkCode);
        return false;
    }
    cout << "[完成] 已回準備姿態，等待下一個H" << endl;
    return true;
}

}  // namespace

int main()
{
    setlocale(LC_ALL, "zh_TW.UTF-8");
    cout << "=== No-Fire走位驗證程式（低速、人工監看用）===" << endl;
    cout << "本程式完全不包含任何氣動DO指令，走到擊球點會停下，不會真的擊球。" << endl;
    cout << "請確認已人工監看、隨時可以斷電/緊急停止，按Enter繼續..." << endl;
    cin.get();

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cout << "[錯誤] WSAStartup失敗。" << endl;
        return -1;
    }

    RobotController robot;
    if (!robot.connect(BilliardConfig::ARM_IP)) {
        cout << "[錯誤] 無法連線到機械手臂 " << BilliardConfig::ARM_IP << endl;
        WSACleanup();
        return -1;
    }
    cout << "[完成] 已連線機械手臂" << endl;

    SocketClient visionClient;
    VisionDataParser visionParser(BilliardConfig::VISION_OBSERVATION_BOUNDS);
    ReceiveEventFactory receiveEventFactory(visionParser);
    ThreeEventStability stability(productionStabilityConfig());
    MotionPlanner motionPlanner;
    const auto& hwConfig = BilliardConfig::REAL_HARDWARE_EXECUTION_CONFIG;

    if (!hwConfig) {
        cout << "[錯誤] REAL_HARDWARE_EXECUTION_CONFIG尚未核准（nullopt），無法執行。"
             << endl;
        WSACleanup();
        return -1;
    }

    StartControlGates gates;
    const ConsoleKeyPoll poll = productionConsoleKeyPoll();
    const ConsoleKeyDownQuery query = productionConsoleKeyDownQuery();
    uint64_t nextCycleIdentity = 1;
    bool waitingAtStrikeReady = false;

    cout << "\n[系統] 就緒。按H開始，按P回準備姿態，Ctrl+C結束程式。" << endl;
    while (true) {
        const StartControlEvent event = BilliardApp::pollStartControl(gates, poll);
        if (event.standbyEdge) {
            waitingAtStrikeReady = false;
            (void)returnToStandby(robot, hwConfig);
            continue;
        }
        if (!event.startEdge) {
            Sleep(BilliardConfig::MOTION_POLL_INTERVAL_MS);
            continue;
        }

        if (!confirmReadyReadOnly(robot, hwConfig)) continue;
        if (!BilliardConfig::STANDBY_JOINT_REFERENCE.isValid()) {
            cout << "[失敗] standby joint reference未核准" << endl;
            continue;
        }
        if (!prepareHardware(robot, hwConfig)) continue;

        if (!waitingAtStrikeReady) {
            array<double, 6> currentJoints{};
            int sdkCode = -1;
            cout << "[讀取] 目前關節角..." ;
            if (!robot.getCurrentJoints(currentJoints, sdkCode)) {
                cout << "失敗（sdkCode=" << sdkCode << ")" << endl;
                continue;
            }
            printJoints("目前關節角", currentJoints);
            const RobotBoolAdapterResult atStandby = robot.isAtConfiguredJoint(
                BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg,
                BilliardConfig::STANDBY_JOINT_TOLERANCE_DEG);
            if (atStandby.status != RobotAdapterStatus::Success || !atStandby.value) {
                cout << "[失敗] standby姿態確認失敗（status="
                     << adapterStatusName(atStandby.status) << "）" << endl;
                continue;
            }
            if (!*atStandby.value) {
                if (!returnToStandby(robot, hwConfig)) continue;
            } else {
                cout << "[跳過] 已經在準備姿態" << endl;
            }
        }

        const uint64_t cycleIdentity = nextCycleIdentity++;
        const StageOutcome outcome = runCameraToStrikeReady(
            robot, visionClient, receiveEventFactory, stability, motionPlanner, hwConfig,
            cycleIdentity);
        waitingAtStrikeReady = (outcome == StageOutcome::Success);

        BilliardApp::resyncStartControlToIdle(gates, poll, query);
    }
}
