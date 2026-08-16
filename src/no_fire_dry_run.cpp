// 低速、人工監看的 no-fire 走位驗證程式。
//
// 跟正式 BilliardApp::run() 的 H/P 流程共用同一套鍵盤 edge-gate 偵測，
// 走的階段也對應正式流程：
// read-only確認
// → 硬體準備
// → 必要時確認/恢復準備姿態
// → CameraPose
// → 連線Python視覺
// → 收三幀
// → Phase1選球
// → P2-01算擊球姿態
// → safeApproachPose
// → strikeReadyPose
//
// 整個檔案完全不包含任何真正擊球用的
// setDigitalOutput / pulseExtend / pulseRetract 等氣動擊球呼叫。
//
// 狀態規則：
//
// 1. 第一次啟動 / Unknown：
//    H → 確認或恢復 Standby → CameraPose → 規劃 → StrikeReady
//
// 2. 已在 Standby：
//    H → 直接 CameraPose，不再先回 Standby
//
// 3. 已在 StrikeReady：
//    H → 直接回 CameraPose，重新拍照、重新規劃、重新走位
//
// 4. P：
//    回 Standby → 清除移動期間累積的鍵盤事件
//    → 等待 H/P 實體按鍵放開
//    → 停在 Standby
//    → 必須重新按一個新的 H 才會去 CameraPose
//
// 注意：
// 本程式目前 motion / vision 階段仍是 blocking。
// 因此 P 只有在主控制 loop 正在讀取鍵盤時才能立即處理；
// 如果正在 checkedPtp / checkedLin / receiveFrame 等 blocking 呼叫中，
// P 不會成為即時的 software abort / emergency stop。
// 真正緊急停止仍必須使用機械手臂 E-Stop / 安全停止機制。
//
// 完全獨立於 BilliardApp.cpp / main.cpp，正式打球流程不受影響。

#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include <winsock2.h>
#include <windows.h>

#include <array>
#include <chrono>
#include <clocale>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "Algorithm.h"
#include "BilliardApp.h"
#include "BilliardConfig.h"
#include "MathUtils.h"
#include "MotionPlanner.h"
#include "RobotController.h"
#include "SocketClient.h"
#include "TableState.h"
#include "VisionDataParser.h"

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "user32.lib")

using namespace std;

namespace {

// ============================================================================
// 顯示工具
// ============================================================================

void printPose(const char* label, const RobotPoseABC& pose)
{
    cout << "    " << label
         << ": X=" << pose.x
         << " Y=" << pose.y
         << " Z=" << pose.z
         << " A=" << pose.a
         << " B=" << pose.b
         << " C=" << pose.c
         << endl;
}

void printJoints(const char* label, const array<double, 6>& joints)
{
    cout << "    " << label
         << ": ("
         << joints[0] << ", "
         << joints[1] << ", "
         << joints[2] << ", "
         << joints[3] << ", "
         << joints[4] << ", "
         << joints[5]
         << ")"
         << endl;
}

const char* adapterStatusName(RobotAdapterStatus status)
{
    switch (status) {
    case RobotAdapterStatus::Success:
        return "Success";

    case RobotAdapterStatus::ConfigurationMissing:
        return "ConfigurationMissing";

    case RobotAdapterStatus::InvalidConfiguration:
        return "InvalidConfiguration";

    case RobotAdapterStatus::Unauthorized:
        return "Unauthorized";

    case RobotAdapterStatus::NotConnected:
        return "NotConnected";

    case RobotAdapterStatus::SdkFailure:
        return "SdkFailure";

    case RobotAdapterStatus::NotReachable:
        return "NotReachable";

    case RobotAdapterStatus::NotStopped:
        return "NotStopped";

    case RobotAdapterStatus::UnknownUnsafe:
        return "UnknownUnsafe";
    }

    return "?";
}

void printFailure(
    const char* step,
    RobotAdapterStatus status,
    int sdkCode)
{
    cout << "[失敗] "
         << step
         << "："
         << adapterStatusName(status)
         << "（sdkCode="
         << sdkCode
         << "）"
         << endl;
}

const char* singleFrameStatusName(SingleFrameStatus status)
{
    switch (status) {
    case SingleFrameStatus::Success:
        return "Success";

    case SingleFrameStatus::WrongFieldCount:
        return "WrongFieldCount（32值CSV欄位數不對）";

    case SingleFrameStatus::EmptyToken:
        return "EmptyToken（有空欄位）";

    case SingleFrameStatus::InvalidNumericToken:
        return "InvalidNumericToken（欄位非合法數字）";

    case SingleFrameStatus::NumericOverflow:
        return "NumericOverflow（數值超出範圍）";

    case SingleFrameStatus::NonFiniteValue:
        return "NonFiniteValue（NaN/Inf）";

    case SingleFrameStatus::InvalidSentinelPair:
        return "InvalidSentinelPair（缺席sentinel配對不合法）";

    case SingleFrameStatus::MissingRequiredCueBall:
        return "MissingRequiredCueBall（母球缺席，母球為必要欄位）";

    case SingleFrameStatus::MissingRequiredPocket:
        return "MissingRequiredPocket（有袋口缺席，袋口為必要欄位）";

    case SingleFrameStatus::OutOfObservationBounds:
        return "OutOfObservationBounds（座標超出觀測邊界）";

    case SingleFrameStatus::ConfigurationMissing:
        return "ConfigurationMissing（VisionDataParser未設定觀測邊界）";

    case SingleFrameStatus::InvalidConfiguration:
        return "InvalidConfiguration（觀測邊界設定不合法）";
    }

    return "?";
}

const char* receiveEventStatusName(ReceiveEventStatus status)
{
    switch (status) {
    case ReceiveEventStatus::Success:
        return "Success";

    case ReceiveEventStatus::NoActiveCycle:
        return "NoActiveCycle（尚未beginCycle）";

    case ReceiveEventStatus::CaptureWindowClosed:
        return "CaptureWindowClosed（capture window未開啟）";

    case ReceiveEventStatus::ConnectionMismatch:
        return "ConnectionMismatch（收到資料的連線身分跟目前cycle不符，可能已重新連線）";

    case ReceiveEventStatus::NonMonotonicReceiveTime:
        return "NonMonotonicReceiveTime（收到時間非遞增）";

    case ReceiveEventStatus::EventIdExhausted:
        return "EventIdExhausted（eventId耗盡）";

    case ReceiveEventStatus::FrameRejected:
        return "FrameRejected（單幀解析/驗證失敗，見下方SingleFrameStatus）";
    }

    return "?";
}

void printReceiveEventFailure(
    const ReceiveEventResult& eventResult,
    const string& rawPayload)
{
    cout
        << "[失敗] ReceiveEvent解析/驗證失敗："
        << receiveEventStatusName(eventResult.status())
        << endl;

    if (const auto& diag = eventResult.diagnostic();
        diag && diag->frameDiagnostic) {

        cout
            << "    SingleFrameStatus："
            << singleFrameStatusName(diag->frameDiagnostic->status)
            << endl;

        if (diag->frameDiagnostic->fieldIndex) {
            cout
                << "    fieldIndex（0-based，32值CSV中出錯的欄位）："
                << *diag->frameDiagnostic->fieldIndex
                << endl;
        }
    }

    cout
        << "    原始payload："
        << rawPayload
        << endl;
}

// ============================================================================
// Robot read-only ready confirmation
// ============================================================================

// 跟正式 BilliardApp::confirmRobotReadyReadOnly 對應。
// 純讀取確認，不主動進行 motion。
[[nodiscard]] bool confirmReadyReadOnly(
    RobotController& robot,
    const optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    cout << "[確認] Robot已連線？已停止？DO1/DO2確認OFF？...";

    if (!robot.isConnected()) {
        cout << "失敗（未連線）" << endl;
        return false;
    }

    RobotAdapterResult stopped = robot.confirmStopped();

    // NotStopped可能是控制器連線本身殘留的過期狀態（而非手臂真的在動），
    // 重連一次拿全新連線狀態再確認一次。
    if (stopped.status == RobotAdapterStatus::NotStopped) {
        cout << "（尚未停止，嘗試重新連線後再次確認...）";
        if (robot.reconnect()) {
            stopped = robot.confirmStopped();
        }
    }

    if (!stopped.succeeded()) {
        cout << (
            stopped.status == RobotAdapterStatus::UnknownUnsafe
                ? "UnknownUnsafe（無法確認是否已停止）"
                : "失敗（尚未停止）")
            << "（get_motion_state="
            << stopped.sdkCode
            << "）"
             << endl;

        return false;
    }

    const RobotBoolAdapterResult doOff =
        robot.confirmPneumaticOutputsOff(config);

    if (doOff.status != RobotAdapterStatus::Success ||
        !doOff.value ||
        !*doOff.value) {

        cout << (
            doOff.status == RobotAdapterStatus::UnknownUnsafe
                ? "UnknownUnsafe（DO讀值異常）"
                : "失敗（DO1/DO2非確認OFF）")
             << endl;

        return false;
    }

    cout << "OK" << endl;
    return true;
}

// ============================================================================
// Hardware preparation
// ============================================================================

// 跟正式 BilliardApp::prepareRobotHardwareForMotion 對應。
// 只能在 confirmReadyReadOnly 通過後呼叫。
[[nodiscard]] bool prepareHardware(
    RobotController& robot,
    const optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    cout
        << "[硬體準備] establishSafeOutputsOff -> clearAlarm -> "
        << "Tool1/Base0 -> 馬達on -> 速度比例"
        << BilliardConfig::NORMAL_SPEED_RATIO
        << "%...";

    if (const auto r = robot.establishSafeOutputsOff(config);
        !r.succeeded()) {

        cout
            << "失敗（establishSafeOutputsOff, "
            << adapterStatusName(r.status)
            << ")"
            << endl;

        return false;
    }

    // 跟production BilliardApp::prepareRobotHardwareForMotion同步：馬達從
    // 上一輪擊球後會持續保持ON，第二輪起呼叫clearAlarm前控制器可能還在
    // 馬達ON狀態下拒絕清除（sdkCode=300）。清alarm前先確保馬達關閉。
    if (const auto r = robot.setMotorState(0); !r.succeeded()) {

        cout
            << "失敗（setMotorState(0) before clearAlarm, "
            << adapterStatusName(r.status)
            << ")"
            << endl;

        return false;
    }

   if (const auto r = robot.clearAlarm();
    !r.succeeded()) {

    cout << endl;

    printFailure(
        "clearAlarm",
        r.status,
        r.sdkCode);

    int alarmSdkCode = -1;
    const vector<uint64_t> alarmCodes =
        robot.getAlarmCodes(alarmSdkCode);

    if (alarmSdkCode != 0) {

        cout
            << "    get_alarm_code查詢本身也失敗"
            << "（sdkCode=" << alarmSdkCode << "）"
            << endl;

    } else if (alarmCodes.empty()) {

        cout
            << "    目前控制器回報沒有alarm"
            << "（clearAlarm的sdkCode="
            << r.sdkCode
            << "另有其他原因，非alarm本身）"
            << endl;

    } else {

        cout
            << "    目前控制器回報的alarm code（共"
            << alarmCodes.size()
            << "筆）："
            << endl;

        for (const uint64_t code : alarmCodes) {

            cout
                << "      0x"
                << std::hex
                << code
                << std::dec
                << "  ("
                << code
                << ")"
                << endl;
        }
    }

    return false;
}

    if (const auto r = robot.activateConfiguredToolAndBase(config);
        !r.succeeded()) {

        cout
            << "失敗（activateConfiguredToolAndBase, "
            << adapterStatusName(r.status)
            << ")"
            << endl;

        return false;
    }

    if (const auto r = robot.setMotorState(1);
        !r.succeeded()) {

        cout
            << "失敗（setMotorState, "
            << adapterStatusName(r.status)
            << ")"
            << endl;

        return false;
    }

    if (const auto r =
            robot.setOverrideRatio(
                BilliardConfig::NORMAL_SPEED_RATIO);
        !r.succeeded()) {

        cout
            << "失敗（setOverrideRatio, "
            << adapterStatusName(r.status)
            << ")"
            << endl;

        return false;
    }

    cout << "OK" << endl;
    return true;
}

// ============================================================================
// Console keyboard handling
// ============================================================================

// BilliardApp.cpp 裡的同名邏輯屬於該 cpp 私有實作，
// 這裡維持同樣的 Windows Console KEY_EVENT 讀取方式。
ConsoleKeyPoll productionConsoleKeyPoll()
{
    return [] {
        vector<ConsoleKeyEvent> events;

        HANDLE stdIn = GetStdHandle(STD_INPUT_HANDLE);

        if (stdIn == INVALID_HANDLE_VALUE ||
            stdIn == nullptr) {

            return events;
        }

        DWORD pending = 0;

        if (!GetNumberOfConsoleInputEvents(
                stdIn,
                &pending) ||
            pending == 0) {

            return events;
        }

        vector<INPUT_RECORD> records(pending);

        DWORD read = 0;

        if (!ReadConsoleInputW(
                stdIn,
                records.data(),
                pending,
                &read)) {

            return events;
        }

        for (DWORD i = 0; i < read; ++i) {

            if (records[i].EventType != KEY_EVENT) {
                continue;
            }

            const KEY_EVENT_RECORD& key =
                records[i].Event.KeyEvent;

            if (key.wVirtualKeyCode == 'H') {
                events.push_back({
                    ConsoleKey::H,
                    key.bKeyDown != 0
                });
            }
            else if (key.wVirtualKeyCode == 'P') {
                events.push_back({
                    ConsoleKey::P,
                    key.bKeyDown != 0
                });
            }
        }

        return events;
    };
}

ConsoleKeyDownQuery productionConsoleKeyDownQuery()
{
    return [](ConsoleKey key) {

        const int virtualKey =
            key == ConsoleKey::H
                ? 'H'
                : 'P';

        return
            (GetAsyncKeyState(virtualKey) & 0x8000) != 0;
    };
}

// 清除 Windows Console Input Buffer。
// 主要用途：
// P → Standby 的 blocking motion 期間可能累積 H/P KEY_EVENT。
// P 完成後這些舊事件必須全部作廢，不能被下一輪當成新的 H。
void discardPendingConsoleInput()
{
    HANDLE stdIn = GetStdHandle(STD_INPUT_HANDLE);

    if (stdIn == INVALID_HANDLE_VALUE ||
        stdIn == nullptr) {

        return;
    }

    FlushConsoleInputBuffer(stdIn);
}

// P 完成後使用。
// 保證：
// 1. motion期間累積的H/P事件作廢
// 2. 如果使用者仍然壓著H/P，等待放開
// 3. 放開產生的KEY_UP也再清掉
// 4. 最後把BilliardApp edge-gate同步回idle
void rearmControlsAfterStandby(
    StartControlGates& gates,
    const ConsoleKeyPoll& poll,
    const ConsoleKeyDownQuery& query)
{
    // 第一次清除：
    // 清掉 P → Standby 過程中排隊的所有舊 console event。
    discardPendingConsoleInput();

    // 如果 H 或 P 還實體按住，先等使用者真正放開。
    while (query(ConsoleKey::H) ||
           query(ConsoleKey::P)) {

        Sleep(10);
    }

    // 放開按鍵本身可能產生 KEY_UP event，
    // 再清一次，確保下一輪從完全乾淨的輸入狀態開始。
    discardPendingConsoleInput();

    // 同步 edge gate。
    BilliardApp::resyncStartControlToIdle(
        gates,
        poll,
        query);
}

// ============================================================================
// Stability
// ============================================================================

StabilityConfig productionStabilityConfig()
{
    optional<chrono::milliseconds> maximumInterval;

    if (BilliardConfig::MAX_INTER_FRAME_INTERVAL_MS) {

        maximumInterval =
            chrono::milliseconds{
                *BilliardConfig::MAX_INTER_FRAME_INTERVAL_MS
            };
    }

    return {
        BilliardConfig::STABLE_FRAME_TOLERANCE_MM,
        BilliardConfig::POCKET_STABILITY_TOLERANCE_MM,
        maximumInterval
    };
}

// ============================================================================
// No-fire state machine
// ============================================================================

enum class NoFireState {

    // 位置未知。
    //
    // 例如：
    // - 程式剛啟動
    // - motion過程失敗
    // - 無法確定目前安全位置
    //
    // 下一次H必須先確認/恢復Standby。
    Unknown,

    // 已明確確認停在準備姿態。
    //
    // 下一次H：
    // 直接CameraPose，不重新回Standby。
    Standby,

    // 已明確確認停在strikeReadyPose。
    //
    // 下一次H：
    // 直接CameraPose重新拍照/規劃。
    StrikeReady
};

enum class StageOutcome {
    Success,
    Failure
};

// ============================================================================
// CameraPose -> planning -> strikeReadyPose
// ============================================================================

// 從 CameraPose 開始一路走到 strikeReadyPose。
// 不包含真正的擊球氣動動作。
//
// Success：
// 已確認停在 strikeReadyPose。
//
// Failure：
// 呼叫端應把位置狀態視為 Unknown，
// 下一次 H 先恢復安全的 Standby。
[[nodiscard]] StageOutcome runCameraToStrikeReady(
    RobotController& robot,
    SocketClient& visionClient,
    ReceiveEventFactory& receiveEventFactory,
    ThreeEventStability& stability,
    MotionPlanner& motionPlanner,
    const optional<BilliardConfig::RealHardwareExecutionConfig>& hwConfig,
    const optional<BilliardConfig::MotionPlanningConfig>& motionConfig,
    uint64_t cycleIdentity)
{
    // ------------------------------------------------------------------------
    // CameraPose
    // ------------------------------------------------------------------------

    cout << "[動作] 準備PTP到 CameraPose" << endl;

    printJoints(
        "目標joint",
        BilliardConfig::CAMERA_JOINT);

    if (const auto r =
            robot.checkedConfiguredJointPtp(
                BilliardConfig::CAMERA_JOINT,
                hwConfig);
        !r.succeeded()) {

        printFailure(
            "PTP到CameraPose",
            r.status,
            r.sdkCode);

        return StageOutcome::Failure;
    }

    if (const auto r = robot.confirmStopped();
        !r.succeeded()) {

        printFailure(
            "CameraPose確認停止",
            r.status,
            r.sdkCode);

        return StageOutcome::Failure;
    }

    // confirmStopped()只確認「目前沒在動」，不確認「真的到了CameraPose」
    // ——checkedConfiguredJointPtp的moveToAxis若被控制器拒絕/中途中止，
    // 手臂會停在別的姿態，但仍然「沒在動」，只看confirmStopped()會被
    // 誤判成已到位，直接Sleep後開始拍照。這裡額外用isAtConfiguredJoint
    // 重新讀真實關節角度核對位置，跟BilliardApp.cpp的
    // seam.confirmCameraPoseStopped用同一套邏輯。
    if (const auto atCamera = robot.isAtConfiguredJoint(
            BilliardConfig::CAMERA_JOINT,
            BilliardConfig::CAMERA_JOINT_TOLERANCE_DEG);
        atCamera.status != RobotAdapterStatus::Success ||
        !atCamera.value || !*atCamera.value) {

        printFailure(
            "CameraPose位置確認",
            atCamera.status,
            atCamera.sdkCode);

        return StageOutcome::Failure;
    }

    cout
        << "[完成] 已到CameraPose，Sleep "
        << BilliardConfig::CAMERA_SETTLE_MS
        << "ms穩定中..."
        << endl;

    Sleep(BilliardConfig::CAMERA_SETTLE_MS);

    // ------------------------------------------------------------------------
    // Vision socket
    // ------------------------------------------------------------------------

    if (!visionClient.isConnected()) {

        cout
            << "[連線] Python視覺尚未連線，嘗試連線 "
            << BilliardConfig::VISION_SERVER_IP
            << ":"
            << BilliardConfig::VISION_SERVER_PORT
            << "..."
            << endl;

        if (!visionClient.connectToServer(
                BilliardConfig::VISION_SERVER_IP,
                BilliardConfig::VISION_SERVER_PORT)) {

            cout
                << "[失敗] Python視覺連線失敗"
                << "（請確認Python程式已啟動）"
                << endl;

            return StageOutcome::Failure;
        }

        cout << "[完成] 已連線" << endl;
    }
    else {
        cout << "[跳過] Python視覺已連線" << endl;
    }

    // ------------------------------------------------------------------------
    // Capture window
    // ------------------------------------------------------------------------

    receiveEventFactory.beginCycle(
        visionClient.connectionIdentity(),
        cycleIdentity);

    if (!receiveEventFactory.openCaptureWindow(
            visionClient.connectionIdentity(),
            cycleIdentity)) {

        cout
            << "[失敗] 開啟capture window失敗"
            << endl;

        return StageOutcome::Failure;
    }

    const auto stopCaptureNow = [] {};

    // ------------------------------------------------------------------------
    // Receive stable frames
    // ------------------------------------------------------------------------

    cout
        << "[收集] 等待三幀穩定資料..."
        << endl;

    optional<PlanningResult> planningResult;
    optional<ResolvedTableGeometry> resolvedTableGeometry;
    optional<StableTableState> stableTableSnapshot;

    int framesAccepted = 0;

    // 人工監看的 no-fire tool 使用較寬鬆的 30 秒。
    const auto waitStart =
        chrono::steady_clock::now();

    while (
        chrono::steady_clock::now() - waitStart
        < chrono::seconds(30)) {

        const SocketReceiveResult received =
            visionClient.receiveFrame();

        if (!received.isValid()) {

            cout
                << "[失敗] SocketReceiveResult invariant失敗"
                << endl;

            stopCaptureNow();
            return StageOutcome::Failure;
        }

        if (received.status !=
                SocketReceiveStatus::FrameReady ||
            !received.frame) {

            if (received.status ==
                SocketReceiveStatus::TimedOut) {

                continue;
            }

            cout
                << "[失敗] Vision socket連線中斷"
                << "（status="
                << static_cast<int>(received.status)
                << "）"
                << endl;

            stopCaptureNow();
            return StageOutcome::Failure;
        }

        const ReceiveEventResult eventResult =
            receiveEventFactory.accept(
                *received.frame,
                visionClient.connectionIdentity(),
                chrono::steady_clock::now());

        if (!eventResult.isValid() ||
            eventResult.status() !=
                ReceiveEventStatus::Success ||
            !eventResult.value()) {

            printReceiveEventFailure(eventResult, *received.frame);

            stopCaptureNow();
            return StageOutcome::Failure;
        }

        const StabilityResult stable =
            stability.accept(
                *eventResult.value());

        if (!stable.isValid()) {

            cout
                << "[失敗] StabilityResult invariant失敗"
                << endl;

            stopCaptureNow();
            return StageOutcome::Failure;
        }

        if (stable.status() ==
            StabilityStatus::NeedMoreEvents) {

            ++framesAccepted;

            cout
                << "    第"
                << framesAccepted
                << "幀已接受，等待更多資料穩定..."
                << endl;

            continue;
        }

        if (stable.status() !=
                StabilityStatus::Stable ||
            !stable.value()) {

            cout
                << "[失敗] 三幀穩定性失敗，reason="
                << (
                    stable.diagnostic()
                        ? static_cast<int>(
                              stable.diagnostic()->reason)
                        : -1)
                << "（可能是球或袋口在偵測間移動過多，重新收集）"
                << endl;

            framesAccepted = 0;

            continue;
        }

        // --------------------------------------------------------------------
        // Phase1
        // --------------------------------------------------------------------

        cout
            << "[完成] 三幀穩定，開始Phase1選球演算法..."
            << endl;

        // 貼庫安全繞行用：解析失敗就留nullopt，等同功能關閉，不擋這一輪。
        const GeometryValueResult<ResolvedTableGeometry> resolvedGeometryResult =
            BilliardPhysics::resolveTableGeometry(
                stable.value()->pockets,
                BilliardConfig::TABLE_GEOMETRY);

        resolvedTableGeometry =
            resolvedGeometryResult.isValid() && resolvedGeometryResult.value()
                ? resolvedGeometryResult.value()
                : optional<ResolvedTableGeometry>{};

        PlanningResult planning =
            BilliardAlgorithm::planShot(
                *stable.value(),
                BilliardConfig::TABLE_GEOMETRY,
                BilliardConfig::BRAIN_CONFIG);

        if (!planning.isValid()) {

            cout
                << "[失敗] PlanningResult invariant失敗"
                << endl;

            stopCaptureNow();
            return StageOutcome::Failure;
        }

        planningResult =
            std::move(planning);
        stableTableSnapshot = *stable.value();

        break;
    }

    stopCaptureNow();

    if (!planningResult) {

        cout
            << "[失敗] 30秒內未收到穩定的三幀資料，"
            << "放棄這一輪"
            << endl;

        return StageOutcome::Failure;
    }

    // ------------------------------------------------------------------------
    // ShotPlan
    // ------------------------------------------------------------------------

    const ShotPlan* shotPlan =
        std::get_if<ShotPlan>(
            &planningResult->value());

    if (!shotPlan) {

        const NoPlan& noPlan =
            std::get<NoPlan>(
                planningResult->value());

        cout
            << "[結束] Phase1沒有找到可行方案，reason="
            << static_cast<int>(noPlan.reason)
            << endl;
        cout
            << "    feasiblePotCount=" << noPlan.feasiblePotCount
            << "  proceededToLegalContact="
            << (noPlan.proceededToLegalContact ? "true" : "false")
            << endl;
        if (noPlan.selectedTarget) {
            cout << "    selectedTarget.ballNumber="
                 << noPlan.selectedTarget->ballNumber << endl;
        }
        cout << "    diagnostic:";
        if (noPlan.diagnostic.targetStatus) {
            cout << " targetStatus="
                 << static_cast<int>(*noPlan.diagnostic.targetStatus);
        }
        if (noPlan.diagnostic.geometryStatus) {
            cout << " geometryStatus="
                 << static_cast<int>(*noPlan.diagnostic.geometryStatus);
        }
        if (noPlan.diagnostic.directStatus) {
            cout << " directStatus="
                 << static_cast<int>(*noPlan.diagnostic.directStatus);
        }
        if (noPlan.diagnostic.kickStatus) {
            cout << " kickStatus="
                 << static_cast<int>(*noPlan.diagnostic.kickStatus);
        }
        if (noPlan.diagnostic.selectionStatus) {
            cout << " selectionStatus="
                 << static_cast<int>(*noPlan.diagnostic.selectionStatus);
        }
        cout << endl;
        const auto printObstacle = [](std::optional<std::size_t> related) {
            if (related) cout << " relatedObstacleIndex=" << *related;
        };
        for (const auto& d : noPlan.directCandidateDiagnostics) {
            cout << "    [direct] pocket=" << static_cast<int>(d.pocketId)
                 << " reason=" << static_cast<int>(d.reason)
                 << " geometryStatus=" << static_cast<int>(d.geometryStatus);
            printObstacle(d.relatedObstacleIndex);
            cout << endl;
        }
        for (const auto& k : noPlan.kickCandidateDiagnostics) {
            cout << "    [kick] pocket=" << static_cast<int>(k.pocketId)
                 << " rail=" << static_cast<int>(k.railId)
                 << " reason=" << static_cast<int>(k.reason)
                 << " geometryStatus=" << static_cast<int>(k.geometryStatus);
            printObstacle(k.relatedObstacleIndex);
            cout << endl;
        }
        for (const auto& l : noPlan.legalContactDiagnostics) {
            cout << "    [legalContact] rail="
                 << (l.railId ? std::to_string(static_cast<int>(*l.railId))
                               : std::string{"-"})
                 << " reason=" << static_cast<int>(l.reason)
                 << " geometryStatus=" << static_cast<int>(l.geometryStatus);
            printObstacle(l.relatedObstacleIndex);
            cout << endl;
        }

        // relatedObstacleIndex對應的是stableTableSnapshot.objectBalls的
        // index（非球號，球號=index+1）；同步印出當輪穩定球位，方便對照
        // 桌面判斷是真的被擋還是vision/geometry算錯。
        if (stableTableSnapshot) {
            cout << "    [球位] 母球=(" << stableTableSnapshot->cueBall.x
                 << "," << stableTableSnapshot->cueBall.y << ")" << endl;
            for (std::size_t i = 0;
                 i < stableTableSnapshot->objectBalls.size(); ++i) {
                const auto& ball = stableTableSnapshot->objectBalls[i];
                if (!ball) continue;
                cout << "    [球位] 球" << (i + 1) << "(index=" << i
                     << ")=(" << ball->x << "," << ball->y << ")" << endl;
            }
        }

        return StageOutcome::Failure;
    }

    cout
        << "[ShotPlan]"
        << " 目標球="
        << shotPlan->selectedTarget.ballNumber

        << "  類型="
        << static_cast<int>(shotPlan->type)

        << "  母球位置=("
        << shotPlan->source.cueBallSnapshot.x
        << ","
        << shotPlan->source.cueBallSnapshot.y
        << ")"

        << "  擊球方向=("
        << shotPlan->shotDirectionXY.x
        << ","
        << shotPlan->shotDirectionXY.y
        << ")"

        << endl;

    // ------------------------------------------------------------------------
    // ExecutionPlan
    // ------------------------------------------------------------------------

    cout
        << "[規劃] 計算ExecutionPlan（P2-01姿態搜尋）..."
        << endl;

    MotionPlanningChecks hardwareChecks;

    hardwareChecks.poseAccepted =
        [&robot, &hwConfig](
            const RobotPoseABC& pose) {

            return robot.checkPoseReachable(
                pose,
                hwConfig);
        };

    hardwareChecks.linearPathAccepted =
        [&robot, &hwConfig](
            const RobotPoseABC& approach,
            const RobotPoseABC& ready) {

            return robot.checkLinearPathAccepted(
                approach,
                ready,
                hwConfig);
        };

    // 跟BilliardApp.cpp的offlineMotionPlanningChecksFor完全相同的純幾何
    // 投影邏輯（不需要硬體I/O），createExecutionPlan要求這個callback一定
    // 要有值，否則會是ConfigurationMissing/MissingValidationSeam。
    hardwareChecks.projectCueForwardAxisToBase0XY =
        [motionConfig](
            const RobotPoseABC& pose,
            const array<double, 3>& axis) -> optional<Vector2D> {

            if (!motionConfig ||
                !motionConfig->cueForwardAxisTool ||
                !motionConfig->cToolOffsetDeg ||
                !pose.isFinite()) {

                return nullopt;
            }

            const auto& pushAxis = *motionConfig->cueForwardAxisTool;
            const array<double, 3> pullAxis{
                -pushAxis[0], -pushAxis[1], -pushAxis[2]};
            const double axisSign = axis == pushAxis
                ? 1.0
                : (axis == pullAxis ? -1.0 : 0.0);

            if (axisSign == 0.0) {
                return nullopt;
            }

            const double radians =
                (pose.c - *motionConfig->cToolOffsetDeg) *
                BilliardMath::PI / 180.0;
            const Vector2D projected{
                axisSign * std::cos(radians),
                axisSign * std::sin(radians)};

            return BilliardMath::isFinite(projected)
                ? optional<Vector2D>{projected}
                : nullopt;
        };

    // 跟BilliardApp.cpp的services.buildExecutionPlan診斷入口用同一個算法：
    // 沒有ranked pot候選、但有legal contact候選，才算「pot已窮盡」。
    // 這裡固定傳false會讓legal contact的production fallback授權
    // （isProductionLegalContactFallback）永遠判定成沒窮盡，即使Phase1
    // 明明就是pot都失敗才退到legal contact——no_fire這個低速監督測試
    // 工具因此永遠測不到legal contact fallback。
    const Phase1ExecutionCandidates& shotPlanCandidates =
        planningResult->executionCandidates();
    const bool rankedPotCandidatesExhausted =
        shotPlanCandidates.rankedPotPlans.empty() &&
        !shotPlanCandidates.legalContactPlans.empty();

    ExecutionPlanResult planResult =
        motionPlanner.createExecutionPlan(
            *planningResult,
            BilliardConfig::TABLE_GEOMETRY,
            motionConfig,
            hardwareChecks,
            rankedPotCandidatesExhausted,
            resolvedTableGeometry);

    // 母球僅觸碰保底：Phase1當初判定shotPlan幾何可行、沒有預先生成
    // cueBallContactOnlyPlans，但shotPlan剛剛在ExecutionPlan建立
    // （P2-01姿態搜尋／FixedForceEnvelope／後方障礙）失敗——例如母球
    // 位置在手臂實際可達範圍邊緣。現場用同一個PlanningSourceAudit補
    // 生成360度方向候選重試一次，讓母球至少有安全推出的機會，不必
    // 這一輪直接失敗、回Unknown。UnknownUnsafe／硬體/設定/數值類失敗
    // 不在這個可重試清單內，維持fail closed。
    if ((!planResult.isValid() ||
         planResult.status() != ExecutionPlanStatus::Success ||
         !planResult.value()) &&
        planResult.diagnostic() &&
        (planResult.diagnostic()->reason ==
             ExecutionPlanFailureReason::FixedForceEnvelopeRejected ||
         planResult.diagnostic()->reason ==
             ExecutionPlanFailureReason::NoAcceptedPoseCandidate ||
         planResult.diagnostic()->reason ==
             ExecutionPlanFailureReason::RearObstacleBlocked) &&
        shotPlanCandidates.cueBallContactOnlyPlans.empty() &&
        resolvedTableGeometry) {

        cout
            << "[保底] shotPlan的ExecutionPlan建立失敗（reason="
            << static_cast<int>(planResult.diagnostic()->reason)
            << "），現場補生成CueBallContactOnly候選重試一次..."
            << endl;

        for (const ShotPlan& fallback :
                BilliardAlgorithm::generateCueBallContactOnlyExecutionFallback(
                    shotPlan->source, *resolvedTableGeometry)) {

            ExecutionPlanResult fallbackResult =
                motionPlanner.createExecutionPlan(
                    PlanningResult::shotPlan(fallback),
                    BilliardConfig::TABLE_GEOMETRY,
                    motionConfig,
                    hardwareChecks,
                    true,
                    resolvedTableGeometry);

            if (fallbackResult.isValid() &&
                fallbackResult.status() == ExecutionPlanStatus::Success &&
                fallbackResult.value()) {

                cout
                    << "[保底] CueBallContactOnly候選建立ExecutionPlan成功"
                    << endl;
                planResult = std::move(fallbackResult);
                break;
            }
        }
    }

    if (!planResult.isValid() ||
        planResult.status() !=
            ExecutionPlanStatus::Success ||
        !planResult.value()) {

        cout
            << "[失敗] ExecutionPlan建立失敗，status="
            << static_cast<int>(
                   planResult.status())

            << "  reason="
            << (
                planResult.diagnostic()
                    ? static_cast<int>(
                          planResult.diagnostic()->reason)
                    : -1)

            << endl;

        return StageOutcome::Failure;
    }

    const ExecutionPlan& plan =
        *planResult.value();

    cout
        << "[ExecutionPlan已建立]"
        << endl;

    printPose(
        "safeApproachPose",
        plan.safeApproachPose);

    printPose(
        "strikeReadyPose ",
        plan.strikeReadyPose);

    cout
        << "    strikeMode="
        << (
            plan.strikeMode == StrikeMode::Push
                ? "Push"
                : "Pull")
        << endl;

    // ------------------------------------------------------------------------
    // Preflight
    // ------------------------------------------------------------------------

    cout
        << "[確認] preflight可達性確認中...";

    const RobotAdapterResult preflight =
        robot.preflightExecution(
            plan,
            hwConfig);

    if (!preflight.succeeded()) {

        cout
            << adapterStatusName(
                   preflight.status)
            << endl;

        printFailure(
            "preflightExecution",
            preflight.status,
            preflight.sdkCode);

        return StageOutcome::Failure;
    }

    cout << "OK" << endl;

    // ------------------------------------------------------------------------
    // Safe approach
    // ------------------------------------------------------------------------

    cout
        << "[動作] 準備移動到 safeApproachPose"
        << endl;

    if (const auto r =
            robot.checkedPtp(
                plan,
                plan.safeApproachPose,
                hwConfig);
        !r.succeeded()) {

        printFailure(
            "PTP到safeApproachPose",
            r.status,
            r.sdkCode);

        return StageOutcome::Failure;
    }

    cout
        << "[完成] 已到safeApproachPose"
        << endl;

    // ------------------------------------------------------------------------
    // Read actual pose
    // ------------------------------------------------------------------------

    const RobotPoseAdapterResult actual =
        robot.readActualPose(
            plan,
            hwConfig);

    if (!actual.isValid() ||
        !actual.value) {

        printFailure(
            "讀取實際位置",
            actual.status,
            actual.sdkCode);

        return StageOutcome::Failure;
    }

    printPose(
        "實際讀取位置",
        *actual.value);

    // ------------------------------------------------------------------------
    // Strike ready
    // ------------------------------------------------------------------------

    cout
        << "[動作] 準備LIN移動到 strikeReadyPose"
        << endl;

    if (const auto r =
            robot.checkedLin(
                plan,
                *actual.value,
                plan.strikeReadyPose,
                hwConfig);
        !r.succeeded()) {

        printFailure(
            "LIN到strikeReadyPose",
            r.status,
            r.sdkCode);

        return StageOutcome::Failure;
    }

    if (const auto r =
            robot.confirmStopped();
        !r.succeeded()) {

        printFailure(
            "strikeReadyPose確認停止",
            r.status,
            r.sdkCode);

        return StageOutcome::Failure;
    }

    cout
        << "\n"
        << "===== 已抵達擊球點，未下任何氣動擊球指令，等待下一個按鍵 ====="
        << endl;

    cout
        << "      H：直接回CameraPose重新規劃"
        << "　P：回準備姿態"
        << "\n"
        << endl;

    return StageOutcome::Success;
}

// ============================================================================
// Standby
// ============================================================================

[[nodiscard]] bool returnToStandby(
    RobotController& robot,
    const optional<BilliardConfig::RealHardwareExecutionConfig>& hwConfig)
{
    cout
        << "[動作] 準備PTP回準備姿態"
        << endl;

    printJoints(
        "目標joint",
        BilliardConfig::
            STANDBY_JOINT_REFERENCE.jointDeg);

    if (const auto r =
            robot.checkedConfiguredJointPtp(
                BilliardConfig::
                    STANDBY_JOINT_REFERENCE.jointDeg,
                hwConfig);
        !r.succeeded()) {

        printFailure(
            "PTP回準備姿態",
            r.status,
            r.sdkCode);

        return false;
    }

    if (const auto r =
            robot.confirmStopped();
        !r.succeeded()) {

        printFailure(
            "準備姿態確認停止",
            r.status,
            r.sdkCode);

        return false;
    }

    cout
        << "[完成] 已回準備姿態"
        << endl;

    return true;
}

}  // namespace

// ============================================================================
// main
// ============================================================================

int main(int argc, char** argv)
{
    // ------------------------------------------------------------------------
    // Optional command-line Z override
    // ------------------------------------------------------------------------
    //
    // --strike-z
    // --safe-approach-z
    //
    // 只覆寫這次執行使用的 motionConfig，
    // 不寫回 BilliardConfig.cpp。

    optional<double> strikeZOverride;
    optional<double> safeApproachZOverride;

    for (int index = 1;
         index < argc;
         ++index) {

        const string argument =
            argv[index];

        if (index + 1 >= argc) {

            cout
                << "[錯誤] 參數 "
                << argument
                << " 缺少數值。"
                << endl;

            return -1;
        }

        const string value =
            argv[++index];

        try {

            if (argument == "--strike-z") {

                strikeZOverride =
                    std::stod(value);
            }
            else if (
                argument ==
                "--safe-approach-z") {

                safeApproachZOverride =
                    std::stod(value);
            }
            else {

                cout
                    << "[錯誤] 未知參數: "
                    << argument
                    << endl;

                return -1;
            }
        }
        catch (const std::exception& error) {

            cout
                << "[錯誤] 解析參數失敗（"
                << argument
                << "="
                << value
                << "）: "
                << error.what()
                << endl;

            return -1;
        }
    }

    if (
        (strikeZOverride &&
         !std::isfinite(*strikeZOverride))
        ||
        (safeApproachZOverride &&
         !std::isfinite(*safeApproachZOverride))) {

        cout
            << "[錯誤] --strike-z / --safe-approach-z "
            << "必須是有限數值。"
            << endl;

        return -1;
    }

    optional<BilliardConfig::MotionPlanningConfig>
        motionConfig =
            BilliardConfig::
                MOTION_PLANNING_CONFIG;

    if (!motionConfig &&
        (strikeZOverride ||
         safeApproachZOverride)) {

        cout
            << "[錯誤] MOTION_PLANNING_CONFIG尚未核准"
            << "（nullopt），無法套用Z覆寫。"
            << endl;

        return -1;
    }

    if (motionConfig &&
        strikeZOverride) {

        motionConfig->strikeZMm =
            *strikeZOverride;
    }

    if (motionConfig &&
        safeApproachZOverride) {

        motionConfig->safeApproachZMm =
            *safeApproachZOverride;
    }

    // ------------------------------------------------------------------------
    // Start
    // ------------------------------------------------------------------------

    setlocale(
        LC_ALL,
        "zh_TW.UTF-8");

    cout
        << "=== No-Fire走位驗證程式（低速、人工監看用）==="
        << endl;

    if (strikeZOverride) {

        cout
            << "[覆寫] strikeZMm = "
            << *strikeZOverride
            << " mm"
            << "（command line，不是BilliardConfig.cpp裡的值）"
            << endl;
    }

    if (safeApproachZOverride) {

        cout
            << "[覆寫] safeApproachZMm = "
            << *safeApproachZOverride
            << " mm"
            << "（command line，不是BilliardConfig.cpp裡的值）"
            << endl;
    }

    cout
        << "本程式完全不包含任何真正擊球用的氣動指令，"
        << "走到擊球點會停下，不會真的擊球。"
        << endl;

    cout
        << "請確認已人工監看、隨時可以斷電/緊急停止，"
        << "按Enter繼續..."
        << endl;

    cin.get();

    // ------------------------------------------------------------------------
    // Winsock
    // ------------------------------------------------------------------------

    WSADATA wsaData;

    if (WSAStartup(
            MAKEWORD(2, 2),
            &wsaData) != 0) {

        cout
            << "[錯誤] WSAStartup失敗。"
            << endl;

        return -1;
    }

    // ------------------------------------------------------------------------
    // Robot
    // ------------------------------------------------------------------------

    RobotController robot;

    if (!robot.connect(
            BilliardConfig::ARM_IP)) {

        cout
            << "[錯誤] 無法連線到機械手臂 "
            << BilliardConfig::ARM_IP
            << endl;

        WSACleanup();

        return -1;
    }

    cout
        << "[完成] 已連線機械手臂"
        << endl;

    // ------------------------------------------------------------------------
    // Vision / planning
    // ------------------------------------------------------------------------

    SocketClient visionClient;

    VisionDataParser visionParser(
        BilliardConfig::
            VISION_OBSERVATION_BOUNDS);

    ReceiveEventFactory receiveEventFactory(
        visionParser);

    ThreeEventStability stability(
        productionStabilityConfig());

    MotionPlanner motionPlanner;

    const auto& hwConfig =
        BilliardConfig::
            REAL_HARDWARE_EXECUTION_CONFIG;

    if (!hwConfig) {

        cout
            << "[錯誤] REAL_HARDWARE_EXECUTION_CONFIG尚未核准"
            << "（nullopt），無法執行。"
            << endl;

        WSACleanup();

        return -1;
    }

    // ------------------------------------------------------------------------
    // Keyboard gates
    // ------------------------------------------------------------------------

    StartControlGates gates;

    const ConsoleKeyPoll poll =
        productionConsoleKeyPoll();

    const ConsoleKeyDownQuery query =
        productionConsoleKeyDownQuery();

    uint64_t nextCycleIdentity = 1;

    // 取代原本：
    //
    // bool waitingAtStrikeReady = false;
    //
    // 現在明確區分 Unknown / Standby / StrikeReady。
    NoFireState state =
        NoFireState::Unknown;

    cout
        << "\n"
        << "[系統] 就緒。"
        << "按H開始，按P回準備姿態，Ctrl+C結束程式。"
        << endl;

    // =========================================================================
    // Main control loop
    // =========================================================================

    while (true) {

        const StartControlEvent event =
            BilliardApp::pollStartControl(
                gates,
                poll);

        // =====================================================================
        // P
        // =====================================================================
        //
        // P 的新規則：
        //
        // 任何主 loop 可處理 P 的時刻：
        //
        // → 回 Standby
        // → 清掉 blocking motion 期間排隊的 H/P
        // → 等 H/P 實體放開
        // → state = Standby
        // → 停住
        // → 必須有一個全新的 H 才會往 CameraPose
        //
        // 這是避免：
        //
        // P
        // → Standby
        // → 舊 H event 還在 buffer
        // → 自己跑 CameraPose

        if (event.standbyEdge) {

            // motion 開始前先視為未知。
            // 只有 returnToStandby 成功後，
            // 才能正式標記為 Standby。
            state =
                NoFireState::Unknown;

            const bool standbySucceeded =
                returnToStandby(
                    robot,
                    hwConfig);

            if (standbySucceeded) {

                state =
                    NoFireState::Standby;

                cout
                    << "[狀態] Standby"
                    << endl;
            }
            else {

                state =
                    NoFireState::Unknown;

                cout
                    << "[狀態] Unknown"
                    << "（回準備姿態失敗）"
                    << endl;
            }

            // --------------------------------------------------------------
            // 重要：
            // 使 P 完成前累積的 H/P 完全失效。
            // --------------------------------------------------------------

            rearmControlsAfterStandby(
                gates,
                poll,
                query);

            if (state ==
                NoFireState::Standby) {

                cout
                    << "\n"
                    << "[系統] 已停在準備姿態。"
                    << "等待新的 H 指令。"
                    << endl;

                cout
                    << "       H → 直接 CameraPose"
                    << "\n"
                    << endl;
            }

            continue;
        }

        // =====================================================================
        // No new key
        // =====================================================================

        if (!event.startEdge) {

            Sleep(
                BilliardConfig::
                    MOTION_POLL_INTERVAL_MS);

            continue;
        }

        // =====================================================================
        // H
        // =====================================================================

        cout
            << "\n[命令] 收到新的 H"
            << endl;

        // ---------------------------------------------------------------------
        // Basic robot confirmation
        // ---------------------------------------------------------------------

        if (!confirmReadyReadOnly(
                robot,
                hwConfig)) {

            continue;
        }

        if (!BilliardConfig::
                STANDBY_JOINT_REFERENCE
                    .isValid()) {

            cout
                << "[失敗] standby joint reference未核准"
                << endl;

            continue;
        }

        if (!prepareHardware(
                robot,
                hwConfig)) {

            continue;
        }

        // =====================================================================
        // H：不論目前state是Unknown／Standby／StrikeReady，一律直接前往
        // CameraPose，不再另外檢查/恢復Standby姿態。
        // =====================================================================
        //
        // 讀者請注意：這代表拿掉了「先確認機械手臂目前在已知的Standby姿態
        // 才移動」這道額外關卡；checkedConfiguredJointPtp本身的reachability
        // 讀值確認、moveToAxis/confirmStopped仍然保留，不是完全不驗證，
        // 只是不再要求先回到一個已知參考姿態才出發。這是操作員明確要求的
        // 行為，取捨請自行評估。

        if (state ==
            NoFireState::Standby) {

            cout
                << "[H] 已知目前在Standby，"
                << "直接前往CameraPose。"
                << endl;
        }
        else if (
            state ==
            NoFireState::StrikeReady) {

            cout
                << "[H] 已知目前在StrikeReady，"
                << "直接回CameraPose重新拍照與規劃。"
                << endl;
        }
        else {

            cout
                << "[H] 目前位置狀態Unknown，"
                << "不做額外確認，直接前往CameraPose。"
                << endl;
        }

        // =====================================================================
        // Run one no-fire planning cycle
        // =====================================================================

        const uint64_t cycleIdentity =
            nextCycleIdentity++;

        const StageOutcome outcome =
            runCameraToStrikeReady(
                robot,
                visionClient,
                receiveEventFactory,
                stability,
                motionPlanner,
                hwConfig,
                motionConfig,
                cycleIdentity);

        // =====================================================================
        // Update state
        // =====================================================================

        if (outcome ==
            StageOutcome::Success) {

            state =
                NoFireState::StrikeReady;

            cout
                << "[狀態] StrikeReady"
                << endl;
        }
        else {

            // 中途任何失敗後，不假設機器人還在哪一個已知安全姿態。
            //
            // 下一個 H：
            // → 先做 Standby recovery
            // → 再 CameraPose
            state =
                NoFireState::Unknown;

            cout
                << "[狀態] Unknown"
                << "（本輪流程失敗，下一次H會先恢復Standby）"
                << endl;
        }

        // H 這一輪結束後重新同步 edge gate。
        //
        // 這可以避免本輪 blocking operation 期間累積的
        // 舊 H event 被當成下一輪新的 H。
        BilliardApp::resyncStartControlToIdle(
            gates,
            poll,
            query);
    }
}
