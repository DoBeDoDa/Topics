// 協調視覺解析、目標選擇、擊球規劃與手臂動作，不實作個別演算法細節。
#include "BilliardApp.h"

#include <chrono>
#include <cmath>
#include <iostream>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
// GetAsyncKeyState()（H/P長按跨邊界resync用）位於user32.lib，
// 預設console連結不包含它，需顯式引入。
#pragma comment(lib, "user32.lib")

#include "BilliardConfig.h"
#include "MathUtils.h"

using namespace std;

namespace {

const char* executionCycleStateName(ExecutionCycleState state)
{
    switch (state) {
    case ExecutionCycleState::WaitingForStart: return "WaitingForStart";
    case ExecutionCycleState::StartRequested: return "StartRequested";
    case ExecutionCycleState::PreparationReturn: return "PreparationReturn";
    case ExecutionCycleState::CameraPose: return "CameraPose";
    case ExecutionCycleState::CameraSettling: return "CameraSettling";
    case ExecutionCycleState::Planning: return "Planning";
    case ExecutionCycleState::StrikeReady: return "StrikeReady";
    case ExecutionCycleState::Pneumatic: return "Pneumatic";
    case ExecutionCycleState::PostStrikeActualPose: return "PostStrikeActualPose";
    case ExecutionCycleState::SafeLift: return "SafeLift";
    case ExecutionCycleState::StandbyReturn: return "StandbyReturn";
    case ExecutionCycleState::CycleCompleted: return "CycleCompleted";
    case ExecutionCycleState::ManualRecoveryRequired: return "ManualRecoveryRequired";
    case ExecutionCycleState::UnknownUnsafe: return "UnknownUnsafe";
    }
    return "Unknown";
}

const char* executionCycleStatusName(ExecutionCycleStatus status)
{
    switch (status) {
    case ExecutionCycleStatus::Completed: return "Completed";
    case ExecutionCycleStatus::SafeFailure: return "SafeFailure";
    case ExecutionCycleStatus::UnknownUnsafe: return "UnknownUnsafe";
    case ExecutionCycleStatus::StartRejected: return "StartRejected";
    }
    return "Unknown";
}

const char* executionCycleFailureReasonName(ExecutionCycleFailureReason reason)
{
    switch (reason) {
    case ExecutionCycleFailureReason::None: return "None";
    case ExecutionCycleFailureReason::CycleAlreadyActive: return "CycleAlreadyActive";
    case ExecutionCycleFailureReason::CycleIdentityExhausted:
        return "CycleIdentityExhausted";
    case ExecutionCycleFailureReason::MissingFakeAdapter: return "MissingFakeAdapter";
    case ExecutionCycleFailureReason::PreparationCheckFailed:
        return "PreparationCheckFailed";
    case ExecutionCycleFailureReason::PreparationReturnMotionFailed:
        return "PreparationReturnMotionFailed";
    case ExecutionCycleFailureReason::PreparationReturnNotStopped:
        return "PreparationReturnNotStopped";
    case ExecutionCycleFailureReason::CameraPoseMotionFailed:
        return "CameraPoseMotionFailed";
    case ExecutionCycleFailureReason::CameraPoseNotStopped:
        return "CameraPoseNotStopped";
    case ExecutionCycleFailureReason::CameraSettleFailed: return "CameraSettleFailed";
    case ExecutionCycleFailureReason::CaptureAndPlanFailed:
        return "CaptureAndPlanFailed";
    case ExecutionCycleFailureReason::VisionReconnectManualRecoveryRequired:
        return "VisionReconnectManualRecoveryRequired";
    case ExecutionCycleFailureReason::InvalidExecutionPlan:
        return "InvalidExecutionPlan";
    case ExecutionCycleFailureReason::ExecutionPlanCycleMismatch:
        return "ExecutionPlanCycleMismatch";
    case ExecutionCycleFailureReason::StrikeReadyValidationFailed:
        return "StrikeReadyValidationFailed";
    case ExecutionCycleFailureReason::StrikeReadyMotionFailed:
        return "StrikeReadyMotionFailed";
    case ExecutionCycleFailureReason::StrikeReadyNotStopped:
        return "StrikeReadyNotStopped";
    case ExecutionCycleFailureReason::PneumaticFailed: return "PneumaticFailed";
    case ExecutionCycleFailureReason::PneumaticStateUnknown:
        return "PneumaticStateUnknown";
    case ExecutionCycleFailureReason::ActualPoseReadFailed:
        return "ActualPoseReadFailed";
    case ExecutionCycleFailureReason::InvalidActualPose: return "InvalidActualPose";
    case ExecutionCycleFailureReason::SafeLiftPathCheckFailed:
        return "SafeLiftPathCheckFailed";
    case ExecutionCycleFailureReason::SafeLiftPathUnreachable:
        return "SafeLiftPathUnreachable";
    case ExecutionCycleFailureReason::SafeLiftMotionFailed:
        return "SafeLiftMotionFailed";
    case ExecutionCycleFailureReason::SafeLiftNotConfirmed:
        return "SafeLiftNotConfirmed";
    case ExecutionCycleFailureReason::StandbyReturnMotionFailed:
        return "StandbyReturnMotionFailed";
    case ExecutionCycleFailureReason::StandbyReturnNotStopped:
        return "StandbyReturnNotStopped";
    }
    return "Unknown";
}

StabilityConfig productionStabilityConfig()
{
    std::optional<std::chrono::milliseconds> maximumInterval;
    if (BilliardConfig::MAX_INTER_FRAME_INTERVAL_MS) {
        maximumInterval = std::chrono::milliseconds{
            *BilliardConfig::MAX_INTER_FRAME_INTERVAL_MS};
    }
    return {
        BilliardConfig::STABLE_FRAME_TOLERANCE_MM,
        BilliardConfig::POCKET_STABILITY_TOLERANCE_MM,
        maximumInterval};
}

StabilityFailureReason stabilityResetReason(ReceiveEventInvalidationReason reason)
{
    switch (reason) {
    case ReceiveEventInvalidationReason::Disconnect:
        return StabilityFailureReason::Disconnected;
    case ReceiveEventInvalidationReason::Reconnect:
        return StabilityFailureReason::Reconnected;
    case ReceiveEventInvalidationReason::ParserFailure:
        return StabilityFailureReason::ParserFailure;
    case ReceiveEventInvalidationReason::Timeout:
        return StabilityFailureReason::TimedOut;
    case ReceiveEventInvalidationReason::CycleChanged:
        return StabilityFailureReason::CycleChanged;
    default:
        return StabilityFailureReason::ExplicitReset;
    }
}

MotionPlanningChecks offlineMotionPlanningChecksFor(
    const std::optional<BilliardConfig::MotionPlanningConfig>& config)
{
    return {
        [](const RobotPoseABC& pose) {
            return std::optional<bool>{pose.isFinite()};
        },
        [config](const RobotPoseABC& pose,
            const std::array<double, 3>& axis) -> std::optional<Vector2D> {
            if (!config || !config->cueForwardAxisTool ||
                !config->cToolOffsetDeg || !pose.isFinite()) {
                return std::nullopt;
            }
            const auto& pushAxis = *config->cueForwardAxisTool;
            const std::array<double, 3> pullAxis{
                -pushAxis[0], -pushAxis[1], -pushAxis[2]};
            const double axisSign = axis == pushAxis
                ? 1.0
                : (axis == pullAxis ? -1.0 : 0.0);
            if (axisSign == 0.0) return std::nullopt;
            const double radians =
                (pose.c - *config->cToolOffsetDeg) *
                BilliardMath::PI / 180.0;
            const Vector2D projected{
                axisSign * std::cos(radians),
                axisSign * std::sin(radians)};
            return BilliardMath::isFinite(projected)
                ? std::optional<Vector2D>{projected}
                : std::nullopt;
        },
        [](const RobotPoseABC& approach, const RobotPoseABC& ready) {
            return std::optional<bool>{
                approach.isFinite() && ready.isFinite()};
        }};
}

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

// 競賽用實體Start按鈕（接在DI1）跟鍵盤H功能等同：兩者都只是把一個
// down/up訊號餵進同一套pollStartControl／KeyEdgeGate，edge偵測、防重
// 觸發完全共用既有機制，這裡不新增另一套判斷。DI讀取失敗（未連線、
// SDK未提供、讀值非0/1）視為「這一輪沒偵測到訊號」，不latch
// unknownUnsafeLatched——誤判成沒按鍵不是安全危害，頂多這一輪沒觸發。
//
// [2026-08-16實測確認] DI1是active-low：沒按時讀到1（高電位），按下時
// 讀到0——不是原本假設的active-high。這裡一律把讀值反相成「按下=true」
// 再送進edge-gate，否則連線後第一次讀值(未按=1)會被誤判成剛按下的
// rising edge，程式一啟動就自己觸發一輪H。
ConsoleKeyPoll combinedStartPoll(
    ConsoleKeyPoll keyboardPoll,
    RobotController& robot,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    // lastLogged只用來在DI原始讀值「改變」時才印一行，避免每次poll都洗版；
    // 純除錯輔助，不影響任何判斷邏輯。
    auto lastLogged = std::make_shared<std::optional<bool>>();
    return [keyboardPoll, &robot, &config, lastLogged]() {
        std::vector<ConsoleKeyEvent> events =
            keyboardPoll ? keyboardPoll() : std::vector<ConsoleKeyEvent>{};
        if (config && config->startDigitalInputIndex) {
            if (const std::optional<bool> di1 =
                    robot.readDigitalInput(*config->startDigitalInputIndex)) {
                if (!*lastLogged || **lastLogged != *di1) {
                    cout << "[DI1] 原始讀值變化：" << (*di1 ? "1" : "0")
                         << "  ->反相後送進edge-gate的keyDown="
                         << (!*di1 ? "true" : "false") << endl;
                    *lastLogged = *di1;
                }
                events.push_back({ConsoleKey::H, !*di1});
            } else if (*lastLogged) {
                cout << "[DI1] 讀取失敗（未連線/SDK未提供/讀值非0或1）"
                     << endl;
                lastLogged->reset();
            }
        }
        return events;
    };
}

ConsoleKeyDownQuery combinedStartQuery(
    ConsoleKeyDownQuery keyboardQuery,
    RobotController& robot,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    return [keyboardQuery, &robot, &config](ConsoleKey key) {
        const bool keyboardDown = keyboardQuery ? keyboardQuery(key) : false;
        if (keyboardDown) return true;
        if (key != ConsoleKey::H || !config || !config->startDigitalInputIndex) {
            return false;
        }
        const std::optional<bool> di1 =
            robot.readDigitalInput(*config->startDigitalInputIndex);
        return di1 && !*di1;
    };
}

}  // namespace

BilliardApp::BilliardApp()
    : receiveEventFactory(visionParser),
      stability(productionStabilityConfig()),
      nextShotCycleIdentity(1)
{
    offlineMotionPlanningChecks = offlineMotionPlanningChecksFor(
        BilliardConfig::MOTION_PLANNING_CONFIG);
}

BilliardApp::BilliardApp(MotionPlanningChecks offlineChecks)
    : BilliardApp()
{
    offlineMotionPlanningChecks = std::move(offlineChecks);
}

#ifdef BILLIARDS_P2_03_TEST_SEAM
BilliardApp::BilliardApp(BilliardAppRunTestSeam seam)
    : BilliardApp()
{
    if (seam.observationBounds) {
        visionParser = VisionDataParser(seam.observationBounds);
    }
    if (seam.stabilityConfig) {
        stability = ThreeEventStability(*seam.stabilityConfig);
    }
    offlineMotionPlanningChecks = seam.motionPlanningChecks
        ? seam.motionPlanningChecks
        : std::optional<MotionPlanningChecks>{
              offlineMotionPlanningChecksFor(seam.motionPlanningConfig)};
    runTestSeam = std::move(seam);
}
#endif

namespace {
std::optional<OfflineStepStatus> requireRobotAdapterSuccess(
    const RobotAdapterResult& result)
{
    if (result.status == RobotAdapterStatus::UnknownUnsafe) {
        return OfflineStepStatus::UnknownUnsafe;
    }
    if (!result.succeeded()) return OfflineStepStatus::Failure;
    return std::nullopt;
}
}  // namespace

OfflineStepResult BilliardApp::confirmRobotReadyReadOnly(
    RobotController& robot,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (!robot.isConnected()) return {OfflineStepStatus::Failure};
    RobotAdapterResult stopped = robot.confirmStopped();
    // NotStopped可能是控制器連線本身殘留的過期狀態（而非手臂真的在動），
    // 重連一次拿全新連線狀態再確認；reconnect失敗或再次確認仍NotStopped
    // 就照原本判斷失敗，不放寬confirmStopped()的判斷邏輯本身。
    if (stopped.status == RobotAdapterStatus::NotStopped && robot.reconnect()) {
        stopped = robot.confirmStopped();
    }
    if (stopped.status == RobotAdapterStatus::UnknownUnsafe) {
        return {OfflineStepStatus::UnknownUnsafe};
    }
    if (!stopped.succeeded()) return {OfflineStepStatus::Failure};
    const RobotBoolAdapterResult doOff = robot.confirmPneumaticOutputsOff(config);
    if (doOff.status == RobotAdapterStatus::UnknownUnsafe) {
        return {OfflineStepStatus::UnknownUnsafe};
    }
    if (doOff.status != RobotAdapterStatus::Success || !doOff.value || !*doOff.value) {
        return {OfflineStepStatus::Failure};
    }
    return {OfflineStepStatus::Success};
}

OfflineStepResult BilliardApp::prepareRobotHardwareForMotion(
    RobotController& robot,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config)
{
    if (const auto bad = requireRobotAdapterSuccess(robot.establishSafeOutputsOff(config))) {
        return {*bad};
    }
    // 馬達從上一輪擊球後會持續保持ON（只有disconnect()才會關），第二輪起
    // 呼叫clearAlarm前控制器可能還在馬達ON狀態下拒絕清除（sdkCode=300，
    // 即使get_alarm_code查無alarm）。清alarm前先確保馬達關閉，稍後才
    // setMotorState(1)重新開啟，讓每一輪clearAlarm都在同樣的馬達OFF前提
    // 下執行。
    if (const auto bad = requireRobotAdapterSuccess(robot.setMotorState(0))) return {*bad};
    if (const auto bad = requireRobotAdapterSuccess(robot.clearAlarm())) return {*bad};
    if (const auto bad =
            requireRobotAdapterSuccess(robot.activateConfiguredToolAndBase(config))) {
        return {*bad};
    }
    if (const auto bad = requireRobotAdapterSuccess(robot.setMotorState(1))) return {*bad};
    if (const auto bad = requireRobotAdapterSuccess(
            robot.setOverrideRatio(BilliardConfig::NORMAL_SPEED_RATIO))) {
        return {*bad};
    }
    return {OfflineStepStatus::Success};
}

ExecutionCycleResult BilliardApp::runRealSingleCycle(
    OfflineExecutionRuntime& runtime,
    std::uint64_t cycleIdentity,
    RobotController& robotController,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config,
    const RealExecutionCycleServices& services,
    const ShotDeadlineClock& deadline)
{
    if (runtime.state != ExecutionCycleState::WaitingForStart ||
        cycleIdentity == 0) {
        return {ExecutionCycleStatus::StartRejected, std::nullopt,
            ExecutionCycleDiagnostic{
                cycleIdentity == 0
                    ? ExecutionCycleFailureReason::CycleIdentityExhausted
                    : ExecutionCycleFailureReason::CycleAlreadyActive,
                runtime.state}};
    }
    if (!services.settleCamera || !services.flushStaleVisionBuffer ||
        !services.resetCycleAccumulation || !services.openCaptureWindow ||
        !services.runPhase1 || !services.isVisionConnected ||
        !services.connectVision || !services.currentPlanningResult ||
        !services.buildExecutionPlanForShot ||
        !RobotController::validateRealHardwareConfiguration(config).succeeded()) {
        return {ExecutionCycleStatus::SafeFailure, std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::InvalidExecutionPlan,
                runtime.state}};
    }
    if (!robotController.isConnected() &&
        !robotController.connect(BilliardConfig::ARM_IP)) {
        return {ExecutionCycleStatus::SafeFailure, std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::PreparationCheckFailed,
                runtime.state}};
    }
    // 注意：establishSafeOutputsOff／clearAlarm／activateConfiguredToolAndBase／
    // setMotorState／setOverrideRatio刻意不在此處執行——這些都會寫入硬體狀態
    // （DO、alarm、tool/base、motor、speed），依核准流程必須排在read-only的
    // 「確認Robot connected、已停止、DO1/DO2 OFF、非UnknownUnsafe」與
    // 「STANDBY_JOINT_REFERENCE已核准」兩道門檻都通過之後才能執行，因此改由
    // seam.prepareHardwareForMotion單獨負責，且排在confirmStandbyReferenceApproved
    // 之後才呼叫；未核准revision時完全不會走到這裡、不會改變任何硬體狀態。

    std::optional<ExecutionPlan> activePlan;
    std::optional<RobotPoseABC> postStrikeActual;
    bool pullExtendCommandCompleted = false;
    bool pullRetractCommandCompleted = false;
    bool pullPreparationUnknownUnsafe = false;
    const auto step = [](const RobotAdapterResult& result) {
        return OfflineStepResult{
            result.status == RobotAdapterStatus::UnknownUnsafe
                ? OfflineStepStatus::UnknownUnsafe
                : (result.succeeded()
                      ? OfflineStepStatus::Success
                      : OfflineStepStatus::Failure)};
    };

    OfflineExecutionSeam seam;
    seam.confirmPreparedForCycle = [&] {
        return BilliardApp::confirmRobotReadyReadOnly(robotController, config);
    };
    seam.confirmStandbyReferenceApproved = [&]() -> OfflineStepResult {
        return {BilliardConfig::STANDBY_JOINT_REFERENCE.isValid()
            ? OfflineStepStatus::Success
            : OfflineStepStatus::Failure};
    };
    seam.prepareHardwareForMotion = [&] {
        return BilliardApp::prepareRobotHardwareForMotion(robotController, config);
    };
    seam.isAtStandby = [&]() -> std::optional<bool> {
        const RobotBoolAdapterResult result = robotController.isAtConfiguredJoint(
            BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg,
            BilliardConfig::STANDBY_JOINT_TOLERANCE_DEG);
        if (result.status != RobotAdapterStatus::Success || !result.value) {
            return std::nullopt;
        }
        return *result.value;
    };
    seam.returnToStandby = [&] {
        return step(robotController.checkedConfiguredJointPtp(
            BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg, config));
    };
    seam.confirmStandbyStopped = [&] {
        return step(robotController.confirmStopped());
    };
    seam.moveToCameraPose = [&] {
        return step(robotController.checkedConfiguredJointPtp(
            BilliardConfig::CAMERA_JOINT, config));
    };
    seam.confirmCameraPoseStopped = [&]() -> OfflineStepResult {
        // confirmStopped()只確認「目前沒在動」，不確認「真的到了
        // CameraPose」——checkedConfiguredJointPtp的moveToAxis若被控制器
        // 拒絕/中途中止，手臂會停在別的姿態，但仍然「沒在動」，若只看
        // confirmStopped()會被誤判成已到位，直接進CameraSettling／開始
        // 拍照。這裡比照confirmSafeAtCameraPose既有作法，額外用
        // isAtConfiguredJoint重新讀真實關節角度核對位置。
        const RobotAdapterResult stopped = robotController.confirmStopped();
        if (!stopped.succeeded()) return step(stopped);
        const RobotBoolAdapterResult atCamera = robotController.isAtConfiguredJoint(
            BilliardConfig::CAMERA_JOINT, BilliardConfig::CAMERA_JOINT_TOLERANCE_DEG);
        if (atCamera.status != RobotAdapterStatus::Success || !atCamera.value) {
            return step({atCamera.status, atCamera.sdkCode});
        }
        if (!*atCamera.value) {
            return {OfflineStepStatus::Failure};
        }
        return step(stopped);
    };
    seam.settleCamera = services.settleCamera;
    // 三態：CameraPose是否確認安全（read-only）。共用於「重連後再開capture
    // window前」與「重試截止後決定收尾」兩處，語意完全相同，不應各自實作。
    enum class CameraPoseSafetyCheck { AllSafe, ManualRecoveryRequired, UnknownUnsafe };
    const auto confirmSafeAtCameraPose = [&]() -> CameraPoseSafetyCheck {
        const RobotBoolAdapterResult atCamera = robotController.isAtConfiguredJoint(
            BilliardConfig::CAMERA_JOINT, BilliardConfig::CAMERA_JOINT_TOLERANCE_DEG);
        if (atCamera.status == RobotAdapterStatus::UnknownUnsafe) {
            return CameraPoseSafetyCheck::UnknownUnsafe;
        }
        const RobotAdapterResult stopped = robotController.confirmStopped();
        if (stopped.status == RobotAdapterStatus::UnknownUnsafe) {
            return CameraPoseSafetyCheck::UnknownUnsafe;
        }
        const RobotBoolAdapterResult doOff =
            robotController.confirmPneumaticOutputsOff(config);
        if (doOff.status == RobotAdapterStatus::UnknownUnsafe) {
            return CameraPoseSafetyCheck::UnknownUnsafe;
        }
        const bool poseKnownWrong = atCamera.status == RobotAdapterStatus::Success &&
            atCamera.value && !*atCamera.value;
        const bool doKnownOn = doOff.status == RobotAdapterStatus::Success &&
            doOff.value && !*doOff.value;
        if (poseKnownWrong || doKnownOn) {
            return CameraPoseSafetyCheck::ManualRecoveryRequired;
        }
        const bool allConfirmedSafe =
            atCamera.status == RobotAdapterStatus::Success && atCamera.value &&
            *atCamera.value && stopped.succeeded() &&
            doOff.status == RobotAdapterStatus::Success && doOff.value && *doOff.value;
        return allConfirmedSafe
            ? CameraPoseSafetyCheck::AllSafe
            : CameraPoseSafetyCheck::UnknownUnsafe;
    };
    seam.acquireExecutionPlan = [&]() -> PlanningPhaseResult {
        const unsigned long retryCutoffMs =
            BilliardConfig::SHOT_CYCLE_TIMING.planningRetryCutoffMs;
        const auto pastCutoff = [&] {
            return deadline.elapsedMs().count() >=
                static_cast<long long>(retryCutoffMs);
        };
        const auto safeEndOrRecoveryAfterCutoff = [&]() -> PlanningPhaseResult {
            switch (confirmSafeAtCameraPose()) {
            case CameraPoseSafetyCheck::AllSafe:
                return {PlanningPhaseStatus::NoPlanSafeEnd, std::nullopt};
            case CameraPoseSafetyCheck::ManualRecoveryRequired:
                return {PlanningPhaseStatus::ManualRecoveryRequired, std::nullopt};
            default:
                return {PlanningPhaseStatus::UnknownUnsafe, std::nullopt};
            }
        };

        while (true) {
            while (!services.isVisionConnected()) {
                if (pastCutoff()) {
                    cout << "[連線] Python視覺連線在planningRetryCutoff內"
                         << "未成功，安全結束這一輪" << endl;
                    return safeEndOrRecoveryAfterCutoff();
                }
                cout << "[連線] 嘗試連線Python視覺..." << endl;
                const VisionConnectResult attempt = services.connectVision();
                cout << "[連線] status=" << static_cast<int>(attempt.status)
                     << " socketError=" << attempt.socketError << endl;
                if (attempt.status == VisionConnectStatus::Connected) break;
                if (attempt.status == VisionConnectStatus::NonRetriable) {
                    cout << "[連線] NonRetriable，放棄這一輪" << endl;
                    return {PlanningPhaseStatus::Failure, std::nullopt};
                }
                if (services.sleepMs) {
                    services.sleepMs(BilliardConfig::VISION_RECONNECT_POLL_INTERVAL_MS);
                }
                if (pastCutoff()) {
                    cout << "[連線] Python視覺連線在planningRetryCutoff內"
                         << "未成功，安全結束這一輪" << endl;
                    return safeEndOrRecoveryAfterCutoff();
                }
            }
            if (pastCutoff()) return safeEndOrRecoveryAfterCutoff();

            // 6.2：連線已知（初次或重連）後，重開capture window前必須
            // read-only重新確認CameraPose／已停止／DO OFF，不可假設沒動過。
            switch (confirmSafeAtCameraPose()) {
            case CameraPoseSafetyCheck::AllSafe:
                break;
            case CameraPoseSafetyCheck::ManualRecoveryRequired:
                return {PlanningPhaseStatus::ManualRecoveryRequired, std::nullopt};
            default:
                return {PlanningPhaseStatus::UnknownUnsafe, std::nullopt};
            }

            if (!services.flushStaleVisionBuffer().succeeded() ||
                !services.resetCycleAccumulation().succeeded() ||
                !services.openCaptureWindow(cycleIdentity).succeeded()) {
                return {PlanningPhaseStatus::Failure, std::nullopt};
            }

            const OfflinePhase1Result phase1 = services.runPhase1();
            // runPhase1()一返回（成功或失敗），這個本地capture window已經結束。
            // V1不向Python新增控制訊息；這個callback只保留為C++內部生命週期接縫。
            if (services.closeCaptureWindow) {
                services.closeCaptureWindow(cycleIdentity);
            }
            if (!phase1.isValid()) {
                return {PlanningPhaseStatus::Failure, std::nullopt};
            }
            if (phase1.status == OfflinePhase1Status::PipelineFailure) {
                if (phase1.failureKind == OfflinePhase1FailureKind::RetryWindowExpired) {
                    return safeEndOrRecoveryAfterCutoff();
                }
                if (phase1.failureKind != OfflinePhase1FailureKind::TransportDisruption) {
                    return {PlanningPhaseStatus::Failure, std::nullopt};
                }
                if (pastCutoff()) return safeEndOrRecoveryAfterCutoff();
                continue;
            }
            if (phase1.status == OfflinePhase1Status::NoPlan) {
                const PlanningResult* current = services.currentPlanningResult();
                const NoPlan* noPlan = current && current->isValid()
                    ? std::get_if<NoPlan>(&current->value())
                    : nullptr;
                if (!noPlan) {
                    return {PlanningPhaseStatus::Failure, std::nullopt};
                }
                if (noPlan->reason == NoPlanReason::NoEligibleTarget) {
                    return {PlanningPhaseStatus::NoPlanSafeEnd, std::nullopt};
                }
                if (noPlan->reason == NoPlanReason::InvalidBrainConfiguration ||
                    noPlan->reason == NoPlanReason::NumericalPlanningFailure) {
                    return {PlanningPhaseStatus::Failure, std::nullopt};
                }
                if (pastCutoff()) {
                    return {PlanningPhaseStatus::NoPlanSafeEnd, std::nullopt};
                }
                continue;
            }

            const PlanningResult* phase1Result = services.currentPlanningResult();
            if (!phase1Result || !phase1Result->isValid()) {
                return {PlanningPhaseStatus::Failure, std::nullopt};
            }
            const Phase1ExecutionCandidates& candidates =
                phase1Result->executionCandidates();
            std::optional<ExecutionPlan> found;
            bool hardFailure = false;
            bool candidateUnknownUnsafe = false;
            const auto tryCandidates = [&](const std::vector<ShotPlan>& plans,
                                           bool potsExhausted) {
                for (const ShotPlan& shot : plans) {
                    cout << "[ShotPlan] 目標球=" << shot.selectedTarget.ballNumber
                         << "  類型=" << static_cast<int>(shot.type)
                         << "  母球位置=(" << shot.source.cueBallSnapshot.x
                         << "," << shot.source.cueBallSnapshot.y << ")"
                         << "  擊球方向=(" << shot.shotDirectionXY.x << ","
                         << shot.shotDirectionXY.y << ")" << endl;
                    // 對單一(shot, forcedStrikeMode)組合建ExecutionPlan；
                    // nullopt代表local-skip理由（下一個forcedStrikeMode或
                    // candidate都可能有用），hardFailure旗標另外代表
                    // 硬體/設定/數值類失敗，呼叫端必須立刻放棄整個candidate
                    // 搜尋，不得再嘗試對側StrikeMode。
                    const auto attemptPlan =
                        [&](std::optional<StrikeMode> forcedStrikeMode)
                            -> std::optional<ExecutionPlan> {
                        cout << "[規劃] 計算ExecutionPlan（P2-01姿態搜尋"
                             << (forcedStrikeMode
                                     ? (*forcedStrikeMode == StrikeMode::Push
                                            ? "，強制Push）..."
                                            : "，強制Pull）...")
                                     : "）...")
                             << endl;
                        const ExecutionPlanResult planned =
                            services.buildExecutionPlanForShot(
                                shot, potsExhausted, forcedStrikeMode);
                        if (!planned.isValid()) {
                            cout << "[硬失敗] ExecutionPlanResult invariant失敗"
                                 << endl;
                            hardFailure = true;
                            return std::nullopt;
                        }
                        if (planned.status() != ExecutionPlanStatus::Success ||
                            !planned.value()) {
                            const ExecutionPlanFailureReason reason =
                                planned.diagnostic()
                                ? planned.diagnostic()->reason
                                : ExecutionPlanFailureReason::
                                      InvalidExecutionPlanValue;
                            cout << "[失敗] ExecutionPlan建立失敗，status="
                                 << static_cast<int>(planned.status())
                                 << "  reason=" << static_cast<int>(reason)
                                 << endl;
                            if (reason == ExecutionPlanFailureReason::
                                    FixedForceEnvelopeRejected ||
                                reason == ExecutionPlanFailureReason::
                                    NoAcceptedPoseCandidate ||
                                reason == ExecutionPlanFailureReason::
                                    RearObstacleBlocked) {
                                return std::nullopt;
                            }
                            hardFailure = true;
                            return std::nullopt;
                        }
                        cout << "[ExecutionPlan已建立] strikeMode="
                             << (planned.value()->strikeMode == StrikeMode::Push
                                     ? "Push" : "Pull")
                             << "  strikeReadyPose=("
                             << planned.value()->strikeReadyPose.x << ","
                             << planned.value()->strikeReadyPose.y << ","
                             << planned.value()->strikeReadyPose.z << ")"
                             << endl;
                        return *planned.value();
                    };
                    std::optional<ExecutionPlan> planned = attemptPlan(std::nullopt);
                    if (hardFailure) return;
                    if (!planned) {
                        cout << "[跳過] 換下一個候選" << endl;
                        continue;
                    }
                    cout << "[確認] preflight可達性確認中..." << endl;
                    RobotAdapterResult preflight =
                        robotController.preflightExecution(*planned, config);
                    // preflight是硬體可達性檢查，跟MotionPlanner自己的pose
                    // search是不同層級的失敗來源。NotReachable且原本規劃是
                    // Push時，用同一個execution direction重建強制Pull的
                    // ExecutionPlan再試一次preflight，才真的放棄這個
                    // candidate；Pull只做為Push的對側重試，反向不需要
                    // （既有Push優先、Pull為fallback的政策）。UnknownUnsafe
                    // 或其他硬失敗不觸發這個重試。
                    if (preflight.status == RobotAdapterStatus::NotReachable &&
                        planned->strikeMode == StrikeMode::Push) {
                        cout << "[preflight] Push NotReachable，改試Pull..."
                             << endl;
                        const std::optional<ExecutionPlan> pullPlanned =
                            attemptPlan(StrikeMode::Pull);
                        if (hardFailure) return;
                        if (pullPlanned) {
                            planned = pullPlanned;
                            cout << "[確認] Pull preflight可達性確認中..."
                                 << endl;
                            preflight = robotController.preflightExecution(
                                *planned, config);
                        }
                    }
                    if (preflight.status == RobotAdapterStatus::NotReachable) {
                        cout << "[preflight] NotReachable，換下一個候選"
                             << endl;
                        continue;
                    }
                    if (preflight.status == RobotAdapterStatus::UnknownUnsafe) {
                        cout << "[preflight] UnknownUnsafe，latch" << endl;
                        candidateUnknownUnsafe = true;
                        return;
                    }
                    if (!preflight.succeeded()) {
                        cout << "[preflight] 硬失敗，status="
                             << static_cast<int>(preflight.status) << endl;
                        hardFailure = true;
                        return;
                    }
                    cout << "[選定] preflight通過，採用這個候選執行" << endl;
                    found = *planned;
                    return;
                }
            };
            cout << "[候選] rankedPotPlans（共" << candidates.rankedPotPlans.size()
                 << "筆）..." << endl;
            tryCandidates(candidates.rankedPotPlans, false);
            if (!found && !hardFailure && !candidateUnknownUnsafe) {
                cout << "[候選] legalContactPlans（共"
                     << candidates.legalContactPlans.size() << "筆）..." << endl;
                tryCandidates(candidates.legalContactPlans, true);
            }
            if (!found && !hardFailure && !candidateUnknownUnsafe) {
                // Pot與LegalContact都窮盡才會有候選；立刻嘗試，不等
                // planningRetryCutoff。
                cout << "[候選] cueBallContactOnlyPlans（共"
                     << candidates.cueBallContactOnlyPlans.size() << "筆）..."
                     << endl;
                tryCandidates(candidates.cueBallContactOnlyPlans, true);
            }
            if (!found && !hardFailure && !candidateUnknownUnsafe &&
                candidates.cueBallContactOnlyPlans.empty()) {
                // Phase1當初判定rankedPotPlans/legalContactPlans幾何可行，
                // 所以沒有預先生成cueBallContactOnlyPlans；但它們剛剛在
                // P2-01姿態搜尋／硬體可達性檢查全部失敗（例如母球位置在
                // 手臂實際可達範圍邊緣）。現場用同一個PlanningSourceAudit
                // 補生成最後一層保底，讓母球至少有安全推出的機會，不必
                // 整輪直接失敗、回Unknown。
                const PlanningSourceAudit* fallbackSource =
                    !candidates.rankedPotPlans.empty()
                    ? &candidates.rankedPotPlans.front().source
                    : (!candidates.legalContactPlans.empty()
                        ? &candidates.legalContactPlans.front().source
                        : nullptr);
                // 沒有geometry就不產生候選（fail closed）：這裡的方向
                // 篩選需要ballDiameterMm/collisionMarginMm才能對
                // otherBallsSnapshot做前方路徑碰撞檢查，不能在沒有這組
                // 資料時盲目產生候選，讓母球有機會直接撞進其他球。
                const std::optional<ResolvedTableGeometry>* resolvedGeometry =
                    services.currentResolvedTableGeometry
                    ? services.currentResolvedTableGeometry()
                    : nullptr;
                if (fallbackSource && resolvedGeometry && *resolvedGeometry) {
                    const std::vector<ShotPlan> onDemandFallback =
                        BilliardAlgorithm::
                            generateCueBallContactOnlyExecutionFallback(
                                *fallbackSource, **resolvedGeometry);
                    cout << "[保底] rankedPotPlans/legalContactPlans全部失敗，"
                         << "現場補生成CueBallContactOnly候選（共"
                         << onDemandFallback.size() << "筆）重試..." << endl;
                    tryCandidates(onDemandFallback, true);
                }
            }
            if (candidateUnknownUnsafe) {
                return {PlanningPhaseStatus::UnknownUnsafe, std::nullopt};
            }
            if (hardFailure) {
                return {PlanningPhaseStatus::Failure, std::nullopt};
            }
            if (!found) {
                if (pastCutoff()) {
                    return {PlanningPhaseStatus::NoPlanSafeEnd, std::nullopt};
                }
                continue;
            }
            if (deadline.remainingMs(BilliardConfig::SHOT_CYCLE_TIMING.shotDeadlineMs)
                    .count() <
                static_cast<long long>(
                    BilliardConfig::SHOT_CYCLE_TIMING.minimumExecutionReserveMs)) {
                return {PlanningPhaseStatus::NoPlanSafeEnd, std::nullopt};
            }
            activePlan = found;
            return {PlanningPhaseStatus::PlanReady, found};
        }
    };
    seam.validateStrikeReady = [&](const ExecutionPlan& plan) {
        const RobotAdapterResult activated =
            robotController.activateConfiguredToolAndBase(plan, config);
        if (!activated.succeeded()) return step(activated);
        if (plan.strikeMode != StrikeMode::Pull) {
            return OfflineStepResult{OfflineStepStatus::Success};
        }
        const RobotAdapterResult stopped = robotController.confirmStopped();
        if (!stopped.succeeded()) return step(stopped);
        // Pull的DO1伸出改到strikeReady+confirmStopped之後才做（見
        // seam.runPneumatic），這裡只確認DO1/DO2已知OFF，移動途中不會伸出
        // 推桿。
        const RobotAdapterResult outputsKnown =
            robotController.establishSafeOutputsOff(config);
        if (outputsKnown.status == RobotAdapterStatus::UnknownUnsafe) {
            pullPreparationUnknownUnsafe = true;
        }
        return step(outputsKnown);
    };
    seam.moveToStrikeReady = [&](const ExecutionPlan& plan) -> OfflineStepResult {
        const RobotAdapterResult approach =
            robotController.checkedPtp(plan, plan.safeApproachPose, config);
        if (approach.status == RobotAdapterStatus::UnknownUnsafe) {
            return {OfflineStepStatus::UnknownUnsafe};
        }
        if (!approach.succeeded()) return {OfflineStepStatus::Failure};
        const RobotPoseAdapterResult actual =
            robotController.readActualPose(plan, config);
        if (!actual.isValid() || !actual.value) {
            return {actual.status == RobotAdapterStatus::UnknownUnsafe
                ? OfflineStepStatus::UnknownUnsafe
                : OfflineStepStatus::Failure};
        }
        const RobotAdapterResult lin = robotController.checkedLin(
            plan, *actual.value, plan.strikeReadyPose, config);
        if (lin.status == RobotAdapterStatus::UnknownUnsafe) {
            return {OfflineStepStatus::UnknownUnsafe};
        }
        return {lin.succeeded() ? OfflineStepStatus::Success : OfflineStepStatus::Failure};
    };
    seam.confirmStrikeReadyStopped = [&] {
        return step(robotController.confirmStopped());
    };
    seam.runPneumatic = [&](const ExecutionPlan& plan) -> PneumaticCompletionResult {
        if (plan.strikeMode == StrikeMode::Pull) {
            // Pull的DO1伸出排在這裡（strikeReady+confirmStopped之後），
            // 不在移動到strikeReady之前伸出。
            const RealPneumaticResult extended =
                robotController.pulseExtend(plan, config);
            if (extended.status == RealPneumaticStatus::UnknownUnsafe) {
                pullPreparationUnknownUnsafe = true;
                return {PneumaticCompletionStatus::UnknownUnsafe, std::nullopt};
            }
            if (extended.status != RealPneumaticStatus::Completed) {
                return mapRealPneumaticResult(extended);
            }
            pullExtendCommandCompleted = true;
            const RobotAdapterResult directionWait =
                robotController.waitDirectionChangeDelay(plan, config);
            if (directionWait.status == RobotAdapterStatus::UnknownUnsafe) {
                return {PneumaticCompletionStatus::UnknownUnsafe, std::nullopt};
            }
            if (!directionWait.succeeded()) {
                return PneumaticCompletionResult{
                    PneumaticCompletionStatus::Failure, std::nullopt};
            }
            const RealPneumaticResult retracted =
                robotController.pulseRetract(plan, config);
            pullRetractCommandCompleted =
                retracted.status == RealPneumaticStatus::Completed;
            if (!pullRetractCommandCompleted) {
                return mapRealPneumaticResult(retracted);
            }
            const RobotAdapterResult wait =
                robotController.waitMechanismCompletion(plan, config);
            if (wait.status == RobotAdapterStatus::UnknownUnsafe) {
                return {PneumaticCompletionStatus::UnknownUnsafe, std::nullopt};
            }
            return wait.succeeded()
                ? mapRealPneumaticResult(retracted)
                : PneumaticCompletionResult{
                    PneumaticCompletionStatus::Failure, std::nullopt};
        }
        const RealPneumaticResult extended =
            robotController.pulseExtend(plan, config);
        if (extended.status != RealPneumaticStatus::Completed) {
            return mapRealPneumaticResult(extended);
        }
        const RobotAdapterResult directionWait =
            robotController.waitDirectionChangeDelay(plan, config);
        if (directionWait.status == RobotAdapterStatus::UnknownUnsafe) {
            return {PneumaticCompletionStatus::UnknownUnsafe, std::nullopt};
        }
        if (!directionWait.succeeded()) {
            return PneumaticCompletionResult{
                PneumaticCompletionStatus::Failure, std::nullopt};
        }
        const RealPneumaticResult retracted =
            robotController.pulseRetract(plan, config);
        if (retracted.status != RealPneumaticStatus::Completed) {
            return mapRealPneumaticResult(retracted);
        }
        const RobotAdapterResult completionWait =
            robotController.waitMechanismCompletion(plan, config);
        if (completionWait.status == RobotAdapterStatus::UnknownUnsafe) {
            return {PneumaticCompletionStatus::UnknownUnsafe, std::nullopt};
        }
        return completionWait.succeeded()
            ? mapRealPneumaticResult(retracted)
            : PneumaticCompletionResult{
                PneumaticCompletionStatus::Failure, std::nullopt};
    };
    seam.readActualPose = [&]() -> RobotPoseAdapterResult {
        if (!activePlan) {
            return {RobotAdapterStatus::InvalidConfiguration, -1, std::nullopt};
        }
        const RobotPoseAdapterResult actual =
            robotController.readActualPose(*activePlan, config);
        postStrikeActual =
            actual.isValid() && actual.status == RobotAdapterStatus::Success
                ? actual.value
                : std::nullopt;
        return actual;
    };
    seam.checkSafeLiftLinearPath = [&](const RobotPoseABC& actual,
                                       const RobotPoseABC& target) {
        if (!activePlan) {
            return LinearPathCheckResult{LinearPathCheckStatus::ApiFailure, false};
        }
        const RobotAdapterResult result = robotController.checkVerticalSafeLift(
            *activePlan, actual, target, config);
        if (result.status == RobotAdapterStatus::UnknownUnsafe) {
            return LinearPathCheckResult{LinearPathCheckStatus::UnknownUnsafe, false};
        }
        return LinearPathCheckResult{
            result.status == RobotAdapterStatus::SdkFailure ||
                    result.status == RobotAdapterStatus::NotConnected
                ? LinearPathCheckStatus::ApiFailure
                : LinearPathCheckStatus::Success,
            result.succeeded()};
    };
    seam.moveSafeLiftLinear = [&](const RobotPoseABC& target) {
        if (!activePlan || !postStrikeActual) {
            return OfflineStepResult{OfflineStepStatus::Failure};
        }
        return step(robotController.checkedVerticalSafeLift(
            *activePlan, *postStrikeActual, target, config));
    };
    seam.confirmSafeLiftStopped = [&](const RobotPoseABC&) {
        return step(robotController.confirmStopped());
    };
    seam.returnToStandbyAfterStrike = [&](const ExecutionPlan& plan) {
        return step(robotController.checkedJointPtp(
            plan, plan.standbyJointReference, config));
    };
    seam.confirmStandbyReturnStopped = [&] {
        return step(robotController.confirmStopped());
    };
    ExecutionCycleResult result = runOfflineSingleCycle(runtime, cycleIdentity, seam);
    if (pullPreparationUnknownUnsafe) {
        runtime.state = ExecutionCycleState::UnknownUnsafe;
        return {ExecutionCycleStatus::UnknownUnsafe, std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::PneumaticStateUnknown,
                ExecutionCycleState::UnknownUnsafe}};
    }
    if (pullExtendCommandCompleted && !pullRetractCommandCompleted &&
        result.status != ExecutionCycleStatus::UnknownUnsafe) {
        runtime.state = ExecutionCycleState::ManualRecoveryRequired;
        result = {ExecutionCycleStatus::SafeFailure, std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::PneumaticFailed,
                ExecutionCycleState::ManualRecoveryRequired}};
    }
    return result;
}

bool BilliardApp::initialize() {
    if (visionClient.configurationStatus() == SocketConfigurationStatus::ConfigurationMissing ||
        visionParser.configurationStatus() == SingleFrameStatus::ConfigurationMissing) {
        cout << "[ConfigurationMissing] P1-03 production vision frame size、"
             << "receive timeout或Base0 observation bounds尚未核准。" << endl;
        return false;
    }
    if (visionClient.configurationStatus() == SocketConfigurationStatus::InvalidConfiguration) {
        cout << "[InvalidConfiguration] P1-03 frame size or receive timeout is invalid." << endl;
        return false;
    }
    if (visionParser.configurationStatus() == SingleFrameStatus::InvalidConfiguration) {
        cout << "[InvalidConfiguration] P1-03 Base0 observation bounds are invalid." << endl;
        return false;
    }
    if (const auto stabilityConfigurationFailure = stability.configurationFailure()) {
        cout << "["
             << (*stabilityConfigurationFailure == StabilityFailureReason::ConfigurationMissing
                     ? "ConfigurationMissing"
                     : "InvalidConfiguration")
             << "] P1-04 stability tolerances or maximum event interval are not approved."
             << endl;
        return false;
    }

    // initialize()只做本地設定檢查，不得在H/P可用之前無限等待Python連線；
    // Python連線改在H進入CameraPose階段時，依同一shot deadline建立／確認。
    cout << "[系統] 本地設定檢查完成。H/P立即可用；Python連線將於H進入"
         << "CameraPose階段時依shot deadline建立或確認。" << endl;
    return true;
}

void BilliardApp::run()
{
    std::optional<BilliardConfig::ExecutionPolicyMode> policyMode =
        BilliardConfig::PRODUCTION_RUNTIME_MODE;
    std::optional<std::string> policyRevision;
    std::optional<bool> legalContactAuthorized;
    std::optional<BilliardConfig::ExecutionPolicyMode> motionPlanningPolicyMode;
    std::optional<BilliardConfig::MotionPlanningConfig> motionPlanningConfig =
        BilliardConfig::MOTION_PLANNING_CONFIG;
    std::optional<BilliardConfig::TableGeometryConfig> tableGeometryConfig =
        BilliardConfig::TABLE_GEOMETRY;
    std::optional<BilliardConfig::RealHardwareExecutionConfig> realConfig =
        BilliardConfig::REAL_HARDWARE_EXECUTION_CONFIG;
    RobotController* cycleRobot = &robot;
    std::optional<RealExecutionCycleServices> realServices;
    ConsoleKeyPoll effectiveKeyPoll = productionConsoleKeyPoll();
    ConsoleKeyDownQuery effectiveKeyQuery = productionConsoleKeyDownQuery();
    ShotDeadlineClock deadline;
    deadline.now = [] { return std::chrono::steady_clock::now(); };

    if (BilliardConfig::MOTION_PLANNING_CONFIG) {
        motionPlanningPolicyMode =
            BilliardConfig::MOTION_PLANNING_CONFIG->policyMode;
        policyRevision =
            BilliardConfig::MOTION_PLANNING_CONFIG->executionPolicyRevision;
        legalContactAuthorized =
            BilliardConfig::MOTION_PLANNING_CONFIG
                ->legalContactExecutionAuthorized;
    }
#ifdef BILLIARDS_P2_03_TEST_SEAM
    if (runTestSeam) {
        policyMode = runTestSeam->policyMode;
        policyRevision = runTestSeam->policyRevision;
        legalContactAuthorized =
            runTestSeam->legalContactExecutionAuthorized;
        realConfig = runTestSeam->realConfig;
        cycleRobot = runTestSeam->robot;
        realServices = runTestSeam->realServices;
        if (runTestSeam->motionPlanningConfig) {
            motionPlanningConfig = runTestSeam->motionPlanningConfig;
        }
        if (runTestSeam->tableGeometryConfig) {
            tableGeometryConfig = runTestSeam->tableGeometryConfig;
        }
        motionPlanningPolicyMode = runTestSeam->motionPlanningPolicyMode;
        if (runTestSeam->consoleKeyPoll) effectiveKeyPoll = runTestSeam->consoleKeyPoll;
        if (runTestSeam->consoleKeyDownQuery) {
            effectiveKeyQuery = runTestSeam->consoleKeyDownQuery;
        }
        if (runTestSeam->fakeNow) deadline.now = runTestSeam->fakeNow;
    }
#endif

    // DI1實體Start按鈕只在真正production路徑接上；P2-03 test seam已經
    // 明確注入自己的consoleKeyPoll/consoleKeyDownQuery做決定性測試，
    // 不應該再被DI讀取（通常對fake robot安全回傳nullopt/no-op，但刻意
    // 排除更明確）。
#ifdef BILLIARDS_P2_03_TEST_SEAM
    if (!runTestSeam)
#endif
    {
        effectiveKeyPoll = combinedStartPoll(effectiveKeyPoll, *cycleRobot, realConfig);
        effectiveKeyQuery = combinedStartQuery(effectiveKeyQuery, *cycleRobot, realConfig);
    }

    if (!policyMode) {
        return;
    }

    OfflineExecutionRuntime executionRuntime;
    const auto productionServices = [&] {
        RealExecutionCycleServices services;
        const bool injectedServices = realServices.has_value();
        if (realServices) {
            services = *realServices;
        } else {
                services.settleCamera = [] {
                    Sleep(BilliardConfig::CAMERA_SETTLE_MS);
                    return OfflineStepResult{OfflineStepStatus::Success};
                };
                services.flushStaleVisionBuffer = [&] {
                    return OfflineStepResult{
                        visionClient.flushBuffer().status ==
                                SocketOperationStatus::Success
                            ? OfflineStepStatus::Success
                            : OfflineStepStatus::Failure};
                };
                services.resetCycleAccumulation = [&] {
                    stability.reset(StabilityFailureReason::ExplicitReset);
                    pendingPlanningResult.reset();
                    pendingResolvedTableGeometry.reset();
                    return OfflineStepResult{OfflineStepStatus::Success};
                };
                services.openCaptureWindow = [&](ShotCycleIdentity cycleIdentity) {
                    if (cycleIdentity == 0) {
                        return OfflineStepResult{OfflineStepStatus::Failure};
                    }
                    receiveEventFactory.beginCycle(
                        visionClient.connectionIdentity(), cycleIdentity);
                    if (!receiveEventFactory.openCaptureWindow(
                            visionClient.connectionIdentity(), cycleIdentity)) {
                        return OfflineStepResult{OfflineStepStatus::Failure};
                    }
                    return OfflineStepResult{OfflineStepStatus::Success};
                };
                services.closeCaptureWindow = [](ShotCycleIdentity) {
                };
                services.runPhase1 = [&] {
                    while (true) {
                        // runPhase1本身可能持續收到資料但一直不穩定，若不在
                        // 此內部迴圈檢查deadline，外層只能等它return才檢查，
                        // 可能讓整輪超過15秒／10秒門檻而不自知。
                        if (deadline.elapsedMs().count() >=
                            static_cast<long long>(
                                BilliardConfig::SHOT_CYCLE_TIMING
                                    .planningRetryCutoffMs)) {
                            return OfflinePhase1Result{
                                OfflinePhase1Status::PipelineFailure,
                                OfflinePhase1FailureKind::RetryWindowExpired};
                        }
                        const SocketReceiveResult received =
                            visionClient.receiveFrame();
                        if (!received.isValid()) {
                            return OfflinePhase1Result{
                                OfflinePhase1Status::PipelineFailure,
                                OfflinePhase1FailureKind::NonRetriable};
                        }
                        if (received.status != SocketReceiveStatus::FrameReady ||
                            !received.frame) {
                            const bool transport =
                                received.status == SocketReceiveStatus::CleanClose ||
                                received.status == SocketReceiveStatus::SocketError ||
                                received.status == SocketReceiveStatus::NotConnected ||
                                received.status == SocketReceiveStatus::TimedOut;
                            return OfflinePhase1Result{
                                OfflinePhase1Status::PipelineFailure,
                                transport
                                    ? OfflinePhase1FailureKind::TransportDisruption
                                    : OfflinePhase1FailureKind::NonRetriable};
                        }
                        const ReceiveEventResult eventResult =
                            receiveEventFactory.accept(
                                *received.frame,
                                visionClient.connectionIdentity(),
                                chrono::steady_clock::now());
                        if (!eventResult.isValid()) {
                            return OfflinePhase1Result{
                                OfflinePhase1Status::PipelineFailure,
                                OfflinePhase1FailureKind::NonRetriable};
                        }
                        if (eventResult.status() != ReceiveEventStatus::Success ||
                            !eventResult.value()) {
                            // ConnectionMismatch在正常流程理論上不該發生
                            // （每次accept都用當下connectionIdentity()），
                            // 出現代表identity一致性有問題，不是單純transport
                            // 中斷，必須fail closed而非重試。
                            return OfflinePhase1Result{
                                OfflinePhase1Status::PipelineFailure,
                                OfflinePhase1FailureKind::NonRetriable};
                        }
                        if (!processReceiveEvent(*eventResult.value())) {
                            if (!pendingPlanningResult ||
                                !pendingPlanningResult->isValid()) {
                                return OfflinePhase1Result{
                                    OfflinePhase1Status::PipelineFailure,
                                    OfflinePhase1FailureKind::NonRetriable};
                            }
                            const auto& candidates =
                                pendingPlanningResult->executionCandidates();
                            return OfflinePhase1Result{
                                std::holds_alternative<ShotPlan>(
                                    pendingPlanningResult->value()) ||
                                    !candidates.rankedPotPlans.empty() ||
                                    !candidates.legalContactPlans.empty()
                                    ? OfflinePhase1Status::ShotPlanReady
                                    : OfflinePhase1Status::NoPlan};
                        }
                    }
                };
                services.isVisionConnected = [&] {
                    return visionClient.isConnected();
                };
                services.connectVision = [&]() -> VisionConnectResult {
                    // connect()上限＝min(VISION_RECEIVE_TIMEOUT_MS,
                    // 距離planning retry cutoff的剩餘時間)，沿用同一個
                    // ShotDeadlineClock換算，不建立第二個獨立deadline。
                    std::optional<unsigned long> boundMs;
                    if (BilliardConfig::VISION_RECEIVE_TIMEOUT_MS) {
                        const auto remaining = deadline.remainingMs(
                            BilliardConfig::SHOT_CYCLE_TIMING.planningRetryCutoffMs);
                        boundMs = (std::min)(
                            *BilliardConfig::VISION_RECEIVE_TIMEOUT_MS,
                            static_cast<unsigned long>(remaining.count()));
                    }
                    const SocketConnectResult result =
                        visionClient.connectToServerResult(
                            BilliardConfig::VISION_SERVER_IP,
                            BilliardConfig::VISION_SERVER_PORT,
                            boundMs);
                    if (result.status == SocketConnectStatus::Success) {
                        return {VisionConnectStatus::Connected, 0};
                    }
                    const bool retriable =
                        result.status == SocketConnectStatus::ConnectError;
                    return {
                        retriable ? VisionConnectStatus::Retriable
                                  : VisionConnectStatus::NonRetriable,
                        result.socketError};
                };
                services.sleepMs = [](unsigned long ms) { Sleep(ms); };
        }
#ifdef BILLIARDS_P2_03_TEST_SEAM
        if (runTestSeam && !runTestSeam->currentCycleFrames.empty() &&
            runTestSeam->connectionIdentity) {
            const ConnectionIdentity connectionIdentity =
                *runTestSeam->connectionIdentity;
            services.settleCamera = [] {
                return OfflineStepResult{OfflineStepStatus::Success};
            };
            services.flushStaleVisionBuffer = [] {
                return OfflineStepResult{OfflineStepStatus::Success};
            };
            services.resetCycleAccumulation = [&] {
                stability.reset(StabilityFailureReason::ExplicitReset);
                pendingPlanningResult.reset();
                pendingResolvedTableGeometry.reset();
                return OfflineStepResult{OfflineStepStatus::Success};
            };
            services.openCaptureWindow =
                [&, connectionIdentity](ShotCycleIdentity cycleIdentity) {
                if (cycleIdentity == 0) {
                    return OfflineStepResult{OfflineStepStatus::Failure};
                }
                receiveEventFactory.beginCycle(
                    connectionIdentity, cycleIdentity);
                return OfflineStepResult{
                    receiveEventFactory.openCaptureWindow(
                        connectionIdentity, cycleIdentity)
                        ? OfflineStepStatus::Success
                        : OfflineStepStatus::Failure};
            };
            services.runPhase1 = [&, connectionIdentity] {
                for (const std::string& frame :
                    runTestSeam->currentCycleFrames) {
                    const ReceiveEventResult eventResult =
                        receiveEventFactory.accept(
                            frame,
                            connectionIdentity,
                            chrono::steady_clock::now());
                    if (!eventResult.isValid() ||
                        eventResult.status() != ReceiveEventStatus::Success ||
                        !eventResult.value()) {
                        return OfflinePhase1Result{
                            OfflinePhase1Status::PipelineFailure,
                            OfflinePhase1FailureKind::NonRetriable};
                    }
                    if (!processReceiveEvent(*eventResult.value())) {
                        if (!pendingPlanningResult ||
                            !pendingPlanningResult->isValid()) {
                            return OfflinePhase1Result{
                                OfflinePhase1Status::PipelineFailure,
                                OfflinePhase1FailureKind::NonRetriable};
                        }
                        const auto& candidates =
                            pendingPlanningResult->executionCandidates();
                        return OfflinePhase1Result{
                            std::holds_alternative<ShotPlan>(
                                pendingPlanningResult->value()) ||
                                !candidates.rankedPotPlans.empty() ||
                                !candidates.legalContactPlans.empty()
                                ? OfflinePhase1Status::ShotPlanReady
                                : OfflinePhase1Status::NoPlan};
                    }
                }
                return OfflinePhase1Result{
                    OfflinePhase1Status::PipelineFailure,
                    OfflinePhase1FailureKind::NonRetriable};
            };
            services.isVisionConnected = [] { return true; };
            services.connectVision = [] {
                return VisionConnectResult{VisionConnectStatus::Connected, 0};
            };
            services.sleepMs = [](unsigned long) {};
        }
#endif
        if (!services.currentPlanningResult && !injectedServices) {
            services.currentPlanningResult = [&]() -> const PlanningResult* {
                return pendingPlanningResult ? &*pendingPlanningResult : nullptr;
            };
        }
        if (!services.currentResolvedTableGeometry && !injectedServices) {
            services.currentResolvedTableGeometry =
                [&]() -> const std::optional<ResolvedTableGeometry>* {
                return &pendingResolvedTableGeometry;
            };
        }
        if (!services.buildExecutionPlanForShot && !injectedServices) {
            MotionPlanningChecks hardwareChecks =
                offlineMotionPlanningChecks.value_or(MotionPlanningChecks{});
            hardwareChecks.poseAccepted =
                [cycleRobot, realConfig](const RobotPoseABC& pose) {
                return cycleRobot
                    ? cycleRobot->checkPoseReachable(pose, realConfig)
                    : std::optional<bool>{};
            };
            hardwareChecks.linearPathAccepted =
                [cycleRobot, realConfig](const RobotPoseABC& approach,
                                         const RobotPoseABC& ready) {
                return cycleRobot
                    ? cycleRobot->checkLinearPathAccepted(
                          approach, ready, realConfig)
                    : std::optional<bool>{};
            };
            services.buildExecutionPlanForShot =
                [&, hardwareChecks](
                    const ShotPlan& shot,
                    bool potsExhausted,
                    std::optional<StrikeMode> forcedStrikeMode) {
                return motionPlanner.createExecutionPlan(
                    PlanningResult::shotPlan(shot),
                    tableGeometryConfig,
                    motionPlanningConfig,
                    hardwareChecks,
                    potsExhausted,
                    pendingResolvedTableGeometry,
                    forcedStrikeMode);
            };
        }
        if (!services.buildExecutionPlan) {
            services.buildExecutionPlan = [&] {
                if (!pendingPlanningResult) {
                    return ExecutionPlanResult::rejected(
                        ExecutionPlanStatus::InvalidExecutionPlan,
                        ExecutionPlanFailureReason::InvalidExecutionPlanValue);
                }
                const Phase1ExecutionCandidates& candidates =
                    pendingPlanningResult->executionCandidates();
                const ShotPlan* selected = !candidates.rankedPotPlans.empty()
                    ? &candidates.rankedPotPlans.front()
                    : (!candidates.legalContactPlans.empty()
                        ? &candidates.legalContactPlans.front()
                        : std::get_if<ShotPlan>(
                              &pendingPlanningResult->value()));
                if (!selected) {
                    return ExecutionPlanResult::rejected(
                        ExecutionPlanStatus::NoExecutablePlan,
                        ExecutionPlanFailureReason::NoAcceptedPoseCandidate);
                }
                const bool potCandidatesExhausted =
                    candidates.rankedPotPlans.empty() &&
                    !candidates.legalContactPlans.empty();
                return motionPlanner.createExecutionPlan(
                    PlanningResult::shotPlan(*selected),
                    tableGeometryConfig,
                    motionPlanningConfig,
                    offlineMotionPlanningChecks.value_or(
                        MotionPlanningChecks{}),
                    potCandidatesExhausted,
                    pendingResolvedTableGeometry);
            };
        }
        return services;
    };

    // H與P共用confirmRobotReadyReadOnly／prepareRobotHardwareForMotion，
    // 且順序完全一致：連線→read-only確認→standby revision核准門檻→
    // 才寫入硬體（alarm／tool-base／motor／speed）→才PTP，避免兩套流程
    // 各自實作、日後又漂移出不一致的前置順序或門檻。
    const auto runPOnly = [&] {
        cout << "[P] 收到，準備回Standby..." << endl;
        if (executionRuntime.state != ExecutionCycleState::WaitingForStart ||
            !cycleRobot) {
            cout << "[P] 忽略（狀態非WaitingForStart或無robot）" << endl;
            return;
        }
        if (!cycleRobot->isConnected() &&
            !cycleRobot->connect(BilliardConfig::ARM_IP)) {
            cout << "[P] 失敗（連線失敗）" << endl;
            return;
        }
        const OfflineStepResult readOnly =
            BilliardApp::confirmRobotReadyReadOnly(*cycleRobot, realConfig);
        if (readOnly.status == OfflineStepStatus::UnknownUnsafe) {
            cout << "[P] UnknownUnsafe（read-only確認失敗），latch。" << endl;
            executionRuntime.state = ExecutionCycleState::UnknownUnsafe;
            return;
        }
        if (!readOnly.succeeded()) {
            cout << "[P] 失敗（confirmRobotReadyReadOnly未通過）" << endl;
            return;
        }
        // P不依賴H先跑過、不依賴Python，但仍受核准的standby joint reference
        // 門檻限制：未核准前不得寫入任何硬體狀態、不得移動。
        if (!BilliardConfig::STANDBY_JOINT_REFERENCE.isValid()) {
            cout << "[P] 失敗（STANDBY_JOINT_REFERENCE未核准）" << endl;
            return;
        }
        const OfflineStepResult prepared =
            BilliardApp::prepareRobotHardwareForMotion(*cycleRobot, realConfig);
        if (prepared.status == OfflineStepStatus::UnknownUnsafe) {
            cout << "[P] UnknownUnsafe（prepareRobotHardwareForMotion失敗），"
                 << "latch。" << endl;
            executionRuntime.state = ExecutionCycleState::UnknownUnsafe;
            return;
        }
        if (!prepared.succeeded()) {
            cout << "[P] 失敗（prepareRobotHardwareForMotion未通過）" << endl;
            return;
        }
        cout << "[P] 硬體準備完成，PTP回Standby joint..." << endl;
        const RobotAdapterResult ptp = cycleRobot->checkedConfiguredJointPtp(
            BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg, realConfig);
        if (ptp.status == RobotAdapterStatus::UnknownUnsafe) {
            cout << "[P] UnknownUnsafe（PTP失敗），latch。" << endl;
            executionRuntime.state = ExecutionCycleState::UnknownUnsafe;
            return;
        }
        if (!ptp.succeeded()) {
            cout << "[P] 失敗（PTP未成功，status="
                 << static_cast<int>(ptp.status) << "）" << endl;
            return;
        }
        const RobotAdapterResult confirmed = cycleRobot->confirmStopped();
        if (confirmed.status == RobotAdapterStatus::UnknownUnsafe) {
            cout << "[P] UnknownUnsafe（confirmStopped失敗），latch。" << endl;
            executionRuntime.state = ExecutionCycleState::UnknownUnsafe;
            return;
        }
        // confirmed.succeeded()若為false（例如NotStopped），P不宣稱完成，
        // 也不特別處理；operator可再次嘗試P，或改用完整H流程。
        cout << "[P] "
             << (confirmed.succeeded() ? "已回Standby並確認停止。"
                                        : "PTP完成但confirmStopped未過"
                                          "（可再按一次P，或改用H）。")
             << endl;
    };

    while (executionRuntime.state == ExecutionCycleState::WaitingForStart) {
        const StartControlEvent event =
            pollStartControl(startControlGates, effectiveKeyPoll);
        if (event.standbyEdge) {
            runPOnly();
            if (executionRuntime.state != ExecutionCycleState::WaitingForStart) {
                break;
            }
        }
        if (!event.startEdge) {
            Sleep(BilliardConfig::MOTION_POLL_INTERVAL_MS);
            continue;
        }
        // 使用者2026-08-16要求：H/DI1按下時如果手臂還沒在Standby，只做
        // 跟P鍵完全同一套、未計時的回Standby準備，不算成一次shot cycle、
        // 不消耗shot deadline；確認已經在Standby後，下一次H/DI1才真正
        // 開始計時的cycle。原因：PreparationReturn這段移動時間曾經吃光
        // 15秒的planningRetryCutoff budget，導致vision連線都還沒機會
        // 嘗試就被判定逾時、安全結束，第一次H形同浪費掉。
        if (cycleRobot) {
            const RobotBoolAdapterResult atStandby = cycleRobot->isAtConfiguredJoint(
                BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg,
                BilliardConfig::STANDBY_JOINT_TOLERANCE_DEG);
            if (atStandby.status != RobotAdapterStatus::Success ||
                !atStandby.value || !*atStandby.value) {
                cout << "[H] 尚未在Standby，先做未計時的回Standby準備"
                     << "（不計入shot cycle）..." << endl;
                runPOnly();
                if (executionRuntime.state != ExecutionCycleState::WaitingForStart) {
                    break;
                }
                continue;
            }
        }
        if (nextShotCycleIdentity == 0) {
            cout << "[系統] shot-cycle identity已耗盡，fail closed，需重啟程式。"
                 << endl;
            break;
        }
        const std::uint64_t cycleIdentity = nextShotCycleIdentity++;
        // 計時從接受H的瞬間開始，不是稍後才進入RealHardware分支的時刻。
        deadline.startedAt = deadline.now
            ? deadline.now()
            : std::chrono::steady_clock::time_point{};
        RealExecutionCycleServices services = productionServices();

        if (*policyMode == BilliardConfig::ExecutionPolicyMode::PlanningTest) {
            if (!motionPlanningPolicyMode ||
                *motionPlanningPolicyMode !=
                    BilliardConfig::ExecutionPolicyMode::PlanningTest) {
                return;
            }
            cout << "=== Planning Result ===" << endl;
            if (!services.settleCamera || !services.flushStaleVisionBuffer ||
                !services.resetCycleAccumulation || !services.openCaptureWindow ||
                !services.runPhase1 || !services.buildExecutionPlan ||
                services.settleCamera().status != OfflineStepStatus::Success ||
                services.flushStaleVisionBuffer().status != OfflineStepStatus::Success ||
                services.resetCycleAccumulation().status != OfflineStepStatus::Success ||
                services.openCaptureWindow(cycleIdentity).status !=
                    OfflineStepStatus::Success) {
                cout << "final status=PipelineFailure" << endl;
                cout << "NO HARDWARE EXECUTION" << endl;
                return;
            }
            const OfflinePhase1Result phase1 = services.runPhase1();
            if (phase1.status == OfflinePhase1Status::NoPlan) {
                cout << "final status=NoPlan" << endl;
                cout << "=== Execution Plan ===" << endl;
                cout << "status=NoExecutablePlan" << endl;
                cout << "NO HARDWARE EXECUTION" << endl;
                cout << "CycleCompleted" << endl;
                cout << "WaitingForStart" << endl;
                invalidateVisionCycle(ReceiveEventInvalidationReason::CycleChanged);
                resyncStartControlToIdle(
                    startControlGates, effectiveKeyPoll, effectiveKeyQuery);
                continue;
            }
            if (phase1.status != OfflinePhase1Status::ShotPlanReady) {
                cout << "final status=PipelineFailure" << endl;
                cout << "NO HARDWARE EXECUTION" << endl;
                return;
            }
            if (pendingPlanningResult && pendingPlanningResult->isValid()) {
                if (const auto* shot = std::get_if<ShotPlan>(
                        &pendingPlanningResult->value())) {
                    cout << "target ball=" << shot->selectedTarget.ballNumber << endl;
                    cout << "plan type=" << static_cast<int>(shot->type) << endl;
                    if (const auto* direct =
                            std::get_if<DirectPotShotPlanPayload>(&shot->payload)) {
                        cout << "pocket="
                             << static_cast<int>(direct->candidate.pocketId) << endl;
                        cout << "Direct cue path="
                             << direct->candidate.cuePath.start.x << ","
                             << direct->candidate.cuePath.start.y << " -> "
                             << direct->candidate.cuePath.end.x << ","
                             << direct->candidate.cuePath.end.y << endl;
                    } else if (const auto* kick =
                                   std::get_if<KickPotShotPlanPayload>(
                                       &shot->payload)) {
                        cout << "pocket="
                             << static_cast<int>(kick->candidate.pocketId) << endl;
                        cout << "Kick rail="
                             << static_cast<int>(kick->candidate.railId) << endl;
                        cout << "Kick rebound="
                             << kick->candidate.reboundPoint.x << ","
                             << kick->candidate.reboundPoint.y << endl;
                    }
                }
            }
            const ExecutionPlanResult planResult = services.buildExecutionPlan();
#ifdef BILLIARDS_P2_03_TEST_SEAM
            if (runTestSeam && runTestSeam->executionPlanObserved) {
                runTestSeam->executionPlanObserved(planResult);
            }
#endif
            cout << "=== Execution Plan ===" << endl;
            cout << "status=" << static_cast<int>(planResult.status()) << endl;
            if (!planResult.isValid() ||
                planResult.status() != ExecutionPlanStatus::Success ||
                !planResult.value()) {
                cout << "NO HARDWARE EXECUTION" << endl;
                cout << "CycleCompleted" << endl;
                cout << "WaitingForStart" << endl;
                invalidateVisionCycle(ReceiveEventInvalidationReason::CycleChanged);
                resyncStartControlToIdle(
                    startControlGates, effectiveKeyPoll, effectiveKeyQuery);
                continue;
            }
            const ExecutionPlan& plan = *planResult.value();
            cout << "plan type=" << static_cast<int>(plan.sourceShotType) << endl;
            cout << "shot direction=" << plan.shotDirectionXY.x << ","
                 << plan.shotDirectionXY.y << endl;
            cout << "StrikeReadyPose:" << endl;
            cout << "X=" << plan.strikeReadyPose.x << endl;
            cout << "Y=" << plan.strikeReadyPose.y << endl;
            cout << "Z=" << plan.strikeReadyPose.z << endl;
            cout << "A=" << plan.strikeReadyPose.a << endl;
            cout << "B=" << plan.strikeReadyPose.b << endl;
            cout << "C=" << plan.strikeReadyPose.c << endl;
            cout << "SafeApproachPose: X=" << plan.safeApproachPose.x
                 << ", Y=" << plan.safeApproachPose.y
                 << ", Z=" << plan.safeApproachPose.z
                 << ", A=" << plan.safeApproachPose.a
                 << ", B=" << plan.safeApproachPose.b
                 << ", C=" << plan.safeApproachPose.c << endl;
            cout << "Base0 revision=" << plan.base0PlanarCalibrationRevision << endl;
            cout << "Motion revision=" << plan.motionCalibrationRevision << endl;
            cout << "Policy revision=" << plan.executionPolicyRevision << endl;
            cout << "NO HARDWARE EXECUTION" << endl;
            cout << "CycleCompleted" << endl;
            cout << "WaitingForStart" << endl;
            invalidateVisionCycle(ReceiveEventInvalidationReason::CycleChanged);
            resyncStartControlToIdle(
                startControlGates, effectiveKeyPoll, effectiveKeyQuery);
            continue;
        }

        if (*policyMode != BilliardConfig::ExecutionPolicyMode::RealHardware ||
            !motionPlanningPolicyMode ||
            *motionPlanningPolicyMode !=
                BilliardConfig::ExecutionPolicyMode::RealHardware ||
            !policyRevision || policyRevision->empty() ||
            !legalContactAuthorized || *legalContactAuthorized ||
            !realConfig || !cycleRobot ||
            !realConfig->authorizationRevision ||
            *realConfig->authorizationRevision != *policyRevision ||
            !RobotController::validateRealHardwareConfiguration(
                 realConfig).succeeded()) {
            cout << "[錯誤] RealHardware policy/config門檻未通過，"
                 << "程式結束（不接受任何H）。" << endl;
            return;
        }

        cout << "[H] cycle #" << cycleIdentity << " 開始，狀態=StartRequested"
             << endl;
        const ExecutionCycleResult result = runRealSingleCycle(
            executionRuntime, cycleIdentity, *cycleRobot, realConfig, services,
            deadline);
        invalidateVisionCycle(ReceiveEventInvalidationReason::CycleChanged);
        resyncStartControlToIdle(
            startControlGates, effectiveKeyPoll, effectiveKeyQuery);
        cout << "[cycle #" << cycleIdentity << "] status="
             << executionCycleStatusName(result.status);
        if (result.diagnostic) {
            cout << " reason=" << executionCycleFailureReasonName(
                        result.diagnostic->reason)
                 << " stoppedState="
                 << executionCycleStateName(result.diagnostic->stoppedState);
        }
        if (result.value) {
            cout << " shotExecuted="
                 << (result.value->shotExecuted ? "true" : "false")
                 << " states=[";
            for (std::size_t i = 0; i < result.value->states.size(); ++i) {
                if (i) cout << "->";
                cout << executionCycleStateName(result.value->states[i]);
            }
            cout << "]";
        }
        cout << endl;
        cout << "[狀態] 下一次H要先恢復"
             << executionCycleStateName(executionRuntime.state) << endl;
        if (!result.isValid()) {
            cout << "[錯誤] ExecutionCycleResult內部不一致（isValid()==false），"
                 << "程式結束。" << endl;
            break;
        }
        // executionRuntime.state已由runOfflineSingleCycle正確設定：
        // WaitingForStart可繼續迴圈；ManualRecoveryRequired／UnknownUnsafe
        // 會讓while條件自然結束，不再接受下一次H。
    }
}

bool BilliardApp::processReceiveEvent(const ReceiveEvent& event)
{
    const StabilityResult result = stability.accept(event);
    if (!result.isValid()) {
        cout << "[P1-04] StabilityResult invariant失敗。" << endl;
        invalidateVisionCycle(ReceiveEventInvalidationReason::CycleChanged);
        return false;
    }
    const std::optional<Phase1PipelineResult> pipelineResult =
        Phase1PipelineResult::fromStability(result);
    if (pipelineResult && !pipelineResult->isValid()) {
        cout << "[P1-04] Phase1PipelineResult invariant失敗。" << endl;
        invalidateVisionCycle(ReceiveEventInvalidationReason::CycleChanged);
        return false;
    }
    if (pipelineResult && pipelineResult->status() == Phase1PipelineStatus::Waiting) {
        cout << "[P1-04] ReceiveEvent id=" << event.eventId
             << " accepted; waiting for three-event stability." << endl;
        return true;
    }
    if (result.status() == StabilityStatus::Stable && result.value()) {
        std::optional<BilliardConfig::TableGeometryConfig> tableGeometry =
            BilliardConfig::TABLE_GEOMETRY;
        BilliardConfig::BrainConfig brainConfig =
            BilliardConfig::BRAIN_CONFIG;
#ifdef BILLIARDS_P2_03_TEST_SEAM
        if (runTestSeam) {
            if (runTestSeam->tableGeometryConfig) {
                tableGeometry = runTestSeam->tableGeometryConfig;
            }
            if (runTestSeam->brainConfig) {
                brainConfig = *runTestSeam->brainConfig;
            }
        }
#endif
        PlanningResult planning = BilliardAlgorithm::planShot(
            *result.value(),
            tableGeometry,
            brainConfig);
        const Phase1PipelineResult completed =
            Phase1PipelineResult::planningCompleted(planning);
        if (!completed.isValid() || !completed.planningResult()) {
            cout << "[P1-09] PlanningCompleted invariant失敗。" << endl;
            invalidateVisionCycle(ReceiveEventInvalidationReason::CycleChanged);
            return false;
        }
        pendingPlanningResult = std::move(planning);
        // 貼庫安全繞行用：解析失敗（例如tableGeometry尚未核准）就留nullopt，
        // 等同功能關閉，不擋這一輪執行——今天沒有這個保護，行為不會變差。
        const GeometryValueResult<ResolvedTableGeometry> resolvedGeometry =
            BilliardPhysics::resolveTableGeometry(
                result.value()->pockets, tableGeometry);
        pendingResolvedTableGeometry =
            resolvedGeometry.isValid() && resolvedGeometry.value()
                ? resolvedGeometry.value()
                : std::optional<ResolvedTableGeometry>{};
#ifdef BILLIARDS_P2_03_TEST_SEAM
        if (runTestSeam && runTestSeam->planningResultObserved) {
            runTestSeam->planningResultObserved(*pendingPlanningResult);
        }
#endif
        if (std::holds_alternative<ShotPlan>(pendingPlanningResult->value())) {
            cout << "[P1-09] Phase 1 ShotPlan ready for P2-01." << endl;
        } else {
            const NoPlan& noPlan = std::get<NoPlan>(pendingPlanningResult->value());
            cout << "[P1-09] Phase 1 stopped with NoPlan reason="
                 << static_cast<int>(noPlan.reason)
                 << "; PotOnly result is unchanged. Precomputed execution "
                    "candidates, if any, remain separate from this result."
                 << endl;
            return false;
        }
        return false;
    }

    cout << "[P1-04] Stability failure status=" << static_cast<int>(result.status())
         << ", reason="
         << static_cast<int>(result.diagnostic()->reason)
         << "; current cycle is invalidated." << endl;
    // ThreeEventStability已以精確原因完成reset；此處只關閉P1-03 event gate，
    // 不再次覆寫stability diagnostic。
    receiveEventFactory.invalidate(ReceiveEventInvalidationReason::CycleChanged);
    return true;
}

void BilliardApp::invalidateVisionCycle(ReceiveEventInvalidationReason reason)
{
    receiveEventFactory.invalidate(reason);
    stability.reset(stabilityResetReason(reason));
    pendingPlanningResult.reset();
    pendingResolvedTableGeometry.reset();
}
