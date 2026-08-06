// 協調視覺解析、目標選擇、擊球規劃與手臂動作，不實作個別演算法細節。
#include "BilliardApp.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>

#include "BilliardConfig.h"

using namespace std;

namespace {

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

}  // namespace

BilliardApp::BilliardApp()
    : receiveEventFactory(visionParser),
      stability(productionStabilityConfig()),
      needCameraMove(true),
      nextShotCycleIdentity(1)
{
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

    if (!robot.connect(BilliardConfig::ARM_IP)) {
        cout << "[錯誤] 手臂連線失敗。" << endl;
        return false;
    }

    robot.setMotorState(1);
    robot.setOverrideRatio(BilliardConfig::NORMAL_SPEED_RATIO);
    robot.setToolNumber(BilliardConfig::TOOL_NUMBER);

    cout << "[系統] 等待 Python 連線..." << endl;
    while (visionClient.connectToServerResult(
               BilliardConfig::VISION_SERVER_IP,
               BilliardConfig::VISION_SERVER_PORT)
               .status != SocketConnectStatus::Success) {
        Sleep(1000);
    }
    cout << "[系統] 與 Python 服務連線成功！" << endl;
    needCameraMove = true;
    return true;
}

void BilliardApp::run() {
    while (true) {
        if (needCameraMove) {
            if (!waitForStartRequest() ||
                !moveToCameraPosition() ||
                !openCaptureWindowAfterCameraPose()) {
                cout << "[系統] 無法安全開啟影像capture window，停止本次執行。" << endl;
                break;
            }
            needCameraMove = false;
        }

        robot.setToolNumber(BilliardConfig::TOOL_NUMBER);

        const SocketReceiveResult received = visionClient.receiveFrame();
        if (!received.isValid()) {
            cout << "[系統] SocketReceiveResult invariant錯誤。" << endl;
            invalidateVisionCycle(ReceiveEventInvalidationReason::Disconnect);
            break;
        }

        if (received.status == SocketReceiveStatus::FrameReady && received.frame) {
            const ReceiveEventResult eventResult = receiveEventFactory.accept(
                *received.frame,
                visionClient.connectionIdentity(),
                chrono::steady_clock::now());
            if (!eventResult.isValid()) {
                cout << "[系統] ReceiveEventResult invariant錯誤。" << endl;
                invalidateVisionCycle(ReceiveEventInvalidationReason::ParserFailure);
                break;
            }
            if (eventResult.status() == ReceiveEventStatus::Success && eventResult.value()) {
                if (!processReceiveEvent(*eventResult.value())) {
                    break;
                }
            } else {
                cout << "[影像資料拒絕] status="
                     << static_cast<int>(eventResult.status())
                     << ", resetRequired="
                     << (eventResult.resetRequired() ? "true" : "false")
                     << endl;
                if (eventResult.resetRequired()) {
                    invalidateVisionCycle(ReceiveEventInvalidationReason::ParserFailure);
                    needCameraMove = true;
                }
            }
        } else if (received.status == SocketReceiveStatus::CleanClose) {
            cout << "[系統] 影像端 Socket 連線正常關閉。" << endl;
            invalidateVisionCycle(ReceiveEventInvalidationReason::Disconnect);
            break;
        } else if (received.status == SocketReceiveStatus::TimedOut) {
            cout << "[系統] 等待影像frame逾時，本cycle失效。" << endl;
            invalidateVisionCycle(ReceiveEventInvalidationReason::Timeout);
            break;
        } else {
            cout << "[系統] 影像端framing／Socket錯誤，status="
                 << static_cast<int>(received.status)
                 << ", socketError=" << received.socketError << endl;
            invalidateVisionCycle(ReceiveEventInvalidationReason::Disconnect);
            break;
        }
    }
}

bool BilliardApp::waitForStartRequest()
{
    cout << "\n[WaitingForStart] 請確認本次shot cycle可安全開始，"
         << "並在【此視窗】按下 [Enter]：";
    cin.clear();
    string input;
    return static_cast<bool>(getline(cin, input));
}

bool BilliardApp::moveToCameraPosition() {  // 移動至拍照點
    cout << "\n[安全鎖] 準備返回拍照點..." << endl;
    cout << "[動作] 移動至拍照點..." << endl;
    if (!requireMotionSuccess(
            "PTP to camera joint pose",
            robot.moveToAxis(BilliardConfig::CAMERA_JOINT.data(), true))) {
        return false;
    }
    Sleep(BilliardConfig::CAMERA_SETTLE_MS);
    return true;
}

bool BilliardApp::openCaptureWindowAfterCameraPose()
{
    const SocketOperationResult flushResult = visionClient.flushBuffer();
    if (flushResult.status != SocketOperationStatus::Success) {
        cout << "[系統] 清除舊Socket buffer失敗，socketError="
             << flushResult.socketError << endl;
        invalidateVisionCycle(ReceiveEventInvalidationReason::ExplicitFlush);
        return false;
    }

    stability.reset(StabilityFailureReason::ExplicitReset);
    pendingPlanningResult.reset();

    if (nextShotCycleIdentity == 0) {
        cout << "[系統] shot-cycle identity已耗盡。" << endl;
        invalidateVisionCycle(ReceiveEventInvalidationReason::CycleChanged);
        return false;
    }
    const ShotCycleIdentity cycleIdentity = nextShotCycleIdentity++;
    receiveEventFactory.beginCycle(
        visionClient.connectionIdentity(),
        cycleIdentity);
    if (!receiveEventFactory.openCaptureWindow(
            visionClient.connectionIdentity(),
            cycleIdentity)) {
        cout << "[系統] capture window identity不一致。" << endl;
        invalidateVisionCycle(ReceiveEventInvalidationReason::CaptureWindowRestart);
        return false;
    }
    cout << "[系統] 已開啟shot cycle " << cycleIdentity
         << "的capture window。" << endl;
    return true;
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
        PlanningResult planning = BilliardAlgorithm::planShot(
            *result.value(),
            BilliardConfig::TABLE_GEOMETRY,
            BilliardConfig::BRAIN_CONFIG);
        const Phase1PipelineResult completed =
            Phase1PipelineResult::planningCompleted(planning);
        if (!completed.isValid() || !completed.planningResult()) {
            cout << "[P1-09] PlanningCompleted invariant失敗。" << endl;
            invalidateVisionCycle(ReceiveEventInvalidationReason::CycleChanged);
            return false;
        }
        pendingPlanningResult = std::move(planning);
        if (std::holds_alternative<ShotPlan>(pendingPlanningResult->value())) {
            cout << "[P1-09] Phase 1 ShotPlan ready for P2-01; "
                 << "Phase 2 execution is not implemented here." << endl;
        } else {
            const NoPlan& noPlan = std::get<NoPlan>(pendingPlanningResult->value());
            cout << "[P1-09] Phase 1 stopped with NoPlan reason="
                 << static_cast<int>(noPlan.reason)
                 << "; returning to WaitingForStart without a fallback shot."
                 << endl;
            invalidateVisionCycle(ReceiveEventInvalidationReason::CycleChanged);
            needCameraMove = true;
            return true;
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
    needCameraMove = true;
    return true;
}

void BilliardApp::invalidateVisionCycle(ReceiveEventInvalidationReason reason)
{
    receiveEventFactory.invalidate(reason);
    stability.reset(stabilityResetReason(reason));
    pendingPlanningResult.reset();
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
    if (toolNumber != BilliardConfig::TOOL_NUMBER ||
        baseNumber != BilliardConfig::BASE_NUMBER) {
        cout << "[Error] Active Tool/Base does not match configured Tool "
             << BilliardConfig::TOOL_NUMBER << "/Base "
             << BilliardConfig::BASE_NUMBER << "." << endl;
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
        cout << "[Error] get_current_position failed at ready pose. SDK code="
             << sdkCode << endl;
        printAlarmCodes();
        return false;
    }

    bool linearPathReachable = false;
    if (!robot.checkLinearPath(
        actualReadyPose,
        plan.strikePose,
        linearPathReachable,
        sdkCode
    )) {
        cout << "[Error] motion_check_lin API failed. SDK code="
             << sdkCode << ". Motion is blocked." << endl;
        printAlarmCodes();
        return false;
    }
    cout << "[Diagnostic] Ready-to-strike LIN path: "
         << (linearPathReachable ? "reachable" : "not reachable")
         << ". This run uses PTP." << endl;
    if (!linearPathReachable) {
        printAlarmCodes();
        return false;
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
