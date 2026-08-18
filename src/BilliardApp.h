// 宣告撞球應用流程協調器及其依賴元件。
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <optional>
#include <string>
#include <vector>

#include "Algorithm.h"
#include "MotionPlanner.h"
#include "RobotController.h"
#include "SocketClient.h"
#include "VisionDataParser.h"

// ============================================================
// H/P鍵盤edge gate
// ============================================================

enum class ConsoleKey {
    H,
    P
};

struct ConsoleKeyEvent {
    ConsoleKey key;
    bool keyDown;
};

// Production：讀取Windows console尚未處理的KEY_EVENT，不阻塞、不用
// getline()。test seam可注入合成序列。
using ConsoleKeyPoll = std::function<std::vector<ConsoleKeyEvent>()>;
// Production：查詢按鍵目前實際物理狀態（非事件）。只在WaitingForStart
// 重新開始監聽時，用來把edge gate與真實物理狀態重新同步，避免長按跨過
// 一輪邊界時被誤判成新的一次down。
using ConsoleKeyDownQuery = std::function<bool(ConsoleKey)>;

class KeyEdgeGate {
public:
    // 回傳true代表一次全新的up->down邊緣；同一次長按產生的後續down事件
    // 回傳false；收到up事件才重新arm，回傳false。
    bool onKeyDown(bool keyDown) noexcept
    {
        if (keyDown) {
            if (down_) return false;
            down_ = true;
            return true;
        }
        down_ = false;
        return false;
    }

    void resyncPhysicalState(bool currentlyDown) noexcept { down_ = currentlyDown; }
    [[nodiscard]] bool isDown() const noexcept { return down_; }

private:
    bool down_ = false;
};

struct StartControlEvent {
    bool startEdge = false;
    bool standbyEdge = false;
};

struct StartControlGates {
    KeyEdgeGate start;
    KeyEdgeGate standby;
};

// ============================================================
// Shot-cycle競賽計時：production authority為steady_clock，可注入fake clock。
// ============================================================

using SteadyClockNow = std::function<std::chrono::steady_clock::time_point()>;

struct ShotDeadlineClock {
    SteadyClockNow now;
    std::chrono::steady_clock::time_point startedAt{};

    [[nodiscard]] std::chrono::milliseconds elapsedMs() const
    {
        // 括號寫法刻意避開Windows.h在未定義NOMINMAX時的max()巨集展開；
        // main.cpp在包含BilliardApp.h前先include了未加NOMINMAX保護的
        // winsock2.h，此檔案不在核准範圍內、不得修改，因此改在此處防禦。
        if (!now) return (std::chrono::milliseconds::max)();
        const auto elapsed = now() - startedAt;
        return elapsed.count() < 0
            ? std::chrono::milliseconds(0)
            : std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
    }

    [[nodiscard]] std::chrono::milliseconds remainingMs(unsigned long deadlineMs) const
    {
        const auto elapsed = elapsedMs();
        const auto deadline = std::chrono::milliseconds(deadlineMs);
        return elapsed >= deadline ? std::chrono::milliseconds(0) : deadline - elapsed;
    }
};

enum class ExecutionCycleState {
    WaitingForStart,
    StartRequested,
    PreparationReturn,
    CameraPose,
    CameraSettling,
    Planning,
    StrikeReady,
    Pneumatic,
    PostStrikeActualPose,
    SafeLift,
    StandbyReturn,
    CycleCompleted,
    ManualRecoveryRequired,
    UnknownUnsafe
};

enum class OfflineStepStatus {
    Success,
    Failure,
    UnknownUnsafe
};

struct OfflineStepResult {
    OfflineStepStatus status;

    [[nodiscard]] bool succeeded() const noexcept
    {
        return status == OfflineStepStatus::Success;
    }
};

enum class OfflinePhase1Status {
    ShotPlanReady,
    NoPlan,
    PipelineFailure
};

// PipelineFailure必須區分成因：只有Vision transport本身中斷（socket
// timeout／斷線）才可在CameraPose內重拍；parser failure、invalid
// event/result、config/numerical failure、connection identity不一致
// 等一律不可重試；重試視窗本身到期則是獨立語意，不可誤標成transport中斷。
enum class OfflinePhase1FailureKind {
    TransportDisruption,
    NonRetriable,
    RetryWindowExpired
};

struct OfflinePhase1Result {
    OfflinePhase1Status status;
    std::optional<OfflinePhase1FailureKind> failureKind;

    [[nodiscard]] bool isValid() const noexcept
    {
        return (status == OfflinePhase1Status::PipelineFailure) ==
                failureKind.has_value() &&
            (status == OfflinePhase1Status::ShotPlanReady ||
                status == OfflinePhase1Status::NoPlan ||
                status == OfflinePhase1Status::PipelineFailure);
    }
};

enum class PneumaticCompletionStatus {
    PolicyAccepted,
    Failure,
    UnknownUnsafe
};

enum class PneumaticCompletionEvidence {
    OffCommandAccepted
};

struct PneumaticCompletionResult {
    PneumaticCompletionStatus status;
    std::optional<PneumaticCompletionEvidence> evidence;

    [[nodiscard]] bool isValid() const noexcept
    {
        const bool knownStatus =
            status == PneumaticCompletionStatus::PolicyAccepted ||
            status == PneumaticCompletionStatus::Failure ||
            status == PneumaticCompletionStatus::UnknownUnsafe;
        const bool knownEvidence = !evidence ||
            *evidence == PneumaticCompletionEvidence::OffCommandAccepted;
        return knownStatus && knownEvidence &&
            ((status == PneumaticCompletionStatus::PolicyAccepted) ==
             evidence.has_value());
    }
};

enum class LinearPathCheckStatus {
    Success,
    ApiFailure,
    UnknownUnsafe
};

struct LinearPathCheckResult {
    LinearPathCheckStatus status;
    bool reachable;

    [[nodiscard]] bool isValid() const noexcept
    {
        return status == LinearPathCheckStatus::Success || !reachable;
    }
};

// Planning狀態內部（capture＋Algorithm＋MotionPlanner姿態搜尋，含CameraPose
// 內重拍與Vision reconnect）的定案結果。PlanReady才附帶ExecutionPlan。
enum class PlanningPhaseStatus {
    PlanReady,
    NoPlanSafeEnd,
    Failure,
    ManualRecoveryRequired,
    UnknownUnsafe
};

struct PlanningPhaseResult {
    PlanningPhaseStatus status;
    std::optional<ExecutionPlan> plan;

    [[nodiscard]] bool isValid() const noexcept
    {
        return (status == PlanningPhaseStatus::PlanReady) == plan.has_value();
    }
};

enum class ExecutionCycleStatus {
    Completed,
    SafeFailure,
    UnknownUnsafe,
    StartRejected
};

enum class ExecutionCycleFailureReason {
    None,
    CycleAlreadyActive,
    CycleIdentityExhausted,
    MissingFakeAdapter,
    PreparationCheckFailed,
    PreparationReturnMotionFailed,
    PreparationReturnNotStopped,
    CameraPoseMotionFailed,
    CameraPoseNotStopped,
    CameraSettleFailed,
    CaptureAndPlanFailed,
    NoExecutablePlan,
    VisionReconnectManualRecoveryRequired,
    InvalidExecutionPlan,
    ExecutionPlanCycleMismatch,
    StrikeReadyValidationFailed,
    StrikeReadyMotionFailed,
    StrikeReadyNotStopped,
    PneumaticFailed,
    PneumaticStateUnknown,
    ActualPoseReadFailed,
    InvalidActualPose,
    SafeLiftPathCheckFailed,
    SafeLiftPathUnreachable,
    SafeLiftMotionFailed,
    SafeLiftNotConfirmed,
    StandbyReturnMotionFailed,
    StandbyReturnNotStopped
};

struct ExecutionCycleAudit {
    std::uint64_t cycleIdentity;
    bool shotExecuted;
    std::optional<Phase1PlanIdentity> sourcePlanIdentity;
    std::optional<PneumaticCompletionEvidence> pneumaticEvidence;
    std::vector<ExecutionCycleState> states;

    [[nodiscard]] bool isValid() const noexcept
    {
        if (states.size() < 2 ||
            states.front() != ExecutionCycleState::WaitingForStart ||
            states.back() != ExecutionCycleState::WaitingForStart ||
            states[1] != ExecutionCycleState::StartRequested) {
            return false;
        }
        // PreparationReturn只在Robot按H時不在準備姿態才會出現，屬選配步驟；
        // 比對固定模板前先移除，其餘序列必須完全固定。
        std::vector<ExecutionCycleState> core = states;
        const auto prepIt = core.begin() + 2;
        if (prepIt != core.end() &&
            *prepIt == ExecutionCycleState::PreparationReturn) {
            core.erase(prepIt);
        }
        const auto matches = [&](std::initializer_list<ExecutionCycleState> expected) {
            return core.size() == expected.size() &&
                std::equal(core.begin(), core.end(), expected.begin());
        };
        const bool noPlanTrace = matches({
            ExecutionCycleState::WaitingForStart,
            ExecutionCycleState::StartRequested,
            ExecutionCycleState::CameraPose,
            ExecutionCycleState::CameraSettling,
            ExecutionCycleState::Planning,
            ExecutionCycleState::StandbyReturn,
            ExecutionCycleState::CycleCompleted,
            ExecutionCycleState::WaitingForStart});
        const bool shotTrace = matches({
            ExecutionCycleState::WaitingForStart,
            ExecutionCycleState::StartRequested,
            ExecutionCycleState::CameraPose,
            ExecutionCycleState::CameraSettling,
            ExecutionCycleState::Planning,
            ExecutionCycleState::StrikeReady,
            ExecutionCycleState::Pneumatic,
            ExecutionCycleState::PostStrikeActualPose,
            ExecutionCycleState::SafeLift,
            ExecutionCycleState::StandbyReturn,
            ExecutionCycleState::CycleCompleted,
            ExecutionCycleState::WaitingForStart});
        return cycleIdentity != 0 &&
            (shotExecuted ? shotTrace : noPlanTrace) &&
            (shotExecuted
                ? sourcePlanIdentity.has_value() &&
                  sourcePlanIdentity->isValid() &&
                  sourcePlanIdentity->shotCycleIdentity == cycleIdentity &&
                  pneumaticEvidence.has_value() &&
                  *pneumaticEvidence ==
                      PneumaticCompletionEvidence::OffCommandAccepted
                : !sourcePlanIdentity.has_value() && !pneumaticEvidence.has_value());
    }
};

struct ExecutionCycleDiagnostic {
    ExecutionCycleFailureReason reason;
    ExecutionCycleState stoppedState;
};

struct ExecutionCycleResult {
    ExecutionCycleStatus status;
    std::optional<ExecutionCycleAudit> value;
    std::optional<ExecutionCycleDiagnostic> diagnostic;

    [[nodiscard]] bool isValid() const noexcept
    {
        const bool completed = status == ExecutionCycleStatus::Completed;
        return value.has_value() == completed &&
            diagnostic.has_value() != completed &&
            (!value || value->isValid()) &&
            (!diagnostic || diagnostic->reason != ExecutionCycleFailureReason::None) &&
            (status != ExecutionCycleStatus::UnknownUnsafe ||
             (diagnostic &&
              diagnostic->stoppedState == ExecutionCycleState::UnknownUnsafe));
    }

};

struct OfflineExecutionRuntime {
    ExecutionCycleState state = ExecutionCycleState::WaitingForStart;
};

struct OfflineExecutionSeam {
    std::function<OfflineStepResult()> confirmPreparedForCycle;
    // 純設定檢查（非hardware read）：STANDBY_JOINT_REFERENCE尚未核准
    // revision時必須Failure（不latch），不得繞過此門檻直接PTP。
    std::function<OfflineStepResult()> confirmStandbyReferenceApproved;
    // 寫入動作（establishSafeOutputsOff／clearAlarm／activateConfiguredToolAndBase／
    // setMotorState／setOverrideRatio）：只能排在confirmPreparedForCycle與
    // confirmStandbyReferenceApproved都通過之後才呼叫，不得提前改變硬體狀態。
    std::function<OfflineStepResult()> prepareHardwareForMotion;
    // nullopt代表無法確認（fail closed，呼叫端須進UnknownUnsafe）。
    std::function<std::optional<bool>()> isAtStandby;
    std::function<OfflineStepResult()> returnToStandby;
    std::function<OfflineStepResult()> confirmStandbyStopped;
    std::function<OfflineStepResult()> moveToCameraPose;
    std::function<OfflineStepResult()> confirmCameraPoseStopped;
    std::function<OfflineStepResult()> settleCamera;
    std::function<PlanningPhaseResult()> acquireExecutionPlan;
    std::function<OfflineStepResult(const ExecutionPlan&)> validateStrikeReady;
    std::function<OfflineStepResult(const ExecutionPlan&)> moveToStrikeReady;
    std::function<OfflineStepResult()> confirmStrikeReadyStopped;
    std::function<PneumaticCompletionResult(const ExecutionPlan&)> runPneumatic;
    // UnknownUnsafe必須完整保留，不得壓成單純nullopt讀取失敗。
    std::function<RobotPoseAdapterResult()> readActualPose;
    std::function<LinearPathCheckResult(
        const RobotPoseABC&,
        const RobotPoseABC&)> checkSafeLiftLinearPath;
    std::function<OfflineStepResult(const RobotPoseABC&)> moveSafeLiftLinear;
    std::function<OfflineStepResult(const RobotPoseABC&)> confirmSafeLiftStopped;
    // Push only: retract the striker after the actual-pose vertical safe lift
    // has reached and stopped at safeApproach Z.
    std::function<PneumaticCompletionResult(const ExecutionPlan&)>
        finishPneumaticAfterSafeLift;
    std::function<OfflineStepResult(const ExecutionPlan&)> returnToStandbyAfterStrike;
    std::function<OfflineStepResult()> confirmStandbyReturnStopped;
};

// connect()失敗原因必須區分：只有transport層真的可能自行恢復的失敗
// （目前僅ConnectError，例如Python尚未啟動、連線被拒、逾時）才可在
// retry cutoff內重試；設定/位址/socket建立層級的失敗永久不會恢復，
// 必須立即fail closed，不得浪費整個重試視窗。
enum class VisionConnectStatus {
    Connected,
    Retriable,
    NonRetriable
};

struct VisionConnectResult {
    VisionConnectStatus status;
    int socketError;
};

// P2-03 僅提供非硬體生命週期依賴；Robot/DO 一律由既有 RobotController 承接。
struct RealExecutionCycleServices {
    std::function<OfflineStepResult()> settleCamera;
    std::function<OfflineStepResult()> flushStaleVisionBuffer;
    std::function<OfflineStepResult()> resetCycleAccumulation;
    // 呼叫端必須每次都傳入同一個已配置cycleIdentity，本函式不得自行配置。
    std::function<OfflineStepResult(ShotCycleIdentity)> openCaptureWindow;
    // runPhase1()一返回（不論結果為何）就呼叫，告知Python這次capture
    // window已經不需要更多資料；best-effort、未設定時（例如既有測試
    // fixture沒配置）視同no-op，不影響cycle成功/失敗判定。
    std::function<void(ShotCycleIdentity)> closeCaptureWindow;
    std::function<OfflinePhase1Result()> runPhase1;
    std::function<bool()> isVisionConnected;
    // 單次連線嘗試；重試／deadline由呼叫端負責。
    std::function<VisionConnectResult()> connectVision;
    std::function<const PlanningResult*()> currentPlanningResult;
    // runRealSingleCycle是static member function、沒有this，這裡是它
    // 讀取pendingResolvedTableGeometry（供ForcedLegalContact與on-demand
    // CueBallContactOnly執行層保底做碰撞檢查用）的唯一管道，
    // 跟currentPlanningResult
    // 同一個wiring模式。回傳nullptr代表目前沒有已解析的桌面幾何，
    // 呼叫端必須fail closed（不產生候選），不能假設沒有障礙。
    std::function<const std::optional<ResolvedTableGeometry>*()>
        currentResolvedTableGeometry;
    // 第三個參數是forcedStrikeMode：nullopt時沿用MotionPlanner既有的
    // preferredStrikeMode自動選擇；有值時強制指定StrikeMode，供preflight
    // NotReachable後的對側重試使用（見tryCandidates）。
    std::function<ExecutionPlanResult(
        const ShotPlan&,
        bool,
        std::optional<StrikeMode>)> buildExecutionPlanForShot;
    // PlanningTest診斷模式使用：單一計畫入口（挑選第一順位候選），
    // 不含retry／reconnect，只服務不動硬體的診斷輸出。
    std::function<ExecutionPlanResult()> buildExecutionPlan;
    std::function<void(unsigned long)> sleepMs;
#ifdef BILLIARDS_P2_03_TEST_SEAM
    // 舊單計畫入口僅供既有離線adapter fixtures；production不可啟用。
    bool testOnlyAllowSinglePlanCompatibility = false;
#endif
};

#ifdef BILLIARDS_P2_03_TEST_SEAM
struct BilliardAppRunTestSeam {
    std::optional<BilliardConfig::ExecutionPolicyMode> policyMode;
    std::optional<std::string> policyRevision;
    std::optional<bool> legalContactExecutionAuthorized;
    RobotController* robot;
    std::optional<BilliardConfig::RealHardwareExecutionConfig> realConfig;
    // 注入原始key事件，讓production的pollStartControl／resyncStartControlToIdle
    // 邏輯本身受測，而不是繞過edge gate另開一條路徑。
    ConsoleKeyPoll consoleKeyPoll;
    ConsoleKeyDownQuery consoleKeyDownQuery;
    std::optional<RealExecutionCycleServices> realServices;
    std::optional<BilliardConfig::ExecutionPolicyMode> motionPlanningPolicyMode;
    std::optional<BilliardConfig::MotionPlanningConfig> motionPlanningConfig;
    std::optional<MotionPlanningChecks> motionPlanningChecks;
    std::optional<AxisAlignedBounds2D> observationBounds;
    std::optional<StabilityConfig> stabilityConfig;
    std::optional<BilliardConfig::TableGeometryConfig> tableGeometryConfig;
    std::optional<BilliardConfig::BrainConfig> brainConfig;
    std::optional<ConnectionIdentity> connectionIdentity;
    std::vector<std::string> currentCycleFrames;
    SteadyClockNow fakeNow;
    std::function<void(const PlanningResult&)> planningResultObserved;
    std::function<void(const ExecutionPlanResult&)> executionPlanObserved;
};
#endif

class BilliardApp {
private:
    RobotController robot;
    SocketClient visionClient;
    VisionDataParser visionParser;
    ReceiveEventFactory receiveEventFactory;
    ThreeEventStability stability;
    std::optional<PlanningResult> pendingPlanningResult;
    // 貼庫安全繞行用：跟pendingPlanningResult同一輪cycle的桌面幾何快照，
    // 同進同出（見processReceiveEvent／invalidateVisionCycle／
    // resetCycleAccumulation）。
    std::optional<ResolvedTableGeometry> pendingResolvedTableGeometry;
    MotionPlanner motionPlanner;
    std::optional<MotionPlanningChecks> offlineMotionPlanningChecks;
    ShotCycleIdentity nextShotCycleIdentity;
    StartControlGates startControlGates;
#ifdef BILLIARDS_P2_03_TEST_SEAM
    std::optional<BilliardAppRunTestSeam> runTestSeam;
#endif

public:
    BilliardApp();
    explicit BilliardApp(MotionPlanningChecks offlineChecks);
#ifdef BILLIARDS_P2_03_TEST_SEAM
    explicit BilliardApp(BilliardAppRunTestSeam seam);
#endif

    bool initialize();
    void run();
    [[nodiscard]] static ExecutionCycleResult runOfflineSingleCycle(
        OfflineExecutionRuntime& runtime,
        std::uint64_t cycleIdentity,
        const OfflineExecutionSeam& seam);
    [[nodiscard]] static ExecutionCycleResult runRealSingleCycle(
        OfflineExecutionRuntime& runtime,
        std::uint64_t cycleIdentity,
        RobotController& robot,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config,
        const RealExecutionCycleServices& services,
        const ShotDeadlineClock& deadline);
    [[nodiscard]] static PneumaticCompletionResult mapRealPneumaticResult(
        const RealPneumaticResult& result) noexcept;
    [[nodiscard]] static StartControlEvent pollStartControl(
        StartControlGates& gates,
        const ConsoleKeyPoll& poll);
    // WaitingForStart重新開始監聽前呼叫：drain累積事件、丟棄其中任何edge，
    // 並把gate與目前真實物理按鍵狀態重新同步，避免長按跨過一輪邊界時
    // 被誤判成新的一次down、也避免busy期間累積的press被排隊成下一輪Start。
    static void resyncStartControlToIdle(
        StartControlGates& gates,
        const ConsoleKeyPoll& poll,
        const ConsoleKeyDownQuery& query);

private:
    bool processReceiveEvent(const ReceiveEvent& event);
    void invalidateVisionCycle(ReceiveEventInvalidationReason reason);
    // H與P共用同一套「先read-only確認、通過才寫入硬體」流程，避免各自
    // 維護一份、日後又漂移出不一致的前置順序。純read-only，不寫任何狀態。
    [[nodiscard]] static OfflineStepResult confirmRobotReadyReadOnly(
        RobotController& robot,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
    // 寫入動作：只能在confirmRobotReadyReadOnly通過後呼叫。
    [[nodiscard]] static OfflineStepResult prepareRobotHardwareForMotion(
        RobotController& robot,
        const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config);
};

inline PneumaticCompletionResult BilliardApp::mapRealPneumaticResult(
    const RealPneumaticResult& result) noexcept
{
    if (!result.isValid()) {
        return {PneumaticCompletionStatus::UnknownUnsafe, std::nullopt};
    }
    switch (result.status) {
    case RealPneumaticStatus::Completed:
        return {PneumaticCompletionStatus::PolicyAccepted,
            PneumaticCompletionEvidence::OffCommandAccepted};
    case RealPneumaticStatus::KnownSafeFailure:
        return {PneumaticCompletionStatus::Failure, std::nullopt};
    case RealPneumaticStatus::UnknownUnsafe:
        return {PneumaticCompletionStatus::UnknownUnsafe, std::nullopt};
    default:
        return {PneumaticCompletionStatus::UnknownUnsafe, std::nullopt};
    }
}

inline StartControlEvent BilliardApp::pollStartControl(
    StartControlGates& gates,
    const ConsoleKeyPoll& poll)
{
    StartControlEvent event;
    if (!poll) return event;
    for (const ConsoleKeyEvent& raw : poll()) {
        if (raw.key == ConsoleKey::H) {
            if (gates.start.onKeyDown(raw.keyDown)) event.startEdge = true;
        } else if (raw.key == ConsoleKey::P) {
            if (gates.standby.onKeyDown(raw.keyDown)) event.standbyEdge = true;
        }
    }
    return event;
}

inline void BilliardApp::resyncStartControlToIdle(
    StartControlGates& gates,
    const ConsoleKeyPoll& poll,
    const ConsoleKeyDownQuery& query)
{
    if (poll) {
        for (const ConsoleKeyEvent& raw : poll()) {
            if (raw.key == ConsoleKey::H) {
                (void)gates.start.onKeyDown(raw.keyDown);
            } else if (raw.key == ConsoleKey::P) {
                (void)gates.standby.onKeyDown(raw.keyDown);
            }
        }
    }
    if (query) {
        gates.start.resyncPhysicalState(query(ConsoleKey::H));
        gates.standby.resyncPhysicalState(query(ConsoleKey::P));
    }
}

inline ExecutionCycleResult BilliardApp::runOfflineSingleCycle(
    OfflineExecutionRuntime& runtime,
    std::uint64_t cycleIdentity,
    const OfflineExecutionSeam& seam)
{
    if (runtime.state != ExecutionCycleState::WaitingForStart) {
        return {
            ExecutionCycleStatus::StartRejected,
            std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::CycleAlreadyActive,
                runtime.state}};
    }
    if (cycleIdentity == 0) {
        return {
            ExecutionCycleStatus::StartRejected,
            std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::CycleIdentityExhausted,
                runtime.state}};
    }

    std::vector<ExecutionCycleState> states{
        ExecutionCycleState::WaitingForStart,
        ExecutionCycleState::StartRequested};
    runtime.state = ExecutionCycleState::StartRequested;
    bool postStrikeRecoveryRequired = false;

    const auto safeFailure = [&](ExecutionCycleFailureReason reason) {
        const ExecutionCycleState stopped = runtime.state;
        runtime.state = postStrikeRecoveryRequired
            ? ExecutionCycleState::ManualRecoveryRequired
            : ExecutionCycleState::WaitingForStart;
        return ExecutionCycleResult{
            ExecutionCycleStatus::SafeFailure,
            std::nullopt,
            ExecutionCycleDiagnostic{reason, stopped}};
    };
    const auto unknownUnsafe = [&](ExecutionCycleFailureReason reason) {
        runtime.state = ExecutionCycleState::UnknownUnsafe;
        return ExecutionCycleResult{
            ExecutionCycleStatus::UnknownUnsafe,
            std::nullopt,
            ExecutionCycleDiagnostic{reason, ExecutionCycleState::UnknownUnsafe}};
    };
    const auto missing = [&]() {
        return safeFailure(ExecutionCycleFailureReason::MissingFakeAdapter);
    };
    const auto enter = [&](ExecutionCycleState state) {
        runtime.state = state;
        states.push_back(state);
    };
    const auto completed = [&](bool shotExecuted,
                               std::optional<Phase1PlanIdentity> source,
                               std::optional<PneumaticCompletionEvidence> evidence) {
        enter(ExecutionCycleState::CycleCompleted);
        states.push_back(ExecutionCycleState::WaitingForStart);
        runtime.state = ExecutionCycleState::WaitingForStart;
        return ExecutionCycleResult{
            ExecutionCycleStatus::Completed,
            ExecutionCycleAudit{
                cycleIdentity,
                shotExecuted,
                source,
                evidence,
                states},
            std::nullopt};
    };
    // 三態step結果的共用分派：UnknownUnsafe必須維持獨立終止路徑，
    // 不得被壓成一般safeFailure。
    const auto dispatchStep = [&](const OfflineStepResult& result,
                                  ExecutionCycleFailureReason failReason)
        -> std::optional<ExecutionCycleResult> {
        if (result.status == OfflineStepStatus::UnknownUnsafe) {
            return unknownUnsafe(failReason);
        }
        if (!result.succeeded()) {
            return safeFailure(failReason);
        }
        return std::nullopt;
    };

    if (!seam.confirmPreparedForCycle) return missing();
    if (const auto stop = dispatchStep(
            seam.confirmPreparedForCycle(),
            ExecutionCycleFailureReason::PreparationCheckFailed)) {
        return *stop;
    }

    if (!seam.confirmStandbyReferenceApproved) return missing();
    if (const auto stop = dispatchStep(
            seam.confirmStandbyReferenceApproved(),
            ExecutionCycleFailureReason::PreparationCheckFailed)) {
        return *stop;
    }

    if (!seam.prepareHardwareForMotion) return missing();
    if (const auto stop = dispatchStep(
            seam.prepareHardwareForMotion(),
            ExecutionCycleFailureReason::PreparationCheckFailed)) {
        return *stop;
    }

    if (!seam.isAtStandby) return missing();
    const std::optional<bool> atStandby = seam.isAtStandby();
    if (!atStandby) {
        return unknownUnsafe(ExecutionCycleFailureReason::PreparationCheckFailed);
    }
    if (!*atStandby) {
        enter(ExecutionCycleState::PreparationReturn);
        if (!seam.returnToStandby) return missing();
        if (const auto stop = dispatchStep(
                seam.returnToStandby(),
                ExecutionCycleFailureReason::PreparationReturnMotionFailed)) {
            return *stop;
        }
        if (!seam.confirmStandbyStopped) return missing();
        if (const auto stop = dispatchStep(
                seam.confirmStandbyStopped(),
                ExecutionCycleFailureReason::PreparationReturnNotStopped)) {
            return *stop;
        }
    }

    enter(ExecutionCycleState::CameraPose);
    if (!seam.moveToCameraPose) return missing();
    if (const auto stop = dispatchStep(
            seam.moveToCameraPose(),
            ExecutionCycleFailureReason::CameraPoseMotionFailed)) {
        return *stop;
    }
    if (!seam.confirmCameraPoseStopped) return missing();
    if (const auto stop = dispatchStep(
            seam.confirmCameraPoseStopped(),
            ExecutionCycleFailureReason::CameraPoseNotStopped)) {
        return *stop;
    }

    enter(ExecutionCycleState::CameraSettling);
    if (!seam.settleCamera) return missing();
    if (const auto stop = dispatchStep(
            seam.settleCamera(),
            ExecutionCycleFailureReason::CameraSettleFailed)) {
        return *stop;
    }

    enter(ExecutionCycleState::Planning);
    if (!seam.acquireExecutionPlan) return missing();
    const PlanningPhaseResult planning = seam.acquireExecutionPlan();
    if (!planning.isValid()) {
        return unknownUnsafe(ExecutionCycleFailureReason::CaptureAndPlanFailed);
    }
    bool planFound = false;
    ExecutionPlan plan{};
    switch (planning.status) {
    case PlanningPhaseStatus::UnknownUnsafe:
        return unknownUnsafe(ExecutionCycleFailureReason::CaptureAndPlanFailed);
    case PlanningPhaseStatus::ManualRecoveryRequired: {
        runtime.state = ExecutionCycleState::ManualRecoveryRequired;
        return ExecutionCycleResult{
            ExecutionCycleStatus::SafeFailure,
            std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::VisionReconnectManualRecoveryRequired,
                ExecutionCycleState::ManualRecoveryRequired}};
    }
    case PlanningPhaseStatus::Failure:
        return safeFailure(ExecutionCycleFailureReason::CaptureAndPlanFailed);
    case PlanningPhaseStatus::NoPlanSafeEnd:
        planFound = false;
        break;
    case PlanningPhaseStatus::PlanReady:
        if (!planning.plan) return safeFailure(ExecutionCycleFailureReason::InvalidExecutionPlan);
        plan = *planning.plan;
        if (plan.sourcePlanIdentity.shotCycleIdentity != cycleIdentity) {
            return safeFailure(ExecutionCycleFailureReason::ExecutionPlanCycleMismatch);
        }
        planFound = true;
        break;
    }

    PneumaticCompletionResult pneumatic{PneumaticCompletionStatus::Failure, std::nullopt};
    if (planFound) {
        enter(ExecutionCycleState::StrikeReady);
        if (!seam.validateStrikeReady) return missing();
        if (const auto stop = dispatchStep(
                seam.validateStrikeReady(plan),
                ExecutionCycleFailureReason::StrikeReadyValidationFailed)) {
            return *stop;
        }
        if (!seam.moveToStrikeReady) return missing();
        if (const auto stop = dispatchStep(
                seam.moveToStrikeReady(plan),
                ExecutionCycleFailureReason::StrikeReadyMotionFailed)) {
            return *stop;
        }
        if (!seam.confirmStrikeReadyStopped) return missing();
        if (const auto stop = dispatchStep(
                seam.confirmStrikeReadyStopped(),
                ExecutionCycleFailureReason::StrikeReadyNotStopped)) {
            return *stop;
        }

        enter(ExecutionCycleState::Pneumatic);
        if (!seam.runPneumatic) return missing();
        pneumatic = seam.runPneumatic(plan);
        if (!pneumatic.isValid() ||
            pneumatic.status == PneumaticCompletionStatus::UnknownUnsafe) {
            return unknownUnsafe(ExecutionCycleFailureReason::PneumaticStateUnknown);
        }
        postStrikeRecoveryRequired = true;
        if (pneumatic.status == PneumaticCompletionStatus::Failure) {
            return safeFailure(ExecutionCycleFailureReason::PneumaticFailed);
        }

        enter(ExecutionCycleState::PostStrikeActualPose);
        if (!seam.readActualPose) return missing();
        const RobotPoseAdapterResult actualPoseResult = seam.readActualPose();
        if (!actualPoseResult.isValid() ||
            actualPoseResult.status == RobotAdapterStatus::UnknownUnsafe) {
            return unknownUnsafe(ExecutionCycleFailureReason::ActualPoseReadFailed);
        }
        if (!actualPoseResult.value) {
            return safeFailure(ExecutionCycleFailureReason::ActualPoseReadFailed);
        }
        const RobotPoseABC actualPose = *actualPoseResult.value;
        const RobotPoseABC safeLift =
            buildSafeLiftTarget(actualPose, plan.safeApproachPose.z);
        if (!isValidSafeLiftTarget(actualPose, plan.safeApproachPose.z, safeLift)) {
            return safeFailure(ExecutionCycleFailureReason::InvalidActualPose);
        }

        enter(ExecutionCycleState::SafeLift);
        if (!seam.checkSafeLiftLinearPath) return missing();
        const LinearPathCheckResult path =
            seam.checkSafeLiftLinearPath(actualPose, safeLift);
        if (!path.isValid() ||
            path.status == LinearPathCheckStatus::UnknownUnsafe) {
            return unknownUnsafe(ExecutionCycleFailureReason::SafeLiftPathCheckFailed);
        }
        if (path.status == LinearPathCheckStatus::ApiFailure) {
            return safeFailure(ExecutionCycleFailureReason::SafeLiftPathCheckFailed);
        }
        if (!path.reachable) {
            return safeFailure(ExecutionCycleFailureReason::SafeLiftPathUnreachable);
        }
        if (!seam.moveSafeLiftLinear) return missing();
        if (const auto stop = dispatchStep(
                seam.moveSafeLiftLinear(safeLift),
                ExecutionCycleFailureReason::SafeLiftMotionFailed)) {
            return *stop;
        }
        if (!seam.confirmSafeLiftStopped) return missing();
        if (const auto stop = dispatchStep(
                seam.confirmSafeLiftStopped(safeLift),
                ExecutionCycleFailureReason::SafeLiftNotConfirmed)) {
            return *stop;
        }
        if (plan.strikeMode == StrikeMode::Push) {
            if (!seam.finishPneumaticAfterSafeLift) return missing();
            pneumatic = seam.finishPneumaticAfterSafeLift(plan);
            if (!pneumatic.isValid() ||
                pneumatic.status == PneumaticCompletionStatus::UnknownUnsafe) {
                return unknownUnsafe(
                    ExecutionCycleFailureReason::PneumaticStateUnknown);
            }
            if (pneumatic.status == PneumaticCompletionStatus::Failure) {
                return safeFailure(ExecutionCycleFailureReason::PneumaticFailed);
            }
        }
        postStrikeRecoveryRequired = false;
    }

    enter(ExecutionCycleState::StandbyReturn);
    if (planFound) {
        if (!seam.returnToStandbyAfterStrike) return missing();
        if (const auto stop = dispatchStep(
                seam.returnToStandbyAfterStrike(plan),
                ExecutionCycleFailureReason::StandbyReturnMotionFailed)) {
            return *stop;
        }
    } else {
        if (!seam.returnToStandby) return missing();
        if (const auto stop = dispatchStep(
                seam.returnToStandby(),
                ExecutionCycleFailureReason::StandbyReturnMotionFailed)) {
            return *stop;
        }
    }
    if (!seam.confirmStandbyReturnStopped) return missing();
    if (const auto stop = dispatchStep(
            seam.confirmStandbyReturnStopped(),
            ExecutionCycleFailureReason::StandbyReturnNotStopped)) {
        return *stop;
    }

    if (!planFound) {
        return safeFailure(ExecutionCycleFailureReason::NoExecutablePlan);
    }

    return completed(
        planFound,
        planFound
            ? std::optional<Phase1PlanIdentity>{plan.sourcePlanIdentity}
            : std::nullopt,
        planFound ? pneumatic.evidence : std::nullopt);
}
