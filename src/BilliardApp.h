// 宣告撞球應用流程協調器及其依賴元件。
#pragma once

#include <algorithm>
#include <array>
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
#include "TargetSelector.h"
#include "VisionDataParser.h"

enum class ExecutionCycleState {
    WaitingForStart,
    StartRequested,
    CameraPose,
    CameraSettling,
    CaptureWindow,
    Planning,
    StrikeReady,
    Pneumatic,
    PostStrikeActualPose,
    SafeLift,
    CameraReturn,
    CycleCompleted,
    ManualRecoveryRequired,
    UnknownUnsafe
};

enum class OfflineStepStatus {
    Success,
    Failure
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

struct OfflinePhase1Result {
    OfflinePhase1Status status;

    [[nodiscard]] bool isValid() const noexcept
    {
        return status == OfflinePhase1Status::ShotPlanReady ||
            status == OfflinePhase1Status::NoPlan ||
            status == OfflinePhase1Status::PipelineFailure;
    }
};

enum class PneumaticCompletionStatus {
    PolicyAccepted,
    Failure,
    UnknownUnsafe
};

enum class PneumaticCompletionEvidence {
    PhysicalOffConfirmed,
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
            *evidence == PneumaticCompletionEvidence::PhysicalOffConfirmed ||
            *evidence == PneumaticCompletionEvidence::OffCommandAccepted;
        return knownStatus && knownEvidence &&
            ((status == PneumaticCompletionStatus::PolicyAccepted) ==
             evidence.has_value());
    }
};

enum class LinearPathCheckStatus {
    Success,
    ApiFailure
};

struct LinearPathCheckResult {
    LinearPathCheckStatus status;
    bool reachable;

    [[nodiscard]] bool isValid() const noexcept
    {
        return status == LinearPathCheckStatus::Success || !reachable;
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
    CameraPoseMotionFailed,
    CameraPoseNotStopped,
    CameraSettleFailed,
    VisionFlushFailed,
    VisionResetFailed,
    CaptureWindowOpenFailed,
    Phase1PipelineFailed,
    InvalidPhase1Result,
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
    CameraReturnFailed,
    CameraReturnNotStopped
};

struct ExecutionCycleAudit {
    std::uint64_t cycleIdentity;
    bool shotExecuted;
    std::optional<Phase1PlanIdentity> sourcePlanIdentity;
    std::optional<PneumaticCompletionEvidence> pneumaticEvidence;
    std::vector<ExecutionCycleState> states;

    [[nodiscard]] bool isValid() const noexcept
    {
        const auto matches = [&](std::initializer_list<ExecutionCycleState> expected) {
            return states.size() == expected.size() &&
                std::equal(states.begin(), states.end(), expected.begin());
        };
        const bool noPlanTrace = matches({
            ExecutionCycleState::WaitingForStart,
            ExecutionCycleState::StartRequested,
            ExecutionCycleState::CameraPose,
            ExecutionCycleState::CameraSettling,
            ExecutionCycleState::CaptureWindow,
            ExecutionCycleState::Planning,
            ExecutionCycleState::CycleCompleted,
            ExecutionCycleState::WaitingForStart});
        const bool shotTrace = matches({
            ExecutionCycleState::WaitingForStart,
            ExecutionCycleState::StartRequested,
            ExecutionCycleState::CameraPose,
            ExecutionCycleState::CameraSettling,
            ExecutionCycleState::CaptureWindow,
            ExecutionCycleState::Planning,
            ExecutionCycleState::StrikeReady,
            ExecutionCycleState::Pneumatic,
            ExecutionCycleState::PostStrikeActualPose,
            ExecutionCycleState::SafeLift,
            ExecutionCycleState::CameraReturn,
            ExecutionCycleState::CycleCompleted,
            ExecutionCycleState::WaitingForStart});
        return cycleIdentity != 0 &&
            (shotExecuted ? shotTrace : noPlanTrace) &&
            (shotExecuted
                ? sourcePlanIdentity.has_value() &&
                  sourcePlanIdentity->isValid() &&
                  sourcePlanIdentity->shotCycleIdentity == cycleIdentity &&
                  pneumaticEvidence.has_value() &&
                  (*pneumaticEvidence ==
                       PneumaticCompletionEvidence::PhysicalOffConfirmed ||
                   *pneumaticEvidence ==
                       PneumaticCompletionEvidence::OffCommandAccepted)
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
    std::uint64_t nextCycleIdentity = 1;
};

struct OfflineExecutionSeam {
    std::function<OfflineStepResult()> moveToCameraPose;
    std::function<OfflineStepResult()> confirmCameraPoseStopped;
    std::function<OfflineStepResult()> settleCamera;
    std::function<OfflineStepResult()> flushStaleVisionBuffer;
    std::function<OfflineStepResult()> resetCycleAccumulation;
    std::function<OfflineStepResult()> openCaptureWindow;
    std::function<OfflinePhase1Result()> runPhase1;
    std::function<ExecutionPlanResult()> buildExecutionPlan;
    std::function<OfflineStepResult(const ExecutionPlan&)> validateStrikeReady;
    std::function<OfflineStepResult(const ExecutionPlan&)> moveToStrikeReady;
    std::function<OfflineStepResult()> confirmStrikeReadyStopped;
    std::function<PneumaticCompletionResult(const ExecutionPlan&)> runPneumatic;
    std::function<std::optional<RobotPoseABC>()> readActualPose;
    std::function<LinearPathCheckResult(
        const RobotPoseABC&,
        const RobotPoseABC&)> checkSafeLiftLinearPath;
    std::function<OfflineStepResult(const RobotPoseABC&)> moveSafeLiftLinear;
    std::function<OfflineStepResult(const RobotPoseABC&)> confirmSafeLiftStopped;
    std::function<OfflineStepResult(const ExecutionPlan&)> returnToCameraPose;
    std::function<OfflineStepResult()> confirmReturnCameraStopped;
};

class BilliardApp {
private:
    RobotController robot;
    SocketClient visionClient;
    VisionDataParser visionParser;
    ReceiveEventFactory receiveEventFactory;
    ThreeEventStability stability;
    std::optional<PlanningResult> pendingPlanningResult;
    TargetSelector targetSelector;
    MotionPlanner motionPlanner;
    bool needCameraMove;
    ShotCycleIdentity nextShotCycleIdentity;

public:
    BilliardApp();

    bool initialize();
    void run();
    [[nodiscard]] static ExecutionCycleResult runOfflineSingleCycle(
        OfflineExecutionRuntime& runtime,
        const OfflineExecutionSeam& seam);

private:
    bool waitForStartRequest();
    bool moveToCameraPosition();
    bool openCaptureWindowAfterCameraPose();
    bool processReceiveEvent(const ReceiveEvent& event);
    void invalidateVisionCycle(ReceiveEventInvalidationReason reason);
    bool executeMotionPlan(const MotionPlan& plan);
    bool requireReachable(
        const std::string& pointName,
        const std::array<double, 6>& pose
    );
    bool requireMotionSuccess(
        const std::string& stepName,
        const MotionResult& result
    );
    void printPose(
        const std::string& label,
        const std::array<double, 6>& pose
    ) const;
    void printAlarmCodes() const;
};

inline ExecutionCycleResult BilliardApp::runOfflineSingleCycle(
    OfflineExecutionRuntime& runtime,
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
    if (runtime.nextCycleIdentity == 0) {
        return {
            ExecutionCycleStatus::StartRejected,
            std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::CycleIdentityExhausted,
                runtime.state}};
    }

    const std::uint64_t cycleIdentity = runtime.nextCycleIdentity++;
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

    enter(ExecutionCycleState::CameraPose);
    if (!seam.moveToCameraPose) return missing();
    if (!seam.moveToCameraPose().succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::CameraPoseMotionFailed);
    }
    if (!seam.confirmCameraPoseStopped) return missing();
    if (!seam.confirmCameraPoseStopped().succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::CameraPoseNotStopped);
    }

    enter(ExecutionCycleState::CameraSettling);
    if (!seam.settleCamera) return missing();
    if (!seam.settleCamera().succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::CameraSettleFailed);
    }
    if (!seam.flushStaleVisionBuffer) return missing();
    if (!seam.flushStaleVisionBuffer().succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::VisionFlushFailed);
    }
    if (!seam.resetCycleAccumulation) return missing();
    if (!seam.resetCycleAccumulation().succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::VisionResetFailed);
    }
    if (!seam.openCaptureWindow) return missing();
    if (!seam.openCaptureWindow().succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::CaptureWindowOpenFailed);
    }
    enter(ExecutionCycleState::CaptureWindow);

    enter(ExecutionCycleState::Planning);
    if (!seam.runPhase1) return missing();
    const OfflinePhase1Result phase1 = seam.runPhase1();
    if (!phase1.isValid()) {
        return safeFailure(ExecutionCycleFailureReason::InvalidPhase1Result);
    }
    if (phase1.status == OfflinePhase1Status::PipelineFailure) {
        return safeFailure(ExecutionCycleFailureReason::Phase1PipelineFailed);
    }
    if (phase1.status == OfflinePhase1Status::NoPlan) {
        return completed(false, std::nullopt, std::nullopt);
    }

    if (!seam.buildExecutionPlan) return missing();
    const ExecutionPlanResult planned = seam.buildExecutionPlan();
    if (!planned.isValid() || planned.status() != ExecutionPlanStatus::Success ||
        !planned.value()) {
        return safeFailure(ExecutionCycleFailureReason::InvalidExecutionPlan);
    }
    const ExecutionPlan& plan = *planned.value();
    if (plan.sourcePlanIdentity.shotCycleIdentity != cycleIdentity) {
        return safeFailure(
            ExecutionCycleFailureReason::ExecutionPlanCycleMismatch);
    }

    enter(ExecutionCycleState::StrikeReady);
    if (!seam.validateStrikeReady) return missing();
    if (!seam.validateStrikeReady(plan).succeeded()) {
        return safeFailure(
            ExecutionCycleFailureReason::StrikeReadyValidationFailed);
    }
    if (!seam.moveToStrikeReady) return missing();
    if (!seam.moveToStrikeReady(plan).succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::StrikeReadyMotionFailed);
    }
    if (!seam.confirmStrikeReadyStopped) return missing();
    if (!seam.confirmStrikeReadyStopped().succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::StrikeReadyNotStopped);
    }

    enter(ExecutionCycleState::Pneumatic);
    if (!seam.runPneumatic) return missing();
    const PneumaticCompletionResult pneumatic = seam.runPneumatic(plan);
    if (!pneumatic.isValid()) {
        runtime.state = ExecutionCycleState::UnknownUnsafe;
        return {
            ExecutionCycleStatus::UnknownUnsafe,
            std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::PneumaticStateUnknown,
                ExecutionCycleState::UnknownUnsafe}};
    }
    if (pneumatic.status == PneumaticCompletionStatus::UnknownUnsafe) {
        runtime.state = ExecutionCycleState::UnknownUnsafe;
        return {
            ExecutionCycleStatus::UnknownUnsafe,
            std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::PneumaticStateUnknown,
                ExecutionCycleState::UnknownUnsafe}};
    }
    postStrikeRecoveryRequired = true;
    if (pneumatic.status == PneumaticCompletionStatus::Failure) {
        return safeFailure(ExecutionCycleFailureReason::PneumaticFailed);
    }

    enter(ExecutionCycleState::PostStrikeActualPose);
    if (!seam.readActualPose) return missing();
    const std::optional<RobotPoseABC> actualPose = seam.readActualPose();
    if (!actualPose) {
        return safeFailure(ExecutionCycleFailureReason::ActualPoseReadFailed);
    }
    if (!actualPose->isFinite() || !plan.safeLiftRule.isValid()) {
        return safeFailure(ExecutionCycleFailureReason::InvalidActualPose);
    }
    RobotPoseABC safeLift = *actualPose;
    safeLift.z += plan.safeLiftRule.heightMm;
    if (!safeLift.isFinite() || safeLift.x != actualPose->x ||
        safeLift.y != actualPose->y || safeLift.a != actualPose->a ||
        safeLift.b != actualPose->b || safeLift.c != actualPose->c ||
        safeLift.z <= actualPose->z) {
        return safeFailure(ExecutionCycleFailureReason::InvalidActualPose);
    }

    enter(ExecutionCycleState::SafeLift);
    if (!seam.checkSafeLiftLinearPath) return missing();
    const LinearPathCheckResult path =
        seam.checkSafeLiftLinearPath(*actualPose, safeLift);
    if (!path.isValid() || path.status == LinearPathCheckStatus::ApiFailure) {
        return safeFailure(ExecutionCycleFailureReason::SafeLiftPathCheckFailed);
    }
    if (!path.reachable) {
        return safeFailure(ExecutionCycleFailureReason::SafeLiftPathUnreachable);
    }
    if (!seam.moveSafeLiftLinear) return missing();
    if (!seam.moveSafeLiftLinear(safeLift).succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::SafeLiftMotionFailed);
    }
    if (!seam.confirmSafeLiftStopped) return missing();
    if (!seam.confirmSafeLiftStopped(safeLift).succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::SafeLiftNotConfirmed);
    }
    postStrikeRecoveryRequired = false;

    enter(ExecutionCycleState::CameraReturn);
    if (!seam.returnToCameraPose) return missing();
    if (!seam.returnToCameraPose(plan).succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::CameraReturnFailed);
    }
    if (!seam.confirmReturnCameraStopped) return missing();
    if (!seam.confirmReturnCameraStopped().succeeded()) {
        return safeFailure(ExecutionCycleFailureReason::CameraReturnNotStopped);
    }
    return completed(
        true,
        std::optional<Phase1PlanIdentity>{plan.sourcePlanIdentity},
        pneumatic.evidence);
}
