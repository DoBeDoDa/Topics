#include "TestHarness.h"

#include "../src/BilliardApp.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace {

enum class Command {
    ConfirmPrepared,
    ConfirmStandbyReferenceApproved,
    PrepareHardware,
    IsAtStandby,
    ReturnToStandby,
    ConfirmStandbyStopped,
    MoveCamera,
    ConfirmCameraStopped,
    SettleCamera,
    AcquireExecutionPlan,
    ValidateStrikeReady,
    MoveStrikeReady,
    ConfirmStrikeStopped,
    Pneumatic,
    ReadActualPose,
    CheckSafeLiftLin,
    MoveSafeLiftLin,
    ConfirmSafeLift,
    ReturnStandbyAfterStrike,
    ConfirmStandbyReturnStopped
};

// 只需滿足runOfflineSingleCycle實際讀取到的欄位（sourcePlanIdentity、
// safeApproachPose.z等）；runOfflineSingleCycle本身不呼叫plan.isValid()，
// 深層跨欄位一致性由p2_01對createExecutionPlan()把關，此處不重複驗證。
ExecutionPlan validExecutionPlan(std::uint64_t cycleIdentity = 1)
{
    ExecutionPlan plan{};
    plan.sourcePlanIdentity = {1, cycleIdentity};
    plan.sourceShotType = ShotPlanType::DirectPot;
    plan.base0PlanarCalibrationRevision = "base0-test-v1";
    plan.tableGeometryRevision = "table-test-v1";
    plan.motionCalibrationRevision = "motion-test-v1";
    plan.cueForwardAxisCalibrationRevision = "tool-axis-test-v1";
    plan.cueBallCenterBase0Mm = {100.0, 100.0};
    plan.shotDirectionXY = {1.0, 0.0};
    plan.strikeMode = StrikeMode::Push;
    plan.physicalPlayingSurfaceBase0Mm = {0.0, 1000.0, 0.0, 500.0};
    plan.tableDownDirectionBase0XY = {0.0, -1.0};
    plan.pullModeMinBottomDistanceMm = 300.0;
    plan.bottomDistanceMm = 100.0;
    plan.tableDownDirectionDot = 0.0;
    plan.strikePositionBiasMm = 0.0;
    plan.ballRadiusMm = 25.0;
    plan.readyGapMm = 10.0;
    plan.directionUnitTolerance = 1e-9;
    plan.cToolOffsetDeg = 0.0;
    plan.validatedStrikeDirectionXY = {1.0, 0.0};
    plan.cueDirectionErrorDeg = 0.0;
    plan.maxCueDirectionErrorDeg = 1.0;
    // approach z(100) > ready z(50)：維持與checkpoint1
    // safeApproachZMm>strikeZMm不變式方向一致的fixture慣例。
    plan.safeApproachPose = {65.0, 100.0, 100.0, 0.0, 0.0, 0.0};
    plan.strikeReadyPose = {65.0, 100.0, 50.0, 0.0, 0.0, 0.0};
    plan.selectedADeg = 0.0;
    plan.selectedBDeg = 0.0;
    plan.selectedCDeg = 0.0;
    plan.selectedSearchOrdinal = 0;
    plan.poseSearchAudit = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0,
        BilliardConfig::PoseSearchOrder::AThenB,
        BilliardConfig::AxisOffsetOrder::LowerThenHigher,
        BilliardConfig::PoseTieBreak::FirstInApprovedSearchOrder, 1};
    plan.standbyJointCalibrationRevision =
        *BilliardConfig::STANDBY_JOINT_REFERENCE.calibrationRevision;
    plan.standbyJointReference = BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg;
    plan.motionIntents = {
        PlannedMotionIntent::CartesianPtpToSafeApproach,
        PlannedMotionIntent::LinearToStrikeReady,
        PlannedMotionIntent::RuntimeActualPoseVerticalSafeLift,
        PlannedMotionIntent::JointPtpToStandby};
    plan.stageContracts = {{
        {PlannedMotionIntent::CartesianPtpToSafeApproach,
         PlannedStagePrecondition::PolicyAndCalibrationAccepted,
         PlannedStageSuccessCondition::TargetReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability},
        {PlannedMotionIntent::LinearToStrikeReady,
         PlannedStagePrecondition::PreviousStageSucceeded,
         PlannedStageSuccessCondition::LinearTargetReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::MotionCheckLinRequired},
        {PlannedMotionIntent::RuntimeActualPoseVerticalSafeLift,
         PlannedStagePrecondition::RuntimeActualPoseAndPneumaticCompletion,
         PlannedStageSuccessCondition::SafeLiftReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::MotionCheckLinRequired},
        {PlannedMotionIntent::JointPtpToStandby,
         PlannedStagePrecondition::PreviousStageSucceeded,
         PlannedStageSuccessCondition::TargetReachedAndStopped,
         PlannedStageFailureTransition::StopFailClosed,
         PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability}
    }};
    plan.fixedForceEnvelope = {"force-test-v1", 100.0, 10.0, std::nullopt,
        0.0, 200.0, 90.0, std::nullopt};
    plan.pneumaticTimingProfile = {"pneumatic-test-v1", 100, 50, 100};
    plan.executionPolicyRevision = "policy-test-v1";
    plan.policyMode = BilliardConfig::ExecutionPolicyMode::PlanningTest;
    plan.policyDecision = ExecutionPolicyDecision::PotAccepted;
    plan.tool1Number = BilliardConfig::TOOL_NUMBER;
    plan.tool1ControllerCalibrationRevision = "tool1-test-v1";
    plan.rankedPotCandidatesExhausted = false;
    return plan;
}

// 可設定各步驟成功/失敗/UnknownUnsafe的record-and-replay假seam。
// atStandby/standbyReferenceApproved/planningStatus/pneumatic/actualPose/
// linearPath全部可獨立配置，涵蓋runOfflineSingleCycle所有分支。
struct FakeCycle {
    std::vector<Command> commands;
    std::optional<Command> failAt;
    std::optional<Command> unknownUnsafeAt;
    bool standbyReferenceApproved = true;
    std::optional<bool> atStandby = true;
    PlanningPhaseStatus planningStatus = PlanningPhaseStatus::PlanReady;
    std::uint64_t planCycleIdentity = 1;
    PneumaticCompletionStatus pneumaticStatus = PneumaticCompletionStatus::PolicyAccepted;
    std::optional<PneumaticCompletionEvidence> pneumaticEvidence =
        PneumaticCompletionEvidence::OffCommandAccepted;
    std::optional<RobotPoseABC> actualPose =
        RobotPoseABC{70.0, 80.0, 55.0, 3.0, 4.0, 5.0};
    bool actualPoseUnknownUnsafe = false;
    LinearPathCheckStatus linearCheckStatus = LinearPathCheckStatus::Success;
    bool linearReachable = true;
    std::optional<RobotPoseABC> checkedLiftStart;
    std::optional<RobotPoseABC> checkedLiftTarget;
    std::optional<RobotPoseABC> movedLiftTarget;
    std::optional<RobotPoseABC> confirmedLiftTarget;

    OfflineStepResult step(Command command)
    {
        commands.push_back(command);
        if (unknownUnsafeAt == command) return {OfflineStepStatus::UnknownUnsafe};
        return {failAt == command
            ? OfflineStepStatus::Failure
            : OfflineStepStatus::Success};
    }

    OfflineExecutionSeam seam()
    {
        OfflineExecutionSeam s;
        s.confirmPreparedForCycle = [this] { return step(Command::ConfirmPrepared); };
        s.confirmStandbyReferenceApproved = [this]() -> OfflineStepResult {
            commands.push_back(Command::ConfirmStandbyReferenceApproved);
            if (unknownUnsafeAt == Command::ConfirmStandbyReferenceApproved) {
                return {OfflineStepStatus::UnknownUnsafe};
            }
            return {standbyReferenceApproved
                ? OfflineStepStatus::Success
                : OfflineStepStatus::Failure};
        };
        s.prepareHardwareForMotion = [this] { return step(Command::PrepareHardware); };
        s.isAtStandby = [this]() -> std::optional<bool> {
            commands.push_back(Command::IsAtStandby);
            return atStandby;
        };
        s.returnToStandby = [this] { return step(Command::ReturnToStandby); };
        s.confirmStandbyStopped = [this] { return step(Command::ConfirmStandbyStopped); };
        s.moveToCameraPose = [this] { return step(Command::MoveCamera); };
        s.confirmCameraPoseStopped = [this] { return step(Command::ConfirmCameraStopped); };
        s.settleCamera = [this] { return step(Command::SettleCamera); };
        s.acquireExecutionPlan = [this]() -> PlanningPhaseResult {
            commands.push_back(Command::AcquireExecutionPlan);
            if (planningStatus != PlanningPhaseStatus::PlanReady) {
                return {planningStatus, std::nullopt};
            }
            return {PlanningPhaseStatus::PlanReady, validExecutionPlan(planCycleIdentity)};
        };
        s.validateStrikeReady = [this](const ExecutionPlan&) {
            return step(Command::ValidateStrikeReady);
        };
        s.moveToStrikeReady = [this](const ExecutionPlan&) {
            return step(Command::MoveStrikeReady);
        };
        s.confirmStrikeReadyStopped = [this] { return step(Command::ConfirmStrikeStopped); };
        s.runPneumatic = [this](const ExecutionPlan&) -> PneumaticCompletionResult {
            commands.push_back(Command::Pneumatic);
            return {pneumaticStatus, pneumaticEvidence};
        };
        s.readActualPose = [this]() -> RobotPoseAdapterResult {
            commands.push_back(Command::ReadActualPose);
            if (actualPoseUnknownUnsafe) {
                return {RobotAdapterStatus::UnknownUnsafe, -1, std::nullopt};
            }
            if (!actualPose) {
                return {RobotAdapterStatus::SdkFailure, -1, std::nullopt};
            }
            return {RobotAdapterStatus::Success, 0, actualPose};
        };
        s.checkSafeLiftLinearPath = [this](
                const RobotPoseABC& start, const RobotPoseABC& lift) -> LinearPathCheckResult {
            commands.push_back(Command::CheckSafeLiftLin);
            checkedLiftStart = start;
            checkedLiftTarget = lift;
            return {linearCheckStatus, linearReachable};
        };
        s.moveSafeLiftLinear = [this](const RobotPoseABC& lift) {
            movedLiftTarget = lift;
            return step(Command::MoveSafeLiftLin);
        };
        s.confirmSafeLiftStopped = [this](const RobotPoseABC& lift) {
            confirmedLiftTarget = lift;
            return step(Command::ConfirmSafeLift);
        };
        s.returnToStandbyAfterStrike = [this](const ExecutionPlan&) {
            return step(Command::ReturnStandbyAfterStrike);
        };
        s.confirmStandbyReturnStopped = [this] {
            return step(Command::ConfirmStandbyReturnStopped);
        };
        return s;
    }
};

ExecutionCycleResult run(
    FakeCycle& fake,
    OfflineExecutionRuntime& runtime,
    std::uint64_t cycleIdentity = 1)
{
    const OfflineExecutionSeam seam = fake.seam();
    return BilliardApp::runOfflineSingleCycle(runtime, cycleIdentity, seam);
}

bool hasCommand(const std::vector<Command>& commands, Command target)
{
    for (const Command c : commands) {
        if (c == target) return true;
    }
    return false;
}

std::size_t indexOf(const std::vector<Command>& commands, Command target)
{
    for (std::size_t i = 0; i < commands.size(); ++i) {
        if (commands[i] == target) return i;
    }
    return static_cast<std::size_t>(-1);
}

} // namespace

int main()
{
    TestHarness tests;

    // ---- 成功路徑：不在standby，走PreparationReturn+完整擊球 ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.atStandby = false;
        fake.planCycleIdentity = 7;
        const ExecutionCycleResult result = run(fake, runtime, 7);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed,
            "not-at-standby full shot cycle completes");
        tests.expectTrue(result.isValid(), "completed result isValid()");
        if (result.value) {
            tests.expectTrue(result.value->shotExecuted, "shot cycle marks shotExecuted");
            tests.expectTrue(
                result.value->cycleIdentity == 7, "audit carries cycleIdentity");
            tests.expectTrue(
                result.value->sourcePlanIdentity.has_value() &&
                    result.value->sourcePlanIdentity->shotCycleIdentity == 7,
                "audit carries sourcePlanIdentity matching cycle");
            tests.expectTrue(
                result.value->pneumaticEvidence.has_value() &&
                    *result.value->pneumaticEvidence ==
                        PneumaticCompletionEvidence::OffCommandAccepted,
                "audit carries pneumatic evidence");
            tests.expectTrue(
                result.value->states.front() == ExecutionCycleState::WaitingForStart &&
                    result.value->states.back() == ExecutionCycleState::WaitingForStart,
                "audit trace starts/ends at WaitingForStart");
        }
        tests.expectTrue(
            hasCommand(fake.commands, Command::ReturnToStandby) &&
                hasCommand(fake.commands, Command::ConfirmStandbyStopped),
            "PreparationReturn steps invoked when not at standby");
        tests.expectTrue(
            indexOf(fake.commands, Command::ReturnToStandby) <
                indexOf(fake.commands, Command::MoveCamera),
            "PreparationReturn happens before CameraPose");
        tests.expectTrue(
            hasCommand(fake.commands, Command::ReturnStandbyAfterStrike) &&
                !hasCommand(fake.commands, Command::ReturnToStandby) ==
                    false,
            "post-strike return uses returnToStandbyAfterStrike");
        tests.expectTrue(
            runtime.state == ExecutionCycleState::WaitingForStart,
            "runtime returns to WaitingForStart after success");
        tests.expectTrue(
            fake.checkedLiftStart.has_value() && fake.actualPose.has_value() &&
                fake.checkedLiftStart->x == fake.actualPose->x &&
                fake.checkedLiftStart->y == fake.actualPose->y &&
                fake.checkedLiftStart->z == fake.actualPose->z &&
                fake.checkedLiftStart->a == fake.actualPose->a &&
                fake.checkedLiftStart->b == fake.actualPose->b &&
                fake.checkedLiftStart->c == fake.actualPose->c,
            "the post-strike actual pose (not the planned pose) is used as the "
            "safe-lift LIN start point");
        tests.expectTrue(
            fake.checkedLiftTarget.has_value() && fake.movedLiftTarget.has_value() &&
                fake.checkedLiftTarget->x == fake.movedLiftTarget->x &&
                fake.checkedLiftTarget->y == fake.movedLiftTarget->y &&
                fake.checkedLiftTarget->a == fake.movedLiftTarget->a &&
                fake.checkedLiftTarget->b == fake.movedLiftTarget->b &&
                fake.checkedLiftTarget->c == fake.movedLiftTarget->c &&
                fake.checkedLiftTarget->z == fake.movedLiftTarget->z &&
                fake.checkedLiftTarget->x == fake.actualPose->x &&
                fake.checkedLiftTarget->y == fake.actualPose->y &&
                fake.checkedLiftTarget->a == fake.actualPose->a &&
                fake.checkedLiftTarget->b == fake.actualPose->b &&
                fake.checkedLiftTarget->c == fake.actualPose->c &&
                fake.checkedLiftTarget->z == 100.0,
            "safe-lift target preserves actual post-strike X/Y/A/B/C exactly and "
            "only Z is set to plan.safeApproachPose.z, checked before the move is issued");
        tests.expectTrue(
            fake.confirmedLiftTarget.has_value() &&
                fake.confirmedLiftTarget->x == fake.checkedLiftTarget->x &&
                fake.confirmedLiftTarget->y == fake.checkedLiftTarget->y &&
                fake.confirmedLiftTarget->z == fake.checkedLiftTarget->z &&
                fake.confirmedLiftTarget->a == fake.checkedLiftTarget->a &&
                fake.confirmedLiftTarget->b == fake.checkedLiftTarget->b &&
                fake.confirmedLiftTarget->c == fake.checkedLiftTarget->c &&
                fake.confirmedLiftTarget->x == fake.movedLiftTarget->x &&
                fake.confirmedLiftTarget->y == fake.movedLiftTarget->y &&
                fake.confirmedLiftTarget->z == fake.movedLiftTarget->z,
            "confirmSafeLiftStopped is confirming a stop at the exact same lift target "
            "that was checked and moved to, not a discarded/different pose, before "
            "standby return is allowed to proceed");
        tests.expectTrue(
            indexOf(fake.commands, Command::CheckSafeLiftLin) <
                indexOf(fake.commands, Command::MoveSafeLiftLin) &&
            indexOf(fake.commands, Command::MoveSafeLiftLin) <
                indexOf(fake.commands, Command::ConfirmSafeLift) &&
            indexOf(fake.commands, Command::ConfirmSafeLift) <
                indexOf(fake.commands, Command::ReturnStandbyAfterStrike),
            "safe-lift sequence happens in the exact required order: "
            "check LIN path -> move LIN -> confirm stopped -> return to standby");
    }

    // ---- 成功路徑：已在standby，不含PreparationReturn ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.atStandby = true;
        fake.planCycleIdentity = 3;
        const ExecutionCycleResult result = run(fake, runtime, 3);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed,
            "at-standby shot cycle completes");
        tests.expectTrue(result.isValid(), "at-standby completed result isValid()");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::ReturnToStandby),
            "no PreparationReturn commands issued when already at standby");
        if (result.value) {
            tests.expectTrue(
                result.value->isValid(),
                "at-standby audit trims to fixed template without PreparationReturn");
        }
    }

    // ---- Re-entrant：cycle正在進行時第二次Start必須被拒 ----
    {
        OfflineExecutionRuntime runtime;
        runtime.state = ExecutionCycleState::CameraPose;
        FakeCycle fake;
        const ExecutionCycleResult result = run(fake, runtime, 1);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::StartRejected,
            "active cycle rejects concurrent start");
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::CycleAlreadyActive,
            "rejection reason is CycleAlreadyActive");
        tests.expectTrue(fake.commands.empty(), "no seam calls made on rejected start");
        tests.expectTrue(
            runtime.state == ExecutionCycleState::CameraPose,
            "runtime state untouched by rejected start");
    }

    // ---- cycleIdentity==0必須被拒（配置耗盡） ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        const ExecutionCycleResult result = run(fake, runtime, 0);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::StartRejected,
            "cycleIdentity zero rejected");
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::CycleIdentityExhausted,
            "rejection reason is CycleIdentityExhausted");
        tests.expectTrue(fake.commands.empty(), "no seam calls made when identity exhausted");
    }

    // ---- confirmPreparedForCycle failure ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.failAt = Command::ConfirmPrepared;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure,
            "confirmPreparedForCycle failure is SafeFailure");
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::PreparationCheckFailed,
            "reason is PreparationCheckFailed");
        tests.expectTrue(
            fake.commands.size() == 1, "stops immediately, no further hardware calls");
        tests.expectTrue(
            runtime.state == ExecutionCycleState::WaitingForStart,
            "runtime returns to WaitingForStart (no post-strike recovery pending)");
    }

    // ---- confirmPreparedForCycle UnknownUnsafe ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.unknownUnsafeAt = Command::ConfirmPrepared;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::UnknownUnsafe,
            "confirmPreparedForCycle UnknownUnsafe preserved distinctly");
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->stoppedState == ExecutionCycleState::UnknownUnsafe,
            "diagnostic stoppedState is UnknownUnsafe");
        tests.expectTrue(
            runtime.state == ExecutionCycleState::UnknownUnsafe,
            "runtime latches UnknownUnsafe");

        FakeCycle fakeNext;
        const ExecutionCycleResult blocked = run(fakeNext, runtime);
        tests.expectTrue(
            blocked.status == ExecutionCycleStatus::StartRejected,
            "UnknownUnsafe latch blocks subsequent start");
    }

    // ---- confirmStandbyReferenceApproved==false（未核准revision，不得繞過） ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.standbyReferenceApproved = false;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::PreparationCheckFailed,
            "unapproved standby reference fails closed with PreparationCheckFailed");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::PrepareHardware),
            "no hardware-mutating call issued before standby reference is approved");
    }

    // ---- isAtStandby()回傳nullopt -> UnknownUnsafe（無法確認即fail closed） ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.atStandby = std::nullopt;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::UnknownUnsafe,
            "undeterminable standby state is UnknownUnsafe");
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::PreparationCheckFailed,
            "reason is PreparationCheckFailed");
    }

    // ---- PreparationReturn: returnToStandby failure ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.atStandby = false;
        fake.failAt = Command::ReturnToStandby;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::PreparationReturnMotionFailed,
            "PreparationReturn motion failure classified correctly");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::MoveCamera),
            "CameraPose not reached after PreparationReturn motion failure");
    }

    // ---- PreparationReturn: confirmStandbyStopped failure ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.atStandby = false;
        fake.failAt = Command::ConfirmStandbyStopped;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::PreparationReturnNotStopped,
            "PreparationReturn stop confirmation failure classified correctly");
    }

    // ---- CameraPose: moveToCameraPose / confirmCameraPoseStopped failure ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.failAt = Command::MoveCamera;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::CameraPoseMotionFailed,
            "camera pose motion failure classified correctly");
    }
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.failAt = Command::ConfirmCameraStopped;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::CameraPoseNotStopped,
            "camera pose stop confirmation failure classified correctly");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::SettleCamera),
            "camera settling not reached after stop confirmation failure");
    }

    // ---- CameraSettling failure ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.failAt = Command::SettleCamera;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::CameraSettleFailed,
            "camera settle failure classified correctly");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::AcquireExecutionPlan),
            "planning not reached after settle failure");
    }

    // ---- Planning: NoPlanSafeEnd -> completed без擊球，仍走standby return ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.planningStatus = PlanningPhaseStatus::NoPlanSafeEnd;
        const ExecutionCycleResult result = run(fake, runtime, 5);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::NoExecutablePlan,
            "NoPlanSafeEnd reports SafeFailure after a safe standby return");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::ValidateStrikeReady) &&
                !hasCommand(fake.commands, Command::Pneumatic),
            "no strike-ready/pneumatic steps when no plan found");
        tests.expectTrue(
            hasCommand(fake.commands, Command::ReturnToStandby),
            "no-plan path still returns to standby via returnToStandby (not AfterStrike)");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::ReturnStandbyAfterStrike),
            "no-plan path does not call returnToStandbyAfterStrike");
        tests.expectTrue(
            runtime.state == ExecutionCycleState::WaitingForStart,
            "NoPlanSafeEnd returns runtime to WaitingForStart");
    }

    // ---- Planning: Failure ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.planningStatus = PlanningPhaseStatus::Failure;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::CaptureAndPlanFailed,
            "planning Failure classified as CaptureAndPlanFailed SafeFailure");
        tests.expectTrue(
            runtime.state == ExecutionCycleState::WaitingForStart,
            "planning failure (pre-strike) allows immediate restart, no manual recovery");
    }

    // ---- Planning: UnknownUnsafe ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.planningStatus = PlanningPhaseStatus::UnknownUnsafe;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::UnknownUnsafe,
            "planning UnknownUnsafe preserved distinctly, not collapsed into SafeFailure");
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::CaptureAndPlanFailed,
            "reason is CaptureAndPlanFailed");
    }

    // ---- Planning: ManualRecoveryRequired（vision reconnect exhausted）----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.planningStatus = PlanningPhaseStatus::ManualRecoveryRequired;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::VisionReconnectManualRecoveryRequired,
            "vision reconnect exhaustion maps to VisionReconnectManualRecoveryRequired");
        tests.expectTrue(
            result.diagnostic->stoppedState == ExecutionCycleState::ManualRecoveryRequired,
            "stoppedState is ManualRecoveryRequired");
        tests.expectTrue(
            runtime.state == ExecutionCycleState::ManualRecoveryRequired,
            "runtime latches ManualRecoveryRequired");

        FakeCycle fakeNext;
        const ExecutionCycleResult blocked = run(fakeNext, runtime);
        tests.expectTrue(
            blocked.status == ExecutionCycleStatus::StartRejected,
            "ManualRecoveryRequired latch blocks subsequent start");
    }

    // ---- Planning: PlanReady但cycleIdentity不符（stale plan） ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.planCycleIdentity = 999;
        const ExecutionCycleResult result = run(fake, runtime, 1);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::ExecutionPlanCycleMismatch,
            "stale plan identity rejected as ExecutionPlanCycleMismatch");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::ValidateStrikeReady),
            "mismatched plan never reaches StrikeReady");
    }

    // ---- StrikeReady: validateStrikeReady / moveToStrikeReady / confirmStrikeReadyStopped failure ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.failAt = Command::ValidateStrikeReady;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::StrikeReadyValidationFailed,
            "strike-ready validation failure classified correctly");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::Pneumatic),
            "pneumatic never fires when strike-ready validation fails");
    }
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.failAt = Command::MoveStrikeReady;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::StrikeReadyMotionFailed,
            "strike-ready motion failure classified correctly");
    }
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.failAt = Command::ConfirmStrikeStopped;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::StrikeReadyNotStopped,
            "strike-ready stop confirmation failure classified correctly");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::Pneumatic),
            "pneumatic never fires before strike-ready stop is confirmed");
    }

    // ---- Pneumatic: known Failure -> ManualRecoveryRequired latch ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.pneumaticStatus = PneumaticCompletionStatus::Failure;
        fake.pneumaticEvidence = std::nullopt;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
                result.diagnostic &&
                result.diagnostic->reason == ExecutionCycleFailureReason::PneumaticFailed,
            "pneumatic known failure classified as PneumaticFailed");
        tests.expectTrue(
            runtime.state == ExecutionCycleState::ManualRecoveryRequired,
            "post-strike pneumatic failure requires manual recovery (cannot auto-restart)");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::ReadActualPose),
            "actual pose never read after pneumatic failure");

        FakeCycle fakeNext;
        const ExecutionCycleResult blocked = run(fakeNext, runtime);
        tests.expectTrue(
            blocked.status == ExecutionCycleStatus::StartRejected,
            "ManualRecoveryRequired after pneumatic failure blocks next start");
    }

    // ---- Pneumatic: UnknownUnsafe ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.pneumaticStatus = PneumaticCompletionStatus::UnknownUnsafe;
        fake.pneumaticEvidence = std::nullopt;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::UnknownUnsafe &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::PneumaticStateUnknown,
            "pneumatic UnknownUnsafe classified as PneumaticStateUnknown, distinct terminal state");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::ReadActualPose) &&
                !hasCommand(fake.commands, Command::CheckSafeLiftLin) &&
                !hasCommand(fake.commands, Command::MoveSafeLiftLin) &&
                !hasCommand(fake.commands, Command::ReturnStandbyAfterStrike),
            "pneumatic UnknownUnsafe stops immediately: no actual-pose read, "
            "safe-lift check/move, or standby return is ever issued");
    }

    // ---- Pneumatic: 結構不合法（PolicyAccepted卻無evidence）視為UnknownUnsafe ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.pneumaticEvidence = std::nullopt; // status仍是PolicyAccepted，isValid()=false
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::UnknownUnsafe,
            "structurally invalid pneumatic result fails closed to UnknownUnsafe");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::ReadActualPose) &&
                !hasCommand(fake.commands, Command::CheckSafeLiftLin) &&
                !hasCommand(fake.commands, Command::MoveSafeLiftLin) &&
                !hasCommand(fake.commands, Command::ReturnStandbyAfterStrike),
            "structurally invalid pneumatic result stops immediately: no actual-pose "
            "read, safe-lift LIN check/move, or standby-return PTP is ever issued");
    }

    // ---- PostStrikeActualPose: SdkFailure（known-safe read failure）----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.actualPose = std::nullopt;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::ActualPoseReadFailed,
            "actual pose read known failure classified correctly");
        tests.expectTrue(
            runtime.state == ExecutionCycleState::ManualRecoveryRequired,
            "post-pneumatic actual-pose read failure requires manual recovery");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::CheckSafeLiftLin),
            "safe-lift path never checked without a valid actual pose");
    }

    // ---- PostStrikeActualPose: UnknownUnsafe必須完整保留，不得壓成單純讀取失敗 ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.actualPoseUnknownUnsafe = true;
        const ExecutionCycleResult result = run(fake, runtime);

        tests.expectTrue(
            result.status == ExecutionCycleStatus::UnknownUnsafe &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::ActualPoseReadFailed,
            "actual pose UnknownUnsafe preserved distinctly from SdkFailure");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::CheckSafeLiftLin) &&
                !hasCommand(fake.commands, Command::MoveSafeLiftLin) &&
                !hasCommand(fake.commands, Command::ReturnStandbyAfterStrike),
            "actual-pose UnknownUnsafe stops immediately: no safe-lift path check/move "
            "or standby return is ever issued");
    }

    // ---- SafeLift: checkSafeLiftLinearPath ApiFailure / UnknownUnsafe / unreachable ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.linearCheckStatus = LinearPathCheckStatus::ApiFailure;
        fake.linearReachable = false;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::SafeLiftPathCheckFailed,
            "safe-lift path check ApiFailure classified correctly");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::MoveSafeLiftLin),
            "safe-lift motion never issued after failed path check");
    }
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.linearCheckStatus = LinearPathCheckStatus::UnknownUnsafe;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.status == ExecutionCycleStatus::UnknownUnsafe &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::SafeLiftPathCheckFailed,
            "safe-lift path check UnknownUnsafe preserved distinctly");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::MoveSafeLiftLin) &&
                !hasCommand(fake.commands, Command::ReturnStandbyAfterStrike),
            "safe-lift path-check UnknownUnsafe stops immediately: no safe-lift move "
            "or standby return is ever issued");
    }
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.linearCheckStatus = LinearPathCheckStatus::Success;
        fake.linearReachable = false;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::SafeLiftPathUnreachable,
            "safe-lift path unreachable (known-safe) classified distinctly from ApiFailure");
    }

    // ---- SafeLift: moveSafeLiftLinear / confirmSafeLiftStopped failure ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.failAt = Command::MoveSafeLiftLin;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::SafeLiftMotionFailed,
            "safe-lift motion failure classified correctly");
        tests.expectTrue(
            runtime.state == ExecutionCycleState::ManualRecoveryRequired,
            "safe-lift motion failure requires manual recovery (post-strike)");
    }
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.failAt = Command::ConfirmSafeLift;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::SafeLiftNotConfirmed,
            "safe-lift stop confirmation failure classified correctly");
        tests.expectTrue(
            !hasCommand(fake.commands, Command::ReturnStandbyAfterStrike),
            "standby return after strike never issued before safe-lift confirmed stopped");
    }

    // ---- StandbyReturn: returnToStandbyAfterStrike / confirmStandbyReturnStopped failure ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.failAt = Command::ReturnStandbyAfterStrike;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::StandbyReturnMotionFailed,
            "post-strike standby return motion failure classified correctly");
        // safe-lift已confirm代表機械臂已安全離開桌面上方，postStrikeRecoveryRequired
        // 已於confirmSafeLiftStopped成功後重置為false；此後失敗視為一般可重啟
        // failure，不需人工復歸。
        tests.expectTrue(
            runtime.state == ExecutionCycleState::WaitingForStart,
            "standby return failure after a confirmed safe-lift allows immediate restart");
    }
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.failAt = Command::ConfirmStandbyReturnStopped;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::StandbyReturnNotStopped,
            "standby return stop confirmation failure classified correctly");
    }

    // ---- StandbyReturn失敗（無plan路徑）不應要求人工復歸：擊發前失敗才需要 ----
    {
        OfflineExecutionRuntime runtime;
        FakeCycle fake;
        fake.planningStatus = PlanningPhaseStatus::NoPlanSafeEnd;
        fake.failAt = Command::ReturnToStandby;
        const ExecutionCycleResult result = run(fake, runtime);
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
                result.diagnostic &&
                result.diagnostic->reason ==
                    ExecutionCycleFailureReason::StandbyReturnMotionFailed,
            "no-plan standby-return failure still classified as StandbyReturnMotionFailed");
        tests.expectTrue(
            runtime.state == ExecutionCycleState::WaitingForStart,
            "no-plan path never sets postStrikeRecoveryRequired, so immediate restart allowed");
    }

    return tests.exitCode();
}
