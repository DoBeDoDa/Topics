#include "TestHarness.h"

#include "../src/BilliardApp.h"

#include <algorithm>
#include <array>
#include <optional>
#include <vector>

namespace {
enum class Command {
    MoveCamera,
    ConfirmCameraStopped,
    SettleCamera,
    FlushVision,
    ResetAccumulation,
    OpenCapture,
    RunPhase1,
    BuildExecutionPlan,
    ValidateStrikeReady,
    MoveStrikeReady,
    ConfirmStrikeStopped,
    Pneumatic,
    ReadActualPose,
    CheckSafeLiftLin,
    MoveSafeLiftLin,
    ConfirmSafeLift,
    ReturnCamera,
    ConfirmReturnStopped
};

ExecutionPlan validExecutionPlan()
{
    const std::array<PlannedMotionIntent, 5> intents{
        PlannedMotionIntent::JointPtpToTransit,
        PlannedMotionIntent::CartesianPtpToSafeApproach,
        PlannedMotionIntent::LinearToStrikeReady,
        PlannedMotionIntent::RuntimeActualPoseVerticalSafeLift,
        PlannedMotionIntent::JointPtpToCamera};
    return {
        {1, 1},
        ShotPlanType::DirectPot,
        "base0-test-v1",
        "table-test-v1",
        "motion-test-v1",
        "tool-axis-test-v1",
        {100.0, 100.0},
        {1.0, 0.0},
        25.0,
        10.0,
        1e-9,
        0.0,
        {1.0, 0.0},
        0.0,
        1.0,
        {65.0, 100.0, 100.0, 0.0, 0.0, 0.0},
        {65.0, 100.0, 50.0, 0.0, 0.0, 0.0},
        0.0,
        0.0,
        0.0,
        0,
        {0.0, 0.0, 0.0, 0.0, 1.0, 1.0,
         BilliardConfig::PoseSearchOrder::AThenB,
         BilliardConfig::AxisOffsetOrder::LowerThenHigher,
         BilliardConfig::PoseTieBreak::FirstInApprovedSearchOrder,
         1},
        {SafeLiftDerivation::RuntimeActualPoseKeepXYABCIncreaseZ, 20.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
        intents,
        {{{intents[0], PlannedStagePrecondition::PolicyAndCalibrationAccepted,
           PlannedStageSuccessCondition::TargetReachedAndStopped,
           PlannedStageFailureTransition::StopFailClosed,
           PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability},
          {intents[1], PlannedStagePrecondition::PreviousStageSucceeded,
           PlannedStageSuccessCondition::TargetReachedAndStopped,
           PlannedStageFailureTransition::StopFailClosed,
           PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability},
          {intents[2], PlannedStagePrecondition::PreviousStageSucceeded,
           PlannedStageSuccessCondition::LinearTargetReachedAndStopped,
           PlannedStageFailureTransition::StopFailClosed,
           PlannedPathCheck::MotionCheckLinRequired},
          {intents[3],
           PlannedStagePrecondition::RuntimeActualPoseAndPneumaticCompletion,
           PlannedStageSuccessCondition::SafeLiftReachedAndStopped,
           PlannedStageFailureTransition::StopFailClosed,
           PlannedPathCheck::MotionCheckLinRequired},
          {intents[4], PlannedStagePrecondition::PreviousStageSucceeded,
           PlannedStageSuccessCondition::TargetReachedAndStopped,
           PlannedStageFailureTransition::StopFailClosed,
           PlannedPathCheck::ApprovedPtpPolicyAndTargetReachability}}},
        {"force-test-v1", 100.0, 10.0, 5.0, std::nullopt,
         0.0, 200.0, 90.0, 90.0, std::nullopt},
        {"pneumatic-test-v1", 100, 50, 100},
        "policy-test-v1",
        BilliardConfig::ExecutionPolicyMode::PlanningTest,
        ExecutionPolicyDecision::PotAccepted,
        BilliardConfig::TOOL_NUMBER, ExecutionToolMode::Primary,
        "tool1-test-v1"};
}

bool hasCommand(const std::vector<Command>& commands, Command command)
{
    return std::find(commands.begin(), commands.end(), command) != commands.end();
}

struct FakeCycle {
    std::vector<Command> commands;
    std::optional<Command> failAt;
    OfflinePhase1Status phase1Status = OfflinePhase1Status::ShotPlanReady;
    bool invalidExecutionPlan = false;
    bool legalContactPolicyRejected = false;
    std::uint64_t planCycleIdentity = 1;
    PneumaticCompletionStatus pneumaticStatus =
        PneumaticCompletionStatus::PolicyAccepted;
    std::optional<PneumaticCompletionEvidence> pneumaticEvidence =
        PneumaticCompletionEvidence::OffCommandAccepted;
    std::optional<RobotPoseABC> actualPose =
        RobotPoseABC{70.0, 80.0, 55.0, 3.0, 4.0, 5.0};
    LinearPathCheckStatus linearCheckStatus = LinearPathCheckStatus::Success;
    bool linearReachable = true;
    std::optional<RobotPoseABC> checkedStart;
    std::optional<RobotPoseABC> checkedLift;
    std::optional<RobotPoseABC> movedLift;

    OfflineStepResult step(Command command)
    {
        commands.push_back(command);
        return {failAt == command
            ? OfflineStepStatus::Failure
            : OfflineStepStatus::Success};
    }

    OfflineExecutionSeam seam()
    {
        return {
            [&] { return step(Command::MoveCamera); },
            [&] { return step(Command::ConfirmCameraStopped); },
            [&] { return step(Command::SettleCamera); },
            [&] { return step(Command::FlushVision); },
            [&] { return step(Command::ResetAccumulation); },
            [&] { return step(Command::OpenCapture); },
            [&] {
                commands.push_back(Command::RunPhase1);
                return OfflinePhase1Result{phase1Status};
            },
            [&] {
                commands.push_back(Command::BuildExecutionPlan);
                if (legalContactPolicyRejected) {
                    return ExecutionPlanResult::rejected(
                        ExecutionPlanStatus::NoExecutablePlan,
                        ExecutionPlanFailureReason::LegalContactNotAuthorized);
                }
                if (invalidExecutionPlan) {
                    return ExecutionPlanResult::rejected(
                          ExecutionPlanStatus::InvalidExecutionPlan,
                          ExecutionPlanFailureReason::InvalidExecutionPlanValue);
                }
                ExecutionPlan plan = validExecutionPlan();
                plan.sourcePlanIdentity.shotCycleIdentity = planCycleIdentity;
                return ExecutionPlanResult::success(plan);
            },
            [&](const ExecutionPlan&) { return step(Command::ValidateStrikeReady); },
            [&](const ExecutionPlan&) { return step(Command::MoveStrikeReady); },
            [&] { return step(Command::ConfirmStrikeStopped); },
            [&](const ExecutionPlan&) {
                commands.push_back(Command::Pneumatic);
                return PneumaticCompletionResult{
                    pneumaticStatus, pneumaticEvidence};
            },
            [&] {
                commands.push_back(Command::ReadActualPose);
                return actualPose;
            },
            [&](const RobotPoseABC& start, const RobotPoseABC& lift) {
                commands.push_back(Command::CheckSafeLiftLin);
                checkedStart = start;
                checkedLift = lift;
                return LinearPathCheckResult{
                    linearCheckStatus, linearReachable};
            },
            [&](const RobotPoseABC& lift) {
                movedLift = lift;
                return step(Command::MoveSafeLiftLin);
            },
            [&](const RobotPoseABC&) { return step(Command::ConfirmSafeLift); },
            [&](const ExecutionPlan&) { return step(Command::ReturnCamera); },
            [&] { return step(Command::ConfirmReturnStopped); }};
    }
};

ExecutionCycleResult run(FakeCycle& fake, OfflineExecutionRuntime& runtime)
{
    const OfflineExecutionSeam seam = fake.seam();
    return BilliardApp::runOfflineSingleCycle(runtime, seam);
}
}

int main()
{
    TestHarness tests;
    const std::vector<Command> successOrder{
        Command::MoveCamera, Command::ConfirmCameraStopped,
        Command::SettleCamera, Command::FlushVision,
        Command::ResetAccumulation, Command::OpenCapture,
        Command::RunPhase1, Command::BuildExecutionPlan,
        Command::ValidateStrikeReady, Command::MoveStrikeReady,
        Command::ConfirmStrikeStopped, Command::Pneumatic,
        Command::ReadActualPose, Command::CheckSafeLiftLin,
        Command::MoveSafeLiftLin, Command::ConfirmSafeLift,
        Command::ReturnCamera, Command::ConfirmReturnStopped};

    FakeCycle success;
    OfflineExecutionRuntime successRuntime;
    const ExecutionCycleResult completed = run(success, successRuntime);
    tests.expectTrue(completed.isValid() &&
        completed.status == ExecutionCycleStatus::Completed &&
        completed.value && completed.value->shotExecuted &&
        successRuntime.state == ExecutionCycleState::WaitingForStart,
        "one Start completes one fake shot cycle and returns to WaitingForStart");
    tests.expectTrue(completed.value &&
        completed.value->sourcePlanIdentity &&
        completed.value->sourcePlanIdentity->shotCycleIdentity ==
            completed.value->cycleIdentity &&
        completed.value->isValid(),
        "Completed audit accepts matching cycle and source-plan identities");
    tests.expectTrue(success.commands == successOrder,
        "full fake cycle records the exact required command order");
    tests.expectTrue(std::count(
        success.commands.begin(), success.commands.end(), Command::Pneumatic) == 1,
        "one Start performs exactly one pneumatic completion");
    FakeCycle acceptedOffCommand;
    acceptedOffCommand.pneumaticEvidence =
        PneumaticCompletionEvidence::OffCommandAccepted;
    OfflineExecutionRuntime acceptedOffRuntime;
    const auto acceptedOffResult = run(acceptedOffCommand, acceptedOffRuntime);
    tests.expectTrue(acceptedOffResult.value &&
        acceptedOffResult.value->pneumaticEvidence ==
            PneumaticCompletionEvidence::OffCommandAccepted,
        "Completed audit preserves policy-accepted OffCommand evidence");

    OfflineExecutionRuntime activeRuntime;
    activeRuntime.state = ExecutionCycleState::StrikeReady;
    FakeCycle reentrant;
    const auto rejectedStart = run(reentrant, activeRuntime);
    tests.expectTrue(rejectedStart.status == ExecutionCycleStatus::StartRejected &&
        reentrant.commands.empty(), "active cycle rejects a second Start");

    FakeCycle cameraMoveFailure;
    cameraMoveFailure.failAt = Command::MoveCamera;
    OfflineExecutionRuntime cameraMoveRuntime;
    const auto cameraMoveResult = run(cameraMoveFailure, cameraMoveRuntime);
    tests.expectTrue(cameraMoveResult.diagnostic &&
        cameraMoveResult.diagnostic->reason ==
            ExecutionCycleFailureReason::CameraPoseMotionFailed &&
        !hasCommand(cameraMoveFailure.commands, Command::OpenCapture),
        "CameraPose motion failure prevents capture");

    FakeCycle cameraStoppedFailure;
    cameraStoppedFailure.failAt = Command::ConfirmCameraStopped;
    OfflineExecutionRuntime cameraStoppedRuntime;
    run(cameraStoppedFailure, cameraStoppedRuntime);
    tests.expectFalse(hasCommand(cameraStoppedFailure.commands, Command::SettleCamera),
        "capture lifecycle waits for CameraPose stopped confirmation");
    tests.expectTrue(
        success.commands[2] == Command::SettleCamera &&
        success.commands[3] == Command::FlushVision &&
        success.commands[4] == Command::ResetAccumulation &&
        success.commands[5] == Command::OpenCapture,
        "capture order is stopped, settle, flush, reset, open");

    FakeCycle settleFailure;
    settleFailure.failAt = Command::SettleCamera;
    OfflineExecutionRuntime settleRuntime;
    run(settleFailure, settleRuntime);
    tests.expectFalse(hasCommand(settleFailure.commands, Command::FlushVision),
        "camera settle failure prevents flush and capture");
    FakeCycle flushFailure;
    flushFailure.failAt = Command::FlushVision;
    OfflineExecutionRuntime flushRuntime;
    run(flushFailure, flushRuntime);
    tests.expectFalse(hasCommand(flushFailure.commands, Command::ResetAccumulation),
        "flush failure prevents accumulation reset");
    FakeCycle resetFailure;
    resetFailure.failAt = Command::ResetAccumulation;
    OfflineExecutionRuntime resetRuntime;
    run(resetFailure, resetRuntime);
    tests.expectFalse(hasCommand(resetFailure.commands, Command::OpenCapture),
        "reset failure prevents capture-window open");
    FakeCycle openFailure;
    openFailure.failAt = Command::OpenCapture;
    OfflineExecutionRuntime openRuntime;
    run(openFailure, openRuntime);
    tests.expectFalse(hasCommand(openFailure.commands, Command::RunPhase1),
        "capture-window failure prevents Phase 1");

    FakeCycle pipelineFailure;
    pipelineFailure.phase1Status = OfflinePhase1Status::PipelineFailure;
    OfflineExecutionRuntime pipelineRuntime;
    const auto pipelineResult = run(pipelineFailure, pipelineRuntime);
    tests.expectTrue(pipelineResult.diagnostic &&
        pipelineResult.diagnostic->reason ==
            ExecutionCycleFailureReason::Phase1PipelineFailed &&
        !hasCommand(pipelineFailure.commands, Command::BuildExecutionPlan),
        "Phase1 pipeline failure is not converted to NoPlan or execution");

    FakeCycle noPlan;
    noPlan.phase1Status = OfflinePhase1Status::NoPlan;
    OfflineExecutionRuntime noPlanRuntime;
    const auto noPlanResult = run(noPlan, noPlanRuntime);
    tests.expectTrue(noPlanResult.status == ExecutionCycleStatus::Completed &&
        noPlanResult.value && !noPlanResult.value->shotExecuted &&
        !hasCommand(noPlan.commands, Command::BuildExecutionPlan),
        "NoPlan completes cycle without Robot execution");

    FakeCycle invalidPlan;
    invalidPlan.invalidExecutionPlan = true;
    OfflineExecutionRuntime invalidPlanRuntime;
    run(invalidPlan, invalidPlanRuntime);
    tests.expectFalse(hasCommand(invalidPlan.commands, Command::ValidateStrikeReady),
        "invalid ExecutionPlan causes no StrikeReady motion");

    FakeCycle stalePlan;
    stalePlan.planCycleIdentity = 99;
    OfflineExecutionRuntime stalePlanRuntime;
    const auto stalePlanResult = run(stalePlan, stalePlanRuntime);
    tests.expectTrue(stalePlanResult.diagnostic &&
        stalePlanResult.diagnostic->reason ==
            ExecutionCycleFailureReason::ExecutionPlanCycleMismatch &&
        !hasCommand(stalePlan.commands, Command::ValidateStrikeReady),
        "ExecutionPlan from another shot cycle cannot execute");

    FakeCycle strikeValidationFailure;
    strikeValidationFailure.failAt = Command::ValidateStrikeReady;
    OfflineExecutionRuntime strikeValidationRuntime;
    run(strikeValidationFailure, strikeValidationRuntime);
    tests.expectFalse(hasCommand(strikeValidationFailure.commands, Command::Pneumatic),
        "StrikeReady validation failure prevents pneumatic completion");

    FakeCycle strikeMotionFailure;
    strikeMotionFailure.failAt = Command::MoveStrikeReady;
    OfflineExecutionRuntime strikeMotionRuntime;
    run(strikeMotionFailure, strikeMotionRuntime);
    tests.expectFalse(hasCommand(strikeMotionFailure.commands, Command::Pneumatic),
        "StrikeReady motion failure prevents pneumatic completion");

    FakeCycle strikeStoppedFailure;
    strikeStoppedFailure.failAt = Command::ConfirmStrikeStopped;
    OfflineExecutionRuntime strikeStoppedRuntime;
    run(strikeStoppedFailure, strikeStoppedRuntime);
    tests.expectFalse(hasCommand(strikeStoppedFailure.commands, Command::Pneumatic),
        "StrikeReady stopped failure prevents pneumatic completion");

    FakeCycle pneumaticFailure;
    pneumaticFailure.pneumaticStatus = PneumaticCompletionStatus::Failure;
    pneumaticFailure.pneumaticEvidence.reset();
    OfflineExecutionRuntime pneumaticRuntime;
    run(pneumaticFailure, pneumaticRuntime);
    tests.expectFalse(hasCommand(pneumaticFailure.commands, Command::ReadActualPose),
        "known pneumatic failure prevents every post-strike motion");
    const std::size_t pneumaticFailureCommands = pneumaticFailure.commands.size();
    const auto blockedAfterPneumaticFailure =
        run(pneumaticFailure, pneumaticRuntime);
    tests.expectTrue(
        pneumaticRuntime.state == ExecutionCycleState::ManualRecoveryRequired &&
        blockedAfterPneumaticFailure.status == ExecutionCycleStatus::StartRejected &&
        pneumaticFailure.commands.size() == pneumaticFailureCommands,
        "pneumatic failure blocks all later Robot commands until manual recovery");

    FakeCycle unknownPneumatic;
    unknownPneumatic.pneumaticStatus = PneumaticCompletionStatus::UnknownUnsafe;
    unknownPneumatic.pneumaticEvidence.reset();
    OfflineExecutionRuntime unknownRuntime;
    const auto unknownResult = run(unknownPneumatic, unknownRuntime);
    tests.expectTrue(unknownResult.status == ExecutionCycleStatus::UnknownUnsafe &&
        unknownRuntime.state == ExecutionCycleState::UnknownUnsafe &&
        !hasCommand(unknownPneumatic.commands, Command::ReadActualPose),
        "UnknownUnsafe is terminal and blocks post-strike motion");
    const std::size_t unknownCommands = unknownPneumatic.commands.size();
    run(unknownPneumatic, unknownRuntime);
    tests.expectTrue(unknownPneumatic.commands.size() == unknownCommands,
        "UnknownUnsafe blocks a later Start");
    FakeCycle missingCompletionEvidence;
    missingCompletionEvidence.pneumaticEvidence.reset();
    OfflineExecutionRuntime missingEvidenceRuntime;
    const auto missingEvidenceResult =
        run(missingCompletionEvidence, missingEvidenceRuntime);
    tests.expectTrue(
        missingEvidenceResult.status == ExecutionCycleStatus::UnknownUnsafe &&
        !hasCommand(missingCompletionEvidence.commands, Command::ReadActualPose),
        "accepted pneumatic result without evidence becomes UnknownUnsafe");
    FakeCycle unknownCompletionStatus;
    unknownCompletionStatus.pneumaticStatus =
        static_cast<PneumaticCompletionStatus>(999);
    unknownCompletionStatus.pneumaticEvidence.reset();
    OfflineExecutionRuntime unknownStatusRuntime;
    const auto unknownStatusResult =
        run(unknownCompletionStatus, unknownStatusRuntime);
    tests.expectTrue(
        unknownStatusResult.status == ExecutionCycleStatus::UnknownUnsafe &&
        !hasCommand(unknownCompletionStatus.commands, Command::ReadActualPose),
        "unknown pneumatic status fails closed as UnknownUnsafe");
    FakeCycle unknownCompletionEvidence;
    unknownCompletionEvidence.pneumaticEvidence =
        static_cast<PneumaticCompletionEvidence>(999);
    OfflineExecutionRuntime unknownEvidenceRuntime;
    const auto unknownEvidenceResult =
        run(unknownCompletionEvidence, unknownEvidenceRuntime);
    tests.expectTrue(
        unknownEvidenceResult.status == ExecutionCycleStatus::UnknownUnsafe &&
        !hasCommand(unknownCompletionEvidence.commands, Command::ReadActualPose),
        "unknown pneumatic evidence fails closed as UnknownUnsafe");

    FakeCycle readFailure;
    readFailure.actualPose.reset();
    OfflineExecutionRuntime readRuntime;
    run(readFailure, readRuntime);
    tests.expectFalse(hasCommand(readFailure.commands, Command::CheckSafeLiftLin),
        "actual-pose read failure prevents safe lift");
    const std::size_t readFailureCommands = readFailure.commands.size();
    const auto blockedAfterReadFailure = run(readFailure, readRuntime);
    tests.expectTrue(
        readRuntime.state == ExecutionCycleState::ManualRecoveryRequired &&
        blockedAfterReadFailure.status == ExecutionCycleStatus::StartRejected &&
        readFailure.commands.size() == readFailureCommands,
        "post-strike failure blocks a later Start until manual recovery");

    tests.expectTrue(success.checkedStart && success.actualPose &&
        success.checkedStart->x == success.actualPose->x &&
        success.checkedStart->y == success.actualPose->y &&
        success.checkedStart->z == success.actualPose->z &&
        success.checkedStart->a == success.actualPose->a &&
        success.checkedStart->b == success.actualPose->b &&
        success.checkedStart->c == success.actualPose->c &&
        success.checkedLift &&
        success.checkedLift->x == success.actualPose->x &&
        success.checkedLift->y == success.actualPose->y &&
        success.checkedLift->a == success.actualPose->a &&
        success.checkedLift->b == success.actualPose->b &&
        success.checkedLift->c == success.actualPose->c &&
        success.checkedLift->z == success.actualPose->z + 20.0 &&
        success.checkedLift->x != validExecutionPlan().strikeReadyPose.x,
        "safe lift derives from actual pose, preserves XYABC, and changes only +Z");

    FakeCycle linApiFailure;
    linApiFailure.linearCheckStatus = LinearPathCheckStatus::ApiFailure;
    linApiFailure.linearReachable = false;
    OfflineExecutionRuntime linApiRuntime;
    run(linApiFailure, linApiRuntime);
    tests.expectFalse(hasCommand(linApiFailure.commands, Command::MoveSafeLiftLin),
        "LIN check API failure prevents safe lift motion");

    FakeCycle linUnreachable;
    linUnreachable.linearReachable = false;
    OfflineExecutionRuntime linUnreachableRuntime;
    run(linUnreachable, linUnreachableRuntime);
    tests.expectFalse(hasCommand(linUnreachable.commands, Command::MoveSafeLiftLin),
        "unreachable LIN prevents safe lift motion");

    FakeCycle liftMotionFailure;
    liftMotionFailure.failAt = Command::MoveSafeLiftLin;
    OfflineExecutionRuntime liftMotionRuntime;
    run(liftMotionFailure, liftMotionRuntime);
    tests.expectFalse(hasCommand(liftMotionFailure.commands, Command::ReturnCamera),
        "safe-lift motion failure prevents CameraPose return");

    FakeCycle liftConfirmFailure;
    liftConfirmFailure.failAt = Command::ConfirmSafeLift;
    OfflineExecutionRuntime liftConfirmRuntime;
    run(liftConfirmFailure, liftConfirmRuntime);
    tests.expectFalse(hasCommand(liftConfirmFailure.commands, Command::ReturnCamera),
        "unconfirmed safe lift prevents CameraPose return");
    tests.expectTrue(
        std::find(success.commands.begin(), success.commands.end(),
            Command::MoveSafeLiftLin) <
        std::find(success.commands.begin(), success.commands.end(),
            Command::ReturnCamera),
        "CameraPose return occurs only after successful vertical lift");
    ExecutionCycleAudit malformedAudit = *completed.value;
    malformedAudit.states.erase(
        malformedAudit.states.begin() + 2,
        malformedAudit.states.begin() + 5);
    tests.expectFalse(malformedAudit.isValid(),
        "Completed audit rejects an incomplete or reordered state trace");
    ExecutionCycleAudit mismatchedCycleAudit = *completed.value;
    mismatchedCycleAudit.sourcePlanIdentity->shotCycleIdentity =
        mismatchedCycleAudit.cycleIdentity + 1;
    tests.expectFalse(mismatchedCycleAudit.isValid(),
        "Completed audit rejects a source plan from another shot cycle");

    FakeCycle returnFailure;
    returnFailure.failAt = Command::ReturnCamera;
    OfflineExecutionRuntime returnRuntime;
    const auto returnResult = run(returnFailure, returnRuntime);
    tests.expectTrue(returnResult.diagnostic &&
        returnResult.diagnostic->reason ==
            ExecutionCycleFailureReason::CameraReturnFailed,
        "CameraPose return failure is named");

    FakeCycle returnStoppedFailure;
    returnStoppedFailure.failAt = Command::ConfirmReturnStopped;
    OfflineExecutionRuntime returnStoppedRuntime;
    const auto returnStoppedResult = run(returnStoppedFailure, returnStoppedRuntime);
    tests.expectTrue(returnStoppedResult.diagnostic &&
        returnStoppedResult.diagnostic->reason ==
            ExecutionCycleFailureReason::CameraReturnNotStopped,
        "CameraPose stopped failure is named");

    FakeCycle legalBlocked;
    legalBlocked.legalContactPolicyRejected = true;
    OfflineExecutionRuntime legalRuntime;
    run(legalBlocked, legalRuntime);
    tests.expectFalse(hasCommand(legalBlocked.commands, Command::ValidateStrikeReady),
        "P2-01 policy rejection, including default LegalContact block, is not bypassed");

    return tests.exitCode();
}
