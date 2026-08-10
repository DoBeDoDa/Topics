// 協調視覺解析、目標選擇、擊球規劃與手臂動作，不實作個別演算法細節。
#include "BilliardApp.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

#include "BilliardConfig.h"
#include "MathUtils.h"

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
                !config->cToolOffsetDeg || !pose.isFinite() ||
                axis != *config->cueForwardAxisTool) {
                return std::nullopt;
            }
            const double radians =
                (pose.c - *config->cToolOffsetDeg) *
                BilliardMath::PI / 180.0;
            const Vector2D projected{std::cos(radians), std::sin(radians)};
            return BilliardMath::isFinite(projected)
                ? std::optional<Vector2D>{projected}
                : std::nullopt;
        }};
}

}  // namespace

BilliardApp::BilliardApp()
    : receiveEventFactory(visionParser),
      stability(productionStabilityConfig()),
      needCameraMove(true),
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

ExecutionCycleResult BilliardApp::runRealSingleCycle(
    OfflineExecutionRuntime& runtime,
    RobotController& robotController,
    const std::optional<BilliardConfig::RealHardwareExecutionConfig>& config,
    const RealExecutionCycleServices& services)
{
    if (runtime.state != ExecutionCycleState::WaitingForStart ||
        runtime.nextCycleIdentity == 0) {
        return {ExecutionCycleStatus::StartRejected, std::nullopt,
            ExecutionCycleDiagnostic{
                runtime.nextCycleIdentity == 0
                    ? ExecutionCycleFailureReason::CycleIdentityExhausted
                    : ExecutionCycleFailureReason::CycleAlreadyActive,
                runtime.state}};
    }
    bool rankedSelectionAvailable =
        services.currentPlanningResult && services.buildExecutionPlanForTool;
#ifdef BILLIARDS_P2_03_TEST_SEAM
    const bool testOnlyLegacySelection =
        services.testOnlyAllowSinglePlanCompatibility &&
        services.buildExecutionPlan;
#else
    const bool testOnlyLegacySelection = false;
#endif
    if (!services.settleCamera || !services.flushStaleVisionBuffer ||
        !services.resetCycleAccumulation || !services.openCaptureWindow ||
        !services.runPhase1 ||
        (!rankedSelectionAvailable && !testOnlyLegacySelection) ||
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
                ExecutionCycleFailureReason::HardwareConnectionFailed,
                runtime.state}};
    }
    const RobotAdapterResult startupOutputs =
        robotController.establishSafeOutputsOff(config);
    if (!startupOutputs.succeeded()) {
        if (startupOutputs.status == RobotAdapterStatus::UnknownUnsafe ||
            startupOutputs.status == RobotAdapterStatus::SdkFailure ||
            startupOutputs.status == RobotAdapterStatus::NotConnected) {
            runtime.state = ExecutionCycleState::UnknownUnsafe;
            return {ExecutionCycleStatus::UnknownUnsafe, std::nullopt,
                ExecutionCycleDiagnostic{
                    ExecutionCycleFailureReason::PneumaticStateUnknown,
                    ExecutionCycleState::UnknownUnsafe}};
        }
        return {ExecutionCycleStatus::SafeFailure, std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::InvalidExecutionPlan,
                runtime.state}};
    }
    if (!robotController.clearAlarm().succeeded() ||
        !robotController.activateConfiguredToolAndBase(config).succeeded() ||
        !robotController.setMotorState(1).succeeded() ||
        !robotController.setOverrideRatio(
             BilliardConfig::NORMAL_SPEED_RATIO).succeeded()) {
        return {ExecutionCycleStatus::SafeFailure, std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::InvalidExecutionPlan,
                runtime.state}};
    }
    std::optional<ExecutionPlan> activePlan;
    std::optional<RobotPoseABC> postStrikeActual;
    bool oppositeToolPrepared = false;
    bool oppositeToolRetracted = false;
    bool oppositePreparationUnknownUnsafe = false;
    const auto step = [](const RobotAdapterResult& result) {
        return OfflineStepResult{result.succeeded()
            ? OfflineStepStatus::Success
            : OfflineStepStatus::Failure};
    };

    OfflineExecutionSeam seam;
    seam.moveToCameraPose = [&] {
        return step(robotController.checkedConfiguredJointPtp(
            BilliardConfig::CAMERA_JOINT, config));
    };
    seam.confirmCameraPoseStopped = [&] {
        return step(robotController.confirmStopped());
    };
    seam.settleCamera = services.settleCamera;
    seam.flushStaleVisionBuffer = services.flushStaleVisionBuffer;
    seam.resetCycleAccumulation = services.resetCycleAccumulation;
    seam.openCaptureWindow = services.openCaptureWindow;
    seam.runPhase1 = services.runPhase1;
    seam.buildExecutionPlan = [&] {
        if (services.currentPlanningResult &&
            services.buildExecutionPlanForTool) {
            const PlanningResult* phase1 = services.currentPlanningResult();
            if (!phase1 || !phase1->isValid()) {
                activePlan.reset();
                return ExecutionPlanResult::rejected(
                    ExecutionPlanStatus::InvalidExecutionPlan,
                    ExecutionPlanFailureReason::InvalidExecutionPlanValue);
            }
            const Phase1ExecutionCandidates& candidates =
                phase1->executionCandidates();
            const auto tryCandidates = [&](const std::vector<ShotPlan>& plans,
                                           bool potsExhausted)
                -> std::optional<ExecutionPlanResult> {
                const std::array<ExecutionToolSelection, 2> tools{{
                    {config->primaryToolNumber, ExecutionToolMode::Primary,
                     *config->requiredPrimaryToolCalibrationRevision,
                     potsExhausted},
                    {config->oppositeToolNumber, ExecutionToolMode::Opposite,
                     *config->requiredOppositeToolCalibrationRevision,
                     potsExhausted}}};
                for (const ShotPlan& shot : plans) {
                    bool candidateRejected = false;
                    for (const ExecutionToolSelection& tool : tools) {
                        const ExecutionPlanResult planned =
                            services.buildExecutionPlanForTool(shot, tool);
                        if (!planned.isValid()) return planned;
                        if (planned.status() != ExecutionPlanStatus::Success ||
                            !planned.value()) {
                            const auto reason = planned.diagnostic()
                                ? planned.diagnostic()->reason
                                : ExecutionPlanFailureReason::
                                      InvalidExecutionPlanValue;
                            if (reason == ExecutionPlanFailureReason::
                                    FixedForceEnvelopeRejected ||
                                reason == ExecutionPlanFailureReason::
                                    NoAcceptedPoseCandidate) {
                                candidateRejected = true;
                                break;
                            }
                            return planned;
                        }
                        const RobotAdapterResult preflight =
                            robotController.preflightExecution(
                                *planned.value(), config);
                        if (preflight.status ==
                            RobotAdapterStatus::NotReachable) {
                            continue;
                        }
                        if (!preflight.succeeded()) {
                            return ExecutionPlanResult::rejected(
                                ExecutionPlanStatus::InvalidExecutionPlan,
                                ExecutionPlanFailureReason::
                                    InvalidExecutionPlanValue);
                        }
                        activePlan = *planned.value();
                        return planned;
                    }
                    if (candidateRejected) continue;
                }
                return std::nullopt;
            };
            if (const auto selected =
                    tryCandidates(candidates.rankedPotPlans, false)) {
                return *selected;
            }
            if (const auto selected =
                    tryCandidates(candidates.legalContactPlans, true)) {
                return *selected;
            }
            activePlan.reset();
            return ExecutionPlanResult::rejected(
                ExecutionPlanStatus::NoExecutablePlan,
                ExecutionPlanFailureReason::NoAcceptedPoseCandidate);
        }
#ifdef BILLIARDS_P2_03_TEST_SEAM
        if (!services.testOnlyAllowSinglePlanCompatibility) {
            activePlan.reset();
            return ExecutionPlanResult::rejected(
                ExecutionPlanStatus::InvalidExecutionPlan,
                ExecutionPlanFailureReason::InvalidExecutionPlanValue);
        }
        const ExecutionPlanResult planned = services.buildExecutionPlan();
        if (!planned.isValid() ||
            planned.status() != ExecutionPlanStatus::Success ||
            !planned.value() ||
            !RobotController::validateRealExecutionConfiguration(
                 *planned.value(), config).succeeded()) {
            activePlan.reset();
            return ExecutionPlanResult::rejected(
                ExecutionPlanStatus::InvalidExecutionPlan,
                ExecutionPlanFailureReason::InvalidExecutionPlanValue);
        }
        activePlan = *planned.value();
        return planned;
#else
        activePlan.reset();
        return ExecutionPlanResult::rejected(
            ExecutionPlanStatus::InvalidExecutionPlan,
            ExecutionPlanFailureReason::InvalidExecutionPlanValue);
#endif
    };
    seam.validateStrikeReady = [&](const ExecutionPlan& plan) {
        const RobotAdapterResult activated =
            robotController.activateConfiguredToolAndBase(plan, config);
        if (!activated.succeeded()) return step(activated);
        if (plan.selectedToolMode != ExecutionToolMode::Opposite) {
            return OfflineStepResult{OfflineStepStatus::Success};
        }
        const RobotAdapterResult stopped = robotController.confirmStopped();
        if (!stopped.succeeded()) return step(stopped);
        const RealPneumaticResult prepared =
            robotController.pulseExtend(plan, config);
        if (prepared.status == RealPneumaticStatus::UnknownUnsafe) {
            oppositePreparationUnknownUnsafe = true;
            return OfflineStepResult{OfflineStepStatus::Failure};
        }
        if (prepared.status != RealPneumaticStatus::Completed) {
            return OfflineStepResult{OfflineStepStatus::Failure};
        }
        oppositeToolPrepared = true;
        return step(robotController.waitDirectionChangeDelay(plan, config));
    };
    seam.moveToStrikeReady = [&](const ExecutionPlan& plan) {
        if (!robotController.checkedJointPtp(
                plan, plan.transitJointReference, config).succeeded() ||
            !robotController.checkedPtp(
                plan, plan.safeApproachPose, config).succeeded()) {
            return OfflineStepResult{OfflineStepStatus::Failure};
        }
        const RobotPoseAdapterResult actual =
            robotController.readActualPose(plan, config);
        if (!actual.isValid() || !actual.value ||
            !robotController.checkedLin(
                plan, *actual.value, plan.strikeReadyPose, config).succeeded()) {
            return OfflineStepResult{OfflineStepStatus::Failure};
        }
        return OfflineStepResult{OfflineStepStatus::Success};
    };
    seam.confirmStrikeReadyStopped = [&] {
        return step(robotController.confirmStopped());
    };
    seam.runPneumatic = [&](const ExecutionPlan& plan) {
        if (plan.selectedToolMode == ExecutionToolMode::Opposite) {
            const RealPneumaticResult retracted =
                robotController.pulseRetract(plan, config);
            oppositeToolRetracted =
                retracted.status == RealPneumaticStatus::Completed;
            if (!oppositeToolRetracted) {
                return mapRealPneumaticResult(retracted);
            }
            const RobotAdapterResult wait =
                robotController.waitMechanismCompletion(plan, config);
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
        return completionWait.succeeded()
            ? mapRealPneumaticResult(retracted)
            : PneumaticCompletionResult{
                PneumaticCompletionStatus::Failure, std::nullopt};
    };
    seam.readActualPose = [&]() -> std::optional<RobotPoseABC> {
        if (!activePlan) return std::nullopt;
        const RobotPoseAdapterResult actual =
            robotController.readActualPose(*activePlan, config);
        postStrikeActual = actual.isValid() ? actual.value : std::nullopt;
        return postStrikeActual;
    };
    seam.checkSafeLiftLinearPath = [&](const RobotPoseABC& actual,
                                       const RobotPoseABC& target) {
        if (!activePlan) {
            return LinearPathCheckResult{LinearPathCheckStatus::ApiFailure, false};
        }
        const RobotAdapterResult result = robotController.checkVerticalSafeLift(
            *activePlan, actual, target, config);
        return LinearPathCheckResult{
            result.status == RobotAdapterStatus::SdkFailure ||
                    result.status == RobotAdapterStatus::UnknownUnsafe ||
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
    seam.returnToCameraPose = [&](const ExecutionPlan& plan) {
        return step(robotController.checkedJointPtp(
            plan, plan.cameraJointReference, config));
    };
    seam.confirmReturnCameraStopped = [&] {
        return step(robotController.confirmStopped());
    };
    ExecutionCycleResult result = runOfflineSingleCycle(runtime, seam);
    if (oppositePreparationUnknownUnsafe) {
        runtime.state = ExecutionCycleState::UnknownUnsafe;
        return {ExecutionCycleStatus::UnknownUnsafe, std::nullopt,
            ExecutionCycleDiagnostic{
                ExecutionCycleFailureReason::PneumaticStateUnknown,
                ExecutionCycleState::UnknownUnsafe}};
    }
    if (oppositeToolPrepared && !oppositeToolRetracted &&
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

void BilliardApp::run()
{
    std::optional<BilliardConfig::ExecutionPolicyMode> policyMode =
        BilliardConfig::PRODUCTION_RUNTIME_MODE;
    std::optional<std::string> policyRevision;
    std::optional<bool> legalContactAuthorized;
    std::optional<BilliardConfig::ExecutionPolicyMode> motionPlanningPolicyMode;
    std::optional<BilliardConfig::MotionPlanningConfig> motionPlanningConfig =
        BilliardConfig::MOTION_PLANNING_CONFIG;
    std::optional<BilliardConfig::RealHardwareExecutionConfig> realConfig =
        BilliardConfig::REAL_HARDWARE_EXECUTION_CONFIG;
    RobotController* cycleRobot = &robot;
    std::function<bool()> startRequested = [this] {
        return waitForStartRequest();
    };
    std::optional<RealExecutionCycleServices> realServices;

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
        startRequested = runTestSeam->startRequested;
        realServices = runTestSeam->realServices;
        if (runTestSeam->motionPlanningConfig) {
            motionPlanningConfig = runTestSeam->motionPlanningConfig;
        }
        motionPlanningPolicyMode = runTestSeam->motionPlanningPolicyMode;
    }
#endif

    if (!policyMode || !startRequested) {
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
                    return OfflineStepResult{OfflineStepStatus::Success};
                };
                services.openCaptureWindow = [&] {
                    if (nextShotCycleIdentity == 0) {
                        return OfflineStepResult{OfflineStepStatus::Failure};
                    }
                    const ShotCycleIdentity cycleIdentity =
                        nextShotCycleIdentity++;
                    receiveEventFactory.beginCycle(
                        visionClient.connectionIdentity(), cycleIdentity);
                    return OfflineStepResult{
                        receiveEventFactory.openCaptureWindow(
                            visionClient.connectionIdentity(), cycleIdentity)
                            ? OfflineStepStatus::Success
                            : OfflineStepStatus::Failure};
                };
                services.runPhase1 = [&] {
                    while (true) {
                        const SocketReceiveResult received =
                            visionClient.receiveFrame();
                        if (!received.isValid() ||
                            received.status != SocketReceiveStatus::FrameReady ||
                            !received.frame) {
                            return OfflinePhase1Result{
                                OfflinePhase1Status::PipelineFailure};
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
                            return OfflinePhase1Result{
                                OfflinePhase1Status::PipelineFailure};
                        }
                        if (!processReceiveEvent(*eventResult.value())) {
                            if (!pendingPlanningResult ||
                                !pendingPlanningResult->isValid()) {
                                return OfflinePhase1Result{
                                    OfflinePhase1Status::PipelineFailure};
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
                return OfflineStepResult{OfflineStepStatus::Success};
            };
            services.openCaptureWindow = [&, connectionIdentity] {
                if (nextShotCycleIdentity == 0) {
                    return OfflineStepResult{OfflineStepStatus::Failure};
                }
                const ShotCycleIdentity cycleIdentity =
                    nextShotCycleIdentity++;
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
                            OfflinePhase1Status::PipelineFailure};
                    }
                    if (!processReceiveEvent(*eventResult.value())) {
                        if (!pendingPlanningResult ||
                            !pendingPlanningResult->isValid()) {
                            return OfflinePhase1Result{
                                OfflinePhase1Status::PipelineFailure};
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
                    OfflinePhase1Status::PipelineFailure};
            };
        }
#endif
        if (!services.currentPlanningResult && !injectedServices) {
            services.currentPlanningResult = [&]() -> const PlanningResult* {
                return pendingPlanningResult ? &*pendingPlanningResult : nullptr;
            };
        }
        if (!services.buildExecutionPlanForTool && !injectedServices) {
            services.buildExecutionPlanForTool =
                [&](const ShotPlan& shot,
                    const ExecutionToolSelection& tool) {
                return motionPlanner.createExecutionPlan(
                    PlanningResult::shotPlan(shot),
                    motionPlanningConfig,
                    offlineMotionPlanningChecks.value_or(
                        MotionPlanningChecks{}),
                    tool);
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
                const ExecutionToolSelection primaryTool{
                    BilliardConfig::TOOL_NUMBER,
                    ExecutionToolMode::Primary,
                    motionPlanningConfig &&
                            motionPlanningConfig
                                ->primaryToolControllerCalibrationRevision
                        ? *motionPlanningConfig
                              ->primaryToolControllerCalibrationRevision
                        : std::string{},
                    potCandidatesExhausted};
                return motionPlanner.createExecutionPlan(
                    PlanningResult::shotPlan(*selected),
                    motionPlanningConfig,
                    offlineMotionPlanningChecks.value_or(
                        MotionPlanningChecks{}),
                    primaryTool);
            };
        }
        return services;
    };

    while (startRequested()) {
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
                services.openCaptureWindow().status != OfflineStepStatus::Success) {
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
            return;
        }
        const ExecutionCycleResult result = runRealSingleCycle(
            executionRuntime, *cycleRobot, realConfig, services);
        invalidateVisionCycle(ReceiveEventInvalidationReason::CycleChanged);
        if (!result.isValid() ||
            result.status != ExecutionCycleStatus::Completed) {
            return;
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
