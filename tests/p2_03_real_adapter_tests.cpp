#include "TestHarness.h"

#include "../src/BilliardApp.h"
#include "../src/MathUtils.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double ROOT_HALF = 0.70710678118654752440;

BilliardConfig::PocketModelConfig planningPocket(
    BilliardConfig::PocketId id,
    BilliardConfig::PocketType type,
    Vector2D outward)
{
    const Vector2D side{-outward.y, outward.x};
    const double half = type == BilliardConfig::PocketType::Corner
        ? 14.142135623730951 : 20.0;
    return {id, type, outward, 30.0,
        {{-side.x * half, -side.y * half}, {side.x * half, side.y * half}},
        15.0, 2.0, 0.01, 45.0};
}

BilliardConfig::TableGeometryConfig planningTableConfig()
{
    using namespace BilliardConfig;
    return {"p2-03-table-v1", {0.0, 1000.0, 0.0, 500.0}, 10.0, 20.0, 2.0,
        {{planningPocket(PocketId::Pocket1, PocketType::Corner,
             {-ROOT_HALF, -ROOT_HALF}),
          planningPocket(PocketId::Pocket2, PocketType::Side, {0.0, -1.0}),
          planningPocket(PocketId::Pocket3, PocketType::Corner, {ROOT_HALF, -ROOT_HALF}),
          planningPocket(PocketId::Pocket4, PocketType::Corner, {-ROOT_HALF, ROOT_HALF}),
          planningPocket(PocketId::Pocket5, PocketType::Side, {0.0, 1.0}),
          planningPocket(PocketId::Pocket6, PocketType::Corner, {ROOT_HALF, ROOT_HALF})}},
        {{{RailId::Rail1, {{0.0, 0.0}, {500.0, 0.0}}, {0.0, 1.0}, 40.0, 40.0},
          {RailId::Rail2, {{500.0, 0.0}, {1000.0, 0.0}}, {0.0, 1.0}, 40.0, 40.0},
          {RailId::Rail3, {{0.0, 500.0}, {500.0, 500.0}}, {0.0, -1.0}, 40.0, 40.0},
          {RailId::Rail4, {{500.0, 500.0}, {1000.0, 500.0}}, {0.0, -1.0}, 40.0, 40.0},
          {RailId::Rail5, {{0.0, 0.0}, {0.0, 500.0}}, {1.0, 0.0}, 40.0, 40.0},
          {RailId::Rail6, {{1000.0, 0.0}, {1000.0, 500.0}}, {-1.0, 0.0}, 40.0, 40.0}}}};
}

std::string currentCycleFrame()
{
    return "500,250,-9999,-9999,-9999,-9999,-9999,-9999,"
           "-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,"
           "-9999,-9999,400,250,20,20,500,10,980,20,20,480,"
           "500,490,980,480";
}

BilliardConfig::BrainConfig planningBrainConfig()
{
    return {
        std::string{"p2-01-base0-v1"},
        BilliardConfig::KickGeometryConfig{89.0, 1e-8, 1e-8},
        BilliardConfig::ScoringConfig{
            BilliardConfig::INITIAL_EXPERIMENTAL_SCORING_WEIGHTS,
            1e-9, 90.0, 0.0, 3000.0, 200.0, 1e-9,
            BilliardConfig::PlanningMode::PotOnly}};
}

BilliardConfig::MotionPlanningConfig planningMotionConfig()
{
    BilliardConfig::MotionPlanningConfig config;
    config.calibrationRevision = "p2-03-motion-v1";
    config.base0PlanarCalibrationRevision = "p2-01-base0-v1";
    config.cueForwardAxisCalibrationRevision = "tool-axis-test-v1";
    config.strikeZMm = -216.0; config.safeApproachZMm = -160.0;
    config.readyGapMm = 15.0; config.safeLiftHeightMm = 50.0;
    config.a0Deg = 0.0; config.b0Deg = 0.0;
    config.deltaADeg = 0.0; config.deltaBDeg = 0.0;
    config.stepADeg = 1.0; config.stepBDeg = 1.0;
    config.searchOrder = BilliardConfig::PoseSearchOrder::AThenB;
    config.axisOffsetOrder = BilliardConfig::AxisOffsetOrder::LowerThenHigher;
    config.tieBreak = BilliardConfig::PoseTieBreak::FirstInApprovedSearchOrder;
    config.cToolOffsetDeg = 7.0;
    config.cueForwardAxisTool = std::array<double, 3>{0.0, 1.0, 0.0};
    config.maxCueDirectionErrorDeg = 0.01; config.directionUnitTolerance = 1e-9;
    config.executionPolicyRevision = "policy-test-v1";
    config.policyMode = BilliardConfig::ExecutionPolicyMode::PlanningTest;
    config.legalContactExecutionAuthorized = false;
    const BilliardConfig::FixedForceEnvelopeLimits direct{
        true, 0.0, 5000.0, 90.0, 180.0, std::nullopt};
    const BilliardConfig::FixedForceEnvelopeLimits kick{
        true, 0.0, 5000.0, 90.0, 180.0, 89.0};
    const BilliardConfig::FixedForceEnvelopeLimits directLegal{
        true, 0.0, 5000.0, std::nullopt, std::nullopt, std::nullopt};
    const BilliardConfig::FixedForceEnvelopeLimits kickLegal{
        true, 0.0, 5000.0, std::nullopt, std::nullopt, 89.0};
    config.fixedForceEnvelope = BilliardConfig::FixedForceEnvelopeConfig{
        "force-test-v1", direct, kick, directLegal, kickLegal};
    config.pneumaticTimingProfile =
        BilliardConfig::PneumaticTimingProfileReference{
            "pneumatic-test-v1", 100, 50, 100};
    return config;
}

ExecutionPlan validPlan()
{
    const std::array<PlannedMotionIntent, 5> intents{
        PlannedMotionIntent::JointPtpToTransit,
        PlannedMotionIntent::CartesianPtpToSafeApproach,
        PlannedMotionIntent::LinearToStrikeReady,
        PlannedMotionIntent::RuntimeActualPoseVerticalSafeLift,
        PlannedMotionIntent::JointPtpToCamera};
    return {
        {1, 1}, ShotPlanType::DirectPot, "base0-test-v1", "table-test-v1",
        "motion-test-v1", "tool-axis-test-v1", {100.0, 100.0}, {1.0, 0.0},
        25.0, 10.0, 1e-9, 0.0, {1.0, 0.0}, 0.0, 1.0,
        {65.0, 100.0, 100.0, 0.0, 0.0, 0.0},
        {65.0, 100.0, 50.0, 0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0,
        {0.0, 0.0, 0.0, 0.0, 1.0, 1.0,
         BilliardConfig::PoseSearchOrder::AThenB,
         BilliardConfig::AxisOffsetOrder::LowerThenHigher,
         BilliardConfig::PoseTieBreak::FirstInApprovedSearchOrder, 1},
        {SafeLiftDerivation::RuntimeActualPoseKeepXYABCIncreaseZ, 20.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, intents,
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
        {"pneumatic-test-v1", 100, 50, 100}, "policy-test-v1",
        BilliardConfig::ExecutionPolicyMode::RealHardware,
        ExecutionPolicyDecision::PotAccepted};
}

BilliardConfig::RealHardwareExecutionConfig validConfig()
{
    return {
        std::string{"policy-test-v1"}, true, 0, 1,
        std::string{"base0-test-v1"}, std::string{"tool1-test-v1"},
        BilliardConfig::HrSdkAngleMappingConfig{
            "abc-map-test-v1",
            {BilliardConfig::RobotAngleComponent::C,
             BilliardConfig::RobotAngleComponent::A,
             BilliardConfig::RobotAngleComponent::B},
            {1.0, -1.0, 2.0}, {10.0, 20.0, -30.0}},
        std::string{"safe-up-test-v1"},
        std::string{"tool1-test-v1"}, std::string{"abc-map-test-v1"},
        std::string{"safe-up-test-v1"}, true, 1, 2,
        BilliardConfig::PneumaticTimingProfileReference{
            "pneumatic-test-v1", 100, 50, 100}};
}

enum class DoFailurePoint { None, StrikeOn, StrikeOff, RetractOn, RetractOff };

struct FakeSdk {
    std::vector<std::string> calls;
    std::vector<unsigned long> sleepDurations;
    int tool = 0;
    int base = -1;
    std::optional<int> reportedTool;
    std::optional<int> reportedBase;
    int setToolCode = 0;
    int setBaseCode = 0;
    int motorCode = 0;
    int reachableCode = 0;
    bool reachable = true;
    int linearCheckCode = 0;
    bool linearReachable = true;
    int ptpCode = 0;
    int linCode = 0;
    int jointPtpCode = 0;
    int motionState = 1;
    bool abortCalled = false;
    int poseReadCode = 0;
    std::array<double, 6> actualPose{100.0, 200.0, 300.0, 43.0, 9.0, 14.0};
    std::array<double, 6> lastPtp{};
    std::array<double, 6> lastLin{};
    std::array<double, 6> lastLinStart{};
    bool outputs[3]{false, false, false};
    bool simultaneousOn = false;
    bool strikeWasOn = false;
    bool retractWasOn = false;
    DoFailurePoint doFailure = DoFailurePoint::None;
    int forcedOffFailureIndex = 0;
    int forcedReadFailureIndex = 0;
    bool failReadWhileOn = false;
    unsigned long ticks = 0;
    unsigned long tickStep = 1;
    int openCode = 7;
    int clearAlarmCode = 0;

    HrSdkApi api()
    {
        return {
            [&](const char*) { calls.push_back("open"); return openCode; },
            [&](int) { calls.push_back("close"); },
            [&](int) { calls.push_back("clearAlarm"); return clearAlarmCode; },
            [&](int, int) { calls.push_back("motor"); return motorCode; },
            [&](int, int) { calls.push_back("override"); return 0; },
            [&](int, int value) {
                calls.push_back("setTool");
                if (setToolCode == 0) tool = value;
                return setToolCode;
            },
            [&](int, int value) {
                calls.push_back("setBase");
                if (setBaseCode == 0) base = value;
                return setBaseCode;
            },
            [&](int) { return reportedTool.value_or(tool); },
            [&](int) { return reportedBase.value_or(base); },
            [&](int, double* pose) {
                calls.push_back("readPose");
                if (poseReadCode == 0) std::copy(actualPose.begin(), actualPose.end(), pose);
                return poseReadCode;
            },
            [&](int, double*) { return 0; },
            [&](int, double*, bool& value) {
                calls.push_back("reachable"); value = reachable; return reachableCode;
            },
            [&](int, double* start, double*, bool& value) {
                calls.push_back("checkLin");
                std::copy(start, start + 6, lastLinStart.begin());
                value = linearReachable;
                return linearCheckCode;
            },
            [&](int, int, double* pose) {
                calls.push_back("ptp");
                std::copy(pose, pose + 6, lastPtp.begin());
                return ptpCode;
            },
            [&](int, int, double, double* pose) {
                calls.push_back("lin");
                std::copy(pose, pose + 6, lastLin.begin());
                return linCode;
            },
            [&](int, int, double*) { calls.push_back("ptpAxis"); return jointPtpCode; },
            [&](int) { return motionState; },
            [&](int) { abortCalled = true; return 0; },
            [&](int, int index, bool state) {
                calls.push_back(std::string{"do"} + std::to_string(index) +
                    (state ? "On" : "Off"));
                const bool fail =
                    (!state && forcedOffFailureIndex == index) ||
                    (doFailure == DoFailurePoint::StrikeOn && index == 1 && state) ||
                    (doFailure == DoFailurePoint::StrikeOff && index == 1 && !state && strikeWasOn) ||
                    (doFailure == DoFailurePoint::RetractOn && index == 2 && state) ||
                    (doFailure == DoFailurePoint::RetractOff && index == 2 && !state && retractWasOn);
                if (fail) return -20;
                outputs[index] = state;
                if (index == 1 && state) strikeWasOn = true;
                if (index == 2 && state) retractWasOn = true;
                simultaneousOn = simultaneousOn || (outputs[1] && outputs[2]);
                return 0;
            },
            [&](int, int index) {
                calls.push_back(std::string{"readDo"} + std::to_string(index));
                if (forcedReadFailureIndex == index) return -31;
                if (failReadWhileOn && (outputs[1] || outputs[2])) return -30;
                return outputs[index] ? 1 : 0;
            },
            [&](int, int&, std::uint64_t*) { return 0; },
            [&] { ticks += tickStep; return ticks; },
            [&](unsigned long duration) {
                calls.push_back("sleep");
                sleepDurations.push_back(duration);
            }};
    }
};

std::unique_ptr<RobotController> connected(FakeSdk& fake)
{
    auto controller = std::make_unique<RobotController>(fake.api());
    controller->connect("offline-test");
    fake.calls.clear();
    fake.sleepDurations.clear();
    return controller;
}

bool called(const FakeSdk& fake, const std::string& name)
{
    return std::find(fake.calls.begin(), fake.calls.end(), name) != fake.calls.end();
}

std::size_t callCount(const FakeSdk& fake, const std::string& name)
{
    return static_cast<std::size_t>(std::count(
        fake.calls.begin(), fake.calls.end(), name));
}

bool orderedSubsequence(
    const std::vector<std::string>& calls,
    std::initializer_list<std::string> expected)
{
    auto position = calls.begin();
    for (const std::string& item : expected) {
        position = std::find(position, calls.end(), item);
        if (position == calls.end()) return false;
        ++position;
    }
    return true;
}

RealExecutionCycleServices successfulServices(
    const ExecutionPlan& plan,
    std::vector<std::string>* calls = nullptr)
{
    const auto ok = [calls](const char* name) {
        if (calls) calls->push_back(name);
        return OfflineStepResult{OfflineStepStatus::Success};
    };
    return {
        [=] { return ok("settle"); },
        [=] { return ok("flush"); },
        [=] { return ok("reset"); },
        [=] { return ok("openCapture"); },
        [calls] {
            if (calls) calls->push_back("phase1");
            return OfflinePhase1Result{OfflinePhase1Status::ShotPlanReady};
        },
        [plan, calls] {
            if (calls) calls->push_back("buildPlan");
            return ExecutionPlanResult::success(plan);
        }};
}

RealExecutionCycleServices successfulNoPlanServices()
{
    const auto ok = [] { return OfflineStepResult{OfflineStepStatus::Success}; };
    return {ok, ok, ok, ok,
        [] { return OfflinePhase1Result{OfflinePhase1Status::NoPlan}; },
        [] {
            return ExecutionPlanResult::rejected(
                ExecutionPlanStatus::NoExecutablePlan,
                ExecutionPlanFailureReason::InvalidExecutionPlanValue);
        }};
}

std::function<bool()> oneStartRequest()
{
    return [remaining = 1]() mutable {
        if (remaining == 0) return false;
        --remaining;
        return true;
    };
}

std::function<bool()> twoStartRequests()
{
    return [remaining = 2]() mutable {
        if (remaining == 0) return false;
        --remaining;
        return true;
    };
}

}  // namespace

int main()
{
    TestHarness tests;
    const ExecutionPlan plan = validPlan();
    const auto config = validConfig();

    FakeSdk activationSdk;
    auto activation = connected(activationSdk);
    tests.expectTrue(activation->activateConfiguredToolAndBase(plan, config).succeeded() &&
        activationSdk.tool == 1 && activationSdk.base == 0,
        "approved Tool1 and Base0 selection checks SDK results and readback");
    FakeSdk toolApiFailureSdk;
    toolApiFailureSdk.setToolCode = -3;
    auto toolApiFailure = connected(toolApiFailureSdk);
    tests.expectTrue(
        toolApiFailure->activateConfiguredToolAndBase(plan, config).status ==
            RobotAdapterStatus::SdkFailure && !called(toolApiFailureSdk, "setBase"),
        "Tool SDK failure stops before Base or motion commands");

    FakeSdk disabledSdk;
    auto disabled = connected(disabledSdk);
    auto disabledConfig = config;
    disabledConfig.realHardwareExecutionEnabled = false;
    tests.expectTrue(
        disabled->checkedPtp(plan, plan.strikeReadyPose, disabledConfig).status ==
            RobotAdapterStatus::Unauthorized && disabledSdk.calls.empty(),
        "real hardware authorization defaults to fail-closed behavior");
    tests.expectTrue(
        disabled->checkedPtp(plan, plan.strikeReadyPose,
            BilliardConfig::REAL_HARDWARE_EXECUTION_CONFIG).status ==
            RobotAdapterStatus::ConfigurationMissing && disabledSdk.calls.empty(),
        "production nullopt calibration sends no command");
    OfflineExecutionRuntime defaultPolicyRuntime;
    const ExecutionCycleResult defaultPolicyCycle =
        BilliardApp::runRealSingleCycle(
            defaultPolicyRuntime, *disabled,
            BilliardConfig::REAL_HARDWARE_EXECUTION_CONFIG,
            successfulServices(plan));
    tests.expectTrue(
        defaultPolicyCycle.status == ExecutionCycleStatus::SafeFailure &&
        disabledSdk.calls.empty(),
        "default production policy performs zero real hardware calls");
    auto missingBaseRevision = config;
    missingBaseRevision.base0CalibrationRevision.reset();
    tests.expectTrue(
        disabled->checkedPtp(plan, plan.strikeReadyPose, missingBaseRevision).status ==
            RobotAdapterStatus::ConfigurationMissing,
        "missing Base0 revision fails closed");
    auto missingToolRevision = config;
    missingToolRevision.tool1ControllerCalibrationRevision.reset();
    tests.expectTrue(
        disabled->checkedPtp(plan, plan.strikeReadyPose, missingToolRevision).status ==
            RobotAdapterStatus::ConfigurationMissing,
        "missing Tool1 revision fails closed");
    auto toolRevisionMismatch = config;
    toolRevisionMismatch.requiredTool1CalibrationRevision = "wrong-tool";
    tests.expectTrue(
        disabled->checkedPtp(plan, plan.strikeReadyPose,
            toolRevisionMismatch).status == RobotAdapterStatus::InvalidConfiguration,
        "Tool1 deployment/policy revision mismatch fails closed");
    tests.expectTrue(disabledSdk.calls.empty(),
        "Tool1 revision mismatch sends no hardware command");
    auto mappingRevisionMismatch = config;
    mappingRevisionMismatch.requiredAbcMappingRevision = "wrong-map";
    tests.expectTrue(
        disabled->checkedPtp(plan, plan.strikeReadyPose,
            mappingRevisionMismatch).status == RobotAdapterStatus::InvalidConfiguration,
        "ABC mapping deployment/policy revision mismatch fails closed");
    tests.expectTrue(disabledSdk.calls.empty(),
        "ABC mapping revision mismatch sends no hardware command");
    auto safeUpRevisionMismatch = config;
    safeUpRevisionMismatch.requiredSafeUpCalibrationRevision = "wrong-safe-up";
    tests.expectTrue(
        disabled->checkedPtp(plan, plan.strikeReadyPose,
            safeUpRevisionMismatch).status == RobotAdapterStatus::InvalidConfiguration,
        "safe-up deployment/policy revision mismatch fails closed");
    tests.expectTrue(disabledSdk.calls.empty(),
        "safe-up revision mismatch sends no hardware command");

    std::vector<BilliardConfig::RealHardwareExecutionConfig>
        missingOrEmptyRevisionConfigs;
    auto missingRequiredTool = config;
    missingRequiredTool.requiredTool1CalibrationRevision.reset();
    missingOrEmptyRevisionConfigs.push_back(missingRequiredTool);
    auto emptyRequiredTool = config;
    emptyRequiredTool.requiredTool1CalibrationRevision = "";
    missingOrEmptyRevisionConfigs.push_back(emptyRequiredTool);
    auto missingRequiredMapping = config;
    missingRequiredMapping.requiredAbcMappingRevision.reset();
    missingOrEmptyRevisionConfigs.push_back(missingRequiredMapping);
    auto emptyRequiredMapping = config;
    emptyRequiredMapping.requiredAbcMappingRevision = "";
    missingOrEmptyRevisionConfigs.push_back(emptyRequiredMapping);
    auto missingRequiredSafeUp = config;
    missingRequiredSafeUp.requiredSafeUpCalibrationRevision.reset();
    missingOrEmptyRevisionConfigs.push_back(missingRequiredSafeUp);
    auto emptyRequiredSafeUp = config;
    emptyRequiredSafeUp.requiredSafeUpCalibrationRevision = "";
    missingOrEmptyRevisionConfigs.push_back(emptyRequiredSafeUp);
    auto allRequiredMissing = config;
    allRequiredMissing.requiredTool1CalibrationRevision.reset();
    allRequiredMissing.requiredAbcMappingRevision.reset();
    allRequiredMissing.requiredSafeUpCalibrationRevision.reset();
    missingOrEmptyRevisionConfigs.push_back(allRequiredMissing);
    auto emptyDeploymentTool = config;
    emptyDeploymentTool.tool1ControllerCalibrationRevision = "";
    missingOrEmptyRevisionConfigs.push_back(emptyDeploymentTool);
    auto emptyDeploymentMapping = config;
    emptyDeploymentMapping.angleMapping->calibrationRevision = "";
    missingOrEmptyRevisionConfigs.push_back(emptyDeploymentMapping);
    auto emptyDeploymentSafeUp = config;
    emptyDeploymentSafeUp.safeUpCalibrationRevision = "";
    missingOrEmptyRevisionConfigs.push_back(emptyDeploymentSafeUp);

    for (const auto& invalidRevisionConfig : missingOrEmptyRevisionConfigs) {
        disabledSdk.calls.clear();
        tests.expectTrue(
            !RobotController::validateRealHardwareConfiguration(
                invalidRevisionConfig).succeeded() &&
            disabled->checkedPtp(
                plan, plan.strikeReadyPose,
                invalidRevisionConfig).status != RobotAdapterStatus::Success &&
            disabledSdk.calls.empty(),
            "each missing/empty Tool1, ABC, or safe-up revision fails with zero hardware calls");
    }

    FakeSdk mappingSdk;
    auto mappingController = connected(mappingSdk);
    const RobotPoseABC mappingPose{10.0, 20.0, 30.0, 11.0, 22.0, 33.0};
    tests.expectTrue(mappingController->checkedPtp(plan, mappingPose, config).succeeded() &&
        mappingSdk.lastPtp == std::array<double, 6>{10.0, 20.0, 30.0, 43.0, 9.0, 14.0} &&
        called(mappingSdk, "ptp") && !called(mappingSdk, "ptpAxis"),
        "approved versioned ABC mapping writes exact HRSDK pose without Tool transform");
    auto missingMapping = config;
    missingMapping.angleMapping.reset();
    mappingSdk.calls.clear();
    tests.expectTrue(
        mappingController->checkedPtp(plan, mappingPose, missingMapping).status ==
            RobotAdapterStatus::ConfigurationMissing && mappingSdk.calls.empty(),
        "missing ABC mapping blocks motion");

    FakeSdk baseMismatchSdk;
    baseMismatchSdk.reportedBase = 9;
    auto baseMismatch = connected(baseMismatchSdk);
    tests.expectFalse(baseMismatch->activateConfiguredToolAndBase(plan, config).succeeded(),
        "Base0 readback mismatch fails closed");
    FakeSdk toolMismatchSdk;
    toolMismatchSdk.reportedTool = 9;
    auto toolMismatch = connected(toolMismatchSdk);
    tests.expectFalse(toolMismatch->activateConfiguredToolAndBase(plan, config).succeeded(),
        "Tool1 readback mismatch fails closed");

    FakeSdk ptpFailureSdk;
    ptpFailureSdk.ptpCode = -7;
    auto ptpFailure = connected(ptpFailureSdk);
    tests.expectFalse(ptpFailure->checkedPtp(plan, mappingPose, config).succeeded(),
        "PTP SDK failure is not success");
    FakeSdk reachableApiFailureSdk;
    reachableApiFailureSdk.reachableCode = -42;
    auto reachableApiFailure = connected(reachableApiFailureSdk);
    tests.expectTrue(
        !reachableApiFailure->checkedPtp(plan, mappingPose, config).succeeded() &&
        !called(reachableApiFailureSdk, "ptp"),
        "motion_reachable API failure sends no Cartesian PTP");
    FakeSdk unreachablePtpSdk;
    unreachablePtpSdk.reachable = false;
    auto unreachablePtp = connected(unreachablePtpSdk);
    tests.expectTrue(
        !unreachablePtp->checkedPtp(plan, mappingPose, config).succeeded() &&
        !called(unreachablePtpSdk, "ptp"),
        "unreachable Cartesian target sends no PTP");
    FakeSdk jointSdk;
    auto joint = connected(jointSdk);
    tests.expectTrue(joint->checkedJointPtp(
        plan, plan.cameraJointReference, config).succeeded() &&
        called(jointSdk, "ptpAxis"),
        "approved CameraPose joint reference uses checked joint PTP");
    FakeSdk timeoutSdk;
    timeoutSdk.motionState = 0;
    timeoutSdk.tickStep = BilliardConfig::MOTION_TIMEOUT_MS;
    auto timeout = connected(timeoutSdk);
    tests.expectTrue(
        timeout->checkedPtp(plan, mappingPose, config).status ==
            RobotAdapterStatus::SdkFailure && timeoutSdk.abortCalled,
        "motion timeout aborts and fails closed");
    FakeSdk linCheckFailureSdk;
    linCheckFailureSdk.linearCheckCode = -8;
    auto linCheckFailure = connected(linCheckFailureSdk);
    tests.expectFalse(linCheckFailure->checkedLin(
        plan, mappingPose, {10.0, 20.0, 50.0, 11.0, 22.0, 33.0}, config).succeeded() &&
        !called(linCheckFailureSdk, "lin"),
        "motion_check_lin API failure sends no LIN");
    FakeSdk linUnreachableSdk;
    linUnreachableSdk.linearReachable = false;
    auto linUnreachable = connected(linUnreachableSdk);
    tests.expectFalse(linUnreachable->checkedLin(
        plan, mappingPose, {10.0, 20.0, 50.0, 11.0, 22.0, 33.0}, config).succeeded() &&
        !called(linUnreachableSdk, "lin"),
        "unreachable LIN sends no motion");
    FakeSdk linFailureSdk;
    linFailureSdk.linCode = -9;
    auto linFailure = connected(linFailureSdk);
    tests.expectFalse(linFailure->checkedLin(
        plan, mappingPose, {10.0, 20.0, 50.0, 11.0, 22.0, 33.0}, config).succeeded(),
        "LIN command SDK failure is not success");

    FakeSdk actualSdk;
    auto actual = connected(actualSdk);
    const auto actualResult = actual->readActualPose(plan, config);
    tests.expectTrue(actualResult.isValid() && actualResult.value &&
        actualResult.value->x == 100.0 && actualResult.value->y == 200.0 &&
        actualResult.value->z == 300.0 && actualResult.value->a == 11.0 &&
        actualResult.value->b == 22.0 && actualResult.value->c == 33.0,
        "actual HRSDK pose is returned through the approved inverse mapping");
    FakeSdk actualFailureSdk;
    actualFailureSdk.poseReadCode = -10;
    auto actualFailure = connected(actualFailureSdk);
    const auto failedActual = actualFailure->readActualPose(plan, config);
    tests.expectTrue(!failedActual.value &&
        failedActual.status == RobotAdapterStatus::SdkFailure,
        "actual-pose read failure has no planned-pose fallback");

    FakeSdk liftSdk;
    auto lift = connected(liftSdk);
    const RobotPoseABC actualPose{70.0, 80.0, 55.0, 3.0, 4.0, 5.0};
    const RobotPoseABC liftPose{70.0, 80.0, 75.0, 3.0, 4.0, 5.0};
    tests.expectTrue(lift->checkedLin(plan, actualPose, liftPose, config).succeeded() &&
        liftSdk.lastLin[0] == liftSdk.lastLinStart[0] &&
        liftSdk.lastLin[1] == liftSdk.lastLinStart[1] &&
        liftSdk.lastLin[3] == liftSdk.lastLinStart[3] &&
        liftSdk.lastLin[4] == liftSdk.lastLinStart[4] &&
        liftSdk.lastLin[5] == liftSdk.lastLinStart[5] &&
        liftSdk.lastLin[2] > liftSdk.lastLinStart[2],
        "checked vertical LIN preserves actual XYABC and only increases Z");
    liftSdk.calls.clear();
    const RobotPoseABC lateralLift{71.0, 80.0, 75.0, 3.0, 4.0, 5.0};
    tests.expectTrue(
        lift->checkVerticalSafeLift(plan, actualPose, lateralLift, config).status ==
            RobotAdapterStatus::InvalidConfiguration &&
        !called(liftSdk, "checkLin") && !called(liftSdk, "lin"),
        "post-strike safe-lift adapter rejects any XYABC change before SDK calls");
    auto unsafeUp = config;
    unsafeUp.base0PositiveZSafeConfirmed = false;
    liftSdk.calls.clear();
    tests.expectTrue(lift->checkedLin(plan, actualPose, liftPose, unsafeUp).status ==
        RobotAdapterStatus::Unauthorized && liftSdk.calls.empty(),
        "unconfirmed Base0 +Z blocks real motion");

    FakeSdk pneumaticSdk;
    auto pneumatic = connected(pneumaticSdk);
    const auto pneumaticSuccess = pneumatic->executePneumaticSequence(plan, config);
    tests.expectTrue(pneumaticSuccess.isValid() &&
        pneumaticSuccess.status == RealPneumaticStatus::Completed &&
        !pneumaticSdk.simultaneousOn && !pneumaticSdk.outputs[1] &&
        !pneumaticSdk.outputs[2] &&
        pneumaticSdk.sleepDurations ==
            std::vector<unsigned long>{100, 50, 100, 100},
        "dual DO sequence is mutually exclusive and ends physically OFF");
    FakeSdk movingSdk;
    movingSdk.motionState = 0;
    auto moving = connected(movingSdk);
    const auto movingResult = moving->executePneumaticSequence(plan, config);
    tests.expectTrue(
        movingResult.status == RealPneumaticStatus::KnownSafeFailure &&
        !called(movingSdk, "do1On") && !called(movingSdk, "do2On"),
        "pneumatic sequence requires confirmed Robot stopped before either ON");

    const std::array<DoFailurePoint, 4> failurePoints{
        DoFailurePoint::StrikeOn, DoFailurePoint::StrikeOff,
        DoFailurePoint::RetractOn, DoFailurePoint::RetractOff};
    for (const DoFailurePoint failurePoint : failurePoints) {
        FakeSdk failureSdk;
        failureSdk.doFailure = failurePoint;
        auto controller = connected(failureSdk);
        const RealPneumaticResult result =
            controller->executePneumaticSequence(plan, config);
        const bool offFailure = failurePoint == DoFailurePoint::StrikeOff ||
            failurePoint == DoFailurePoint::RetractOff;
        tests.expectTrue(
            result.status == (offFailure
                ? RealPneumaticStatus::UnknownUnsafe
                : RealPneumaticStatus::KnownSafeFailure),
            "DO failures map to known-safe or UnknownUnsafe without continuation");
    }
    FakeSdk communicationLossSdk;
    communicationLossSdk.failReadWhileOn = true;
    auto communicationLoss = connected(communicationLossSdk);
    const RealPneumaticResult communicationLossResult =
        communicationLoss->executePneumaticSequence(plan, config);
    tests.expectTrue(
        communicationLossResult.status == RealPneumaticStatus::UnknownUnsafe,
        "communication loss during DO becomes UnknownUnsafe");
    const std::size_t callsBeforeBlockedMotion = communicationLossSdk.calls.size();
    tests.expectTrue(
        BilliardApp::mapRealPneumaticResult(communicationLossResult).status ==
            PneumaticCompletionStatus::UnknownUnsafe &&
        communicationLoss->checkedPtp(plan, mappingPose, config).status ==
            RobotAdapterStatus::UnknownUnsafe &&
        communicationLossSdk.calls.size() == callsBeforeBlockedMotion,
        "real UnknownUnsafe maps to the P2-02 terminal result with no motion");

    auto missingTiming = config;
    missingTiming.approvedTimingProfile.reset();
    FakeSdk timingSdk;
    auto timing = connected(timingSdk);
    timingSdk.calls.clear();
    tests.expectTrue(
        timing->executePneumaticSequence(plan, missingTiming).status ==
            RealPneumaticStatus::KnownSafeFailure && timingSdk.calls.empty(),
        "missing approved pneumatic timing sends no DO command");

    ExecutionPlan legal = plan;
    legal.sourceShotType = ShotPlanType::DirectLegalContact;
    legal.policyDecision = ExecutionPolicyDecision::LegalContactExplicitlyAuthorized;
    FakeSdk legalSdk;
    auto legalController = connected(legalSdk);
    legalSdk.calls.clear();
    tests.expectTrue(
        legalController->executePneumaticSequence(legal, config).status ==
            RealPneumaticStatus::KnownSafeFailure && legalSdk.calls.empty(),
        "LegalContact real hardware remains blocked by default");

    FakeSdk cycleSdk;
    auto cycleRobot = connected(cycleSdk);
    OfflineExecutionRuntime cycleRuntime;
    const ExecutionCycleResult cycle = BilliardApp::runRealSingleCycle(
        cycleRuntime, *cycleRobot, config,
        successfulServices(plan, &cycleSdk.calls));
    tests.expectTrue(
        cycle.isValid() && cycle.status == ExecutionCycleStatus::Completed &&
        cycle.value && cycle.value->shotExecuted &&
        callCount(cycleSdk, "ptpAxis") == 3 &&
        callCount(cycleSdk, "ptp") == 1 &&
        callCount(cycleSdk, "lin") == 2 &&
        callCount(cycleSdk, "readPose") == 2 &&
        !cycleSdk.simultaneousOn &&
        orderedSubsequence(cycleSdk.calls,
            {"do1Off", "readDo1", "do2Off", "readDo2", "clearAlarm",
             "setTool", "setBase", "motor", "override", "ptpAxis",
             "settle", "flush", "reset", "openCapture", "phase1",
             "buildPlan", "ptpAxis", "ptp", "readPose", "checkLin", "lin",
             "do1On", "do1Off", "do2On", "do2Off", "readPose",
             "checkLin", "lin", "ptpAxis"}),
        "CameraPose and current-cycle capture precede the current plan and real execution");

    const auto planningConfig = planningMotionConfig();
    FakeSdk planningSdk;
    RobotController planningRobot(planningSdk.api());
    BilliardAppRunTestSeam planningRun{
        BilliardConfig::ExecutionPolicyMode::PlanningTest,
        std::string{"policy-test-v1"}, false, &planningRobot, std::nullopt,
        oneStartRequest(), std::nullopt};
    planningRun.motionPlanningPolicyMode =
        BilliardConfig::ExecutionPolicyMode::PlanningTest;
    planningRun.motionPlanningConfig = planningConfig;
    planningRun.observationBounds = AxisAlignedBounds2D{0.0, 1000.0, 0.0, 500.0};
    planningRun.stabilityConfig = StabilityConfig{
        1.0, 1.0, std::chrono::milliseconds{1000}};
    planningRun.tableGeometryConfig = planningTableConfig();
    planningRun.brainConfig = planningBrainConfig();
    planningRun.connectionIdentity = 1;
    planningRun.currentCycleFrames = {
        currentCycleFrame(), currentCycleFrame(), currentCycleFrame()};
    std::optional<ExecutionPlan> observedPlanningPlan;
    planningRun.executionPlanObserved = [&](const ExecutionPlanResult& result) {
        if (result.isValid() &&
            result.status() == ExecutionPlanStatus::Success &&
            result.value()) {
            observedPlanningPlan = *result.value();
        }
    };
    BilliardApp planningApp(std::move(planningRun));
    std::ostringstream planningOutput;
    std::streambuf* originalOutput = std::cout.rdbuf(planningOutput.rdbuf());
    planningApp.run();
    std::cout.rdbuf(originalOutput);
    const std::string planningText = planningOutput.str();
    std::ostringstream expectedPose;
    if (observedPlanningPlan) {
        const RobotPoseABC& pose = observedPlanningPlan->strikeReadyPose;
        expectedPose << "X=" << pose.x << "\nY=" << pose.y
                     << "\nZ=" << pose.z << "\nA=" << pose.a
                     << "\nB=" << pose.b << "\nC=" << pose.c;
    }
    tests.expectTrue(
        observedPlanningPlan.has_value() &&
        planningText.find("=== Planning Result ===") != std::string::npos &&
        planningText.find("ReceiveEvent id=1 accepted") != std::string::npos &&
        planningText.find("ReceiveEvent id=2 accepted") != std::string::npos &&
        planningText.find("Phase 1 ShotPlan ready") != std::string::npos &&
        planningText.find("status=0") != std::string::npos &&
        planningText.find(expectedPose.str()) != std::string::npos &&
        planningText.find("NO HARDWARE EXECUTION") != std::string::npos &&
        planningText.find("MissingValidationSeam") == std::string::npos &&
        planningSdk.calls.empty(),
        "PlanningTest parses three current-cycle frames, reaches Phase1 and the production P2-01 builder, and prints its exact StrikeReadyPose without hardware");

    FakeSdk missingChecksSdk;
    RobotController missingChecksRobot(missingChecksSdk.api());
    BilliardAppRunTestSeam missingChecksRun{
        BilliardConfig::ExecutionPolicyMode::PlanningTest,
        std::string{"policy-test-v1"}, false, &missingChecksRobot, std::nullopt,
        oneStartRequest(), std::nullopt};
    missingChecksRun.motionPlanningPolicyMode =
        BilliardConfig::ExecutionPolicyMode::PlanningTest;
    missingChecksRun.motionPlanningConfig = planningConfig;
    missingChecksRun.motionPlanningChecks = MotionPlanningChecks{};
    missingChecksRun.observationBounds =
        AxisAlignedBounds2D{0.0, 1000.0, 0.0, 500.0};
    missingChecksRun.stabilityConfig = StabilityConfig{
        1.0, 1.0, std::chrono::milliseconds{1000}};
    missingChecksRun.tableGeometryConfig = planningTableConfig();
    missingChecksRun.brainConfig = planningBrainConfig();
    missingChecksRun.connectionIdentity = 1;
    missingChecksRun.currentCycleFrames = {
        currentCycleFrame(), currentCycleFrame(), currentCycleFrame()};
    BilliardApp missingChecksApp(std::move(missingChecksRun));
    std::ostringstream missingChecksOutput;
    originalOutput = std::cout.rdbuf(missingChecksOutput.rdbuf());
    missingChecksApp.run();
    std::cout.rdbuf(originalOutput);
    tests.expectTrue(
        missingChecksOutput.str().find(
            "status=" + std::to_string(static_cast<int>(
                ExecutionPlanStatus::ConfigurationMissing))) !=
            std::string::npos &&
        missingChecksOutput.str().find("StrikeReadyPose:") ==
            std::string::npos &&
        missingChecksSdk.calls.empty(),
        "missing offline P2-01 checks is named and has no fallback pose or hardware");

    BilliardAppRunTestSeam noPlanRun{
        BilliardConfig::ExecutionPolicyMode::PlanningTest,
        std::string{"policy-test-v1"}, false, nullptr, std::nullopt,
        oneStartRequest(), successfulNoPlanServices()};
    noPlanRun.motionPlanningPolicyMode =
        BilliardConfig::ExecutionPolicyMode::PlanningTest;
    BilliardApp noPlanApp(std::move(noPlanRun));
    std::ostringstream noPlanOutput;
    originalOutput = std::cout.rdbuf(noPlanOutput.rdbuf());
    noPlanApp.run();
    std::cout.rdbuf(originalOutput);
    tests.expectTrue(
        noPlanOutput.str().find("final status=NoPlan") != std::string::npos &&
        noPlanOutput.str().find("StrikeReadyPose:") == std::string::npos,
        "PlanningTest NoPlan prints no fallback Cartesian pose");

    RealExecutionCycleServices rejectedPlanServices =
        successfulServices(plan);
    rejectedPlanServices.buildExecutionPlan = [] {
        return ExecutionPlanResult::rejected(
            ExecutionPlanStatus::InvalidExecutionPlan,
            ExecutionPlanFailureReason::InvalidExecutionPlanValue);
    };
    BilliardAppRunTestSeam rejectedPlanRun{
        BilliardConfig::ExecutionPolicyMode::PlanningTest,
        std::string{"policy-test-v1"}, false, nullptr, std::nullopt,
        oneStartRequest(), rejectedPlanServices};
    rejectedPlanRun.motionPlanningPolicyMode =
        BilliardConfig::ExecutionPolicyMode::PlanningTest;
    BilliardApp rejectedPlanApp(std::move(rejectedPlanRun));
    std::ostringstream rejectedPlanOutput;
    originalOutput = std::cout.rdbuf(rejectedPlanOutput.rdbuf());
    rejectedPlanApp.run();
    std::cout.rdbuf(originalOutput);
    tests.expectTrue(
        rejectedPlanOutput.str().find("NO HARDWARE EXECUTION") != std::string::npos &&
        rejectedPlanOutput.str().find("StrikeReadyPose:") == std::string::npos,
        "PlanningTest ExecutionPlan failure prints no fallback Cartesian pose");

    FakeSdk productionRunSdk;
    auto productionRunRobot = connected(productionRunSdk);
    RealExecutionCycleServices productionServices =
        successfulServices(plan, &productionRunSdk.calls);
    BilliardAppRunTestSeam productionRun{
        BilliardConfig::ExecutionPolicyMode::RealHardware,
        std::string{"policy-test-v1"}, false, productionRunRobot.get(), config,
        oneStartRequest(), productionServices};
    productionRun.motionPlanningPolicyMode =
        BilliardConfig::ExecutionPolicyMode::RealHardware;
    BilliardApp productionApp(std::move(productionRun));
    productionApp.run();
    tests.expectTrue(
        orderedSubsequence(productionRunSdk.calls,
            {"do1Off", "readDo1", "do2Off", "readDo2", "clearAlarm",
             "ptpAxis", "settle", "flush", "reset", "openCapture",
             "phase1", "buildPlan"}) &&
        callCount(productionRunSdk, "do1On") == 1 &&
        callCount(productionRunSdk, "ptpAxis") == 3,
        "BilliardApp::run captures after CameraPose and executes exactly one current plan");

    FakeSdk stalePlanSdk;
    auto stalePlanRobot = connected(stalePlanSdk);
    ExecutionPlan stalePlan = plan;
    stalePlan.sourcePlanIdentity.shotCycleIdentity = 99;
    OfflineExecutionRuntime stalePlanRuntime;
    const ExecutionCycleResult stalePlanResult = BilliardApp::runRealSingleCycle(
        stalePlanRuntime, *stalePlanRobot, config,
        successfulServices(stalePlan, &stalePlanSdk.calls));
    tests.expectTrue(
        stalePlanResult.status == ExecutionCycleStatus::SafeFailure &&
        orderedSubsequence(stalePlanSdk.calls,
            {"ptpAxis", "settle", "flush", "reset", "openCapture",
             "phase1", "buildPlan"}) &&
        callCount(stalePlanSdk, "ptpAxis") == 1 &&
        !called(stalePlanSdk, "ptp") && !called(stalePlanSdk, "lin") &&
        !called(stalePlanSdk, "do1On"),
        "stale or pre-camera plan identity cannot reach strike execution");

    FakeSdk twoCycleSdk;
    auto twoCycleRobot = connected(twoCycleSdk);
    auto nextPlanIdentity = std::make_shared<std::size_t>(1);
    RealExecutionCycleServices twoCycleServices = successfulServices(plan);
    twoCycleServices.buildExecutionPlan = [plan, nextPlanIdentity] {
        ExecutionPlan current = plan;
        current.sourcePlanIdentity.shotCycleIdentity = *nextPlanIdentity;
        ++*nextPlanIdentity;
        return ExecutionPlanResult::success(std::move(current));
    };
    BilliardAppRunTestSeam twoCycleRun{
        BilliardConfig::ExecutionPolicyMode::RealHardware,
        std::string{"policy-test-v1"}, false, twoCycleRobot.get(), config,
        twoStartRequests(), twoCycleServices};
    twoCycleRun.motionPlanningPolicyMode =
        BilliardConfig::ExecutionPolicyMode::RealHardware;
    BilliardApp twoCycleApp(std::move(twoCycleRun));
    twoCycleApp.run();
    tests.expectTrue(
        callCount(twoCycleSdk, "do1On") == 2 &&
        callCount(twoCycleSdk, "ptpAxis") == 6 &&
        *nextPlanIdentity == 3,
        "two explicit Starts execute exactly two identity-matched cycles without drift");

    FakeSdk missingPolicySdk;
    auto missingPolicyRobot = connected(missingPolicySdk);
    BilliardAppRunTestSeam missingPolicyRun{
        BilliardConfig::ExecutionPolicyMode::RealHardware, std::nullopt,
        false, missingPolicyRobot.get(), config, oneStartRequest(),
        successfulServices(plan)};
    missingPolicyRun.motionPlanningPolicyMode =
        BilliardConfig::ExecutionPolicyMode::RealHardware;
    BilliardApp missingPolicyApp(std::move(missingPolicyRun));
    missingPolicyApp.run();
    tests.expectTrue(missingPolicySdk.calls.empty(),
        "missing ExecutionPolicy revision blocks DO and motion in BilliardApp::run");

    FakeSdk missingPolicyModeSdk;
    RobotController missingPolicyModeRobot(missingPolicyModeSdk.api());
    BilliardAppRunTestSeam missingPolicyModeRun{
        BilliardConfig::ExecutionPolicyMode::RealHardware,
        std::string{"policy-test-v1"}, false, &missingPolicyModeRobot, config,
        oneStartRequest(), successfulServices(plan)};
    BilliardApp missingPolicyModeApp(std::move(missingPolicyModeRun));
    missingPolicyModeApp.run();
    tests.expectTrue(missingPolicyModeSdk.calls.empty(),
        "missing MotionPlanningConfig policyMode fails closed before every hardware call");

    FakeSdk policyMismatchSdk;
    RobotController policyMismatchRobot(policyMismatchSdk.api());
    BilliardAppRunTestSeam policyMismatchRun{
        BilliardConfig::ExecutionPolicyMode::RealHardware,
        std::string{"policy-test-v1"}, false, &policyMismatchRobot, config,
        oneStartRequest(), successfulServices(plan)};
    policyMismatchRun.motionPlanningPolicyMode =
        BilliardConfig::ExecutionPolicyMode::PlanningTest;
    BilliardApp policyMismatchApp(std::move(policyMismatchRun));
    policyMismatchApp.run();
    tests.expectTrue(policyMismatchSdk.calls.empty(),
        "RealHardware runtime with PlanningTest P2-01 policy performs zero HRSDK calls");

    FakeSdk invalidRunConfigSdk;
    auto invalidRunConfigRobot = connected(invalidRunConfigSdk);
    BilliardAppRunTestSeam invalidRunConfig{
        BilliardConfig::ExecutionPolicyMode::RealHardware,
        std::string{"policy-test-v1"}, false, invalidRunConfigRobot.get(),
        mappingRevisionMismatch, oneStartRequest(), successfulServices(plan)};
    invalidRunConfig.motionPlanningPolicyMode =
        BilliardConfig::ExecutionPolicyMode::RealHardware;
    BilliardApp invalidRunConfigApp(std::move(invalidRunConfig));
    invalidRunConfigApp.run();
    tests.expectTrue(invalidRunConfigSdk.calls.empty(),
        "invalid deployment revision blocks all hardware in BilliardApp::run");

    FakeSdk preconnectGateSdk;
    RobotController preconnectGateRobot(preconnectGateSdk.api());
    OfflineExecutionRuntime preconnectGateRuntime;
    const ExecutionCycleResult preconnectGateResult =
        BilliardApp::runRealSingleCycle(
            preconnectGateRuntime, preconnectGateRobot,
            mappingRevisionMismatch, successfulServices(plan));
    tests.expectTrue(
        preconnectGateResult.status == ExecutionCycleStatus::SafeFailure &&
        preconnectGateSdk.calls.empty(),
        "static RealHardware calibration validation occurs before HRSDK connect");

    FakeSdk connectFailureSdk;
    connectFailureSdk.openCode = -1;
    RobotController connectFailureRobot(connectFailureSdk.api());
    OfflineExecutionRuntime connectFailureRuntime;
    const ExecutionCycleResult connectFailureResult =
        BilliardApp::runRealSingleCycle(
            connectFailureRuntime, connectFailureRobot, config,
            successfulServices(plan));
    tests.expectTrue(
        connectFailureResult.status == ExecutionCycleStatus::SafeFailure &&
        connectFailureResult.diagnostic &&
        connectFailureResult.diagnostic->reason ==
            ExecutionCycleFailureReason::HardwareConnectionFailed &&
        !called(connectFailureSdk, "do1Off") &&
        !called(connectFailureSdk, "ptpAxis"),
        "HRSDK connect failure is named and sends no DO or motion command");

    FakeSdk clearAlarmFailureSdk;
    clearAlarmFailureSdk.clearAlarmCode = -43;
    auto clearAlarmFailureRobot = connected(clearAlarmFailureSdk);
    OfflineExecutionRuntime clearAlarmFailureRuntime;
    const ExecutionCycleResult clearAlarmFailureResult =
        BilliardApp::runRealSingleCycle(
            clearAlarmFailureRuntime, *clearAlarmFailureRobot, config,
            successfulServices(plan));
    tests.expectTrue(
        clearAlarmFailureResult.status == ExecutionCycleStatus::SafeFailure &&
        orderedSubsequence(clearAlarmFailureSdk.calls,
            {"do1Off", "readDo1", "do2Off", "readDo2", "clearAlarm"}) &&
        !called(clearAlarmFailureSdk, "ptpAxis") &&
        !called(clearAlarmFailureSdk, "ptp") &&
        !called(clearAlarmFailureSdk, "lin"),
        "clear-alarm SDK failure occurs only after both DOs are safe and sends no motion");

    FakeSdk runStartupFailureSdk;
    runStartupFailureSdk.forcedReadFailureIndex = 1;
    auto runStartupFailureRobot = connected(runStartupFailureSdk);
    BilliardAppRunTestSeam runStartupFailure{
        BilliardConfig::ExecutionPolicyMode::RealHardware,
        std::string{"policy-test-v1"}, false, runStartupFailureRobot.get(),
        config, oneStartRequest(), successfulServices(plan)};
    runStartupFailure.motionPlanningPolicyMode =
        BilliardConfig::ExecutionPolicyMode::RealHardware;
    BilliardApp runStartupFailureApp(std::move(runStartupFailure));
    runStartupFailureApp.run();
    tests.expectTrue(
        callCount(runStartupFailureSdk, "ptpAxis") == 0 &&
        callCount(runStartupFailureSdk, "ptp") == 0 &&
        callCount(runStartupFailureSdk, "lin") == 0,
        "startup DO uncertainty through BilliardApp::run permits zero motion");

    FakeSdk startupDo1FailureSdk;
    startupDo1FailureSdk.forcedOffFailureIndex = 1;
    auto startupDo1Failure = connected(startupDo1FailureSdk);
    OfflineExecutionRuntime startupDo1Runtime;
    const ExecutionCycleResult startupDo1Result = BilliardApp::runRealSingleCycle(
        startupDo1Runtime, *startupDo1Failure, config, successfulServices(plan));
    tests.expectTrue(
        startupDo1Result.status == ExecutionCycleStatus::UnknownUnsafe &&
        !called(startupDo1FailureSdk, "do2Off") &&
        callCount(startupDo1FailureSdk, "ptpAxis") == 0 &&
        callCount(startupDo1FailureSdk, "ptp") == 0 &&
        callCount(startupDo1FailureSdk, "lin") == 0,
        "DO1 OFF failure stops normal DO2 initialization and all Robot motion");

    FakeSdk startupDo2UnknownSdk;
    startupDo2UnknownSdk.forcedReadFailureIndex = 2;
    auto startupDo2Unknown = connected(startupDo2UnknownSdk);
    OfflineExecutionRuntime startupDo2Runtime;
    const ExecutionCycleResult startupDo2Result = BilliardApp::runRealSingleCycle(
        startupDo2Runtime, *startupDo2Unknown, config, successfulServices(plan));
    tests.expectTrue(
        startupDo2Result.status == ExecutionCycleStatus::UnknownUnsafe &&
        callCount(startupDo2UnknownSdk, "ptpAxis") == 0 &&
        callCount(startupDo2UnknownSdk, "ptp") == 0 &&
        callCount(startupDo2UnknownSdk, "lin") == 0,
        "DO2 readback uncertainty is terminal before the first motion");

    FakeSdk invalidStartupSdk;
    auto invalidStartup = connected(invalidStartupSdk);
    invalidStartupSdk.calls.clear();
    OfflineExecutionRuntime invalidStartupRuntime;
    const ExecutionCycleResult invalidStartupResult = BilliardApp::runRealSingleCycle(
        invalidStartupRuntime, *invalidStartup, mappingRevisionMismatch,
        successfulServices(plan));
    tests.expectTrue(
        invalidStartupResult.status == ExecutionCycleStatus::SafeFailure &&
        invalidStartupSdk.calls.empty(),
        "invalid revision config performs zero DO and zero motion calls");

    FakeSdk unsafeCycleSdk;
    unsafeCycleSdk.failReadWhileOn = true;
    auto unsafeCycleRobot = connected(unsafeCycleSdk);
    OfflineExecutionRuntime unsafeCycleRuntime;
    const ExecutionCycleResult unsafeCycle = BilliardApp::runRealSingleCycle(
        unsafeCycleRuntime, *unsafeCycleRobot, config, successfulServices(plan));
    const std::size_t unsafeCalls = unsafeCycleSdk.calls.size();
    const ExecutionCycleResult unsafeRestart = BilliardApp::runRealSingleCycle(
        unsafeCycleRuntime, *unsafeCycleRobot, config, successfulServices(plan));
    tests.expectTrue(
        unsafeCycle.status == ExecutionCycleStatus::UnknownUnsafe &&
        unsafeCycleRuntime.state == ExecutionCycleState::UnknownUnsafe &&
        callCount(unsafeCycleSdk, "readPose") == 1 &&
        callCount(unsafeCycleSdk, "lin") == 1 &&
        callCount(unsafeCycleSdk, "ptpAxis") == 2 &&
        unsafeRestart.status == ExecutionCycleStatus::StartRejected &&
        unsafeCycleSdk.calls.size() == unsafeCalls,
        "UnknownUnsafe from real pneumatic adapter prevents actual-pose, lift, and camera-return commands");

    FakeSdk stoppedFailureSdk;
    stoppedFailureSdk.motionState = -12;
    auto stoppedFailure = connected(stoppedFailureSdk);
    tests.expectTrue(
        stoppedFailure->confirmStopped().status == RobotAdapterStatus::SdkFailure,
        "unknown Robot motion state fails closed");

    FakeSdk disconnectFailureSdk;
    disconnectFailureSdk.motorCode = -41;
    auto disconnectFailure = connected(disconnectFailureSdk);
    tests.expectTrue(
        disconnectFailure->disconnect().status == RobotAdapterStatus::UnknownUnsafe,
        "motor-OFF SDK failure during disconnect remains observable as UnknownUnsafe");

    return tests.exitCode();
}
