#include "TestHarness.h"

#include "../src/Algorithm.h"
#include "../src/BilliardApp.h"
#include "../src/BilliardPhysics.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <optional>
#include <string>
#include <vector>

namespace {

// MotionPlanner.cpp的normalizeAngleDeg是檔案內部helper未匯出，測試端另建
// 一份等價實作（fmod-based wrap至(-180,180]）僅用於算出Pull模式的期望C角。
double normalizeAngleDeg(double angle) noexcept
{
    double normalized = std::fmod(angle + 180.0, 360.0);
    if (normalized < 0.0) normalized += 360.0;
    return normalized - 180.0;
}

// ============================================================
// 一顆「真的有效」的ShotPlan：PlanningResult::isValid()／ShotPlan::isValid()
// 對幾何一致性要求極深（與ExecutionPlan同等級），手刻placeholder不切實際。
// 沿用p2_01已驗證過的Algorithm pipeline現生一顆，內容本身對candidate-search
// 測試無意義（buildExecutionPlanForShot完全接管回傳值），只需要「合法」。
// ============================================================

BilliardConfig::TableGeometryConfig tableConfig()
{
    using namespace BilliardConfig;
    return {
        "p2-03-table-v1", {0.0, 1000.0, 0.0, 500.0}, 10.0, 20.0, 2.0,
        {{
            {RailId::Rail1, PocketId::Pocket1, PocketId::Pocket2, 40.0, 40.0, 0.0},
            {RailId::Rail2, PocketId::Pocket2, PocketId::Pocket3, 40.0, 40.0, 0.0},
            {RailId::Rail3, PocketId::Pocket3, PocketId::Pocket4, 40.0, 40.0, 0.0},
            {RailId::Rail4, PocketId::Pocket4, PocketId::Pocket5, 40.0, 40.0, 0.0},
            {RailId::Rail5, PocketId::Pocket5, PocketId::Pocket6, 40.0, 40.0, 0.0},
            {RailId::Rail6, PocketId::Pocket6, PocketId::Pocket1, 40.0, 40.0, 0.0}
        }}};
}

std::array<Point, 6> pocketCenters()
{
    return {{{0.0, 0.0}, {500.0, 0.0}, {1000.0, 0.0},
             {1000.0, 500.0}, {500.0, 500.0}, {0.0, 500.0}}};
}

StableTableState state(Point cueBall, std::array<std::optional<Point>, 9> balls)
{
    const auto now = std::chrono::steady_clock::now();
    return {
        std::move(balls), cueBall, pocketCenters(), 11, 22,
        {{{101, now}, {102, now}, {103, now}}}};
}

BilliardConfig::KickGeometryConfig kickConfig() { return {89.0, 1e-8, 1e-8}; }

BilliardConfig::ScoringConfig scoringConfig(
    BilliardConfig::PlanningMode mode = BilliardConfig::PlanningMode::PotOnly)
{
    return {
        BilliardConfig::INITIAL_EXPERIMENTAL_SCORING_WEIGHTS,
        1e-9, 90.0, 0.0, 3000.0, 200.0, 1e-9, mode};
}

BilliardConfig::BrainConfig brainConfig(
    BilliardConfig::PlanningMode mode = BilliardConfig::PlanningMode::PotOnly)
{
    return {
        std::optional<std::string>{"p2-01-base0-v1"},
        std::optional<BilliardConfig::KickGeometryConfig>{kickConfig()},
        std::optional<BilliardConfig::ScoringConfig>{scoringConfig(mode)}};
}

StableTableState kickStateForRebound(
    const ResolvedTableGeometry& geometry, std::size_t railIndex, std::size_t pocketIndex)
{
    const auto& rail = geometry.railReflectionRegion.rails[railIndex];
    const Point rebound{
        (rail.segment.start.x + rail.segment.end.x) / 2.0,
        (rail.segment.start.y + rail.segment.end.y) / 2.0};
    const auto& pocket = geometry.pockets[pocketIndex];
    const auto& bounds = tableConfig().physicalPlayingSurface;
    const Point tableCenter{
        (bounds.minX + bounds.maxX) / 2.0, (bounds.minY + bounds.maxY) / 2.0};
    const Vector2D inwardRaw{
        tableCenter.x - pocket.center.x, tableCenter.y - pocket.center.y};
    const double inwardLength = std::hypot(inwardRaw.x, inwardRaw.y);
    const Vector2D inward{inwardRaw.x / inwardLength, inwardRaw.y / inwardLength};
    const Point target{
        pocket.center.x + 100.0 * inward.x, pocket.center.y + 100.0 * inward.y};
    const Point ghost{
        target.x + 2.0 * geometry.ballRadiusMm * inward.x,
        target.y + 2.0 * geometry.ballRadiusMm * inward.y};
    const Vector2D outgoingRaw{ghost.x - rebound.x, ghost.y - rebound.y};
    const double outgoingLength = std::hypot(outgoingRaw.x, outgoingRaw.y);
    const Vector2D outgoing{outgoingRaw.x / outgoingLength, outgoingRaw.y / outgoingLength};
    const double projection =
        outgoing.x * rail.inwardUnitNormal.x + outgoing.y * rail.inwardUnitNormal.y;
    const Vector2D incoming{
        outgoing.x - 2.0 * projection * rail.inwardUnitNormal.x,
        outgoing.y - 2.0 * projection * rail.inwardUnitNormal.y};
    std::array<std::optional<Point>, 9> balls{};
    balls[0] = target;
    return state(
        {rebound.x - 120.0 * incoming.x, rebound.y - 120.0 * incoming.y}, balls);
}

ShotPlan buildRealShotPlan()
{
    const auto resolved = BilliardPhysics::resolveTableGeometry(pocketCenters(), tableConfig());
    const ResolvedTableGeometry geometry = *resolved.value();
    const StableTableState fixture = kickStateForRebound(geometry, 0, 4);
    PlanningResult result = BilliardAlgorithm::planShot(
        fixture, std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        brainConfig());
    return std::get<ShotPlan>(result.value());
}

// Phase1ExecutionCandidates::isValid()要求legalContactPlans內每筆type必須是
// DirectLegalContact／KickLegalContact，不能沿用DirectPot的real plan佔位。
ShotPlan buildRealLegalContactPlan()
{
    const auto resolved = BilliardPhysics::resolveTableGeometry(pocketCenters(), tableConfig());
    const ResolvedTableGeometry geometry = *resolved.value();
    std::array<std::optional<Point>, 9> balls{};
    balls[0] = Point{500.0, 250.0};
    for (std::size_t pocket = 0; pocket < geometry.pockets.size(); ++pocket) {
        const Point target = geometry.pockets[pocket].center;
        balls[pocket + 1] = Point{
            balls[0]->x + 0.75 * (target.x - balls[0]->x),
            balls[0]->y + 0.75 * (target.y - balls[0]->y)};
    }
    PlanningResult result = BilliardAlgorithm::planShot(
        state({400.0, 250.0}, balls),
        std::optional<BilliardConfig::TableGeometryConfig>{tableConfig()},
        brainConfig(BilliardConfig::PlanningMode::ManualResearch));
    return std::get<ShotPlan>(result.value());
}

// ============================================================
// Section 0: 共用fixture（與p2_02同款named-field手法，避免positional
// init在大型aggregate上的排序風險）
// ============================================================

// ExecutionPlan::isValid()對strikeReadyPose/safeApproachPose/strikeMode有
// 嚴格的跨欄位一致性要求（座標由cueBallCenterBase0Mm/shotDirectionXY/bias反推——
// Tool1 TCP已核准校正在氣壓推桿行程中點，不再扣ballRadius/readyGap，
// C角由directionToCDeg()決定，strikeMode由
// resolveStrikeMode()依bottomDistance/pullModeMinBottomDistanceMm/
// tableDownDirectionDot決定），因此改用production同一份directionToCDegForTest
// 現算C角，而非手推三角函數，避免與production公式漂移。
ExecutionPlan validExecutionPlan(
    std::uint64_t cycleIdentity = 1,
    StrikeMode mode = StrikeMode::Push)
{
    ExecutionPlan plan{};
    plan.sourcePlanIdentity = {1, cycleIdentity};
    plan.sourceShotType = ShotPlanType::DirectPot;
    plan.base0PlanarCalibrationRevision = "base0-test-v1";
    plan.tableGeometryRevision = "table-test-v1";
    plan.motionCalibrationRevision = "motion-test-v1";
    plan.cueForwardAxisCalibrationRevision = "tool-axis-test-v1";
    plan.strikeMode = mode;
    plan.physicalPlayingSurfaceBase0Mm = {0.0, 1000.0, 0.0, 500.0};
    plan.tableDownDirectionBase0XY = {0.0, -1.0};
    plan.pullModeMinBottomDistanceMm = 300.0;
    if (mode == StrikeMode::Pull) {
        // dot(shotDirection, tableDown)>0且bottomDistance>閾值才會解析成Pull。
        plan.cueBallCenterBase0Mm = {100.0, 400.0};
        plan.shotDirectionXY = {0.0, -1.0};
    } else {
        plan.cueBallCenterBase0Mm = {100.0, 100.0};
        plan.shotDirectionXY = {1.0, 0.0};
    }
    // 非貼庫覆寫：executionDirectionPolicy=Normal時
    // plannedShotDirectionXY必須跟shotDirectionXY逐位元一致（見
    // ExecutionPlan::isValid()的validDirectionPolicy不變量）。
    plan.plannedShotDirectionXY = plan.shotDirectionXY;
    plan.executionDirectionPolicy = ExecutionDirectionPolicy::Normal;
    plan.bottomDistanceMm =
        plan.cueBallCenterBase0Mm.y - plan.physicalPlayingSurfaceBase0Mm.minY;
    plan.tableDownDirectionDot =
        plan.shotDirectionXY.x * plan.tableDownDirectionBase0XY.x +
        plan.shotDirectionXY.y * plan.tableDownDirectionBase0XY.y;
    plan.strikePositionBiasMm = 0.0;
    plan.ballRadiusMm = 25.0;
    plan.readyGapMm = 10.0;
    plan.directionUnitTolerance = 1e-9;
    plan.cToolOffsetDeg = 0.0;
    plan.validatedStrikeDirectionXY = plan.shotDirectionXY;
    plan.cueDirectionErrorDeg = 0.0;
    plan.maxCueDirectionErrorDeg = 1.0;
    // Tool1的TCP已核准校正在氣壓推桿行程中點，XY直接對齊母球中心
    // （見production MotionPlanner.cpp同一處註解），不再扣ballRadius/readyGap。
    const double strikeX = plan.cueBallCenterBase0Mm.x;
    const double strikeY = plan.cueBallCenterBase0Mm.y;
    const double pushC = *MotionPlanner::directionToCDegForTest(
        plan.shotDirectionXY, plan.cToolOffsetDeg, plan.directionUnitTolerance);
    const double strikeC = mode == StrikeMode::Pull
        ? normalizeAngleDeg(pushC + 180.0)
        : pushC;
    plan.safeApproachPose = {strikeX, strikeY, 100.0, 0.0, 0.0, strikeC};
    plan.strikeReadyPose = {strikeX, strikeY, 50.0, 0.0, 0.0, strikeC};
    plan.selectedADeg = 0.0;
    plan.selectedBDeg = 0.0;
    plan.selectedCDeg = strikeC;
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
    plan.policyMode = BilliardConfig::ExecutionPolicyMode::RealHardware;
    plan.policyDecision = ExecutionPolicyDecision::PotAccepted;
    plan.tool1Number = BilliardConfig::TOOL_NUMBER;
    plan.tool1ControllerCalibrationRevision = "tool1-test-v1";
    plan.rankedPotCandidatesExhausted = false;
    return plan;
}

BilliardConfig::RealHardwareExecutionConfig validConfig()
{
    BilliardConfig::RealHardwareExecutionConfig config{};
    config.authorizationRevision = "policy-test-v1";
    config.realHardwareExecutionEnabled = true;
    config.baseNumber = 0;
    config.tool1Number = BilliardConfig::TOOL_NUMBER;
    config.base0CalibrationRevision = "base0-test-v1";
    config.tool1ControllerCalibrationRevision = "tool1-test-v1";
    config.angleMapping = BilliardConfig::HrSdkAngleMappingConfig{
        "abc-map-test-v1",
        {BilliardConfig::RobotAngleComponent::C,
         BilliardConfig::RobotAngleComponent::A,
         BilliardConfig::RobotAngleComponent::B},
        {1.0, -1.0, 2.0}, {10.0, 20.0, -30.0}};
    config.safeUpCalibrationRevision = "safe-up-test-v1";
    config.requiredTool1CalibrationRevision = "tool1-test-v1";
    config.requiredAbcMappingRevision = "abc-map-test-v1";
    config.requiredSafeUpCalibrationRevision = "safe-up-test-v1";
    config.base0PositiveZSafeConfirmed = true;
    config.extendDoIndex = 1;
    config.retractDoIndex = 2;
    config.approvedTimingProfile = BilliardConfig::PneumaticTimingProfileReference{
        "pneumatic-test-v1", 100, 50, 100};
    return config;
}

enum class DoFailurePoint { None, StrikeOn, StrikeOff, RetractOn, RetractOff };

// HRSDK呼叫紀錄與行為注入的假SDK；純硬體API層級，與Tool1/Tool2架構無關，
// 因此本次migration照搬既有pattern。
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
    // clearAlarm()新的決定性流程：預設「馬達已斷電」讓poll立刻通過；
    // getAlarmCodes預設回報「有一筆待清alarm」讓既有測試斷言的
    // "clearAlarm"呼叫仍會發生，clearAlarm()執行過一次後才轉報0筆，
    // 讓新增的「清除後re-query必須歸零」通過。
    int motorState = 0;
    int getAlarmCodesSdkCode = 0;
    bool alarmClearedOnce = false;
    int reachableCode = 0;
    bool reachable = true;
    std::vector<bool> reachableResponses;
    std::size_t reachableResponseIndex = 0;
    int linearCheckCode = 0;
    bool linearReachable = true;
    int ptpCode = 0;
    int linCode = 0;
    int jointPtpCode = 0;
    int motionState = 1;
    // waitForMotion()是兩階段poll：先等狀態離開1（確認motion真的啟動），
    // 再等狀態回到1（確認完成）。真實SDK在下達motion命令後狀態會短暫
    // 離開1；固定回傳1會讓第一階段永遠等不到離開、卡到
    // MOTION_START_CONFIRMATION_TIMEOUT_MS逾時。以此旗標模擬那一次性的
    // 「剛下命令」瞬間離開1，之後恢復穩態motionState。
    bool motionJustCommanded = false;
    bool abortCalled = false;
    int poseReadCode = 0;
    // z=55預設低於validExecutionPlan()的safeApproachPose.z(100)，滿足
    // isValidSafeLiftTarget()「安全高度必須高於目前實際高度」的不變式；
    // 需要特定z值的個別測試（如ABC mapping精確性）另外覆寫。
    std::array<double, 6> actualPose{100.0, 200.0, 55.0, 43.0, 9.0, 14.0};
    // isAtConfiguredJoint()／confirmSafeAtCameraPose()仰賴getCurrentJoints
    // 讀回實際關節角；預設回報「目前在CameraPose」，讓Planning前置的
    // read-only安全確認在happy path上通過。
    std::array<double, 6> currentJoints = BilliardConfig::CAMERA_JOINT;
    int getJointsCode = 0;
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
    int digitalInputValue = 0;
    // 供「deadline在執行中途過期不得中止已開始的執行」測試使用：模擬
    // pulse/wait期間deadline真的走過期限。
    std::function<void()> onSleepCallback;

    HrSdkApi api()
    {
        return {
            [&](const char*) { calls.push_back("open"); return openCode; },
            [&](int) { calls.push_back("close"); },
            [&](int) {
                calls.push_back("clearAlarm");
                alarmClearedOnce = true;
                return clearAlarmCode;
            },
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
            [&](int, double* joints) {
                calls.push_back("readJoints");
                if (getJointsCode == 0) {
                    std::copy(currentJoints.begin(), currentJoints.end(), joints);
                }
                return getJointsCode;
            },
            [&](int, double*, bool& value) {
                calls.push_back("reachable");
                value = reachableResponseIndex < reachableResponses.size()
                    ? reachableResponses[reachableResponseIndex++]
                    : reachable;
                return reachableCode;
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
                if (ptpCode == 0) motionJustCommanded = true;
                return ptpCode;
            },
            [&](int, int, double, double* pose) {
                calls.push_back("lin");
                std::copy(pose, pose + 6, lastLin.begin());
                if (linCode == 0) motionJustCommanded = true;
                return linCode;
            },
            [&](int, int, double* joints) {
                calls.push_back("ptpAxis");
                if (jointPtpCode == 0) {
                    motionJustCommanded = true;
                    // 模擬真的到位：readJoints讀的currentJoints要跟著更新，
                    // 否則同一輪cycle內先後對Standby/CameraPose的位置確認
                    // （isAtConfiguredJoint）不可能同時成立，靜態值只能滿足
                    // 其中一個目標關節角度。
                    std::copy(joints, joints + 6, currentJoints.begin());
                }
                return jointPtpCode;
            },
            [&](int) {
                if (motionJustCommanded) {
                    motionJustCommanded = false;
                    return motionState == 1 ? 0 : motionState;
                }
                return motionState;
            },
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
            [&](int, int index) {
                calls.push_back(std::string{"readDi"} + std::to_string(index));
                return digitalInputValue;
            },
            [&](int, int& count, std::uint64_t* alarms) {
                calls.push_back("getAlarmCodes");
                count = alarmClearedOnce ? 0 : 1;
                if (count > 0) alarms[0] = 0x1;
                return getAlarmCodesSdkCode;
            },
            [&] { ticks += tickStep; return ticks; },
            [&](unsigned long duration) {
                calls.push_back("sleep");
                sleepDurations.push_back(duration);
                if (onSleepCallback) onSleepCallback();
            },
            [&](int) { calls.push_back("getMotorState"); return motorState; }};
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

std::size_t lastIndexOf(const std::vector<std::string>& calls, const std::string& name)
{
    for (std::size_t i = calls.size(); i-- > 0;) {
        if (calls[i] == name) return i;
    }
    return static_cast<std::size_t>(-1);
}

bool noneCalledAfter(
    const std::vector<std::string>& calls,
    std::size_t afterIndex,
    std::initializer_list<std::string> forbidden)
{
    if (afterIndex == static_cast<std::size_t>(-1)) return false;
    for (std::size_t i = afterIndex + 1; i < calls.size(); ++i) {
        for (const std::string& name : forbidden) {
            if (calls[i] == name) return false;
        }
    }
    return true;
}

struct FakeClock {
    std::chrono::steady_clock::time_point base = std::chrono::steady_clock::now();
    std::chrono::milliseconds offset{0};
    [[nodiscard]] std::chrono::steady_clock::time_point now() const { return base + offset; }
};

ShotDeadlineClock deadlineFor(FakeClock& clock)
{
    return ShotDeadlineClock{[&clock] { return clock.now(); }, clock.base};
}

Phase1ExecutionCandidates candidatesWith(std::size_t rankedPotCount, std::size_t legalCount = 0)
{
    const ShotPlan real = buildRealShotPlan();
    return {
        std::vector<ShotPlan>(rankedPotCount, real),
        std::vector<ShotPlan>(legalCount, real)};
}

// acquireExecutionPlan仰賴的RealExecutionCycleServices仍可完全注入；ShotPlan
// 本身內容對candidate-search測試無意義（buildExecutionPlanForShot被完全接管），
// 但PlanningResult::isValid()仍要求其為一顆結構合法的ShotPlan，因此用
// buildRealShotPlan()產生的真實plan佔位，而非手刻的空白值。
struct FakeRealServices {
    std::vector<std::string> calls;
    bool visionConnected = true;
    std::vector<VisionConnectResult> connectResponses;
    std::size_t connectCallIndex = 0;
    OfflinePhase1Status phase1Status = OfflinePhase1Status::ShotPlanReady;
    std::optional<OfflinePhase1FailureKind> phase1FailureKind;
    PlanningResult planningResult = PlanningResult::shotPlan(buildRealShotPlan(), candidatesWith(1));
    std::function<ExecutionPlanResult(const ShotPlan&, bool)> buildPlan;
    std::size_t phase1CallCount = 0;
    std::optional<ShotCycleIdentity> openedWindowFor;
    FakeClock* clock = nullptr;
    std::chrono::milliseconds advancePerPhase1{0};

    RealExecutionCycleServices services()
    {
        RealExecutionCycleServices s;
        s.settleCamera = [this] {
            calls.push_back("settle");
            return OfflineStepResult{OfflineStepStatus::Success};
        };
        s.flushStaleVisionBuffer = [this] {
            calls.push_back("flush");
            return OfflineStepResult{OfflineStepStatus::Success};
        };
        s.resetCycleAccumulation = [this] {
            calls.push_back("reset");
            return OfflineStepResult{OfflineStepStatus::Success};
        };
        s.openCaptureWindow = [this](ShotCycleIdentity id) {
            calls.push_back("openCapture");
            openedWindowFor = id;
            return OfflineStepResult{OfflineStepStatus::Success};
        };
        s.runPhase1 = [this]() -> OfflinePhase1Result {
            calls.push_back("phase1");
            ++phase1CallCount;
            if (clock) clock->offset += advancePerPhase1;
            return {phase1Status, phase1FailureKind};
        };
        s.isVisionConnected = [this] { return visionConnected; };
        s.connectVision = [this]() -> VisionConnectResult {
            calls.push_back("connectVision");
            if (connectCallIndex < connectResponses.size()) {
                const VisionConnectResult r = connectResponses[connectCallIndex++];
                if (r.status == VisionConnectStatus::Connected) visionConnected = true;
                return r;
            }
            visionConnected = true;
            return {VisionConnectStatus::Connected, 0};
        };
        s.currentPlanningResult = [this]() -> const PlanningResult* {
            return &planningResult;
        };
        s.buildExecutionPlanForShot = [this](
            const ShotPlan& shot,
            bool potsExhausted,
            std::optional<StrikeMode>) {
            calls.push_back("buildPlan");
            return buildPlan
                ? buildPlan(shot, potsExhausted)
                : ExecutionPlanResult::rejected(
                    ExecutionPlanStatus::NoExecutablePlan,
                    ExecutionPlanFailureReason::InvalidExecutionPlanValue);
        };
        s.sleepMs = [this](unsigned long) { calls.push_back("sleep"); };
        return s;
    }
};

bool called(const FakeRealServices& fake, const std::string& name)
{
    return std::find(fake.calls.begin(), fake.calls.end(), name) != fake.calls.end();
}

std::size_t callCount(const FakeRealServices& fake, const std::string& name)
{
    return static_cast<std::size_t>(std::count(
        fake.calls.begin(), fake.calls.end(), name));
}

// 單一candidate即成功的最小成功情境：ranked pot只有一筆，preflight
// (=checkReachable兩次)皆通過。
FakeRealServices singleCandidateSuccess(const ExecutionPlan& plan)
{
    FakeRealServices fake;
    fake.planningResult = PlanningResult::shotPlan(buildRealShotPlan(), candidatesWith(1));
    fake.buildPlan = [plan](const ShotPlan&, bool) {
        return ExecutionPlanResult::success(plan);
    };
    return fake;
}

}  // namespace

int main()
{
    TestHarness tests;
    const ExecutionPlan plan = validExecutionPlan();
    const auto config = validConfig();

    // ============================================================
    // Section A: RobotController硬體adapter層級（migration自既有測試，
    // 與Tool1/Tool2架構無關的部分照搬，欄位改名/簽章處已同步修正）
    // ============================================================

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
        disabled->checkedPtp(plan, plan.strikeReadyPose, std::nullopt).status ==
            RobotAdapterStatus::ConfigurationMissing && disabledSdk.calls.empty(),
        "nullopt calibration config sends no command");

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
        disabled->checkedPtp(plan, plan.strikeReadyPose, toolRevisionMismatch).status ==
            RobotAdapterStatus::InvalidConfiguration && disabledSdk.calls.empty(),
        "Tool1 deployment/policy revision mismatch fails closed with zero hardware calls");
    auto mappingRevisionMismatch = config;
    mappingRevisionMismatch.requiredAbcMappingRevision = "wrong-map";
    tests.expectTrue(
        disabled->checkedPtp(plan, plan.strikeReadyPose, mappingRevisionMismatch).status ==
            RobotAdapterStatus::InvalidConfiguration && disabledSdk.calls.empty(),
        "ABC mapping deployment/policy revision mismatch fails closed with zero hardware calls");
    auto safeUpRevisionMismatch = config;
    safeUpRevisionMismatch.requiredSafeUpCalibrationRevision = "wrong-safe-up";
    tests.expectTrue(
        disabled->checkedPtp(plan, plan.strikeReadyPose, safeUpRevisionMismatch).status ==
            RobotAdapterStatus::InvalidConfiguration && disabledSdk.calls.empty(),
        "safe-up deployment/policy revision mismatch fails closed with zero hardware calls");

    std::vector<BilliardConfig::RealHardwareExecutionConfig> missingOrEmptyRevisionConfigs;
    auto missingRequiredTool = config;
    missingRequiredTool.requiredTool1CalibrationRevision.reset();
    missingOrEmptyRevisionConfigs.push_back(missingRequiredTool);
    auto emptyRequiredTool = config;
    emptyRequiredTool.requiredTool1CalibrationRevision = "";
    missingOrEmptyRevisionConfigs.push_back(emptyRequiredTool);
    auto missingRequiredMapping = config;
    missingRequiredMapping.requiredAbcMappingRevision.reset();
    missingOrEmptyRevisionConfigs.push_back(missingRequiredMapping);
    auto missingRequiredSafeUp = config;
    missingRequiredSafeUp.requiredSafeUpCalibrationRevision.reset();
    missingOrEmptyRevisionConfigs.push_back(missingRequiredSafeUp);
    auto emptyDeploymentTool = config;
    emptyDeploymentTool.tool1ControllerCalibrationRevision = "";
    missingOrEmptyRevisionConfigs.push_back(emptyDeploymentTool);
    for (const auto& invalidRevisionConfig : missingOrEmptyRevisionConfigs) {
        disabledSdk.calls.clear();
        tests.expectTrue(
            !RobotController::validateRealHardwareConfiguration(invalidRevisionConfig)
                .succeeded() &&
            disabled->checkedPtp(plan, plan.strikeReadyPose, invalidRevisionConfig).status !=
                RobotAdapterStatus::Success &&
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
    tests.expectTrue(joint->checkedConfiguredJointPtp(
        BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg, config).succeeded() &&
        called(jointSdk, "ptpAxis"),
        "approved standby joint reference uses checked configured joint PTP");
    FakeSdk timeoutSdk;
    timeoutSdk.motionState = 0;
    timeoutSdk.tickStep = BilliardConfig::MOTION_TIMEOUT_MS;
    auto timeout = connected(timeoutSdk);
    tests.expectTrue(
        timeout->checkedPtp(plan, mappingPose, config).status ==
            RobotAdapterStatus::UnknownUnsafe && timeoutSdk.abortCalled,
        "motion timeout aborts and latches UnknownUnsafe (unconfirmed final position)");
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
    actualSdk.actualPose = {100.0, 200.0, 300.0, 43.0, 9.0, 14.0};
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
    const RobotPoseABC liftActualPose{70.0, 80.0, 55.0, 3.0, 4.0, 5.0};
    const RobotPoseABC liftTargetPose = buildSafeLiftTarget(liftActualPose, plan.safeApproachPose.z);
    // 序列（check LIN -> move LIN -> confirm stopped -> return standby）由
    // runOfflineSingleCycle單一控制流保證，runRealSingleCycle直接沿用同一段
    // 程式碼（見p2_02對應的order assertion），此處只需驗證checkedLin本身的
    // XYZABC不變式，不必在這個較低層級的adapter測試裡重複整條序列。
    tests.expectTrue(lift->checkedLin(plan, liftActualPose, liftTargetPose, config).succeeded() &&
        liftSdk.lastLin[0] == liftSdk.lastLinStart[0] &&
        liftSdk.lastLin[1] == liftSdk.lastLinStart[1] &&
        liftSdk.lastLin[2] > liftSdk.lastLinStart[2] &&
        liftTargetPose.x == liftActualPose.x && liftTargetPose.y == liftActualPose.y &&
        liftTargetPose.a == liftActualPose.a && liftTargetPose.b == liftActualPose.b &&
        liftTargetPose.c == liftActualPose.c &&
        liftTargetPose.z == plan.safeApproachPose.z,
        "safe-lift target preserves actual X/Y/A/B/C exactly and sets Z to exactly "
        "plan.safeApproachPose.z; the checked vertical LIN uses the actual pose as its "
        "start point and only increases Z");
    liftSdk.calls.clear();
    const RobotPoseABC lateralLift{71.0, 80.0, 100.0, 3.0, 4.0, 5.0};
    tests.expectTrue(
        lift->checkVerticalSafeLift(plan, liftActualPose, lateralLift, config).status ==
            RobotAdapterStatus::InvalidConfiguration &&
        !called(liftSdk, "checkLin") && !called(liftSdk, "lin"),
        "post-strike safe-lift adapter rejects any XYABC change before SDK calls");
    auto unsafeUp = config;
    unsafeUp.base0PositiveZSafeConfirmed = false;
    liftSdk.calls.clear();
    tests.expectTrue(lift->checkedLin(plan, liftActualPose, liftTargetPose, unsafeUp).status ==
        RobotAdapterStatus::Unauthorized && liftSdk.calls.empty(),
        "unconfirmed Base0 +Z blocks real motion");

    FakeSdk pneumaticSdk;
    auto pneumatic = connected(pneumaticSdk);
    const auto pneumaticSuccess = pneumatic->executePneumaticSequence(plan, config);
    tests.expectTrue(pneumaticSuccess.isValid() &&
        pneumaticSuccess.status == RealPneumaticStatus::Completed &&
        pneumaticSuccess.evidence == RealPneumaticEvidence::OutputOffConfirmed &&
        BilliardApp::mapRealPneumaticResult(pneumaticSuccess).evidence ==
            PneumaticCompletionEvidence::OffCommandAccepted &&
        !pneumaticSdk.simultaneousOn && !pneumaticSdk.outputs[1] && !pneumaticSdk.outputs[2],
        "dual DO sequence is mutually exclusive and reports electrical output OFF");
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
        const RealPneumaticResult result = controller->executePneumaticSequence(plan, config);
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
        "real UnknownUnsafe maps to the terminal result with no further motion");

    auto missingTiming = config;
    missingTiming.approvedTimingProfile.reset();
    FakeSdk timingSdk;
    auto timing = connected(timingSdk);
    timingSdk.calls.clear();
    tests.expectTrue(
        timing->executePneumaticSequence(plan, missingTiming).status ==
            RealPneumaticStatus::KnownSafeFailure && timingSdk.calls.empty(),
        "missing approved pneumatic timing sends no DO command");

    FakeSdk stoppedFailureSdk;
    stoppedFailureSdk.motionState = -12;
    auto stoppedFailure = connected(stoppedFailureSdk);
    tests.expectTrue(
        stoppedFailure->confirmStopped().status == RobotAdapterStatus::UnknownUnsafe,
        "unknown (negative) Robot motion state latches UnknownUnsafe, not a plain SdkFailure");
    FakeSdk disconnectFailureSdk;
    disconnectFailureSdk.motorCode = -41;
    auto disconnectFailure = connected(disconnectFailureSdk);
    tests.expectTrue(
        disconnectFailure->disconnect().status == RobotAdapterStatus::UnknownUnsafe,
        "motor-OFF SDK failure during disconnect remains observable as UnknownUnsafe");

    // ============================================================
    // Section B: H/P鍵盤edge gate（純函式，section 10 keyboard需求
    // 不需透過run()即可直接驗證）
    // ============================================================
    {
        StartControlGates gates;
        std::vector<ConsoleKeyEvent> queued{{ConsoleKey::H, true}};
        const ConsoleKeyPoll poll = [&] { auto v = queued; queued.clear(); return v; };
        const StartControlEvent first = BilliardApp::pollStartControl(gates, poll);
        tests.expectTrue(first.startEdge && !first.standbyEdge,
            "H key-down produces exactly one start edge");
        const StartControlEvent second = BilliardApp::pollStartControl(gates, poll);
        tests.expectFalse(second.startEdge,
            "held H key (no intervening up event) does not re-trigger a new edge");
        queued = {{ConsoleKey::H, false}, {ConsoleKey::H, true}};
        const StartControlEvent third = BilliardApp::pollStartControl(gates, poll);
        tests.expectTrue(third.startEdge,
            "release followed by a new down event produces a fresh edge");
    }
    {
        StartControlGates gates;
        std::vector<ConsoleKeyEvent> queued{{ConsoleKey::P, true}};
        const ConsoleKeyPoll poll = [&] { auto v = queued; queued.clear(); return v; };
        const StartControlEvent event = BilliardApp::pollStartControl(gates, poll);
        tests.expectTrue(event.standbyEdge && !event.startEdge,
            "P key-down produces exactly one standby edge, independent gate from H");
    }
    {
        StartControlGates gates;
        std::vector<ConsoleKeyEvent> queued{{ConsoleKey::H, true}, {ConsoleKey::P, true}};
        const ConsoleKeyPoll poll = [&] { auto v = queued; queued.clear(); return v; };
        const ConsoleKeyDownQuery query = [](ConsoleKey) { return false; };
        BilliardApp::resyncStartControlToIdle(gates, poll, query);
        tests.expectTrue(!gates.start.isDown() && !gates.standby.isDown(),
            "resyncStartControlToIdle drains queued events and resyncs to physical idle state "
            "without producing edges");
    }

    // ============================================================
    // Section C: runRealSingleCycle（Phase2新增的核心安全流程：連線、
    // vision reconnect、retry cutoff、candidate search、Pull序列）
    // ============================================================

    {
        OfflineExecutionRuntime runtime;
        runtime.state = ExecutionCycleState::CameraPose;
        RobotController robot;
        FakeClock clock;
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, robot, config, FakeRealServices{}.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::StartRejected &&
            result.diagnostic->reason == ExecutionCycleFailureReason::CycleAlreadyActive,
            "active cycle rejects concurrent real start");
    }
    {
        OfflineExecutionRuntime runtime;
        RobotController robot;
        FakeClock clock;
        RealExecutionCycleServices missingServices = singleCandidateSuccess(plan).services();
        missingServices.isVisionConnected = nullptr;
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, robot, config, missingServices, deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            result.diagnostic->reason == ExecutionCycleFailureReason::InvalidExecutionPlan,
            "missing required service function fails closed with InvalidExecutionPlan");
    }
    {
        OfflineExecutionRuntime runtime;
        RobotController robot;
        FakeClock clock;
        auto invalidConfig = config;
        invalidConfig.requiredTool1CalibrationRevision = "wrong-tool";
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, robot, invalidConfig, singleCandidateSuccess(plan).services(),
            deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure,
            "invalid calibration configuration fails before any HRSDK connect attempt");
    }
    {
        FakeSdk connectFailureSdk;
        connectFailureSdk.openCode = -1;
        RobotController connectFailureRobot(connectFailureSdk.api());
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, connectFailureRobot, config, singleCandidateSuccess(plan).services(),
            deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            result.diagnostic->reason == ExecutionCycleFailureReason::PreparationCheckFailed &&
            !called(connectFailureSdk, "do1Off") && !called(connectFailureSdk, "ptpAxis"),
            "HRSDK connect failure is classified as PreparationCheckFailed with zero DO/motion");
    }
    {
        FakeSdk do1FailureSdk;
        do1FailureSdk.forcedOffFailureIndex = 1;
        auto do1FailureRobot = connected(do1FailureSdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *do1FailureRobot, config, singleCandidateSuccess(plan).services(),
            deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::UnknownUnsafe &&
            !called(do1FailureSdk, "do2Off") &&
            callCount(do1FailureSdk, "ptpAxis") == 0,
            "DO1 OFF failure during startup readiness check stops before DO2 and all motion");
    }
    {
        FakeSdk clearAlarmFailureSdk;
        clearAlarmFailureSdk.clearAlarmCode = -43;
        auto clearAlarmFailureRobot = connected(clearAlarmFailureSdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *clearAlarmFailureRobot, config, singleCandidateSuccess(plan).services(),
            deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            orderedSubsequence(clearAlarmFailureSdk.calls,
                {"readDo1", "readDo2", "clearAlarm"}) &&
            callCount(clearAlarmFailureSdk, "ptpAxis") == 0,
            "clear-alarm SDK failure occurs only after both DOs confirmed safe, sends no motion");
    }
    {
        FakeSdk cycleSdk;
        auto cycleRobot = connected(cycleSdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fakeServices = singleCandidateSuccess(validExecutionPlan(5));
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 5, *cycleRobot, config, fakeServices.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed &&
            result.value && result.value->shotExecuted &&
            fakeServices.openedWindowFor == 5 &&
            callCount(cycleSdk, "do1On") == 1 &&
            orderedSubsequence(fakeServices.calls,
                {"settle", "flush", "reset", "openCapture", "phase1", "buildPlan"}) &&
            orderedSubsequence(cycleSdk.calls,
                {"clearAlarm", "setTool", "setBase", "motor", "override", "ptpAxis",
                 "reachable", "reachable", "ptp", "readPose", "checkLin", "lin",
                 "do1On", "do1Off", "do2On", "do2Off", "readPose", "checkLin", "lin",
                 "ptpAxis"}),
            "happy path: connected vision, one reachable candidate completes with matching cycleIdentity");
    }
    {
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(plan);
        fake.visionConnected = false;
        fake.connectResponses = {
            {VisionConnectStatus::Retriable, 1},
            {VisionConnectStatus::Connected, 0}};
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed &&
            callCount(fake, "connectVision") == 2,
            "vision reconnect: Retriable then Connected proceeds to a normal successful cycle");
    }
    {
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(plan);
        fake.visionConnected = false;
        fake.connectResponses = {{VisionConnectStatus::NonRetriable, 99}};
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            result.diagnostic->reason == ExecutionCycleFailureReason::CaptureAndPlanFailed &&
            callCount(fake, "connectVision") == 1 && !called(fake, "phase1"),
            "vision connect NonRetriable fails immediately without retrying or reaching Phase1");
    }
    {
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(plan);
        fake.visionConnected = false;
        fake.connectResponses = {{VisionConnectStatus::Retriable, 1}};
        clock.offset = std::chrono::milliseconds(
            BilliardConfig::SHOT_CYCLE_TIMING.planningRetryCutoffMs + 1000);
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed &&
            result.value && !result.value->shotExecuted,
            "vision never connects and retry cutoff already elapsed: safe no-plan end, "
            "not an infinite retry loop");
    }
    {
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(plan);
        fake.phase1Status = OfflinePhase1Status::PipelineFailure;
        fake.phase1FailureKind = OfflinePhase1FailureKind::NonRetriable;
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            result.diagnostic->reason == ExecutionCycleFailureReason::CaptureAndPlanFailed &&
            fake.phase1CallCount == 1,
            "Phase1 non-retriable pipeline failure fails immediately without retry");
    }
    {
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(plan);
        // NoPlan::isValid()對NoEligibleTarget要求source存在且diagnostic.targetStatus
        // 精確吻合，不像InvalidBrainConfiguration有全nullopt的寬鬆分支。
        NoPlan noEligible{};
        noEligible.reason = NoPlanReason::NoEligibleTarget;
        const auto sourceNow = std::chrono::steady_clock::now();
        noEligible.source = PlanningSourceAudit{
            {1, 1},
            {{{101, sourceNow}, {102, sourceNow}, {103, sourceNow}}},
            "base0-test-v1", "table-test-v1", {100.0, 100.0}, 25.0};
        noEligible.diagnostic.targetStatus = TargetQualificationStatus::NoEligibleTarget;
        fake.planningResult = PlanningResult::noPlan(noEligible);
        fake.phase1Status = OfflinePhase1Status::NoPlan;
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed &&
            result.value && !result.value->shotExecuted,
            "NoEligibleTarget is a safe, immediate no-plan end (not a hard failure)");
    }
    {
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(plan);
        fake.planningResult = PlanningResult::noPlan(
            NoPlan{NoPlanReason::InvalidBrainConfiguration, std::nullopt, std::nullopt});
        fake.phase1Status = OfflinePhase1Status::NoPlan;
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            result.diagnostic->reason == ExecutionCycleFailureReason::CaptureAndPlanFailed,
            "InvalidBrainConfiguration NoPlan reason is a hard failure, not a safe end");
    }
    {
        // Section 10: NoPotCandidate／NoLegalContact可重拍——Phase1第一次
        // 回NoPlan/NoPotCandidate必須觸發重新收集（runPhase1再跑一次），
        // 不是立即安全結束（像NoEligibleTarget）也不是hard failure
        // （像InvalidBrainConfiguration）。
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake;
        NoPlan noPotCandidate{};
        noPotCandidate.reason = NoPlanReason::NoPotCandidate;
        const auto sourceNow = std::chrono::steady_clock::now();
        noPotCandidate.source = PlanningSourceAudit{
            {1, 1}, {{{101, sourceNow}, {102, sourceNow}, {103, sourceNow}}},
            "base0-test-v1", "table-test-v1", {100.0, 100.0}, 25.0};
        noPotCandidate.selectedTarget = EligibleTarget{1, {200.0, 200.0}};
        noPotCandidate.diagnostic.selectionStatus = PotSelectionStatus::Success;
        fake.planningResult = PlanningResult::noPlan(noPotCandidate);
        bool firstPhase1Call = true;
        RealExecutionCycleServices services = fake.services();
        services.runPhase1 = [&]() -> OfflinePhase1Result {
            fake.calls.push_back("phase1");
            ++fake.phase1CallCount;
            if (firstPhase1Call) {
                firstPhase1Call = false;
                return {OfflinePhase1Status::NoPlan};
            }
            fake.planningResult = PlanningResult::shotPlan(
                buildRealShotPlan(), candidatesWith(1));
            return {OfflinePhase1Status::ShotPlanReady};
        };
        services.currentPlanningResult = [&]() -> const PlanningResult* {
            return &fake.planningResult;
        };
        services.buildExecutionPlanForShot = [&fake, plan](
            const ShotPlan&, bool, std::optional<StrikeMode>) {
            fake.calls.push_back("buildPlan");
            return ExecutionPlanResult::success(plan);
        };
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, services, deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed && result.value &&
                result.value->shotExecuted && fake.phase1CallCount == 2,
            "NoPotCandidate triggers a recollect-and-retry (runPhase1 runs a "
            "second time), not an immediate safe end or hard failure");
    }
    {
        FakeSdk sdk;
        sdk.reachableResponses = {false, true, true};
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake;
        // 兩個候選內容相同(candidatesWith只能重複同一個real plan)，會讓
        // 「重試同一候選」跟「換下一候選」無法區分；改用vector元素的
        // 位址身分區分究竟被嘗試的是候選0還是候選1，直接斷言嘗試順序。
        Phase1ExecutionCandidates candidates = candidatesWith(2);
        const ShotPlan* candidate0Ptr = &candidates.rankedPotPlans[0];
        const ShotPlan* candidate1Ptr = &candidates.rankedPotPlans[1];
        fake.planningResult =
            PlanningResult::shotPlan(buildRealShotPlan(), std::move(candidates));
        // strikeMode=Pull（不是預設Push）：preflight NotReachable後的
        // Push->Pull對側重試只在原始strikeMode為Push時觸發（見
        // tryCandidates），Pull本身沒有反向fallback，這裡才單純測到
        // 「NotReachable換下一個candidate、不是重試同一個candidate」，
        // 不會跟新加的對側重試機制混在一起。
        const ExecutionPlan pullPlan = validExecutionPlan(1, StrikeMode::Pull);
        std::vector<int> attemptedCandidateIndices;
        fake.buildPlan = [pullPlan, candidate0Ptr, candidate1Ptr, &attemptedCandidateIndices](
                const ShotPlan& shot, bool) {
            attemptedCandidateIndices.push_back(
                &shot == candidate0Ptr ? 0 : (&shot == candidate1Ptr ? 1 : -1));
            return ExecutionPlanResult::success(pullPlan);
        };
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed &&
            // preflightExecution消耗3次(候選0 approach不可達1次+候選1
            // approach/ready各1次)，moveToStrikeReady自己的checkedPtp再
            // 消耗1次，共4次。
            callCount(sdk, "reachable") == 4 &&
            attemptedCandidateIndices == std::vector<int>{0, 1},
            "first ranked-pot candidate (index 0) NotReachable is attempted exactly "
            "once, then falls through to the distinct next candidate (index 1) — "
            "not a retry of the same candidate");
    }
    {
        // 用2個候選（而不是1個）才能真的證明「SDK/config/numerical failure
        // 不得換候選」——只有1個候選時，不管有沒有停止都不會嘗試第2個，
        // 無法區別「正確地不換」跟「根本沒有第2個可換」。
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake;
        fake.planningResult = PlanningResult::shotPlan(buildRealShotPlan(), candidatesWith(2));
        std::size_t hardFailureBuiltCount = 0;
        fake.buildPlan = [&hardFailureBuiltCount](const ShotPlan&, bool) {
            ++hardFailureBuiltCount;
            return ExecutionPlanResult::rejected(
                ExecutionPlanStatus::InvalidExecutionPlan,
                ExecutionPlanFailureReason::InvalidExecutionPlanValue);
        };
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            result.diagnostic->reason == ExecutionCycleFailureReason::CaptureAndPlanFailed &&
            hardFailureBuiltCount == 1,
            "candidate ExecutionPlan build hard failure (SDK/config/numerical, not a "
            "pose/force rejection) fails the cycle immediately without trying the "
            "second available candidate");
    }
    {
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(plan);
        fake.buildPlan = [](const ShotPlan&, bool) {
            return ExecutionPlanResult::rejected(
                ExecutionPlanStatus::NoExecutablePlan,
                ExecutionPlanFailureReason::FixedForceEnvelopeRejected);
        };
        clock.offset = std::chrono::milliseconds(
            BilliardConfig::SHOT_CYCLE_TIMING.planningRetryCutoffMs + 1000);
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed &&
            result.value && !result.value->shotExecuted,
            "FixedForceEnvelopeRejected candidates are skipped, not hard failures; "
            "exhausting the candidate list past cutoff safely ends with no plan");
    }
    {
        // Phase1ExecutionCandidates::isValid()要求legalContactPlans每筆都
        // 具備與anchor相同的Phase1 target；用DirectPot real plan硬塞進legal
        // 槽位無法通過。改用rankedPotPlans=空、legalContactPlans=1筆真實
        // LegalContact plan，anchor自己比對自己，結構天然合法，仍完整涵蓋
        // 「沒有可行ranked-pot候選時fallback到legal-contact、potsExhausted=true」
        // 這條核心安全路徑。
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake;
        fake.planningResult = PlanningResult::shotPlan(
            buildRealShotPlan(),
            Phase1ExecutionCandidates{{}, {buildRealLegalContactPlan()}});
        std::size_t builtCount = 0;
        std::vector<bool> potsExhaustedSeen;
        fake.buildPlan = [plan, &builtCount, &potsExhaustedSeen](
                const ShotPlan&, bool potsExhausted) {
            ++builtCount;
            potsExhaustedSeen.push_back(potsExhausted);
            return ExecutionPlanResult::success(plan);
        };
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed &&
            builtCount == 1 && potsExhaustedSeen.size() == 1 && potsExhaustedSeen[0],
            "no viable ranked-pot candidates falls through to legal-contact candidates "
            "with potsExhausted=true");
    }
    {
        // Section 10: SDK failure（非明確NotReachable）不得換候選——preflight
        // 的checkReachable API本身失敗與「目標點不可達」是不同語意，前者
        // 是不確定/系統性失敗，必須立即停止，不可嘗試下一個候選。用2個
        // 候選、只讓第1個的reachable API失敗，驗證第2個從未被嘗試。
        FakeSdk sdk;
        sdk.reachableCode = -99;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake;
        fake.planningResult = PlanningResult::shotPlan(buildRealShotPlan(), candidatesWith(2));
        std::size_t sdkFailureBuiltCount = 0;
        fake.buildPlan = [plan, &sdkFailureBuiltCount](const ShotPlan&, bool) {
            ++sdkFailureBuiltCount;
            return ExecutionPlanResult::success(plan);
        };
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            sdkFailureBuiltCount == 1,
            "reachability-check SDK API failure (distinct from a confirmed NotReachable "
            "target) fails the cycle immediately without trying the next candidate");
    }
    {
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(plan);
        clock.offset = std::chrono::milliseconds(
            BilliardConfig::SHOT_CYCLE_TIMING.shotDeadlineMs -
            BilliardConfig::SHOT_CYCLE_TIMING.minimumExecutionReserveMs + 1000);
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed &&
            result.value && !result.value->shotExecuted &&
            !called(sdk, "ptp"),
            "a found candidate is discarded as a safe no-plan end when too little "
            "execution reserve time remains before the shot deadline, and safeApproachPose "
            "motion (checkedPtp) is never issued");
    }
    {
        // 同一道reserve-time門檻對Pull模式同樣適用：不得先做pre-extend
        // pulse（do1On）才發現時間不夠。
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        const ExecutionPlan lowReservePullPlan = validExecutionPlan(1, StrikeMode::Pull);
        FakeRealServices fake = singleCandidateSuccess(lowReservePullPlan);
        clock.offset = std::chrono::milliseconds(
            BilliardConfig::SHOT_CYCLE_TIMING.shotDeadlineMs -
            BilliardConfig::SHOT_CYCLE_TIMING.minimumExecutionReserveMs + 1000);
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed &&
            result.value && !result.value->shotExecuted &&
            !called(sdk, "do1On") && !called(sdk, "ptp"),
            "Pull mode: insufficient execution reserve time is caught before any "
            "pre-extend pulse or safeApproachPose motion is ever issued");
    }
    {
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        const ExecutionPlan pullPlan = validExecutionPlan(1, StrikeMode::Pull);
        FakeRealServices fake = singleCandidateSuccess(pullPlan);
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed &&
            orderedSubsequence(sdk.calls, {"do1On", "do1Off", "sleep", "do2On", "do2Off", "sleep"}),
            "Pull mode: extend-prepare before StrikeReady, retract during Pneumatic, full success");
    }
    {
        FakeSdk sdk;
        sdk.doFailure = DoFailurePoint::RetractOn;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        const ExecutionPlan pullPlan = validExecutionPlan(1, StrikeMode::Pull);
        FakeRealServices fake = singleCandidateSuccess(pullPlan);
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            result.diagnostic->reason == ExecutionCycleFailureReason::PneumaticFailed &&
            runtime.state == ExecutionCycleState::ManualRecoveryRequired,
            "Pull mode: extend completed but retract never completes requires manual recovery");
    }
    {
        FakeSdk sdk;
        sdk.doFailure = DoFailurePoint::StrikeOff;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        const ExecutionPlan pullPlan = validExecutionPlan(1, StrikeMode::Pull);
        FakeRealServices fake = singleCandidateSuccess(pullPlan);
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::UnknownUnsafe &&
            result.diagnostic->reason == ExecutionCycleFailureReason::PneumaticStateUnknown &&
            runtime.state == ExecutionCycleState::UnknownUnsafe,
            "Pull mode: extend-prepare pulse UnknownUnsafe overrides the final result to UnknownUnsafe");
        // 整輪ptpAxis並非零——StandbyReturn(準備姿態)/CameraPose在
        // pulseExtend失敗前已合法動過；要驗證的是「失敗點之後」沒有更多
        // 動作，不是「整輪」零計數。失敗點＝最後一次do1Off（strikeWasOn後
        // 的off pulse，正是UnknownUnsafe的觸發點）。
        const std::size_t failureIndex = lastIndexOf(sdk.calls, "do1Off");
        tests.expectTrue(
            noneCalledAfter(sdk.calls, failureIndex, {"ptp", "ptpAxis", "readPose", "lin"}),
            "Pull-prepare UnknownUnsafe stops immediately at the failing do1Off: no "
            "strike-ready PTP, actual-pose read, safe-lift LIN, or standby-return PTP "
            "occurs after that point");
    }
    {
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(plan);
        fake.buildPlan = [](const ShotPlan&, bool) {
            ExecutionPlan stale = validExecutionPlan(99);
            return ExecutionPlanResult::success(stale);
        };
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            result.diagnostic->reason == ExecutionCycleFailureReason::ExecutionPlanCycleMismatch &&
            !called(sdk, "ptp"),
            "a plan carrying a mismatched cycleIdentity never reaches strike execution");
    }
    {
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        const ExecutionCycleResult first = BilliardApp::runRealSingleCycle(
            runtime, 10, *robot, config, singleCandidateSuccess(validExecutionPlan(10)).services(),
            deadlineFor(clock));
        const ExecutionCycleResult second = BilliardApp::runRealSingleCycle(
            runtime, 11, *robot, config, singleCandidateSuccess(validExecutionPlan(11)).services(),
            deadlineFor(clock));
        tests.expectTrue(
            first.status == ExecutionCycleStatus::Completed &&
            second.status == ExecutionCycleStatus::Completed &&
            callCount(sdk, "do1On") == 2,
            "two sequential real cycles with distinct cycle identities both execute cleanly");
    }
    {
        // Section 10: 執行已開始後不以deadline強制abort。deadline的檢查
        // 只存在acquireExecutionPlan的重試迴圈裡；Planning結束、進入
        // StrikeReady/Pneumatic之後完全沒有deadline檢查碼。用onSleepCallback
        // 在Pull模式的waitDirectionChangeDelay/waitMechanismCompletion
        // 期間把clock推到遠超過shotDeadlineMs，證明cycle仍然正常完成。
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        // 第1次sleep來自Planning之前的establishSafeOutputsOff prep pulse
        // （跟本測試無關，若在這裡就把clock推過期，會讓Planning自己的
        // reserve-time門檻先擋下來，反而測不到「執行開始後deadline過期
        // 不abort」這件事）；從第2次sleep開始才是Planning完成、進入
        // Pull模式pre-extend/retract之後的waitDirectionChangeDelay／
        // waitMechanismCompletion，才是這個測試真正要驗證的時間點。
        int sleepCount = 0;
        sdk.onSleepCallback = [&clock, &sleepCount] {
            ++sleepCount;
            if (sleepCount < 2) return;
            clock.offset = std::chrono::milliseconds(
                BilliardConfig::SHOT_CYCLE_TIMING.shotDeadlineMs + 60000);
        };
        const ExecutionPlan midFlightPlan = validExecutionPlan(1, StrikeMode::Pull);
        FakeRealServices fake = singleCandidateSuccess(midFlightPlan);
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, fake.services(), deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed &&
            result.value && result.value->shotExecuted,
            "once execution has passed Planning and started (Pull pre-extend/retract "
            "waits trigger sleepMs), the shot deadline elapsing mid-flight does not "
            "abort the already-started cycle");
    }
    {
        // Section 10: 重連不重置15秒／10秒deadline——elapsed time必須是
        // 「跨重試累加」，不是每次重連嘗試各自重新從0起算。自訂
        // connectVision每次都往前推進時間；若deadline真的被重連重置，
        // pastCutoff()永遠不會成立，會一路重試到第3次才連上、走向完整
        // 成功流程而非retry-cutoff safe end。
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(validExecutionPlan(1));
        fake.visionConnected = false;
        int connectAttempts = 0;
        RealExecutionCycleServices services = fake.services();
        services.connectVision = [&]() -> VisionConnectResult {
            fake.calls.push_back("connectVision");
            ++connectAttempts;
            clock.offset += std::chrono::milliseconds(
                BilliardConfig::SHOT_CYCLE_TIMING.planningRetryCutoffMs / 2 + 100);
            if (connectAttempts >= 3) {
                fake.visionConnected = true;
                return {VisionConnectStatus::Connected, 0};
            }
            return {VisionConnectStatus::Retriable, 1};
        };
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, services, deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::Completed && result.value &&
                !result.value->shotExecuted && connectAttempts == 2,
            "elapsed time accumulates across reconnect attempts instead of resetting per "
            "attempt: retry-cutoff safe-end triggers after 2 accumulated attempts, never "
            "reaching the 3rd attempt that would have connected");
    }
    {
        // Section 10: 重連成功後、重開capture window前必須重新確認
        // CameraPose是否仍正確；姿態不符時必須ManualRecoveryRequired，
        // 不可假設重連期間機構完全沒動過。sdk.currentJoints維持預設
        // CAMERA_JOINT，讓最初seam.moveToCameraPose/confirmCameraPoseStopped
        // （現在也會核對真實關節角度，見P1-04後CameraPose位置確認修正）
        // 正常通過；只在connectVision成功那一刻才把currentJoints改成別的
        // 值，模擬重連期間機構真的動過，這樣才單獨測到
        // confirmSafeAtCameraPose在重連路徑上的檢查，不會跟seam自己的
        // CameraPose位置確認混在一起、在更早的步驟就先失敗。
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(validExecutionPlan(1));
        fake.visionConnected = false;
        RealExecutionCycleServices services = fake.services();
        services.connectVision = [&]() -> VisionConnectResult {
            fake.calls.push_back("connectVision");
            sdk.currentJoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 明顯不是CAMERA_JOINT
            fake.visionConnected = true;
            return {VisionConnectStatus::Connected, 0};
        };
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, services, deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            result.diagnostic->reason ==
                ExecutionCycleFailureReason::VisionReconnectManualRecoveryRequired &&
            runtime.state == ExecutionCycleState::ManualRecoveryRequired,
            "reconnect-time CameraPose mismatch is detected before reopening the "
            "capture window and requires manual recovery, not a silent retry");
    }
    {
        // Section 10: 重連時DO非OFF同樣禁止繼續。DO在cycle一開始的
        // read-only前置確認時是OFF（通過），但在vision斷線期間變成ON，
        // 於重連後的安全確認才被偵測到。
        FakeSdk sdk;
        auto robot = connected(sdk);
        OfflineExecutionRuntime runtime;
        FakeClock clock;
        FakeRealServices fake = singleCandidateSuccess(validExecutionPlan(1));
        fake.visionConnected = false;
        RealExecutionCycleServices services = fake.services();
        services.connectVision = [&]() -> VisionConnectResult {
            fake.calls.push_back("connectVision");
            sdk.outputs[1] = true;
            fake.visionConnected = true;
            return {VisionConnectStatus::Connected, 0};
        };
        const ExecutionCycleResult result = BilliardApp::runRealSingleCycle(
            runtime, 1, *robot, config, services, deadlineFor(clock));
        tests.expectTrue(
            result.status == ExecutionCycleStatus::SafeFailure &&
            result.diagnostic->reason ==
                ExecutionCycleFailureReason::VisionReconnectManualRecoveryRequired &&
            runtime.state == ExecutionCycleState::ManualRecoveryRequired,
            "reconnect-time DO-ON (pneumatic outputs not confirmed OFF) requires manual "
            "recovery before continuing, not a silent retry");
    }
    {
        // Section 10: Vision reconnect使用新connection identity、相同
        // shot-cycle identity。
        //
        // 這個block不是端到端測試，只測identity配發器本身；請勿把它的
        // PASS當成「run()的reconnect wiring已驗證」的證據。
        //
        // run()裡實際的connectionIdentity來自private成員visionClient
        // （真實SocketClient，連往寫死的VISION_SERVER_IP:VISION_SERVER_PORT，
        // 兩者都不透過BilliardAppRunTestSeam注入）。要在不修改SocketClient.*
        // （第13節禁止）的前提下驅動它走完整個run()迴圈，需要在測試裡自建
        // 綁定該固定位址的真實TCP監聽器並跨執行緒協調——會引入port佔用
        // 衝突與執行緒時序的額外不穩定風險，代價明顯超過這一項單獨的
        // 驗證價值，因此本session不建立這種整合測試。
        //
        // 以下只直接測試SocketClient.h公開匯出的LocalConnectionLifecycle
        // ——這是SocketClient::connectionIdentity()背後真正的identity
        // 配發機制（真實程式碼，非mock），證明「每次open()都拿到全新、
        // 嚴格遞增的identity，兩次open()之間維持穩定」這件事本身成立。
        // run()端的wiring（openCaptureWindow每次都讀取當下
        // visionClient.connectionIdentity()、cycleIdentity參數全程不變）
        // 未經任何自動測試驗證，只有程式碼審查佐證，見計畫書10.1節第14項。
        // 若之後要補端到端驗證，需要先讓VISION_SERVER_IP/PORT可經test
        // seam覆寫，那是一次獨立、需另外核准的原始碼變更。
        LocalConnectionLifecycle lifecycle;
        const ConnectionIdentity first = lifecycle.open();
        tests.expectTrue(
            first != 0 && lifecycle.current() == first,
            "[allocator-only, not end-to-end] first open() issues a nonzero identity "
            "that current() reflects");
        tests.expectTrue(
            lifecycle.current() == first && lifecycle.current() == first,
            "[allocator-only, not end-to-end] current() stays stable across repeated "
            "queries without a new open() (same shot-cycle window, no reconnect)");
        const ConnectionIdentity second = lifecycle.open();
        tests.expectTrue(
            second != first && second > first && lifecycle.current() == second,
            "[allocator-only, not end-to-end] a second open() issues a strictly new, "
            "distinct connection identity, and current() reflects only the latest one");
    }

    // ============================================================
    // Section D: BilliardApp::run()鍵盤／cycle-identity外層迴圈。
    // run()是真實阻塞式while迴圈（無key事件時呼叫真實Sleep()），必須設計
    // 成有界：poll mock在腳本化事件序列之外一律拋出下面這個「僅供測試
    // 邊界」用的專屬型別，而不是std::runtime_error/std::exception這類
    // 一般型別——run()本身若意外拋出其他std::exception（真正的bug），
    // catch (const std::exception&)會把它吃掉變成假綠，所以只能抓這個
    // 專屬sentinel，其餘一律讓它往外傳播、讓測試明顯崩潰。
    // ============================================================
    struct TestPollBoundaryReached {};

    {
        // Section 10:「P只回準備姿態，不啟動shot cycle」+「shot-cycle
        // identity只在真的按下新的H才會往上加」。額外擷取「P處理完、H
        // 尚未處理」那個時間點的呼叫紀錄快照，直接證明P自己的流程完全
        // 不含pneumatic或full-strike motion（不是只看最終總數）。
        FakeSdk sdk;
        // 2026-08-16新增：H/DI1按下時若手臂還沒在Standby，run()現在會先
        // 做未計時的runPOnly()式回Standby準備、不算成一次shot cycle
        // （見run()裡的pre-standby check）。這裡的P本來就會PTP到
        // STANDBY_JOINT_REFERENCE，但FakeSdk.currentJoints是靜態欄位、
        // 不會真的因為模擬的PTP指令而更新，所以要顯式設成Standby，讓
        // 之後的H直接被視為「已在Standby」，才能照原本測試意圖驗證
        // P→H的cycle-identity/呼叫次序，不被新加的pre-standby check擋下。
        sdk.currentJoints = BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg;
        auto robot = connected(sdk);
        FakeRealServices fakeServices = singleCandidateSuccess(validExecutionPlan(1));
        int pollCall = 0;
        std::vector<std::string> callsAfterPBeforeH;
        BilliardAppRunTestSeam seam;
        seam.policyMode = BilliardConfig::ExecutionPolicyMode::RealHardware;
        seam.policyRevision = "policy-test-v1";
        seam.legalContactExecutionAuthorized = false;
        seam.robot = robot.get();
        seam.realConfig = config;
        seam.motionPlanningPolicyMode = BilliardConfig::ExecutionPolicyMode::RealHardware;
        seam.realServices = fakeServices.services();
        seam.consoleKeyPoll = [&]() -> std::vector<ConsoleKeyEvent> {
            ++pollCall;
            switch (pollCall) {
            case 1: return {{ConsoleKey::P, true}};
            case 2:
                // runPOnly()（P觸發）已在call 1後、call 2前完整跑完；
                // 這裡擷取的快照就是P自己造成的全部硬體呼叫。
                callsAfterPBeforeH = sdk.calls;
                return {{ConsoleKey::P, false}, {ConsoleKey::H, true}};
            case 3: return {};  // resyncStartControlToIdle's own poll after cycle 1
            default:
                throw TestPollBoundaryReached{};
            }
        };
        seam.consoleKeyDownQuery = [](ConsoleKey) { return false; };
        BilliardApp app(std::move(seam));
        bool reachedExpectedBoundary = false;
        try {
            app.run();
        } catch (const TestPollBoundaryReached&) {
            reachedExpectedBoundary = true;
        }
        tests.expectTrue(reachedExpectedBoundary && pollCall == 4,
            "run() loop reaches the expected post-cycle re-poll boundary at exactly "
            "the 4th poll (confirms it returned to WaitingForStart normally after "
            "one P then one H, not some other exit)");
        tests.expectTrue(
            !callsAfterPBeforeH.empty() &&
                std::find(callsAfterPBeforeH.begin(), callsAfterPBeforeH.end(), "do1On") ==
                    callsAfterPBeforeH.end() &&
                std::find(callsAfterPBeforeH.begin(), callsAfterPBeforeH.end(), "ptp") ==
                    callsAfterPBeforeH.end() &&
                std::find(callsAfterPBeforeH.begin(), callsAfterPBeforeH.end(), "lin") ==
                    callsAfterPBeforeH.end(),
            "P key-down's own effects (before H is ever processed) contain no "
            "pneumatic fire and no strike-ready/safe-lift motion — P truly only "
            "returns to standby");
        tests.expectTrue(
            callCount(sdk, "do1On") == 1 && fakeServices.openedWindowFor == 1,
            "P key-down alone never consumes a shot-cycle identity; the "
            "following H runs as cycle 1 (not cycle 2), firing pneumatic exactly once");
    }
    {
        // Section 10: active cycle期間排入console queue的H事件不得在回到
        // WaitingForStart後自動啟動下一輪。
        FakeSdk sdk;
        // 見上一個測試區塊的說明：顯式設成Standby，讓H不被新加的
        // pre-standby check攔截成未計時的準備動作。
        sdk.currentJoints = BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg;
        auto robot = connected(sdk);
        FakeRealServices fakeServices = singleCandidateSuccess(validExecutionPlan(1));
        int pollCall = 0;
        BilliardAppRunTestSeam seam;
        seam.policyMode = BilliardConfig::ExecutionPolicyMode::RealHardware;
        seam.policyRevision = "policy-test-v1";
        seam.legalContactExecutionAuthorized = false;
        seam.robot = robot.get();
        seam.realConfig = config;
        seam.motionPlanningPolicyMode = BilliardConfig::ExecutionPolicyMode::RealHardware;
        seam.realServices = fakeServices.services();
        seam.consoleKeyPoll = [&pollCall]() -> std::vector<ConsoleKeyEvent> {
            ++pollCall;
            switch (pollCall) {
            case 1: return {{ConsoleKey::H, true}};
            // 模擬cycle 1執行期間使用者又按了一次H（尚未放開）：這次事件
            // 只能在resyncStartControlToIdle被drain，不得產生新edge。
            case 2: return {{ConsoleKey::H, true}};
            // 主迴圈回到WaitingForStart後的下一次poll：H仍是同一次長按，
            // 不得被誤判成新的一次down。
            case 3: return {};
            default:
                throw TestPollBoundaryReached{};
            }
        };
        seam.consoleKeyDownQuery = [](ConsoleKey key) { return key == ConsoleKey::H; };
        BilliardApp app(std::move(seam));
        bool reachedExpectedBoundary = false;
        try {
            app.run();
        } catch (const TestPollBoundaryReached&) {
            reachedExpectedBoundary = true;
        }
        tests.expectTrue(reachedExpectedBoundary && pollCall == 4,
            "run() loop reaches the expected post-cycle re-poll boundary at exactly "
            "the 4th poll after absorbing the queued H, without exiting for some other reason");
        tests.expectTrue(
            callCount(sdk, "do1On") == 1 && fakeServices.openedWindowFor == 1 &&
                callCount(sdk, "ptpAxis") == 2,
            "an H event queued/held during the active cycle does not auto-start a "
            "second cycle once WaitingForStart is reached again (exactly one pneumatic "
            "fire; ptpAxis count matches a single cycle's PreparationReturn+StandbyReturn, "
            "not a phantom extra cycle)");
    }
    {
        // Section 10: active cycle期間排入console queue的P事件同樣不得在
        // 回到WaitingForStart後自動觸發任何動作（runPOnly()只在主迴圈的
        // standbyEdge分支呼叫；resync本身結構上不可能觸發它，這裡用
        // ptpAxis計數驗證沒有多出一次runPOnly()專屬的standby PTP）。
        FakeSdk sdk;
        // 見前面測試區塊的說明：顯式設成Standby，讓H不被新加的
        // pre-standby check攔截成未計時的準備動作。
        sdk.currentJoints = BilliardConfig::STANDBY_JOINT_REFERENCE.jointDeg;
        auto robot = connected(sdk);
        FakeRealServices fakeServices = singleCandidateSuccess(validExecutionPlan(1));
        int pollCall = 0;
        BilliardAppRunTestSeam seam;
        seam.policyMode = BilliardConfig::ExecutionPolicyMode::RealHardware;
        seam.policyRevision = "policy-test-v1";
        seam.legalContactExecutionAuthorized = false;
        seam.robot = robot.get();
        seam.realConfig = config;
        seam.motionPlanningPolicyMode = BilliardConfig::ExecutionPolicyMode::RealHardware;
        seam.realServices = fakeServices.services();
        seam.consoleKeyPoll = [&pollCall]() -> std::vector<ConsoleKeyEvent> {
            ++pollCall;
            switch (pollCall) {
            case 1: return {{ConsoleKey::H, true}};
            case 2: return {{ConsoleKey::P, true}};  // queued during cycle 1
            case 3: return {};
            default:
                throw TestPollBoundaryReached{};
            }
        };
        seam.consoleKeyDownQuery = [](ConsoleKey key) { return key == ConsoleKey::P; };
        BilliardApp app(std::move(seam));
        bool reachedExpectedBoundary = false;
        try {
            app.run();
        } catch (const TestPollBoundaryReached&) {
            reachedExpectedBoundary = true;
        }
        tests.expectTrue(reachedExpectedBoundary && pollCall == 4,
            "run() loop reaches the expected post-cycle re-poll boundary at exactly "
            "the 4th poll after absorbing the queued P, without exiting for some other reason");
        tests.expectTrue(
            callCount(sdk, "do1On") == 1 && fakeServices.openedWindowFor == 1 &&
                callCount(sdk, "ptpAxis") == 2,
            "a P event queued/held during the active cycle never triggers a "
            "spurious extra runPOnly() (would add a 3rd ptpAxis call)");
    }
    // Section 10項目「Vision reconnect使用新connection identity、相同
    // shot-cycle identity」未涵蓋：connectionIdentity來自private成員
    // visionClient（SocketClient），不透過BilliardAppRunTestSeam注入，
    // 且SocketClient.*依section 13明確不得修改，無法在不違反此限制的
    // 前提下獨立單元測試這一項；程式碼檢視確認openCaptureWindow每次都
    // 讀取當下visionClient.connectionIdentity()、cycleIdentity參數維持
    // 不變，結構上滿足需求，但缺少獨立可執行測試佐證。

    return tests.exitCode();
}
