// 離線工具：讀取既有32值視覺CSV，套用Phase1既有流程（VisionDataParser→
// ThreeEventStability→TargetSelector→BilliardAlgorithm::planShot），
// 並列印最終打擊plan。僅供離線示範/研究用途，不驅動硬體。
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <optional>
#include <string>
#include <variant>

#include "Algorithm.h"
#include "BilliardConfig.h"
#include "BilliardPhysics.h"
#include "TableState.h"
#include "TargetSelector.h"
#include "VisionDataParser.h"

namespace {

constexpr double PI = 3.14159265358979323846;

const char* frameStatusName(SingleFrameStatus status)
{
    switch (status) {
        case SingleFrameStatus::Success: return "Success";
        case SingleFrameStatus::WrongFieldCount: return "WrongFieldCount(不是32個欄位)";
        case SingleFrameStatus::EmptyToken: return "EmptyToken(有欄位是空字串)";
        case SingleFrameStatus::InvalidNumericToken: return "InvalidNumericToken(無法解析成數字)";
        case SingleFrameStatus::NumericOverflow: return "NumericOverflow(數值溢位)";
        case SingleFrameStatus::NonFiniteValue: return "NonFiniteValue(NaN/Inf)";
        case SingleFrameStatus::InvalidSentinelPair: return "InvalidSentinelPair(缺值sentinel只有一半)";
        case SingleFrameStatus::MissingRequiredCueBall: return "MissingRequiredCueBall(母球座標不可缺)";
        case SingleFrameStatus::MissingRequiredPocket: return "MissingRequiredPocket(袋口座標不可缺)";
        case SingleFrameStatus::OutOfObservationBounds: return "OutOfObservationBounds(超出VISION_OBSERVATION_BOUNDS)";
        case SingleFrameStatus::ConfigurationMissing: return "ConfigurationMissing(VISION_OBSERVATION_BOUNDS未設定)";
        case SingleFrameStatus::InvalidConfiguration: return "InvalidConfiguration";
    }
    return "Unknown";
}

const char* stabilityFailureName(StabilityFailureReason reason)
{
    switch (reason) {
        case StabilityFailureReason::AwaitingEvents: return "AwaitingEvents";
        case StabilityFailureReason::ConfigurationMissing: return "ConfigurationMissing";
        case StabilityFailureReason::InvalidConfiguration: return "InvalidConfiguration";
        case StabilityFailureReason::InvalidEvent: return "InvalidEvent";
        case StabilityFailureReason::ConnectionChanged: return "ConnectionChanged";
        case StabilityFailureReason::CycleChanged: return "CycleChanged";
        case StabilityFailureReason::EventIdNotIncreasing: return "EventIdNotIncreasing";
        case StabilityFailureReason::ReceiveTimeWentBackward: return "ReceiveTimeWentBackward";
        case StabilityFailureReason::TimedOut: return "TimedOut";
        case StabilityFailureReason::PresenceChanged: return "PresenceChanged";
        case StabilityFailureReason::BallMoved: return "BallMoved(三幀同一顆球超出容差)";
        case StabilityFailureReason::PocketMoved: return "PocketMoved(三幀袋口超出容差)";
        case StabilityFailureReason::Disconnected: return "Disconnected";
        case StabilityFailureReason::Reconnected: return "Reconnected";
        case StabilityFailureReason::ParserFailure: return "ParserFailure";
        case StabilityFailureReason::ExplicitReset: return "ExplicitReset";
    }
    return "Unknown";
}

const char* noPlanReasonName(NoPlanReason reason)
{
    switch (reason) {
        case NoPlanReason::NoEligibleTarget: return "NoEligibleTarget(桌上沒有可打的號碼球)";
        case NoPlanReason::NoPotCandidate: return "NoPotCandidate(有目標球但找不到可進袋路徑)";
        case NoPlanReason::NoLegalContact: return "NoLegalContact(連合法觸球都找不到)";
        case NoPlanReason::InvalidBrainConfiguration: return "InvalidBrainConfiguration(BrainConfig/幾何設定缺失)";
        case NoPlanReason::NumericalPlanningFailure: return "NumericalPlanningFailure(內部數值/狀態驗證失敗)";
    }
    return "Unknown";
}

const char* pocketLabel(std::size_t index)
{
    // 對應 pocketCenters() 逆時鐘走訪順序：BL, BM, BR, TR, TM, TL
    static const char* labels[6] = {
        "P1 左下角", "P2 下中袋", "P3 右下角", "P4 右上角", "P5 上中袋", "P6 左上角"};
    return index < 6 ? labels[index] : "P?";
}

void printPoint(const char* label, Point p)
{
    std::printf("  %-14s (%.2f, %.2f)\n", label, p.x, p.y);
}

void printSegment(const char* label, Segment2D seg)
{
    const double dx = seg.end.x - seg.start.x;
    const double dy = seg.end.y - seg.start.y;
    const double len = std::hypot(dx, dy);
    const double headingDeg = std::atan2(dy, dx) * 180.0 / PI;
    std::printf(
        "  %-14s (%.2f, %.2f) -> (%.2f, %.2f)  距離=%.2fmm  方向角=%.2fdeg\n",
        label, seg.start.x, seg.start.y, seg.end.x, seg.end.y, len, headingDeg);
}

// production BilliardConfig::BRAIN_CONFIG 的 kickGeometry / scoring /
// base0PlanarCalibrationRevision 目前皆為 nullopt（尚未完成人工核准與標定），
// BilliardAlgorithm::planShot 依規格會直接 fail closed 回傳
// NoPlan(InvalidBrainConfiguration)。這裡採用與
// tests/phase1_algorithm_regression_tests.cpp 完全相同的研究/測試預設值
// （kickConfig()/scoringConfig()/brainConfig()），純粹用來離線示範完整演算法
// 流程；正式上線前仍須由人工核准 BilliardConfig.cpp 對應區塊，不可直接拿本
// 檔案的數值當作正式標定值。
BilliardConfig::BrainConfig demoBrainConfig()
{
    const BilliardConfig::KickGeometryConfig kick{89.0, 1e-8, 1e-8};
    const BilliardConfig::ScoringConfig scoring{
        BilliardConfig::INITIAL_EXPERIMENTAL_SCORING_WEIGHTS,
        1e-9,
        90.0,
        0.0,
        3000.0,
        200.0,
        1e-9,
        BilliardConfig::PlanningMode::PotOnly};
    return BilliardConfig::BrainConfig{
        std::optional<std::string>{"offline-demo-v1"},
        std::optional<BilliardConfig::KickGeometryConfig>{kick},
        std::optional<BilliardConfig::ScoringConfig>{scoring}};
}

void printShotPlan(const ShotPlan& plan)
{
    const char* typeName = "?";
    switch (plan.type) {
        case ShotPlanType::DirectPot: typeName = "直擊進袋 (Direct Pot)"; break;
        case ShotPlanType::KickPot: typeName = "一次碰庫進袋 (Kick Pot)"; break;
        case ShotPlanType::DirectLegalContact: typeName = "直擊合法觸球 (無進球把握，Manual Research)"; break;
        case ShotPlanType::KickLegalContact: typeName = "碰庫合法觸球 (無進球把握，Manual Research)"; break;
    }

    std::printf("\n================ 最終打擊 Plan ================\n");
    std::printf("類型 Type      : %s\n", typeName);
    std::printf("目標球 Target  : 第 %d 號球  (%.2f, %.2f)\n",
        plan.selectedTarget.ballNumber,
        plan.selectedTarget.center.x,
        plan.selectedTarget.center.y);
    std::printf("母球起始位置    : (%.2f, %.2f)\n",
        plan.source.cueBallSnapshot.x, plan.source.cueBallSnapshot.y);
    std::printf("出桿方向單位向量 : (%.6f, %.6f)  即 %.2f deg\n",
        plan.shotDirectionXY.x, plan.shotDirectionXY.y,
        std::atan2(plan.shotDirectionXY.y, plan.shotDirectionXY.x) * 180.0 / PI);
    printPoint("Ghost Ball 瞄準點", plan.ghostBallPoint.center);

    for (std::size_t i = 0; i < plan.cuePathSegments.size(); ++i) {
        const std::string label = "母球路徑" + std::to_string(i + 1);
        printSegment(label.c_str(), plan.cuePathSegments[i]);
    }

    if (plan.minimumClearanceMm) {
        std::printf("障礙淨空 Clearance: %.2f mm\n", *plan.minimumClearanceMm);
    } else {
        std::printf("障礙淨空 Clearance: 無其他球需考慮\n");
    }

    if (const auto* direct = std::get_if<DirectPotShotPlanPayload>(&plan.payload)) {
        std::printf("進袋口 Pocket    : %s\n", pocketLabel(static_cast<std::size_t>(direct->candidate.pocketId)));
        std::printf("目標球路徑        : (%.2f,%.2f) -> (%.2f,%.2f)\n",
            direct->candidate.targetPath.start.x, direct->candidate.targetPath.start.y,
            direct->candidate.targetPath.end.x, direct->candidate.targetPath.end.y);
        std::printf("切球角 Cut Angle : %.2f deg\n", direct->candidate.cuttingAngleDeg);
        std::printf("評分 Total Cost  : %.4f (越低越好, 0~1)\n", direct->scoring.totalCost);
    } else if (const auto* kick = std::get_if<KickPotShotPlanPayload>(&plan.payload)) {
        std::printf("進袋口 Pocket    : %s\n", pocketLabel(static_cast<std::size_t>(kick->candidate.pocketId)));
        std::printf("碰庫 Rail        : Rail%d\n", static_cast<int>(kick->candidate.railId) + 1);
        printPoint("反彈點 Rebound", kick->candidate.reboundPoint);
        std::printf("切球角 Cut Angle : %.2f deg\n", kick->candidate.cuttingAngleDeg);
        std::printf("碰庫角 Rail Angle: %.2f deg\n", kick->candidate.incidenceAngleDeg);
        std::printf("評分 Total Cost  : %.4f (越低越好, 0~1)\n", kick->scoring.totalCost);
    } else if (const auto* directLegal = std::get_if<DirectLegalContactShotPlanPayload>(&plan.payload)) {
        std::printf("[注意] 此為 ManualResearch 合法觸球，非把握進球；預設不可授權真實硬體執行。\n");
        std::printf("總路徑長度        : %.2f mm\n", directLegal->candidate.totalPathLengthMm);
    } else if (const auto* kickLegal = std::get_if<KickLegalContactShotPlanPayload>(&plan.payload)) {
        std::printf("[注意] 此為 ManualResearch 碰庫合法觸球，非把握進球；預設不可授權真實硬體執行。\n");
        std::printf("碰庫 Rail         : Rail%d\n", static_cast<int>(kickLegal->candidate.railId) + 1);
        std::printf("總路徑長度        : %.2f mm\n", kickLegal->candidate.totalPathLengthMm);
    }
    std::printf("=================================================\n");
}

void printNoPlan(const NoPlan& noPlan)
{
    std::printf("\n================ 無可執行 Plan ================\n");
    std::printf("原因 Reason      : %s\n", noPlanReasonName(noPlan.reason));
    if (noPlan.selectedTarget) {
        std::printf("已選定目標球      : 第 %d 號球 (%.2f, %.2f)\n",
            noPlan.selectedTarget->ballNumber,
            noPlan.selectedTarget->center.x,
            noPlan.selectedTarget->center.y);
    }
    std::printf("feasiblePotCount : %zu\n", noPlan.feasiblePotCount);
    std::printf("proceededToLegal : %s\n", noPlan.proceededToLegalContact ? "true" : "false");
    std::printf("=================================================\n");
}

}  // namespace

int main(int argc, char** argv)
{
    std::string line;
    if (argc > 1) {
        line = argv[1];
    } else {
        std::printf("請輸入既有32值視覺CSV（9顆號碼球x,y + 母球x,y + 6袋口x,y，缺球以-9999,-9999表示）：\n");
        std::getline(std::cin, line);
    }

    VisionDataParser parser;
    const SingleFrameResult frameResult = parser.parse(line);
    if (frameResult.status() != SingleFrameStatus::Success || !frameResult.value()) {
        std::fprintf(stderr, "[Vision解析失敗] status=%s", frameStatusName(frameResult.status()));
        if (frameResult.diagnostic() && frameResult.diagnostic()->fieldIndex) {
            std::fprintf(stderr, "  fieldIndex=%zu", *frameResult.diagnostic()->fieldIndex);
        }
        std::fprintf(stderr, "\n");
        return 1;
    }
    const ValidatedVisionFrame& frame = *frameResult.value();

    if (!BilliardConfig::STABLE_FRAME_TOLERANCE_MM ||
        !BilliardConfig::POCKET_STABILITY_TOLERANCE_MM ||
        !BilliardConfig::MAX_INTER_FRAME_INTERVAL_MS) {
        std::fprintf(stderr, "[設定缺失] 三幀穩定性設定未完整。\n");
        return 1;
    }

    ThreeEventStability stability(StabilityConfig{
        BilliardConfig::STABLE_FRAME_TOLERANCE_MM,
        BilliardConfig::POCKET_STABILITY_TOLERANCE_MM,
        std::optional<std::chrono::milliseconds>{
            std::chrono::milliseconds(*BilliardConfig::MAX_INTER_FRAME_INTERVAL_MS)}});

    const auto now = std::chrono::steady_clock::now();
    StabilityResult stabilityResult = StabilityResult::needMoreEvents(0);
    // 離線單張影像沒有第二、三幀真實抖動資料，此處以同一幀重複三次餵入
    // 既有P1-04三幀中位數/容差流程，換取與production完全相同的驗證路徑。
    for (int i = 0; i < 3; ++i) {
        ReceiveEvent event{
            1,
            1,
            static_cast<ReceiveEventId>(i + 1),
            now + std::chrono::milliseconds(i * 10),
            frame};
        stabilityResult = stability.accept(event);
    }

    if (stabilityResult.status() != StabilityStatus::Stable || !stabilityResult.value()) {
        std::fprintf(stderr, "[三幀穩定性失敗] reason=%s\n",
            stabilityResult.diagnostic()
                ? stabilityFailureName(stabilityResult.diagnostic()->reason)
                : "Unknown");
        return 1;
    }
    const StableTableState table = *stabilityResult.value();

    std::printf("================ 解析後桌面狀態 ================\n");
    printPoint("母球 CueBall", table.cueBall);
    for (std::size_t i = 0; i < table.objectBalls.size(); ++i) {
        if (table.objectBalls[i]) {
            const std::string label = std::to_string(i + 1) + "號球";
            printPoint(label.c_str(), *table.objectBalls[i]);
        }
    }
    for (std::size_t i = 0; i < table.pockets.size(); ++i) {
        printPoint(pocketLabel(i), table.pockets[i]);
    }
    std::printf("=================================================\n");

    const PlanningResult planning = BilliardAlgorithm::planShot(
        table,
        BilliardConfig::TABLE_GEOMETRY,
        demoBrainConfig());

    if (!planning.isValid()) {
        std::fprintf(stderr, "[內部錯誤] PlanningResult.isValid() == false\n");
        return 1;
    }

    if (const auto* plan = std::get_if<ShotPlan>(&planning.value())) {
        printShotPlan(*plan);
    } else if (const auto* np = std::get_if<NoPlan>(&planning.value())) {
        printNoPlan(*np);
    }
    return 0;
}
