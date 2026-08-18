#pragma once

#include "BilliardPhysics.h"
#include "TableState.h"

class BilliardAlgorithm {
public:
    [[nodiscard]] static PlanningResult planShot(
        const StableTableState& table,
        const std::optional<BilliardConfig::TableGeometryConfig>& geometryConfig,
        const BilliardConfig::BrainConfig& brainConfig);

    [[nodiscard]] static DirectPotGenerationResult generateDirectPotCandidates(
        const StableTableState& table,
        const EligibleTarget& selectedTarget,
        const ResolvedTableGeometry& geometry);

    [[nodiscard]] static KickPotGenerationResult generateKickPotCandidates(
        const StableTableState& table,
        const EligibleTarget& selectedTarget,
        const ResolvedTableGeometry& geometry,
        const std::optional<BilliardConfig::KickGeometryConfig>& config);

    [[nodiscard]] static PotSelectionResult selectBestPot(
        const StableTableState& table,
        const EligibleTarget& selectedTarget,
        const ResolvedTableGeometry& geometry,
        const DirectPotEvaluation& directCandidates,
        const KickPotEvaluation& kickCandidates,
        const std::optional<BilliardConfig::ScoringConfig>& scoringConfig,
        const std::optional<BilliardConfig::KickGeometryConfig>& kickConfig);

    [[nodiscard]] static RankedPotSelectionResult rankPotCandidates(
        const StableTableState& table,
        const EligibleTarget& selectedTarget,
        const ResolvedTableGeometry& geometry,
        const DirectPotEvaluation& directCandidates,
        const KickPotEvaluation& kickCandidates,
        const std::optional<BilliardConfig::ScoringConfig>& scoringConfig,
        const std::optional<BilliardConfig::KickGeometryConfig>& kickConfig);

    // 執行層最後一道保底：Phase1當初判定rankedPotPlans/legalContactPlans
    // 幾何可行、所以沒有預先生成cueBallContactOnlyPlans，但這些候選後來
    // 在P2-01姿態搜尋／硬體可達性檢查全部失敗時，呼叫端（BilliardApp的
    // tryCandidates、no_fire_dry_run的單一shotPlan流程）用同一個
    // PlanningSourceAudit現場補生成360度方向候選，讓母球至少有安全推出
    // 的機會，不必整輪直接失敗。跟Phase1既有的cueBallContactOnlyPlans
    // 共用同一個內部生成邏輯、角度間距設定，以及對source.otherBallsSnapshot
    // 的前方路徑碰撞檢查（geometry用來取得ballDiameterMm/collisionMarginMm）。
    [[nodiscard]] static std::vector<ShotPlan>
    generateCueBallContactOnlyExecutionFallback(
        const PlanningSourceAudit& source,
        const ResolvedTableGeometry& geometry);

    // Production exhaustion fallback: rebuilds a stable-table view from the
    // audited snapshot, selects the real lowest-number present target, and
    // emits only collision-cleared direct legal-contact plans in the approved
    // deterministic offset order (0, +5, -5, +10, -10 degrees).
    [[nodiscard]] static std::vector<ShotPlan>
    generateForcedLegalContactExecutionFallback(
        const PlanningSourceAudit& source,
        const ResolvedTableGeometry& geometry);

#ifdef BILLIARDS_P1_08_TEST_SEAM
    struct LegalContactTestEvaluation {
        std::optional<DirectLegalContactCandidate> direct;
        std::array<std::optional<KickLegalContactCandidate>, 6> kicks;
        std::vector<LegalContactCandidateDiagnostic> diagnostics;
        bool selectedDirect;
    };

    [[nodiscard]] static bool tieBreakBetterForTest(
        const ScoredPotCandidate& candidate,
        const ScoredPotCandidate& current) noexcept;

    [[nodiscard]] static LegalContactTestEvaluation generateLegalContactForTest(
        const StableTableState& table,
        const EligibleTarget& selectedTarget,
        const ResolvedTableGeometry& geometry,
        const BilliardConfig::KickGeometryConfig& kickConfig);

    [[nodiscard]] static bool legalKickBetterForTest(
        const KickLegalContactCandidate& candidate,
        const KickLegalContactCandidate& current) noexcept;
#endif
};
