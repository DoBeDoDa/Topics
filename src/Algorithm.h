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

#ifdef BILLIARDS_P1_08_TEST_SEAM
    [[nodiscard]] static bool tieBreakBetterForTest(
        const ScoredPotCandidate& candidate,
        const ScoredPotCandidate& current) noexcept;
#endif
};
