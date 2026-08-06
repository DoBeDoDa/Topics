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
