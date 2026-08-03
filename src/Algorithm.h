#pragma once

#include "BilliardPhysics.h"
#include "TableState.h"

class BilliardAlgorithm {
public:
    [[nodiscard]] static DirectPotGenerationResult generateDirectPotCandidates(
        const StableTableState& table,
        const EligibleTarget& selectedTarget,
        const ResolvedTableGeometry& geometry);

    [[nodiscard]] static KickPotGenerationResult generateKickPotCandidates(
        const StableTableState& table,
        const EligibleTarget& selectedTarget,
        const ResolvedTableGeometry& geometry,
        const std::optional<BilliardConfig::KickGeometryConfig>& config);
};
