#include "TargetSelector.h"

#include "MathUtils.h"

TargetQualificationResult TargetSelector::select(
    const StableTableState& table) const
{
    for (std::size_t index = 0; index < table.objectBalls.size(); ++index) {
        const auto& ball = table.objectBalls[index];
        if (ball && !BilliardMath::isFinite(*ball)) {
            return TargetQualificationResult::rejected(
                TargetQualificationStatus::InvalidStableState);
        }
    }

    for (std::size_t index = 0; index < table.objectBalls.size(); ++index) {
        if (table.objectBalls[index]) {
            return TargetQualificationResult::success({
                static_cast<int>(index) + 1,
                *table.objectBalls[index]});
        }
    }

    return TargetQualificationResult::rejected(
        TargetQualificationStatus::NoEligibleTarget);
}
