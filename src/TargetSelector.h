#pragma once

#include "TableState.h"

class TargetSelector {
public:
    [[nodiscard]] TargetQualificationResult select(
        const StableTableState& table) const;
};
