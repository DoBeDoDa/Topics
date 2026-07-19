#pragma once

#include <string>

#include "TableState.h"

class TargetSelector {
public:
    bool select(
        const TableState& table,
        TargetSelection& output,
        std::string& error
    ) const;
};
