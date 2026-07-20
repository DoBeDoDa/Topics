// 宣告根據球桌狀態建立目標選擇結果的元件。
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
