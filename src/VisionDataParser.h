// 宣告 Python 視覺 CSV 封包到 TableState 的解析介面。
#pragma once

#include <string>

#include "TableState.h"

class VisionDataParser {
public:
    bool parse(const std::string& message, TableState& output, std::string& error) const;
};
