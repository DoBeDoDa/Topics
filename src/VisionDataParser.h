#pragma once

#include <string>

#include "TableState.h"

class VisionDataParser {
public:
    bool parse(const std::string& message, TableState& output, std::string& error) const;
};
