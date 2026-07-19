#pragma once

#include "Algorithm.h"
#include "TableState.h"

class ShotPlanner {
public:
    ShotDecision createPlan(const TargetSelection& target) const;
};
