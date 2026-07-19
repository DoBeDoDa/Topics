#pragma once

#include <array>
#include <string>

#include "BilliardConfig.h"
#include "Point.h"

struct MotionPlan {
    std::array<double, 6> transitJoint;
    std::array<double, 6> readyPose;
    std::array<double, 6> strikePose;
    double aimAngleDeg = 0.0;
};

class MotionPlanner {
public:
    bool createPlan(
        Point cueBall,
        Point aimTarget,
        const BilliardConfig::MotionProfile& profile,
        MotionPlan& output,
        std::string& error
    ) const;
};
