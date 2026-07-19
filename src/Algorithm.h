#pragma once
#include <string>
#include <vector>
#include "Point.h"
#include "MathUtils.h"
#include "BilliardPhysics.h"
#include "TableState.h"

struct ShotDecision {
    Point best_aim_target;
    std::string strategy_name;
    double angle_deg;
    bool direct_path_blocked;
    bool bank_route_safe;
};

class BilliardAlgorithm {
public:
    static ShotDecision decideShot(const TargetSelection& target);

    static ShotDecision decideShot(
        Point bw,
        Point target_arm,
        Point destination,
        Point rail_A,
        Point rail_B,
        const std::vector<Point>& obs_list,
        double BALL_D,
        int best_pocket_idx
    );
};
