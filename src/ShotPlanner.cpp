#include "ShotPlanner.h"

#include "BilliardConfig.h"

ShotDecision ShotPlanner::createPlan(const TargetSelection& target) const {
    return BilliardAlgorithm::decideShot(
        target.cueBall,
        target.targetBall,
        target.destinationPocket,
        target.railA,
        target.railB,
        target.obstacles,
        BilliardConfig::BALL_DIAMETER_MM,
        target.pocketNumber
    );
}
