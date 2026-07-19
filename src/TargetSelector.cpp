#include "TargetSelector.h"

#include <limits>

#include "MathUtils.h"

namespace {

Point compensated(const DetectedPoint& point) {
    return BilliardMath::applyCameraCompensation(point.position);
}

}  // namespace

bool TargetSelector::select(
    const TableState& table,
    TargetSelection& output,
    std::string& error
) const {
    if (!table.cueBall.detected) {
        error = "尚未偵測到母球。";
        return false;
    }

    output = TargetSelection();
    output.cueBall = compensated(table.cueBall);

    int targetIndex = -1;
    for (std::size_t index = 0; index < table.objectBalls.size(); ++index) {
        if (table.objectBalls[index].detected) {
            targetIndex = static_cast<int>(index);
            break;
        }
    }

    if (targetIndex < 0) {
        error = "沒有偵測到任何目標球，拒絕建立手臂路徑。";
        return false;
    }

    output.targetBallNumber = targetIndex + 1;
    output.targetName = std::to_string(output.targetBallNumber) + "號球";
    output.targetBall = compensated(table.objectBalls[targetIndex]);

    Vector2D cueVector = BilliardMath::getVector(output.cueBall, output.targetBall);
    double minimumAngle = std::numeric_limits<double>::max();

    for (std::size_t index = 0; index < table.pockets.size(); ++index) {
        if (!table.pockets[index].detected) {
            continue;
        }

        Point pocket = compensated(table.pockets[index]);
        Vector2D pocketVector = BilliardMath::getVector(output.targetBall, pocket);
        double angle = BilliardMath::getAngleBetweenVectors(
            cueVector.x, cueVector.y, pocketVector.x, pocketVector.y
        );

        if (angle < minimumAngle) {
            minimumAngle = angle;
            output.pocketNumber = static_cast<int>(index) + 1;
            output.destinationPocket = pocket;
        }
    }

    if (output.pocketNumber < 0) {
        error = "沒有偵測到任何有效球袋。";
        return false;
    }
    output.pocketAngleDeg = minimumAngle;

    // 目前反彈球演算法以 p2、p3 定義上方顆星邊界。
    if (!table.pockets[1].detected || !table.pockets[2].detected) {
        error = "缺少 2 號或 3 號球袋，無法建立反彈邊界。";
        return false;
    }
    output.railA = compensated(table.pockets[1]);
    output.railB = compensated(table.pockets[2]);

    for (std::size_t index = 0; index < table.objectBalls.size(); ++index) {
        if (table.objectBalls[index].detected && static_cast<int>(index) != targetIndex) {
            output.obstacles.push_back(compensated(table.objectBalls[index]));
        }
    }

    error.clear();
    return true;
}
