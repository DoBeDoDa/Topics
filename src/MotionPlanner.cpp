// 將瞄準方向轉換成中繼關節點、預備姿態與正式擊球姿態。
#include "MotionPlanner.h"

#include "MathUtils.h"

bool MotionPlanner::createPlan(
    Point cueBall,
    Point aimTarget,
    const BilliardConfig::MotionProfile& profile,
    MotionPlan& output,
    std::string& error
) const {
    Vector2D direction = BilliardMath::getVector(cueBall, aimTarget);
    double distance = BilliardMath::getLength(direction.x, direction.y);
    if (distance < BilliardConfig::MIN_AIM_DISTANCE_MM) {
        error = "母球與瞄準點距離過短，無法建立安全移動路徑。";
        return false;
    }

    double armRz = BilliardMath::getVectorAngle(direction.x, direction.y) +
        BilliardConfig::YAW_OFFSET_DEG;
    double standoff = (BilliardConfig::BALL_DIAMETER_MM / 2.0) +
        profile.standoffExtraMm;
    double strikeX = cueBall.x - (direction.x / distance) * standoff;
    double strikeY = cueBall.y - (direction.y / distance) * standoff;

    Offset3D offset = BilliardMath::getTiltOffset(
        armRz, profile.tiltRyDeg, profile.moveBackMm
    );

    output.transitJoint = BilliardConfig::TRANSIT_JOINT;
    output.readyPose = {
        strikeX + offset.x,
        strikeY + offset.y,
        profile.safeZ,
        profile.rxDeg,
        profile.tiltRyDeg,
        armRz
    };
    output.strikePose = {
        strikeX + offset.x,
        strikeY + offset.y,
        profile.strikeZ,
        profile.rxDeg,
        profile.tiltRyDeg,
        armRz
    };
    output.aimAngleDeg = armRz;

    error.clear();
    return true;
}
