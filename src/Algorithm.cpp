#include "Algorithm.h"
#include <iostream>

using namespace std;

ShotDecision BilliardAlgorithm::decideShot(
    Point bw,
    Point target_arm,
    Point destination,
    Point rail_A,
    Point rail_B,
    const std::vector<Point>& obs_list,
    double BALL_D,
    int best_pocket_idx
) {
    ShotDecision decision;
    decision.direct_path_blocked = false;
    decision.bank_route_safe = false;

    // 1. 直擊假想球與路徑防撞檢測
    Point ghost_direct = BilliardPhysics::getGhostBall(destination, target_arm, BALL_D);
    
    for (const auto& obs : obs_list) {
        if (BilliardPhysics::isPathBlocked(bw, ghost_direct, obs, BALL_D)) {
            decision.direct_path_blocked = true;
        }
        if (BilliardPhysics::isPathBlocked(target_arm, destination, obs, BALL_D)) {
            decision.direct_path_blocked = true;
        }
    }
    if (BilliardPhysics::isPathBlocked(target_arm, destination, bw, BALL_D)) {
        decision.direct_path_blocked = true;
    }

    // 2. 夾角計算
    Vector2D vec1 = BilliardMath::getVector(target_arm, destination);
    Vector2D vec2 = BilliardMath::getVector(bw, target_arm);
    decision.angle_deg = BilliardMath::getAngleBetweenVectors(vec1.x, vec1.y, vec2.x, vec2.y);

    // 3. 顆星集球 / 直線擊球決策
    if (decision.angle_deg > 90.0 || decision.direct_path_blocked) {
        if (decision.direct_path_blocked) {
            cout << "\n[防撞提示] 偵測到直擊路徑受阻，自動切換至顆星解球模式。" << endl;
        }
        
        Point mirrored_pocket = BilliardPhysics::getSlantedBankTarget(destination, rail_A, rail_B);
        Point mirrored_target = BilliardPhysics::getSlantedBankTarget(target_arm, rail_A, rail_B);
        Point ghost_bank = BilliardPhysics::getGhostBall(mirrored_pocket, mirrored_target, BALL_D);

        Point pt_wall;
        if (BilliardPhysics::getIntersection(bw, ghost_bank, rail_A, rail_B, pt_wall)) {
            bool current_bank_blocked = false;

            for (const auto& obs : obs_list) {
                if (BilliardPhysics::isPathBlocked(bw, pt_wall, obs, BALL_D)) {
                    current_bank_blocked = true;
                }
                if (BilliardPhysics::isPathBlocked(pt_wall, target_arm, obs, BALL_D)) {
                    current_bank_blocked = true;
                }
                if (BilliardPhysics::isPathBlocked(target_arm, destination, obs, BALL_D)) {
                    current_bank_blocked = true;
                }
            }
            if (BilliardPhysics::isPathBlocked(target_arm, destination, bw, BALL_D)) {
                current_bank_blocked = true;
            }

            if (!current_bank_blocked) {
                decision.bank_route_safe = true;
            }
        }

        decision.best_aim_target = ghost_bank;
        if (decision.bank_route_safe) {
            decision.strategy_name = "雙鏡射顆星擊球 (洞2-洞3牆壁 -> p" + to_string(best_pocket_idx) + ")";
        } else {
            decision.strategy_name = "雙鏡射顆星擊球 (安全路徑受阻，強制開火洞2-洞3牆壁)";
        }
    } 
    else {
        decision.strategy_name = "直線直擊 (Direct Shot -> p" + to_string(best_pocket_idx) + ")";
        decision.best_aim_target = ghost_direct;
    }

    // 4. 工作半徑檢測與降級
    double MAX_REACH_RADIUS = 850.0;
    double target_reach = BilliardMath::getLength(decision.best_aim_target.x, decision.best_aim_target.y);
    
    if (target_reach > MAX_REACH_RADIUS) {
        cout << "\n[警告] 計算出之擊球點超出工作半徑 (" << target_reach << " > " << MAX_REACH_RADIUS << " mm)！" << endl;
        cout << "[降級] 放棄當前路徑，強制切換為直線直擊策略..." << endl;
        
        decision.best_aim_target = ghost_direct; 
        decision.strategy_name = "[降級] 強制直線直擊";
    }

    return decision;
}
