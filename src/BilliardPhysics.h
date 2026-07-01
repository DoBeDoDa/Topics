#pragma once
#include "Point.h"

class BilliardPhysics {
public:
    // 檢查從起點到終點的路徑是否被障礙物阻擋
    static bool isPathBlocked(Point start, Point end, Point obs, double ball_d);

    // 計算垂直於牆面的預備點位（用於微調或撞擊準備）
    static Point getPerpendicularTarget(Point base, Point edgeA, Point edgeB, double backward_dist);

    // 依據目標袋口與子球位置，計算母球碰撞子球所需的「虛擬碰撞點」（Ghost Ball）
    static Point getGhostBall(Point destination, Point target_ball, double ball_d);

    // 計算相對於兩袋口連線牆面的鏡射點（用於顆星路徑規劃）
    static Point getSlantedBankTarget(Point I, Point p1, Point p2);

    // 計算兩線段之交點
    static bool getIntersection(Point ray_start, Point ray_target, Point segA, Point segB, Point &out_intersect);
};
