// 實作鬼球點、路徑遮擋、反射點與線段交點等撞球幾何計算。
#include "BilliardPhysics.h"
#include "MathUtils.h"
#include <cmath>
#include <algorithm>

bool BilliardPhysics::isRouteBlocked(Point start, Point end, const std::vector<Point>& obs_list, double ball_d) {
    for (const auto& obs : obs_list) {
        if (isPathBlocked(start, end, obs, ball_d)) {
            return true;
        }
    }
    return false;
}

bool BilliardPhysics::isPathBlocked(Point start, Point end, Point obs, double ball_d) {
    double line_dx = end.x - start.x;
    double line_dy = end.y - start.y;
    double line_len_sq = line_dx * line_dx + line_dy * line_dy;
    if (line_len_sq < 0.001) return false;

    double t = ((obs.x - start.x) * line_dx + (obs.y - start.y) * line_dy) / line_len_sq;
    double dist = 0.0;

    if (t < 0.0) {
        dist = BilliardMath::getDistance(start, obs);
    } else if (t > 1.0) {
        dist = BilliardMath::getDistance(end, obs);
    } else {
        Point proj = { start.x + t * line_dx, start.y + t * line_dy };
        dist = BilliardMath::getDistance(proj, obs);
    }
    return (dist < ball_d); 
}

Point BilliardPhysics::getPerpendicularTarget(Point base, Point edgeA, Point edgeB, double backward_dist) {
    double dx = edgeB.x - edgeA.x;
    double dy = edgeB.y - edgeA.y;
    double len = BilliardMath::getLength(dx, dy);
    if (len < 0.001) return base;
    double ux = dx / len, uy = dy / len;
    return { base.x - uy * backward_dist, base.y + ux * backward_dist };
}

Point BilliardPhysics::getGhostBall(Point destination, Point target_ball, double ball_d) {
    double dx = target_ball.x - destination.x;
    double dy = target_ball.y - destination.y;
    double dist = BilliardMath::getLength(dx, dy);
    Point ghost = target_ball;
    if (dist > 0.001) {
        ghost.x += (dx / dist) * ball_d;
        ghost.y += (dy / dist) * ball_d;
    }
    return ghost;
}

Point BilliardPhysics::getSlantedBankTarget(Point I, Point p1, Point p2) {
    double line_dx = p2.x - p1.x;
    double line_dy = p2.y - p1.y;
    double line_length_sq = line_dx * line_dx + line_dy * line_dy;
    if (line_length_sq < 0.001) return I;

    double t = (((I.x - p1.x) * line_dx) + ((I.y - p1.y) * line_dy)) / line_length_sq;
    Point proj = { p1.x + t * line_dx, p1.y + t * line_dy };
    return { proj.x * 2 - I.x, proj.y * 2 - I.y };
}

bool BilliardPhysics::getIntersection(Point ray_start, Point ray_target, Point segA, Point segB, Point &out_intersect) {
    double den = (ray_start.x - ray_target.x) * (segA.y - segB.y) - (ray_start.y - ray_target.y) * (segA.x - segB.x);
    if (std::abs(den) < 0.001) return false;

    double t_ray = ((ray_start.x - segA.x) * (segA.y - segB.y) - (ray_start.y - segA.y) * (segA.x - segB.x)) / den;
    double u_seg = -((ray_start.x - ray_target.x) * (ray_start.y - segA.y) - (ray_start.y - ray_target.y) * (ray_start.x - segA.x)) / den;

    if (t_ray >= 0.0 && u_seg >= 0.0 && u_seg <= 1.0) {
        out_intersect = { ray_start.x + t_ray * (ray_target.x - ray_start.x), ray_start.y + t_ray * (ray_target.y - ray_start.y) };
        return true;
    }
    return false;
}
