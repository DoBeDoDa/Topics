#pragma once
#include <cmath>

#include "Point.h"

namespace BilliardMath {
    const double PI = 3.14159265358979323846;

    // 相機角度與位置畸變補償函數
    Point applyCameraCompensation(Point raw_pt);

    // 計算兩點間的距離
    double getDistance(double x1, double y1, double x2, double y2);
    double getDistance(Point p1, Point p2);

    // 計算二維向量長度
    double getLength(double dx, double dy);

    // 計算兩個向量之間的夾角（度數）
    double getAngleBetweenVectors(double v1_x, double v1_y, double v2_x, double v2_y);

    // 計算向量相對於 X 軸的角度（度數，等同 RZ，範圍為 -180 到 180）
    double getVectorAngle(double dx, double dy);
}
