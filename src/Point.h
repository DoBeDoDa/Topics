// 定義點、球桌邊界與二維向量等基礎幾何型別。
#pragma once
#include <string>

struct Point {
    double x;
    double y;
};

struct Rail {
    Point pA;
    Point pB;
    std::string name;
};

struct Vector2D {
    double x;
    double y;
};

struct AxisAlignedBounds2D {
    double minX;
    double maxX;
    double minY;
    double maxY;
};
