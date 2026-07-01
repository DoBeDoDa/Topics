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

struct Offset3D {
    double x;
    double y;
    double z;
};
