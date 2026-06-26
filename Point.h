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
