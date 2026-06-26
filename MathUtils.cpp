#include "MathUtils.h"
#include "Point.h"

namespace BilliardMath {
    double getDistance(double x1, double y1, double x2, double y2) {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    double getDistance(Point p1, Point p2) {
        return getDistance(p1.x, p1.y, p2.x, p2.y);
    }

    double getLength(double dx, double dy) {
        return sqrt(dx * dx + dy * dy);
    }

    double getAngleBetweenVectors(double v1_x, double v1_y, double v2_x, double v2_y) {
        double len1 = getLength(v1_x, v1_y);
        double len2 = getLength(v2_x, v2_y);
        if (len1 < 0.001 || len2 < 0.001) return 0.0;
        
        double cos_theta = (v1_x * v2_x + v1_y * v2_y) / (len1 * len2);
        if (cos_theta > 1.0) cos_theta = 1.0;
        if (cos_theta < -1.0) cos_theta = -1.0;
        return acos(cos_theta) * 180.0 / PI;
    }

    double getVectorAngle(double dx, double dy) {
        return atan2(dy, dx) * 180.0 / PI;
    }
}
