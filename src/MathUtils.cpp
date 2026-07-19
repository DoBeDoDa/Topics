#include "MathUtils.h"
#include "Point.h"
#include "BilliardConfig.h"

namespace BilliardMath {
    Point applyCameraCompensation(Point raw_pt) {
        // 如果是無效座標點，不進行補償
        if (raw_pt.x < -9000.0 || raw_pt.y < -9000.0) {
            return raw_pt;
        }

        const double offset_x = BilliardConfig::CAMERA_OFFSET_X_MM;
        const double offset_y = BilliardConfig::CAMERA_OFFSET_Y_MM;

        const double ref_x = BilliardConfig::CAMERA_REFERENCE_X_MM;
        const double ref_y = BilliardConfig::CAMERA_REFERENCE_Y_MM;

        const double k_x = BilliardConfig::CAMERA_COMPENSATION_KX;
        const double k_y = BilliardConfig::CAMERA_COMPENSATION_KY;

        // 1. 計算相對於基準點的偏移值
        double dx = raw_pt.x - ref_x;
        double dy = raw_pt.y - ref_y;

        // 2. 比例計算補償值 (離基準點越近，補償越小；越遠，補償越大)
        double comp_x = k_x * dx;
        double comp_y = k_y * dy;

        Point compensated_pt;
        compensated_pt.x = raw_pt.x + offset_x + comp_x;
        compensated_pt.y = raw_pt.y + offset_y + comp_y;

        return compensated_pt;
    }

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

    Vector2D getVector(Point start, Point end) {
        return { end.x - start.x, end.y - start.y };
    }

    Offset3D getTiltOffset(double arm_rz, double tilt_ry_deg, double move_back_mm) {
        double rad_rz = arm_rz * PI / 180.0;
        double rad_tilt = tilt_ry_deg * PI / 180.0;

        double dx = -move_back_mm * cos(rad_tilt) * cos(rad_rz);
        double dy = -move_back_mm * cos(rad_tilt) * sin(rad_rz);
        double dz =  move_back_mm * sin(rad_tilt);
        return { dx, dy, dz };
    }
}
