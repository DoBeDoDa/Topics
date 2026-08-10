#pragma once

#include "rgb_base0/types.h"

#include <string>

namespace rgb_base0 {

double norm(const Vec3& value);
Vec3 normalize(const Vec3& value);
Vec3 multiply(const Mat3& matrix, const Vec3& value);
Mat3 multiply(const Mat3& left, const Mat3& right);
Mat3 rotationBase0FromTool2Zyx(double aDegrees, double bDegrees, double cDegrees);
Mat3 transpose(const Mat3& matrix);
RotationDiagnostics rotationDiagnostics(const Mat3& matrix);
void validateRotationMatrix(const Mat3& matrix, const std::string& label, double tolerance = 1e-9);
double shortestWrappedDifferenceDegrees(double lhs, double rhs);
double wrappedSpreadDegrees(const std::vector<double>& values);
double linearSpread(const std::vector<double>& values);
RobotPose meanRobotPose(const std::vector<RobotPose>& samples);
bool robotPoseStable(const std::vector<RobotPose>& samples,
                     double xyzToleranceMm,
                     double abcToleranceDeg,
                     std::string* reason);
PlaneIntersection intersectRayWithHorizontalPlane(const Vec3& originBase0,
                                                  const Vec3& unitRayBase0,
                                                  double targetZMm);
void validateCalibration(const CalibrationData& calibration);

}  // namespace rgb_base0
