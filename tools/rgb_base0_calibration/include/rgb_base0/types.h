#pragma once

#include <array>
#include <string>
#include <vector>

namespace rgb_base0 {

inline constexpr double kBallDiameterMm = 44.5;
inline constexpr double kBallRadiusMm = kBallDiameterMm / 2.0;
inline constexpr int kBrownInverseMaxIterations = 50;
inline constexpr double kBrownInverseConvergenceTolerance = 1e-12;
inline constexpr double kBrownReprojectionTolerancePx = 0.25;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

using Mat3 = std::array<std::array<double, 3>, 3>;

struct RobotPose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
};

struct CameraProfileData {
    int width = 0;
    int height = 0;
    int fps = 0;
    std::string format;
};

struct CameraIntrinsicData {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

struct CameraDistortionData {
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
    double k5 = 0.0;
    double k6 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
};

struct CalibrationData {
    std::string schemaVersion = "1.3";
    std::string createdUtc;
    bool experimental = true;
    bool authorizedForRobotMotion = false;

    std::string sdkVersion = "1.10.18";
    std::string cameraModel = "Orbbec Gemini 2 XL";
    std::string deviceName;
    std::string serialNumber;
    std::string firmwareVersion;
    CameraProfileData profile;
    CameraIntrinsicData intrinsic;
    CameraDistortionData distortion;
    std::string distortionFamily = "orbbec_brown";
    std::string distortionVariant = "not_exposed_by_profile_api";
    std::string distortionHandling = "versioned_iterative_brown_inverse";
    std::string distortionModelAssumption =
        "engineering_assumption_pending_ground_truth_validation";
    std::string distortionCoefficientMapping =
        "radial_numerator=k1,k2,k3; radial_denominator=k4,k5,k6; tangential=p1,p2";
    std::string inverseProjectionVersion = "rgb_brown_rational_v1";
    int inverseMaxIterations = kBrownInverseMaxIterations;
    double inverseConvergenceTolerance = kBrownInverseConvergenceTolerance;
    double inverseReprojectionTolerancePx = kBrownReprojectionTolerancePx;
    std::string opticalAxes = "+X image-right, +Y image-down, +Z forward";
    std::string cameraFrameName = "gemini2xl_rgb_optical_frame";
    std::string baseFrameName = "Base0";

    std::string robotModel = "HIWIN RA605-GC";
    std::string robotIp = "192.168.0.1";
    int toolNumber = 3;
    int baseNumber = 0;
    RobotPose robotPose;
    double xyzSpreadToleranceMm = 0.1;
    double abcSpreadToleranceDeg = 0.05;
    int robotPoseSampleCount = 3;
    int robotPoseSampleWindowMs = 500;

    std::string rotationConventionSource = "user_approved_temporary";
    std::string rotationConvention =
        "degrees; active column-vector rotation; A=X roll, B=Y pitch, C=Z yaw; R=Rz(C)*Ry(B)*Rx(A)";
    std::string tool3AngleUnit = "degree";
    Mat3 rBase0FromTool3{{{{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}}};
    Mat3 rTool3FromRgb{{{{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}}};
    Vec3 tTool3ToRgb;
    Mat3 rBase0FromRgb{{{{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}}};
    Vec3 tBase0FromRgb;

    double zTableMm = 0.0;
    double ballDiameterMm = kBallDiameterMm;
    double ballRadiusMm = kBallRadiusMm;
    std::string tablePlaneModel = "constant_z";
    std::string translationUnit = "mm";
};

struct RotationDiagnostics {
    Mat3 rtR{};
    double orthogonalityError = 0.0;
    double determinant = 0.0;
};

struct PixelRayDiagnostics {
    Vec2 distortedNormalized;
    Vec2 undistortedNormalized;
    Vec2 reprojectedPixel;
    Vec3 unitRayRgb;
    int inverseIterations = 0;
    double finalNormalizedResidual = 0.0;
    double reprojectionErrorPx = 0.0;
};

struct PlaneIntersection {
    Vec3 rayOriginBase0;
    Vec3 unitRayBase0;
    double targetZMm = 0.0;
    double lambdaMm = 0.0;
    Vec3 pointBase0;
};

struct StableDetection {
    int classId = -1;
    std::string className;
    double u = 0.0;
    double v = 0.0;
    double medianRadialDistancePx = 0.0;
    int observationCount = 0;
    int inlierCount = 0;
};

}  // namespace rgb_base0
