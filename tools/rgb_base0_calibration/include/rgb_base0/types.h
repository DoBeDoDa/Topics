#pragma once

#include <array>
#include <string>
#include <vector>

namespace rgb_base0 {

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
    std::string schemaVersion = "1.0";
    std::string createdUtc;
    bool experimental = true;

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
    std::string distortionHandling = "orbbec_sdk_inverse_projection";
    std::string opticalAxes = "+X image-right, +Y image-down, +Z forward";
    std::string cameraFrameName = "gemini2xl_rgb_optical_frame";
    std::string baseFrameName = "Base0";

    std::string robotModel = "HIWIN RA605-GC";
    std::string robotIp = "192.168.0.1";
    int toolNumber = 2;
    int baseNumber = 0;
    RobotPose robotPose;
    double xyzSpreadToleranceMm = 0.1;
    double abcSpreadToleranceDeg = 0.05;
    int robotPoseSampleCount = 3;
    int robotPoseSampleWindowMs = 500;

    std::string rotationConventionSource = "user_approved_temporary";
    std::string rotationConvention =
        "degrees; active column-vector rotation; A=X roll, B=Y pitch, C=Z yaw; R=Rz(C)*Ry(B)*Rx(A)";
    std::string tool2AngleUnit = "degree";
    Mat3 rBase0FromTool2{{{{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}}};
    Mat3 rTool2FromRgb{{{{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}}};
    Vec3 tTool2ToRgb;
    Mat3 rBase0FromRgb{{{{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}}};
    Vec3 tBase0FromRgb;

    double zTableMm = 0.0;
    double ballRadiusMm = 24.76;
    std::string tablePlaneModel = "constant_z";
    std::string translationUnit = "mm";
};

struct RotationDiagnostics {
    Mat3 rtR{};
    double orthogonalityError = 0.0;
    double determinant = 0.0;
};

struct PixelRayDiagnostics {
    Vec3 qAt1Mm;
    Vec3 qAt1000Mm;
    Vec3 unitRayRgb;
    double normalizedDirectionDifference = 0.0;
    double scaleResidualMm = 0.0;
    double reprojectionErrorPx = 0.0;
    double xyTableDirectionDifference = 0.0;
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
