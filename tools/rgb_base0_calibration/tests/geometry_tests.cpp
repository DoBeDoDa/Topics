#include "rgb_base0/geometry.h"
#include "rgb_base0/calibration_io.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(const bool condition, const std::string& message) {
    if(!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(const double actual, const double expected, const double tolerance, const std::string& label) {
    require(std::abs(actual - expected) <= tolerance,
            label + ": actual=" + std::to_string(actual) + " expected=" + std::to_string(expected));
}

}  // namespace

int main() {
    try {
        const rgb_base0::Mat3 identity = rgb_base0::rotationBase0FromTool2Zyx(0.0, 0.0, 0.0);
        const rgb_base0::Vec3 value = rgb_base0::multiply(identity, rgb_base0::Vec3{1.0, 2.0, 3.0});
        requireNear(value.x, 1.0, 1e-12, "identity x");
        requireNear(value.y, 2.0, 1e-12, "identity y");
        requireNear(value.z, 3.0, 1e-12, "identity z");

        const rgb_base0::Vec3 yaw90 = rgb_base0::multiply(
            rgb_base0::rotationBase0FromTool2Zyx(0.0, 0.0, 90.0), rgb_base0::Vec3{1.0, 0.0, 0.0});
        requireNear(yaw90.x, 0.0, 1e-12, "yaw x");
        requireNear(yaw90.y, 1.0, 1e-12, "yaw y");

        const rgb_base0::Vec3 pitch90 = rgb_base0::multiply(
            rgb_base0::rotationBase0FromTool2Zyx(0.0, 90.0, 0.0), rgb_base0::Vec3{0.0, 0.0, 1.0});
        requireNear(pitch90.x, 1.0, 1e-12, "pitch x");
        requireNear(pitch90.z, 0.0, 1e-12, "pitch z");

        requireNear(rgb_base0::shortestWrappedDifferenceDegrees(-179.0, 179.0), 2.0, 1e-12,
                    "wrapped difference");
        requireNear(rgb_base0::wrappedSpreadDegrees({179.98, -179.99, 180.0}), 0.03, 1e-9,
                    "wrapped spread");

        const std::vector<rgb_base0::RobotPose> stable{
            {10.00, 20.00, 30.00, 179.99, 0.00, -179.99},
            {10.04, 19.96, 30.01, -179.99, 0.02, 180.00},
            {10.02, 20.01, 29.98, 180.00, -0.01, 179.99},
        };
        std::string reason;
        require(rgb_base0::robotPoseStable(stable, 0.1, 0.05, &reason), "stable pose rejected: " + reason);
        const rgb_base0::RobotPose mean = rgb_base0::meanRobotPose(stable);
        requireNear(mean.x, 10.02, 1e-12, "mean x");
        require(std::abs(std::abs(mean.a) - 180.0) < 0.02, "circular mean failed near wrap");

        std::vector<rgb_base0::RobotPose> unstable = stable;
        unstable[2].x = 10.2;
        require(!rgb_base0::robotPoseStable(unstable, 0.1, 0.05, &reason), "unstable pose accepted");

        const rgb_base0::PlaneIntersection hit = rgb_base0::intersectRayWithHorizontalPlane(
            {100.0, 200.0, 500.0}, {0.0, 0.0, -2.0}, rgb_base0::kBallRadiusMm);
        requireNear(hit.lambdaMm, 477.75, 1e-9, "plane lambda");
        requireNear(hit.pointBase0.z, 22.25, 1e-9, "plane z");

        bool behindRejected = false;
        try {
            static_cast<void>(rgb_base0::intersectRayWithHorizontalPlane(
                {0.0, 0.0, 500.0}, {0.0, 0.0, 1.0}, rgb_base0::kBallRadiusMm));
        }
        catch(const std::runtime_error&) {
            behindRejected = true;
        }
        require(behindRejected, "intersection behind camera was not rejected");

        rgb_base0::CalibrationData calibration;
        calibration.createdUtc = "2026-08-10T00:00:00Z";
        calibration.deviceName = "Orbbec Gemini 2 XL";
        calibration.serialNumber = "UNIT-TEST-SERIAL";
        calibration.firmwareVersion = "test";
        calibration.profile = {1280, 720, 20, "MJPG"};
        calibration.intrinsic = {900.1, 901.2, 640.3, 360.4};
        calibration.distortion = {0.1, -0.2, 0.003, 0.004, 0.005, 0.006, 0.007, -0.008};
        calibration.robotPose = mean;
        calibration.rBase0FromTool2 = rgb_base0::rotationBase0FromTool2Zyx(mean.a, mean.b, mean.c);
        calibration.rTool2FromRgb = rgb_base0::rotationBase0FromTool2Zyx(0.0, 0.0, 1.0);
        calibration.rBase0FromRgb = rgb_base0::multiply(calibration.rBase0FromTool2, calibration.rTool2FromRgb);
        calibration.tTool2ToRgb = {1.0, 2.0, 3.0};
        const rgb_base0::Vec3 testOffset = rgb_base0::multiply(calibration.rBase0FromTool2, calibration.tTool2ToRgb);
        calibration.tBase0FromRgb = {mean.x + testOffset.x, mean.y + testOffset.y, mean.z + testOffset.z};
        calibration.zTableMm = 12.34;
        const std::filesystem::path jsonPath = "calibration_roundtrip_test.json";
        const std::filesystem::path yamlPath = "calibration_roundtrip_test.yaml";
        rgb_base0::writeCalibrationJson(calibration, jsonPath);
        rgb_base0::writeCalibrationYaml(calibration, yamlPath);
        const rgb_base0::CalibrationData json = rgb_base0::readCalibration(jsonPath);
        const rgb_base0::CalibrationData yaml = rgb_base0::readCalibration(yamlPath);
        require(rgb_base0::equivalentCalibration(calibration, json), "JSON round-trip mismatch");
        require(rgb_base0::equivalentCalibration(calibration, yaml), "YAML round-trip mismatch");
        require(rgb_base0::equivalentCalibration(json, yaml), "JSON/YAML semantic mismatch");
        std::filesystem::remove(jsonPath);
        std::filesystem::remove(yamlPath);

        rgb_base0::CalibrationData inconsistent = calibration;
        inconsistent.rBase0FromRgb[0][0] += 0.01;
        bool inconsistentRejected = false;
        try {
            rgb_base0::validateCalibration(inconsistent);
        }
        catch(const std::runtime_error&) {
            inconsistentRejected = true;
        }
        require(inconsistentRejected, "inconsistent pose/rotation calibration was not rejected");

        rgb_base0::CalibrationData wrongDiameter = calibration;
        wrongDiameter.ballDiameterMm = 49.52;
        bool wrongDiameterRejected = false;
        try {
            rgb_base0::validateCalibration(wrongDiameter);
        }
        catch(const std::runtime_error&) {
            wrongDiameterRejected = true;
        }
        require(wrongDiameterRejected, "obsolete ball diameter was not rejected");

        rgb_base0::CalibrationData inconsistentRadius = calibration;
        inconsistentRadius.ballRadiusMm = 24.76;
        bool inconsistentRadiusRejected = false;
        try {
            rgb_base0::validateCalibration(inconsistentRadius);
        }
        catch(const std::runtime_error&) {
            inconsistentRadiusRejected = true;
        }
        require(inconsistentRadiusRejected, "diameter/radius mismatch was not rejected");

        std::cout << "PASS: geometry, wrapped angles, stability, fail-closed plane intersection, and calibration IO\n";
        return 0;
    }
    catch(const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
