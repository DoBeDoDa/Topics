#include "rgb_base0/geometry.h"
#include "rgb_base0/calibration_io.h"
#include "rgb_base0/brown_projection.h"
#include "rgb_base0/camera_profile.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
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

std::string readText(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    require(static_cast<bool>(input), "cannot read test fixture: " + path.string());
    std::ostringstream text;
    text << input.rdbuf();
    return text.str();
}

void writeText(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    require(static_cast<bool>(output), "cannot write test fixture: " + path.string());
    output << text;
    require(static_cast<bool>(output), "failed writing test fixture: " + path.string());
}

std::string replaceJsonScalar(std::string text,
                              const std::string& field,
                              const std::string& replacement) {
    const std::string token = "\"" + field + "\"";
    const std::size_t key = text.find(token);
    require(key != std::string::npos, "missing JSON field in test fixture: " + field);
    const std::size_t colon = text.find(':', key + token.size());
    require(colon != std::string::npos, "missing JSON colon in test fixture: " + field);
    std::size_t valueStart = colon + 1;
    while(valueStart < text.size() && text[valueStart] == ' ') {
        ++valueStart;
    }
    const std::size_t valueEnd = text.find_first_of(",\r\n", valueStart);
    require(valueEnd != std::string::npos, "missing JSON value terminator in test fixture: " + field);
    text.replace(valueStart, valueEnd - valueStart, replacement);
    return text;
}

bool calibrationReadRejected(const std::filesystem::path& path) {
    try {
        static_cast<void>(rgb_base0::readCalibration(path));
        return false;
    }
    catch(const std::exception&) {
        return true;
    }
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

        const std::vector<rgb_base0::CameraProfileData> availableProfiles{
            {640, 480, 30, "MJPG"},
            {1280, 720, 10, "MJPG"},
            {1280, 720, 30, "MJPG"},
            {1280, 720, 60, "YUYV"},
        };
        require(rgb_base0::selectRequiredColorProfile(availableProfiles) == 2,
                "highest-FPS 1280x720 MJPG profile was not selected");

        bool missingProfileRejected = false;
        try {
            static_cast<void>(rgb_base0::selectRequiredColorProfile({
                {1280, 720, 60, "YUYV"}, {640, 480, 30, "MJPG"},
            }));
        }
        catch(const std::runtime_error&) {
            missingProfileRejected = true;
        }
        require(missingProfileRejected, "missing required Color profile was not rejected");

        bool zeroDevicesRejected = false;
        try {
            rgb_base0::requireExactlyOneOrbbecDevice(0);
        }
        catch(const std::runtime_error&) {
            zeroDevicesRejected = true;
        }
        require(zeroDevicesRejected, "zero Orbbec devices was not rejected");

        bool multipleDevicesRejected = false;
        try {
            rgb_base0::requireExactlyOneOrbbecDevice(2);
        }
        catch(const std::runtime_error&) {
            multipleDevicesRejected = true;
        }
        require(multipleDevicesRejected, "multiple Orbbec devices were not rejected");

        bool wrongModelRejected = false;
        try {
            rgb_base0::validateGemini2XlIdentity("Orbbec Gemini 2", "SERIAL");
        }
        catch(const std::runtime_error&) {
            wrongModelRejected = true;
        }
        require(wrongModelRejected, "wrong Orbbec model was not rejected");
        rgb_base0::validateGemini2XlIdentity("Orbbec Gemini 2 XL", "SERIAL");

        const rgb_base0::CameraProfileData rgbProfile{1280, 720, 10, "MJPG"};
        const rgb_base0::CameraIntrinsicData rgbIntrinsic{900.0, 905.0, 640.0, 360.0};
        const rgb_base0::CameraDistortionData rgbDistortion{
            0.12, -0.04, 0.008, 0.01, -0.003, 0.0005, 0.0012, -0.0008,
        };
        const rgb_base0::Vec2 sourceNormalized{0.31, -0.22};
        const rgb_base0::Vec2 distortedNormalized =
            rgb_base0::distortBrownRational(sourceNormalized, rgbDistortion);
        const double distortedU = distortedNormalized.x * rgbIntrinsic.fx + rgbIntrinsic.cx;
        const double distortedV = distortedNormalized.y * rgbIntrinsic.fy + rgbIntrinsic.cy;
        const rgb_base0::PixelRayDiagnostics recovered = rgb_base0::inverseProjectBrownPixel(
            distortedU, distortedV, rgbProfile, rgbIntrinsic, rgbDistortion);
        requireNear(recovered.undistortedNormalized.x, sourceNormalized.x, 1e-10,
                    "Brown inverse x");
        requireNear(recovered.undistortedNormalized.y, sourceNormalized.y, 1e-10,
                    "Brown inverse y");
        require(recovered.reprojectionErrorPx <= rgb_base0::kBrownReprojectionTolerancePx,
                "Brown round-trip exceeded the pixel gate");
        requireNear(rgb_base0::norm(recovered.unitRayRgb), 1.0, 1e-12,
                    "Brown RGB ray norm");
        require(recovered.unitRayRgb.z > 0.0, "Brown RGB ray did not point forward");

        const rgb_base0::PixelRayDiagnostics center = rgb_base0::inverseProjectBrownPixel(
            rgbIntrinsic.cx, rgbIntrinsic.cy, rgbProfile, rgbIntrinsic, rgbDistortion);
        requireNear(center.unitRayRgb.x, 0.0, 1e-12, "center ray x");
        requireNear(center.unitRayRgb.y, 0.0, 1e-12, "center ray y");
        requireNear(center.unitRayRgb.z, 1.0, 1e-12, "center ray z");

        for(const rgb_base0::Vec2 pixel : std::vector<rgb_base0::Vec2>{
                {0.0, 0.0}, {1279.0, 0.0}, {0.0, 719.0}, {1279.0, 719.0}, {513.25, 287.75}}) {
            const rgb_base0::PixelRayDiagnostics boundary = rgb_base0::inverseProjectBrownPixel(
                pixel.x, pixel.y, rgbProfile, rgbIntrinsic, rgbDistortion);
            require(boundary.reprojectionErrorPx <= rgb_base0::kBrownReprojectionTolerancePx,
                    "Brown boundary/subpixel round-trip failed");
        }

        const rgb_base0::CameraDistortionData strongDistortion{
            0.45, 0.08, 0.01, 0.03, 0.01, 0.001, 0.006, -0.004,
        };
        const rgb_base0::Vec2 strongSource{0.48, 0.31};
        const rgb_base0::Vec2 strongTarget =
            rgb_base0::distortBrownRational(strongSource, strongDistortion);
        const rgb_base0::PixelRayDiagnostics strongRecovered = rgb_base0::inverseProjectBrownPixel(
            strongTarget.x * rgbIntrinsic.fx + rgbIntrinsic.cx,
            strongTarget.y * rgbIntrinsic.fy + rgbIntrinsic.cy,
            rgbProfile, rgbIntrinsic, strongDistortion);
        requireNear(strongRecovered.undistortedNormalized.x, strongSource.x, 1e-9,
                    "strong Brown inverse x");
        requireNear(strongRecovered.undistortedNormalized.y, strongSource.y, 1e-9,
                    "strong Brown inverse y");

        bool outOfBoundsRejected = false;
        try {
            static_cast<void>(rgb_base0::inverseProjectBrownPixel(
                1280.0, 100.0, rgbProfile, rgbIntrinsic, rgbDistortion));
        }
        catch(const std::runtime_error&) {
            outOfBoundsRejected = true;
        }
        require(outOfBoundsRejected, "out-of-bounds RGB pixel was not rejected");

        bool denominatorRejected = false;
        try {
            const rgb_base0::CameraProfileData syntheticProfile{10, 10, 1, "MJPG"};
            const rgb_base0::CameraIntrinsicData syntheticIntrinsic{1.0, 1.0, 0.0, 0.0};
            rgb_base0::CameraDistortionData singularDistortion{};
            singularDistortion.k4 = -1.0;
            static_cast<void>(rgb_base0::inverseProjectBrownPixel(
                1.0, 0.0, syntheticProfile, syntheticIntrinsic, singularDistortion));
        }
        catch(const std::runtime_error&) {
            denominatorRejected = true;
        }
        require(denominatorRejected, "near-zero Brown denominator was not rejected");

        bool nonConvergenceRejected = false;
        try {
            rgb_base0::BrownInverseOptions oneIteration;
            oneIteration.maxIterations = 1;
            oneIteration.convergenceTolerance = 1e-16;
            static_cast<void>(rgb_base0::inverseProjectBrownPixel(
                strongTarget.x * rgbIntrinsic.fx + rgbIntrinsic.cx,
                strongTarget.y * rgbIntrinsic.fy + rgbIntrinsic.cy,
                rgbProfile, rgbIntrinsic, strongDistortion, oneIteration));
        }
        catch(const std::runtime_error&) {
            nonConvergenceRejected = true;
        }
        require(nonConvergenceRejected, "non-convergent Brown solve was not rejected");

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

        const std::string validJson = readText(jsonPath);
        const std::filesystem::path malformedPath = "calibration_malformed_test.json";
        const std::string schemaField = "\"schema_version\": \"1.2\",";
        const std::size_t schemaPosition = validJson.find(schemaField);
        require(schemaPosition != std::string::npos, "schema field missing from generated JSON");
        std::string duplicateJson = validJson;
        duplicateJson.insert(schemaPosition + schemaField.size(), "\n  " + schemaField);
        writeText(malformedPath, duplicateJson);
        require(calibrationReadRejected(malformedPath), "duplicate calibration field was not rejected");

        std::string missingJson = validJson;
        const std::string inverseKey = "\"inverse_projection_version\"";
        const std::size_t inversePosition = missingJson.find(inverseKey);
        require(inversePosition != std::string::npos, "inverse version missing from generated JSON");
        missingJson.replace(inversePosition, inverseKey.size(), "\"removed_inverse_projection_version\"");
        writeText(malformedPath, missingJson);
        require(calibrationReadRejected(malformedPath), "missing calibration field was not rejected");

        writeText(malformedPath, replaceJsonScalar(validJson, "inverse_max_iterations", "\"50\""));
        require(calibrationReadRejected(malformedPath), "wrong calibration field type was not rejected");

        writeText(malformedPath, replaceJsonScalar(validJson, "fx", "nan"));
        require(calibrationReadRejected(malformedPath), "non-finite calibration value was not rejected");

        std::filesystem::remove(jsonPath);
        std::filesystem::remove(yamlPath);
        std::filesystem::remove(malformedPath);

        rgb_base0::CalibrationData oldSchema = calibration;
        oldSchema.schemaVersion = "1.1";
        bool oldSchemaRejected = false;
        try {
            rgb_base0::validateCalibration(oldSchema);
        }
        catch(const std::runtime_error&) {
            oldSchemaRejected = true;
        }
        require(oldSchemaRejected, "obsolete depth-assisted schema was not rejected");

        rgb_base0::CalibrationData motionAuthorized = calibration;
        motionAuthorized.authorizedForRobotMotion = true;
        bool motionAuthorizationRejected = false;
        try {
            rgb_base0::validateCalibration(motionAuthorized);
        }
        catch(const std::runtime_error&) {
            motionAuthorizationRejected = true;
        }
        require(motionAuthorizationRejected, "RGB-only calibration incorrectly authorized robot motion");

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

        std::cout << "PASS: RGB profile gates, Brown inverse projection, geometry, stability, and calibration IO\n";
        return 0;
    }
    catch(const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
