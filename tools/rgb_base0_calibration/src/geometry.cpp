#include "rgb_base0/geometry.h"

#include "rgb_base0/brown_projection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace rgb_base0 {
namespace {

constexpr double kPi = 3.1415926535897932384626433832795;

double radians(const double degrees) {
    return degrees * kPi / 180.0;
}

bool finite(const double value) {
    return std::isfinite(value) != 0;
}

double circularMeanDegrees(const std::vector<double>& values) {
    if(values.empty()) {
        throw std::invalid_argument("Cannot calculate circular mean of an empty list");
    }
    double sumSin = 0.0;
    double sumCos = 0.0;
    for(const double value : values) {
        sumSin += std::sin(radians(value));
        sumCos += std::cos(radians(value));
    }
    return std::atan2(sumSin, sumCos) * 180.0 / kPi;
}

}  // namespace

double norm(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 normalize(const Vec3& value) {
    const double length = norm(value);
    if(!finite(length) || length <= 1e-12) {
        throw std::runtime_error("Cannot normalize a non-finite or zero-length vector");
    }
    return {value.x / length, value.y / length, value.z / length};
}

Vec3 multiply(const Mat3& matrix, const Vec3& value) {
    return {
        matrix[0][0] * value.x + matrix[0][1] * value.y + matrix[0][2] * value.z,
        matrix[1][0] * value.x + matrix[1][1] * value.y + matrix[1][2] * value.z,
        matrix[2][0] * value.x + matrix[2][1] * value.y + matrix[2][2] * value.z,
    };
}

Mat3 multiply(const Mat3& left, const Mat3& right) {
    Mat3 result{};
    for(std::size_t row = 0; row < 3; ++row) {
        for(std::size_t column = 0; column < 3; ++column) {
            for(std::size_t inner = 0; inner < 3; ++inner) {
                result[row][column] += left[row][inner] * right[inner][column];
            }
        }
    }
    return result;
}

Mat3 rotationBase0FromTool3Zyx(const double aDegrees,
                               const double bDegrees,
                               const double cDegrees) {
    const double a = radians(aDegrees);
    const double b = radians(bDegrees);
    const double c = radians(cDegrees);
    const Mat3 rx{{{{1.0, 0.0, 0.0}},
                   {{0.0, std::cos(a), -std::sin(a)}},
                   {{0.0, std::sin(a), std::cos(a)}}}};
    const Mat3 ry{{{{std::cos(b), 0.0, std::sin(b)}},
                   {{0.0, 1.0, 0.0}},
                   {{-std::sin(b), 0.0, std::cos(b)}}}};
    const Mat3 rz{{{{std::cos(c), -std::sin(c), 0.0}},
                   {{std::sin(c), std::cos(c), 0.0}},
                   {{0.0, 0.0, 1.0}}}};
    return multiply(multiply(rz, ry), rx);
}

Mat3 transpose(const Mat3& matrix) {
    Mat3 result{};
    for(std::size_t row = 0; row < 3; ++row) {
        for(std::size_t column = 0; column < 3; ++column) {
            result[row][column] = matrix[column][row];
        }
    }
    return result;
}

RotationDiagnostics rotationDiagnostics(const Mat3& matrix) {
    RotationDiagnostics result;
    result.rtR = multiply(transpose(matrix), matrix);
    double squaredError = 0.0;
    for(std::size_t row = 0; row < 3; ++row) {
        for(std::size_t column = 0; column < 3; ++column) {
            const double expected = row == column ? 1.0 : 0.0;
            const double difference = result.rtR[row][column] - expected;
            squaredError += difference * difference;
        }
    }
    result.orthogonalityError = std::sqrt(squaredError);
    result.determinant =
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    return result;
}

void validateRotationMatrix(const Mat3& matrix, const std::string& label, const double tolerance) {
    for(const auto& row : matrix) {
        for(const double value : row) {
            if(!finite(value)) {
                throw std::runtime_error(label + " contains a non-finite value");
            }
        }
    }
    const RotationDiagnostics diagnostics = rotationDiagnostics(matrix);
    if(!finite(diagnostics.orthogonalityError) || !finite(diagnostics.determinant)
       || diagnostics.orthogonalityError > tolerance || std::abs(diagnostics.determinant - 1.0) > tolerance) {
        std::ostringstream message;
        message << label << " is not a valid rotation matrix: orthogonality_error="
                << diagnostics.orthogonalityError << " determinant=" << diagnostics.determinant;
        throw std::runtime_error(message.str());
    }
}

double shortestWrappedDifferenceDegrees(const double lhs, const double rhs) {
    return std::remainder(lhs - rhs, 360.0);
}

double wrappedSpreadDegrees(const std::vector<double>& values) {
    if(values.empty()) {
        throw std::invalid_argument("Cannot calculate spread of an empty list");
    }
    double best = std::numeric_limits<double>::infinity();
    for(const double reference : values) {
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = -std::numeric_limits<double>::infinity();
        for(const double value : values) {
            const double unwrapped = reference + shortestWrappedDifferenceDegrees(value, reference);
            minimum = std::min(minimum, unwrapped);
            maximum = std::max(maximum, unwrapped);
        }
        best = std::min(best, maximum - minimum);
    }
    return best;
}

double linearSpread(const std::vector<double>& values) {
    if(values.empty()) {
        throw std::invalid_argument("Cannot calculate spread of an empty list");
    }
    const auto limits = std::minmax_element(values.begin(), values.end());
    return *limits.second - *limits.first;
}

RobotPose meanRobotPose(const std::vector<RobotPose>& samples) {
    if(samples.empty()) {
        throw std::invalid_argument("Cannot calculate mean robot pose from zero samples");
    }
    RobotPose mean;
    std::vector<double> a;
    std::vector<double> b;
    std::vector<double> c;
    for(const RobotPose& sample : samples) {
        mean.x += sample.x;
        mean.y += sample.y;
        mean.z += sample.z;
        a.push_back(sample.a);
        b.push_back(sample.b);
        c.push_back(sample.c);
    }
    const double count = static_cast<double>(samples.size());
    mean.x /= count;
    mean.y /= count;
    mean.z /= count;
    mean.a = circularMeanDegrees(a);
    mean.b = circularMeanDegrees(b);
    mean.c = circularMeanDegrees(c);
    return mean;
}

bool robotPoseStable(const std::vector<RobotPose>& samples,
                     const double xyzToleranceMm,
                     const double abcToleranceDeg,
                     std::string* reason) {
    if(samples.size() < 2) {
        if(reason != nullptr) {
            *reason = "At least two robot pose samples are required";
        }
        return false;
    }
    std::array<std::vector<double>, 6> axes;
    for(const RobotPose& sample : samples) {
        const std::array<double, 6> values{sample.x, sample.y, sample.z, sample.a, sample.b, sample.c};
        for(std::size_t index = 0; index < values.size(); ++index) {
            if(!finite(values[index])) {
                if(reason != nullptr) {
                    *reason = "Robot pose contains a non-finite value";
                }
                return false;
            }
            axes[index].push_back(values[index]);
        }
    }
    const char* labels[] = {"X", "Y", "Z", "A", "B", "C"};
    for(std::size_t index = 0; index < 3; ++index) {
        const double spread = linearSpread(axes[index]);
        if(spread > xyzToleranceMm) {
            if(reason != nullptr) {
                std::ostringstream message;
                message << labels[index] << " spread " << spread << " mm exceeds " << xyzToleranceMm << " mm";
                *reason = message.str();
            }
            return false;
        }
    }
    for(std::size_t index = 3; index < 6; ++index) {
        const double spread = wrappedSpreadDegrees(axes[index]);
        if(spread > abcToleranceDeg) {
            if(reason != nullptr) {
                std::ostringstream message;
                message << labels[index] << " wrapped spread " << spread << " deg exceeds " << abcToleranceDeg << " deg";
                *reason = message.str();
            }
            return false;
        }
    }
    if(reason != nullptr) {
        reason->clear();
    }
    return true;
}

PlaneIntersection intersectRayWithHorizontalPlane(const Vec3& originBase0,
                                                  const Vec3& unitRayBase0,
                                                  const double targetZMm) {
    if(!finite(originBase0.x) || !finite(originBase0.y) || !finite(originBase0.z) || !finite(targetZMm)) {
        throw std::runtime_error("Ray origin or target plane contains a non-finite value");
    }
    const Vec3 direction = normalize(unitRayBase0);
    if(std::abs(direction.z) <= 1e-9) {
        throw std::runtime_error("Ray is parallel or nearly parallel to the requested horizontal plane");
    }
    const double lambda = (targetZMm - originBase0.z) / direction.z;
    if(!finite(lambda) || lambda <= 0.0) {
        throw std::runtime_error("Plane intersection lies behind the RGB optical center");
    }
    const Vec3 point{originBase0.x + lambda * direction.x,
                     originBase0.y + lambda * direction.y,
                     originBase0.z + lambda * direction.z};
    if(!finite(point.x) || !finite(point.y) || !finite(point.z)
       || std::abs(point.z - targetZMm) > 1e-7) {
        throw std::runtime_error("Final plane intersection is non-finite or fails the target-Z consistency check");
    }

    // Base0 XY ground-truth radial compensation, applied only after the RGB ray / Z-target intersection.
    // Xc/Yc define the Base0-mm center; w1/w2 independently scale X/Y by radial distance in mm. Z is unchanged.
    constexpr double Xc = 0.285;
    constexpr double Yc = 565.348;
    constexpr double w1 = 0.00004228;
    constexpr double w2 = 0.00004228;
    const double dx = point.x - Xc;
    const double dy = point.y - Yc;
    const double r = std::sqrt(dx * dx + dy * dy);
    const Vec3 correctedPoint{Xc + dx * (1.0 + r * w1),
                              Yc + dy * (1.0 + r * w2),
                              point.z};
    return {originBase0, direction, targetZMm, lambda, correctedPoint};
}

PlaneIntersection projectRgbPixelToBallCenterPlane(const CalibrationData& calibration,
                                                   const double u,
                                                   const double v) {
    validateCalibration(calibration);
    const PixelRayDiagnostics ray = inverseProjectBrownPixel(
        u, v, calibration.profile, calibration.intrinsic, calibration.distortion,
        {calibration.inverseMaxIterations,
         calibration.inverseConvergenceTolerance,
         calibration.inverseReprojectionTolerancePx});
    const Vec3 rayBase0 = multiply(calibration.rBase0FromRgb, ray.unitRayRgb);
    const double targetZMm = calibration.zTableMm + calibration.ballRadiusMm;
    return intersectRayWithHorizontalPlane(calibration.tBase0FromRgb, rayBase0, targetZMm);
}

void validateCalibration(const CalibrationData& calibration) {
    if(calibration.schemaVersion != "1.3") {
        throw std::runtime_error("Unsupported calibration schema_version: " + calibration.schemaVersion);
    }
    if(calibration.createdUtc.empty()) {
        throw std::runtime_error("Calibration timestamp is missing");
    }
    if(calibration.sdkVersion != "1.10.18") {
        throw std::runtime_error("Calibration requires exact Orbbec SDK version 1.10.18");
    }
    if(calibration.cameraModel != "Orbbec Gemini 2 XL" || calibration.deviceName.empty()
       || calibration.serialNumber.empty()) {
        throw std::runtime_error("Calibration camera identity is missing or unsupported");
    }
    if(!calibration.experimental) {
        throw std::runtime_error("This schema only permits experimental calibration data");
    }
    if(calibration.authorizedForRobotMotion) {
        throw std::runtime_error("RGB-only calibration data cannot authorize robot motion");
    }
    if(calibration.profile.width != 1280 || calibration.profile.height != 720 || calibration.profile.format != "MJPG") {
        throw std::runtime_error("Calibration profile must be exactly 1280x720 MJPG");
    }
    if(calibration.profile.fps <= 0 || calibration.profile.fps > 120) {
        throw std::runtime_error("Calibration FPS is invalid");
    }
    if(!finite(calibration.intrinsic.fx) || !finite(calibration.intrinsic.fy)
       || !finite(calibration.intrinsic.cx) || !finite(calibration.intrinsic.cy)
       || calibration.intrinsic.fx <= 0.0 || calibration.intrinsic.fy <= 0.0
       || calibration.intrinsic.cx < 0.0 || calibration.intrinsic.cx >= calibration.profile.width
       || calibration.intrinsic.cy < 0.0 || calibration.intrinsic.cy >= calibration.profile.height) {
        throw std::runtime_error("Calibration focal lengths must be positive");
    }
    const std::array<double, 8> distortion{
        calibration.distortion.k1, calibration.distortion.k2, calibration.distortion.k3,
        calibration.distortion.k4, calibration.distortion.k5, calibration.distortion.k6,
        calibration.distortion.p1, calibration.distortion.p2,
    };
    if(!std::all_of(distortion.begin(), distortion.end(), finite)) {
        throw std::runtime_error("Calibration distortion contains a non-finite value");
    }
    const bool factoryOrbbecMetadata =
        calibration.distortionFamily == "orbbec_brown"
        && calibration.distortionVariant == "not_exposed_by_profile_api"
        && calibration.distortionModelAssumption ==
               "engineering_assumption_pending_ground_truth_validation";
    const bool manualOfflineMetadata =
        calibration.distortionFamily == "brown_rational"
        && calibration.distortionVariant == "manual_offline_chessboard_5_parameter"
        && calibration.distortionModelAssumption == "manual_offline_chessboard_calibration";
    if((!factoryOrbbecMetadata && !manualOfflineMetadata)
       || calibration.distortionHandling != "versioned_iterative_brown_inverse"
       || calibration.distortionCoefficientMapping !=
              "radial_numerator=k1,k2,k3; radial_denominator=k4,k5,k6; tangential=p1,p2"
       || calibration.inverseProjectionVersion != "rgb_brown_rational_v1"
       || calibration.inverseMaxIterations != kBrownInverseMaxIterations
       || !finite(calibration.inverseConvergenceTolerance)
       || !finite(calibration.inverseReprojectionTolerancePx)
       || std::abs(calibration.inverseConvergenceTolerance
                   - kBrownInverseConvergenceTolerance) > 1e-18
       || std::abs(calibration.inverseReprojectionTolerancePx
                   - kBrownReprojectionTolerancePx) > 1e-12) {
        throw std::runtime_error("Calibration distortion metadata is unsupported");
    }
    if(calibration.opticalAxes != "+X image-right, +Y image-down, +Z forward") {
        throw std::runtime_error("Calibration RGB optical-axis declaration is unsupported");
    }
    if(calibration.cameraFrameName != "gemini2xl_rgb_optical_frame"
       || calibration.baseFrameName != "Base0") {
        throw std::runtime_error("Calibration frame name is unsupported");
    }
    if(calibration.toolNumber != 3 || calibration.baseNumber != 0) {
        throw std::runtime_error("Calibration must have been captured using Tool3 and Base0");
    }
    if(calibration.robotModel != "HIWIN RA605-GC" || calibration.robotIp.empty()
       || calibration.robotPoseSampleCount < 2 || calibration.robotPoseSampleWindowMs < 0
       || calibration.xyzSpreadToleranceMm <= 0.0 || calibration.abcSpreadToleranceDeg <= 0.0) {
        throw std::runtime_error("Calibration robot identity or stability settings are invalid");
    }
    if(calibration.rotationConventionSource != "user_approved_temporary") {
        throw std::runtime_error("Unexpected rotation convention source");
    }
    const std::string expectedConvention =
        "degrees; active column-vector rotation; A=X roll, B=Y pitch, C=Z yaw; R=Rz(C)*Ry(B)*Rx(A)";
    if(calibration.rotationConvention != expectedConvention) {
        throw std::runtime_error("Unexpected temporary rotation convention declaration");
    }
    if(calibration.tool3AngleUnit != "degree") {
        throw std::runtime_error("Tool3 angle unit must be degree");
    }
    const std::array<double, 6> pose{
        calibration.robotPose.x, calibration.robotPose.y, calibration.robotPose.z,
        calibration.robotPose.a, calibration.robotPose.b, calibration.robotPose.c,
    };
    if(!std::all_of(pose.begin(), pose.end(), finite)) {
        throw std::runtime_error("Robot pose contains a non-finite value");
    }
    if(!finite(calibration.zTableMm) || !finite(calibration.ballDiameterMm)
       || !finite(calibration.ballRadiusMm)
       || std::abs(calibration.ballDiameterMm - kBallDiameterMm) > 1e-9
       || std::abs(calibration.ballRadiusMm - kBallRadiusMm) > 1e-9
       || std::abs(calibration.ballDiameterMm - 2.0 * calibration.ballRadiusMm) > 1e-9) {
        throw std::runtime_error("Table or ball geometry is invalid");
    }
    validateRotationMatrix(calibration.rBase0FromTool3, "R_Base0_from_Tool3");
    validateRotationMatrix(calibration.rTool3FromRgb, "R_Tool3_from_RGB");
    validateRotationMatrix(calibration.rBase0FromRgb, "R_Base0_from_RGB");
    if(!finite(calibration.tBase0FromRgb.x) || !finite(calibration.tBase0FromRgb.y)
       || !finite(calibration.tBase0FromRgb.z)) {
        throw std::runtime_error("Translation contains a non-finite value");
    }
    const Mat3 expectedRotation = rotationBase0FromTool3Zyx(
        calibration.robotPose.a, calibration.robotPose.b, calibration.robotPose.c);
    for(std::size_t row = 0; row < 3; ++row) {
        for(std::size_t column = 0; column < 3; ++column) {
            if(std::abs(calibration.rBase0FromTool3[row][column] - expectedRotation[row][column]) > 1e-9) {
                throw std::runtime_error("R_Base0_from_Tool3 does not match the declared temporary Z-Y-X rule");
            }
        }
    }
    const Mat3 expectedFinalRotation = multiply(calibration.rBase0FromTool3, calibration.rTool3FromRgb);
    for(std::size_t row = 0; row < 3; ++row) {
        for(std::size_t column = 0; column < 3; ++column) {
            if(std::abs(calibration.rBase0FromRgb[row][column] - expectedFinalRotation[row][column]) > 1e-9) {
                throw std::runtime_error("R_Base0_from_RGB does not equal R_Base0_from_Tool3 * R_Tool3_from_RGB");
            }
        }
    }
    if(!finite(calibration.tTool3ToRgb.x) || !finite(calibration.tTool3ToRgb.y)
       || !finite(calibration.tTool3ToRgb.z)) {
        throw std::runtime_error("t_Tool3_to_RGB contains a non-finite value");
    }
    const Vec3 baseOffset = multiply(calibration.rBase0FromTool3, calibration.tTool3ToRgb);
    if(std::abs(calibration.tBase0FromRgb.x - (calibration.robotPose.x + baseOffset.x)) > 1e-9
       || std::abs(calibration.tBase0FromRgb.y - (calibration.robotPose.y + baseOffset.y)) > 1e-9
       || std::abs(calibration.tBase0FromRgb.z - (calibration.robotPose.z + baseOffset.z)) > 1e-9) {
        throw std::runtime_error("C_Base0 is inconsistent with Tool3 pose and t_Tool3_to_RGB");
    }
    if(calibration.tablePlaneModel != "constant_z" || calibration.translationUnit != "mm") {
        throw std::runtime_error("Only constant_z table plane with millimeter translation is supported");
    }
}

}  // namespace rgb_base0
