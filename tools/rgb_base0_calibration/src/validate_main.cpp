#include "rgb_base0/calibration_io.h"
#include "rgb_base0/geometry.h"
#include "rgb_base0/orbbec_camera.h"
#include "rgb_base0/robot_pose_reader.h"
#include "rgb_base0/runtime_utils.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

struct PixelInput {
    int classId = -1;
    std::string name;
    double u = 0.0;
    double v = 0.0;
    int observationCount = 0;
    int inlierCount = 0;
    double medianRadialDistancePx = 0.0;
    std::string source;
};

struct PointResult {
    PixelInput input;
    rgb_base0::PixelRayDiagnostics ray;
    rgb_base0::PlaneIntersection intersection;
};

struct GroundTruth {
    std::string name;
    double xMm = 0.0;
    double yMm = 0.0;
    double zMm = 0.0;
};

struct GroundTruthError {
    std::string name;
    double u = 0.0;
    double v = 0.0;
    double calculatedX = 0.0;
    double calculatedY = 0.0;
    double truthX = 0.0;
    double truthY = 0.0;
    double truthZ = 0.0;
    double errorX = 0.0;
    double errorY = 0.0;
    double errorZ = 0.0;
    double errorXy = 0.0;
    double error3d = 0.0;
};

struct GroundTruthSummary {
    std::string status = "not_provided";
    std::string reason = "No ground-truth CSV supplied";
    double rmsXyMm = 0.0;
    double maxXyMm = 0.0;
    double corrErrorXWithU = 0.0;
    double corrErrorXWithV = 0.0;
    double corrErrorYWithU = 0.0;
    double corrErrorYWithV = 0.0;
    double correlationLimit = 0.7;
    std::vector<GroundTruthError> errors;
};

struct Options {
    std::filesystem::path calibrationPath;
    std::filesystem::path outputRoot = std::filesystem::path(RGB_BASE0_REPO_ROOT) / "output" / "rgb_base0_validation";
    std::filesystem::path pythonPath = std::filesystem::path(RGB_BASE0_REPO_ROOT) / ".venv" / "Scripts" / "python.exe";
    std::filesystem::path weightsPath = std::filesystem::path(RGB_BASE0_REPO_ROOT) / "bin" / "best.pt";
    std::optional<std::filesystem::path> groundTruthPath;
    std::vector<PixelInput> manualPixels;
    double confidence = 0.3;
    double trendCorrelationLimit = 0.7;
};

void printUsage() {
    std::cout
        << "Standalone RGB pixel -> Robot Base0 validation\n\n"
        << "Usage (automatic YOLO, default):\n"
        << "  rgb_base0_validate --calibration <camera_calibration.json|yaml> [options]\n\n"
        << "Manual pixel mode (repeat as needed):\n"
        << "  rgb_base0_validate --calibration <file> --manual <name,u,v> [--manual <name,u,v>]\n\n"
        << "Options:\n"
        << "  --ground-truth <csv>   Columns: class_name,x_mm,y_mm,z_mm; >=6 matched points required\n"
        << "  --confidence <0..1>    YOLO confidence threshold (default 0.3)\n"
        << "  --output-root <dir>     Timestamped run directory is created below this path\n"
        << "  --python <exe>          Existing Ultralytics environment Python\n"
        << "  --weights <best.pt>     Existing YOLO weights\n"
        << "  --trend-limit <0..1>    Absolute Pearson residual/image correlation gate (default 0.7)\n"
        << "  --help                  Show this text without connecting to hardware\n\n"
        << "The tool reads the stationary Tool3/Base0 pose and the Gemini 2 XL only. It never sends a\n"
        << "robot motion, motor, alarm-clear, or output command. Results are experimental diagnostics.\n";
}

std::vector<std::string> split(const std::string& value, const char delimiter) {
    std::vector<std::string> parts;
    std::istringstream input(value);
    std::string part;
    while(std::getline(input, part, delimiter)) {
        parts.push_back(part);
    }
    return parts;
}

PixelInput parseManualPixel(const std::string& value) {
    const std::vector<std::string> parts = split(value, ',');
    if(parts.size() != 3 || parts[0].empty()) {
        throw std::runtime_error("--manual must be name,u,v; got: " + value);
    }
    if(!std::all_of(parts[0].begin(), parts[0].end(), [](const unsigned char character) {
           return std::isalnum(character) != 0 || character == '_' || character == '-';
       })) {
        throw std::runtime_error("Manual point name may contain only letters, digits, '_' and '-'");
    }
    return {-1, parts[0], rgb_base0::parseFiniteDouble(parts[1], "manual u"),
            rgb_base0::parseFiniteDouble(parts[2], "manual v"), 1, 1, 0.0, "manual"};
}

Options parseOptions(const int argc, char** argv) {
    Options options;
    for(int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if(argument == "--help" || argument == "-h") {
            printUsage();
            std::exit(0);
        }
        if(index + 1 >= argc) {
            throw std::runtime_error("Missing value after argument: " + argument);
        }
        const std::string value = argv[++index];
        if(argument == "--calibration") {
            options.calibrationPath = value;
        }
        else if(argument == "--output-root") {
            options.outputRoot = value;
        }
        else if(argument == "--python") {
            options.pythonPath = value;
        }
        else if(argument == "--weights") {
            options.weightsPath = value;
        }
        else if(argument == "--ground-truth") {
            options.groundTruthPath = std::filesystem::path(value);
        }
        else if(argument == "--manual") {
            options.manualPixels.push_back(parseManualPixel(value));
        }
        else if(argument == "--confidence") {
            options.confidence = rgb_base0::parseFiniteDouble(value, "--confidence");
        }
        else if(argument == "--trend-limit") {
            options.trendCorrelationLimit = rgb_base0::parseFiniteDouble(value, "--trend-limit");
        }
        else {
            throw std::runtime_error("Unknown argument: " + argument);
        }
    }
    if(options.calibrationPath.empty()) {
        throw std::runtime_error("--calibration is required");
    }
    if(options.confidence <= 0.0 || options.confidence > 1.0) {
        throw std::runtime_error("--confidence must be in (0,1]");
    }
    if(options.trendCorrelationLimit <= 0.0 || options.trendCorrelationLimit > 1.0) {
        throw std::runtime_error("--trend-limit must be in (0,1]");
    }
    std::map<std::string, bool> manualNames;
    for(const PixelInput& pixel : options.manualPixels) {
        if(!manualNames.emplace(pixel.name, true).second) {
            throw std::runtime_error("Manual pixel name is duplicated: " + pixel.name);
        }
    }
    return options;
}

std::string poseText(const rgb_base0::RobotPose& pose) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(6) << "XYZABC=[" << pose.x << ", " << pose.y << ", " << pose.z
         << ", " << pose.a << ", " << pose.b << ", " << pose.c << ']';
    return text.str();
}

std::string vectorText(const rgb_base0::Vec3& value) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(9) << '[' << value.x << ", " << value.y << ", " << value.z << ']';
    return text.str();
}

std::string matrixText(const rgb_base0::Mat3& matrix) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(12) << "[[" << matrix[0][0] << ", " << matrix[0][1] << ", "
         << matrix[0][2] << "], [" << matrix[1][0] << ", " << matrix[1][1] << ", " << matrix[1][2]
         << "], [" << matrix[2][0] << ", " << matrix[2][1] << ", " << matrix[2][2] << "]]";
    return text.str();
}

void logRotation(rgb_base0::Logger& logger, const std::string& name, const rgb_base0::Mat3& matrix) {
    const rgb_base0::RotationDiagnostics check = rgb_base0::rotationDiagnostics(matrix);
    logger.line("[ROTATION CHECK] " + name + " R^T R=" + matrixText(check.rtR)
                + " orthogonality_error=" + std::to_string(check.orthogonalityError)
                + " determinant=" + std::to_string(check.determinant));
}

bool isIdentity(const rgb_base0::Mat3& matrix) {
    double maximumError = 0.0;
    for(std::size_t row = 0; row < 3; ++row) {
        for(std::size_t column = 0; column < 3; ++column) {
            maximumError = std::max(maximumError,
                                    std::abs(matrix[row][column] - (row == column ? 1.0 : 0.0)));
        }
    }
    return maximumError <= 1e-12;
}

void requirePoseMatchesCalibration(rgb_base0::RobotPoseReader& robot,
                                   const rgb_base0::RobotPoseCapture& live,
                                   const rgb_base0::CalibrationData& calibration) {
    rgb_base0::RobotPoseCapture stored;
    stored.samples.push_back(calibration.robotPose);
    stored.mean = calibration.robotPose;
    robot.requireSamePose(stored, live, calibration.xyzSpreadToleranceMm, calibration.abcSpreadToleranceDeg);
}

std::string readText(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if(!input) {
        throw std::runtime_error("Cannot read text artifact: " + path.string());
    }
    std::ostringstream value;
    value << input.rdbuf();
    return value.str();
}

std::vector<PixelInput> runYolo(const Options& options,
                                const std::filesystem::path& runDirectory,
                                rgb_base0::Logger& logger) {
    const std::filesystem::path script = std::filesystem::path(RGB_BASE0_REPO_ROOT)
                                         / "tools" / "rgb_base0_calibration" / "scripts" / "yolo_detect.py";
    const std::filesystem::path yoloLog = runDirectory / "yolo_terminal_log.txt";
    if(!std::filesystem::is_regular_file(options.pythonPath)) {
        throw std::runtime_error("Python executable not found: " + options.pythonPath.string());
    }
    if(!std::filesystem::is_regular_file(options.weightsPath) || !std::filesystem::is_regular_file(script)) {
        throw std::runtime_error("YOLO weights or sidecar script is missing");
    }
    std::ostringstream confidence;
    confidence << std::setprecision(17) << options.confidence;
    const std::string command = rgb_base0::quoteWindowsArgument(options.pythonPath.string()) + " "
                                + rgb_base0::quoteWindowsArgument(script.string()) + " --frames "
                                + rgb_base0::quoteWindowsArgument((runDirectory / "raw_frames").string()) + " --output "
                                + rgb_base0::quoteWindowsArgument(runDirectory.string()) + " --weights "
                                + rgb_base0::quoteWindowsArgument(options.weightsPath.string()) + " --confidence "
                                + confidence.str() + " --expected-frames 10 --log-file "
                                + rgb_base0::quoteWindowsArgument(yoloLog.string()) + " > NUL 2>&1";
    logger.line("[YOLO] invoking existing Ultralytics environment; C++ retains all camera geometry.");
    const int exitCode = std::system(command.c_str());
    if(std::filesystem::exists(yoloLog)) {
        std::istringstream yoloOutput(readText(yoloLog));
        std::string line;
        while(std::getline(yoloOutput, line)) {
            logger.line(line);
        }
    }
    if(exitCode != 0) {
        throw std::runtime_error("YOLO sidecar failed with process exit code " + std::to_string(exitCode));
    }

    const auto rows = rgb_base0::readCsv(runDirectory / "stable_ball_pixels.csv");
    if(rows.empty()) {
        throw std::runtime_error("YOLO stable-ball CSV is empty");
    }
    const auto header = rgb_base0::csvHeaderMap(rows.front());
    const auto column = [&header](const std::string& name) -> std::size_t {
        const auto found = header.find(name);
        if(found == header.end()) {
            throw std::runtime_error("YOLO stable-ball CSV missing column: " + name);
        }
        return found->second;
    };
    const std::size_t classIdColumn = column("class_id");
    const std::size_t classNameColumn = column("class_name");
    const std::size_t observationColumn = column("observation_count");
    const std::size_t inlierColumn = column("inlier_count");
    const std::size_t uColumn = column("final_median_u");
    const std::size_t vColumn = column("final_median_v");
    const std::size_t radialColumn = column("median_radial_distance_px");
    const std::size_t statusColumn = column("status");
    std::vector<PixelInput> accepted;
    for(std::size_t rowIndex = 1; rowIndex < rows.size(); ++rowIndex) {
        const auto& row = rows[rowIndex];
        if(row.size() != header.size()) {
            throw std::runtime_error("YOLO stable-ball CSV row has incorrect column count");
        }
        if(row[statusColumn] != "accepted") {
            continue;
        }
        accepted.push_back({std::stoi(row[classIdColumn]), row[classNameColumn],
                            rgb_base0::parseFiniteDouble(row[uColumn], "stable u"),
                            rgb_base0::parseFiniteDouble(row[vColumn], "stable v"),
                            std::stoi(row[observationColumn]), std::stoi(row[inlierColumn]),
                            rgb_base0::parseFiniteDouble(row[radialColumn], "median radial distance"), "yolo"});
    }
    return accepted;
}

std::vector<GroundTruth> loadGroundTruth(const std::filesystem::path& path) {
    const auto rows = rgb_base0::readCsv(path);
    if(rows.empty()) {
        throw std::runtime_error("Ground-truth CSV is empty");
    }
    const auto header = rgb_base0::csvHeaderMap(rows.front());
    const auto required = [&header](const std::string& name) {
        const auto found = header.find(name);
        if(found == header.end()) {
            throw std::runtime_error("Ground-truth CSV missing column: " + name);
        }
        return found->second;
    };
    const std::size_t nameColumn = required("class_name");
    const std::size_t xColumn = required("x_mm");
    const std::size_t yColumn = required("y_mm");
    const std::size_t zColumn = required("z_mm");
    std::vector<GroundTruth> result;
    std::map<std::string, bool> names;
    for(std::size_t index = 1; index < rows.size(); ++index) {
        if(rows[index].size() != rows.front().size()) {
            throw std::runtime_error("Ground-truth CSV row has incorrect column count");
        }
        const std::string name = rows[index][nameColumn];
        if(name.empty() || !names.emplace(name, true).second) {
            throw std::runtime_error("Ground-truth class_name is empty or duplicated: " + name);
        }
        result.push_back({name, rgb_base0::parseFiniteDouble(rows[index][xColumn], "ground truth x_mm"),
                          rgb_base0::parseFiniteDouble(rows[index][yColumn], "ground truth y_mm"),
                          rgb_base0::parseFiniteDouble(rows[index][zColumn], "ground truth z_mm")});
    }
    return result;
}

double correlation(const std::vector<double>& left, const std::vector<double>& right) {
    if(left.size() != right.size() || left.empty()) {
        throw std::runtime_error("Cannot calculate correlation for mismatched or empty data");
    }
    const double leftMean = std::accumulate(left.begin(), left.end(), 0.0) / static_cast<double>(left.size());
    const double rightMean = std::accumulate(right.begin(), right.end(), 0.0) / static_cast<double>(right.size());
    double numerator = 0.0;
    double leftSquared = 0.0;
    double rightSquared = 0.0;
    for(std::size_t index = 0; index < left.size(); ++index) {
        const double dl = left[index] - leftMean;
        const double dr = right[index] - rightMean;
        numerator += dl * dr;
        leftSquared += dl * dl;
        rightSquared += dr * dr;
    }
    if(leftSquared <= 1e-18 || rightSquared <= 1e-18) {
        return 0.0;
    }
    return numerator / std::sqrt(leftSquared * rightSquared);
}

GroundTruthSummary evaluateGroundTruth(const std::vector<PointResult>& points,
                                      const std::filesystem::path& path,
                                      const double correlationLimit) {
    GroundTruthSummary summary;
    summary.status = "failed";
    summary.reason.clear();
    summary.correlationLimit = correlationLimit;
    const std::vector<GroundTruth> truth = loadGroundTruth(path);
    std::map<std::string, PointResult> byName;
    for(const PointResult& point : points) {
        byName.emplace(point.input.name, point);
    }
    for(const GroundTruth& expected : truth) {
        const auto found = byName.find(expected.name);
        if(found == byName.end()) {
            continue;
        }
        const PointResult& point = found->second;
        const double errorX = point.intersection.pointBase0.x - expected.xMm;
        const double errorY = point.intersection.pointBase0.y - expected.yMm;
        const double errorZ = point.intersection.pointBase0.z - expected.zMm;
        const double errorXy = std::hypot(errorX, errorY);
        summary.errors.push_back({expected.name, point.input.u, point.input.v,
                                  point.intersection.pointBase0.x, point.intersection.pointBase0.y,
                                  expected.xMm, expected.yMm, expected.zMm, errorX, errorY, errorZ,
                                  errorXy, std::hypot(errorXy, errorZ)});
    }
    if(summary.errors.size() < 6) {
        summary.reason = "fewer than 6 ground-truth names matched accepted points";
        return summary;
    }
    std::vector<double> u;
    std::vector<double> v;
    std::vector<double> errorX;
    std::vector<double> errorY;
    double squaredSum = 0.0;
    for(const GroundTruthError& error : summary.errors) {
        u.push_back(error.u);
        v.push_back(error.v);
        errorX.push_back(error.errorX);
        errorY.push_back(error.errorY);
        squaredSum += error.errorXy * error.errorXy;
        summary.maxXyMm = std::max(summary.maxXyMm, error.errorXy);
    }
    summary.rmsXyMm = std::sqrt(squaredSum / static_cast<double>(summary.errors.size()));
    summary.corrErrorXWithU = correlation(errorX, u);
    summary.corrErrorXWithV = correlation(errorX, v);
    summary.corrErrorYWithU = correlation(errorY, u);
    summary.corrErrorYWithV = correlation(errorY, v);
    const double maximumCorrelation = std::max({std::abs(summary.corrErrorXWithU),
                                                std::abs(summary.corrErrorXWithV),
                                                std::abs(summary.corrErrorYWithU),
                                                std::abs(summary.corrErrorYWithV)});
    if(summary.rmsXyMm > 3.0) {
        summary.reason = "RMS XY error exceeds 3 mm";
    }
    else if(summary.maxXyMm > 5.0) {
        summary.reason = "at least one XY error exceeds 5 mm";
    }
    else if(maximumCorrelation >= correlationLimit) {
        summary.reason = "residual error has a systematic image-position correlation at/above the configured limit";
    }
    else {
        summary.status = "passed";
        summary.reason = ">=6 points, RMS<=3 mm, every point<=5 mm, and residual/image correlations below limit";
    }
    return summary;
}

std::string escapeJson(const std::string& value) {
    std::string result;
    for(const char character : value) {
        if(character == '\\' || character == '"') {
            result.push_back('\\');
        }
        if(character == '\n') {
            result += "\\n";
        }
        else {
            result.push_back(character);
        }
    }
    return result;
}

void writeManualDetectionArtifacts(const std::filesystem::path& runDirectory,
                                   const std::vector<PixelInput>& inputs) {
    std::ofstream json(runDirectory / "detections.json", std::ios::binary);
    std::ofstream csv(runDirectory / "detections.csv", std::ios::binary);
    if(!json || !csv) {
        throw std::runtime_error("Cannot create manual-mode detection artifacts");
    }
    json << "{\n  \"schema_version\": \"1.0\",\n  \"mode\": \"manual_pixels\",\n  \"pixels\": [\n";
    csv << "class_name,u,v,source\n";
    for(std::size_t index = 0; index < inputs.size(); ++index) {
        json << "    {\"class_name\": \"" << escapeJson(inputs[index].name) << "\", \"u\": "
             << std::setprecision(17) << inputs[index].u << ", \"v\": " << inputs[index].v << "}"
             << (index + 1 == inputs.size() ? "\n" : ",\n");
        csv << '"' << inputs[index].name << "\"," << inputs[index].u << ',' << inputs[index].v << ",manual\n";
    }
    json << "  ]\n}\n";
    if(!json || !csv) {
        throw std::runtime_error("Failed writing manual-mode detection artifacts");
    }
}

void writeResults(const std::filesystem::path& runDirectory,
                  const rgb_base0::CalibrationData& calibration,
                  const std::vector<PointResult>& results,
                  const GroundTruthSummary& truth,
                  const std::string& mode) {
    std::ofstream json(runDirectory / "results.json", std::ios::binary);
    std::ofstream csv(runDirectory / "results.csv", std::ios::binary);
    if(!json || !csv) {
        throw std::runtime_error("Cannot create validation result artifacts");
    }
    json << std::setprecision(17)
         << "{\n  \"schema_version\": \"1.1\",\n"
         << "  \"experimental\": true,\n"
         << "  \"authorized_for_robot_motion\": false,\n"
         << "  \"mode\": \"" << mode << "\",\n"
         << "  \"camera_serial\": \"" << escapeJson(calibration.serialNumber) << "\",\n"
         << "  \"distortion_model_assumption\": \""
         << escapeJson(calibration.distortionModelAssumption) << "\",\n"
         << "  \"distortion_coefficient_mapping\": \""
         << escapeJson(calibration.distortionCoefficientMapping) << "\",\n"
         << "  \"inverse_projection_version\": \""
         << escapeJson(calibration.inverseProjectionVersion) << "\",\n"
         << "  \"rotation_convention_source\": \"user_approved_temporary\",\n"
         << "  \"target_ball_center_z_mm\": " << calibration.zTableMm + calibration.ballRadiusMm << ",\n"
         << "  \"points\": [\n";
    csv << "class_id,class_name,source,u,v,observation_count,inlier_count,median_radial_distance_px,"
           "ray_rgb_x,ray_rgb_y,ray_rgb_z,ray_base0_x,ray_base0_y,ray_base0_z,lambda_mm,base0_x_mm,base0_y_mm,base0_z_mm,"
            "distorted_normalized_x,distorted_normalized_y,undistorted_normalized_x,undistorted_normalized_y,"
            "inverse_iterations,final_normalized_residual,reprojection_error_px\n";
    for(std::size_t index = 0; index < results.size(); ++index) {
        const PointResult& result = results[index];
        const auto& input = result.input;
        const auto& ray = result.ray;
        const auto& hit = result.intersection;
        json << "    {\n"
             << "      \"class_id\": " << input.classId << ",\n"
             << "      \"class_name\": \"" << escapeJson(input.name) << "\",\n"
             << "      \"source\": \"" << input.source << "\",\n"
             << "      \"uv\": [" << input.u << ", " << input.v << "],\n"
             << "      \"observation_count\": " << input.observationCount << ",\n"
             << "      \"inlier_count\": " << input.inlierCount << ",\n"
             << "      \"median_radial_distance_px\": " << input.medianRadialDistancePx << ",\n"
             << "      \"distorted_normalized\": [" << ray.distortedNormalized.x << ", "
             << ray.distortedNormalized.y << "],\n"
             << "      \"undistorted_normalized\": [" << ray.undistortedNormalized.x << ", "
             << ray.undistortedNormalized.y << "],\n"
             << "      \"reprojected_pixel\": [" << ray.reprojectedPixel.x << ", "
             << ray.reprojectedPixel.y << "],\n"
             << "      \"unit_ray_rgb\": [" << ray.unitRayRgb.x << ", " << ray.unitRayRgb.y << ", " << ray.unitRayRgb.z << "],\n"
             << "      \"unit_ray_base0\": [" << hit.unitRayBase0.x << ", " << hit.unitRayBase0.y << ", " << hit.unitRayBase0.z << "],\n"
             << "      \"lambda_mm\": " << hit.lambdaMm << ",\n"
             << "      \"point_base0_mm\": [" << hit.pointBase0.x << ", " << hit.pointBase0.y << ", " << hit.pointBase0.z << "],\n"
             << "      \"diagnostics\": {\"inverse_iterations\": " << ray.inverseIterations
             << ", \"final_normalized_residual\": " << ray.finalNormalizedResidual
             << ", \"reprojection_error_px\": " << ray.reprojectionErrorPx << "}\n"
             << "    }" << (index + 1 == results.size() ? "\n" : ",\n");
        csv << input.classId << ",\"" << input.name << "\"," << input.source << ',' << input.u << ',' << input.v << ','
            << input.observationCount << ',' << input.inlierCount << ',' << input.medianRadialDistancePx << ','
            << ray.unitRayRgb.x << ',' << ray.unitRayRgb.y << ',' << ray.unitRayRgb.z << ','
            << hit.unitRayBase0.x << ',' << hit.unitRayBase0.y << ',' << hit.unitRayBase0.z << ',' << hit.lambdaMm << ','
            << hit.pointBase0.x << ',' << hit.pointBase0.y << ',' << hit.pointBase0.z << ','
            << ray.distortedNormalized.x << ',' << ray.distortedNormalized.y << ','
            << ray.undistortedNormalized.x << ',' << ray.undistortedNormalized.y << ','
            << ray.inverseIterations << ',' << ray.finalNormalizedResidual << ','
            << ray.reprojectionErrorPx << '\n';
    }
    json << "  ],\n  \"ground_truth\": {\n"
         << "    \"status\": \"" << escapeJson(truth.status) << "\",\n"
         << "    \"reason\": \"" << escapeJson(truth.reason) << "\",\n"
         << "    \"matched_count\": " << truth.errors.size() << ",\n"
         << "    \"rms_xy_mm\": " << truth.rmsXyMm << ",\n"
         << "    \"max_xy_mm\": " << truth.maxXyMm << ",\n"
         << "    \"trend_correlation_limit\": " << truth.correlationLimit << ",\n"
         << "    \"correlations\": {\"error_x_vs_u\": " << truth.corrErrorXWithU
         << ", \"error_x_vs_v\": " << truth.corrErrorXWithV << ", \"error_y_vs_u\": "
         << truth.corrErrorYWithU << ", \"error_y_vs_v\": " << truth.corrErrorYWithV << "},\n"
         << "    \"errors\": [\n";
    for(std::size_t index = 0; index < truth.errors.size(); ++index) {
        const GroundTruthError& error = truth.errors[index];
        json << "      {\"class_name\": \"" << escapeJson(error.name) << "\", \"uv\": [" << error.u << ", "
             << error.v << "], \"calculated_xyz_mm\": [" << error.calculatedX << ", " << error.calculatedY
             << ", " << calibration.zTableMm + calibration.ballRadiusMm << "], \"truth_xyz_mm\": ["
             << error.truthX << ", " << error.truthY << ", " << error.truthZ << "], \"error_xyz_mm\": ["
             << error.errorX << ", " << error.errorY << ", " << error.errorZ << "], \"error_xy_mm\": "
             << error.errorXy << ", \"error_3d_mm\": " << error.error3d << "}"
             << (index + 1 == truth.errors.size() ? "\n" : ",\n");
    }
    json << "    ]\n  },\n"
         << "  \"hole_calculation\": \"deferred_until_ball_points_confirmed\"\n}\n";
    if(!json || !csv) {
        throw std::runtime_error("Failed writing validation result artifacts");
    }
}

}  // namespace

int main(const int argc, char** argv) {
    std::unique_ptr<rgb_base0::Logger> logger;
    std::unique_ptr<rgb_base0::RobotPoseReader> robot;
    try {
        const Options options = parseOptions(argc, argv);
        const rgb_base0::CalibrationData calibration = rgb_base0::readCalibration(options.calibrationPath);
        const std::filesystem::path runDirectory = options.outputRoot / rgb_base0::localCompactTimestamp();
        if(std::filesystem::exists(runDirectory)) {
            throw std::runtime_error("Timestamped validation directory already exists: " + runDirectory.string());
        }
        std::filesystem::create_directories(runDirectory / "raw_frames");
        std::filesystem::create_directories(runDirectory / "annotated_frames");
        logger = std::make_unique<rgb_base0::Logger>(runDirectory / "terminal_log.txt");
        logger->line("[START] Standalone experimental RGB pixel -> Base0 validation");
        logger->line("[SAFETY] No robot motion/motor/alarm/DO command is implemented; outputs are diagnostics only.");
        logger->line("[WARNING] rotation_convention_source=user_approved_temporary, not HIWIN-verified.");
        logger->line("[CALIBRATION] " + options.calibrationPath.string());
        logger->line("[CALIBRATION] camera serial=" + calibration.serialNumber + " pose=" + poseText(calibration.robotPose));
        logger->line("[CAMERA] model=" + calibration.cameraModel + " serial=" + calibration.serialNumber
                     + " stored_profile=" + std::to_string(calibration.profile.width) + "x"
                     + std::to_string(calibration.profile.height) + "@" + std::to_string(calibration.profile.fps)
                     + " " + calibration.profile.format);
        logger->line("[INTRINSICS] fx=" + std::to_string(calibration.intrinsic.fx)
                     + " fy=" + std::to_string(calibration.intrinsic.fy)
                     + " cx=" + std::to_string(calibration.intrinsic.cx)
                     + " cy=" + std::to_string(calibration.intrinsic.cy));
        logger->line("  K=[[" + std::to_string(calibration.intrinsic.fx) + ",0,"
                     + std::to_string(calibration.intrinsic.cx) + "],[0,"
                     + std::to_string(calibration.intrinsic.fy) + ","
                     + std::to_string(calibration.intrinsic.cy) + "],[0,0,1]]");
        logger->line("[DISTORTION] model=" + calibration.distortionFamily + " variant="
                     + calibration.distortionVariant + " method=" + calibration.distortionHandling);
        logger->line("  assumption=" + calibration.distortionModelAssumption);
        logger->line("  mapping=" + calibration.distortionCoefficientMapping);
        logger->line("  inverse_version=" + calibration.inverseProjectionVersion
                     + " max_iterations=" + std::to_string(calibration.inverseMaxIterations)
                     + " convergence_tolerance=" + std::to_string(calibration.inverseConvergenceTolerance)
                     + " reprojection_tolerance_px="
                     + std::to_string(calibration.inverseReprojectionTolerancePx));
        logger->line("  D[k1,k2,k3,k4,k5,k6,p1,p2]=[" + std::to_string(calibration.distortion.k1) + ","
                     + std::to_string(calibration.distortion.k2) + "," + std::to_string(calibration.distortion.k3) + ","
                     + std::to_string(calibration.distortion.k4) + "," + std::to_string(calibration.distortion.k5) + ","
                     + std::to_string(calibration.distortion.k6) + "," + std::to_string(calibration.distortion.p1) + ","
                     + std::to_string(calibration.distortion.p2) + "]");
        logger->line("[CAMERA FRAME] name=" + calibration.cameraFrameName + " axes=" + calibration.opticalAxes);
        logger->line("[ROBOT] Tool3 raw " + poseText(calibration.robotPose) + " angle_unit="
                     + calibration.tool3AngleUnit + " convention=" + calibration.rotationConvention);
        logger->line("[EXTRINSICS] R_Base0_from_Tool3=" + matrixText(calibration.rBase0FromTool3));
        logger->line("  R_Tool3_from_RGB=" + matrixText(calibration.rTool3FromRgb));
        logger->line("  t_Tool3_to_RGB_mm=" + vectorText(calibration.tTool3ToRgb));
        logger->line("  R_Base0_from_RGB=" + matrixText(calibration.rBase0FromRgb));
        logger->line("  C_Base0_mm=" + vectorText(calibration.tBase0FromRgb));
        logRotation(*logger, "R_Base0_from_Tool3", calibration.rBase0FromTool3);
        logRotation(*logger, "R_Tool3_from_RGB", calibration.rTool3FromRgb);
        logRotation(*logger, "R_Base0_from_RGB", calibration.rBase0FromRgb);
        logger->line("[TABLE] Z_table_mm=" + std::to_string(calibration.zTableMm)
                     + " ball_diameter_mm=" + std::to_string(calibration.ballDiameterMm)
                     + " ball_radius_mm=" + std::to_string(calibration.ballRadiusMm)
                     + " Z_target_mm=" + std::to_string(calibration.zTableMm + calibration.ballRadiusMm)
                     + " table_plane_model=" + calibration.tablePlaneModel);
        if(rgb_base0::norm(calibration.tTool3ToRgb) <= 1e-12) {
            logger->line("[WARNING] Tool3 origin is currently assumed to coincide with RGB optical center; t_Tool3_to_RGB is zero.");
        }
        if(isIdentity(calibration.rTool3FromRgb)) {
            logger->line("[WARNING] R_Tool3_from_RGB is identity; Tool3 axes are assumed aligned with RGB optical axes.");
        }
        logger->line("[WARNING] Physical Tool3/RGB alignment and constant-Z table model require ground-truth validation.");

        robot = std::make_unique<rgb_base0::RobotPoseReader>(calibration.robotIp);
        logger->line("[ROBOT] connected, original Tool=" + std::to_string(robot->originalToolNumber())
                     + " Base=" + std::to_string(robot->originalBaseNumber()) + "; Tool3/Base0 verified.");
        const auto logBeforeSample = [&](const int index, const int motionStateRaw,
                                         const rgb_base0::RobotPose& pose) {
            logger->line("[ROBOT] before sample=" + std::to_string(index)
                         + " get_motion_state_raw=" + std::to_string(motionStateRaw)
                         + ' ' + poseText(pose));
        };
        const rgb_base0::RobotPoseCapture before = robot->captureStablePose(
            calibration.robotPoseSampleCount, calibration.robotPoseSampleWindowMs,
            calibration.xyzSpreadToleranceMm, calibration.abcSpreadToleranceDeg,
            logBeforeSample);
        requirePoseMatchesCalibration(*robot, before, calibration);
        logger->line("[ROBOT] live pre-capture pose stable and matches calibration: " + poseText(before.mean));

        rgb_base0::OrbbecCamera camera;
        camera.requireMatches(calibration);
        logger->line("[CAMERA] serial/profile/intrinsic/distortion match passed: " + camera.profileDescription());
        for(const std::string& line : camera.diagnosticLines()) {
            logger->line(line);
        }
        const bool manualMode = !options.manualPixels.empty();
        const int frameCount = manualMode ? 1 : 10;
        const auto frames = camera.captureMjpgFrames(runDirectory / "raw_frames", frameCount);
        for(const auto& frame : frames) {
            logger->line("[CAMERA] raw unchanged frame=" + frame.path.filename().string()
                         + " index=" + std::to_string(frame.frameIndex)
                         + " device_timestamp_us=" + std::to_string(frame.deviceTimestampUs));
        }
        const auto logAfterSample = [&](const int index, const int motionStateRaw,
                                        const rgb_base0::RobotPose& pose) {
            logger->line("[ROBOT] after sample=" + std::to_string(index)
                         + " get_motion_state_raw=" + std::to_string(motionStateRaw)
                         + ' ' + poseText(pose));
        };
        const rgb_base0::RobotPoseCapture after = robot->captureStablePose(
            calibration.robotPoseSampleCount, calibration.robotPoseSampleWindowMs,
            calibration.xyzSpreadToleranceMm, calibration.abcSpreadToleranceDeg,
            logAfterSample);
        robot->requireSamePose(before, after, calibration.xyzSpreadToleranceMm, calibration.abcSpreadToleranceDeg);
        requirePoseMatchesCalibration(*robot, after, calibration);
        logger->line("[ROBOT] post-capture pose stable, unchanged, and matches calibration: " + poseText(after.mean));
        robot->restoreAndClose();
        robot.reset();
        logger->line("[ROBOT] original Tool/Base restored; connection closed without changing motor state.");

        std::vector<PixelInput> pixels;
        if(manualMode) {
            pixels = options.manualPixels;
            writeManualDetectionArtifacts(runDirectory, pixels);
            logger->line("[INPUT] manual raw-image pixel mode; input count=" + std::to_string(pixels.size()));
        }
        else {
            logger->line("[WARNING] YOLO bbox center is treated as the projected ball center (first-order approximation).");
            pixels = runYolo(options, runDirectory, *logger);
            logger->line("[YOLO] accepted stable ball classes for Base0 projection=" + std::to_string(pixels.size()));
        }

        std::vector<PointResult> results;
        const double targetZ = calibration.zTableMm + calibration.ballRadiusMm;
        for(const PixelInput& pixel : pixels) {
            logger->line("[PIXEL INPUT] name=" + pixel.name + " u=" + std::to_string(pixel.u)
                         + " v=" + std::to_string(pixel.v) + " source=" + pixel.source);
            const rgb_base0::PixelRayDiagnostics ray = camera.pixelToUnitRay(pixel.u, pixel.v);
            const rgb_base0::Vec3 rayBase0 = rgb_base0::multiply(calibration.rBase0FromRgb, ray.unitRayRgb);
            const double rayBase0Norm = rgb_base0::norm(rayBase0);
            if(!std::isfinite(rayBase0Norm) || std::abs(rayBase0Norm - 1.0) > 1e-9) {
                throw std::runtime_error("Rotated Base0 ray is non-finite or not unit length");
            }
            const rgb_base0::PlaneIntersection intersection = rgb_base0::intersectRayWithHorizontalPlane(
                calibration.tBase0FromRgb, rayBase0, targetZ);
            results.push_back({pixel, ray, intersection});
            logger->line("[UNDISTORTION / INVERSE PROJECTION] algorithm="
                         + calibration.inverseProjectionVersion + " RGB-only");
            logger->line("  distorted_normalized=[" + std::to_string(ray.distortedNormalized.x) + ","
                         + std::to_string(ray.distortedNormalized.y) + "] undistorted_normalized=["
                         + std::to_string(ray.undistortedNormalized.x) + ","
                         + std::to_string(ray.undistortedNormalized.y) + "] iterations="
                         + std::to_string(ray.inverseIterations) + " normalized_residual="
                         + std::to_string(ray.finalNormalizedResidual) + " reprojection_error_px="
                         + std::to_string(ray.reprojectionErrorPx));
            logger->line("[RAY CAMERA] d_camera=" + vectorText(ray.unitRayRgb)
                         + " norm=" + std::to_string(rgb_base0::norm(ray.unitRayRgb)));
            logger->line("[RAY BASE0] d_Base0=" + vectorText(intersection.unitRayBase0)
                         + " norm=" + std::to_string(rgb_base0::norm(intersection.unitRayBase0)));
            logger->line("[INTERSECTION] dz=" + std::to_string(intersection.unitRayBase0.z)
                         + " lambda_mm=" + std::to_string(intersection.lambdaMm)
                         + " target_z_mm=" + std::to_string(targetZ));
            logger->line("[RESULT] P_ball_Base0_mm=" + vectorText(intersection.pointBase0)
                         + " z_consistency_error_mm="
                         + std::to_string(std::abs(intersection.pointBase0.z - targetZ)));
            std::ostringstream line;
            line << std::fixed << std::setprecision(6) << "[POINT] " << pixel.name << " uv=(" << pixel.u << ','
                 << pixel.v << ") ray_RGB=(" << ray.unitRayRgb.x << ',' << ray.unitRayRgb.y << ','
                 << ray.unitRayRgb.z << ") ray_Base0=(" << intersection.unitRayBase0.x << ','
                 << intersection.unitRayBase0.y << ',' << intersection.unitRayBase0.z << ") lambda_mm="
                 << intersection.lambdaMm << " Base0_mm=(" << intersection.pointBase0.x << ','
                 << intersection.pointBase0.y << ',' << intersection.pointBase0.z << ") reprojection_px="
                 << ray.reprojectionErrorPx << " inverse_iterations=" << ray.inverseIterations;
            logger->line(line.str());
        }

        GroundTruthSummary truth;
        truth.correlationLimit = options.trendCorrelationLimit;
        if(options.groundTruthPath) {
            truth = evaluateGroundTruth(results, *options.groundTruthPath, options.trendCorrelationLimit);
            std::ostringstream summary;
            summary << std::fixed << std::setprecision(6) << "[GROUND TRUTH] status=" << truth.status
                    << " matched=" << truth.errors.size() << " rms_xy_mm=" << truth.rmsXyMm
                    << " max_xy_mm=" << truth.maxXyMm << " correlations=[" << truth.corrErrorXWithU << ','
                    << truth.corrErrorXWithV << ',' << truth.corrErrorYWithU << ',' << truth.corrErrorYWithV
                    << "] limit=" << truth.correlationLimit << " reason=" << truth.reason;
            logger->line(summary.str());
            for(const GroundTruthError& error : truth.errors) {
                logger->line("  " + error.name + " gt_xyz_mm=[" + std::to_string(error.truthX) + ","
                             + std::to_string(error.truthY) + "," + std::to_string(error.truthZ)
                             + "] estimated_xyz_mm=[" + std::to_string(error.calculatedX) + ","
                             + std::to_string(error.calculatedY) + ","
                             + std::to_string(calibration.zTableMm + calibration.ballRadiusMm)
                             + "] error_xy_mm=" + std::to_string(error.errorXy)
                             + " error_x_mm=" + std::to_string(error.errorX)
                             + " error_y_mm=" + std::to_string(error.errorY)
                             + " error_z_mm=" + std::to_string(error.errorZ)
                             + " error_3d_mm=" + std::to_string(error.error3d));
            }
        }
        else {
            logger->line("[GROUND TRUTH] not supplied; accuracy acceptance is not evaluated.");
        }

        writeResults(runDirectory, calibration, results, truth, manualMode ? "manual_pixels" : "automatic_yolo");
        logger->line("[OUTPUT] " + runDirectory.string());
        logger->line("[DEFERRED] Ball points must be confirmed first. Then remind the operator to add and validate pocket calculation.");
        if(results.empty()) {
            logger->line("[FAILED] No stable/input pixel produced a Base0 point; diagnostics were still written.");
        }
        logger->line("[DONE] Results remain experimental and are explicitly unauthorized for robot motion or main-program input.");
        if(options.groundTruthPath && truth.status != "passed") {
            return 2;
        }
        return results.empty() ? 3 : 0;
    }
    catch(const std::exception& error) {
        if(robot) {
            try {
                robot->restoreAndClose();
                if(logger) {
                    logger->line("[CLEANUP] original Tool/Base restored and controller connection closed after failure.");
                }
            }
            catch(const std::exception& cleanupError) {
                if(logger) {
                    try {
                        logger->line(std::string("[CLEANUP ERROR] ") + cleanupError.what());
                    }
                    catch(...) {
                    }
                }
                std::cerr << "CLEANUP ERROR: " << cleanupError.what() << '\n';
            }
        }
        if(logger) {
            try {
                logger->line(std::string("[ERROR] ") + error.what());
            }
            catch(...) {
            }
        }
        std::cerr << "ERROR: " << error.what() << '\n';
        return 1;
    }
}
