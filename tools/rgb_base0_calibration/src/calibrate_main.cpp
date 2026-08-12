#include "rgb_base0/calibration_io.h"
#include "rgb_base0/geometry.h"
#include "rgb_base0/orbbec_camera.h"
#include "rgb_base0/robot_pose_reader.h"
#include "rgb_base0/runtime_utils.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    double zTableMm = 0.0;
    bool hasZTable = false;
    std::string robotIp = "192.168.0.1";
    std::filesystem::path outputDirectory;
    rgb_base0::Vec3 tTool3ToRgb;
    rgb_base0::Mat3 rTool3FromRgb{{{{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 0.0, 1.0}}}};
};

void printUsage() {
    std::cout
        << "Standalone RGB optical center -> Base0 calibration capture\n\n"
        << "Usage:\n"
        << "  rgb_base0_calibrate --z-table <Base0 cloth Z mm> [options]\n\n"
        << "Options:\n"
        << "  --robot-ip <ip>   Controller address (default 192.168.0.1)\n"
        << "  --output <dir>     Output directory (default output/rgb_base0_calibration/<timestamp>)\n"
        << "  --tool3-to-rgb-mm <x,y,z>        RGB-center offset expressed in Tool3 (default 0,0,0)\n"
        << "  --r-tool3-from-rgb <9 values>     Row-major RGB-to-Tool3 rotation (default identity)\n"
        << "  --help             Show this text without connecting to hardware\n\n"
        << "Safety: this program sets Tool3/Base0 temporarily and reads pose only. It sends no motion,\n"
        << "motor, alarm-clear, or digital-output command. The robot must already be completely stopped.\n";
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
        if(argument == "--z-table") {
            options.zTableMm = rgb_base0::parseFiniteDouble(value, "--z-table");
            options.hasZTable = true;
        }
        else if(argument == "--robot-ip") {
            if(value.empty()) {
                throw std::runtime_error("--robot-ip cannot be empty");
            }
            options.robotIp = value;
        }
        else if(argument == "--output") {
            options.outputDirectory = value;
        }
        else if(argument == "--tool3-to-rgb-mm") {
            std::istringstream input(value);
            std::string part;
            std::vector<double> numbers;
            while(std::getline(input, part, ',')) {
                numbers.push_back(rgb_base0::parseFiniteDouble(part, "--tool3-to-rgb-mm"));
            }
            if(numbers.size() != 3) {
                throw std::runtime_error("--tool3-to-rgb-mm requires exactly x,y,z");
            }
            options.tTool3ToRgb = {numbers[0], numbers[1], numbers[2]};
        }
        else if(argument == "--r-tool3-from-rgb") {
            std::istringstream input(value);
            std::string part;
            std::vector<double> numbers;
            while(std::getline(input, part, ',')) {
                numbers.push_back(rgb_base0::parseFiniteDouble(part, "--r-tool3-from-rgb"));
            }
            if(numbers.size() != 9) {
                throw std::runtime_error("--r-tool3-from-rgb requires exactly 9 row-major values");
            }
            std::size_t offset = 0;
            for(auto& row : options.rTool3FromRgb) {
                for(double& element : row) {
                    element = numbers[offset++];
                }
            }
            rgb_base0::validateRotationMatrix(options.rTool3FromRgb, "--r-tool3-from-rgb");
        }
        else {
            throw std::runtime_error("Unknown argument: " + argument);
        }
    }
    if(!options.hasZTable) {
        throw std::runtime_error("--z-table is required; it is the Base0 Z of the cloth surface in millimeters");
    }
    if(options.outputDirectory.empty()) {
        options.outputDirectory = std::filesystem::path(RGB_BASE0_REPO_ROOT) / "output" / "rgb_base0_calibration"
                                  / rgb_base0::localCompactTimestamp();
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
    logger.line("[ROTATION CHECK] " + name);
    logger.line("  R=" + matrixText(matrix));
    logger.line("  R^T R=" + matrixText(check.rtR));
    logger.line("  orthogonality_error=" + std::to_string(check.orthogonalityError));
    logger.line("  determinant=" + std::to_string(check.determinant));
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

}  // namespace

int main(const int argc, char** argv) {
    std::unique_ptr<rgb_base0::Logger> logger;
    std::unique_ptr<rgb_base0::RobotPoseReader> robot;
    try {
        const Options options = parseOptions(argc, argv);
        if(std::filesystem::exists(options.outputDirectory)
           && !std::filesystem::is_empty(options.outputDirectory)) {
            throw std::runtime_error("Refusing to write into a non-empty calibration output directory: "
                                     + options.outputDirectory.string());
        }
        std::filesystem::create_directories(options.outputDirectory);
        const std::filesystem::path jsonPath = options.outputDirectory / "camera_calibration.json";
        const std::filesystem::path yamlPath = options.outputDirectory / "camera_calibration.yaml";
        const std::filesystem::path currentJson =
            std::filesystem::path(RGB_BASE0_REPO_ROOT) / "config" / "vision" / "camera_calibration.json";
        if(std::filesystem::exists(jsonPath) || std::filesystem::exists(yamlPath)) {
            throw std::runtime_error("Refusing to overwrite an existing calibration file in "
                                     + options.outputDirectory.string());
        }
        logger = std::make_unique<rgb_base0::Logger>(options.outputDirectory / "terminal_log.txt");
        logger->line("[START] Standalone experimental RGB->Base0 calibration");
        logger->line("[SAFETY] Read-only pose capture: no robot motion/motor/alarm/DO command is implemented.");
        logger->line("[WARNING] rotation_convention_source=user_approved_temporary");
        logger->line("[WARNING] HIWIN's public documents do not verify the full ABC-to-matrix convention.");
        logger->line("[INPUT] robot_ip=" + options.robotIp + " Tool3 Base0 z_table_mm="
                     + std::to_string(options.zTableMm) + " ball_diameter_mm="
                     + std::to_string(rgb_base0::kBallDiameterMm) + " ball_radius_mm="
                     + std::to_string(rgb_base0::kBallRadiusMm));

        rgb_base0::CalibrationData calibration;
        calibration.createdUtc = rgb_base0::utcIsoTimestamp();
        calibration.robotIp = options.robotIp;
        calibration.zTableMm = options.zTableMm;
        calibration.tTool3ToRgb = options.tTool3ToRgb;
        calibration.rTool3FromRgb = options.rTool3FromRgb;
        if(rgb_base0::norm(options.tTool3ToRgb) <= 1e-12) {
            logger->line("[WARNING] Tool3 origin is currently assumed to coincide with RGB optical center.");
            logger->line("[WARNING] t_Tool3_to_RGB is currently zero: " + vectorText(options.tTool3ToRgb));
        }
        if(isIdentity(options.rTool3FromRgb)) {
            logger->line("[WARNING] R_Tool3_from_RGB is currently identity: " + matrixText(options.rTool3FromRgb));
        }
        logger->line("[WARNING] Physical Tool3/RGB axis alignment still requires validation.");
        logger->line("[WARNING] Table is currently modeled as constant Base0 Z.");

        robot = std::make_unique<rgb_base0::RobotPoseReader>(options.robotIp);
        logger->line("[ROBOT] connected; original Tool=" + std::to_string(robot->originalToolNumber())
                     + " Base=" + std::to_string(robot->originalBaseNumber()));
        logger->line("[ROBOT] Tool3/Base0 set and verified.");
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
        logger->line("[ROBOT] before-camera stable mean " + poseText(before.mean));

        {
            rgb_base0::OrbbecCamera camera;
            logger->line("[CAMERA] " + camera.profileDescription());
            for(const std::string& line : camera.diagnosticLines()) {
                logger->line(line);
            }
            camera.copyCameraFieldsTo(calibration);
            const auto frames = camera.captureMjpgFrames(options.outputDirectory / "raw_frames", 1);
            logger->line("[CAMERA] saved unmodified raw MJPG " + frames.front().path.string()
                         + " frame_index=" + std::to_string(frames.front().frameIndex));

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
            logger->line("[ROBOT] after-camera stable mean " + poseText(after.mean));
            robot->requireSamePose(before, after);
            logger->line("[ROBOT] before/after pose agreement passed (XYZ <=0.1 mm, wrapped ABC <=0.05 deg).");
        }

        calibration.robotPose = before.mean;
        calibration.rBase0FromTool3 = rgb_base0::rotationBase0FromTool3Zyx(
            before.mean.a, before.mean.b, before.mean.c);
        calibration.rBase0FromRgb = rgb_base0::multiply(calibration.rBase0FromTool3, calibration.rTool3FromRgb);
        const rgb_base0::Vec3 baseOffset = rgb_base0::multiply(calibration.rBase0FromTool3, calibration.tTool3ToRgb);
        calibration.tBase0FromRgb = {before.mean.x + baseOffset.x,
                                     before.mean.y + baseOffset.y,
                                     before.mean.z + baseOffset.z};

        robot->restoreAndClose();
        robot.reset();
        logger->line("[ROBOT] original Tool/Base restored and connection closed without changing motor state.");
        rgb_base0::validateCalibration(calibration);
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
        logger->line("[ROBOT] Tool3 " + poseText(calibration.robotPose));
        logger->line("  convention=" + calibration.rotationConvention + " source="
                     + calibration.rotationConventionSource + " angle_unit=" + calibration.tool3AngleUnit);
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
                     + " model=" + calibration.tablePlaneModel);
        rgb_base0::writeCalibrationJson(calibration, jsonPath);
        rgb_base0::writeCalibrationYaml(calibration, yamlPath);
        const rgb_base0::CalibrationData json = rgb_base0::readCalibration(jsonPath);
        const rgb_base0::CalibrationData yaml = rgb_base0::readCalibration(yamlPath);
        if(!rgb_base0::equivalentCalibration(calibration, json)
           || !rgb_base0::equivalentCalibration(calibration, yaml)
           || !rgb_base0::equivalentCalibration(json, yaml)) {
            throw std::runtime_error("Written JSON/YAML calibration equivalence check failed");
        }
        logger->line("[OUTPUT] JSON/YAML strict read-back and equivalence check passed.");
        logger->line("[OUTPUT] historical calibration JSON: " + jsonPath.string());
        logger->line("[OUTPUT] historical calibration YAML: " + yamlPath.string());
        rgb_base0::publishCurrentCalibrationJson(jsonPath, json, currentJson);
        logger->line("[OUTPUT] current calibration updated: " + currentJson.string());
        logger->line("[DONE] Experimental calibration captured and current calibration published successfully.");
        logger->line("[DONE] Current calibration is available for production RGB geometry; it does not authorize robot motion.");
        return 0;
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
