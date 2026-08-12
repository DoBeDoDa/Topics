#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include "rgb_base0/calibration_io.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

void require(const bool condition, const std::string& message) {
    if(!condition) {
        throw std::runtime_error(message);
    }
}

std::string readBytes(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    require(static_cast<bool>(input), "cannot read test file: " + path.string());
    std::ostringstream bytes;
    bytes << input.rdbuf();
    require(!input.bad(), "failed reading test file: " + path.string());
    return bytes.str();
}

void writeBytes(const std::filesystem::path& path, const std::string& bytes) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    require(static_cast<bool>(output), "cannot write test file: " + path.string());
    output << bytes;
    output.flush();
    require(static_cast<bool>(output), "failed writing test file: " + path.string());
}

rgb_base0::CalibrationData makeCalibration(const std::string& timestamp,
                                           const std::string& serial,
                                           const double zTableMm) {
    rgb_base0::CalibrationData calibration;
    calibration.createdUtc = timestamp;
    calibration.deviceName = "Orbbec Gemini 2 XL";
    calibration.serialNumber = serial;
    calibration.firmwareVersion = "publication-test";
    calibration.profile = {1280, 720, 10, "MJPG"};
    calibration.intrinsic = {610.25, 610.42, 642.50, 352.96};
    calibration.distortion = {-1.19, 0.73, -0.16, -1.17, 0.71, -0.15, -0.0003, -0.0002};
    calibration.robotPose = {15.0, 500.0, 460.0, 0.0, 0.0, 0.0};
    calibration.tBase0FromRgb = {15.0, 500.0, 460.0};
    calibration.zTableMm = zTableMm;
    return calibration;
}

bool publishRejected(const std::filesystem::path& source,
                     const rgb_base0::CalibrationData& calibration,
                     const std::filesystem::path& current) {
    try {
        rgb_base0::publishCurrentCalibrationJson(source, calibration, current);
        return false;
    }
    catch(const std::exception&) {
        return true;
    }
}

void removeTree(const std::filesystem::path& root) {
    std::error_code error;
    std::filesystem::remove_all(root, error);
    require(!error, "cannot clean publication test workspace: " + error.message());
}

}  // namespace

int main() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "rgb_base0_publication_tests";
    try {
        removeTree(root);
        const std::filesystem::path history = root / "history";
        const std::filesystem::path current = root / "config" / "vision" / "camera_calibration.json";
        const std::filesystem::path temporary = current.string() + ".tmp";
        const std::filesystem::path rollback = current.string() + ".rollback.tmp";
        std::filesystem::create_directories(history);

        const rgb_base0::CalibrationData first = makeCalibration(
            "2026-08-11T08:00:00Z", "PUBLICATION-FIRST", -233.51);
        const std::filesystem::path firstJson = history / "first" / "camera_calibration.json";
        const std::filesystem::path firstYaml = history / "first" / "camera_calibration.yaml";
        std::filesystem::create_directories(firstJson.parent_path());
        rgb_base0::writeCalibrationJson(first, firstJson);
        rgb_base0::writeCalibrationYaml(first, firstYaml);
        const std::string firstBytes = readBytes(firstJson);
        const rgb_base0::CalibrationData verifiedFirst = rgb_base0::readCalibration(firstJson);
        require(rgb_base0::equivalentCalibration(first, verifiedFirst), "first fixture read-back mismatch");

        rgb_base0::publishCurrentCalibrationJson(firstJson, verifiedFirst, current);
        require(std::filesystem::is_regular_file(current), "CASE A current JSON was not created");
        require(std::filesystem::is_regular_file(firstJson), "CASE A historical JSON was removed");
        require(std::filesystem::is_regular_file(firstYaml), "CASE A historical YAML was removed");
        require(readBytes(current) == firstBytes, "CASE A current bytes differ from historical JSON");
        require(rgb_base0::equivalentCalibration(verifiedFirst, rgb_base0::readCalibration(current)),
                "CASE A current semantic verification failed");
        require(!std::filesystem::exists(temporary), "CASE A temporary file remains");
        require(!std::filesystem::exists(rollback), "CASE A rollback file remains");

        const rgb_base0::CalibrationData second = makeCalibration(
            "2026-08-11T08:01:00Z", "PUBLICATION-SECOND", -234.25);
        const std::filesystem::path secondJson = history / "second" / "camera_calibration.json";
        std::filesystem::create_directories(secondJson.parent_path());
        rgb_base0::writeCalibrationJson(second, secondJson);
        const std::string secondBytes = readBytes(secondJson);
        const rgb_base0::CalibrationData verifiedSecond = rgb_base0::readCalibration(secondJson);
        const std::string preservedFirstHistory = readBytes(firstJson);
        rgb_base0::publishCurrentCalibrationJson(secondJson, verifiedSecond, current);
        require(readBytes(current) == secondBytes, "CASE B current was not replaced by second calibration");
        require(readBytes(firstJson) == preservedFirstHistory, "CASE B old historical JSON changed");
        require(readBytes(secondJson) == secondBytes, "CASE B new historical JSON changed");
        require(!std::filesystem::exists(temporary), "CASE B temporary file remains");
        require(!std::filesystem::exists(rollback), "CASE B rollback file remains");

        const std::string protectedCurrent = readBytes(current);
        require(publishRejected(history / "missing" / "camera_calibration.json", verifiedSecond, current),
                "CASE C missing historical JSON was accepted");
        require(readBytes(current) == protectedCurrent, "CASE C current changed after missing source failure");

        const std::filesystem::path malformedJson = history / "malformed" / "camera_calibration.json";
        std::filesystem::create_directories(malformedJson.parent_path());
        writeBytes(malformedJson, "{\"schema_version\":\"1.3\"}");
        require(publishRejected(malformedJson, verifiedSecond, current),
                "CASE D malformed historical JSON was accepted");
        require(readBytes(current) == protectedCurrent, "CASE D current changed after strict parse failure");

        require(SetFileAttributesW(current.c_str(), FILE_ATTRIBUTE_READONLY) != FALSE,
                "CASE E could not mark current read-only");
        require(publishRejected(firstJson, verifiedFirst, current),
                "CASE E read-only current unexpectedly allowed replacement");
        require(readBytes(current) == protectedCurrent, "CASE E current changed after replacement failure");
        require(SetFileAttributesW(current.c_str(), FILE_ATTRIBUTE_NORMAL) != FALSE,
                "CASE E could not restore current attributes");
        require(!std::filesystem::exists(temporary), "CASE E failed temporary file was not cleaned");
        require(!std::filesystem::exists(rollback), "CASE E failed rollback file was not cleaned");

        writeBytes(temporary, "stale temporary data");
        writeBytes(rollback, "stale rollback data");
        rgb_base0::publishCurrentCalibrationJson(firstJson, verifiedFirst, current);
        require(readBytes(current) == firstBytes, "CASE F stale temporary data became current");
        require(!std::filesystem::exists(temporary), "CASE F stale temporary file remains");
        require(!std::filesystem::exists(rollback), "CASE F stale rollback file remains");

        const rgb_base0::CalibrationData finalCalibration = rgb_base0::readCalibration(current);
        require(rgb_base0::equivalentCalibration(verifiedFirst, finalCalibration),
                "CASE G final current semantic equivalence failed");
        require(readBytes(current) == readBytes(firstJson), "CASE G final current byte equivalence failed");

        removeTree(root);
        std::cout << "PASS: current calibration publication cases A-G\n";
        return 0;
    }
    catch(const std::exception& error) {
        std::error_code ignored;
        SetFileAttributesW((root / "config" / "vision" / "camera_calibration.json").c_str(),
                           FILE_ATTRIBUTE_NORMAL);
        std::filesystem::remove_all(root, ignored);
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
