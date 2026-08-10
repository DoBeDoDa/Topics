#pragma once

#include "rgb_base0/types.h"

#include <filesystem>

namespace rgb_base0 {

void writeCalibrationJson(const CalibrationData& calibration, const std::filesystem::path& path);
void writeCalibrationYaml(const CalibrationData& calibration, const std::filesystem::path& path);
CalibrationData readCalibration(const std::filesystem::path& path);
bool equivalentCalibration(const CalibrationData& left, const CalibrationData& right, double tolerance = 1e-9);

}  // namespace rgb_base0
