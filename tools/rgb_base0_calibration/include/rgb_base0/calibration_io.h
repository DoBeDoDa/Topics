#pragma once

#include "rgb_base0/types.h"

#include <filesystem>

namespace rgb_base0 {

void writeCalibrationJson(const CalibrationData& calibration, const std::filesystem::path& path);
void writeCalibrationYaml(const CalibrationData& calibration, const std::filesystem::path& path);
CalibrationData readCalibration(const std::filesystem::path& path);
bool equivalentCalibration(const CalibrationData& left, const CalibrationData& right, double tolerance = 1e-9);
void publishCurrentCalibrationJson(const std::filesystem::path& verifiedHistoricalJson,
                                   const CalibrationData& verifiedCalibration,
                                   const std::filesystem::path& currentJson);

}  // namespace rgb_base0
