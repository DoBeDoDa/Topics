#pragma once

#include "rgb_base0/types.h"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace rgb_base0 {

struct CapturedFrame {
    std::filesystem::path path;
    std::uint64_t frameIndex = 0;
    std::uint64_t deviceTimestampUs = 0;
};

class OrbbecCamera final {
public:
    OrbbecCamera();
    ~OrbbecCamera() noexcept;

    OrbbecCamera(const OrbbecCamera&) = delete;
    OrbbecCamera& operator=(const OrbbecCamera&) = delete;

    // Copies live camera identity and profile only. Formal K/D always remains
    // owned by the fixed calibration JSON.
    void copyLiveCameraMetadataTo(CalibrationData& calibration) const;
    void requireMatches(const CalibrationData& stored) const;
    std::vector<CapturedFrame> captureMjpgFrames(const std::filesystem::path& directory,
                                                 int count,
                                                 int warmupFrames = 5);
    PixelRayDiagnostics pixelToUnitRay(double u,
                                       double v,
                                       const CalibrationData& calibration) const;
    std::vector<std::string> diagnosticLines() const;
    std::string profileDescription() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rgb_base0
