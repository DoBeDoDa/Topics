#pragma once

#include "rgb_base0/types.h"

#include <string>
#include <vector>

namespace rgb_base0 {

struct RobotPoseCapture {
    std::vector<RobotPose> samples;
    RobotPose mean;
};

class RobotPoseReader final {
public:
    explicit RobotPoseReader(std::string ipAddress);
    ~RobotPoseReader() noexcept;

    RobotPoseReader(const RobotPoseReader&) = delete;
    RobotPoseReader& operator=(const RobotPoseReader&) = delete;

    RobotPoseCapture captureStablePose(int sampleCount = 3,
                                       int sampleWindowMs = 500,
                                       double xyzToleranceMm = 0.1,
                                       double abcToleranceDeg = 0.05);
    void requireSamePose(const RobotPoseCapture& before,
                         const RobotPoseCapture& after,
                         double xyzToleranceMm = 0.1,
                         double abcToleranceDeg = 0.05) const;
    void restoreAndClose();

    int originalToolNumber() const noexcept { return originalToolNumber_; }
    int originalBaseNumber() const noexcept { return originalBaseNumber_; }

private:
    void restoreNoThrow() noexcept;

    std::string ipAddress_;
    int handle_ = -1;
    int originalToolNumber_ = -1;
    int originalBaseNumber_ = -1;
};

}  // namespace rgb_base0
