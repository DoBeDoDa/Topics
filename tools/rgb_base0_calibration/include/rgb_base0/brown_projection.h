#pragma once

#include "rgb_base0/types.h"

namespace rgb_base0 {

struct BrownInverseOptions {
    int maxIterations = kBrownInverseMaxIterations;
    double convergenceTolerance = kBrownInverseConvergenceTolerance;
    double reprojectionTolerancePx = kBrownReprojectionTolerancePx;
};

Vec2 distortBrownRational(const Vec2& undistorted,
                          const CameraDistortionData& distortion);

PixelRayDiagnostics inverseProjectBrownPixel(double u,
                                             double v,
                                             const CameraProfileData& profile,
                                             const CameraIntrinsicData& intrinsic,
                                             const CameraDistortionData& distortion,
                                             const BrownInverseOptions& options = {});

}  // namespace rgb_base0
