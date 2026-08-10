#include "rgb_base0/brown_projection.h"

#include "rgb_base0/geometry.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace rgb_base0 {
namespace {

bool finite(const double value) {
    return std::isfinite(value) != 0;
}

void requireFinite(const Vec2& value, const char* label) {
    if(!finite(value.x) || !finite(value.y)) {
        throw std::runtime_error(std::string(label) + " contains a non-finite value");
    }
}

double residualNorm(const Vec2& left, const Vec2& right) {
    return std::hypot(left.x - right.x, left.y - right.y);
}

}  // namespace

Vec2 distortBrownRational(const Vec2& undistorted,
                          const CameraDistortionData& distortion) {
    requireFinite(undistorted, "Brown input");
    const double x = undistorted.x;
    const double y = undistorted.y;
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double numerator = 1.0 + distortion.k1 * r2 + distortion.k2 * r4 + distortion.k3 * r6;
    const double denominator = 1.0 + distortion.k4 * r2 + distortion.k5 * r4 + distortion.k6 * r6;
    if(!finite(numerator) || !finite(denominator) || std::abs(denominator) <= 1e-12) {
        throw std::runtime_error("Brown radial model has a non-finite or near-zero denominator");
    }
    const double radial = numerator / denominator;
    if(!finite(radial)) {
        throw std::runtime_error("Brown radial scale is non-finite");
    }
    const double twoXy = 2.0 * x * y;
    const Vec2 distorted{
        x * radial + distortion.p1 * twoXy + distortion.p2 * (r2 + 2.0 * x * x),
        y * radial + distortion.p1 * (r2 + 2.0 * y * y) + distortion.p2 * twoXy,
    };
    requireFinite(distorted, "Brown output");
    return distorted;
}

PixelRayDiagnostics inverseProjectBrownPixel(const double u,
                                             const double v,
                                             const CameraProfileData& profile,
                                             const CameraIntrinsicData& intrinsic,
                                             const CameraDistortionData& distortion,
                                             const BrownInverseOptions& options) {
    if(profile.width <= 0 || profile.height <= 0 || !finite(u) || !finite(v)
       || u < 0.0 || v < 0.0 || u >= static_cast<double>(profile.width)
       || v >= static_cast<double>(profile.height)) {
        throw std::runtime_error("RGB pixel is non-finite or outside the raw color image bounds");
    }
    if(!finite(intrinsic.fx) || !finite(intrinsic.fy) || !finite(intrinsic.cx) || !finite(intrinsic.cy)
       || intrinsic.fx <= 0.0 || intrinsic.fy <= 0.0) {
        throw std::runtime_error("RGB intrinsic is non-finite or has a non-positive focal length");
    }
    if(options.maxIterations <= 0 || !finite(options.convergenceTolerance)
       || !finite(options.reprojectionTolerancePx) || options.convergenceTolerance <= 0.0
       || options.reprojectionTolerancePx <= 0.0) {
        throw std::runtime_error("Brown inverse options are invalid");
    }

    const Vec2 target{(u - intrinsic.cx) / intrinsic.fx,
                      (v - intrinsic.cy) / intrinsic.fy};
    requireFinite(target, "Distorted normalized pixel");
    Vec2 estimate = target;
    Vec2 forward{};
    double normalizedResidual = 0.0;
    int iterations = 0;
    bool converged = false;

    for(int iteration = 0; iteration <= options.maxIterations; ++iteration) {
        forward = distortBrownRational(estimate, distortion);
        normalizedResidual = residualNorm(forward, target);
        if(!finite(normalizedResidual)) {
            throw std::runtime_error("Brown inverse residual is non-finite");
        }
        if(normalizedResidual <= options.convergenceTolerance) {
            iterations = iteration;
            converged = true;
            break;
        }
        if(iteration == options.maxIterations) {
            break;
        }

        const double stepX = 1e-7 * std::max(1.0, std::abs(estimate.x));
        const double stepY = 1e-7 * std::max(1.0, std::abs(estimate.y));
        const Vec2 forwardPlusX = distortBrownRational({estimate.x + stepX, estimate.y}, distortion);
        const Vec2 forwardMinusX = distortBrownRational({estimate.x - stepX, estimate.y}, distortion);
        const Vec2 forwardPlusY = distortBrownRational({estimate.x, estimate.y + stepY}, distortion);
        const Vec2 forwardMinusY = distortBrownRational({estimate.x, estimate.y - stepY}, distortion);
        const double j00 = (forwardPlusX.x - forwardMinusX.x) / (2.0 * stepX);
        const double j10 = (forwardPlusX.y - forwardMinusX.y) / (2.0 * stepX);
        const double j01 = (forwardPlusY.x - forwardMinusY.x) / (2.0 * stepY);
        const double j11 = (forwardPlusY.y - forwardMinusY.y) / (2.0 * stepY);
        const double determinant = j00 * j11 - j01 * j10;
        if(!finite(determinant) || std::abs(determinant) <= 1e-14) {
            throw std::runtime_error("Brown inverse Jacobian is singular or non-finite");
        }
        const double residualX = target.x - forward.x;
        const double residualY = target.y - forward.y;
        const Vec2 correction{
            (j11 * residualX - j01 * residualY) / determinant,
            (-j10 * residualX + j00 * residualY) / determinant,
        };
        requireFinite(correction, "Brown inverse correction");
        if(std::hypot(correction.x, correction.y) > 4.0) {
            throw std::runtime_error("Brown inverse correction diverged");
        }
        estimate.x += correction.x;
        estimate.y += correction.y;
        if(!finite(estimate.x) || !finite(estimate.y)
           || std::abs(estimate.x) > 10.0 || std::abs(estimate.y) > 10.0) {
            throw std::runtime_error("Brown inverse estimate diverged outside the supported domain");
        }
    }

    if(!converged) {
        std::ostringstream message;
        message << "Brown inverse did not converge in " << options.maxIterations
                << " iterations; final normalized residual=" << normalizedResidual;
        throw std::runtime_error(message.str());
    }

    const Vec2 reprojectedPixel{forward.x * intrinsic.fx + intrinsic.cx,
                                forward.y * intrinsic.fy + intrinsic.cy};
    requireFinite(reprojectedPixel, "Brown reprojected pixel");
    const double reprojectionError = std::hypot(reprojectedPixel.x - u, reprojectedPixel.y - v);
    if(!finite(reprojectionError) || reprojectionError > options.reprojectionTolerancePx) {
        std::ostringstream message;
        message << "Brown inverse reprojection error " << reprojectionError
                << " px exceeds " << options.reprojectionTolerancePx << " px";
        throw std::runtime_error(message.str());
    }

    PixelRayDiagnostics diagnostics;
    diagnostics.distortedNormalized = target;
    diagnostics.undistortedNormalized = estimate;
    diagnostics.reprojectedPixel = reprojectedPixel;
    diagnostics.unitRayRgb = normalize({estimate.x, estimate.y, 1.0});
    diagnostics.inverseIterations = iterations;
    diagnostics.finalNormalizedResidual = normalizedResidual;
    diagnostics.reprojectionErrorPx = reprojectionError;
    if(diagnostics.unitRayRgb.z <= 0.0
       || !finite(norm(diagnostics.unitRayRgb))
       || std::abs(norm(diagnostics.unitRayRgb) - 1.0) > 1e-12) {
        throw std::runtime_error("Brown inverse returned an invalid RGB optical unit ray");
    }
    return diagnostics;
}

}  // namespace rgb_base0
