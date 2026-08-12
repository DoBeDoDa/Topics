#include "rgb_base0/calibration_io.h"
#include "rgb_base0/geometry.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

struct GeometryHandle {
    rgb_base0::CalibrationData calibration;
};

void copyError(const std::string& message, char* output, const std::size_t capacity) noexcept {
    if(output == nullptr || capacity == 0) {
        return;
    }
    const std::size_t length = std::min(message.size(), capacity - 1);
    std::memcpy(output, message.data(), length);
    output[length] = '\0';
}

template <typename Operation>
int protect(Operation operation, char* error, const std::size_t errorCapacity) noexcept {
    try {
        operation();
        copyError("", error, errorCapacity);
        return 1;
    }
    catch(const std::exception& exception) {
        copyError(exception.what(), error, errorCapacity);
        return 0;
    }
    catch(...) {
        copyError("Unknown RGB-to-Base0 geometry bridge error", error, errorCapacity);
        return 0;
    }
}

GeometryHandle& requireHandle(void* opaqueHandle) {
    if(opaqueHandle == nullptr) {
        throw std::runtime_error("RGB-to-Base0 geometry handle is null");
    }
    return *static_cast<GeometryHandle*>(opaqueHandle);
}

}  // namespace

extern "C" {

__declspec(dllexport) int __cdecl rgb_base0_geometry_create(
    const wchar_t* calibrationPath,
    void** outputHandle,
    char* error,
    const std::size_t errorCapacity) noexcept {
    return protect([&]() {
        if(calibrationPath == nullptr || calibrationPath[0] == L'\0') {
            throw std::runtime_error("Current calibration path is empty");
        }
        if(outputHandle == nullptr) {
            throw std::runtime_error("Geometry output handle pointer is null");
        }
        *outputHandle = nullptr;
        auto handle = std::make_unique<GeometryHandle>();
        handle->calibration = rgb_base0::readCalibration(std::filesystem::path(calibrationPath));
        rgb_base0::validateCalibration(handle->calibration);
        *outputHandle = handle.release();
    }, error, errorCapacity);
}

__declspec(dllexport) void __cdecl rgb_base0_geometry_destroy(void* opaqueHandle) noexcept {
    delete static_cast<GeometryHandle*>(opaqueHandle);
}

__declspec(dllexport) int __cdecl rgb_base0_geometry_profile(
    void* opaqueHandle,
    int* width,
    int* height,
    int* fps,
    double* targetZMm,
    char* serialNumber,
    const std::size_t serialCapacity,
    char* error,
    const std::size_t errorCapacity) noexcept {
    return protect([&]() {
        GeometryHandle& handle = requireHandle(opaqueHandle);
        if(width == nullptr || height == nullptr || fps == nullptr || targetZMm == nullptr) {
            throw std::runtime_error("Geometry profile output pointer is null");
        }
        *width = handle.calibration.profile.width;
        *height = handle.calibration.profile.height;
        *fps = handle.calibration.profile.fps;
        *targetZMm = handle.calibration.zTableMm + handle.calibration.ballRadiusMm;
        if(serialNumber == nullptr || serialCapacity == 0) {
            throw std::runtime_error("Camera serial output buffer is empty");
        }
        copyError(handle.calibration.serialNumber, serialNumber, serialCapacity);
        if(handle.calibration.serialNumber.size() >= serialCapacity) {
            throw std::runtime_error("Camera serial output buffer is too small");
        }
    }, error, errorCapacity);
}

__declspec(dllexport) int __cdecl rgb_base0_geometry_project(
    void* opaqueHandle,
    const double u,
    const double v,
    double* base0X,
    double* base0Y,
    double* base0Z,
    char* error,
    const std::size_t errorCapacity) noexcept {
    return protect([&]() {
        GeometryHandle& handle = requireHandle(opaqueHandle);
        if(base0X == nullptr || base0Y == nullptr || base0Z == nullptr) {
            throw std::runtime_error("Geometry projection output pointer is null");
        }
        const rgb_base0::PlaneIntersection intersection =
            rgb_base0::projectRgbPixelToBallCenterPlane(handle.calibration, u, v);
        *base0X = intersection.pointBase0.x;
        *base0Y = intersection.pointBase0.y;
        *base0Z = intersection.pointBase0.z;
    }, error, errorCapacity);
}

}  // extern "C"
