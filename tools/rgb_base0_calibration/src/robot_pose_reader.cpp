#include "rgb_base0/robot_pose_reader.h"

#include "rgb_base0/geometry.h"

#include "HRSDK.h"

#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace rgb_base0 {
namespace {

void __stdcall emptyCallback(uint16_t, uint16_t, uint16_t*, int) {}

void requireSdkSuccess(const int code, const std::string& operation) {
    if(code != 0) {
        throw std::runtime_error(operation + " failed with HRSDK code " + std::to_string(code));
    }
}

}  // namespace

RobotPoseReader::RobotPoseReader(std::string ipAddress) : ipAddress_(std::move(ipAddress)) {
    handle_ = open_connection(ipAddress_.c_str(), 1, emptyCallback);
    if(handle_ < 0) {
        throw std::runtime_error("Failed to connect to HIWIN controller at " + ipAddress_);
    }
    try {
        originalToolNumber_ = get_tool_number(handle_);
        originalBaseNumber_ = get_base_number(handle_);
        if(originalToolNumber_ < 0 || originalBaseNumber_ < 0) {
            throw std::runtime_error("Failed to read original Tool/Base numbers");
        }
        requireSdkSuccess(set_tool_number(handle_, 3), "set_tool_number(3)");
        requireSdkSuccess(set_base_number(handle_, 0), "set_base_number(0)");
        if(get_tool_number(handle_) != 3 || get_base_number(handle_) != 0) {
            throw std::runtime_error("Controller did not confirm Tool3/Base0 after setting them");
        }
    }
    catch(...) {
        const std::exception_ptr originalFailure = std::current_exception();
        try {
            restoreAndClose();
        }
        catch(const std::exception& cleanupError) {
            throw std::runtime_error(std::string("Robot session initialization failed and Tool/Base cleanup also failed: ")
                                     + cleanupError.what());
        }
        std::rethrow_exception(originalFailure);
    }
}

RobotPoseReader::~RobotPoseReader() noexcept {
    restoreNoThrow();
}

RobotPoseCapture RobotPoseReader::captureStablePose(const int sampleCount,
                                                    const int sampleWindowMs,
                                                    const double xyzToleranceMm,
                                                    const double abcToleranceDeg,
                                                    const PoseSampleObserver& sampleObserver) {
    if(handle_ < 0) {
        throw std::runtime_error("Robot pose reader is closed");
    }
    if(sampleCount < 2 || sampleWindowMs < 0) {
        throw std::invalid_argument("Robot sampling requires at least 2 samples and a non-negative window");
    }
    if(get_tool_number(handle_) != 3 || get_base_number(handle_) != 0) {
        throw std::runtime_error("Tool/Base changed during capture; expected Tool3/Base0");
    }
    RobotPoseCapture capture;
    capture.samples.reserve(static_cast<std::size_t>(sampleCount));
    capture.motionStateRaw.reserve(static_cast<std::size_t>(sampleCount));
    const auto delay = std::chrono::milliseconds(sampleWindowMs / (sampleCount - 1));
    for(int index = 0; index < sampleCount; ++index) {
        const int motionStateRaw = get_motion_state(handle_);
        std::array<double, 6> pose{};
        requireSdkSuccess(get_current_position(handle_, pose.data()), "get_current_position");
        capture.samples.push_back({pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]});
        capture.motionStateRaw.push_back(motionStateRaw);
        if(sampleObserver) {
            sampleObserver(index, motionStateRaw, capture.samples.back());
        }
        if(index + 1 < sampleCount) {
            std::this_thread::sleep_for(delay);
        }
    }
    std::string reason;
    if(!robotPoseStable(capture.samples, xyzToleranceMm, abcToleranceDeg, &reason)) {
        throw std::runtime_error("Robot pose stability check failed: " + reason);
    }
    capture.mean = meanRobotPose(capture.samples);
    return capture;
}

void RobotPoseReader::requireSamePose(const RobotPoseCapture& before,
                                      const RobotPoseCapture& after,
                                      const double xyzToleranceMm,
                                      const double abcToleranceDeg) const {
    const std::array<double, 3> beforeXyz{before.mean.x, before.mean.y, before.mean.z};
    const std::array<double, 3> afterXyz{after.mean.x, after.mean.y, after.mean.z};
    const char* labels[] = {"X", "Y", "Z", "A", "B", "C"};
    for(std::size_t index = 0; index < beforeXyz.size(); ++index) {
        const double difference = std::abs(afterXyz[index] - beforeXyz[index]);
        if(difference > xyzToleranceMm) {
            std::ostringstream message;
            message << "Robot pose changed across camera capture: " << labels[index] << " changed " << difference
                    << " mm (limit " << xyzToleranceMm << " mm)";
            throw std::runtime_error(message.str());
        }
    }
    const std::array<double, 3> beforeAbc{before.mean.a, before.mean.b, before.mean.c};
    const std::array<double, 3> afterAbc{after.mean.a, after.mean.b, after.mean.c};
    for(std::size_t index = 0; index < beforeAbc.size(); ++index) {
        const double difference = std::abs(shortestWrappedDifferenceDegrees(afterAbc[index], beforeAbc[index]));
        if(difference > abcToleranceDeg) {
            std::ostringstream message;
            message << "Robot pose changed across camera capture: " << labels[index + 3] << " changed " << difference
                    << " deg (limit " << abcToleranceDeg << " deg)";
            throw std::runtime_error(message.str());
        }
    }
}

void RobotPoseReader::restoreAndClose() {
    if(handle_ < 0) {
        return;
    }
    std::string failures;
    if(originalToolNumber_ >= 0) {
        const int code = set_tool_number(handle_, originalToolNumber_);
        if(code != 0 || get_tool_number(handle_) != originalToolNumber_) {
            failures += " failed to restore original Tool" + std::to_string(originalToolNumber_);
        }
    }
    if(originalBaseNumber_ >= 0) {
        const int code = set_base_number(handle_, originalBaseNumber_);
        if(code != 0 || get_base_number(handle_) != originalBaseNumber_) {
            failures += " failed to restore original Base" + std::to_string(originalBaseNumber_);
        }
    }
    close_connection(handle_);
    handle_ = -1;
    if(!failures.empty()) {
        throw std::runtime_error("Controller cleanup:" + failures + "; connection was closed");
    }
}

void RobotPoseReader::restoreNoThrow() noexcept {
    if(handle_ < 0) {
        return;
    }
    if(originalToolNumber_ >= 0) {
        static_cast<void>(set_tool_number(handle_, originalToolNumber_));
    }
    if(originalBaseNumber_ >= 0) {
        static_cast<void>(set_base_number(handle_, originalBaseNumber_));
    }
    close_connection(handle_);
    handle_ = -1;
}

}  // namespace rgb_base0
