#include "rgb_base0/orbbec_camera.h"

#include "rgb_base0/brown_projection.h"
#include "rgb_base0/camera_profile.h"
#include "rgb_base0/geometry.h"

#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rgb_base0 {
namespace {

std::string sdkVersionString() {
    std::ostringstream version;
    version << ob::Version::getMajor() << '.' << ob::Version::getMinor() << '.' << ob::Version::getPatch();
    return version.str();
}

std::string boolText(const bool value) {
    return value ? "true" : "false";
}

std::string formatName(const OBFormat format) {
    return format == OB_FORMAT_MJPG ? "MJPG" : "OB_FORMAT_" + std::to_string(static_cast<int>(format));
}

bool distortionIsFinite(const OBCameraDistortion& distortion) {
    return std::isfinite(distortion.k1) && std::isfinite(distortion.k2)
           && std::isfinite(distortion.k3) && std::isfinite(distortion.k4)
           && std::isfinite(distortion.k5) && std::isfinite(distortion.k6)
           && std::isfinite(distortion.p1) && std::isfinite(distortion.p2);
}

void validateRgbCameraParameters(const OBCameraIntrinsic& intrinsic,
                                 const OBCameraDistortion& distortion,
                                 const CameraProfileData& liveProfile) {
    const bool fxFinite = std::isfinite(intrinsic.fx);
    const bool fyFinite = std::isfinite(intrinsic.fy);
    const bool focalLengthsPositive = intrinsic.fx > 0.0f && intrinsic.fy > 0.0f;
    const bool principalPointFinite = std::isfinite(intrinsic.cx) && std::isfinite(intrinsic.cy);
    const bool principalPointValid = principalPointFinite
                                     && intrinsic.cx >= 0.0f && intrinsic.cx < liveProfile.width
                                     && intrinsic.cy >= 0.0f && intrinsic.cy < liveProfile.height;
    const bool intrinsicSizeMatches = intrinsic.width == liveProfile.width
                                      && intrinsic.height == liveProfile.height;
    const bool distortionFinite = distortionIsFinite(distortion);
    if(!fxFinite || !fyFinite || !focalLengthsPositive || !principalPointValid
       || !intrinsicSizeMatches || !distortionFinite) {
        std::ostringstream message;
        message << "Invalid RGB camera parameters from Pipeline::getCameraParam():"
                << " fx_finite=" << boolText(fxFinite)
                << " fy_finite=" << boolText(fyFinite)
                << " focal_lengths_positive=" << boolText(focalLengthsPositive)
                << " principal_point_valid=" << boolText(principalPointValid)
                << " intrinsic_size_matches_live_frame=" << boolText(intrinsicSizeMatches)
                << " distortion_finite=" << boolText(distortionFinite);
        throw std::runtime_error(message.str());
    }
}

}  // namespace

class OrbbecCamera::Impl {
public:
    Impl() {
        if(sdkVersionString() != "1.10.18") {
            throw std::runtime_error("Loaded Orbbec SDK is " + sdkVersionString() + "; exact v1.10.18 is required");
        }
        context = std::make_unique<ob::Context>();
        const std::shared_ptr<ob::DeviceList> devices = context->queryDeviceList();
        const std::uint32_t count = devices ? devices->deviceCount() : 0;
        requireExactlyOneOrbbecDevice(count);
        device = devices->getDevice(0);
        if(!device) {
            throw std::runtime_error("Orbbec device handle is null");
        }
        info = device->getDeviceInfo();
        if(!info) {
            throw std::runtime_error("Orbbec device information is unavailable");
        }
        deviceName = info->name() ? info->name() : "";
        serialNumber = info->serialNumber() ? info->serialNumber() : "";
        firmwareVersion = info->firmwareVersion() ? info->firmwareVersion() : "";
        validateGemini2XlIdentity(deviceName, serialNumber);

        pipeline = std::make_unique<ob::Pipeline>(device);
        const std::shared_ptr<ob::StreamProfileList> profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
        if(!profiles) {
            throw std::runtime_error("Gemini 2 XL returned no color stream profile list");
        }
        std::vector<std::shared_ptr<ob::VideoStreamProfile>> videoProfiles;
        std::vector<CameraProfileData> profileData;
        for(std::uint32_t index = 0; index < profiles->count(); ++index) {
            const std::shared_ptr<ob::StreamProfile> generic = profiles->getProfile(index);
            if(!generic || !generic->is<ob::VideoStreamProfile>()) {
                continue;
            }
            const std::shared_ptr<ob::VideoStreamProfile> video = generic->as<ob::VideoStreamProfile>();
            videoProfiles.push_back(video);
            profileData.push_back({static_cast<int>(video->width()), static_cast<int>(video->height()),
                                   static_cast<int>(video->fps()),
                                   video->format() == OB_FORMAT_MJPG
                                       ? "MJPG"
                                       : "OB_FORMAT_" + std::to_string(static_cast<int>(video->format()))});
        }
        selectedProfile = videoProfiles.at(selectRequiredColorProfile(profileData));

        config = std::make_shared<ob::Config>();
        config->disableAllStream();
        config->enableStream(selectedProfile);
        pipeline->start(config);
        started = true;
        try {
            const auto frames = pipeline->waitForFrames(3000);
            const auto color = frames ? frames->colorFrame() : nullptr;
            if(!color) {
                throw std::runtime_error("Timed out waiting for the first Gemini 2 XL Color frame");
            }
            if(color->width() != 1280 || color->height() != 720
               || color->format() != selectedProfile->format()
               || color->width() != selectedProfile->width()
               || color->height() != selectedProfile->height()) {
                std::ostringstream message;
                message << "Live Color frame does not match selected profile: selected="
                        << selectedProfile->width() << 'x' << selectedProfile->height() << '@'
                        << selectedProfile->fps() << ' ' << formatName(selectedProfile->format())
                        << " actual=" << color->width() << 'x' << color->height() << ' '
                        << formatName(color->format());
                throw std::runtime_error(message.str());
            }
            actualProfile = {static_cast<int>(color->width()), static_cast<int>(color->height()),
                             static_cast<int>(selectedProfile->fps()), formatName(color->format())};

            const OBCameraParam cameraParameters = pipeline->getCameraParam();
            intrinsic = cameraParameters.rgbIntrinsic;
            distortion = cameraParameters.rgbDistortion;
            validateRgbCameraParameters(intrinsic, distortion, actualProfile);
        }
        catch(...) {
            stopNoThrow();
            throw;
        }
    }

    ~Impl() noexcept {
        stopNoThrow();
    }

    void stopNoThrow() noexcept {
        if(started && pipeline) {
            try {
                pipeline->stop();
            }
            catch(...) {
            }
            started = false;
        }
    }

    std::unique_ptr<ob::Context> context;
    std::shared_ptr<ob::Device> device;
    std::shared_ptr<ob::DeviceInfo> info;
    std::unique_ptr<ob::Pipeline> pipeline;
    std::shared_ptr<ob::Config> config;
    std::shared_ptr<ob::VideoStreamProfile> selectedProfile;
    CameraProfileData actualProfile;
    OBCameraIntrinsic intrinsic{};
    OBCameraDistortion distortion{};
    std::string deviceName;
    std::string serialNumber;
    std::string firmwareVersion;
    bool started = false;
};

OrbbecCamera::OrbbecCamera() : impl_(std::make_unique<Impl>()) {}
OrbbecCamera::~OrbbecCamera() noexcept = default;

void OrbbecCamera::copyLiveCameraMetadataTo(CalibrationData& value) const {
    value.sdkVersion = sdkVersionString();
    value.deviceName = impl_->deviceName;
    value.serialNumber = impl_->serialNumber;
    value.firmwareVersion = impl_->firmwareVersion;
    value.profile = impl_->actualProfile;
}

void OrbbecCamera::requireMatches(const CalibrationData& stored) const {
    CalibrationData live = stored;
    copyLiveCameraMetadataTo(live);
    if(live.sdkVersion != stored.sdkVersion || live.deviceName != stored.deviceName
       || live.serialNumber != stored.serialNumber || live.firmwareVersion != stored.firmwareVersion
       || live.profile.width != stored.profile.width
       || live.profile.height != stored.profile.height || live.profile.fps != stored.profile.fps
       || live.profile.format != stored.profile.format) {
        std::ostringstream message;
        message << "Live camera/profile does not match calibration. Stored serial/profile=" << stored.serialNumber << ' '
                << stored.profile.width << 'x' << stored.profile.height << '@' << stored.profile.fps << ' '
                << stored.profile.format << "; live=" << live.serialNumber << ' ' << live.profile.width << 'x'
                << live.profile.height << '@' << live.profile.fps << ' ' << live.profile.format;
        throw std::runtime_error(message.str());
    }
}

std::vector<CapturedFrame> OrbbecCamera::captureMjpgFrames(const std::filesystem::path& directory,
                                                           const int count,
                                                           const int warmupFrames) {
    if(count <= 0 || warmupFrames < 0) {
        throw std::invalid_argument("Frame count must be positive and warmup frame count non-negative");
    }
    std::filesystem::create_directories(directory);
    for(int index = 0; index < warmupFrames; ++index) {
        const auto frames = impl_->pipeline->waitForFrames(3000);
        const auto color = frames ? frames->colorFrame() : nullptr;
        if(!color) {
            throw std::runtime_error("Timed out while warming up Gemini 2 XL RGB stream");
        }
        if(color->format() != impl_->selectedProfile->format()
           || static_cast<int>(color->width()) != impl_->actualProfile.width
           || static_cast<int>(color->height()) != impl_->actualProfile.height) {
            throw std::runtime_error("Warm-up Color frame no longer matches the selected live profile");
        }
    }
    std::vector<CapturedFrame> result;
    result.reserve(static_cast<std::size_t>(count));
    for(int index = 0; index < count; ++index) {
        const auto frames = impl_->pipeline->waitForFrames(3000);
        const auto color = frames ? frames->colorFrame() : nullptr;
        if(!color) {
            throw std::runtime_error("Timed out waiting for Gemini 2 XL RGB frame " + std::to_string(index));
        }
        if(color->format() != impl_->selectedProfile->format()
           || static_cast<int>(color->width()) != impl_->actualProfile.width
           || static_cast<int>(color->height()) != impl_->actualProfile.height) {
            throw std::runtime_error("Received frame does not match selected 1280x720 MJPG profile");
        }
        if(color->data() == nullptr || color->dataSize() == 0) {
            throw std::runtime_error("Received an empty MJPG frame");
        }
        std::ostringstream filename;
        filename << "frame_" << std::setw(2) << std::setfill('0') << index << ".jpg";
        const std::filesystem::path path = directory / filename.str();
        std::ofstream output(path, std::ios::binary);
        if(!output) {
            throw std::runtime_error("Cannot create raw frame: " + path.string());
        }
        output.write(static_cast<const char*>(color->data()), static_cast<std::streamsize>(color->dataSize()));
        output.flush();
        if(!output) {
            throw std::runtime_error("Failed writing raw MJPG frame: " + path.string());
        }
        result.push_back({path, color->index(), color->timeStampUs()});
    }
    return result;
}

PixelRayDiagnostics OrbbecCamera::pixelToUnitRay(const double u,
                                                  const double v,
                                                  const CalibrationData& calibration) const {
    return inverseProjectBrownPixel(u, v, calibration.profile, calibration.intrinsic,
                                    calibration.distortion);
}

std::vector<std::string> OrbbecCamera::diagnosticLines() const {
    std::vector<std::string> lines;
    lines.push_back("[CAMERA] selected_color_profile="
                    + std::to_string(impl_->selectedProfile->width()) + "x"
                    + std::to_string(impl_->selectedProfile->height()) + " "
                    + formatName(impl_->selectedProfile->format()) + " "
                    + std::to_string(impl_->selectedProfile->fps()) + " FPS");
    lines.push_back("[CAMERA] actual_color_frame=" + std::to_string(impl_->actualProfile.width) + "x"
                    + std::to_string(impl_->actualProfile.height) + " format=" + impl_->actualProfile.format);
    lines.push_back("[SDK FACTORY RGB INTRINSIC / DIAGNOSTIC ONLY]");
    lines.push_back("fx=" + std::to_string(impl_->intrinsic.fx));
    lines.push_back("fy=" + std::to_string(impl_->intrinsic.fy));
    lines.push_back("cx=" + std::to_string(impl_->intrinsic.cx));
    lines.push_back("cy=" + std::to_string(impl_->intrinsic.cy));
    lines.push_back("[SDK FACTORY RGB DISTORTION / DIAGNOSTIC ONLY]");
    lines.push_back("k1=" + std::to_string(impl_->distortion.k1));
    lines.push_back("k2=" + std::to_string(impl_->distortion.k2));
    lines.push_back("k3=" + std::to_string(impl_->distortion.k3));
    lines.push_back("k4=" + std::to_string(impl_->distortion.k4));
    lines.push_back("k5=" + std::to_string(impl_->distortion.k5));
    lines.push_back("k6=" + std::to_string(impl_->distortion.k6));
    lines.push_back("p1=" + std::to_string(impl_->distortion.p1));
    lines.push_back("p2=" + std::to_string(impl_->distortion.p2));
    lines.push_back("[CHECK]");
    lines.push_back("fx_finite=" + boolText(std::isfinite(impl_->intrinsic.fx)));
    lines.push_back("fy_finite=" + boolText(std::isfinite(impl_->intrinsic.fy)));
    lines.push_back("focal_lengths_positive="
                    + boolText(impl_->intrinsic.fx > 0.0f && impl_->intrinsic.fy > 0.0f));
    lines.push_back("principal_point_valid="
                    + boolText(std::isfinite(impl_->intrinsic.cx) && std::isfinite(impl_->intrinsic.cy)
                               && impl_->intrinsic.cx >= 0.0f && impl_->intrinsic.cx < impl_->actualProfile.width
                               && impl_->intrinsic.cy >= 0.0f && impl_->intrinsic.cy < impl_->actualProfile.height));
    lines.push_back("intrinsic_size_matches_live_frame="
                    + boolText(impl_->intrinsic.width == impl_->actualProfile.width
                               && impl_->intrinsic.height == impl_->actualProfile.height));
    lines.push_back("distortion_finite=" + boolText(distortionIsFinite(impl_->distortion)));
    return lines;
}

std::string OrbbecCamera::profileDescription() const {
    std::ostringstream description;
    description << impl_->deviceName << " serial=" << impl_->serialNumber << " firmware=" << impl_->firmwareVersion
                << " SDK=" << sdkVersionString() << " profile=" << impl_->selectedProfile->width() << 'x'
                << impl_->selectedProfile->height() << '@' << impl_->selectedProfile->fps()
                << " " << formatName(impl_->selectedProfile->format())
                << " RGB-only K/D=Pipeline::getCameraParam().rgb";
    return description.str();
}

}  // namespace rgb_base0
