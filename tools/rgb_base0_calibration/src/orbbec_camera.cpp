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

bool near(const double left, const double right, const double absoluteTolerance = 1e-6) {
    return std::abs(left - right) <= absoluteTolerance * std::max({1.0, std::abs(left), std::abs(right)});
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
        intrinsic = selectedProfile->getIntrinsic();
        distortion = selectedProfile->getDistortion();
        if(intrinsic.width != 1280 || intrinsic.height != 720 || intrinsic.fx <= 0.0f || intrinsic.fy <= 0.0f) {
            throw std::runtime_error("Selected live RGB intrinsic is inconsistent with 1280x720 profile");
        }
        pipeline->start(config);
        started = true;
    }

    ~Impl() noexcept {
        if(started && pipeline) {
            try {
                pipeline->stop();
            }
            catch(...) {
            }
        }
    }

    std::unique_ptr<ob::Context> context;
    std::shared_ptr<ob::Device> device;
    std::shared_ptr<ob::DeviceInfo> info;
    std::unique_ptr<ob::Pipeline> pipeline;
    std::shared_ptr<ob::Config> config;
    std::shared_ptr<ob::VideoStreamProfile> selectedProfile;
    OBCameraIntrinsic intrinsic{};
    OBCameraDistortion distortion{};
    std::string deviceName;
    std::string serialNumber;
    std::string firmwareVersion;
    bool started = false;
};

OrbbecCamera::OrbbecCamera() : impl_(std::make_unique<Impl>()) {}
OrbbecCamera::~OrbbecCamera() noexcept = default;

void OrbbecCamera::copyCameraFieldsTo(CalibrationData& value) const {
    value.sdkVersion = sdkVersionString();
    value.deviceName = impl_->deviceName;
    value.serialNumber = impl_->serialNumber;
    value.firmwareVersion = impl_->firmwareVersion;
    value.profile = {static_cast<int>(impl_->selectedProfile->width()),
                     static_cast<int>(impl_->selectedProfile->height()),
                     static_cast<int>(impl_->selectedProfile->fps()), "MJPG"};
    value.intrinsic = {impl_->intrinsic.fx, impl_->intrinsic.fy, impl_->intrinsic.cx, impl_->intrinsic.cy};
    value.distortion = {impl_->distortion.k1, impl_->distortion.k2, impl_->distortion.k3, impl_->distortion.k4,
                        impl_->distortion.k5, impl_->distortion.k6, impl_->distortion.p1, impl_->distortion.p2};
}

void OrbbecCamera::requireMatches(const CalibrationData& stored) const {
    CalibrationData live = stored;
    copyCameraFieldsTo(live);
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
    const std::vector<std::pair<double, double>> pairs{
        {stored.intrinsic.fx, live.intrinsic.fx}, {stored.intrinsic.fy, live.intrinsic.fy},
        {stored.intrinsic.cx, live.intrinsic.cx}, {stored.intrinsic.cy, live.intrinsic.cy},
        {stored.distortion.k1, live.distortion.k1}, {stored.distortion.k2, live.distortion.k2},
        {stored.distortion.k3, live.distortion.k3}, {stored.distortion.k4, live.distortion.k4},
        {stored.distortion.k5, live.distortion.k5}, {stored.distortion.k6, live.distortion.k6},
        {stored.distortion.p1, live.distortion.p1}, {stored.distortion.p2, live.distortion.p2},
    };
    for(const auto& pair : pairs) {
        if(!near(pair.first, pair.second)) {
            throw std::runtime_error("Live RGB intrinsic/distortion does not match the stored calibration");
        }
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
        if(!frames || !frames->colorFrame()) {
            throw std::runtime_error("Timed out while warming up Gemini 2 XL RGB stream");
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
        if(color->format() != OB_FORMAT_MJPG || color->width() != 1280 || color->height() != 720) {
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

PixelRayDiagnostics OrbbecCamera::pixelToUnitRay(const double u, const double v) const {
    const CameraProfileData profile{static_cast<int>(impl_->selectedProfile->width()),
                                    static_cast<int>(impl_->selectedProfile->height()),
                                    static_cast<int>(impl_->selectedProfile->fps()), "MJPG"};
    const CameraIntrinsicData intrinsic{impl_->intrinsic.fx, impl_->intrinsic.fy,
                                        impl_->intrinsic.cx, impl_->intrinsic.cy};
    const CameraDistortionData distortion{
        impl_->distortion.k1, impl_->distortion.k2, impl_->distortion.k3, impl_->distortion.k4,
        impl_->distortion.k5, impl_->distortion.k6, impl_->distortion.p1, impl_->distortion.p2,
    };
    return inverseProjectBrownPixel(u, v, profile, intrinsic, distortion);
}

std::string OrbbecCamera::profileDescription() const {
    std::ostringstream description;
    description << impl_->deviceName << " serial=" << impl_->serialNumber << " firmware=" << impl_->firmwareVersion
                << " SDK=" << sdkVersionString() << " profile=" << impl_->selectedProfile->width() << 'x'
                << impl_->selectedProfile->height() << '@' << impl_->selectedProfile->fps()
                << " MJPG RGB-only";
    return description.str();
}

}  // namespace rgb_base0
