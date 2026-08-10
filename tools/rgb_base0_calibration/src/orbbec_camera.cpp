#include "rgb_base0/orbbec_camera.h"

#include "rgb_base0/geometry.h"

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>

#include <algorithm>
#include <cctype>
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

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::string sdkVersionString() {
    std::ostringstream version;
    version << ob::Version::getMajor() << '.' << ob::Version::getMinor() << '.' << ob::Version::getPatch();
    return version.str();
}

bool near(const double left, const double right, const double absoluteTolerance = 1e-6) {
    return std::abs(left - right) <= absoluteTolerance * std::max({1.0, std::abs(left), std::abs(right)});
}

Vec3 fromObPoint(const OBPoint3f& value) {
    return {value.x, value.y, value.z};
}

double distance(const Vec3& left, const Vec3& right) {
    return norm({left.x - right.x, left.y - right.y, left.z - right.z});
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
        if(count == 0) {
            throw std::runtime_error("Gemini 2 XL not found; Orbbec device count is zero");
        }
        if(count != 1) {
            throw std::runtime_error("Exactly one Orbbec device must be connected; detected " + std::to_string(count));
        }
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
        if(lower(deviceName).find("gemini 2 xl") == std::string::npos) {
            throw std::runtime_error("Connected Orbbec device is not identified as Gemini 2 XL: " + deviceName);
        }
        if(serialNumber.empty()) {
            throw std::runtime_error("Gemini 2 XL returned an empty serial number");
        }

        pipeline = std::make_unique<ob::Pipeline>(device);
        const std::shared_ptr<ob::StreamProfileList> profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
        if(!profiles) {
            throw std::runtime_error("Gemini 2 XL returned no color stream profile list");
        }
        for(std::uint32_t index = 0; index < profiles->count(); ++index) {
            const std::shared_ptr<ob::StreamProfile> generic = profiles->getProfile(index);
            if(!generic || !generic->is<ob::VideoStreamProfile>()) {
                continue;
            }
            const std::shared_ptr<ob::VideoStreamProfile> video = generic->as<ob::VideoStreamProfile>();
            std::ostringstream description;
            description << video->width() << 'x' << video->height() << " fps=" << video->fps()
                        << " format=" << static_cast<int>(video->format());
            allProfiles.push_back(description.str());
            if(video->width() == 1280 && video->height() == 720 && video->format() == OB_FORMAT_MJPG
               && (!selectedProfile || video->fps() > selectedProfile->fps())) {
                selectedProfile = video;
            }
        }
        if(!selectedProfile) {
            std::ostringstream message;
            message << "Required 1280x720 MJPG color profile is unavailable. Reported profiles:";
            for(const std::string& profile : allProfiles) {
                message << "\n  " << profile;
            }
            throw std::runtime_error(message.str());
        }

        const std::shared_ptr<ob::StreamProfileList> depthProfiles =
            pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
        if(!depthProfiles || depthProfiles->count() == 0) {
            throw std::runtime_error(
                "Gemini 2 XL returned no depth stream profile; a matched Color/Depth configuration is required "
                "to retrieve SDK calibration parameters");
        }
        try {
            selectedDepthProfile = depthProfiles->getVideoStreamProfile(
                640, OB_HEIGHT_ANY, OB_FORMAT_ANY, selectedProfile->fps());
        }
        catch(...) {
            selectedDepthProfile.reset();
        }
        if(!selectedDepthProfile) {
            const std::shared_ptr<ob::StreamProfile> fallback = depthProfiles->getProfile(OB_PROFILE_DEFAULT);
            if(fallback && fallback->is<ob::VideoStreamProfile>()) {
                selectedDepthProfile = fallback->as<ob::VideoStreamProfile>();
            }
        }
        if(!selectedDepthProfile) {
            throw std::runtime_error(
                "Gemini 2 XL has no usable depth profile for retrieving matched SDK calibration parameters");
        }

        config = std::make_shared<ob::Config>();
        config->disableAllStream();
        config->enableStream(selectedProfile);
        config->enableStream(selectedDepthProfile);
        config->setAlignMode(ALIGN_DISABLE);
        intrinsic = selectedProfile->getIntrinsic();
        distortion = selectedProfile->getDistortion();
        if(intrinsic.width != 1280 || intrinsic.height != 720 || intrinsic.fx <= 0.0f || intrinsic.fy <= 0.0f) {
            throw std::runtime_error("Selected live RGB intrinsic is inconsistent with 1280x720 profile");
        }
        pipeline->start(config);
        started = true;
        try {
            calibrationParam = pipeline->getCalibrationParam(config);
        }
        catch(...) {
            pipeline->stop();
            started = false;
            throw;
        }
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
    std::shared_ptr<ob::VideoStreamProfile> selectedDepthProfile;
    OBCalibrationParam calibrationParam{};
    OBCameraIntrinsic intrinsic{};
    OBCameraDistortion distortion{};
    std::string deviceName;
    std::string serialNumber;
    std::string firmwareVersion;
    std::vector<std::string> allProfiles;
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
    if(!std::isfinite(u) || !std::isfinite(v) || u < 0.0 || v < 0.0
       || u >= impl_->selectedProfile->width() || v >= impl_->selectedProfile->height()) {
        throw std::runtime_error("RGB pixel is non-finite or outside raw 1280x720 image bounds");
    }
    const OBPoint2f pixel{static_cast<float>(u), static_cast<float>(v)};
    OBPoint3f q1{};
    OBPoint3f q1000{};
    if(!ob::CoordinateTransformHelper::calibration2dTo3dUndistortion(
           impl_->calibrationParam, pixel, 1.0f, OB_SENSOR_COLOR, OB_SENSOR_COLOR, &q1)
       || !ob::CoordinateTransformHelper::calibration2dTo3dUndistortion(
           impl_->calibrationParam, pixel, 1000.0f, OB_SENSOR_COLOR, OB_SENSOR_COLOR, &q1000)) {
        throw std::runtime_error("Orbbec SDK RGB undistorted inverse projection failed");
    }
    PixelRayDiagnostics diagnostics;
    diagnostics.qAt1Mm = fromObPoint(q1);
    diagnostics.qAt1000Mm = fromObPoint(q1000);
    if(diagnostics.qAt1Mm.z <= 0.0 || diagnostics.qAt1000Mm.z <= 0.0) {
        throw std::runtime_error("Orbbec inverse projection returned a ray with non-positive optical Z");
    }
    diagnostics.unitRayRgb = normalize(diagnostics.qAt1Mm);
    if(!std::isfinite(norm(diagnostics.unitRayRgb)) || std::abs(norm(diagnostics.unitRayRgb) - 1.0) > 1e-9) {
        throw std::runtime_error("Normalized Orbbec RGB ray is non-finite or not unit length");
    }
    diagnostics.normalizedDirectionDifference = distance(diagnostics.unitRayRgb, normalize(diagnostics.qAt1000Mm));
    diagnostics.scaleResidualMm = distance(
        diagnostics.qAt1000Mm,
        {diagnostics.qAt1Mm.x * 1000.0, diagnostics.qAt1Mm.y * 1000.0, diagnostics.qAt1Mm.z * 1000.0});
    if(diagnostics.normalizedDirectionDifference > 1e-6 || diagnostics.scaleResidualMm > 0.1) {
        throw std::runtime_error("Synthetic 1 mm and 1000 mm Orbbec rays are not scale-equivalent");
    }

    OBPoint2f reprojected{};
    if(!ob::CoordinateTransformHelper::calibration3dTo2d(
           impl_->calibrationParam, q1000, OB_SENSOR_COLOR, OB_SENSOR_COLOR, &reprojected)) {
        throw std::runtime_error("Orbbec SDK RGB ray reprojection failed");
    }
    diagnostics.reprojectionErrorPx = std::hypot(reprojected.x - u, reprojected.y - v);
    if(diagnostics.reprojectionErrorPx > 0.25) {
        throw std::runtime_error("Orbbec RGB ray reprojection error exceeds 0.25 pixels");
    }

    const std::size_t pixelCount = static_cast<std::size_t>(impl_->intrinsic.width)
                                   * static_cast<std::size_t>(impl_->intrinsic.height);
    std::vector<float> xyStorage(pixelCount * 2U);
    std::uint32_t byteCount = static_cast<std::uint32_t>(xyStorage.size() * sizeof(float));
    OBXYTables tables{};
    if(!ob::CoordinateTransformHelper::transformationInitXYTables(
           impl_->calibrationParam, OB_SENSOR_COLOR, xyStorage.data(), &byteCount, &tables)
       || tables.xTable == nullptr || tables.yTable == nullptr || tables.width != impl_->intrinsic.width
       || tables.height != impl_->intrinsic.height) {
        throw std::runtime_error("Orbbec RGB XY-table initialization failed");
    }
    const int nearestU = std::clamp(static_cast<int>(std::lround(u)), 0, tables.width - 1);
    const int nearestV = std::clamp(static_cast<int>(std::lround(v)), 0, tables.height - 1);
    const std::size_t tableIndex = static_cast<std::size_t>(nearestV) * static_cast<std::size_t>(tables.width)
                                   + static_cast<std::size_t>(nearestU);
    const Vec3 tableRay = normalize({tables.xTable[tableIndex], tables.yTable[tableIndex], 1.0});
    diagnostics.xyTableDirectionDifference = distance(diagnostics.unitRayRgb, tableRay);
    if(diagnostics.xyTableDirectionDifference > 0.003) {
        throw std::runtime_error("Orbbec SDK inverse projection disagrees with RGB XY table");
    }
    return diagnostics;
}

std::string OrbbecCamera::profileDescription() const {
    std::ostringstream description;
    description << impl_->deviceName << " serial=" << impl_->serialNumber << " firmware=" << impl_->firmwareVersion
                << " SDK=" << sdkVersionString() << " profile=" << impl_->selectedProfile->width() << 'x'
                << impl_->selectedProfile->height() << '@' << impl_->selectedProfile->fps() << " MJPG"
                << " calibration_depth=" << impl_->selectedDepthProfile->width() << 'x'
                << impl_->selectedDepthProfile->height() << '@' << impl_->selectedDepthProfile->fps()
                << " format=" << static_cast<int>(impl_->selectedDepthProfile->format());
    return description.str();
}

}  // namespace rgb_base0
