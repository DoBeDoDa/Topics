#include "rgb_base0/camera_profile.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>

namespace rgb_base0 {
namespace {

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

}  // namespace

void requireExactlyOneOrbbecDevice(const std::uint32_t deviceCount) {
    if(deviceCount == 0) {
        throw std::runtime_error("Gemini 2 XL not found; Orbbec device count is zero");
    }
    if(deviceCount != 1) {
        throw std::runtime_error("Exactly one Orbbec device must be connected; detected "
                                 + std::to_string(deviceCount));
    }
}

void validateGemini2XlIdentity(const std::string& deviceName,
                              const std::string& serialNumber) {
    if(lower(deviceName).find("gemini 2 xl") == std::string::npos) {
        throw std::runtime_error("Connected Orbbec device is not identified as Gemini 2 XL: " + deviceName);
    }
    if(serialNumber.empty()) {
        throw std::runtime_error("Gemini 2 XL returned an empty serial number");
    }
}

std::size_t selectRequiredColorProfile(const std::vector<CameraProfileData>& profiles) {
    std::size_t selected = profiles.size();
    for(std::size_t index = 0; index < profiles.size(); ++index) {
        const CameraProfileData& profile = profiles[index];
        if(profile.width == 1280 && profile.height == 720 && profile.format == "MJPG"
           && profile.fps > 0
           && (selected == profiles.size() || profile.fps > profiles[selected].fps)) {
            selected = index;
        }
    }
    if(selected == profiles.size()) {
        std::ostringstream message;
        message << "Required 1280x720 MJPG color profile is unavailable. Reported profiles:";
        for(const CameraProfileData& profile : profiles) {
            message << "\n  " << profile.width << 'x' << profile.height
                    << " fps=" << profile.fps << " format=" << profile.format;
        }
        throw std::runtime_error(message.str());
    }
    return selected;
}

}  // namespace rgb_base0
