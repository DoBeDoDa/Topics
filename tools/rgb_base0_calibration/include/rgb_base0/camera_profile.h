#pragma once

#include "rgb_base0/types.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rgb_base0 {

void requireExactlyOneOrbbecDevice(std::uint32_t deviceCount);
void validateGemini2XlIdentity(const std::string& deviceName,
                              const std::string& serialNumber);
std::size_t selectRequiredColorProfile(const std::vector<CameraProfileData>& profiles);

}  // namespace rgb_base0
