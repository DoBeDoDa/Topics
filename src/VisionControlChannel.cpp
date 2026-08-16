#include "VisionControlChannel.h"

#include <string>

namespace {

VisionControlSendStatus sendControlLine(
    SocketClient& visionClient,
    const std::string& command,
    ShotCycleIdentity cycleIdentity)
{
    if (cycleIdentity == 0) {
        return VisionControlSendStatus::InvalidCycleIdentity;
    }
    const std::string line =
        command + "," + std::to_string(cycleIdentity) + "\n";
    return visionClient.sendData(line) == -1
        ? VisionControlSendStatus::SendFailed
        : VisionControlSendStatus::Success;
}

}  // namespace

VisionControlSendStatus sendStartCapture(
    SocketClient& visionClient,
    ShotCycleIdentity cycleIdentity)
{
    return sendControlLine(visionClient, "START_CAPTURE", cycleIdentity);
}

VisionControlSendStatus sendStopCapture(
    SocketClient& visionClient,
    ShotCycleIdentity cycleIdentity)
{
    return sendControlLine(visionClient, "STOP_CAPTURE", cycleIdentity);
}
