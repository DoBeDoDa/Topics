// 實作唯一Windows TCP transport、newline framing與本地connection生命週期。
#include "SocketClient.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "BilliardConfig.h"

NewlineFrameBuffer::NewlineFrameBuffer(std::size_t maximumFrameBytes)
    : maximumFrameBytes_(maximumFrameBytes)
{
}

FrameBufferStatus NewlineFrameBuffer::append(const char* data, std::size_t size)
{
    if (maximumFrameBytes_ == 0 || (data == nullptr && size != 0)) {
        reset();
        return FrameBufferStatus::InvalidConfiguration;
    }

    for (std::size_t index = 0; index < size; ++index) {
        const char value = data[index];
        if (value == '\n') {
            if (!partialFrame_.empty() && partialFrame_.back() == '\r') {
                partialFrame_.pop_back();
            }
            completedFrames_.push_back(std::move(partialFrame_));
            partialFrame_.clear();
            continue;
        }

        if (partialFrame_.size() >= maximumFrameBytes_) {
            reset();
            return FrameBufferStatus::FrameTooLong;
        }
        partialFrame_.push_back(value);
    }
    return FrameBufferStatus::Accepted;
}

bool NewlineFrameBuffer::popFrame(std::string& frame)
{
    if (completedFrames_.empty()) {
        return false;
    }
    frame = std::move(completedFrames_.front());
    completedFrames_.pop_front();
    return true;
}

void NewlineFrameBuffer::reset() noexcept
{
    partialFrame_.clear();
    completedFrames_.clear();
}

std::size_t NewlineFrameBuffer::maximumFrameBytes() const noexcept
{
    return maximumFrameBytes_;
}

std::size_t NewlineFrameBuffer::partialFrameBytes() const noexcept
{
    return partialFrame_.size();
}

SocketReceiveStatus classifyTransportOutcome(
    int receivedBytes,
    int socketError) noexcept
{
    if (receivedBytes > 0) {
        return SocketReceiveStatus::NeedMoreData;
    }
    if (receivedBytes == 0) {
        return SocketReceiveStatus::CleanClose;
    }
    if (socketError == WSAETIMEDOUT) {
        return SocketReceiveStatus::TimedOut;
    }
    return SocketReceiveStatus::SocketError;
}

LocalConnectionLifecycle::LocalConnectionLifecycle() noexcept
    : currentIdentity_(0), lastIssuedIdentity_(0)
{
}

ConnectionIdentity LocalConnectionLifecycle::open() noexcept
{
    if (lastIssuedIdentity_ == (std::numeric_limits<ConnectionIdentity>::max)()) {
        currentIdentity_ = 0;
        return 0;
    }
    currentIdentity_ = ++lastIssuedIdentity_;
    return currentIdentity_;
}

void LocalConnectionLifecycle::invalidate() noexcept
{
    currentIdentity_ = 0;
}

ConnectionIdentity LocalConnectionLifecycle::current() const noexcept
{
    return currentIdentity_;
}

SocketReceiveState::SocketReceiveState(std::size_t maximumFrameBytes)
    : frameBuffer_(maximumFrameBytes)
{
}

ConnectionIdentity SocketReceiveState::openConnection() noexcept
{
    frameBuffer_.reset();
    return connectionLifecycle_.open();
}

void SocketReceiveState::invalidateConnection() noexcept
{
    connectionLifecycle_.invalidate();
    frameBuffer_.reset();
}

void SocketReceiveState::flush() noexcept
{
    frameBuffer_.reset();
}

FrameBufferStatus SocketReceiveState::append(const char* data, std::size_t size)
{
    return frameBuffer_.append(data, size);
}

bool SocketReceiveState::popFrame(std::string& frame)
{
    return frameBuffer_.popFrame(frame);
}

ConnectionIdentity SocketReceiveState::connectionIdentity() const noexcept
{
    return connectionLifecycle_.current();
}

std::size_t SocketReceiveState::maximumFrameBytes() const noexcept
{
    return frameBuffer_.maximumFrameBytes();
}

SocketClient::SocketClient()
    : clientSocket_(INVALID_SOCKET),
      connected_(false),
      receiveTimeoutMs_(BilliardConfig::VISION_RECEIVE_TIMEOUT_MS.value_or(0)),
      receiveState_(BilliardConfig::VISION_MAX_FRAME_BYTES.value_or(0)),
      configurationStatus_(
          !BilliardConfig::VISION_MAX_FRAME_BYTES ||
                  !BilliardConfig::VISION_RECEIVE_TIMEOUT_MS
              ? SocketConfigurationStatus::ConfigurationMissing
              : (*BilliardConfig::VISION_MAX_FRAME_BYTES == 0 ||
                         *BilliardConfig::VISION_RECEIVE_TIMEOUT_MS == 0
                     ? SocketConfigurationStatus::InvalidConfiguration
                     : SocketConfigurationStatus::Valid))
{
}

SocketClient::SocketClient(
    std::size_t maximumFrameBytes,
    unsigned long receiveTimeoutMs)
    : clientSocket_(INVALID_SOCKET),
      connected_(false),
      receiveTimeoutMs_(receiveTimeoutMs),
      receiveState_(maximumFrameBytes),
      configurationStatus_(
          maximumFrameBytes == 0 || receiveTimeoutMs == 0
              ? SocketConfigurationStatus::InvalidConfiguration
              : SocketConfigurationStatus::Valid)
{
}

SocketClient::~SocketClient()
{
    closeConnection();
}

bool SocketClient::connectToServer(const std::string& ip, int port)
{
    return connectToServerResult(ip, port).status == SocketConnectStatus::Success;
}

SocketConnectResult SocketClient::connectToServerResult(const std::string& ip, int port)
{
    closeConnection();
    if (configurationStatus_ == SocketConfigurationStatus::ConfigurationMissing) {
        return {SocketConnectStatus::ConfigurationMissing, 0};
    }
    if (configurationStatus_ == SocketConfigurationStatus::InvalidConfiguration ||
        port <= 0 || port > 65535) {
        return {SocketConnectStatus::InvalidConfiguration, 0};
    }

    clientSocket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (clientSocket_ == INVALID_SOCKET) {
        return {SocketConnectStatus::SocketCreationError, WSAGetLastError()};
    }

    sockaddr_in serverAddress = {};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(static_cast<u_short>(port));
    if (InetPtonA(AF_INET, ip.c_str(), &serverAddress.sin_addr) != 1) {
        closesocket(clientSocket_);
        clientSocket_ = INVALID_SOCKET;
        return {SocketConnectStatus::InvalidAddress, 0};
    }

    if (connect(
            clientSocket_,
            reinterpret_cast<sockaddr*>(&serverAddress),
            sizeof(serverAddress)) == SOCKET_ERROR) {
        const int socketError = WSAGetLastError();
        closesocket(clientSocket_);
        clientSocket_ = INVALID_SOCKET;
        return {SocketConnectStatus::ConnectError, socketError};
    }

    const DWORD timeout = receiveTimeoutMs_;
    if (setsockopt(
            clientSocket_,
            SOL_SOCKET,
            SO_RCVTIMEO,
            reinterpret_cast<const char*>(&timeout),
            sizeof(timeout)) == SOCKET_ERROR) {
        const int socketError = WSAGetLastError();
        closesocket(clientSocket_);
        clientSocket_ = INVALID_SOCKET;
        return {SocketConnectStatus::TimeoutConfigurationError, socketError};
    }

    const ConnectionIdentity identity = receiveState_.openConnection();
    if (identity == 0) {
        closesocket(clientSocket_);
        clientSocket_ = INVALID_SOCKET;
        return {SocketConnectStatus::InvalidConfiguration, 0};
    }
    connected_ = true;
    return {SocketConnectStatus::Success, 0};
}

SocketConfigurationStatus SocketClient::configurationStatus() const noexcept
{
    return configurationStatus_;
}

void SocketClient::closeConnection()
{
    if (clientSocket_ != INVALID_SOCKET) {
        closesocket(clientSocket_);
        clientSocket_ = INVALID_SOCKET;
    }
    connected_ = false;
    receiveState_.invalidateConnection();
    invalidateReceiveState();
}

bool SocketClient::isConnected() const
{
    return connected_;
}

ConnectionIdentity SocketClient::connectionIdentity() const noexcept
{
    return receiveState_.connectionIdentity();
}

int SocketClient::receiveData(char* buffer, int maximumLength)
{
    if (!connected_ || buffer == nullptr || maximumLength <= 0) {
        return -1;
    }
    return recv(clientSocket_, buffer, maximumLength, 0);
}

SocketReceiveResult SocketClient::receiveFrame()
{
    if (!connected_) {
        return {SocketReceiveStatus::NotConnected, std::nullopt, 0};
    }
    if (receiveState_.maximumFrameBytes() == 0) {
        return {SocketReceiveStatus::InvalidConfiguration, std::nullopt, 0};
    }

    std::string frame;
    if (receiveState_.popFrame(frame)) {
        return {SocketReceiveStatus::FrameReady, std::move(frame), 0};
    }

    char buffer[1024];
    while (true) {
        const int received = recv(clientSocket_, buffer, sizeof(buffer), 0);
        if (received > 0) {
            const FrameBufferStatus framing = receiveState_.append(
                buffer,
                static_cast<std::size_t>(received));
            if (framing == FrameBufferStatus::FrameTooLong) {
                return {SocketReceiveStatus::FrameTooLong, std::nullopt, 0};
            }
            if (framing == FrameBufferStatus::InvalidConfiguration) {
                return {SocketReceiveStatus::InvalidConfiguration, std::nullopt, 0};
            }
            if (receiveState_.popFrame(frame)) {
                return {SocketReceiveStatus::FrameReady, std::move(frame), 0};
            }
            continue;
        }

        const int error = received == SOCKET_ERROR ? WSAGetLastError() : 0;
        const SocketReceiveStatus status = classifyTransportOutcome(received, error);
        invalidateReceiveState();
        if (status == SocketReceiveStatus::CleanClose ||
            status == SocketReceiveStatus::SocketError) {
            if (clientSocket_ != INVALID_SOCKET) {
                closesocket(clientSocket_);
                clientSocket_ = INVALID_SOCKET;
            }
            connected_ = false;
            receiveState_.invalidateConnection();
        }
        return {status, std::nullopt, error};
    }
}

int SocketClient::receiveLine(std::string& line)
{
    const SocketReceiveResult result = receiveFrame();
    if (result.status == SocketReceiveStatus::FrameReady && result.frame) {
        line = *result.frame;
        return static_cast<int>(line.size());
    }
    if (result.status == SocketReceiveStatus::CleanClose) {
        return 0;
    }
    return -1;
}

int SocketClient::sendData(const std::string& data)
{
    if (!connected_) {
        return -1;
    }
    int totalSent = 0;
    const int totalLength = static_cast<int>(data.length());
    while (totalSent < totalLength) {
        const int sent = send(
            clientSocket_,
            data.c_str() + totalSent,
            totalLength - totalSent,
            0);
        if (sent == SOCKET_ERROR || sent == 0) {
            return -1;
        }
        totalSent += sent;
    }
    return totalSent;
}

SocketOperationResult SocketClient::flushBuffer()
{
    invalidateReceiveState();
    if (!connected_) {
        return {SocketOperationStatus::NotConnected, 0};
    }

    char discard[1024];
    while (true) {
        u_long bytesAvailable = 0;
        if (ioctlsocket(clientSocket_, FIONREAD, &bytesAvailable) == SOCKET_ERROR) {
            return {SocketOperationStatus::SocketError, WSAGetLastError()};
        }
        if (bytesAvailable == 0) {
            return {SocketOperationStatus::Success, 0};
        }
        const int received = recv(
            clientSocket_,
            discard,
            static_cast<int>((std::min)(bytesAvailable, static_cast<u_long>(sizeof(discard)))),
            0);
        if (received <= 0) {
            const int error = received == SOCKET_ERROR ? WSAGetLastError() : 0;
            return {SocketOperationStatus::SocketError, error};
        }
    }
}

void SocketClient::invalidateReceiveState() noexcept
{
    receiveState_.flush();
}
