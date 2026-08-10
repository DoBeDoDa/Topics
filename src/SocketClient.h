// 宣告唯一Windows TCP Client、嚴格newline framing及本地connection identity。
#pragma once

#include <winsock2.h>
#include <ws2tcpip.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

#include "TableState.h"

enum class FrameBufferStatus {
    Accepted,
    FrameTooLong,
    InvalidConfiguration
};

class NewlineFrameBuffer {
public:
    explicit NewlineFrameBuffer(std::size_t maximumFrameBytes);

    [[nodiscard]] FrameBufferStatus append(const char* data, std::size_t size);
    [[nodiscard]] bool popFrame(std::string& frame);
    void reset() noexcept;
    [[nodiscard]] std::size_t maximumFrameBytes() const noexcept;
    [[nodiscard]] std::size_t partialFrameBytes() const noexcept;

private:
    std::size_t maximumFrameBytes_;
    std::string partialFrame_;
    std::deque<std::string> completedFrames_;
};

enum class SocketReceiveStatus {
    FrameReady,
    NeedMoreData,
    CleanClose,
    TimedOut,
    FrameTooLong,
    SocketError,
    NotConnected,
    InvalidConfiguration
};

struct SocketReceiveResult {
    SocketReceiveStatus status;
    std::optional<std::string> frame;
    int socketError;

    [[nodiscard]] bool isValid() const noexcept
    {
        return frame.has_value() == (status == SocketReceiveStatus::FrameReady);
    }
};

enum class SocketOperationStatus {
    Success,
    NotConnected,
    SocketError
};

enum class SocketConfigurationStatus {
    Valid,
    ConfigurationMissing,
    InvalidConfiguration
};

enum class SocketConnectStatus {
    Success,
    ConfigurationMissing,
    InvalidConfiguration,
    SocketCreationError,
    InvalidAddress,
    ConnectError,
    TimeoutConfigurationError
};

struct SocketConnectResult {
    SocketConnectStatus status;
    int socketError;
};

struct SocketOperationResult {
    SocketOperationStatus status;
    int socketError;
};

[[nodiscard]] SocketReceiveStatus classifyTransportOutcome(
    int receivedBytes,
    int socketError) noexcept;

class LocalConnectionLifecycle {
public:
    LocalConnectionLifecycle() noexcept;

    [[nodiscard]] ConnectionIdentity open() noexcept;
    void invalidate() noexcept;
    [[nodiscard]] ConnectionIdentity current() const noexcept;

private:
    ConnectionIdentity currentIdentity_;
    ConnectionIdentity lastIssuedIdentity_;
};

// SocketClient實際使用、亦可由fake bytes離線驗證的receive-state seam。
class SocketReceiveState {
public:
    explicit SocketReceiveState(std::size_t maximumFrameBytes);

    [[nodiscard]] ConnectionIdentity openConnection() noexcept;
    void invalidateConnection() noexcept;
    void flush() noexcept;
    [[nodiscard]] FrameBufferStatus append(const char* data, std::size_t size);
    [[nodiscard]] bool popFrame(std::string& frame);
    [[nodiscard]] ConnectionIdentity connectionIdentity() const noexcept;
    [[nodiscard]] std::size_t maximumFrameBytes() const noexcept;

private:
    NewlineFrameBuffer frameBuffer_;
    LocalConnectionLifecycle connectionLifecycle_;
};

class SocketClient {
public:
    SocketClient();
    SocketClient(
        std::size_t maximumFrameBytes,
        unsigned long receiveTimeoutMs);
    ~SocketClient();

    bool connectToServer(const std::string& ip, int port);
    [[nodiscard]] SocketConnectResult connectToServerResult(const std::string& ip, int port);
    [[nodiscard]] SocketConfigurationStatus configurationStatus() const noexcept;
    void closeConnection();
    [[nodiscard]] bool isConnected() const;
    [[nodiscard]] ConnectionIdentity connectionIdentity() const noexcept;

    int receiveData(char* buf, int maxLen);
    [[nodiscard]] SocketReceiveResult receiveFrame();
    int receiveLine(std::string& line);  // legacy call-site compatibility
    int sendData(const std::string& data);
    [[nodiscard]] SocketOperationResult flushBuffer();

private:
    void invalidateReceiveState() noexcept;

    SOCKET clientSocket_;
    bool connected_;
    unsigned long receiveTimeoutMs_;
    SocketReceiveState receiveState_;
    SocketConfigurationStatus configurationStatus_;
};
